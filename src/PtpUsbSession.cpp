#include "PtpUsbSession.h"

#ifdef PTP_USB_SUPPORTED

#include "PtpIpConstants.h"
#include "PtpIpLog.h"
#include <Arduino.h>
#include <string.h>

#define LOG_TAG "USB"

// PTP USB container: UINT32 length, UINT16 type, UINT16 code, UINT32
// transaction ID, then parameters (command/response/event) or data.
static constexpr uint16_t CT_COMMAND  = 1;
static constexpr uint16_t CT_DATA     = 2;
static constexpr uint16_t CT_RESPONSE = 3;
static constexpr uint16_t CT_EVENT    = 4;
static constexpr size_t   CT_HEADER   = 12;

// Still Image class interface: class 6 (USB_CLASS_STILL_IMAGE), subclass 1, protocol 1

static inline uint16_t rd16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline uint32_t rd32(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline void wr16(uint8_t* p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static inline void wr32(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

// ============================================================================
// USB Host Library plumbing
// ============================================================================

bool PtpUsbSession::begin() {
    if (_client) return true;

    usb_host_config_t hostCfg = {};
    hostCfg.skip_phy_setup = false;
    hostCfg.intr_flags = ESP_INTR_FLAG_LEVEL1;
    esp_err_t err = usb_host_install(&hostCfg);
    if (err != ESP_OK) {
        LOG_ERROR("usb_host_install: %s", esp_err_to_name(err));
        return false;
    }
    xTaskCreatePinnedToCore(_hostTask, "usb_host", 4096, this, 5, nullptr, 0);

    usb_host_client_config_t clientCfg = {};
    clientCfg.is_synchronous = false;
    clientCfg.max_num_event_msg = 5;
    clientCfg.async.client_event_callback = _clientEvent;
    clientCfg.async.callback_arg = this;
    err = usb_host_client_register(&clientCfg, &_client);
    if (err != ESP_OK) {
        LOG_ERROR("usb_host_client_register: %s", esp_err_to_name(err));
        return false;
    }

    _done = xSemaphoreCreateBinary();
    _events = xQueueCreate(16, sizeof(PtpEvent));
    if (usb_host_transfer_alloc(IN_BUF_SIZE, 0, &_in) != ESP_OK ||
        usb_host_transfer_alloc(OUT_BUF_SIZE, 0, &_out) != ESP_OK ||
        usb_host_transfer_alloc(INT_BUF_SIZE, 0, &_int) != ESP_OK) {
        LOG_ERROR("usb_host_transfer_alloc failed");
        return false;
    }
    _in->callback = _out->callback = _bulkDone;
    _int->callback = _intDone;
    _in->context = _out->context = _int->context = this;

    // Transfer callbacks and device events are delivered from this task
    xTaskCreatePinnedToCore(_clientTask, "usb_ptp", 4096, this, 5, nullptr, 0);
    LOG_INFO("USB host ready; waiting for a camera");
    return true;
}

void PtpUsbSession::_hostTask(void*) {
    while (true) {
        uint32_t flags;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
    }
}

void PtpUsbSession::_clientTask(void* arg) {
    PtpUsbSession* self = (PtpUsbSession*)arg;
    while (true) usb_host_client_handle_events(self->_client, portMAX_DELAY);
}

void PtpUsbSession::_clientEvent(const usb_host_client_event_msg_t* msg, void* arg) {
    PtpUsbSession* self = (PtpUsbSession*)arg;
    if (msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV) {
        self->_newAddr = msg->new_dev.address;
        self->_attached = true;
    } else if (msg->event == USB_HOST_CLIENT_EVENT_DEV_GONE) {
        self->_gone = true;
        self->_attached = false;
    }
}

void PtpUsbSession::_bulkDone(usb_transfer_t* t) {
    xSemaphoreGive(((PtpUsbSession*)t->context)->_done);
}

// Interrupt endpoint: one event container per transfer. Queue it and listen again.
void PtpUsbSession::_intDone(usb_transfer_t* t) {
    PtpUsbSession* self = (PtpUsbSession*)t->context;
    if (t->status == USB_TRANSFER_STATUS_COMPLETED && t->actual_num_bytes >= (int)CT_HEADER &&
        rd16(t->data_buffer + 4) == CT_EVENT) {
        PtpEvent ev = {};
        ev.code = rd16(t->data_buffer + 6);
        int n = (t->actual_num_bytes - (int)CT_HEADER) / 4;
        ev.numParams = (uint8_t)(n > 3 ? 3 : n);
        for (uint8_t i = 0; i < ev.numParams; i++) ev.params[i] = rd32(t->data_buffer + CT_HEADER + 4 * i);
        xQueueSend(self->_events, &ev, 0);
    }
    if (t->status == USB_TRANSFER_STATUS_COMPLETED && self->_open && !self->_gone) {
        t->num_bytes = INT_BUF_SIZE;
        usb_host_transfer_submit(t);
    }
}

// ============================================================================
// Device
// ============================================================================

bool PtpUsbSession::connect(const char*) {
    if (!_client) {
        LOG_ERROR("connect: call begin() first");
        return false;
    }
    if (_open && !_gone) return true;
    if (_open) _closeDevice();

    // Prefer the device just announced; otherwise look at everything attached
    uint8_t addrs[8];
    int n = 0;
    if (_attached && _newAddr) addrs[n++] = _newAddr;
    int listed = 0;
    uint8_t list[8];
    if (usb_host_device_addr_list_fill(8, list, &listed) == ESP_OK) {
        for (int i = 0; i < listed && n < 8; i++) if (list[i] != _newAddr) addrs[n++] = list[i];
    }
    for (int i = 0; i < n; i++) {
        if (_openDevice(addrs[i])) return true;
    }
    LOG_WARNING("connect: no USB still-image (PTP) device attached");
    return false;
}

bool PtpUsbSession::_openDevice(uint8_t addr) {
    if (usb_host_device_open(_client, addr, &_dev) != ESP_OK) return false;

    const usb_config_desc_t* cfg;
    if (usb_host_get_active_config_descriptor(_dev, &cfg) != ESP_OK) {
        usb_host_device_close(_client, _dev);
        _dev = nullptr;
        return false;
    }

    // Walk the configuration for the still-image interface and its endpoints
    const uint8_t* p = (const uint8_t*)cfg;
    uint16_t total = cfg->wTotalLength;
    bool inPtp = false, found = false;
    _epIn = _epOut = _epInt = 0;
    for (uint16_t off = 0; off + 2 <= total && p[off] >= 2; off += p[off]) {
        const uint8_t* d = p + off;
        if (d[1] == USB_B_DESCRIPTOR_TYPE_INTERFACE && d[0] >= 9) {
            if (found) break;                       // endpoints of the PTP interface are done
            inPtp = d[5] == USB_CLASS_STILL_IMAGE && d[6] == 1 && d[7] == 1;
            if (inPtp) { _intf = d[2]; found = true; }
        } else if (inPtp && d[1] == USB_B_DESCRIPTOR_TYPE_ENDPOINT && d[0] >= 7) {
            uint8_t ep = d[2], type = d[3] & 0x03;
            uint16_t mps = rd16(d + 4);
            if (type == 2 && (ep & 0x80)) { _epIn = ep; _mpsIn = mps ? mps : 64; }
            else if (type == 2)           _epOut = ep;
            else if (type == 3 && (ep & 0x80)) _epInt = ep;
        }
    }
    if (!found || !_epIn || !_epOut) {
        usb_host_device_close(_client, _dev);
        _dev = nullptr;
        return false;
    }
    if (usb_host_interface_claim(_client, _dev, _intf, 0) != ESP_OK) {
        LOG_ERROR("could not claim PTP interface %u", _intf);
        usb_host_device_close(_client, _dev);
        _dev = nullptr;
        return false;
    }

    _in->device_handle = _out->device_handle = _int->device_handle = _dev;
    _in->bEndpointAddress = _epIn;
    _out->bEndpointAddress = _epOut;
    _out->flags = USB_TRANSFER_FLAG_ZERO_PACK;   // PTP: a full last packet is followed by a ZLP
    _open = true;
    _gone = false;
    _attached = false;
    _sessionOpen = false;
    xQueueReset(_events);

    if (_epInt) {
        _int->bEndpointAddress = _epInt;
        _int->num_bytes = INT_BUF_SIZE;
        usb_host_transfer_submit(_int);
    }

    const usb_device_desc_t* dd;
    if (usb_host_get_device_descriptor(_dev, &dd) == ESP_OK) {
        LOG_INFO("PTP device %04X:%04X at address %u, interface %u (bulk in 0x%02X out 0x%02X, event 0x%02X, %u-byte packets)",
                 dd->idVendor, dd->idProduct, addr, _intf, _epIn, _epOut, _epInt, _mpsIn);
    }
    return true;
}

void PtpUsbSession::_closeDevice() {
    if (!_dev) return;
    if (_epInt) {
        usb_host_endpoint_halt(_dev, _epInt);
        usb_host_endpoint_flush(_dev, _epInt);
    }
    usb_host_interface_release(_client, _dev, _intf);
    usb_host_device_close(_client, _dev);
    _dev = nullptr;
    _open = false;
    _sessionOpen = false;
}

void PtpUsbSession::disconnect() {
    _closeDevice();
}

// One bulk transfer, waiting for it to finish. On a timeout the endpoint is
// halted and flushed so the transfer object can be reused.
bool PtpUsbSession::_transfer(usb_transfer_t* t, int numBytes) {
    if (!_open || _gone) return false;
    t->num_bytes = numBytes;
    xSemaphoreTake(_done, 0);
    if (usb_host_transfer_submit(t) != ESP_OK) {
        LOG_ERROR("transfer submit failed (ep 0x%02X)", t->bEndpointAddress);
        return false;
    }
    if (xSemaphoreTake(_done, pdMS_TO_TICKS(_timeoutMs)) != pdTRUE) {
        LOG_ERROR("transfer timed out (ep 0x%02X)", t->bEndpointAddress);
        usb_host_endpoint_halt(_dev, t->bEndpointAddress);
        usb_host_endpoint_flush(_dev, t->bEndpointAddress);
        xSemaphoreTake(_done, pdMS_TO_TICKS(100));   // the flushed transfer completes as canceled
        usb_host_endpoint_clear(_dev, t->bEndpointAddress);
        return false;
    }
    if (t->status != USB_TRANSFER_STATUS_COMPLETED) {
        LOG_ERROR("transfer failed (ep 0x%02X, status %d)", t->bEndpointAddress, (int)t->status);
        if (t->status == USB_TRANSFER_STATUS_STALL) usb_host_endpoint_clear(_dev, t->bEndpointAddress);
        return false;
    }
    return true;
}

// ============================================================================
// Containers and transactions
// ============================================================================

bool PtpUsbSession::_sendContainer(uint16_t type, uint16_t code, uint32_t txId,
                                   const uint32_t* params, uint8_t numParams,
                                   const uint8_t* payload, size_t payloadLen) {
    size_t len = CT_HEADER + (size_t)numParams * 4 + payloadLen;
    if (len > OUT_BUF_SIZE) {
        LOG_ERROR("container too large (%u bytes)", (unsigned)len);
        return false;
    }
    uint8_t* b = _out->data_buffer;
    wr32(b, (uint32_t)len);
    wr16(b + 4, type);
    wr16(b + 6, code);
    wr32(b + 8, txId);
    for (uint8_t i = 0; i < numParams; i++) wr32(b + CT_HEADER + 4 * i, params[i]);
    if (payloadLen) memcpy(b + CT_HEADER + numParams * 4, payload, payloadLen);
    return _transfer(_out, (int)len);
}

// Read the response container. A zero-length packet ending the data phase
// may arrive first; skip it.
bool PtpUsbSession::_recvResponse(uint32_t txId, uint16_t& responseCode) {
    for (int tries = 0; tries < 3; tries++) {
        if (!_transfer(_in, (int)IN_BUF_SIZE)) return false;
        int n = _in->actual_num_bytes;
        if (n == 0) continue;
        const uint8_t* b = _in->data_buffer;
        if (n < (int)CT_HEADER || rd16(b + 4) != CT_RESPONSE) {
            LOG_ERROR("expected a response container, got type %u (%d bytes)", n >= 6 ? rd16(b + 4) : 0, n);
            return false;
        }
        if (rd32(b + 8) != txId) LOG_WARNING("response for transaction %u, expected %u",
                                             (unsigned)rd32(b + 8), (unsigned)txId);
        responseCode = rd16(b + 6);
        return true;
    }
    LOG_ERROR("no response container");
    return false;
}

bool PtpUsbSession::openSession() {
    _txId = 0;                                  // OpenSession uses transaction ID 0
    uint32_t sessionId = 1;
    uint16_t rc;
    if (!_sendContainer(CT_COMMAND, PTP_OP_OPEN_SESSION, 0, &sessionId, 1, nullptr, 0)) return false;
    if (!_recvResponse(0, rc)) return false;
    if (rc != PTP_RSP_OK && rc != 0x201E) {     // 0x201E: session already open
        LOG_ERROR("OpenSession: response 0x%04X", rc);
        return false;
    }
    _sessionOpen = true;
    return true;
}

bool PtpUsbSession::closeSession() {
    if (!_sessionOpen) return true;
    uint16_t rc;
    bool ok = command(PTP_OP_CLOSE_SESSION, nullptr, 0, rc);
    _sessionOpen = false;
    return ok && rc == PTP_RSP_OK;
}

bool PtpUsbSession::command(uint16_t opCode, const uint32_t* params, uint8_t numParams,
                            uint16_t& responseCode) {
    uint32_t tx = _nextTxId();
    if (!_sendContainer(CT_COMMAND, opCode, tx, params, numParams, nullptr, 0)) return false;
    return _recvResponse(tx, responseCode);
}

bool PtpUsbSession::dataIn(uint16_t opCode, const uint32_t* params, uint8_t numParams,
                           uint8_t* data, size_t maxDataLen, size_t& dataLen,
                           uint16_t& responseCode) {
    dataLen = 0;
    uint32_t tx = _nextTxId();
    if (!_sendContainer(CT_COMMAND, opCode, tx, params, numParams, nullptr, 0)) return false;

    if (!_transfer(_in, (int)IN_BUF_SIZE)) return false;
    int n = _in->actual_num_bytes;
    const uint8_t* b = _in->data_buffer;
    if (n < (int)CT_HEADER) {
        LOG_ERROR("dataIn 0x%04X: short container (%d bytes)", opCode, n);
        return false;
    }
    uint16_t type = rd16(b + 4);
    if (type == CT_RESPONSE) {                  // no data phase (usually an error)
        responseCode = rd16(b + 6);
        return true;
    }
    if (type != CT_DATA) {
        LOG_ERROR("dataIn 0x%04X: unexpected container type %u", opCode, type);
        return false;
    }

    // The data container may span several transfers
    size_t total = rd32(b) >= CT_HEADER ? rd32(b) - CT_HEADER : 0;
    size_t got = 0;
    auto take = [&](const uint8_t* src, size_t len) {
        if (dataLen < maxDataLen) {
            size_t keep = len < maxDataLen - dataLen ? len : maxDataLen - dataLen;
            memcpy(data + dataLen, src, keep);
            dataLen += keep;
        }
        got += len;
    };
    take(b + CT_HEADER, (size_t)n - CT_HEADER);
    while (got < total) {
        if (!_transfer(_in, (int)IN_BUF_SIZE)) return false;
        if (_in->actual_num_bytes == 0) break;
        take(_in->data_buffer, (size_t)_in->actual_num_bytes);
    }
    if (got < total) LOG_WARNING("dataIn 0x%04X: %u of %u data bytes", opCode, (unsigned)got, (unsigned)total);
    return _recvResponse(tx, responseCode);
}

bool PtpUsbSession::dataOut(uint16_t opCode, const uint32_t* params, uint8_t numParams,
                            const uint8_t* data, size_t dataLen,
                            uint16_t& responseCode) {
    uint32_t tx = _nextTxId();
    if (!_sendContainer(CT_COMMAND, opCode, tx, params, numParams, nullptr, 0)) return false;
    if (!_sendContainer(CT_DATA, opCode, tx, nullptr, 0, data, dataLen)) return false;
    return _recvResponse(tx, responseCode);
}

bool PtpUsbSession::nextEvent(PtpEvent& event) {
    if (_open && _gone) {
        LOG_WARNING("camera unplugged");
        _closeDevice();
    }
    return _events && xQueueReceive(_events, &event, 0) == pdTRUE;
}

#endif  // PTP_USB_SUPPORTED
