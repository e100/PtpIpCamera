#pragma once
#include "PtpSession.h"
#include <sdkconfig.h>

// ============================================================================
// PtpUsbSession
// PTP over USB (Still Image class, PIMA 15740 USB transport) using the
// ESP-IDF USB Host Library. ESP32-S2/S3 only: they have a USB OTG port.
//
// The ESP32 is the USB host, so it must supply 5 V on VBUS to the camera and
// present itself as a default-power source (a USB-C to USB-A cable does
// this). The native USB port can then not also be used for the serial
// console: build with ARDUINO_USB_CDC_ON_BOOT=0 and use the UART.
//
// Usage:
//   PtpUsbSession usb;
//   NikonCamera camera(usb);
//   usb.begin();                       // once, in setup()
//   if (usb.cameraAttached()) camera.begin("usb");   // host string is ignored
//
// No pairing over USB: a wired host is trusted.
// ============================================================================

#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32S2)
#define PTP_USB_SUPPORTED 1

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <usb/usb_host.h>

class PtpUsbSession : public PtpSession {
public:
    PtpUsbSession() {}

    // Install the USB Host Library and start its tasks. Call once.
    bool begin();

    // A device has been attached since the last connect()/disconnect(), or a
    // still-image device is currently open.
    bool cameraAttached() const { return _attached || _open; }

    // Open the attached still-image device and claim its PTP interface.
    // host is ignored.
    bool connect(const char* host) override;
    void disconnect() override;
    bool isConnected() override { return _open && !_gone; }

    bool openSession() override;
    bool closeSession() override;

    bool command(uint16_t opCode, const uint32_t* params, uint8_t numParams,
                 uint16_t& responseCode) override;
    bool dataIn(uint16_t opCode, const uint32_t* params, uint8_t numParams,
                uint8_t* data, size_t maxDataLen, size_t& dataLen,
                uint16_t& responseCode) override;
    bool dataOut(uint16_t opCode, const uint32_t* params, uint8_t numParams,
                 const uint8_t* data, size_t dataLen,
                 uint16_t& responseCode) override;
    bool nextEvent(PtpEvent& event) override;

    // Timeout for each bulk transfer, ms. Captures can take a while.
    void setTimeout(uint32_t ms) { _timeoutMs = ms; }

private:
    static constexpr size_t IN_BUF_SIZE  = 4096;   // multiple of the 64-byte full-speed packet size
    static constexpr size_t OUT_BUF_SIZE = 1024;
    static constexpr size_t INT_BUF_SIZE = 64;

    usb_host_client_handle_t _client = nullptr;
    usb_device_handle_t      _dev = nullptr;
    usb_transfer_t*          _in = nullptr;
    usb_transfer_t*          _out = nullptr;
    usb_transfer_t*          _int = nullptr;
    SemaphoreHandle_t        _done = nullptr;      // bulk transfer completed
    QueueHandle_t            _events = nullptr;    // PtpEvent from the interrupt endpoint

    volatile bool    _attached = false;   // NEW_DEV seen
    volatile uint8_t _newAddr = 0;
    volatile bool    _gone = false;       // DEV_GONE for the open device
    bool             _open = false;
    bool             _sessionOpen = false;
    uint8_t          _intf = 0;
    uint8_t          _epIn = 0, _epOut = 0, _epInt = 0;
    uint16_t         _mpsIn = 64;
    uint32_t         _txId = 0;
    uint32_t         _timeoutMs = 5000;

    static void _hostTask(void* arg);
    static void _clientTask(void* arg);
    static void _clientEvent(const usb_host_client_event_msg_t* msg, void* arg);
    static void _bulkDone(usb_transfer_t* t);
    static void _intDone(usb_transfer_t* t);

    bool _openDevice(uint8_t addr);
    void _closeDevice();
    bool _transfer(usb_transfer_t* t, int numBytes);
    bool _sendContainer(uint16_t type, uint16_t code, uint32_t txId,
                        const uint32_t* params, uint8_t numParams,
                        const uint8_t* payload, size_t payloadLen);
    bool _recvResponse(uint32_t txId, uint16_t& responseCode);
    uint32_t _nextTxId() { return ++_txId; }
};

#endif
