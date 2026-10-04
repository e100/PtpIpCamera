#include "NikonCamera.h"
#include "PtpIpConstants.h"
#include "PtpIpLog.h"
#include <Arduino.h>
#include <math.h>
#include <string.h>

#define LOG_TAG "NIKON"

// Bits in NikonState::known
static constexpr uint8_t NIKON_KNOWN_FNUMBER  = 0x01;
static constexpr uint8_t NIKON_KNOWN_EXPOSURE = 0x02;
static constexpr uint8_t NIKON_KNOWN_ISO      = 0x04;
static constexpr uint8_t NIKON_KNOWN_BIAS     = 0x08;
static constexpr uint8_t NIKON_KNOWN_WB       = 0x10;
static constexpr uint8_t NIKON_KNOWN_PROGRAM  = 0x20;

// Exposure properties kept in NikonState, in the order poll() reads them.
// `code` (the standard photo property) also identifies the state field; in
// movie mode the camera is read and written through movieCode instead.
struct NikonPropSpec {
    uint16_t       code;
    uint8_t        size;       // bytes on the wire
    uint16_t       movieCode;  // 0 = same property in both modes
    uint8_t        movieSize;
    uint8_t        knownBit;
    bool           reportable; // has a CameraProperty for onPropChanged
    CameraProperty prop;
};

static const NikonPropSpec NIKON_PROPS[] = {
    { PTP_PROP_APERTURE,         2, NIKON_PROP_MOVIE_FNUMBER, 2, NIKON_KNOWN_FNUMBER,  true,  CameraProperty::APERTURE },
    { PTP_PROP_SHUTTER_SPEED,    4, NIKON_PROP_MOVIE_SHUTTER, 4, NIKON_KNOWN_EXPOSURE, true,  CameraProperty::SHUTTER_SPEED },
    { PTP_PROP_ISO,              2, NIKON_PROP_MOVIE_ISO,     4, NIKON_KNOWN_ISO,      true,  CameraProperty::ISO },
    { PTP_PROP_EXPOSURE_BIAS,    2, NIKON_PROP_MOVIE_BIAS,    2, NIKON_KNOWN_BIAS,     true,  CameraProperty::EXPOSURE_COMPENSATION },
    { PTP_PROP_WHITE_BALANCE,    2, NIKON_PROP_MOVIE_WB,      2, NIKON_KNOWN_WB,       true,  CameraProperty::WHITE_BALANCE },
    { PTP_PROP_EXPOSURE_PROGRAM, 2, 0,                        0, NIKON_KNOWN_PROGRAM,  false, CameraProperty::APERTURE },
};

static uint32_t stateField(const NikonState& s, uint16_t code) {
    switch (code) {
        case PTP_PROP_APERTURE:         return s.fNumber;
        case PTP_PROP_SHUTTER_SPEED:    return s.exposureTime;
        case PTP_PROP_ISO:              return s.iso;
        case PTP_PROP_EXPOSURE_BIAS:    return (uint16_t)s.exposureBias;
        case PTP_PROP_WHITE_BALANCE:    return s.whiteBalance;
        case PTP_PROP_EXPOSURE_PROGRAM: return s.program;
        default:                        return 0;
    }
}

static void setStateField(NikonState& s, uint16_t code, uint32_t v) {
    switch (code) {
        case PTP_PROP_APERTURE:         s.fNumber      = (uint16_t)v; break;
        case PTP_PROP_SHUTTER_SPEED:    s.exposureTime = v;           break;
        case PTP_PROP_ISO:              s.iso          = v;           break;
        case PTP_PROP_EXPOSURE_BIAS:    s.exposureBias = (int16_t)(uint16_t)v; break;
        case PTP_PROP_WHITE_BALANCE:    s.whiteBalance = (uint16_t)v; break;
        case PTP_PROP_EXPOSURE_PROGRAM: s.program      = (uint16_t)v; break;
        default: break;
    }
}

static const NikonPropSpec* findSpec(uint16_t code) {
    for (const NikonPropSpec& p : NIKON_PROPS) if (p.code == code) return &p;
    return nullptr;
}

static uint16_t propCodeFor(CameraProperty prop) {
    switch (prop) {
        case CameraProperty::APERTURE:              return PTP_PROP_APERTURE;
        case CameraProperty::SHUTTER_SPEED:         return PTP_PROP_SHUTTER_SPEED;
        case CameraProperty::ISO:                   return PTP_PROP_ISO;
        case CameraProperty::EXPOSURE_COMPENSATION: return PTP_PROP_EXPOSURE_BIAS;
        case CameraProperty::WHITE_BALANCE:         return PTP_PROP_WHITE_BALANCE;
        default:                                    return 0;
    }
}

// ============================================================================
// Constructor
// ============================================================================

NikonCamera::NikonCamera(PtpIpSession& session)
    : _session(session)
    , _ready(false)
    , _pollFailCount(0)
    , _host{}
    , _state{}
    , _pairState(NikonPairingState::IDLE)
    , _info{}
    , _cameraConfirmed(false)
    , _movie(false)
    , _hasModeProp(true)
    , _onPropChanged(nullptr),       _onPropChangedCtx(nullptr)
    , _onCaptureComplete(nullptr),   _onCaptureCompleteCtx(nullptr)
    , _onConnectionChanged(nullptr), _onConnectionChangedCtx(nullptr)
    , _onBracketedShot(nullptr),     _onBracketedShotCtx(nullptr)
{}

// ============================================================================
// Lifecycle
// ============================================================================

CameraResult NikonCamera::_open(const char* host) {
    strncpy(_host, host, sizeof(_host) - 1);
    _host[sizeof(_host) - 1] = '\0';

    if (!_session.connect(host)) {
        // A ZR that already has a paired host refuses any other GUID here
        if (_session.lastInitFailReason() == PTPIP_INIT_FAIL_REJECTED) {
            LOG_WARNING("camera refused this host's GUID — not paired");
            return CAM_E_NOT_PAIRED;
        }
        if (_session.lastInitFailReason() == PTPIP_INIT_FAIL_BUSY) return CAM_E_BUSY;
        LOG_ERROR("handshake failed");
        return CAM_E_PROTOCOL;
    }
    if (!_session.openSession()) {
        LOG_ERROR("OpenSession failed");
        _session.disconnect();
        return CAM_E_PROTOCOL;
    }
    return CAM_OK;
}

CameraResult NikonCamera::begin(const char* host) {
    _ready = false;
    _pollFailCount = 0;
    _state = {};
    _pairState = NikonPairingState::IDLE;
    _movie = false;
    _hasModeProp = true;    // re-detect: an unpaired camera refuses every property

    CameraResult r = _open(host);
    if (r != CAM_OK) return r;

    r = _readDeviceInfo();
    if (r != CAM_OK) {
        _session.disconnect();
        return r;
    }
    LOG_INFO("Connected to %s %s (firmware %s)", _info.manufacturer, _info.model, _info.deviceVersion);

    // Pairing can't be judged from DeviceInfo: the ZR returns the same restricted
    // operation list (no property ops) to paired and unpaired hosts alike, yet
    // serves property reads to a paired host. Reading a property is the test.
    r = _refreshState(false);
    if (r != CAM_OK) {
        if (r == CAM_E_NOT_PAIRED) LOG_WARNING("begin: camera refused property access — host not paired");
        else                       LOG_ERROR("begin: initial property read failed (%s)", cameraResultStr(r));
        _session.closeSession();
        _session.disconnect();
        return r;
    }

    // The first DeviceInfo may be the restricted pre-authentication list.
    // Read it again now that the camera has accepted us; keep the longer one.
    uint16_t opsBefore = _info.numOperations;
    if (_readDeviceInfo() == CAM_OK && _info.numOperations != opsBefore) {
        LOG_INFO("DeviceInfo after authentication: %u operations (was %u)",
                 _info.numOperations, opsBefore);
    }

    // NX Tether switches the camera to remote application mode before changing
    // settings; do the same. Failure is not fatal.
    if (_info.supportsOperation(PTP_OP_NIKON_CHANGE_APP_MODE)) {
        uint16_t rc;
        if (!_session.sendCommand(PTP_OP_NIKON_CHANGE_APP_MODE, rc, 1) || rc != PTP_RSP_OK) {
            LOG_WARNING("ChangeApplicationMode(1) failed (response 0x%04X)", rc);
        }
    }

    if (!_extraInit()) {
        LOG_ERROR("begin: model-specific init failed");
        _session.closeSession();
        _session.disconnect();
        return CAM_E_PROTOCOL;
    }

    _ready = true;
    if (_onConnectionChanged) _onConnectionChanged(true, _onConnectionChangedCtx);
    LOG_INFO("Nikon camera ready");
    return CAM_OK;
}

void NikonCamera::end() {
    bool wasReady = _ready;
    _ready = false;
    _state = {};
    _pairState = NikonPairingState::IDLE;
    _session.closeSession();
    _session.disconnect();
    if (wasReady && _onConnectionChanged) _onConnectionChanged(false, _onConnectionChangedCtx);
    LOG_INFO("Camera session closed");
}

CameraResult NikonCamera::reconnect() {
    if (_host[0] == '\0') {
        LOG_ERROR("reconnect: begin() was never called on this instance");
        return CAM_E_NOT_CONNECTED;
    }
    char host[sizeof(_host)];
    strcpy(host, _host);
    end();
    return begin(host);
}

bool NikonCamera::isReady() const {
    return _ready;
}

// ============================================================================
// Pairing
// Sequence observed on a Nikon ZR; mirrors a known-working client exactly:
// OpenSession, 0x952B (no other operations first), 0x935A(0x2001), keep the
// connection open until the user confirms on the camera, close without
// CloseSession, reconnect with the same identity.
// ============================================================================

CameraResult NikonCamera::startPairing(const char* host, char* codeOut, size_t codeSize) {
    if (codeOut == nullptr || codeSize == 0) return CAM_E_INVALID_ARG;
    codeOut[0] = '\0';

    if (_ready || _pairState != NikonPairingState::IDLE) end();

    CameraResult r = _open(host);
    if (r == CAM_E_NOT_PAIRED) {
        LOG_ERROR("startPairing: camera refused this host — it may only accept a new host "
                  "while it is waiting for a computer connection");
    }
    if (r != CAM_OK) return r;

    uint8_t data[16];
    size_t  dataLen = 0;
    uint16_t responseCode;
    if (!_session.dataIn(PTP_OP_NIKON_GET_PAIRING_CODE, nullptr, 0,
                         data, sizeof(data), dataLen, responseCode)) {
        _session.disconnect();
        return CAM_E_PROTOCOL;
    }
    if (responseCode != PTP_RSP_OK) {
        LOG_ERROR("startPairing: GetPairingCode response 0x%04X", responseCode);
        _session.disconnect();
        return _mapResponse(responseCode);
    }

    // Payload: uint32 digit count, then one byte per digit holding its value (not ASCII)
    uint32_t digits = (dataLen >= 4)
        ? ((uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24))
        : 0;
    if (digits < 1 || digits > 8 || dataLen < 4 + digits) {
        LOG_ERROR("startPairing: unexpected pairing code payload (%u bytes, %u digits)",
                  (unsigned)dataLen, (unsigned)digits);
        _session.disconnect();
        return CAM_E_PROTOCOL;
    }
    if (codeSize < digits + 1) {
        _session.disconnect();
        return CAM_E_INVALID_ARG;
    }
    for (uint32_t i = 0; i < digits; i++) {
        uint8_t d = data[4 + i];
        if (d > 9) {
            LOG_ERROR("startPairing: unexpected digit encoding 0x%02X", d);
            _session.disconnect();
            return CAM_E_PROTOCOL;
        }
        codeOut[i] = (char)('0' + d);
    }
    codeOut[digits] = '\0';

    _pairState = NikonPairingState::CODE_SHOWN;
    LOG_INFO("Pairing code: %s", codeOut);
    return CAM_OK;
}

CameraResult NikonCamera::confirmPairing() {
    if (_pairState != NikonPairingState::CODE_SHOWN) {
        LOG_ERROR("confirmPairing: call startPairing() first");
        return CAM_E_NOT_READY;
    }

    uint16_t responseCode;
    if (!_session.sendCommand(PTP_OP_NIKON_CONFIRM_PAIRING, responseCode, NIKON_PAIRING_CONFIRM)) {
        cancelPairing();
        return CAM_E_PROTOCOL;
    }
    if (responseCode != PTP_RSP_OK) {
        LOG_ERROR("confirmPairing: response 0x%04X", responseCode);
        cancelPairing();
        return _mapResponse(responseCode);
    }

    _pairState = NikonPairingState::AWAITING_CAMERA;
    _cameraConfirmed = false;
    LOG_INFO("Pairing accepted — confirm this host on the camera");
    return CAM_OK;
}

CameraResult NikonCamera::pollCameraConfirmation() {
    if (_pairState != NikonPairingState::AWAITING_CAMERA) return CAM_E_NOT_READY;
    if (_cameraConfirmed) return CAM_OK;

    uint32_t type;
    uint8_t  payload[32];
    size_t   len;
    while (_session.pollEventChannel(type, payload, sizeof(payload), len)) {
        if (type != PTPIP_TYPE_EVENT || len < 2) continue;
        uint16_t code = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
        LOG_DEBUG("pairing: event 0x%04X", code);
        if (code == PTP_EVT_DEVICE_INFO_CHANGED) {
            _cameraConfirmed = true;
            LOG_INFO("User confirmed pairing on the camera");
            return CAM_OK;
        }
    }
    if (!_session.isConnected()) return CAM_E_NOT_CONNECTED;
    return CAM_E_NOT_READY;
}

CameraResult NikonCamera::waitForCameraConfirmation(uint32_t timeoutMs) {
    uint32_t start = millis();
    while (true) {
        CameraResult r = pollCameraConfirmation();
        if (r != CAM_E_NOT_READY || _pairState != NikonPairingState::AWAITING_CAMERA) return r;
        if (millis() - start >= timeoutMs) return CAM_E_TIMEOUT;
        delay(50);
    }
}

CameraResult NikonCamera::finishPairing() {
    if (_pairState != NikonPairingState::AWAITING_CAMERA) {
        LOG_ERROR("finishPairing: call confirmPairing() first");
        return CAM_E_NOT_READY;
    }
    // Log anything the camera sent while the user was confirming — the protocol
    // around this step is not fully known yet.
    _serviceEvents(PTPIP_LOG_INFO);
    _session.disconnect();   // no CloseSession: the camera resets the connection
    _pairState = NikonPairingState::IDLE;
    LOG_INFO("Pairing finished — reconnect with begin()");
    return CAM_OK;
}

void NikonCamera::cancelPairing() {
    _session.disconnect();
    _pairState = NikonPairingState::IDLE;
}

// ============================================================================
// DeviceInfo
// ============================================================================

CameraResult NikonCamera::_readDeviceInfo() {
    size_t dataLen = 0;
    uint16_t responseCode;
    if (!_session.dataIn(PTP_OP_GET_DEVICE_INFO, nullptr, 0,
                         _dataBuf, sizeof(_dataBuf), dataLen, responseCode)) {
        return CAM_E_PROTOCOL;
    }
    if (responseCode != PTP_RSP_OK) {
        LOG_ERROR("GetDeviceInfo: response 0x%04X", responseCode);
        return _mapResponse(responseCode);
    }
    if (!ptpParseDeviceInfo(_dataBuf, dataLen, _info)) {
        LOG_ERROR("GetDeviceInfo: malformed dataset (%u bytes)", (unsigned)dataLen);
        return CAM_E_PROTOCOL;
    }
    if (_info.truncated) LOG_WARNING("GetDeviceInfo: code lists truncated");
    LOG_DEBUG("DeviceInfo: %u operations, %u events, %u properties",
              _info.numOperations, _info.numEvents, _info.numProperties);
    return CAM_OK;
}

// ============================================================================
// Polling
// ============================================================================

void NikonCamera::poll() {
    if (!_ready) return;

    _serviceEvents(PTPIP_LOG_DEBUG);

    CameraResult r = _refreshState(true);
    if (r == CAM_E_PROTOCOL || r == CAM_E_TIMEOUT) {
        _pollFailCount++;
        LOG_WARNING("poll: property read failed (%u/%u)",
                    (unsigned)_pollFailCount, (unsigned)NIKON_POLL_FAIL_THRESHOLD);
        if (_pollFailCount >= NIKON_POLL_FAIL_THRESHOLD) {
            LOG_ERROR("poll: %u consecutive failures — session dead",
                      (unsigned)NIKON_POLL_FAIL_THRESHOLD);
            _ready = false;
            if (_onConnectionChanged) _onConnectionChanged(false, _onConnectionChangedCtx);
        }
        return;
    }
    _pollFailCount = 0;
}

CameraResult NikonCamera::_refreshState(bool fireCallbacks) {
    _readMode();
    for (const NikonPropSpec& p : NIKON_PROPS) {
        if (!_hasProp(p.code)) continue;
        uint32_t movieCode = _movie ? _movieCode(p) : 0;
        bool movieProp = movieCode != 0;

        uint32_t value = 0;
        CameraResult r = _getProp(movieProp ? movieCode : p.code, value);
        if (r == CAM_E_BUSY) continue;   // try again next poll
        if (movieProp && r != CAM_OK && r != CAM_E_PROTOCOL && r != CAM_E_TIMEOUT) continue;
        if (r != CAM_OK) {
            // An unpaired Nikon refuses property access rather than omitting it
            if ((r == CAM_E_NOT_SUPPORTED || r == CAM_E_NOT_READY) &&
                _info.supportsOperation(PTP_OP_NIKON_GET_PAIRING_CODE)) {
                return CAM_E_NOT_PAIRED;
            }
            return r;
        }

        bool changed = !(_state.known & p.knownBit) || stateField(_state, p.code) != value;
        setStateField(_state, p.code, value);
        bool firstRead = !(_state.known & p.knownBit);
        _state.known |= p.knownBit;

        if (changed && !firstRead && fireCallbacks && p.reportable && _onPropChanged) {
            _onPropChanged(p.prop, value, _onPropChangedCtx);
        }
        if (changed) {
            PtpIpDiagInstance().record(DIAG_EVT_PROP_CHANGED, (uint8_t)(p.code >> 8), (uint8_t)(p.code & 0xFF),
                                       (uint8_t)(value & 0xFF));
        }
    }
    return CAM_OK;
}

bool NikonCamera::_hasExProps() const {
    return _info.supportsOperation(PTP_OP_NIKON_GET_PROP_EX) && _info.supportsOperation(PTP_OP_NIKON_SET_PROP_EX) &&
           _info.supportsOperation(PTP_OP_NIKON_GET_PROP_DESC_EX);
}

// Movie ISO: the ZR's 0xD1AA is read-only; NX Tether writes 32-bit 0x1D09E.
uint32_t NikonCamera::_movieCode(const NikonPropSpec& spec) const {
    if (spec.code == PTP_PROP_ISO && _hasExProps()) return NIKON_PROP_MOVIE_ISO_EX;
    return spec.movieCode;
}

uint8_t NikonCamera::_movieSize(const NikonPropSpec& spec) const {
    if (spec.code == PTP_PROP_ISO && _hasExProps()) return 4;
    return spec.movieSize;
}

bool NikonCamera::_readMode() {
    if (!_hasModeProp) return false;
    uint32_t v;
    CameraResult r = _getProp(NIKON_PROP_PHOTO_MOVIE, v);
    if (r == CAM_E_NOT_SUPPORTED) {
        _hasModeProp = false;    // body without a photo/movie switch: always photo
        _movie = false;
        return false;
    }
    if (r != CAM_OK) return false;
    bool movie = v == 1;
    if (movie == _movie) return false;
    _movie = movie;
    LOG_INFO("Camera switched to %s mode", movie ? "movie" : "photo");
    return true;
}

void NikonCamera::_serviceEvents(uint8_t logLevel) {
    uint32_t type;
    uint8_t  payload[32];
    size_t   len;
    while (_session.pollEventChannel(type, payload, sizeof(payload), len)) {
        if (type == PTPIP_TYPE_EVENT && len >= 2) {
            // Event: code u16, transaction ID u32, then up to three u32 parameters
            uint16_t code = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
            if (len >= 10) {
                uint32_t p1 = (uint32_t)payload[6] | ((uint32_t)payload[7] << 8) |
                              ((uint32_t)payload[8] << 16) | ((uint32_t)payload[9] << 24);
                _ptpLog(logLevel, LOG_TAG, "event 0x%04X param 0x%04X", code, (unsigned)p1);
            } else {
                _ptpLog(logLevel, LOG_TAG, "event 0x%04X (%u bytes)", code, (unsigned)len);
            }
        } else {
            _ptpLog(logLevel, LOG_TAG, "event channel packet type 0x%02X (%u bytes)",
                    (unsigned)type, (unsigned)len);
        }
    }
}

// ============================================================================
// Exposure
// ============================================================================

CameraResult NikonCamera::setAperture(float fstop) {
    if (!(fstop > 0.0f)) return CAM_E_INVALID_ARG;
    return setCameraProperty(CameraProperty::APERTURE, (uint32_t)lroundf(fstop * 100.0f));
}

CameraResult NikonCamera::setShutterSpeed(float seconds) {
    if (!(seconds > 0.0f)) return CAM_E_INVALID_ARG;
    if (_movie) {
        // numerator << 16 | denominator: 1/n for fast speeds, whole seconds otherwise
        uint32_t raw = seconds >= 1.0f ? ((uint32_t)lroundf(seconds) << 16) | 1u
                                       : (1u << 16) | (uint32_t)lroundf(1.0f / seconds);
        return setCameraProperty(CameraProperty::SHUTTER_SPEED, raw);
    }
    long units = lroundf(seconds * 10000.0f);
    if (units < 1) return CAM_E_INVALID_ARG;   // faster than 1/10000 s is not representable
    return setCameraProperty(CameraProperty::SHUTTER_SPEED, (uint32_t)units);
}

CameraResult NikonCamera::setISO(uint16_t iso) {
    // ISO is UINT16 in photo mode and UINT32 in movie mode; _setProp sizes it
    if (iso == 0) return CAM_E_INVALID_ARG;
    return setCameraProperty(CameraProperty::ISO, iso);
}

CameraResult NikonCamera::setExposureCompensation(float ev) {
    if (ev < -32.0f || ev > 32.0f) return CAM_E_INVALID_ARG;
    int16_t units = (int16_t)lroundf(ev * 1000.0f);
    return setCameraProperty(CameraProperty::EXPOSURE_COMPENSATION, (uint16_t)units);
}

CameraResult NikonCamera::setCameraProperty(CameraProperty prop, uint32_t value) {
    if (!_ready) return CAM_E_NOT_CONNECTED;
    uint16_t code = propCodeFor(prop);
    const NikonPropSpec* spec = findSpec(code);
    if (spec == nullptr || !_hasProp(code)) return CAM_E_NOT_SUPPORTED;

    uint32_t movieCode = _movie ? _movieCode(*spec) : 0;
    CameraResult r = movieCode ? _setProp(movieCode, value, _movieSize(*spec))
                               : _setProp(code, value, spec->size);
    if (r == CAM_OK) {
        setStateField(_state, code, value);
        _state.known |= spec->knownBit;
    }
    return r;
}

CameraResult NikonCamera::getCameraProperty(CameraProperty prop, uint32_t& value) {
    if (!_ready) return CAM_E_NOT_CONNECTED;
    uint16_t code = propCodeFor(prop);
    const NikonPropSpec* spec = findSpec(code);
    if (spec == nullptr || !_hasProp(code)) return CAM_E_NOT_SUPPORTED;

    if (_state.known & spec->knownBit) {
        value = stateField(_state, code);
        return CAM_OK;
    }
    return _getProp(_movie && _movieCode(*spec) ? _movieCode(*spec) : code, value);
}

// PTP integer data types: 1 = INT8, 2 = UINT8, ... 8 = UINT64. Returns the size
// in bytes, or 0 for anything else (arrays, strings, 128-bit).
static uint8_t ptpTypeSize(uint16_t type) {
    switch (type) {
        case 0x0001: case 0x0002: return 1;
        case 0x0003: case 0x0004: return 2;
        case 0x0005: case 0x0006: return 4;
        default:                  return 0;
    }
}

// Little-endian integer of `size` bytes at p, sign-extended if the type is signed.
static int32_t ptpReadInt(const uint8_t* p, uint8_t size, bool isSigned) {
    uint32_t v = 0;
    for (uint8_t i = 0; i < size; i++) v |= (uint32_t)p[i] << (8 * i);
    if (isSigned && size < 4 && (v & (1u << (8 * size - 1)))) v |= ~0u << (8 * size);
    return (int32_t)v;
}

CameraResult NikonCamera::readPropDesc(uint32_t propCode, const uint8_t*& data, size_t& len) {
    data = _dataBuf;
    len = 0;
    if (!_ready) return CAM_E_NOT_CONNECTED;
    uint16_t responseCode;
    uint32_t param = propCode;
    uint16_t op = (propCode > 0xFFFF && _hasExProps()) ? PTP_OP_NIKON_GET_PROP_DESC_EX : PTP_OP_GET_PROP_DESC;
    if (!_session.dataIn(op, &param, 1, _dataBuf, sizeof(_dataBuf), len, responseCode)) {
        return CAM_E_PROTOCOL;
    }
    if (responseCode != PTP_RSP_OK) {
        LOG_DEBUG("GetDevicePropDesc 0x%04lX: response 0x%04X", (unsigned long)propCode, responseCode);
        return _mapResponse(responseCode);
    }
    if (len < 5) return CAM_E_PROTOCOL;
    // The Ex dataset starts with a 32-bit code field. Drop its high half so
    // the rest of the dataset sits at the standard offsets.
    if (propCode > 0xFFFF && len >= 9) {
        uint32_t code32 = (uint32_t)_dataBuf[0] | ((uint32_t)_dataBuf[1] << 8) |
                          ((uint32_t)_dataBuf[2] << 16) | ((uint32_t)_dataBuf[3] << 24);
        if (code32 == propCode) {
            memmove(_dataBuf + 2, _dataBuf + 4, len - 4);
            len -= 2;
        }
    }
    return CAM_OK;
}

CameraResult NikonCamera::getVendorPropCodes(uint32_t* codes, size_t max, size_t& count) {
    count = 0;
    if (!_ready) return CAM_E_NOT_CONNECTED;
    if (!_info.supportsOperation(PTP_OP_NIKON_GET_VENDOR_CODES)) return CAM_E_NOT_SUPPORTED;
    size_t   len = 0;
    uint16_t responseCode;
    uint32_t param = NIKON_VENDOR_CODES_PROPS;
    if (!_session.dataIn(PTP_OP_NIKON_GET_VENDOR_CODES, &param, 1, _dataBuf, sizeof(_dataBuf), len, responseCode)) {
        return CAM_E_PROTOCOL;
    }
    if (responseCode != PTP_RSP_OK) {
        LOG_WARNING("GetVendorCodes: response 0x%04X", responseCode);
        return _mapResponse(responseCode);
    }
    if (len < 4) return CAM_E_PROTOCOL;
    uint32_t n = (uint32_t)_dataBuf[0] | ((uint32_t)_dataBuf[1] << 8) |
                 ((uint32_t)_dataBuf[2] << 16) | ((uint32_t)_dataBuf[3] << 24);
    if (len < 4 + (size_t)n * 4) {
        LOG_WARNING("GetVendorCodes: %u codes announced, %u bytes received", (unsigned)n, (unsigned)len);
        n = (uint32_t)((len - 4) / 4);
    }
    for (uint32_t i = 0; i < n && i < max; i++) {
        const uint8_t* p = _dataBuf + 4 + i * 4;
        codes[i] = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    }
    count = n;
    return CAM_OK;
}

CameraResult NikonCamera::getPropOptions(CameraProperty prop, NikonPropOptions& out) {
    uint16_t code = propCodeFor(prop);
    if (code == 0 || !_hasProp(code)) {
        out.form = NikonPropOptions::Form::None;
        out.count = 0;
        return CAM_E_NOT_SUPPORTED;
    }
    const NikonPropSpec* spec = findSpec(code);
    uint32_t wire = code;
    if (_movie && spec && _movieCode(*spec)) wire = _movieCode(*spec);
    return getPropOptions(wire, out);
}

CameraResult NikonCamera::getPropOptions(uint32_t code, NikonPropOptions& out) {
    out.form = NikonPropOptions::Form::None;
    out.writable = false;
    out.count = 0;
    out.truncated = false;

    const uint8_t* d;
    size_t len;
    CameraResult r = readPropDesc(code, d, len);
    if (r != CAM_OK) {
        if (r != CAM_E_NOT_CONNECTED) LOG_WARNING("GetDevicePropDesc 0x%04lX: %s", (unsigned long)code, cameraResultStr(r));
        return r;
    }

    // PropertyCode u16, DataType u16, GetSet u8, FactoryDefault, Current, FormFlag u8, form
    uint16_t type = (uint16_t)(d[2] | (d[3] << 8));
    uint8_t  size = ptpTypeSize(type);
    bool     isSigned = (type & 1) != 0;
    if (size == 0) return CAM_E_NOT_SUPPORTED;
    out.writable = d[4] != 0;
    size_t pos = 5 + size;                       // skip the factory default
    if (len < pos + size + 1) return CAM_E_PROTOCOL;
    out.current = ptpReadInt(d + pos, size, isSigned);
    pos += size;
    uint8_t form = d[pos++];

    if (form == 0x01) {                          // Range: min, max, step
        if (len < pos + 3 * size) return CAM_E_PROTOCOL;
        out.min  = ptpReadInt(d + pos, size, isSigned);
        out.max  = ptpReadInt(d + pos + size, size, isSigned);
        out.step = ptpReadInt(d + pos + 2 * size, size, isSigned);
        out.form = NikonPropOptions::Form::Range;
    } else if (form == 0x02) {                   // Enumeration: u16 count, values
        if (len < pos + 2) return CAM_E_PROTOCOL;
        uint16_t n = (uint16_t)(d[pos] | (d[pos + 1] << 8));
        pos += 2;
        if (len < pos + (size_t)n * size) return CAM_E_PROTOCOL;
        if (n > NikonPropOptions::MAX_VALUES) {
            LOG_WARNING("GetDevicePropDesc 0x%04lX: %u values, keeping %u",
                        (unsigned long)code, n, (unsigned)NikonPropOptions::MAX_VALUES);
            out.truncated = true;
            n = NikonPropOptions::MAX_VALUES;
        }
        for (uint16_t i = 0; i < n; i++) out.values[i] = ptpReadInt(d + pos + (size_t)i * size, size, isSigned);
        out.count = n;
        out.form = NikonPropOptions::Form::List;
    }
    return CAM_OK;
}

CameraResult NikonCamera::setPropertyRaw(uint32_t propCode, uint32_t value) {
    const uint8_t* d;
    size_t len;
    CameraResult r = readPropDesc(propCode, d, len);
    if (r != CAM_OK) return r;
    uint8_t size = ptpTypeSize((uint16_t)(d[2] | (d[3] << 8)));
    if (size == 0) return CAM_E_NOT_SUPPORTED;
    return _setProp(propCode, value, size);
}

bool NikonCamera::_hasProp(uint16_t code) const {
    return _info.numProperties == 0 || _info.supportsProperty(code);
}

// ============================================================================
// Not yet implemented
// ============================================================================

CameraResult NikonCamera::setAEBPriority(AEBPriority)                { return CAM_E_NOT_SUPPORTED; }
CameraResult NikonCamera::setAEBSequenceOrder(AEBSequenceOrder)      { return CAM_E_NOT_SUPPORTED; }
CameraResult NikonCamera::setAEBApertureLimit(float, float)          { return CAM_E_NOT_SUPPORTED; }
CameraResult NikonCamera::setAEBShutterLimit(float, float)           { return CAM_E_NOT_SUPPORTED; }
CameraResult NikonCamera::setAEBISOLimit(uint16_t, uint16_t)         { return CAM_E_NOT_SUPPORTED; }
CameraResult NikonCamera::setAEBStep(float)                          { return CAM_E_NOT_SUPPORTED; }
CameraResult NikonCamera::takeBracketedSequence(int, float)          { return CAM_E_NOT_SUPPORTED; }
CameraResult NikonCamera::initiateCapture()                          { return CAM_E_NOT_SUPPORTED; }
CameraResult NikonCamera::releaseShutter()                           { return CAM_E_NOT_SUPPORTED; }

// ============================================================================
// Query
// ============================================================================

const char* NikonCamera::_modeStr(uint16_t program) const {
    switch (program) {
        case 0x0001: return "M";
        case 0x0002: return "P";
        case 0x0003: return "A";
        case 0x0004: return "S";
        case 0x8010: return "AUTO";
        default:     return "?";
    }
}

CameraResult NikonCamera::getSettings(CameraSettings& out) {
    if (!_ready) return CAM_E_NOT_CONNECTED;
    if (_state.known == 0) _refreshState(false);

    out = {};
    out.version              = CAMERA_SETTINGS_VERSION;
    out.aperture             = NAN;
    out.shutterSpeed         = NAN;
    out.exposureCompensation = NAN;

    // FNumber 0 = no lens, or a lens without electronic contacts: leave as unknown
    if ((_state.known & NIKON_KNOWN_FNUMBER) && _state.fNumber != 0) out.aperture = _state.fNumber / 100.0f;
    if (_state.known & NIKON_KNOWN_EXPOSURE) {
        if (!_movie) {
            out.shutterSpeed = _state.exposureTime / 10000.0f;
        } else {
            uint32_t num = _state.exposureTime >> 16, den = _state.exposureTime & 0xFFFF;
            if (den != 0) out.shutterSpeed = (float)num / den;
        }
    }
    if (_state.known & NIKON_KNOWN_ISO)      out.iso = _state.iso > 0xFFFF ? 0xFFFF : (uint16_t)_state.iso;
    if (_state.known & NIKON_KNOWN_BIAS)     out.exposureCompensation = _state.exposureBias / 1000.0f;
    if (_state.known & NIKON_KNOWN_PROGRAM)  out.mode = _modeStr(_state.program);
    return CAM_OK;
}

void NikonCamera::dumpDiagnostics(Print& out) {
    PtpIpDiagInstance().dump(out);
}

// ============================================================================
// Callbacks
// ============================================================================

void NikonCamera::setOnPropChanged(PropChangedCb cb, void* ctx) {
    _onPropChanged    = cb;
    _onPropChangedCtx = ctx;
}

void NikonCamera::setOnCaptureComplete(CaptureCompleteCb cb, void* ctx) {
    _onCaptureComplete    = cb;
    _onCaptureCompleteCtx = ctx;
}

void NikonCamera::setOnConnectionChanged(ConnectionChangedCb cb, void* ctx) {
    _onConnectionChanged    = cb;
    _onConnectionChangedCtx = ctx;
}

void NikonCamera::setOnBracketedShotComplete(BracketedShotCompleteCb cb, void* ctx) {
    _onBracketedShot    = cb;
    _onBracketedShotCtx = ctx;
}

// ============================================================================
// Property helpers
// ============================================================================

CameraResult NikonCamera::_getProp(uint32_t propCode, uint32_t& value) {
    uint8_t  data[8];
    size_t   dataLen = 0;
    uint16_t responseCode;
    uint32_t param = propCode;
    uint16_t op = (propCode > 0xFFFF && _hasExProps()) ? PTP_OP_NIKON_GET_PROP_EX : PTP_OP_GET_DEVICE_PROP;
    if (!_session.dataIn(op, &param, 1,
                         data, sizeof(data), dataLen, responseCode)) {
        return CAM_E_PROTOCOL;
    }
    if (responseCode != PTP_RSP_OK) {
        LOG_DEBUG("GetDevicePropValue 0x%04lX: response 0x%04X", (unsigned long)propCode, responseCode);
        return _mapResponse(responseCode);
    }
    if (dataLen == 0 || dataLen > 4) {
        LOG_ERROR("GetDevicePropValue 0x%04lX: unexpected %u-byte value", (unsigned long)propCode, (unsigned)dataLen);
        return CAM_E_PROTOCOL;
    }
    value = 0;
    for (size_t i = 0; i < dataLen; i++) value |= (uint32_t)data[i] << (8 * i);
    return CAM_OK;
}

CameraResult NikonCamera::_setProp(uint32_t propCode, uint32_t value, uint8_t size) {
    uint8_t data[4];
    for (uint8_t i = 0; i < size; i++) data[i] = (uint8_t)(value >> (8 * i));
    uint16_t responseCode;
    uint32_t param = propCode;
    uint16_t op = (propCode > 0xFFFF && _hasExProps()) ? PTP_OP_NIKON_SET_PROP_EX : PTP_OP_SET_DEVICE_PROP;
    if (!_session.dataOut(op, &param, 1, data, size, responseCode)) {
        return CAM_E_PROTOCOL;
    }
    if (responseCode != PTP_RSP_OK) {
        LOG_WARNING("SetDevicePropValue 0x%04lX = %u: response 0x%04X",
                    (unsigned long)propCode, (unsigned)value, responseCode);
    }
    return _mapResponse(responseCode);
}

CameraResult NikonCamera::_mapResponse(uint16_t responseCode) const {
    switch (responseCode) {
        case PTP_RSP_OK:                        return CAM_OK;
        case PTP_RSP_DEVICE_BUSY:               return CAM_E_BUSY;
        case PTP_RSP_INVALID_DEVICE_PROP_VALUE: return CAM_E_INVALID_ARG;
        case PTP_RSP_OP_NOT_SUPPORTED:
        case PTP_RSP_PROP_NOT_SUPPORTED:        return CAM_E_NOT_SUPPORTED;
        case PTP_RSP_ACCESS_DENIED:             return CAM_E_NOT_READY;
        default:                                return CAM_E_PROTOCOL;
    }
}
