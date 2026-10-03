#pragma once
#include "ICameraControl.h"
#include "PtpIpSession.h"
#include "PtpDeviceInfo.h"
#include "PtpIpDiag.h"

// ============================================================================
// NikonCamera
// Layer 3: Nikon PTP/IP camera control.
// Depends only on PtpIpSession; has no knowledge of TCP sockets.
//
// Exposure uses the standard PTP device properties (FNumber 0x5007,
// ExposureTime 0x500D, ExposureIndex 0x500F, ExposureBiasCompensation 0x5010,
// WhiteBalance 0x5005), so the values are in PTP units, not Canon APEX codes.
//
// WiFi pairing (newer Nikon Z bodies, e.g. ZR, Z8)
// -----------------------------------------------
// The camera remembers a paired host by the GUID sent in the handshake, so
// call PtpIpSession::setHostIdentity() with the same persisted values before
// every begin() and startPairing().
//
// begin() returns CAM_E_NOT_PAIRED when the camera refuses this host, either
// during the handshake (a ZR with another host paired sends Init_Fail) or by
// refusing property access (reported for a Z8). DeviceInfo is the same
// restricted list for paired and unpaired hosts, so it can't be used to tell.
// To pair:
//
//   char code[9];
//   camera.startPairing(host, code, sizeof(code));  // camera shows a code too
//   // show `code`; the user checks it matches the camera's screen
//   camera.confirmPairing();                        // camera asks the user to confirm
//   camera.waitForCameraConfirmation(60000);        // or call pollCameraConfirmation() from loop()
//   camera.finishPairing();
//   // the camera may restart its WiFi network here: retry begin() for up to a minute
//   camera.begin(host);
//
// Use cancelPairing() to abandon pairing at any step.
// ============================================================================

enum class NikonPairingState {
    IDLE,               ///< Not pairing.
    CODE_SHOWN,         ///< startPairing() succeeded; waiting for confirmPairing().
    AWAITING_CAMERA     ///< confirmPairing() succeeded; waiting for the user to confirm on the camera.
};

// Values a property accepts right now, from GetDevicePropDesc. The camera
// changes them with the exposure mode, lens and other settings, so read them
// again when stepping rather than keeping them.
struct NikonPropOptions {
    static constexpr uint16_t MAX_VALUES = 128;
    enum class Form : uint8_t { None, Range, List };

    Form     form;
    bool     writable;          // false: the camera reports the property as read-only
    int32_t  current;           // sign-extended for signed types (e.g. exposure bias)
    int32_t  min, max, step;    // Form::Range
    uint16_t count;             // Form::List: entries in values[], in the camera's order
    bool     truncated;         // the camera offered more than MAX_VALUES
    int32_t  values[MAX_VALUES];
};

// Cached exposure state, kept current by poll(). Values are raw PTP units.
struct NikonState {
    uint16_t fNumber;        // f-number * 100
    uint32_t exposureTime;   // seconds * 10000
    uint16_t iso;
    int16_t  exposureBias;   // EV * 1000
    uint16_t whiteBalance;
    uint16_t program;        // ExposureProgramMode
    uint8_t  known;          // bitmask of NIKON_KNOWN_* — which fields have been read
};

class NikonCamera : public ICameraControl {
public:
    explicit NikonCamera(PtpIpSession& session);

    // -------------------------------------------------------------------------
    // Lifecycle
    // -------------------------------------------------------------------------

    // Returns CAM_E_NOT_PAIRED if the camera requires this host to be paired.
    CameraResult begin(const char* host) override;
    void end() override;
    bool isReady() const override;
    CameraResult reconnect() override;

    // Re-reads the exposure properties and fires onPropChanged for changes.
    // Also services the event channel (answers keep-alive probes).
    void poll() override;

    // -------------------------------------------------------------------------
    // Pairing (see class comment)
    // -------------------------------------------------------------------------

    // Connect as an unpaired host and request a pairing code. The camera shows
    // the same code. codeOut receives the digits as a null-terminated string
    // (up to 8 digits, so codeSize >= 9 is always enough).
    CameraResult startPairing(const char* host, char* codeOut, size_t codeSize);

    // Accept the code. The camera then asks the user to confirm this host.
    CameraResult confirmPairing();

    // Non-blocking check, for calling from loop() after confirmPairing():
    //   CAM_OK              the user confirmed on the camera (event 0x4008 seen)
    //   CAM_E_NOT_READY     still waiting
    //   CAM_E_NOT_CONNECTED the camera closed the connection
    // Seen on a ZR: the camera sends DeviceInfoChanged (0x4008) when the user confirms.
    CameraResult pollCameraConfirmation();

    // Blocking form of pollCameraConfirmation(). Returns CAM_E_TIMEOUT if the
    // user hasn't confirmed within timeoutMs; the pairing is still pending then.
    CameraResult waitForCameraConfirmation(uint32_t timeoutMs);

    // Call after the user has confirmed on the camera. Closes the connection
    // without CloseSession (the camera resets it during pairing). Then call
    // begin(), retrying: the camera may take a while to bring its WiFi back.
    CameraResult finishPairing();

    // Abandon pairing and close the connection.
    void cancelPairing();

    NikonPairingState pairingState() const { return _pairState; }

    // DeviceInfo read during begin(); empty strings / zero counts before that.
    const PtpDeviceInfo& deviceInfo() const { return _info; }

    // -------------------------------------------------------------------------
    // Core exposure
    // -------------------------------------------------------------------------

    CameraResult setAperture(float fstop) override;
    CameraResult setShutterSpeed(float seconds) override;
    CameraResult setISO(uint16_t iso) override;

    // Raw PTP units: APERTURE f*100, SHUTTER_SPEED s*10000, ISO,
    // EXPOSURE_COMPENSATION EV*1000 (as int16), WHITE_BALANCE PTP enum.
    CameraResult setCameraProperty(CameraProperty prop, uint32_t value) override;
    CameraResult getCameraProperty(CameraProperty prop, uint32_t& value) override;

    CameraResult setExposureCompensation(float ev) override;

    // Read the values the camera accepts for prop, and its current value
    // (raw PTP units, as setCameraProperty()).
    CameraResult getPropOptions(CameraProperty prop, NikonPropOptions& out);

    // -------------------------------------------------------------------------
    // Not yet implemented for Nikon — return CAM_E_NOT_SUPPORTED
    // -------------------------------------------------------------------------

    CameraResult setAEBPriority(AEBPriority priority) override;
    CameraResult setAEBSequenceOrder(AEBSequenceOrder order) override;
    CameraResult setAEBApertureLimit(float minFstop, float maxFstop) override;
    CameraResult setAEBShutterLimit(float minSeconds, float maxSeconds) override;
    CameraResult setAEBISOLimit(uint16_t minISO, uint16_t maxISO) override;
    CameraResult setAEBStep(float ev) override;
    CameraResult takeBracketedSequence(int shotCount, float evStep) override;
    CameraResult initiateCapture() override;
    CameraResult releaseShutter() override;

    // -------------------------------------------------------------------------
    // Query
    // -------------------------------------------------------------------------

    CameraResult getSettings(CameraSettings& out) override;
    void dumpDiagnostics(Print& out) override;

    // -------------------------------------------------------------------------
    // Callbacks
    // -------------------------------------------------------------------------

    void setOnPropChanged(PropChangedCb cb, void* ctx = nullptr) override;
    void setOnCaptureComplete(CaptureCompleteCb cb, void* ctx = nullptr) override;
    void setOnConnectionChanged(ConnectionChangedCb cb, void* ctx = nullptr) override;
    void setOnBracketedShotComplete(BracketedShotCompleteCb cb, void* ctx = nullptr) override;

protected:
    // Called at the end of begin() after the standard steps succeed.
    // Return false to abort begin() with CAM_E_PROTOCOL.
    virtual bool _extraInit() { return true; }

    // Human-readable name for an ExposureProgramMode value.
    virtual const char* _modeStr(uint16_t program) const;

    static constexpr uint8_t NIKON_POLL_FAIL_THRESHOLD = 3;

    PtpIpSession&     _session;
    bool              _ready;
    uint8_t           _pollFailCount;
    char              _host[64];
    NikonState        _state;
    NikonPairingState _pairState;
    PtpDeviceInfo     _info;
    bool              _cameraConfirmed;   // 0x4008 seen while AWAITING_CAMERA
    uint8_t           _dataBuf[8192];   // DeviceInfo and other data-in payloads

    PropChangedCb           _onPropChanged;       void* _onPropChangedCtx;
    CaptureCompleteCb       _onCaptureComplete;   void* _onCaptureCompleteCtx;
    ConnectionChangedCb     _onConnectionChanged; void* _onConnectionChangedCtx;
    BracketedShotCompleteCb _onBracketedShot;     void* _onBracketedShotCtx;

    // Connect, run the handshake and open a session. Disconnects on failure.
    CameraResult _open(const char* host);

    CameraResult _readDeviceInfo();

    // Read every supported exposure property into _state.
    // fireCallbacks: invoke onPropChanged for values that changed.
    CameraResult _refreshState(bool fireCallbacks);

    // Drain the event channel (answers probes; logs events).
    void _serviceEvents(uint8_t logLevel);

    // GetDevicePropValue / SetDevicePropValue for integer properties of 1, 2 or 4 bytes.
    CameraResult _getProp(uint16_t propCode, uint32_t& value);
    CameraResult _setProp(uint16_t propCode, uint32_t value, uint8_t size);

    CameraResult _mapResponse(uint16_t responseCode) const;

    // DeviceInfo lists the property, or lists none at all (the ZR's restricted
    // pre-authentication DeviceInfo, which can't be trusted to be complete).
    bool _hasProp(uint16_t code) const;
};
