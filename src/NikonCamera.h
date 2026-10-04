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

// Cached exposure state, kept current by poll(). Values are raw PTP units,
// from the movie properties while the camera is in movie mode.
struct NikonState {
    uint16_t fNumber;        // f-number * 100
    uint32_t exposureTime;   // seconds * 10000; movie mode: numerator << 16 | denominator
    uint32_t iso;
    int16_t  exposureBias;   // EV * 1000
    uint16_t whiteBalance;
    uint16_t program;        // ExposureProgramMode
    uint8_t  known;          // bitmask of NIKON_KNOWN_* — which fields have been read
};

struct NikonPropSpec;   // NikonCamera.cpp

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

    // True while the camera's photo/movie switch is on movie (read by begin()
    // and poll()). Exposure then uses the movie properties: APERTURE,
    // SHUTTER_SPEED, ISO and EXPOSURE_COMPENSATION address the movie settings,
    // WHITE_BALANCE uses the movie white balance too,
    // and SHUTTER_SPEED's raw value is numerator << 16 | denominator seconds
    // (0x0001007D = 1/125 s) instead of seconds * 10000. The typed setters and
    // getSettings() convert either way. False on bodies without the switch.
    bool movieMode() const { return _movie; }

    // Send ChangeApplicationMode(1) (0x9435) in begin(), as NX Tether does, and
    // ChangeApplicationMode(0) in end(). A ZR needs it to record movies
    // remotely (otherwise MovRecProhibitCondition bit 14 is set and
    // StartMovieRecInCard fails), and shows --m--s for the remaining recording
    // time while a host is connected without it. Settings work either way.
    // Off by default: a host that vanished while the camera was in this mode
    // (an ESP32 being reflashed) has been seen to leave a ZR unresponsive.
    void setApplicationMode(bool enable) { _appMode = enable; }

    // Why movie recording can't start, from MovRecProhibitCondition (0xD0A4):
    // 0 = it can. startMovie() checks it and returns CAM_E_NOT_READY if set.
    CameraResult getMovieProhibit(uint32_t& bits);
    uint32_t     lastMovieProhibit() const { return _lastProhibit; }
    // First reason in a prohibit bitmask, e.g. "card full" (as libgphoto2).
    static const char* movieProhibitText(uint32_t bits);

    // Movie shutter mode (ZR, property 0x1D074): shutter speed or shutter
    // angle. CAM_E_NOT_SUPPORTED on bodies without it. While the camera is in
    // angle mode (movieShutterAngle(), read by poll()), SHUTTER_SPEED addresses
    // the shutter angle (0x1D075) and its raw value is degrees * 100
    // (18000 = 180 degrees); the movie shutter speed is read-only then.
    static constexpr uint8_t NIKON_SHUTTER_MODE_SPEED = 1;
    static constexpr uint8_t NIKON_SHUTTER_MODE_ANGLE = 2;
    CameraResult getMovieShutterMode(uint8_t& mode);
    CameraResult setMovieShutterMode(uint8_t mode);
    bool movieShutterAngle() const { return _movie && _angle; }

    // Dual base ISO for movies (ZR, property 0x1D09D): NIKON_BASE_ISO_LOW or
    // NIKON_BASE_ISO_HIGH. CAM_E_NOT_SUPPORTED on bodies without it. Changing
    // the base changes the ISO values getPropOptions() offers.
    static constexpr uint8_t NIKON_BASE_ISO_LOW  = 1;
    static constexpr uint8_t NIKON_BASE_ISO_HIGH = 2;
    CameraResult getMovieBaseIso(uint8_t& base);
    CameraResult setMovieBaseIso(uint8_t base);

    // DeviceInfo read during begin(); empty strings / zero counts before that.
    const PtpDeviceInfo& deviceInfo() const { return _info; }

    // -------------------------------------------------------------------------
    // Core exposure
    // -------------------------------------------------------------------------

    CameraResult setAperture(float fstop) override;
    CameraResult setShutterSpeed(float seconds) override;
    CameraResult setISO(uint16_t iso) override;

    // Raw PTP units: APERTURE f*100, SHUTTER_SPEED s*10000 (movie mode: see
    // movieMode()), ISO, EXPOSURE_COMPENSATION EV*1000 (as int16),
    // WHITE_BALANCE PTP enum.
    CameraResult setCameraProperty(CameraProperty prop, uint32_t value) override;
    CameraResult getCameraProperty(CameraProperty prop, uint32_t& value) override;

    CameraResult setExposureCompensation(float ev) override;

    // Read the values the camera accepts for prop, and its current value
    // (raw PTP units, as setCameraProperty()).
    CameraResult getPropOptions(CameraProperty prop, NikonPropOptions& out);

    // The same for any property by its PTP code (e.g. a Nikon vendor property).
    // Integer properties only; others return CAM_E_NOT_SUPPORTED.
    CameraResult getPropOptions(uint32_t propCode, NikonPropOptions& out);

    // The raw DevicePropDesc dataset for propCode. data points into an internal
    // buffer and stays valid until the next camera call. Property codes above
    // 0xFFFF (newer Nikon Z bodies) are read with Nikon's Ex operations
    // (0x943A-0x943C); their dataset starts with a 32-bit code, which is cut
    // to 16 bits so the dataset always has the standard layout.
    CameraResult readPropDesc(uint32_t propCode, const uint8_t*& data, size_t& len);

    // The vendor property codes the camera reports with Nikon GetVendorCodes
    // (0x9439), including 32-bit codes (0x1Dxxx) on newer Z bodies. These are
    // not in DeviceInfo. Returns CAM_E_NOT_SUPPORTED if the camera lacks the
    // operation; count is the number of codes found (only max are stored).
    CameraResult getVendorPropCodes(uint32_t* codes, size_t max, size_t& count);

    // Write any integer property by its PTP code, sized from its DevicePropDesc.
    // For experiments: bypasses the cached state, which poll() then refreshes.
    CameraResult setPropertyRaw(uint32_t propCode, uint32_t value);

    // -------------------------------------------------------------------------
    // Capture and recording (sequences seen from NX Tether on a ZR)
    // -------------------------------------------------------------------------

    // Photo: autofocus then release (initiateCapture) or release only
    // (releaseShutter), saving to the card: InitiateCaptureRecInMedia (0x9207).
    CameraResult initiateCapture() override;
    CameraResult releaseShutter() override;

    // Autofocus once without releasing (AfDrive, 0x90C1). Not seen from NX
    // Tether; libgphoto2 uses it on other Nikon bodies.
    CameraResult autofocus();

    // Movie recording to the card. startMovie(): StartLiveView (0x9201), then
    // StartMovieRecInCard (0x920A). stopMovie(): EndMovieRec (0x920B), then
    // EndLiveView (0x9202). isRecording() follows the camera, whoever started
    // the recording: poll() reads MovRecProhibitCondition (0xD0A4), whose bit
    // 10 means "already recording".
    CameraResult startMovie();
    CameraResult stopMovie();
    bool isRecording() const { return _recording; }

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
    bool              _movie;             // camera's photo/movie switch is on movie
    bool              _hasModeProp;       // camera answers NIKON_PROP_PHOTO_MOVIE
    bool              _angle;             // movie shutter is in angle mode
    bool              _appMode;           // send ChangeApplicationMode(1) in begin()
    bool              _recording;         // camera is recording a movie (D0A4 bit 10, or our start/stop)
    bool              _inAppMode;         // ChangeApplicationMode(1) succeeded this session
    uint32_t          _lastProhibit;      // MovRecProhibitCondition seen by the last startMovie()
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
    CameraResult _getProp(uint32_t propCode, uint32_t& value);
    CameraResult _setProp(uint32_t propCode, uint32_t value, uint8_t size);

    CameraResult _mapResponse(uint16_t responseCode) const;

    // DeviceInfo lists the property, or lists none at all (the ZR's restricted
    // pre-authentication DeviceInfo, which can't be trusted to be complete).
    bool _hasProp(uint16_t code) const;

    // Read the photo/movie switch. Returns true if the mode changed.
    bool _readMode();

    // The property a spec addresses in movie mode (0 = same as photo).
    uint32_t _movieCode(const NikonPropSpec& spec) const;
    uint8_t  _movieSize(const NikonPropSpec& spec) const;

    // The camera has Nikon's 32-bit property operations (0x943A-0x943C).
    bool _hasExProps() const;

    // Send an operation with no data phase; maps the response.
    CameraResult _command(uint16_t op, uint8_t numParams = 0, uint32_t p1 = 0, uint32_t p2 = 0);

    // Poll DeviceReady (0x90C8) until the camera is not busy, up to timeoutMs.
    CameraResult _waitReady(uint32_t timeoutMs);
};
