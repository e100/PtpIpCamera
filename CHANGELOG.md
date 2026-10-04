# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- `PtpIpSession::setHostIdentity()` sets the GUID and host name sent in
  Init_Command_Request, with `hostGuid()` / `hostName()` accessors. Needed for
  cameras that remember a paired host by identity. The default identity is unchanged.
- `NikonCamera`: Nikon PTP/IP control via standard PTP exposure properties, with
  WiFi PIN pairing for newer Z bodies (`startPairing()` / `confirmPairing()` /
  `finishPairing()` / `cancelPairing()`), with `pollCameraConfirmation()` /
  `waitForCameraConfirmation()` to detect the user confirming on the camera.
  Capture and AEB not yet implemented.
- `NikonCamera::getPropOptions()`: reads a property's allowed values (list or
  range), current value and whether it is writable, via GetDevicePropDesc (0x1014).
  Overload by PTP property code, and `readPropDesc()` for the raw dataset.
- `NikonCamera` follows the camera's photo/movie switch (Nikon property 0xD1A6):
  in movie mode aperture, shutter, ISO and exposure compensation use the movie
  properties (0xD1A8-0xD1AB), and white balance uses 0xD23A. `movieMode()`
  reports the switch.
- `NikonCamera::setPropertyRaw()`: write any integer property by PTP code.
- `CAM_E_NOT_PAIRED` result code.
- `PtpDeviceInfo` / `ptpParseDeviceInfo()`: header-only parser for the DeviceInfo dataset.
- `PtpIpSession::dataIn()` / `dataOut()`: transactions using the PTP/IP specification's
  packet types (Start_Data 0x09, Data 0x0A, End_Data 0x0C), any payload size, up to
  5 parameters. Existing Canon transactions are unchanged.
- `PtpIpSession::closeSession()` and `pollEventChannel()` (answers Probe_Request
  keep-alives); `PtpIpTransport::availableEvent()`.

### Changed
- Init_Fail "rejected" log message no longer assumes a Canon camera.
- `NikonCamera` reads and writes the exposure properties even when DeviceInfo
  lists no properties (the ZR's restricted DeviceInfo), instead of skipping them.

## [1.0.0] - 2026-03-15

### Added
- Initial public release
- ICameraControl abstract interface for multi-manufacturer support
- CanonCamera: full PTP/IP implementation for Canon EOS cameras
  - Aperture, shutter speed, ISO control via Canon vendor opcode 0x9110
  - AF + shutter capture (initiateCapture) and shutter-only (releaseShutter)
  - GetEventData polling (0x9116) for state sync with physical dials
  - Automatic Exposure Bracketing (AEB) with configurable priority, order, and limits
  - Exposure compensation (setExposureCompensation) with model-specific range via `_maxECEv()`
  - Generic property access via CameraProperty enum
  - Event callbacks: onPropChanged, onCaptureComplete, onConnectionChanged, onBracketedShotComplete
  - reconnect(): end() + begin() using stored host; use after onConnectionChanged(false)
  - Connection drop detection: 3 consecutive poll() failures set isReady() false and fire onConnectionChanged(false)
  - Input validation: setAperture/setShutterSpeed reject values not in the Canon step table (CAM_E_INVALID_ARG + nearest-valid log message)
  - getSettings() returns NAN for aperture/shutterSpeed until first successful poll()
- SimCamera: full-fidelity simulated camera for testing without hardware
  - Fires onPropChanged callbacks when settings are changed
  - Fires onCaptureComplete after simulated capture delay
  - reconnect() supported
- CanonExposure.h: public APEX encoding/decoding utilities for host-side testing
  - canonApertureIsExact() / canonShutterIsExact() validation helpers
- CameraResult error enum (CAM_OK, CAM_E_BUSY, CAM_E_TIMEOUT, etc.)
- PTPIP_HEX_DUMP build flag for protocol debugging (zero overhead when off)
- PTPIP_NO_DIAG build flag to disable ring buffer and reclaim ~3 KB PSRAM
- SerialShell example: interactive serial test harness with all commands
  - Prompt shows `CAM[name!]>` when camera is selected but disconnected
  - `s` command shows connection status and returns early when disconnected
- ConnectAndShoot example: minimal connect-and-shoot sketch
