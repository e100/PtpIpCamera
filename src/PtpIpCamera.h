#pragma once

// PtpIpCamera — primary include
//
// Including this header brings in the full public API:
//   ICameraControl   — abstract interface your application depends on
//   CanonCamera      — Canon EOS base class
//   Canon5DMkIV      — Canon EOS 5D Mark IV concrete class
//   NikonCamera      — Nikon (incl. Z-series WiFi PIN pairing)
//   SimCamera        — simulated camera for testing without hardware
//   PtpIpLog         — logging callback registration
//
// For most applications, this is the only header you need to include.

#include "ICameraControl.h"
#include "CanonCamera.h"
#include "Canon5DMkIV.h"
#include "NikonCamera.h"
#include "SimCamera.h"
#include "PtpIpTransport.h"
#include "PtpSession.h"
#include "PtpIpSession.h"
#include "PtpUsbSession.h"   // ESP32-S2/S3 only (PTP_USB_SUPPORTED)
#include "PtpIpLog.h"
