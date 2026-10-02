#pragma once
#include <stdint.h>
#include <stddef.h>

// ============================================================================
// PTP/IP Protocol Constants
// Reference: PIMA 15740:2000 + Canon proprietary extensions
// All multi-byte values are Little-Endian on the wire
// ============================================================================

// Network
static constexpr uint16_t PTPIP_PORT        = 15740;
static constexpr uint8_t  PTPIP_HEADER_SIZE = 8;  // length (4) + type (4)

// Host identity sent in Init_Command_Request
static constexpr size_t   PTPIP_GUID_SIZE          = 16;
static constexpr size_t   PTPIP_MAX_HOST_NAME_LEN  = 63;  // characters, excluding null terminator

// ----------------------------------------------------------------------------
// PTP/IP Packet Types
// ----------------------------------------------------------------------------
static constexpr uint32_t PTPIP_PKT_INIT_CMD_REQ        = 0x00000001;
static constexpr uint32_t PTPIP_PKT_INIT_CMD_ACK        = 0x00000002;
static constexpr uint32_t PTPIP_PKT_INIT_EVT_REQ        = 0x00000003;
static constexpr uint32_t PTPIP_PKT_INIT_EVT_ACK        = 0x00000004;
static constexpr uint32_t PTPIP_PKT_CMD_REQUEST         = 0x00000006;
static constexpr uint32_t PTPIP_PKT_CMD_RESPONSE        = 0x00000007;
static constexpr uint32_t PTPIP_PKT_EVENT               = 0x00000009;
static constexpr uint32_t PTPIP_PKT_DATA_START          = 0x0000000A;
static constexpr uint32_t PTPIP_PKT_DATA_END            = 0x0000000C;
// Canon vendor extension: Start_Data uses type 0x09 instead of 0x0A
static constexpr uint32_t PTPIP_PKT_CANON_DATA_START    = 0x00000009;

// Packet types as numbered by the PTP/IP specification (CIPA DC-005).
// The constants above are used by the Canon code paths and are numbered
// differently (the spec's Start_Data is 0x09, Data is 0x0A, Event is 0x08).
// New code — PtpIpSession::dataIn()/dataOut() and NikonCamera — uses these.
static constexpr uint32_t PTPIP_TYPE_EVENT              = 0x00000008;
static constexpr uint32_t PTPIP_TYPE_START_DATA         = 0x00000009;
static constexpr uint32_t PTPIP_TYPE_DATA               = 0x0000000A;
static constexpr uint32_t PTPIP_TYPE_CANCEL             = 0x0000000B;
static constexpr uint32_t PTPIP_TYPE_END_DATA           = 0x0000000C;
static constexpr uint32_t PTPIP_TYPE_PROBE_REQUEST      = 0x0000000D;
static constexpr uint32_t PTPIP_TYPE_PROBE_RESPONSE     = 0x0000000E;

// Init_Fail reason codes (payload of packet type 0x05)
static constexpr uint32_t PTPIP_INIT_FAIL_REJECTED      = 0x00000001;  // host refused (e.g. not paired)
static constexpr uint32_t PTPIP_INIT_FAIL_BUSY          = 0x00000002;
static constexpr uint32_t PTPIP_INIT_FAIL_UNSPECIFIED   = 0x00000003;

// Maximum number of parameters in a PTP operation request
static constexpr uint8_t  PTP_MAX_PARAMS                = 5;

// ----------------------------------------------------------------------------
// PTP Operation Codes
// ----------------------------------------------------------------------------
static constexpr uint16_t PTP_OP_GET_DEVICE_INFO  = 0x1001;
static constexpr uint16_t PTP_OP_OPEN_SESSION     = 0x1002;
static constexpr uint16_t PTP_OP_CLOSE_SESSION    = 0x1003;
static constexpr uint16_t PTP_OP_GET_DEVICE_PROP  = 0x1015;
static constexpr uint16_t PTP_OP_SET_DEVICE_PROP  = 0x1016;
static constexpr uint16_t PTP_OP_INITIATE_CAPTURE = 0x100E;

// Canon proprietary
static constexpr uint16_t PTP_OP_CANON_SET_PROP      = 0x9110;  // vendor SetDevicePropValue
static constexpr uint16_t PTP_OP_CANON_SET_REMOTE    = 0x9114;
static constexpr uint16_t PTP_OP_CANON_SET_EVENT     = 0x9115;
static constexpr uint16_t PTP_OP_CANON_GET_EVENT     = 0x9116;  // poll camera event queue
static constexpr uint16_t PTP_OP_CANON_START_CAPTURE = 0x9128;  // half-press(focus) or full-press(release)
static constexpr uint16_t PTP_OP_CANON_STOP_CAPTURE  = 0x9129;  // release full-press or half-press

// Nikon proprietary — WiFi PIN pairing (Nikon Z bodies, e.g. ZR, Z8)
static constexpr uint16_t PTP_OP_NIKON_GET_PAIRING_CODE = 0x952B;  // data in: uint32 digit count + one byte per digit (0-9)
static constexpr uint16_t PTP_OP_NIKON_CONFIRM_PAIRING  = 0x935A;  // param1 = NIKON_PAIRING_CONFIRM
static constexpr uint32_t NIKON_PAIRING_CONFIRM         = 0x00002001;

// Canon GetEventData segment event types (uint32 in the payload)
static constexpr uint32_t CANON_EVT_PROP_CHANGED     = 0x0000c189; // property value changed
static constexpr uint32_t CANON_EVT_ALLOWED_CHANGED  = 0x0000c18a; // allowed values changed
static constexpr uint32_t CANON_EVT_CAMERA_STATUS    = 0x0000c18b; // camera busy/ready state transition (fires twice per shot)
static constexpr uint32_t CANON_EVT_OBJECT_ADDED     = 0x0000c1a5; // file written to storage (once per output file)
static constexpr uint32_t CANON_EVT_TERMINATOR       = 0x00000000; // end of event list

// Canon capture phase parameter values
static constexpr uint32_t CANON_CAPTURE_PHASE_FOCUS   = 0x00000001; // AF / half-press
static constexpr uint32_t CANON_CAPTURE_PHASE_RELEASE = 0x00000002; // shutter / full-press

// ----------------------------------------------------------------------------
// PTP Response Codes
// ----------------------------------------------------------------------------
static constexpr uint16_t PTP_RSP_OK             = 0x2001;
static constexpr uint16_t PTP_RSP_DEVICE_BUSY    = 0x2019;
static constexpr uint16_t PTP_RSP_INCOMPLETE_TX  = 0x2007;
static constexpr uint16_t PTP_RSP_OP_NOT_SUPPORTED          = 0x2005;
static constexpr uint16_t PTP_RSP_PROP_NOT_SUPPORTED        = 0x200A;
static constexpr uint16_t PTP_RSP_ACCESS_DENIED             = 0x200F;
static constexpr uint16_t PTP_RSP_INVALID_DEVICE_PROP_VALUE = 0x201C;

// ----------------------------------------------------------------------------
// PTP Event Codes
// ----------------------------------------------------------------------------
static constexpr uint16_t PTP_EVT_OBJECT_ADDED       = 0x4002;
static constexpr uint16_t PTP_EVT_CAPTURE_COMPLETE   = 0x400D;
static constexpr uint16_t PTP_EVT_DEVICE_INFO_CHANGED = 0x4008;  // Nikon: sent when the user confirms pairing on the camera

// ----------------------------------------------------------------------------
// PTP Device Property Codes
// ----------------------------------------------------------------------------

// Standard PTP property codes (used for GetDevicePropValue reads)
// Data types are fixed by the PTP standard.
static constexpr uint16_t PTP_PROP_WHITE_BALANCE = 0x5005;  // UINT16 enum
static constexpr uint16_t PTP_PROP_APERTURE      = 0x5007;  // UINT16, f-number * 100
static constexpr uint16_t PTP_PROP_SHUTTER_SPEED = 0x500D;  // UINT32, seconds * 10000
static constexpr uint16_t PTP_PROP_EXPOSURE_PROGRAM = 0x500E;  // UINT16 enum
static constexpr uint16_t PTP_PROP_ISO           = 0x500F;  // UINT16
static constexpr uint16_t PTP_PROP_EXPOSURE_BIAS = 0x5010;  // INT16, EV * 1000

// Canon vendor property codes (used with PTP_OP_CANON_SET_PROP 0x9110)
// Values are APEX-encoded single bytes; see CanonExposure.h for the full tables.
//   Aperture   0xd101  e.g. f/5.6  -> 0x30
//   Shutter    0xd102  e.g. 1/100s -> 0x6D
//   ISO        0xd103  e.g. ISO400 -> 0x58
//   CaptureDst 0xd11c  0x02 = SD card, 0x01 = RAM only
static constexpr uint16_t PTP_PROP_CANON_APERTURE      = 0xd101;
static constexpr uint16_t PTP_PROP_CANON_SHUTTER_SPEED = 0xd102;
static constexpr uint16_t PTP_PROP_CANON_ISO           = 0xd103;
// EC wire encoding: wire = (int8_t)round(ev * 8), stored as uint8 (two's complement for negative).
// 1 stop = 8 units. Range ±3 EV = ±24 units (0x18 / 0xE8). Uses Canon vendor SetProp (0x9110).
static constexpr uint16_t PTP_PROP_CANON_EC            = 0xd104;
static constexpr uint16_t PTP_PROP_CANON_SHOOTING_MODE = 0xd138; // physical mode dial
static constexpr uint16_t PTP_PROP_CANON_CAPTURE_DEST  = 0xd11c;


// ----------------------------------------------------------------------------
// Session / Handshake Constants
// ----------------------------------------------------------------------------
static constexpr uint32_t PTP_VERSION            = 0x00010000;  // PTP v1.0
static constexpr uint32_t PTP_SESSION_ID         = 0x00000001;
static constexpr uint32_t PTP_TXID_OPEN_SESSION  = 0x00000000;  // TransactionID for OpenSession only
static constexpr uint32_t PTP_FIRST_TXID         = 0x00000001;  // TransactionID for all subsequent ops
