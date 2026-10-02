#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

// ============================================================================
// PtpDeviceInfo
// Parsed form of the PTP DeviceInfo dataset (GetDeviceInfo, 0x1001).
// Vendor-neutral and header-only — no Arduino dependency, usable in native tests.
//
// Dataset layout (PIMA 15740, all little-endian):
//   UINT16  StandardVersion
//   UINT32  VendorExtensionID
//   UINT16  VendorExtensionVersion
//   String  VendorExtensionDesc
//   UINT16  FunctionalMode
//   AUINT16 OperationsSupported
//   AUINT16 EventsSupported
//   AUINT16 DevicePropertiesSupported
//   AUINT16 CaptureFormats
//   AUINT16 ImageFormats
//   String  Manufacturer, Model, DeviceVersion, SerialNumber
//
// String:  UINT8 numChars (including null; 0 = empty), then numChars UTF-16LE units.
// AUINT16: UINT32 count, then count UINT16 values.
// ============================================================================

struct PtpDeviceInfo {
    static constexpr size_t MAX_OPERATIONS = 256;
    static constexpr size_t MAX_EVENTS     = 128;
    static constexpr size_t MAX_PROPERTIES = 512;
    static constexpr size_t MAX_STRING     = 48;   // bytes incl. null; longer strings are truncated

    uint16_t standardVersion;
    uint32_t vendorExtensionId;
    uint16_t vendorExtensionVersion;
    uint16_t functionalMode;

    uint16_t operations[MAX_OPERATIONS];  uint16_t numOperations;
    uint16_t events[MAX_EVENTS];          uint16_t numEvents;
    uint16_t properties[MAX_PROPERTIES];  uint16_t numProperties;
    bool     truncated;   // a code list was longer than its array

    char manufacturer[MAX_STRING];
    char model[MAX_STRING];
    char deviceVersion[MAX_STRING];
    char serialNumber[MAX_STRING];

    bool supportsOperation(uint16_t code) const { return _contains(operations, numOperations, code); }
    bool supportsEvent(uint16_t code) const     { return _contains(events, numEvents, code); }
    bool supportsProperty(uint16_t code) const  { return _contains(properties, numProperties, code); }

private:
    static bool _contains(const uint16_t* list, uint16_t n, uint16_t code) {
        for (uint16_t i = 0; i < n; i++) if (list[i] == code) return true;
        return false;
    }
};

// Bounds-checked little-endian reader used by ptpParseDeviceInfo().
class PtpDataReader {
public:
    PtpDataReader(const uint8_t* buf, size_t len) : _buf(buf), _len(len), _off(0), _ok(true) {}

    bool ok() const { return _ok; }

    uint8_t u8() {
        if (!_need(1)) return 0;
        return _buf[_off++];
    }
    uint16_t u16() {
        if (!_need(2)) return 0;
        uint16_t v = (uint16_t)_buf[_off] | ((uint16_t)_buf[_off + 1] << 8);
        _off += 2;
        return v;
    }
    uint32_t u32() {
        if (!_need(4)) return 0;
        uint32_t v = (uint32_t)_buf[_off]
                   | ((uint32_t)_buf[_off + 1] << 8)
                   | ((uint32_t)_buf[_off + 2] << 16)
                   | ((uint32_t)_buf[_off + 3] << 24);
        _off += 4;
        return v;
    }

    // PTP String -> ASCII (non-ASCII characters become '?'). out always null-terminated.
    void string(char* out, size_t outSize) {
        uint8_t numChars = u8();
        size_t w = 0;
        for (uint8_t i = 0; i < numChars && _ok; i++) {
            uint16_t c = u16();
            if (c == 0) continue;
            if (w + 1 < outSize) out[w++] = (c >= 0x20 && c < 0x7F) ? (char)c : '?';
        }
        if (outSize > 0) out[w] = '\0';
    }

    // AUINT16 -> out[], keeping at most maxCount values. Returns the stored count;
    // sets truncated if the array was longer.
    uint16_t u16Array(uint16_t* out, size_t maxCount, bool& truncated) {
        uint32_t count = u32();
        if (!_ok) return 0;
        if ((size_t)count * 2 > _len - _off) { _ok = false; return 0; }
        uint16_t stored = 0;
        for (uint32_t i = 0; i < count; i++) {
            uint16_t v = u16();
            if (out && stored < maxCount) out[stored++] = v;
            else if (out) truncated = true;
        }
        return stored;
    }

private:
    const uint8_t* _buf;
    size_t _len;
    size_t _off;
    bool   _ok;

    bool _need(size_t n) {
        if (!_ok || _len - _off < n) { _ok = false; return false; }
        return true;
    }
};

// Parse a DeviceInfo dataset. Returns false if the data is malformed or short.
inline bool ptpParseDeviceInfo(const uint8_t* buf, size_t len, PtpDeviceInfo& out) {
    memset(&out, 0, sizeof(out));
    PtpDataReader r(buf, len);
    char scratch[2];

    out.standardVersion        = r.u16();
    out.vendorExtensionId      = r.u32();
    out.vendorExtensionVersion = r.u16();
    r.string(scratch, sizeof(scratch));  // VendorExtensionDesc — not kept
    out.functionalMode         = r.u16();
    out.numOperations = r.u16Array(out.operations, PtpDeviceInfo::MAX_OPERATIONS, out.truncated);
    out.numEvents     = r.u16Array(out.events,     PtpDeviceInfo::MAX_EVENTS,     out.truncated);
    out.numProperties = r.u16Array(out.properties, PtpDeviceInfo::MAX_PROPERTIES, out.truncated);
    bool unused = false;
    r.u16Array(nullptr, 0, unused);  // CaptureFormats
    r.u16Array(nullptr, 0, unused);  // ImageFormats
    r.string(out.manufacturer,  sizeof(out.manufacturer));
    r.string(out.model,         sizeof(out.model));
    r.string(out.deviceVersion, sizeof(out.deviceVersion));
    r.string(out.serialNumber,  sizeof(out.serialNumber));
    return r.ok();
}
