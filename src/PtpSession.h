#pragma once
#include <stdint.h>
#include <stddef.h>

// ============================================================================
// PtpSession
// Transport-independent PTP transaction interface. NikonCamera talks to the
// camera through this, so the same camera code runs over PTP/IP (WiFi,
// PtpIpSession) or USB (PtpUsbSession).
//
// Transactions follow the PTP model: an operation with up to 5 parameters,
// an optional data phase in one direction, then the camera's response code.
// The methods return false on a transport or framing error; otherwise true,
// with the camera's response code (which may still be an error) set.
// ============================================================================

// One camera event: code plus up to three parameters.
struct PtpEvent {
    uint16_t code;
    uint8_t  numParams;
    uint32_t params[3];
};

class PtpSession {
public:
    virtual ~PtpSession() {}

    // Reach the camera. PTP/IP: host is the camera's address, and the
    // PTP/IP handshake runs. USB: host is ignored; the camera must be
    // attached (see PtpUsbSession).
    virtual bool connect(const char* host) = 0;
    virtual void disconnect() = 0;
    virtual bool isConnected() = 0;

    // Reason the camera refused connect(), if the transport reports one
    // (PTP/IP Init_Fail reason); 0 otherwise.
    virtual uint32_t lastInitFailReason() const { return 0; }

    // OpenSession (0x1002) / CloseSession (0x1003). closeSession() leaves the
    // transport connected; call disconnect() after.
    virtual bool openSession() = 0;
    virtual bool closeSession() = 0;

    // Operation with no data phase.
    virtual bool command(uint16_t opCode, const uint32_t* params, uint8_t numParams,
                         uint16_t& responseCode) = 0;

    // Operation whose data phase comes from the camera. Data beyond
    // maxDataLen is read and discarded; dataLen is the number of bytes stored.
    // If the camera answers without a data phase, dataLen is 0.
    virtual bool dataIn(uint16_t opCode, const uint32_t* params, uint8_t numParams,
                        uint8_t* data, size_t maxDataLen, size_t& dataLen,
                        uint16_t& responseCode) = 0;

    // Operation whose data phase goes to the camera.
    virtual bool dataOut(uint16_t opCode, const uint32_t* params, uint8_t numParams,
                         const uint8_t* data, size_t dataLen,
                         uint16_t& responseCode) = 0;

    // Non-blocking: the next camera event, if one has arrived. Also does the
    // transport's housekeeping (PTP/IP: answers keep-alive probes), so call
    // it regularly while connected.
    virtual bool nextEvent(PtpEvent& event) = 0;

    // Convenience forms of command()
    bool sendCommand(uint16_t opCode, uint16_t& responseCode) {
        return command(opCode, nullptr, 0, responseCode);
    }
    bool sendCommand(uint16_t opCode, uint16_t& responseCode, uint32_t p1) {
        uint32_t params[1] = { p1 };
        return command(opCode, params, 1, responseCode);
    }
    bool sendCommand(uint16_t opCode, uint16_t& responseCode, uint32_t p1, uint32_t p2) {
        uint32_t params[2] = { p1, p2 };
        return command(opCode, params, 2, responseCode);
    }
};
