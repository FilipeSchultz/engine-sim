#include "../include/avl_emcon.h"

#include <cstring>

namespace {
    // x86/x64 is little-endian, matching the DBC byte order
    void putFloat(uint8_t *data, int byteOffset, float value) {
        std::memcpy(data + byteOffset, &value, sizeof(float));
    }

    void putUint16(uint8_t *data, int byteOffset, uint16_t value) {
        data[byteOffset] = static_cast<uint8_t>(value & 0xFF);
        data[byteOffset + 1] = static_cast<uint8_t>(value >> 8);
    }

    float getFloat(const uint8_t *data, int byteOffset) {
        float value;
        std::memcpy(&value, data + byteOffset, sizeof(float));
        return value;
    }

    uint16_t getUint16(const uint8_t *data, int byteOffset) {
        return static_cast<uint16_t>(data[byteOffset] | (data[byteOffset + 1] << 8));
    }
}

bool avl_emcon::decodeHostControl(const CanInterface::Frame &frame, HostControl *control) {
    if (frame.length < 7) return false;

    control->remoteCommand = frame.data[0] == 1;
    control->controlMode = (frame.data[1] == 1) ? ControlMode::Torque : ControlMode::Speed;
    control->dynoOn = frame.data[2] == 1;
    control->resetKey = frame.data[3] == 1;
    control->remoteTakeover = frame.data[4] == 1;
    control->ignitionKey = (frame.data[5] & 0x01) != 0;
    control->starter = (frame.data[5] & 0x02) != 0;
    control->torqueMode = (frame.data[6] == 1) ? TorqueMode::Td : TorqueMode::Te;

    return true;
}

bool avl_emcon::decodeHostDemand1(const CanInterface::Frame &frame, HostDemand1 *demand) {
    if (frame.length < 8) return false;

    demand->speedDemand_rpm = getFloat(frame.data, 0);
    demand->torqueDemand_Nm = getFloat(frame.data, 4);

    return true;
}

bool avl_emcon::decodeHostDemand2(const CanInterface::Frame &frame, HostDemand2 *demand) {
    if (frame.length < 6) return false;

    demand->alphaDemand_percent = getFloat(frame.data, 0);
    demand->lifesignalFeedback = getUint16(frame.data, 4);

    return true;
}

CanInterface::Frame avl_emcon::encodeBmeControl(const BmeControl &control) {
    CanInterface::Frame frame;
    frame.id = BmeControlId;
    frame.length = 5;
    frame.data[0] = static_cast<uint8_t>(control.systemState);
    frame.data[1] = static_cast<uint8_t>(control.controlMode);
    frame.data[2] = static_cast<uint8_t>(control.systemError);
    frame.data[3] = static_cast<uint8_t>(control.dynoOnOffState);
    frame.data[4] = static_cast<uint8_t>(control.torqueMode);

    return frame;
}

CanInterface::Frame avl_emcon::encodeBmeActual1(float dynoSpeed_rpm, float throttleSet_percent) {
    CanInterface::Frame frame;
    frame.id = BmeActual1Id;
    frame.length = 8;
    putFloat(frame.data, 0, dynoSpeed_rpm);
    putFloat(frame.data, 4, throttleSet_percent);

    return frame;
}

CanInterface::Frame avl_emcon::encodeBmeActual2(float dynoTorque_Nm) {
    CanInterface::Frame frame;
    frame.id = BmeActual2Id;
    frame.length = 6;
    putFloat(frame.data, 0, dynoTorque_Nm);

    return frame;
}

CanInterface::BeforeSend avl_emcon::makeLifesignalWriter() {
    return [lifesignal = uint16_t(0)](CanInterface::Frame *bmeActual2) mutable {
        putUint16(bmeActual2->data, 4, lifesignal);
        lifesignal = (lifesignal >= LifesignalMax) ? 0 : lifesignal + 1;
    };
}
