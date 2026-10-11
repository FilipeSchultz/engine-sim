#include "../include/dyno_can.h"

#include <algorithm>
#include <cmath>

namespace {
    void putUint16(uint8_t *data, int byteOffset, uint16_t value) {
        data[byteOffset] = static_cast<uint8_t>(value & 0xFF);
        data[byteOffset + 1] = static_cast<uint8_t>(value >> 8);
    }

    uint16_t getUint16(const uint8_t *data, int byteOffset) {
        return static_cast<uint16_t>(data[byteOffset] | (data[byteOffset + 1] << 8));
    }

    uint16_t scaledU16(double value, double resolution) {
        return static_cast<uint16_t>(std::clamp(std::lround(value / resolution), 0L, 65535L));
    }

    int16_t scaledI16(double value, double resolution) {
        return static_cast<int16_t>(std::clamp(std::lround(value / resolution), -32768L, 32767L));
    }
}

bool dyno_can::decodeCommand(const CanInterface::Frame &frame, Command *command) {
    if (frame.length < 3) return false;

    command->load = std::clamp(getUint16(frame.data, 0) * 0.0001, 0.0, 1.0);
    command->enable = (frame.data[2] & 0x01) != 0;
    command->counter = (frame.length >= 4) ? frame.data[3] : 0;

    return true;
}

bool dyno_can::decodeEngineCommand(const CanInterface::Frame &frame, EngineCommand *command) {
    if (frame.length < 3) return false;

    command->ignition = (frame.data[0] & 0x01) != 0;
    command->starter = (frame.data[0] & 0x02) != 0;
    command->throttle = std::clamp(getUint16(frame.data, 1) * 0.0001, 0.0, 1.0);
    command->counter = (frame.length >= 4) ? frame.data[3] : 0;

    return true;
}

CanInterface::Frame dyno_can::encodeMeasure(uint32_t baseId, const Measure &measure) {
    CanInterface::Frame frame;
    frame.id = baseId + 1;
    frame.length = 8;
    putUint16(frame.data, 0, static_cast<uint16_t>(scaledI16(measure.torque_Nm, 0.001)));
    putUint16(frame.data, 2, scaledU16(measure.speed_rpm, 0.25));
    putUint16(frame.data, 4, scaledU16(measure.power_W, 1.0));
    // data[6]: counter, written at each transmission
    frame.data[7] = static_cast<uint8_t>(
        (measure.external ? 0x01 : 0)
        | (measure.timeout ? 0x02 : 0)
        | (measure.overPower ? 0x04 : 0)
        | (measure.internalControl ? 0x08 : 0));

    return frame;
}

CanInterface::Frame dyno_can::encodeStatus(uint32_t baseId, const Status &status) {
    CanInterface::Frame frame;
    frame.id = baseId + 2;
    frame.length = 6;
    putUint16(frame.data, 0, scaledU16(status.load * 100.0, 0.01));
    putUint16(frame.data, 2, scaledU16(status.duty * 100.0, 0.01));
    putUint16(frame.data, 4, scaledU16(status.excitation * 100.0, 0.01));

    return frame;
}

CanInterface::Frame dyno_can::encodeEngineStatus(uint32_t baseId, const EngineStatus &status) {
    CanInterface::Frame frame;
    frame.id = baseId + 4;
    frame.length = 3;
    frame.data[0] = static_cast<uint8_t>(
        (status.remote ? 0x01 : 0)
        | (status.ignition ? 0x02 : 0)
        | (status.starter ? 0x04 : 0)
        | (status.timeout ? 0x08 : 0));
    putUint16(frame.data, 1, scaledU16(status.throttle * 100.0, 0.01));

    return frame;
}

CanInterface::BeforeSend dyno_can::makeCounterWriter() {
    return [counter = uint8_t(0)](CanInterface::Frame *measure) mutable {
        measure->data[6] = counter++;
    };
}
