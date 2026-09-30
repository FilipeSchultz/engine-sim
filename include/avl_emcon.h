#ifndef ATG_ENGINE_SIM_AVL_EMCON_H
#define ATG_ENGINE_SIM_AVL_EMCON_H

#include "can_interface.h"

#include <cstdint>

// AVL dyno EMCON CAN protocol, as defined in AVL_Dyno_EMCON_v2.7.dbc.
// The simulator plays the role of the BME (dyno controller) and the test
// bench automation is the HOST. All signals are little-endian; speeds,
// torques and percentages are IEEE 754 floats.
namespace avl_emcon {
    constexpr uint32_t HostControlId = 0x100;
    constexpr uint32_t HostDemand1Id = 0x110;
    constexpr uint32_t HostDemand2Id = 0x111;
    constexpr uint32_t BmeControlId = 0x200;
    constexpr uint32_t BmeActual1Id = 0x211;
    constexpr uint32_t BmeActual2Id = 0x212;

    constexpr int BmeActualPeriodMs = 20;
    constexpr int BmeControlPeriodMs = 100;

    constexpr uint16_t LifesignalMax = 10000;

    enum class SystemState : uint8_t {
        NotManual = 0,
        Manual = 1,
        Remote = 2
    };

    enum class ControlMode : uint8_t {
        Speed = 0,
        Torque = 1
    };

    enum class SystemError : uint8_t {
        Trouble = 0,
        NoTrouble = 1
    };

    enum class DynoOnOffState : uint8_t {
        Off = 0,
        On = 1,
        Transition = 2
    };

    enum class TorqueMode : uint8_t {
        Te = 0,
        Td = 1
    };

    struct BmeControl {
        SystemState systemState = SystemState::Manual;
        ControlMode controlMode = ControlMode::Speed;
        SystemError systemError = SystemError::NoTrouble;
        DynoOnOffState dynoOnOffState = DynoOnOffState::Off;
        TorqueMode torqueMode = TorqueMode::Te;
    };

    struct HostControl {
        bool remoteCommand = false;
        ControlMode controlMode = ControlMode::Speed;
        bool dynoOn = false;
        bool resetKey = false;
        bool remoteTakeover = false;
        bool ignitionKey = false;   // engine-sim extension
        bool starter = false;       // engine-sim extension
        TorqueMode torqueMode = TorqueMode::Te;
    };

    struct HostDemand1 {
        float speedDemand_rpm = 0.0f;
        float torqueDemand_Nm = 0.0f;
    };

    struct HostDemand2 {
        float alphaDemand_percent = 0.0f;
        uint16_t lifesignalFeedback = 0;
    };

    // Return false if the frame is too short to hold the message
    bool decodeHostControl(const CanInterface::Frame &frame, HostControl *control);
    bool decodeHostDemand1(const CanInterface::Frame &frame, HostDemand1 *demand);
    bool decodeHostDemand2(const CanInterface::Frame &frame, HostDemand2 *demand);

    CanInterface::Frame encodeBmeControl(const BmeControl &control);
    CanInterface::Frame encodeBmeActual1(float dynoSpeed_rpm, float throttleSet_percent);
    // Lifesignal bytes are filled in by the writer from makeLifesignalWriter()
    CanInterface::Frame encodeBmeActual2(float dynoTorque_Nm);

    // BeforeSend callback for BME_Actual_2: writes a lifesignal that advances
    // once per transmission, regardless of the render frame rate
    CanInterface::BeforeSend makeLifesignalWriter();
}

#endif /* ATG_ENGINE_SIM_AVL_EMCON_H */
