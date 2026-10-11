#ifndef ATG_ENGINE_SIM_DYNO_CAN_H
#define ATG_ENGINE_SIM_DYNO_CAN_H

#include "can_interface.h"

#include <cstdint>

// CAN protocol of the small engine bench, as defined in Dyno_Eddy_Brake.dbc:
// the eddy current brake's controller (the bench's ESP32) and the engine's
// controls. The bench automation (LabVIEW RCS) runs the speed / torque PID
// and sends the coil PWM and the engine commands; the brake returns the load
// cell torque and the measured speed. The simulator plays the brake and the
// engine. Little-endian, scaled integers. The ids are consecutive from a
// base (default 0x300).
//
//   base + 0  DYNO_Command (automation -> brake), event or cyclic
//       load           u16  0.01 %, 0..100 % of the brake's maximum torque at
//                           the present speed (linearised by the brake's
//                           controller: torque = drag + load x maximum)
//       flags          u8   bit 0 coil enable
//       counter        u8   alive counter (informative)
//   base + 1  DYNO_Measure (brake -> automation), 10 ms
//       torque         i16  0.001 N m, load cell
//       speed          u16  0.25 rpm, pulse count
//       power          u16  1 W, braking power
//       counter        u8   +1 per transmission
//       status         u8   bit 0 PWM from DYNO_Command (else internal
//                           controller or coil off), bit 1 command timed
//                           out, bit 2 above the brake's power rating,
//                           bit 3 internal controller active
//   base + 2  DYNO_Status (brake -> automation), 100 ms
//       load           u16  0.01 %, load applied (DYNO_Command or internal
//                           controller)
//       duty           u16  0.01 %, coil PWM duty
//       excitation     u16  0.01 %, coil current / maximum (after the coil's lag)
//   base + 3  ENGINE_Command (automation -> engine), cyclic (20 ms). While
//             it keeps coming the automation is in control (remote); when
//             it stops the engine is brought to a safe state
//       flags          u8   bit 0 ignition, bit 1 starter
//       throttle       u16  0.01 %
//       counter        u8   alive counter (informative)
//   base + 4  ENGINE_Status (engine -> automation), 100 ms
//       flags          u8   bit 0 remote (automation in control), bit 1
//                           ignition on, bit 2 starter on, bit 3 command
//                           timed out
//       throttle       u16  0.01 %, applied
namespace dyno_can {
    constexpr uint32_t DefaultBaseId = 0x300;
    constexpr int MeasurePeriodMs = 10;
    constexpr int StatusPeriodMs = 100;

    struct Command {
        double load = 0;            // 0 .. 1
        bool enable = false;
        uint8_t counter = 0;
    };

    struct Measure {
        double torque_Nm = 0;
        double speed_rpm = 0;
        double power_W = 0;
        bool external = false;
        bool timeout = false;
        bool overPower = false;
        bool internalControl = false;
    };

    struct Status {
        double load = 0;            // 0 .. 1
        double duty = 0;            // 0 .. 1
        double excitation = 0;      // 0 .. 1
    };

    constexpr int EngineStatusPeriodMs = 100;

    struct EngineCommand {
        bool ignition = false;
        bool starter = false;
        double throttle = 0;        // 0 .. 1
        uint8_t counter = 0;
    };

    struct EngineStatus {
        bool remote = false;
        bool ignition = false;
        bool starter = false;
        bool timeout = false;
        double throttle = 0;        // 0 .. 1
    };

    // false if the frame is too short to hold the message
    bool decodeCommand(const CanInterface::Frame &frame, Command *command);
    bool decodeEngineCommand(const CanInterface::Frame &frame, EngineCommand *command);

    CanInterface::Frame encodeMeasure(uint32_t baseId, const Measure &measure);
    CanInterface::Frame encodeStatus(uint32_t baseId, const Status &status);
    CanInterface::Frame encodeEngineStatus(uint32_t baseId, const EngineStatus &status);

    // BeforeSend callback for DYNO_Measure: alive counter, +1 per transmission
    CanInterface::BeforeSend makeCounterWriter();
}

#endif /* ATG_ENGINE_SIM_DYNO_CAN_H */
