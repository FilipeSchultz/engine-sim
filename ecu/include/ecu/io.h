#ifndef ECU_IO_H
#define ECU_IO_H

#include <vector>

// The ECU's "pins": everything the ECU knows about the engine comes in through
// EcuInputs (sensors) and everything it does goes out through EcuOutputs
// (actuators). The ECU never sees the simulation; the simulation side
// (EcuHardware in engine-sim) plays the sensors, injectors and coils.
namespace ecu {
    // Engine variant coding, set once when the engine is attached
    struct EcuConfig {
        int cylinders = 0;

        // Firing TDC of each cylinder on the 0..720 deg crank / cam angle
        std::vector<double> firingTdc_deg;

        // Angle domain on an external I/O controller (the ECU on a PC, the
        // crank, injectors and coils on a microcontroller): rpm and map_kPa
        // arrive averaged over the last firing interval, cycleAngle_deg is
        // not used, and the injection goes out as a command (injection_ms,
        // injectionAngle_deg) instead of pulses scheduled per step
        bool remoteIo = false;
    };

    // Sampled every simulation step (remote I/O: every sample received from
    // the I/O controller)
    struct EcuInputs {
        double time_s = 0;

        // Crank + cam sensor
        double cycleAngle_deg = 0;      // 0..720, increasing with rotation
        double rpm = 0;                 // instantaneous

        double map_kPa = 0;             // manifold pressure, instantaneous
        double iat_C = 25;              // intake air temperature
        double clt_C = 90;              // coolant temperature
        double tps_percent = 0;         // throttle plate opening
        double lambda = 0;              // wideband lambda sensor (0 = not ready)
        double battery_V = 14;
    };

    struct EcuOutputs {
        static constexpr int MaxInjections = 32;

        // Function ownership: when false the engine runs on its own mixture
        // (carburettor) / timing curve, as defined by the engine script
        bool fuelControl = false;
        bool sparkControl = false;

        double sparkAdvance_deg = 0;    // before firing TDC, all cylinders

        // Injection command, for every cylinder: pulse width (dead time
        // included; 0 = no injection) starting at injectionAngle_deg before
        // the cylinder's firing TDC. With remote I/O the I/O controller
        // schedules it; otherwise it is informative.
        double injection_ms = 0;
        double injectionAngle_deg = 0;

        // Injector pulses started during this step (none with remote I/O)
        struct Injection {
            int cylinder = 0;
            double pulseWidth_ms = 0;   // includes the injector dead time
        };
        int injectionCount = 0;
        Injection injections[MaxInjections];
    };
}

#endif /* ECU_IO_H */
