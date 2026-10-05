#ifndef ECU_SIGNALS_H
#define ECU_SIGNALS_H

#include "calibration.h"

namespace ecu {
    enum EngineState {
        Stopped = 0,
        Cranking = 1,
        Running = 2
    };

    // Internal signals shared by the ECU functions (the ECU's RAM). Each
    // function writes its own outputs and reads what the others produced;
    // flags are 0 / 1 so every signal can be logged as a measurement.
    struct EcuSignals {
        // --- Coding (EcuConfig)
        double cylinders = 1;
        double remoteIo = 0;                // inputs averaged by the I/O controller

        // --- Calibration
        double calSaveCount = 0;            // calibration saves requested by the tool

        // --- InputProcessing
        double time_s = 0;
        double cycleAngle_deg = 0;
        double rpmInstant = 0;
        double mapInstant_kPa = 0;
        double rpm = 0;                     // averaged over the last firing interval
        double map_kPa = 0;                 // averaged over the last firing interval
        double iat_C = 25;
        double clt_C = 90;
        double tps_percent = 0;
        double lambdaSensor = 0;            // 0 = sensor not ready
        double battery_V = 14;

        // --- EngineState
        double engineState = Stopped;
        double runningTime_s = 0;           // since leaving cranking

        // --- RevLimiter
        double fuelCut = 0;

        // --- LambdaControl
        double lambdaTarget = 1;
        double lambdaControlActive = 0;
        double fuelCorrLambda = 1;

        // --- FuelControl
        double fuelControl = 0;
        double injectionBase_ms = 0;
        double fuelCorrStart = 1;
        double fuelCorrClt = 1;
        double fuelCorrIat = 1;
        double injection_ms = 0;            // final pulse width, dead time included
        CellPosition fuelCell;

        // --- IgnitionControl
        double sparkControl = 0;
        double advanceBase_deg = 0;
        double advanceCorrClt_deg = 0;
        double advanceCorrIat_deg = 0;
        double advance_deg = 0;
        CellPosition ignitionCell;
    };
}

#endif /* ECU_SIGNALS_H */
