#include "ecu/functions.h"

void ecu::EngineStateControl::declare(CalibrationLayout &cal, MeasurementTable &meas, EcuSignals &s) {
    m_crankingRpm = cal.addValue("CrankingRpm",
        "Engine speed below which the engine is considered cranking",
        "rpm", 0, 2000, 400);

    meas.add("engine_state", "-", "0 = stopped, 1 = cranking, 2 = running", &s.engineState);
    meas.add("running_time_s", "s", "Time since the engine left cranking", &s.runningTime_s);
}

void ecu::EngineStateControl::run(const float *cal, EcuSignals &s, double dt) {
    const double crankingRpm = m_crankingRpm(cal);

    // Hysteresis: back to cranking only when the engine nearly stops, so the
    // state doesn't toggle at the threshold while the engine catches
    if (s.rpm > crankingRpm) {
        s.engineState = Running;
    }
    else if (s.engineState == Running && s.rpm >= 0.25 * crankingRpm) {
        // still running
    }
    else {
        s.engineState = (s.rpm > 30.0) ? Cranking : Stopped;
    }

    s.runningTime_s = (s.engineState == Running) ? s.runningTime_s + dt : 0.0;
}
