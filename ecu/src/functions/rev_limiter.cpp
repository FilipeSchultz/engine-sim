#include "ecu/functions.h"

void ecu::RevLimiter::declare(CalibrationLayout &cal, MeasurementTable &meas, EcuSignals &s) {
    m_limit = cal.addValue("RevLimitRpm",
        "Fuel cut above this engine speed (0 = off)",
        "rpm", 0, 20000, 0);
    m_hysteresis = cal.addValue("RevLimitHysteresis",
        "Fuel comes back below RevLimitRpm minus this",
        "rpm", 0, 2000, 100);

    meas.add("fuel_cut", "-", "1 = rev limiter cutting fuel", &s.fuelCut);
}

void ecu::RevLimiter::run(const float *cal, EcuSignals &s, double dt) {
    const double limit = m_limit(cal);
    if (limit <= 0) {
        s.fuelCut = 0;
    }
    else if (s.rpm > limit) {
        s.fuelCut = 1;
    }
    else if (s.rpm < limit - m_hysteresis(cal)) {
        s.fuelCut = 0;
    }
}
