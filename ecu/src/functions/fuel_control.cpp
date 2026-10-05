#include "ecu/functions.h"

#include <algorithm>

namespace {
    const ecu::AxisSpec RpmAxis = { "rpm", 0, 20000, {
        500, 750, 1000, 1250, 1500, 2000, 2500, 3000,
        3500, 4000, 4500, 5000, 5500, 6000, 6500, 7000 }, "rpm" };

    // Closed throttle at high rpm pulls the manifold down to ~10 kPa
    const ecu::AxisSpec MapAxis = { "kPa", 0, 400, {
        10, 15, 20, 30, 40, 50, 60, 70, 80, 90, 100, 105 }, "map_kPa" };

    const ecu::AxisSpec CltAxis = { "degC", -50, 200, { -40, -20, 0, 20, 40, 60, 80, 100 }, "clt_C" };
    const ecu::AxisSpec IatAxis = { "degC", -50, 200, { -20, 0, 20, 40, 60, 80 }, "iat_C" };
}

void ecu::FuelControl::declare(CalibrationLayout &cal, MeasurementTable &meas, EcuSignals &s) {
    m_enable = cal.addValue("EcuFuelEnable",
        "Fuel from FuelBaseMap (0 = engine script mixture)",
        "-", 0, 1, 0);
    cal.addValue("InjectorFlow",
        "Injector static flow",
        "cc/min", 1, 20000, 440);
    m_injectorDeadTime = cal.addValue("InjectorDeadTime",
        "Injector dead time",
        "ms", 0, 10, 1.0f);
    m_baseMap = cal.addMap("FuelBaseMap",
        "Base injection time (dead time included)",
        "ms", 0, 100, RpmAxis, MapAxis, { 1.0f });
    m_crankingFactor = cal.addValue("CrankingFuelFactor",
        "Injection time multiplier while cranking (0 = start on the script mixture)",
        "-", 0, 5, 1.0f);
    m_afterStartTime = cal.addValue("AfterStartTime",
        "Time for the cranking factor to fade to 1 once the engine runs",
        "s", 0, 60, 0);
    m_cltCorrection = cal.addCurve("FuelCltCorr",
        "Warm-up fuel factor vs coolant temperature",
        "-", 0, 5, CltAxis, { 1.0f });
    m_iatCorrection = cal.addCurve("FuelIatCorr",
        "Fuel factor vs intake air temperature",
        "-", 0, 5, IatAxis, { 1.0f });

    meas.add("fuel_ecu", "-", "1 = injection controlled by the ECU", &s.fuelControl);
    meas.add("injection_ms", "ms", "Final injection time (dead time included)", &s.injection_ms);
    meas.add("injection_base_ms", "ms", "FuelBaseMap value", &s.injectionBase_ms);
    meas.add("fuel_corr_start", "-", "Cranking / after-start factor", &s.fuelCorrStart);
    meas.add("fuel_corr_clt", "-", "Coolant temperature factor", &s.fuelCorrClt);
    meas.add("fuel_corr_iat", "-", "Intake air temperature factor", &s.fuelCorrIat);
}

void ecu::FuelControl::run(const float *cal, EcuSignals &s, double dt) {
    const bool running = (s.engineState == Running);
    const double crankingFactor = m_crankingFactor(cal);

    // CrankingFuelFactor = 0: start on the engine script's mixture and let the
    // map take over once the engine runs
    s.fuelControl = (m_enable(cal) >= 0.5f && (running || crankingFactor > 0)) ? 1 : 0;

    s.injectionBase_ms = m_baseMap.lookup(cal, (float)s.rpm, (float)s.map_kPa, &s.fuelCell);

    // The map is calibrated with the engine running; cranking fills the
    // cylinders much less per cycle. After start the factor fades out.
    const double afterStart = m_afterStartTime(cal);
    if (!running) {
        s.fuelCorrStart = crankingFactor;
    }
    else if (s.runningTime_s < afterStart) {
        const double t = s.runningTime_s / afterStart;
        s.fuelCorrStart = crankingFactor * (1 - t) + t;
    }
    else {
        s.fuelCorrStart = 1;
    }

    s.fuelCorrClt = m_cltCorrection.lookup(cal, (float)s.clt_C);
    s.fuelCorrIat = m_iatCorrection.lookup(cal, (float)s.iat_C);

    // Corrections scale the injector open time; the dead time is added back
    const double deadTime = m_injectorDeadTime(cal);
    const double openTime =
        std::max(s.injectionBase_ms - deadTime, 0.0)
        * s.fuelCorrStart * s.fuelCorrClt * s.fuelCorrIat * s.fuelCorrLambda;

    s.injection_ms = (s.fuelCut >= 0.5 || openTime <= 0) ? 0.0 : openTime + deadTime;
}
