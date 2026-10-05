#include "ecu/functions.h"

namespace {
    const ecu::AxisSpec RpmAxis = { "rpm", 0, 20000, {
        500, 750, 1000, 1250, 1500, 2000, 2500, 3000,
        3500, 4000, 4500, 5000, 5500, 6000, 6500, 7000 }, "rpm" };

    const ecu::AxisSpec MapAxis = { "kPa", 0, 400, {
        10, 15, 20, 30, 40, 50, 60, 70, 80, 90, 100, 105 }, "map_kPa" };

    const ecu::AxisSpec CltAxis = { "degC", -50, 200, { -40, -20, 0, 20, 40, 60, 80, 100 }, "clt_C" };
    const ecu::AxisSpec IatAxis = { "degC", -50, 200, { -20, 0, 20, 40, 60, 80 }, "iat_C" };
}

void ecu::IgnitionControl::declare(CalibrationLayout &cal, MeasurementTable &meas, EcuSignals &s) {
    m_enable = cal.addValue("EcuIgnEnable",
        "Spark advance from IgnAdvanceMap (0 = engine script curve)",
        "-", 0, 1, 0);
    m_baseMap = cal.addMap("IgnAdvanceMap",
        "Base spark advance",
        "deg", -30, 70, RpmAxis, MapAxis, { 10.0f });
    m_cltCorrection = cal.addCurve("IgnCltCorr",
        "Spark advance added vs coolant temperature",
        "deg", -30, 30, CltAxis, { 0.0f });
    m_iatCorrection = cal.addCurve("IgnIatCorr",
        "Spark advance added vs intake air temperature",
        "deg", -30, 30, IatAxis, { 0.0f });

    meas.add("ignition_ecu", "-", "1 = spark advance controlled by the ECU", &s.sparkControl);
    meas.add("advance_deg", "deg", "Final spark advance (before TDC)", &s.advance_deg);
    meas.add("advance_base_deg", "deg", "IgnAdvanceMap value", &s.advanceBase_deg);
    meas.add("advance_corr_clt_deg", "deg", "Coolant temperature correction", &s.advanceCorrClt_deg);
    meas.add("advance_corr_iat_deg", "deg", "Intake air temperature correction", &s.advanceCorrIat_deg);
}

void ecu::IgnitionControl::run(const float *cal, EcuSignals &s, double dt) {
    s.sparkControl = (m_enable(cal) >= 0.5f) ? 1 : 0;

    s.advanceBase_deg = m_baseMap.lookup(cal, (float)s.rpm, (float)s.map_kPa, &s.ignitionCell);
    s.advanceCorrClt_deg = m_cltCorrection.lookup(cal, (float)s.clt_C);
    s.advanceCorrIat_deg = m_iatCorrection.lookup(cal, (float)s.iat_C);

    s.advance_deg = s.advanceBase_deg + s.advanceCorrClt_deg + s.advanceCorrIat_deg;
}
