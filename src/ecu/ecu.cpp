#include "../../include/ecu/ecu.h"

#include "../../include/engine.h"
#include "../../include/units.h"

#include <cmath>

namespace {
    // Minimum learning weight (about 0.1 s at 10 kHz) for an RPM column to count as learned
    constexpr double MinLearnedWeight = 1000.0;

    // Below this MAP variance [kPa^2] a column has no usable slope and uses its mean
    constexpr double MinMapVariance = 4.0;

    // Fuel mass delivered by one injection of the given length
    double injectionMass(const ecu::CalibrationPage &cal, double fuelDensity, double injectionTime_ms) {
        const double flow = units::volume(cal.InjectorFlow, units::cc) / units::minute;
        const double openTime = std::fmax(injectionTime_ms - cal.InjectorDeadTime, 0.0) / 1000.0;
        return flow * fuelDensity * openTime;
    }

    double injectionTimeForMass(const ecu::CalibrationPage &cal, double fuelDensity, double mass) {
        const double flow = units::volume(cal.InjectorFlow, units::cc) / units::minute;
        return 1000.0 * mass / (flow * fuelDensity) + cal.InjectorDeadTime;
    }

}

void ecu::Ecu::ColumnFit::add(double weight, double x, double y) {
    w += weight;
    wx += weight * x;
    wy += weight * y;
    wxx += weight * x * x;
    wxy += weight * x * y;
}

bool ecu::Ecu::ColumnFit::valid() const {
    return w >= MinLearnedWeight;
}

double ecu::Ecu::ColumnFit::evaluate(double x) const {
    const double denominator = w * wxx - wx * wx;
    if (denominator <= MinMapVariance * w * w) {
        return wy / w;
    }

    const double slope = (w * wxy - wx * wy) / denominator;
    const double intercept = (wy - slope * wx) / w;
    return intercept + slope * x;
}

void ecu::Ecu::addSample(ColumnFit *fits, const float *axisX, double rpm, double map_kPa, double value) {
    // Each sample feeds the two neighbouring RPM columns, weighted by distance
    int i;
    float t;
    locateOnAxis(axisX, MapRpmPoints, (float)rpm, &i, &t);

    fits[i].add(1.0 - t, map_kPa, value);
    fits[i + 1].add(t, map_kPa, value);
}

int ecu::Ecu::fillMap(const ColumnFit *fits, const float *axisY, float values[][MapRpmPoints]) {
    int learnedColumns = 0;
    for (int i = 0; i < MapRpmPoints; ++i) {
        if (!fits[i].valid()) continue;

        ++learnedColumns;
        for (int j = 0; j < MapLoadPoints; ++j) {
            values[j][i] = static_cast<float>(fits[i].evaluate(axisY[j]));
        }
    }

    if (learnedColumns == 0) return 0;

    // Columns without data copy the nearest learned column
    for (int i = 0; i < MapRpmPoints; ++i) {
        if (fits[i].valid()) continue;

        int nearest = -1;
        for (int ii = 0; ii < MapRpmPoints; ++ii) {
            if (fits[ii].valid() && (nearest < 0 || std::abs(ii - i) < std::abs(nearest - i))) {
                nearest = ii;
            }
        }

        for (int j = 0; j < MapLoadPoints; ++j) {
            values[j][i] = values[j][nearest];
        }
    }

    return learnedColumns;
}

ecu::Ecu::Ecu() {
    m_lastTotalFuel = -1;
    m_filteredScriptInjectionTime = 0;
    m_engineRunning = false;
    m_runningTime = 0;
    m_lastMap_kPa = 0;
    m_filteredMapRate = 0;
    m_learning = false;
    resetLearning();
}

void ecu::Ecu::beginFrame() {
    m_calibration.updateSnapshot();
}

void ecu::Ecu::update(Engine *engine, double dt) {
    const CalibrationPage &cal = m_calibration.snapshot();
    IgnitionModule *ignitionModule = engine->getIgnitionModule();
    const Fuel *fuel = engine->getFuel();

    const int cylinderCount = engine->getCylinderCount();
    const int intakeCount = engine->getIntakeCount();

    const double rpm = engine->getRpm();
    const double map_kPa = units::convert(engine->getManifoldPressure(), units::kPa);
    const double injectionsPerSecond = cylinderCount * rpm / 120.0;

    m_measurements.rpm = rpm;
    m_measurements.map_kPa = map_kPa;

    if (dt > 0) {
        const double mapRate = (map_kPa - m_lastMap_kPa) / dt;
        const double alpha = dt / (dt + 0.05);
        m_filteredMapRate += alpha * (mapRate - m_filteredMapRate);
    }
    m_lastMap_kPa = map_kPa;
    if (rpm > cal.CrankingRpm) m_engineRunning = true;
    else if (rpm < 0.25 * cal.CrankingRpm) m_engineRunning = false;
    const bool cranking = !m_engineRunning;

    m_runningTime = cranking ? 0.0 : m_runningTime + dt;

    // CrankingFuelFactor = 0: start on the engine script's mixture and let the
    // fuel map take over once the engine runs
    const bool scriptStart = cranking && cal.CrankingFuelFactor <= 0.0f;
    m_measurements.fuelActive = cal.EcuFuelEnable >= 0.5f && !scriptStart;
    m_measurements.ignitionActive = cal.EcuIgnEnable >= 0.5f;

    // Ignition
    const double scriptAdvance_deg = units::convert(ignitionModule->getBaseTimingAdvance(), units::deg);
    if (m_measurements.ignitionActive) {
        m_measurements.advance_deg = lookupMap(
            cal.IgnAdvanceMap_X, MapRpmPoints,
            cal.IgnAdvanceMap_Y, MapLoadPoints,
            &cal.IgnAdvanceMap[0][0],
            (float)rpm, (float)map_kPa,
            &m_measurements.ignitionCell);
        ignitionModule->setAdvanceOverride(units::angle(m_measurements.advance_deg, units::deg));
    }
    else {
        m_measurements.advance_deg = scriptAdvance_deg;
        ignitionModule->clearAdvanceOverride();
    }

    // Fuel injected over the previous step (by the script mixture or the ECU)
    double totalFuel = 0;
    for (int i = 0; i < intakeCount; ++i) {
        totalFuel += engine->getIntake(i)->m_totalFuelInjected;
    }

    const double stepFuel = (m_lastTotalFuel >= 0 && totalFuel >= m_lastTotalFuel)
        ? totalFuel - m_lastTotalFuel
        : 0.0;
    m_lastTotalFuel = totalFuel;

    if (m_measurements.fuelActive) {
        m_measurements.injectionTime_ms = lookupMap(
            cal.FuelBaseMap_X, MapRpmPoints,
            cal.FuelBaseMap_Y, MapLoadPoints,
            &cal.FuelBaseMap[0][0],
            (float)rpm, (float)map_kPa,
            &m_measurements.fuelCell);

        // The map is calibrated with the engine running; while cranking the
        // cylinders fill much less per cycle and need a correction
        if (cranking) {
            m_measurements.injectionTime_ms *= cal.CrankingFuelFactor;
        }
        else if (m_runningTime < cal.AfterStartTime) {
            // After start: fade the cranking factor out instead of stepping to 1
            const double s = m_runningTime / cal.AfterStartTime;
            m_measurements.injectionTime_ms *= cal.CrankingFuelFactor * (1 - s) + s;
        }

        const double fuelRate_mol_s =
            injectionMass(cal, fuel->getDensity(), m_measurements.injectionTime_ms)
            * injectionsPerSecond
            / fuel->getMolecularMass();

        // One port injector per cylinder
        for (int i = 0; i < intakeCount; ++i) {
            engine->getIntake(i)->m_ecuFuelActive = true;
        }
        for (int i = 0; i < cylinderCount; ++i) {
            engine->getChamber(i)->m_ecuFuelRate = fuelRate_mol_s / cylinderCount;
        }
    }
    else {
        for (int i = 0; i < intakeCount; ++i) {
            engine->getIntake(i)->m_ecuFuelActive = false;
        }
        for (int i = 0; i < cylinderCount; ++i) {
            engine->getChamber(i)->m_ecuFuelRate = 0;
        }

        // Injection time that would deliver the script mixture's fuel
        double scriptInjectionTime_ms = 0;
        if (injectionsPerSecond > 0 && dt > 0) {
            const double massPerInjection =
                stepFuel * fuel->getMolecularMass() / dt / injectionsPerSecond;
            scriptInjectionTime_ms = injectionTimeForMass(cal, fuel->getDensity(), massPerInjection);
        }

        // The per-step value pulses with the intake flow; smooth it for display
        const double alpha = dt / (dt + 0.1);
        m_filteredScriptInjectionTime += alpha * (scriptInjectionTime_ms - m_filteredScriptInjectionTime);
        m_measurements.injectionTime_ms = m_filteredScriptInjectionTime;

        constexpr double MaxSteadyMapRate = 10.0;   // [kPa/s]
        if (m_learning
            && rpm > 300
            && ignitionModule->m_enabled
            && std::abs(m_filteredMapRate) < MaxSteadyMapRate)
        {
            learn(cal, scriptInjectionTime_ms, scriptAdvance_deg);
        }
    }
}

void ecu::Ecu::learn(const CalibrationPage &cal, double scriptInjectionTime_ms, double scriptAdvance_deg) {
    addSample(m_fuelFit, cal.FuelBaseMap_X, m_measurements.rpm, m_measurements.map_kPa, scriptInjectionTime_ms);

    if (m_measurements.ignitionActive) return;

    addSample(m_ignitionFit, cal.IgnAdvanceMap_X, m_measurements.rpm, m_measurements.map_kPa, scriptAdvance_deg);
}

void ecu::Ecu::resetLearning() {
    for (int i = 0; i < MapRpmPoints; ++i) {
        m_fuelFit[i] = ColumnFit();
        m_ignitionFit[i] = ColumnFit();
    }
}

ecu::CalibrationPage ecu::Ecu::buildLearnedPage(int *fuelColumns, int *ignitionColumns) const {
    CalibrationPage page = m_calibration.snapshot();

    *fuelColumns = fillMap(m_fuelFit, page.FuelBaseMap_Y, page.FuelBaseMap);
    *ignitionColumns = fillMap(m_ignitionFit, page.IgnAdvanceMap_Y, page.IgnAdvanceMap);

    // Keep extrapolated cells inside the parameter limits
    for (int j = 0; j < MapLoadPoints; ++j) {
        for (int i = 0; i < MapRpmPoints; ++i) {
            page.FuelBaseMap[j][i] = std::fmin(std::fmax(page.FuelBaseMap[j][i], 0.0f), 100.0f);
            page.IgnAdvanceMap[j][i] = std::fmin(std::fmax(page.IgnAdvanceMap[j][i], -30.0f), 70.0f);
        }
    }

    page.EcuFuelEnable = 1;
    page.EcuIgnEnable = 1;

    return page;
}
