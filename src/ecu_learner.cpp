#include "../include/ecu_learner.h"

#include "../include/engine.h"
#include "../include/units.h"

#include <algorithm>
#include <cmath>

namespace {
    // Minimum learning weight (simulation steps, about 0.1 s at 20 kHz) for a
    // MAP bin to be used
    constexpr double MinBinWeight = 2000.0;

    // End segments used for extrapolation span at least this much MAP [kPa]
    constexpr double MinExtrapolationSpan = 10.0;

    // Steady state: the cycle-average MAP hasn't moved since the previous cycle
    constexpr double MaxCycleMapChange = 1.0;   // [kPa]

    float parameter(const ecu::Ecu &ecu, const ecu::CalibrationPage &page, const char *name) {
        const ecu::ParameterInfo *p = ecu.layout().find(name);
        return (p != nullptr) ? page[p->offset] : 0.0f;
    }
}

void EcuLearner::ColumnFit::add(double weight, double x, double y) {
    const int bin = (int)std::floor(x / BinWidth);
    if (bin < 0 || bin >= Bins || weight <= 0) return;

    w[bin] += weight;
    wx[bin] += weight * x;
    wy[bin] += weight * y;
}

bool EcuLearner::ColumnFit::valid() const {
    for (int b = 0; b < Bins; ++b) {
        if (w[b] >= MinBinWeight) return true;
    }

    return false;
}

double EcuLearner::ColumnFit::evaluate(double x) const {
    // Learned points (bin averages), in increasing MAP
    double px[Bins], py[Bins];
    int n = 0;
    for (int b = 0; b < Bins; ++b) {
        if (w[b] < MinBinWeight) continue;
        px[n] = wx[b] / w[b];
        py[n] = wy[b] / w[b];
        ++n;
    }

    if (n == 0) return 0.0;
    if (n == 1) return py[0];

    auto line = [&](int i, int j) {
        return py[i] + (py[j] - py[i]) * (x - px[i]) / (px[j] - px[i]);
    };

    if (x <= px[0]) {
        int j = 1;
        while (j < n - 1 && px[j] - px[0] < MinExtrapolationSpan) ++j;
        return line(0, j);
    }
    if (x >= px[n - 1]) {
        int i = n - 2;
        while (i > 0 && px[n - 1] - px[i] < MinExtrapolationSpan) --i;
        return line(i, n - 1);
    }

    int k = 1;
    while (px[k] < x) ++k;
    return line(k - 1, k);
}

void EcuLearner::reset() {
    m_cycle = CycleAccumulator();
    m_fuelFit.clear();
    m_ignitionFit.clear();
    m_lastTotalFuel = -1;
}

void EcuLearner::addSample(
    std::vector<ColumnFit> &fits, const float *axisX, int sizeX,
    double rpm, double map_kPa, double value, double weight)
{
    // Each sample feeds the two neighbouring rpm columns, weighted by distance
    int i;
    float t;
    ecu::locateOnAxis(axisX, sizeX, (float)rpm, &i, &t);

    fits[i].add(weight * (1.0 - t), map_kPa, value);
    fits[i + 1].add(weight * t, map_kPa, value);
}

void EcuLearner::sample(Engine *engine, const ecu::Ecu &ecu, const ecu::EcuOutputs &outputs, double dt) {
    // Fuel delivered by the script mixture (carburettor) during this step
    double totalFuel = 0;
    for (int i = 0; i < engine->getIntakeCount(); ++i) {
        totalFuel += engine->getIntake(i)->m_totalFuelInjected;
    }
    const double stepFuel = (m_lastTotalFuel >= 0 && totalFuel >= m_lastTotalFuel)
        ? totalFuel - m_lastTotalFuel
        : 0.0;
    m_lastTotalFuel = totalFuel;

    const ecu::EcuSignals &s = ecu.signals();
    IgnitionModule *ignition = engine->getIgnitionModule();
    if (outputs.fuelControl || s.engineState != ecu::Running || !ignition->m_enabled) {
        m_cycle = CycleAccumulator();
        return;
    }

    const ecu::CalibrationPage &cal = ecu.calibration().snapshot();
    const ecu::ParameterInfo *fuelMap = ecu.layout().find("FuelBaseMap");
    const ecu::ParameterInfo *ignitionMap = ecu.layout().find("IgnAdvanceMap");
    if (fuelMap == nullptr || ignitionMap == nullptr) return;

    if ((int)m_fuelFit.size() != fuelMap->sizeX) m_fuelFit.assign(fuelMap->sizeX, ColumnFit());
    if ((int)m_ignitionFit.size() != ignitionMap->sizeX) m_ignitionFit.assign(ignitionMap->sizeX, ColumnFit());

    // Manifold pressure and fuel flow pulse within each cycle (strongly with
    // few cylinders), so learning works on whole 720 deg cycles
    const double scriptAdvance_deg = units::convert(ignition->getBaseTimingAdvance(), units::deg);
    m_cycle.angle += s.rpmInstant * 6.0 * dt;
    m_cycle.fuel += stepFuel;
    m_cycle.mapTime += s.mapInstant_kPa * dt;
    m_cycle.advanceTime += scriptAdvance_deg * dt;
    m_cycle.time += dt;
    ++m_cycle.steps;

    if (m_cycle.angle < 720.0) return;

    const double map_kPa = m_cycle.mapTime / m_cycle.time;
    const double rpm = m_cycle.angle / (6.0 * m_cycle.time);
    const double advance_deg = m_cycle.advanceTime / m_cycle.time;

    // Equivalent injection time with the calibrated injector
    const Fuel *fuel = engine->getFuel();
    const double massPerInjection = m_cycle.fuel * fuel->getMolecularMass() / engine->getCylinderCount();
    const double flow = units::volume(parameter(ecu, cal, "InjectorFlow"), units::cc) / units::minute;
    const double injection_ms =
        1000.0 * massPerInjection / (flow * fuel->getDensity()) + parameter(ecu, cal, "InjectorDeadTime");

    const bool steady =
        m_cycle.previousMap_kPa >= 0
        && std::abs(map_kPa - m_cycle.previousMap_kPa) < MaxCycleMapChange;

    if (steady) {
        const double weight = m_cycle.steps;
        addSample(m_fuelFit, &cal[fuelMap->offsetX], fuelMap->sizeX, rpm, map_kPa, injection_ms, weight);
        if (!outputs.sparkControl) {
            addSample(m_ignitionFit, &cal[ignitionMap->offsetX], ignitionMap->sizeX, rpm, map_kPa, advance_deg, weight);
        }
    }

    const double previousMap = map_kPa;
    m_cycle = CycleAccumulator();
    m_cycle.previousMap_kPa = previousMap;
}

int EcuLearner::fillMap(
    const std::vector<ColumnFit> &fits, const ecu::ParameterInfo &map, ecu::CalibrationPage *page)
{
    if ((int)fits.size() != map.sizeX) return 0;

    float *values = &(*page)[map.offset];
    const float *axisY = &(*page)[map.offsetY];

    int learnedColumns = 0;
    for (int i = 0; i < map.sizeX; ++i) {
        if (!fits[i].valid()) continue;

        ++learnedColumns;
        for (int j = 0; j < map.sizeY; ++j) {
            const float v = static_cast<float>(fits[i].evaluate(axisY[j]));
            values[j * map.sizeX + i] = std::clamp(v, map.min, map.max);
        }
    }

    if (learnedColumns == 0) return 0;

    // Columns without data copy the nearest learned column
    for (int i = 0; i < map.sizeX; ++i) {
        if (fits[i].valid()) continue;

        int nearest = -1;
        for (int ii = 0; ii < map.sizeX; ++ii) {
            if (fits[ii].valid() && (nearest < 0 || std::abs(ii - i) < std::abs(nearest - i))) {
                nearest = ii;
            }
        }

        for (int j = 0; j < map.sizeY; ++j) {
            values[j * map.sizeX + i] = values[j * map.sizeX + nearest];
        }
    }

    return learnedColumns;
}

ecu::CalibrationPage EcuLearner::buildPage(const ecu::Ecu &ecu, int *fuelColumns, int *ignitionColumns) const {
    ecu::CalibrationPage page = ecu.calibration().snapshot();
    const ecu::CalibrationLayout &layout = ecu.layout();

    *fuelColumns = *ignitionColumns = 0;
    if (const ecu::ParameterInfo *p = layout.find("FuelBaseMap")) {
        *fuelColumns = fillMap(m_fuelFit, *p, &page);
    }
    if (const ecu::ParameterInfo *p = layout.find("IgnAdvanceMap")) {
        *ignitionColumns = fillMap(m_ignitionFit, *p, &page);
    }

    if (const ecu::ParameterInfo *p = layout.find("EcuFuelEnable")) page[p->offset] = 1;
    if (const ecu::ParameterInfo *p = layout.find("EcuIgnEnable")) page[p->offset] = 1;

    return page;
}
