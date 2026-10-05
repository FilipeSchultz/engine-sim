#include "../include/indicating_system.h"

#include "../include/engine.h"
#include "../include/constants.h"
#include "../include/units.h"

#include <cmath>

namespace {
    // First angle at which values (normalized 0..1) reach the given fraction,
    // interpolated between samples; searched from index 'from'
    bool crossing(
        const std::vector<double> &angles,
        const std::vector<double> &values,
        size_t from,
        double fraction,
        double *angle)
    {
        for (size_t k = from + 1; k < values.size(); ++k) {
            if (values[k] < fraction) continue;

            const double v0 = values[k - 1], v1 = values[k];
            const double t = (v1 > v0) ? (fraction - v0) / (v1 - v0) : 0.0;
            *angle = angles[k - 1] + t * (angles[k] - angles[k - 1]);
            return true;
        }

        return false;
    }
}

IndicatingSystem::IndicatingSystem() {
    m_angle = 0;
    m_time = 0;
    m_analysis = false;
    m_rawCapture = false;
    m_fuelEnergyDensity = 0;
    m_airTemperature = units::celcius(25.0);
}

void IndicatingSystem::reset() {
    m_cylinders.clear();
    m_results.clear();
    m_raw.clear();
    m_info = EngineInfo();
    m_angle = 0;
    m_time = 0;
}

void IndicatingSystem::updateEngineInfo(Engine *engine) {
    const int cylinderCount = engine->getCylinderCount();

    // Geometry doesn't change while the engine is loaded (reset() on reload)
    if (m_info.cylinders != cylinderCount) {
        m_info.name = engine->getName();
        m_info.cylinders = cylinderCount;
        m_info.bore_mm = units::convert(
            engine->getChamber(0)->getCylinderHead()->getCylinderBank()->getBore(), units::mm);
        m_info.stroke_mm = units::convert(2 * engine->getOutputCrankshaft()->getThrow(), units::mm);
        m_info.rodLength_mm = units::convert(engine->getConnectingRod(0)->getLength(), units::mm);
    }

    const Cylinder &c = m_cylinders[0];
    m_info.compressionRatio = (c.minVolume > 0 && c.maxVolume > c.minVolume)
        ? c.maxVolume / c.minVolume
        : 0.0;

    if ((int)m_info.tdcEncoderAngle_deg.size() != cylinderCount) {
        m_info.tdcEncoderAngle_deg.assign(cylinderCount, -1.0);
    }
}

void IndicatingSystem::sample(Engine *engine, double dt) {
    const int cylinderCount = engine->getCylinderCount();
    if ((int)m_cylinders.size() != cylinderCount) {
        m_cylinders.assign(cylinderCount, Cylinder());
    }

    // Engines run with negative angular velocity; count forward rotation positive
    const double omega = -engine->getOutputCrankshaft()->m_body.v_theta;
    m_angle += units::convert(omega * dt, units::deg);
    m_time += dt;

    // Crank angle encoder: position within the 720 deg cycle
    const double encoder = units::convert(engine->getOutputCrankshaft()->getCycleAngle(), units::deg);
    if (m_rawCapture) {
        m_raw.push_back(static_cast<float>(encoder));
    }

    IgnitionModule *ignitionModule = engine->getIgnitionModule();
    for (int i = 0; i < cylinderCount; ++i) {
        CombustionChamber *chamber = engine->getChamber(i);
        Cylinder &cylinder = m_cylinders[i];

        Sample s;
        s.angle = m_angle;
        s.encoder = encoder;
        s.time = m_time;
        s.pressure = chamber->m_system.pressure();
        s.volume = chamber->getVolume();
        s.burnedFuel = chamber->m_nBurntFuel;
        s.intakeOpen = chamber->isIntakeValveOpen();
        s.valvesClosed = !s.intakeOpen && !chamber->isExhaustValveOpen();
        s.intakeFlow = chamber->getLastTimestepIntakeFlow();
        s.moles = chamber->m_system.n();
        s.exhaustFlow = chamber->getLastTimestepExhaustFlow();
        s.gasTemp = chamber->m_system.temperature();
        s.portTemp = chamber->m_exhaustRunnerAndPrimary.temperature();

        if (m_rawCapture) {
            m_raw.push_back(static_cast<float>(units::convert(s.pressure, units::bar)));
        }

        cylinder.minVolume = std::fmin(cylinder.minVolume, s.volume);
        cylinder.maxVolume = std::fmax(cylinder.maxVolume, s.volume);

        if (ignitionModule->getIgnitionEvent(i)) {
            cylinder.lastSparkAngle = m_angle;
        }

        // TDC = volume minimum; it is the firing TDC when both valves are closed
        // (at the gas exchange TDC they overlap)
        const size_t n = cylinder.samples.size();
        if (n >= 2 && omega > 1.0) {
            const Sample &a = cylinder.samples[n - 2];
            const Sample &b = cylinder.samples[n - 1];
            if (b.volume < a.volume && s.volume >= b.volume && b.valvesClosed) {
                if (m_analysis) cylinder.pendingTdc.push_back(b.angle);

                // The volume is nearly flat around TDC: locate the minimum with
                // a parabola through the three samples, then average over cycles
                const double curvature = a.volume - 2 * b.volume + s.volume;
                const double shift = (curvature > 0)
                    ? 0.5 * (a.volume - s.volume) / curvature
                    : 0.0;
                double tdc = b.encoder + shift * (s.angle - a.angle) / 2.0;
                tdc = std::fmod(tdc + 720.0, 720.0);

                if (i < (int)m_info.tdcEncoderAngle_deg.size()) {
                    double &known = m_info.tdcEncoderAngle_deg[i];
                    if (known < 0) {
                        known = tdc;
                    }
                    else {
                        // Average on the circle (TDC may sit next to the 0/720 wrap)
                        double delta = std::fmod(tdc - known + 1080.0, 720.0) - 360.0;
                        known = std::fmod(known + 0.1 * delta + 720.0, 720.0);
                    }
                }
            }
        }

        cylinder.samples.push_back(s);

        // A cycle is analysed once the full 720 deg around its firing TDC is recorded
        while (!cylinder.pendingTdc.empty() && m_angle >= cylinder.pendingTdc.front() + 360.0) {
            analyze(i, cylinder.pendingTdc.front());
            cylinder.pendingTdc.pop_front();
        }

        const double keepFrom = cylinder.pendingTdc.empty()
            ? m_angle - 800.0
            : cylinder.pendingTdc.front() - 400.0;
        while (cylinder.samples.size() > 2 && cylinder.samples.front().angle < keepFrom) {
            cylinder.samples.pop_front();
        }
    }

    m_fuelEnergyDensity = engine->getFuel()->getEnergyDensity();
    if (engine->getIntakeCount() > 0) {
        m_airTemperature = engine->getIntake(0)->getAirTemperature();
    }
    updateEngineInfo(engine);
}

bool IndicatingSystem::popResult(CycleResult *result) {
    if (m_results.empty()) return false;

    *result = m_results.front();
    m_results.pop_front();
    return true;
}

void IndicatingSystem::analyze(int cylinderIndex, double tdcAngle) {
    Cylinder &cylinder = m_cylinders[cylinderIndex];

    std::vector<const Sample *> cycle;
    for (const Sample &s : cylinder.samples) {
        if (s.angle >= tdcAngle - 360.0 && s.angle <= tdcAngle + 360.0) {
            cycle.push_back(&s);
        }
    }

    // Needs the whole cycle (not the case right after start-up)
    if (cycle.size() < 20
        || cycle.front()->angle > tdcAngle - 350.0
        || cycle.back()->angle < tdcAngle + 350.0)
    {
        return;
    }

    CycleResult result;
    result.cylinder = cylinderIndex + 1;
    result.cycle = ++cylinder.cycles;
    result.time = cycle.back()->time;

    const double revolutions = (cycle.back()->angle - cycle.front()->angle) / 360.0;
    const double duration = cycle.back()->time - cycle.front()->time;
    result.rpm = (duration > 0) ? revolutions / (duration / 60.0) : 0.0;

    const double sparkAdvance = tdcAngle - cylinder.lastSparkAngle;
    result.sparkValid = sparkAdvance > -40.0 && sparkAdvance < 120.0;
    result.sparkAdvance_deg = result.sparkValid ? sparkAdvance : 0.0;

    double vMin = cycle.front()->volume, vMax = cycle.front()->volume;
    for (const Sample *s : cycle) {
        vMin = std::fmin(vMin, s->volume);
        vMax = std::fmax(vMax, s->volume);
    }
    const double displacement = vMax - vMin;

    // Work (trapezoidal p dV), peak pressure and apparent heat release
    constexpr double g = HeatCapacityRatio;
    double netWork = 0, grossWork = 0;
    result.pmax_bar = 0;

    std::vector<double> angles, heat, burned;
    double q = 0;
    for (size_t k = 1; k < cycle.size(); ++k) {
        const Sample &s0 = *cycle[k - 1];
        const Sample &s1 = *cycle[k];
        const double p = 0.5 * (s0.pressure + s1.pressure);
        const double v = 0.5 * (s0.volume + s1.volume);
        const double dV = s1.volume - s0.volume;
        const double dp = s1.pressure - s0.pressure;

        netWork += p * dV;

        const double a = s1.angle - tdcAngle;
        if (a < -180.0 || a > 180.0) continue;

        grossWork += p * dV;
        q += g / (g - 1) * p * dV + 1 / (g - 1) * v * dp;

        angles.push_back(a);
        heat.push_back(q);
        burned.push_back(s1.burnedFuel);

        if (s1.pressure > units::pressure(result.pmax_bar, units::bar)) {
            result.pmax_bar = units::convert(s1.pressure, units::bar);
            result.pmaxAngle_deg = a;
        }
    }

    if (displacement > 0) {
        result.imepNet_bar = units::convert(netWork / displacement, units::bar);
        result.imepGross_bar = units::convert(grossWork / displacement, units::bar);
    }

    // Mass fraction burned: heat release from its minimum (end of compression
    // losses) to its maximum (end of combustion)
    size_t kMax = 0;
    for (size_t k = 1; k < heat.size(); ++k) {
        if (heat[k] > heat[kMax]) kMax = k;
    }
    size_t kStart = 0;
    for (size_t k = 1; k <= kMax; ++k) {
        if (heat[k] < heat[kStart]) kStart = k;
    }

    const double fuelBurned = burned.empty() ? 0.0 : burned.back() - burned.front();
    const double heatReleased = heat.empty() ? 0.0 : heat[kMax] - heat[kStart];
    result.misfire = fuelBurned <= 1e-9 || heatReleased <= 0;

    if (!result.misfire) {
        std::vector<double> mfb(heat.size()), reference(burned.size());
        for (size_t k = 0; k < heat.size(); ++k) {
            mfb[k] = (k < kStart) ? 0.0 : (heat[k] - heat[kStart]) / heatReleased;
            reference[k] = (burned[k] - burned.front()) / fuelBurned;
        }

        crossing(angles, mfb, kStart, 0.1, &result.ca10_deg);
        crossing(angles, mfb, kStart, 0.5, &result.ca50_deg);
        crossing(angles, mfb, kStart, 0.9, &result.ca90_deg);
        crossing(angles, reference, 0, 0.5, &result.ca50Reference_deg);
    }

    // Gas exchange: net flow through the intake valve from the gas exchange TDC
    // to the firing TDC (backflow counts negative) and the cylinder content
    // when the intake valve closes. Masses use the molar mass of air.
    double freshMoles = 0, trappedMoles = 0;
    for (size_t k = 0; k < cycle.size(); ++k) {
        const Sample &s = *cycle[k];
        if (s.angle <= tdcAngle - 360.0 || s.angle > tdcAngle) continue;

        freshMoles += s.intakeFlow;
        if (k > 0 && cycle[k - 1]->intakeOpen && !s.intakeOpen) {
            trappedMoles = s.moles;
        }
    }

    const double freshMass = freshMoles * units::AirMolecularMass;
    const double trappedMass = trappedMoles * units::AirMolecularMass;
    const double ambientDensity =
        units::pressure(1.0, units::atm) * units::AirMolecularMass
        / (constants::R * m_airTemperature);

    result.freshCharge_mg = units::convert(freshMass, units::g) * 1000.0;
    result.trappedMass_mg = units::convert(trappedMass, units::g) * 1000.0;
    result.residual_percent = (trappedMass > 0)
        ? 100.0 * (trappedMass - freshMass) / trappedMass
        : 0.0;
    result.volumetricEfficiency_percent = (displacement > 0)
        ? 100.0 * freshMass / (ambientDensity * displacement)
        : 0.0;
    result.fuelBurned_mg = units::convert(fuelBurned, units::g) * 1000.0;

    const double fuelEnergy = fuelBurned * m_fuelEnergyDensity;
    result.indicatedEfficiency_percent = (fuelEnergy > 0)
        ? 100.0 * grossWork / fuelEnergy
        : 0.0;

    // Exhaust gas temperature over the cycle
    double outflow = 0, outflowTemp = 0, portTime = 0, portTemp = 0;
    for (size_t k = 1; k < cycle.size(); ++k) {
        const Sample &s = *cycle[k];
        const double dt = s.time - cycle[k - 1]->time;
        if (s.exhaustFlow > 0) {
            outflow += s.exhaustFlow;
            outflowTemp += s.exhaustFlow * s.gasTemp;
        }
        portTime += dt;
        portTemp += dt * s.portTemp;
    }
    const double zeroC = units::celcius(0.0);
    result.exhaustTempFlowWeighted_C = (outflow > 0) ? outflowTemp / outflow - zeroC : 0.0;
    result.exhaustPortTemp_C = (portTime > 0) ? portTemp / portTime - zeroC : 0.0;

    m_results.push_back(result);
}
