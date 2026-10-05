#include "../include/ecu_hardware.h"

#include "../include/ecu_learner.h"
#include "../include/engine.h"
#include "../include/units.h"
#include "../include/constants.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace {
    // External ECU (I/O controller side of ecu/link.h)
    constexpr double SamplePeriod_s = 0.010;
    constexpr double PollPeriod_s = 0.001;          // commands read between samples
    constexpr double CommandTimeout_s = 0.1;        // fail-safe without commands
    constexpr int LockstepTimeout_ms = 50;          // wait for the answer to a sample
    constexpr int MaxUnanswered = 3;                // lockstep stops waiting after these

    // No crank signal, no injection: below this speed the angle only jitters
    constexpr double MinRpm = 10.0;
}

bool EcuHardware::openLink(const LinkSettings &settings, std::string *error) {
    m_linkSettings = settings;
    if (!m_link.open("0.0.0.0", 0, error)) return false;
    if (!m_link.setPeer(settings.address, settings.port, error)) {
        m_link.close();
        return false;
    }

    m_attached = nullptr;
    return true;
}

EcuHardware::LinkState EcuHardware::linkState() const {
    if (!m_link.isOpen()) return LinkState::Off;
    if (!m_commandValid) return LinkState::Waiting;
    if (m_failsafe) return LinkState::Lost;
    return LinkState::Connected;
}

double EcuHardware::averagedMap_kPa() const {
    if (m_link.isOpen()) return m_avgMap_kPa;
    return (m_ecu != nullptr) ? m_ecu->signals().map_kPa : 0.0;
}

void EcuHardware::attach(Engine *engine) {
    m_attached = engine;
    m_time = 0;
    m_lambda = 0;
    m_iatValid = false;
    m_outputs = ecu::EcuOutputs();

    m_commandValid = false;
    m_commandAge = 0;
    m_unanswered = 0;
    m_failsafe = true;
    m_sampleTimer = 0;
    m_pollTimer = 0;
    m_cycles = 0;
    m_avgAngle = m_avgTime = m_avgMapTime = 0;
    m_avgValid = false;
    m_angleValid = false;

    if (engine == nullptr) return;

    // Variant coding: cylinder count and firing TDCs on the cam cycle (the
    // spark plugs fire at these angles minus the advance)
    IgnitionModule *ignition = engine->getIgnitionModule();
    m_config = ecu::EcuConfig();
    m_config.cylinders = engine->getCylinderCount();
    for (int i = 0; i < m_config.cylinders; ++i) {
        m_config.firingTdc_deg.push_back(units::convert(ignition->getFiringAngle(i), units::deg));
    }

    if (m_ecu != nullptr && !m_link.isOpen()) {
        m_ecu->configure(m_config);
        if (m_learner != nullptr) m_learner->reset();
    }
}

void EcuHardware::step(Engine *engine, double dt) {
    if (m_ecu == nullptr && !m_link.isOpen()) return;
    if (engine != m_attached) attach(engine);

    readSensors(engine, dt);
    if (m_link.isOpen()) {
        stepIoController(dt);
    }
    else {
        m_ecu->step(m_inputs, dt, &m_outputs);
    }
    driveActuators(engine);

    if (m_learner != nullptr && !m_link.isOpen()) {
        m_learner->sample(engine, *m_ecu, m_outputs, dt);
    }
}

void EcuHardware::readSensors(Engine *engine, double dt) {
    m_time += dt;
    m_inputs.time_s = m_time;

    // Crank and cam sensors
    m_inputs.cycleAngle_deg =
        units::convert(engine->getOutputCrankshaft()->getCycleAngle(), units::deg);
    m_inputs.rpm = engine->getRpm();

    m_inputs.map_kPa = units::convert(engine->getManifoldPressure(), units::kPa);

    // Intake air thermistor in the plenum (slow)
    if (engine->getIntakeCount() > 0) {
        const double plenum_C =
            (engine->getIntake(0)->m_system.temperature() - units::celcius(0.0)) / units::C;
        if (!m_iatValid) {
            m_iat_C = plenum_C;
            m_iatValid = true;
        }
        m_iat_C += (plenum_C - m_iat_C) * dt / (m_iatTimeConstant_s + dt);
    }
    m_inputs.iat_C = m_iat_C;

    m_inputs.clt_C = m_coolantTemperature_C;
    m_inputs.tps_percent = 100.0 * engine->getThrottlePlateAngle() / (constants::pi / 2);

    // Wideband lambda: not ready until the first combustion
    const double lambda = engine->getCylinderLambda();
    if (lambda > 0) {
        m_lambda = (m_lambda > 0)
            ? m_lambda + (lambda - m_lambda) * dt / (m_lambdaTimeConstant_s + dt)
            : lambda;
    }
    m_inputs.lambda = m_lambda;

    m_inputs.battery_V = 14.0;
}

void EcuHardware::stepIoController(double dt) {
    averageInputs(dt);

    m_sampleTimer += dt;
    if (m_sampleTimer >= SamplePeriod_s - 1e-9) {
        // On the 10 ms grid; after a long step, the next sample a period later
        m_sampleTimer = std::fmod(m_sampleTimer, SamplePeriod_s);

        ++m_counter;
        sendSample();
        ++m_unanswered;

        // Lockstep: wait for this sample's command, unless the ECU stopped
        // answering (then the simulation runs on, in fail-safe)
        const bool wait = m_linkSettings.lockstep && m_commandValid && m_unanswered <= MaxUnanswered;
        receiveCommands(wait ? LockstepTimeout_ms : 0, wait);
        m_pollTimer = 0;
    }
    else {
        m_pollTimer += dt;
        if (m_pollTimer >= PollPeriod_s) {
            m_pollTimer = 0;
            receiveCommands(0, false);
        }
    }

    m_commandAge += dt;
    m_failsafe = !m_commandValid || m_commandAge > CommandTimeout_s;

    // Fail-safe: the ECU's outputs are cut (the engine script's mixture and
    // timing take over)
    const bool fuel = !m_failsafe && m_command.fuelControl;
    const bool spark = !m_failsafe && m_command.sparkControl;
    m_outputs.fuelControl = fuel;
    m_outputs.sparkControl = spark;
    m_outputs.sparkAdvance_deg = spark ? m_command.sparkAdvance_deg : 0;
    m_outputs.injection_ms = fuel ? m_command.injection_ms : 0;
    m_outputs.injectionAngle_deg = m_command.injectionAngle_deg;
    scheduleInjections(m_outputs.injection_ms, m_outputs.injectionAngle_deg);
}

void EcuHardware::averageInputs(double dt) {
    // rpm and MAP averaged over the last firing interval: removes the intake
    // pulsation, synchronously with the crank (as a real I/O controller does)
    const double rpm = m_inputs.rpm;
    if (rpm < 30.0) {
        m_avgAngle = m_avgTime = m_avgMapTime = 0;
        m_avgValid = false;
    }
    else {
        m_avgAngle += rpm * 6.0 * dt;
        m_avgMapTime += m_inputs.map_kPa * dt;
        m_avgTime += dt;

        const double interval = 720.0 / std::max(m_config.cylinders, 1);
        if (m_avgAngle >= interval) {
            m_avgMap_kPa = m_avgMapTime / m_avgTime;
            m_avgRpm = m_avgAngle / (6.0 * m_avgTime);
            m_avgValid = true;
            m_avgAngle = m_avgTime = m_avgMapTime = 0;
        }
    }

    if (!m_avgValid) {
        m_avgRpm = rpm;
        m_avgMap_kPa = m_inputs.map_kPa;
    }
}

void EcuHardware::sendSample() {
    ecu::link::Sample sample;
    sample.counter = m_counter;
    sample.synchronized = std::fabs(m_inputs.rpm) >= MinRpm;
    sample.failsafe = m_failsafe;
    sample.time_us = (uint32_t)(uint64_t)std::llround(m_time * 1e6);
    sample.cycles = m_cycles;

    sample.rpm = m_avgRpm;
    sample.map_kPa = m_avgMap_kPa;
    sample.tps_percent = m_inputs.tps_percent;
    sample.iat_C = m_inputs.iat_C;
    sample.clt_C = m_inputs.clt_C;
    sample.lambda = m_inputs.lambda;
    sample.battery_V = m_inputs.battery_V;

    ecu::link::Frame frames[3];
    ecu::link::encodeSample(sample, frames);
    m_link.send(frames, 3);
}

void EcuHardware::receiveCommands(int timeout_ms, bool untilAnswer) {
    using Clock = std::chrono::steady_clock;
    const Clock::time_point deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);

    ecu::link::Frame frames[ecu::link::MaxFramesPerDatagram];
    bool answered = false;
    while (true) {
        int wait_ms = 0;
        if (untilAnswer && !answered) {
            wait_ms = (int)std::max<long long>(0,
                std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now()).count());
        }

        const int count = m_link.receive(frames, ecu::link::MaxFramesPerDatagram, wait_ms);
        if (count <= 0) {
            if (count == 0 && untilAnswer && !answered && Clock::now() < deadline) continue;
            break;
        }

        for (int i = 0; i < count; ++i) {
            ecu::link::Command command;
            if (!ecu::link::decodeCommand(frames[i], &command)) continue;

            m_command = command;
            m_commandValid = true;
            m_commandAge = 0;
            if (command.counter == m_counter) {
                m_unanswered = 0;
                answered = true;
            }
        }
    }
}

void EcuHardware::scheduleInjections(double injection_ms, double injectionAngle_deg) {
    m_outputs.injectionCount = 0;

    const double rpm = m_inputs.rpm;
    double a0 = m_lastCycleAngle;
    double a1 = m_inputs.cycleAngle_deg;
    m_lastCycleAngle = a1;

    const bool wasValid = m_angleValid;
    m_angleValid = true;
    if (!wasValid || rpm < MinRpm) return;

    if (a1 < a0) {
        // Wrapped through 720 -> 0; a small step back is the crank rocking
        if (a0 - a1 < 360.0) return;
        a1 += 720.0;
        ++m_cycles;
    }

    if (injection_ms <= 0) return;

    // One pulse per cylinder and cycle, when the crank passes its injection angle
    const int cylinders = std::min((int)m_config.firingTdc_deg.size(), ecu::EcuOutputs::MaxInjections);
    for (int i = 0; i < cylinders; ++i) {
        const double target = std::fmod(m_config.firingTdc_deg[i] - injectionAngle_deg + 1440.0, 720.0);
        const bool crossed =
            (target > a0 && target <= a1)
            || (target + 720.0 > a0 && target + 720.0 <= a1);

        if (crossed && m_outputs.injectionCount < ecu::EcuOutputs::MaxInjections) {
            ecu::EcuOutputs::Injection &injection = m_outputs.injections[m_outputs.injectionCount++];
            injection.cylinder = i;
            injection.pulseWidth_ms = injection_ms;
        }
    }
}

void EcuHardware::driveActuators(Engine *engine) {
    // Ignition: the ECU's advance replaces the engine script's timing curve
    IgnitionModule *ignition = engine->getIgnitionModule();
    if (m_outputs.sparkControl) {
        ignition->setAdvanceOverride(units::angle(m_outputs.sparkAdvance_deg, units::deg));
    }
    else {
        ignition->clearAdvanceOverride();
    }

    // Port injectors: with the ECU in control only air comes through the
    // throttle (the carburettor circuits are off)
    for (int i = 0; i < engine->getIntakeCount(); ++i) {
        engine->getIntake(i)->m_ecuFuelActive = m_outputs.fuelControl;
    }
    for (int i = 0; i < engine->getCylinderCount(); ++i) {
        engine->getChamber(i)->setEcuFuelControl(m_outputs.fuelControl);
    }

    if (m_outputs.injectionCount == 0) return;

    // The engine's injectors (engine script: injector_flow, injector_dead_time)
    const double flow =
        units::volume(engine->getInjectorFlow_ccmin(), units::cc) / units::minute;
    const double deadTime_ms = engine->getInjectorDeadTime_ms();
    const Fuel *fuel = engine->getFuel();

    for (int k = 0; k < m_outputs.injectionCount; ++k) {
        const ecu::EcuOutputs::Injection &injection = m_outputs.injections[k];
        if (injection.cylinder < 0 || injection.cylinder >= engine->getCylinderCount()) continue;

        const double openTime = std::fmax(injection.pulseWidth_ms - deadTime_ms, 0.0) / 1000.0;
        const double mass = flow * fuel->getDensity() * openTime;
        engine->getChamber(injection.cylinder)->injectFuel(mass / fuel->getMolecularMass());
    }
}
