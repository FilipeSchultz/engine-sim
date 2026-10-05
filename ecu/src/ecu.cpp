#include "ecu/ecu.h"

#include "ecu/functions.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <stdexcept>

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#endif

namespace {
    // The ECU's RAM and calibration memory: static, so their addresses are
    // fixed inside the executable
    ecu::EcuSignals EcuRam;
    float CalibrationMemory[ecu::CalibrationCapacity];

    constexpr const char *SoftwareName = "ESIM_ECU";

    // FNV-1a
    void hash(uint32_t *h, const std::string &text) {
        for (const char c : text) {
            *h ^= static_cast<uint8_t>(c);
            *h *= 16777619u;
        }
        *h ^= 0xff;
        *h *= 16777619u;
    }
}

ecu::Ecu::Ecu() : m_signals(EcuRam) {
    m_signals = EcuSignals();

    // Execution order within each task
    m_functions.push_back(std::make_unique<InputProcessing>());
    m_functions.push_back(std::make_unique<EngineStateControl>());
    m_functions.push_back(std::make_unique<RevLimiter>());
    m_functions.push_back(std::make_unique<LambdaControl>());
    m_functions.push_back(std::make_unique<FuelControl>());
    m_functions.push_back(std::make_unique<IgnitionControl>());

    for (const auto &function : m_functions) {
        m_layout.setFunction(function->name());
        m_measurements.setFunction(function->name());
        function->declare(m_layout, m_measurements, m_signals);
    }

    m_layout.setFunction("Calibration");
    m_measurements.setFunction("Calibration");
    m_calSaveRequest = m_layout.addValue("CalSaveRequest",
        "Set from 0 to 1 to save the calibration to the DCM file (set back to 0 for the next save)",
        "-", 0, 1, 0);
    m_measurements.add("cal_save_count", "-", "Calibration saves requested", &m_signals.calSaveCount);

    m_layout.setFunction("OutputStage");
    m_injectionAngle = m_layout.addValue("InjectionAngle",
        "Start of injection, degrees before the cylinder's firing TDC "
        "(port injection: the fuel waits in the port until the intake valve opens)",
        "deg", 0, 720, 400);

    checkLayout();
    m_calibration.initialize(CalibrationMemory, m_layout.defaults());
    m_epk = fingerprint();
}

void ecu::Ecu::checkLayout() const {
    // Declaration mistakes are programming errors: stop at start-up
    if (m_layout.defaults().size() > CalibrationCapacity) {
        throw std::runtime_error("ECU: calibration needs " + std::to_string(m_layout.defaults().size())
            + " floats, CalibrationCapacity is " + std::to_string(CalibrationCapacity));
    }

    for (const ParameterInfo &p : m_layout.parameters()) {
        if (m_layout.find(p.name) != &p) {
            throw std::runtime_error("ECU: calibration parameter " + p.name + " declared twice");
        }

        for (const std::string &input : { p.inputX, p.inputY }) {
            if (!input.empty() && m_measurements.find(input) == nullptr) {
                throw std::runtime_error("ECU: axis input '" + input + "' of " + p.name
                    + " is not a measurement");
            }
        }
    }

    for (const MeasurementInfo &m : m_measurements.list()) {
        if (m_measurements.find(m.name) != &m) {
            throw std::runtime_error("ECU: measurement " + m.name + " declared twice");
        }
    }
}

std::string ecu::Ecu::fingerprint() const {
    uint32_t h = 2166136261u;
    for (const ParameterInfo &p : m_layout.parameters()) {
        hash(&h, p.name);
        hash(&h, std::to_string((int)p.type) + "," + std::to_string(p.sizeX) + "," + std::to_string(p.sizeY)
            + "," + std::to_string(p.offset) + "," + std::to_string(p.offsetX) + "," + std::to_string(p.offsetY));
    }
    for (const MeasurementInfo &m : m_measurements.list()) {
        hash(&h, m.name);
        hash(&h, std::to_string(reinterpret_cast<const char *>(m.value) - reinterpret_cast<const char *>(&m_signals)));
    }

    // Where the signals and the calibration memory sit in the executable:
    // XCP addresses are offsets from the module base, and they move with
    // every build that changes the code or data before them
#ifdef _WIN32
    const char *base = reinterpret_cast<const char *>(GetModuleHandleW(nullptr));
    hash(&h, std::to_string(reinterpret_cast<const char *>(&m_signals) - base));
    hash(&h, std::to_string(reinterpret_cast<const char *>(m_calibration.workingMemory()) - base));
#endif

    char text[32];
    std::snprintf(text, sizeof(text), "%s_%08X", SoftwareName, h);
    return text;
}

bool ecu::Ecu::setCalibration(const CalibrationPage &page, std::string *error) {
    if (m_xcp.status().connected) {
        *error = "not loaded while an XCP tool is connected";
        return false;
    }

    m_calibration.setWorkingPage(page);
    return m_xcp.loadWorkingPage(m_calibration, error);
}

bool ecu::Ecu::startXcp(const XcpSettings &settings, std::string *error) {
    return m_xcp.start("engine_sim_ecu", *this, settings, error);
}

void ecu::Ecu::beginFrame() {
    m_xcp.syncCalibration(m_calibration);
    m_calibration.updateSnapshot();

    // Rising edge of CalSaveRequest (its level at start-up doesn't count)
    const bool level = m_calSaveRequest(m_calibration.snapshot()) >= 0.5f;
    if (m_saveRequestSeen && level && !m_saveRequestLevel) {
        m_saveRequested = true;
        m_signals.calSaveCount += 1;
    }
    m_saveRequestLevel = level;
    m_saveRequestSeen = true;
}

bool ecu::Ecu::takeSaveRequest() {
    const bool requested = m_saveRequested;
    m_saveRequested = false;
    return requested;
}

float ecu::Ecu::calibrationValue(const std::string &name) const {
    const ParameterInfo *p = m_layout.find(name);
    return (p != nullptr) ? m_calibration.snapshot()[p->offset] : 0.0f;
}

void ecu::Ecu::configure(const EcuConfig &config) {
    m_config = config;
    reset();
}

void ecu::Ecu::reset() {
    const double calSaveCount = m_signals.calSaveCount;
    m_signals = EcuSignals();
    m_signals.calSaveCount = calSaveCount;
    m_signals.cylinders = m_config.cylinders;
    m_signals.remoteIo = m_config.remoteIo ? 1 : 0;
    for (const auto &function : m_functions) {
        function->reset(m_signals);
    }

    m_angleValid = false;
    m_timer10 = m_timer100 = 0;
    m_firstStep = true;
}

void ecu::Ecu::runTask(Task task, double dt) {
    const float *cal = m_calibration.snapshot().data();
    for (const auto &function : m_functions) {
        if (function->task() == task) {
            function->run(cal, m_signals, dt);
        }
    }
}

void ecu::Ecu::step(const EcuInputs &inputs, double dt, EcuOutputs *outputs) {
    EcuSignals &s = m_signals;
    s.time_s = inputs.time_s;
    s.cycleAngle_deg = inputs.cycleAngle_deg;
    s.rpmInstant = inputs.rpm;
    s.mapInstant_kPa = inputs.map_kPa;
    s.iat_C = inputs.iat_C;
    s.clt_C = inputs.clt_C;
    s.tps_percent = inputs.tps_percent;
    s.lambdaSensor = inputs.lambda;
    s.battery_V = inputs.battery_V;

    m_xcp.advanceClock(dt);
    runTask(Task::Step, dt);

    // The tolerance keeps a task due when the steps add up to its period
    // only within rounding (remote I/O: 10 x 10 ms samples)
    constexpr double Tolerance = 1e-9;
    m_timer10 += dt;
    m_timer100 += dt;
    if (m_firstStep || m_timer10 >= 0.010 - Tolerance) {
        runTask(Task::Ms10, m_timer10);
        m_timer10 = 0;
        m_xcp.event10ms();
    }
    if (m_firstStep || m_timer100 >= 0.100 - Tolerance) {
        runTask(Task::Ms100, m_timer100);
        m_timer100 = 0;
        m_xcp.event100ms();
    }
    m_firstStep = false;

    outputs->fuelControl = s.fuelControl >= 0.5;
    outputs->sparkControl = s.sparkControl >= 0.5;
    outputs->sparkAdvance_deg = s.advance_deg;
    outputs->injection_ms = outputs->fuelControl ? s.injection_ms : 0;
    outputs->injectionAngle_deg = m_injectionAngle(m_calibration.snapshot());

    if (m_config.remoteIo) {
        outputs->injectionCount = 0;
    }
    else {
        scheduleInjections(inputs, outputs);
    }
}

void ecu::Ecu::scheduleInjections(const EcuInputs &inputs, EcuOutputs *outputs) {
    outputs->injectionCount = 0;

    // No crank signal, no injection: below this speed the angle only jitters
    constexpr double MinRpm = 10.0;

    const double angle = inputs.cycleAngle_deg;
    double a0 = m_lastCycleAngle;
    double a1 = angle;
    m_lastCycleAngle = angle;

    const bool wasValid = m_angleValid;
    m_angleValid = true;
    if (!wasValid || inputs.rpm < MinRpm) return;

    if (a1 < a0) {
        // Wrapped through 720 -> 0; a small step back is the crank rocking
        if (a0 - a1 < 360.0) return;
        a1 += 720.0;
    }

    if (!outputs->fuelControl || m_signals.injection_ms <= 0) return;

    // One pulse per cylinder and cycle, when the crank passes its injection angle
    const double injectionAngle = m_injectionAngle(m_calibration.snapshot());
    const int cylinders = std::min((int)m_config.firingTdc_deg.size(), EcuOutputs::MaxInjections);
    for (int i = 0; i < cylinders; ++i) {
        const double target = std::fmod(m_config.firingTdc_deg[i] - injectionAngle + 1440.0, 720.0);
        const bool crossed =
            (target > a0 && target <= a1)
            || (target + 720.0 > a0 && target + 720.0 <= a1);

        if (crossed && outputs->injectionCount < EcuOutputs::MaxInjections) {
            EcuOutputs::Injection &injection = outputs->injections[outputs->injectionCount++];
            injection.cylinder = i;
            injection.pulseWidth_ms = m_signals.injection_ms;
        }
    }
}
