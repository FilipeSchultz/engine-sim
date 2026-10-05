#include "ecu/xcp.h"

#include "ecu/ecu.h"

#ifndef ECU_XCP_ENABLED

bool ecu::XcpInterface::available() { return false; }

bool ecu::XcpInterface::start(const std::string &name, Ecu &ecu, const XcpSettings &settings, std::string *error) {
    *error = "the ECU was built without XCP (CMake option ECU_XCP_ENABLED)";
    return false;
}

void ecu::XcpInterface::stop() { /* void */ }
ecu::XcpStatus ecu::XcpInterface::status() const { return XcpStatus(); }
void ecu::XcpInterface::syncCalibration(Calibration &calibration) { /* void */ }

bool ecu::XcpInterface::loadWorkingPage(const Calibration &calibration, std::string *error) {
    return true;
}

void ecu::XcpInterface::advanceClock(double dt) { /* void */ }
void ecu::XcpInterface::event10ms() { /* void */ }
void ecu::XcpInterface::event100ms() { /* void */ }

#else

#include <a2l.h>
#include <xcplib.h>

#include <atomic>
#include <map>
#include <mutex>
#include <set>

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#pragma comment(lib, "winmm.lib")
#endif

// XCPlite internals (src/cal.h): write the XCP working page of a calibration
// segment, as a tool's DOWNLOAD does, and publish pending writes to the ECU.
// The public API has no way for the application to replace the working page.
// Both must run in the XCP server thread (single writer), hence the idle
// callback below.
extern "C" uint8_t XcpCalSegWriteMemory(uint32_t dst, uint16_t size, const uint8_t *src);
extern "C" uint8_t XcpCalSegPublishAll(bool wait);

namespace {
    // Simulated time [ns], read by the XCP server thread (GET_DAQ_CLOCK)
    std::atomic<uint64_t> SimulatedClock{ 0 };

    uint64_t simulatedClock() { return SimulatedClock.load(std::memory_order_relaxed); }

    // Segment relative address (XcpAddrEncodeSegIndex in XCPlite's xcp_cfg.h)
    uint32_t segmentAddress(uint16_t segment, uint32_t offset) {
        return 0x80000000u + (static_cast<uint32_t>(segment) << 16) + offset;
    }

    // Display range of the measurements in the A2L (the signals have none)
    constexpr double MeasurementRange = 1e6;

    const char *orNull(const std::string &s) { return s.empty() ? nullptr : s.c_str(); }

    // Calibration file loaded: page to write into the working page, handed to
    // the XCP server thread
    std::mutex PendingMutex;
    std::vector<float> PendingPage;
    uint16_t PendingSegment = 0xFFFF;
    std::atomic<bool> PendingLoad{ false };

    uint8_t writeWorkingPage(uint16_t segment, const std::vector<float> &page) {
        return XcpCalSegWriteMemory(
            segmentAddress(segment, 0),
            static_cast<uint16_t>(page.size() * sizeof(float)),
            reinterpret_cast<const uint8_t *>(page.data()));
    }

    // XCP server thread, between commands
    void serverIdle() {
        if (!PendingLoad.load() || XcpIsConnected()) return;

        std::lock_guard<std::mutex> lock(PendingMutex);
        writeWorkingPage(PendingSegment, PendingPage);
        PendingLoad = false;
    }
}

bool ecu::XcpInterface::available() { return true; }

bool ecu::XcpInterface::start(const std::string &name, Ecu &ecu, const XcpSettings &settings, std::string *error) {
    if (m_running) return true;

    XcpSetLogLevel(static_cast<uint8_t>(settings.logLevel));
    if (!XcpInit(name.c_str(), ecu.epk().c_str(), XCP_MODE_LOCAL)) {
        *error = "XcpInit failed";
        return false;
    }

    m_simulatedClock = settings.simulatedClock;
    if (m_simulatedClock) {
        ApplXcpRegisterGetClockCallback(simulatedClock);
    }

    if (settings.server
        && !XcpEthServerInit(settings.address, settings.port, false, settings.queueSize))
    {
        *error = "cannot open UDP port " + std::to_string(settings.port)
            + " (another XCP slave running?)";
        return false;
    }
    m_server = settings.server;
    if (m_server) {
        ApplXcpRegisterIdleCallback(serverIdle);
    }

#ifdef _WIN32
    // XCPlite's server threads sleep with Sleep(): 1 ms resolution
    timeBeginPeriod(1);
#endif

    const bool a2l = settings.generateA2l
        && A2lInit(settings.address, settings.port, false, A2L_MODE_WRITE_ALWAYS);

    // Cyclic events (XcpCreateEvent directly: the Daq... macros drop the
    // cycle time on MSVC, and VISION configures no DAQ on sporadic events)
    m_event10ms = XcpCreateEvent("Ecu_10ms", 10000000, 0);
    m_event100ms = XcpCreateEvent("Ecu_100ms", 100000000, 0);

    // Calibration segment: the default page is the ECU's static calibration
    // memory, so parameters have absolute addresses (ACSDD mode)
    const Calibration &calibration = ecu.calibration();
    const float *memory = calibration.workingMemory();
    m_calibrationSegment = XcpCreateCalSeg("EcuCalibration", memory,
        static_cast<uint16_t>(calibration.size() * sizeof(float)));
    if (m_calibrationSegment == XCP_UNDEFINED_CALSEG) {
        *error = "cannot create the calibration segment";
        return false;
    }

    if (a2l) {
        const CalibrationLayout &layout = ecu.layout();

        A2lSetSegmentAddrMode__i(m_calibrationSegment, reinterpret_cast<const uint8_t *>(memory));
        std::map<std::string, std::vector<std::string>> parameterGroups;
        for (const ParameterInfo &p : layout.parameters()) {
            std::vector<std::string> &group = parameterGroups[p.function];
            const std::string axisX = p.name + "_X", axisY = p.name + "_Y";

            if (p.type == ParameterInfo::Type::Value) {
                A2lCreateParameter_(p.name.c_str(), A2L_TYPE_FLOAT, memory + p.offset,
                    p.description.c_str(), p.unit.c_str(), p.min, p.max);
            }
            else {
                // Shared, editable axes (AXIS_PTS + COM_AXIS); the input
                // quantity marks the operating point on the table
                A2lSetInputQuantity_x(orNull(p.inputX));
                A2lCreateAxis_(axisX.c_str(), A2L_TYPE_FLOAT, memory + p.offsetX, p.sizeX,
                    ("X axis of " + p.name).c_str(), p.unitX.c_str(), p.minX, p.maxX);
                group.push_back(axisX);

                if (p.type == ParameterInfo::Type::Curve) {
                    A2lCreateCurve_(p.name.c_str(), A2L_TYPE_FLOAT, memory + p.offset, p.sizeX,
                        p.description.c_str(), p.unit.c_str(), p.min, p.max, axisX.c_str());
                }
                else {
                    A2lSetInputQuantity_x(orNull(p.inputY));
                    A2lCreateAxis_(axisY.c_str(), A2L_TYPE_FLOAT, memory + p.offsetY, p.sizeY,
                        ("Y axis of " + p.name).c_str(), p.unitY.c_str(), p.minY, p.maxY);
                    group.push_back(axisY);

                    // Values row by row, one row per Y breakpoint ([y][x])
                    A2lSetInputQuantity_x(orNull(p.inputX));
                    A2lSetInputQuantity_y(orNull(p.inputY));
                    A2lCreateMap_(p.name.c_str(), A2L_TYPE_FLOAT, memory + p.offset, p.sizeX, p.sizeY,
                        p.description.c_str(), p.unit.c_str(), p.min, p.max, axisX.c_str(), axisY.c_str());
                }

                A2lSetInputQuantity_x(nullptr);
                A2lSetInputQuantity_y(nullptr);
            }

            group.push_back(p.name);
        }

        // Measurements: absolute addresses, default event by the task that
        // computes them
        std::set<std::string> slowFunctions;
        for (const auto &function : ecu.functions()) {
            if (function->task() == Task::Ms100) slowFunctions.insert(function->name());
        }

        std::map<std::string, std::vector<std::string>> measurementGroups;
        for (const bool slow : { false, true }) {
            A2lSetAbsoluteAddrMode__i(slow ? m_event100ms : m_event10ms);
            for (const MeasurementInfo &m : ecu.measurements().list()) {
                if ((slowFunctions.count(m.function) > 0) != slow) continue;

                A2lCreateMeasurement_(nullptr, m.name.c_str(), A2L_TYPE_DOUBLE, 1, m.value,
                    m.unit.c_str(), -MeasurementRange, MeasurementRange, m.description.c_str());
                measurementGroups[m.function].push_back(m.name);
            }
        }

        // One group of parameters and one of measurements per ECU function
        for (const auto &group : parameterGroups) {
            std::vector<const char *> names;
            for (const std::string &n : group.second) names.push_back(n.c_str());
            A2lCreateParameterGroupFromList(("Cal_" + group.first).c_str(), names.data(), (int)names.size());
        }
        for (auto &group : measurementGroups) {
            std::vector<char *> names;
            for (std::string &n : group.second) names.push_back(&n[0]);
            A2lCreateMeasurementGroupFromList(("Meas_" + group.first).c_str(), names.data(), (uint32_t)names.size());
        }

        A2lFinalize();
        const char *file = A2lGetFilename();
        m_a2lFile = (file != nullptr) ? file : "";
    }

    m_running = true;
    return true;
}

void ecu::XcpInterface::stop() {
    if (!m_running) return;

    XcpDisconnect();
    if (m_server) XcpEthServerShutdown();
    m_running = false;

#ifdef _WIN32
    timeEndPeriod(1);
#endif
}

ecu::XcpStatus ecu::XcpInterface::status() const {
    XcpStatus status;
    status.running = m_running;
    status.connected = m_running && XcpIsConnected();
    status.a2lFile = m_a2lFile;
    return status;
}

void ecu::XcpInterface::syncCalibration(Calibration &calibration) {
    if (!m_running) return;

    // The tool measures parameters by DAQ at their absolute address, the
    // default page; mirroring the active page there shows the values in use.
    // (The default page so no longer keeps the values the ECU started with:
    // the DCM file is the reference.)
    const float *page = reinterpret_cast<const float *>(XcpLockCalSeg(m_calibrationSegment));
    if (page != calibration.workingMemory()) {
        calibration.mirror(page);
    }
    XcpUnlockCalSeg(m_calibrationSegment);
}

bool ecu::XcpInterface::loadWorkingPage(const Calibration &calibration, std::string *error) {
    if (!m_running) return true;

    if (XcpIsConnected()) {
        *error = "an XCP tool is connected";
        return false;
    }

    const std::vector<float> page(
        calibration.workingMemory(), calibration.workingMemory() + calibration.size());

    if (m_server) {
        // Written by the server thread within its receive timeout; until
        // then the ECU keeps the previous values
        std::lock_guard<std::mutex> lock(PendingMutex);
        PendingPage = page;
        PendingSegment = m_calibrationSegment;
        PendingLoad = true;
        return true;
    }

    // No server thread (tests): write and publish here
    const uint8_t result = writeWorkingPage(m_calibrationSegment, page);
    if (result != 0) {
        *error = "XCP working page update failed (error " + std::to_string(result) + ")";
        return false;
    }
    XcpCalSegPublishAll(false);

    return true;
}

void ecu::XcpInterface::advanceClock(double dt) {
    m_clock_s += dt;
    SimulatedClock.store(static_cast<uint64_t>(m_clock_s * 1e9), std::memory_order_relaxed);
}

void ecu::XcpInterface::event10ms() {
    if (!m_running) return;
    if (m_simulatedClock) XcpEventAt(m_event10ms, simulatedClock());
    else XcpEvent(m_event10ms);
}

void ecu::XcpInterface::event100ms() {
    if (!m_running) return;
    if (m_simulatedClock) XcpEventAt(m_event100ms, simulatedClock());
    else XcpEvent(m_event100ms);
}

#endif
