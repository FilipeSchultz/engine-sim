#include "ecu/host.h"

#include "ecu/dcm_file.h"

#include <cstdio>
#include <ctime>

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#endif

namespace {
    // Directory of the running executable (empty if unknown)
    std::filesystem::path executableDirectory() {
#ifdef _WIN32
        wchar_t exe[MAX_PATH];
        const DWORD length = GetModuleFileNameW(nullptr, exe, MAX_PATH);
        if (length == 0 || length >= MAX_PATH) return {};
        return std::filesystem::path(exe).parent_path();
#else
        std::error_code ec;
        return std::filesystem::read_symlink("/proc/self/exe", ec).parent_path();
#endif
    }
}

bool ecu::parseIpv4(const std::string &text, uint8_t address[4]) {
    int a = 0, b = 0, c = 0, d = 0;
    char end = 0;
    if (std::sscanf(text.c_str(), "%d.%d.%d.%d%c", &a, &b, &c, &d, &end) != 4) return false;

    for (const int v : { a, b, c, d }) {
        if (v < 0 || v > 255) return false;
    }

    address[0] = (uint8_t)a;
    address[1] = (uint8_t)b;
    address[2] = (uint8_t)c;
    address[3] = (uint8_t)d;
    return true;
}

void ecu::EcuHost::message(const std::string &text) const {
    if (m_message) m_message(text);
}

void ecu::EcuHost::openCalibration(const std::string &path) {
    m_calibrationPath = path;

    // First run: write the default (disabled) calibration so there is a file to edit
    std::error_code ec;
    if (!std::filesystem::exists(m_calibrationPath, ec)) {
        const std::filesystem::path parent = std::filesystem::path(m_calibrationPath).parent_path();
        if (!parent.empty()) std::filesystem::create_directories(parent, ec);

        std::string error;
        if (!writeDcm(
            m_calibrationPath,
            m_ecu.layout(),
            m_ecu.layout().defaults(),
            "Default calibration: ECU functions disabled (engine script behaviour)\n"
            "ECU software: " + m_ecu.epk(),
            &error))
        {
            message("ECU: " + error);
        }
    }

    reloadCalibration();
}

bool ecu::EcuHost::reloadCalibration() {
    std::error_code ec;
    m_calibrationFileTime = std::filesystem::last_write_time(m_calibrationPath, ec);

    CalibrationPage page = m_ecu.calibration().getWorkingPage();
    std::vector<std::string> warnings;
    std::string error;
    if (!readDcm(m_calibrationPath, m_ecu.layout(), &page, &warnings, &error)) {
        // Keep running with the previous calibration
        m_calibrationOk = false;
        message("ECU: " + error);
        return false;
    }

    if (!m_ecu.setCalibration(page, &error)) {
        m_calibrationOk = false;
        message("ECU: calibration file " + error);
        return false;
    }
    m_calibrationOk = true;

    auto enabled = [&](const char *name) {
        const ParameterInfo *p = m_ecu.layout().find(name);
        return (p != nullptr && page[p->offset] >= 0.5f) ? "ON" : "OFF";
    };

    std::string text = "ECU: calibration loaded (fuel ";
    text += enabled("EcuFuelEnable");
    text += ", ignition ";
    text += enabled("EcuIgnEnable");
    text += ")";
    if (!warnings.empty()) {
        text += " - " + warnings.front();
    }

    message(text);
    return true;
}

bool ecu::EcuHost::saveCalibration() {
    // Snapshot of the calibration in use, saved with the request flag cleared
    CalibrationPage page = m_ecu.calibration().snapshot();
    if (const ParameterInfo *p = m_ecu.layout().find("CalSaveRequest")) {
        page[p->offset] = 0;
    }

    std::string error, backup;
    if (!backupFile(m_calibrationPath, &backup, &error)) {
        message("ECU: calibration not saved, " + error);
        return false;
    }

    const std::time_t now = std::time(nullptr);
    char date[32];
    std::strftime(date, sizeof(date), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
    const std::string comment =
        std::string("Saved from the ECU on ") + date + "\n"
        "ECU software: " + m_ecu.epk();
    if (!writeDcm(m_calibrationPath, m_ecu.layout(), page, comment, &error)) {
        message("ECU: calibration not saved, " + error);
        return false;
    }

    // Our own write: not a change to reload
    std::error_code ec;
    m_calibrationFileTime = std::filesystem::last_write_time(m_calibrationPath, ec);

    message("ECU: calibration saved (previous file in "
        + std::filesystem::path(backup).parent_path().filename().string() + "/)");
    return true;
}

bool ecu::EcuHost::startXcp(const XcpSettings &settings) {
    // The A2L is written to the working directory and includes XCP_104.aml
    // from there; the build puts a copy next to the executable
    std::error_code ec;
    if (!std::filesystem::exists("XCP_104.aml", ec)) {
        const std::filesystem::path aml = executableDirectory() / "XCP_104.aml";
        std::filesystem::copy_file(aml, "XCP_104.aml", ec);
    }

    std::string error;
    if (!m_ecu.startXcp(settings, &error)) {
        message("XCP: " + error);
        return false;
    }

    const XcpStatus status = m_ecu.xcpStatus();
    char text[128];
    std::snprintf(text, sizeof(text), "XCP: waiting on %u.%u.%u.%u:%u, A2L ",
        settings.address[0], settings.address[1], settings.address[2], settings.address[3], settings.port);
    message(text + std::filesystem::absolute(status.a2lFile, ec).string());
    return true;
}

void ecu::EcuHost::update(double dt) {
    m_ecu.beginFrame();

    // XCP tool connection
    const XcpStatus xcp = m_ecu.xcpStatus();
    if (xcp.running && xcp.connected != m_xcpConnected) {
        m_xcpConnected = xcp.connected;
        message(m_xcpConnected
            ? "XCP: tool connected (calibration file reload paused)"
            : "XCP: tool disconnected");
    }

    if (m_ecu.takeSaveRequest()) {
        saveCalibration();
    }

    // Reload the calibration file whenever it is saved. While a calibration
    // tool is connected it owns the calibration: the change is ignored (save
    // the file again after disconnecting to load it).
    m_calibrationCheckTimer -= dt;
    if (m_calibrationCheckTimer <= 0 && !m_calibrationPath.empty()) {
        m_calibrationCheckTimer = 0.5;

        std::error_code ec;
        const auto fileTime = std::filesystem::last_write_time(m_calibrationPath, ec);
        if (!ec && fileTime != m_calibrationFileTime) {
            if (m_xcpConnected) {
                m_calibrationFileTime = fileTime;
                message("ECU: calibration file changed, not loaded while an XCP tool is connected");
            }
            else {
                reloadCalibration();
            }
        }
    }
}

bool ecu::EcuHost::openLog(const std::string &path, const std::vector<std::string> &extraColumns) {
    closeLog();

    m_log.open(path, std::ios::out);
    if (!m_log.is_open()) {
        message("ECU: cannot open the datalog " + path);
        return false;
    }

    m_log << "time_s";
    for (const MeasurementInfo &m : m_ecu.measurements().list()) {
        m_log << "," << m.name;
    }
    for (const std::string &column : extraColumns) {
        m_log << "," << column;
    }
    m_log << "\n";

    m_logTime = m_nextRow = 0.0;
    m_logLines = 0;
    return true;
}

void ecu::EcuHost::closeLog() {
    if (m_log.is_open()) m_log.close();
}

bool ecu::EcuHost::logDue(double dt) {
    if (!m_log.is_open()) return false;

    // A row at the first call, then on the 50 ms grid whatever the step;
    // the tolerance keeps rounding from skipping a row, and a long pause
    // doesn't make a burst
    constexpr double Period = 0.05;
    m_logTime += dt;
    if (m_logTime < m_nextRow - 1e-9) return false;

    m_nextRow += Period;
    if (m_nextRow <= m_logTime) m_nextRow = m_logTime + Period;
    return true;
}

void ecu::EcuHost::writeLogRow(const std::vector<double> &extra) {
    if (!m_log.is_open()) return;

    m_log << m_logTime;
    for (const MeasurementInfo &m : m_ecu.measurements().list()) {
        m_log << "," << *m.value;
    }
    for (const double v : extra) {
        m_log << "," << v;
    }
    m_log << "\n";

    // Keep the file usable even if the program is killed
    if (++m_logLines % 20 == 0) m_log.flush();
}
