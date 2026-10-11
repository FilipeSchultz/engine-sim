#ifndef ECU_HOST_H
#define ECU_HOST_H

#include "ecu.h"
#include "link.h"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <string>
#include <vector>

namespace ecu {
    // The ECU's environment on a PC, shared by every program that runs the
    // ECU (the simulator, the standalone ECU): the calibration file (loaded at
    // start-up, reloaded whenever it is saved, written on the tool's save
    // request), the XCP server and the datalog. Reports what happens through
    // the message handler (one line per event).
    class EcuHost {
        public:
            using MessageHandler = std::function<void(const std::string &)>;

            explicit EcuHost(Ecu &ecu) : m_ecu(ecu) { /* void */ }
            ~EcuHost() { closeLog(); }

            EcuHost(const EcuHost &) = delete;
            EcuHost &operator=(const EcuHost &) = delete;

            void setMessageHandler(MessageHandler handler) { m_message = std::move(handler); }

            // Loads the calibration file; a missing file is first written with
            // the default calibration (ECU functions disabled)
            void openCalibration(const std::string &path);
            const std::string &calibrationPath() const { return m_calibrationPath; }

            bool reloadCalibration();
            bool saveCalibration();

            // False when the last load of the file was rejected (the ECU keeps
            // the previous values)
            bool calibrationFileOk() const { return m_calibrationOk; }

            // XCP on Ethernet; the A2L is written to the working directory,
            // with a copy of XCP_104.aml from the executable's directory
            bool startXcp(const XcpSettings &settings);
            bool xcpConnected() const { return m_xcpConnected; }

            // Once per frame, before the ECU steps: takes the calibration
            // snapshot, reports tool connections, saves on the tool's request
            // and reloads the calibration file when it changes
            void update(double dt);

            // Datalog (CSV): time, every ECU measurement, then the extra
            // columns, whose values the caller gives with each row
            bool openLog(const std::string &path, const std::vector<std::string> &extraColumns = {});
            void closeLog();
            bool logging() const { return m_log.is_open(); }

            // Advances the log clock; true when a row is due (first call,
            // then 20 Hz)
            bool logDue(double dt);
            void writeLogRow(const std::vector<double> &extra = {});

        private:
            void message(const std::string &text) const;

            Ecu &m_ecu;
            MessageHandler m_message;

            std::string m_calibrationPath;
            std::filesystem::file_time_type m_calibrationFileTime;
            double m_calibrationCheckTimer = 0.0;
            bool m_calibrationOk = false;
            bool m_xcpConnected = false;

            std::ofstream m_log;
            double m_logTime = 0.0;
            double m_nextRow = 0.0;
            int m_logLines = 0;
    };
}

#endif /* ECU_HOST_H */
