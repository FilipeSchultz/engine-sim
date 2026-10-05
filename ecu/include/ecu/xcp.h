#ifndef ECU_XCP_H
#define ECU_XCP_H

#include "calibration.h"
#include "measurement.h"

#include <cstdint>
#include <string>
#include <vector>

namespace ecu {
    class Ecu;

    struct XcpSettings {
        // UDP server: 0.0.0.0 = every interface (local and remote tools),
        // 127.0.0.1 = this computer only
        uint8_t address[4] = { 0, 0, 0, 0 };
        uint16_t port = 5555;
        uint32_t queueSize = 256 * 1024;    // DAQ transmit queue [bytes]

        // DAQ timestamps and GET_DAQ_CLOCK in simulated time (plots follow
        // the engine even when the simulation doesn't run in real time) or
        // in the PC's clock
        bool simulatedClock = true;

        int logLevel = 2;                   // XCPlite debug output (0..4, 4 = every command)

        // Off for tests: no UDP server, no A2L file
        bool server = true;
        bool generateA2l = true;
    };

    struct XcpStatus {
        bool running = false;
        bool connected = false;
        std::string a2lFile;    // generated A2L (relative to the working directory)
    };

    // XCP on Ethernet through XCPlite: the ECU's calibration memory becomes a
    // calibration segment and its measurements are sampled by the DAQ events
    // Ecu_10ms and Ecu_100ms. The A2L file is generated from the parameter
    // and measurement declarations. Used by Ecu; does nothing when the
    // library is built without ECU_XCP_ENABLED.
    class XcpInterface {
        public:
            static bool available();

            // name: XCP project name (A2L file name)
            bool start(const std::string &name, Ecu &ecu, const XcpSettings &settings, std::string *error);
            void stop();

            XcpStatus status() const;

            // Once per frame: copies the active calibration page into the
            // ECU's calibration memory (the default page, which is where the
            // tool reads the parameters by DAQ)
            void syncCalibration(Calibration &calibration);

            // Writes the ECU's calibration memory into the working page (file
            // reload), through the XCP server thread (XCPlite has a single
            // writer) within ~0.1 s. Refused while a tool is connected.
            bool loadWorkingPage(const Calibration &calibration, std::string *error);

            // Simulation step: advances the simulated clock
            void advanceClock(double dt);
            void event10ms();
            void event100ms();

        private:
            bool m_running = false;
            bool m_server = false;
            bool m_simulatedClock = true;
            double m_clock_s = 0;           // simulated time [s]
            uint16_t m_calibrationSegment = 0xFFFF;
            uint16_t m_event10ms = 0xFFFF;
            uint16_t m_event100ms = 0xFFFF;
            std::string m_a2lFile;
    };
}

#endif /* ECU_XCP_H */
