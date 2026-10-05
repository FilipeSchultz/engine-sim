#ifndef ECU_ECU_H
#define ECU_ECU_H

#include "calibration.h"
#include "function.h"
#include "io.h"
#include "measurement.h"
#include "signals.h"
#include "xcp.h"

#include <memory>
#include <string>
#include <vector>

namespace ecu {
    // Virtual engine control unit. Self-contained: it only knows its inputs
    // (sensors), outputs (injectors, spark) and calibration; the simulation
    // drives it through EcuInputs / EcuOutputs.
    //
    // Every simulation step:
    //   1. the inputs are copied into the signals
    //   2. the scheduler runs the functions (every step, 10 ms, 100 ms tasks)
    //   3. the output stage fires the injectors at their crank angle and
    //      passes the spark advance on
    //
    // The signals and the working calibration page live in static memory
    // (fixed addresses, as XCP measurement and calibration need), so there is
    // one ECU per process: a new Ecu takes that memory over.
    class Ecu {
        public:
            Ecu();
            ~Ecu() { stopXcp(); }
            Ecu(const Ecu &) = delete;
            Ecu &operator=(const Ecu &) = delete;

            // Software identification (XCP EPK): name and a fingerprint of the
            // calibration and measurement layout and of their addresses in
            // the executable. It changes whenever a parameter or measurement
            // is added, removed or resized, or a rebuild moves them, so a
            // calibration tool with an old A2L can't connect to the new layout.
            const std::string &epk() const { return m_epk; }

            const CalibrationLayout &layout() const { return m_layout; }
            Calibration &calibration() { return m_calibration; }
            const Calibration &calibration() const { return m_calibration; }
            const MeasurementTable &measurements() const { return m_measurements; }
            const EcuSignals &signals() const { return m_signals; }
            const std::vector<std::unique_ptr<EcuFunction>> &functions() const { return m_functions; }

            // Value of a calibration parameter in the current snapshot
            // (first value for curves and maps)
            float calibrationValue(const std::string &name) const;

            // Engine attached (cylinders, firing order); resets the functions
            void configure(const EcuConfig &config);
            void reset();

            // Replaces the calibration (file loaded). With XCP running, also
            // the tool's working page; refused while a tool is connected.
            bool setCalibration(const CalibrationPage &page, std::string *error);

            // XCP on Ethernet (see xcp.h); the calibration must be loaded first
            bool startXcp(const XcpSettings &settings, std::string *error);
            void stopXcp() { m_xcp.stop(); }
            XcpStatus xcpStatus() const { return m_xcp.status(); }

            // Once per frame: takes the calibration snapshot used by step()
            // (from the tool's active page with XCP)
            void beginFrame();

            // CalSaveRequest went from 0 to 1 since the last call (the tool
            // asks for the calibration to be saved)
            bool takeSaveRequest();

            // Once per simulation step
            void step(const EcuInputs &inputs, double dt, EcuOutputs *outputs);

        private:
            void runTask(Task task, double dt);
            void scheduleInjections(const EcuInputs &inputs, EcuOutputs *outputs);

            void checkLayout() const;
            std::string fingerprint() const;

            CalibrationLayout m_layout;
            Calibration m_calibration;
            MeasurementTable m_measurements;
            EcuSignals &m_signals;
            std::string m_epk;
            std::vector<std::unique_ptr<EcuFunction>> m_functions;

            XcpInterface m_xcp;

            Value m_calSaveRequest;
            bool m_saveRequestSeen = false;
            bool m_saveRequestLevel = false;
            bool m_saveRequested = false;

            // Output stage
            Value m_injectionAngle;
            EcuConfig m_config;
            double m_lastCycleAngle = 0;
            bool m_angleValid = false;

            // Scheduler
            double m_timer10 = 0;
            double m_timer100 = 0;
            bool m_firstStep = true;
    };
}

#endif /* ECU_ECU_H */
