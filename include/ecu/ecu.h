#ifndef ATG_ENGINE_SIM_ECU_ECU_H
#define ATG_ENGINE_SIM_ECU_ECU_H

#include "calibration.h"

class Engine;

namespace ecu {
    // Virtual engine control unit. Runs once per physics step and applies the
    // calibration snapshot to the engine:
    //   - fuel:     FuelBaseMap (RPM x MAP -> injection time) drives a manifold
    //               injector instead of the script's fixed mixture
    //   - ignition: IgnAdvanceMap (RPM x MAP -> advance) replaces the script's
    //               timing curve
    // Each function can be switched off (EcuFuelEnable / EcuIgnEnable), which
    // restores the behaviour defined by the engine script.
    class Ecu {
        public:
            struct Measurements {
                double rpm = 0;
                double map_kPa = 0;
                double injectionTime_ms = 0;    // applied, or script equivalent when fuel is off
                double advance_deg = 0;         // applied spark advance
                CellPosition fuelCell;
                CellPosition ignitionCell;
                bool fuelActive = false;
                bool ignitionActive = false;
            };

        public:
            Ecu();

            Calibration &calibration() { return m_calibration; }

            // Once per frame: takes the calibration snapshot used by update()
            void beginFrame();

            // Once per physics step, before ignition and gas flow
            void update(Engine *engine, double dt);

            const Measurements &getMeasurements() const { return m_measurements; }

            // Learning: while the ECU functions are off, records the script's
            // mixture (as equivalent injection time) and timing curve per map
            // cell, to build a dataset that reproduces the script behaviour
            void setLearning(bool learning) { m_learning = learning; }
            bool isLearning() const { return m_learning; }
            void resetLearning();

            // Snapshot with the learned maps and both ECU functions enabled.
            // Returns the number of RPM columns with data; columns without data
            // copy the nearest learned column
            CalibrationPage buildLearnedPage(int *fuelColumns, int *ignitionColumns) const;

        private:
            // Weighted linear regression of a value against MAP, one per RPM
            // breakpoint: at fixed RPM the fuel per cycle is very close to
            // linear in MAP (measured: within a few % from 10 to 100 kPa)
            struct ColumnFit {
                double w = 0, wx = 0, wy = 0, wxx = 0, wxy = 0;

                void add(double weight, double x, double y);
                bool valid() const;
                double evaluate(double x) const;
            };

            void learn(const CalibrationPage &cal, double scriptInjectionTime_ms, double scriptAdvance_deg);
            static void addSample(ColumnFit *fits, const float *axisX, double rpm, double map_kPa, double value);
            static int fillMap(const ColumnFit *fits, const float *axisY, float values[][MapRpmPoints]);

            Calibration m_calibration;
            Measurements m_measurements;

            double m_lastTotalFuel;
            double m_filteredScriptInjectionTime;

            // Cranking until the engine exceeds CrankingRpm; back to cranking
            // only when it nearly stops (hysteresis avoids toggling at the
            // threshold while the engine catches)
            bool m_engineRunning;
            double m_runningTime;   // since the engine left cranking [s]

            // Learning only uses steady manifold pressure: while the plenum is
            // filling or emptying, the fuel entering it doesn't match the fuel
            // reaching the cylinders
            double m_lastMap_kPa;
            double m_filteredMapRate;

            bool m_learning;
            ColumnFit m_fuelFit[MapRpmPoints];
            ColumnFit m_ignitionFit[MapRpmPoints];
    };
}

#endif /* ATG_ENGINE_SIM_ECU_ECU_H */
