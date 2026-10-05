#ifndef ATG_ENGINE_SIM_ECU_LEARNER_H
#define ATG_ENGINE_SIM_ECU_LEARNER_H

#include "ecu/ecu.h"

#include <vector>

class Engine;

// Bench tool (not part of the ECU): while the ECU functions are off, records
// the engine script's mixture (as the equivalent injection time with the
// calibrated injector) and timing curve per map cell, to build a dataset that
// reproduces the script behaviour with the ECU in control.
class EcuLearner {
    public:
        void reset();

        // Once per physics step, after the ECU
        void sample(Engine *engine, const ecu::Ecu &ecu, const ecu::EcuOutputs &outputs, double dt);

        // The current calibration with the learned maps and both ECU
        // functions enabled. Columns without data copy the nearest learned one.
        ecu::CalibrationPage buildPage(const ecu::Ecu &ecu, int *fuelColumns, int *ignitionColumns) const;

    private:
        // Value against MAP for one rpm breakpoint: steady cycles are averaged
        // in 2 kPa bins, then the curve is interpolated piecewise linearly
        // between bins and extrapolated from the end segments. A single
        // straight line is not enough: at low rpm the fuel per cycle has a
        // knee (exhaust backflow at light load).
        struct ColumnFit {
            static constexpr double BinWidth = 2.0;     // [kPa]
            static constexpr int Bins = 200;            // 0 .. 400 kPa

            double w[Bins] = {};
            double wx[Bins] = {};
            double wy[Bins] = {};

            void add(double weight, double x, double y);
            bool valid() const;
            double evaluate(double x) const;
        };

        // One engine cycle (720 deg) of the script behaviour
        struct CycleAccumulator {
            double angle = 0;           // [deg]
            double fuel = 0;            // [mol]
            double mapTime = 0;         // [kPa s]
            double advanceTime = 0;     // [deg s]
            double time = 0;            // [s]
            int steps = 0;
            double previousMap_kPa = -1;
        };

        static void addSample(
            std::vector<ColumnFit> &fits, const float *axisX, int sizeX,
            double rpm, double map_kPa, double value, double weight);
        static int fillMap(
            const std::vector<ColumnFit> &fits, const ecu::ParameterInfo &map,
            ecu::CalibrationPage *page);

        CycleAccumulator m_cycle;
        std::vector<ColumnFit> m_fuelFit;
        std::vector<ColumnFit> m_ignitionFit;
        double m_lastTotalFuel = -1;
};

#endif /* ATG_ENGINE_SIM_ECU_LEARNER_H */
