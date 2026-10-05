#ifndef ECU_FUNCTIONS_H
#define ECU_FUNCTIONS_H

#include "function.h"

// The ECU functions, in execution order. Each one lives in its own file in
// ecu/src/functions/.
namespace ecu {
    // Sensor processing: engine speed and manifold pressure averaged over the
    // last firing interval (as an ECU sampling them in sync with the crank),
    // which removes the intake pulsation
    class InputProcessing : public EcuFunction {
        public:
            InputProcessing() : EcuFunction("InputProcessing", Task::Step) { /* void */ }
            void declare(CalibrationLayout &cal, MeasurementTable &meas, EcuSignals &s) override;
            void reset(EcuSignals &s) override;
            void run(const float *cal, EcuSignals &s, double dt) override;

        private:
            double m_angle = 0;         // [deg] within the current interval
            double m_mapTime = 0;       // [kPa s]
            double m_time = 0;          // [s]
            bool m_valid = false;
    };

    // Stopped / cranking / running, with hysteresis, and time since start
    class EngineStateControl : public EcuFunction {
        public:
            EngineStateControl() : EcuFunction("EngineState", Task::Step) { /* void */ }
            void declare(CalibrationLayout &cal, MeasurementTable &meas, EcuSignals &s) override;
            void run(const float *cal, EcuSignals &s, double dt) override;

        private:
            Value m_crankingRpm;
    };

    // Fuel cut above a speed limit
    class RevLimiter : public EcuFunction {
        public:
            RevLimiter() : EcuFunction("RevLimiter", Task::Step) { /* void */ }
            void declare(CalibrationLayout &cal, MeasurementTable &meas, EcuSignals &s) override;
            void run(const float *cal, EcuSignals &s, double dt) override;

        private:
            Value m_limit;
            Value m_hysteresis;
    };

    // Closed loop lambda: PI controller on the wideband sensor, output as a
    // fuel correction factor
    class LambdaControl : public EcuFunction {
        public:
            LambdaControl() : EcuFunction("LambdaControl", Task::Ms10) { /* void */ }
            void declare(CalibrationLayout &cal, MeasurementTable &meas, EcuSignals &s) override;
            void reset(EcuSignals &s) override;
            void run(const float *cal, EcuSignals &s, double dt) override;

        private:
            Value m_enable;
            Value m_target;
            Value m_kp;
            Value m_ki;
            Value m_limit;
            Value m_startDelay;

            double m_integral = 0;
    };

    // Injection time: base map (rpm x MAP) times the corrections
    class FuelControl : public EcuFunction {
        public:
            FuelControl() : EcuFunction("FuelControl", Task::Ms10) { /* void */ }
            void declare(CalibrationLayout &cal, MeasurementTable &meas, EcuSignals &s) override;
            void run(const float *cal, EcuSignals &s, double dt) override;

        private:
            Value m_enable;
            Value m_injectorDeadTime;
            Map m_baseMap;
            Value m_crankingFactor;
            Value m_afterStartTime;
            Curve m_cltCorrection;
            Curve m_iatCorrection;
    };

    // Spark advance: base map (rpm x MAP) plus the corrections
    class IgnitionControl : public EcuFunction {
        public:
            IgnitionControl() : EcuFunction("IgnitionControl", Task::Ms10) { /* void */ }
            void declare(CalibrationLayout &cal, MeasurementTable &meas, EcuSignals &s) override;
            void run(const float *cal, EcuSignals &s, double dt) override;

        private:
            Value m_enable;
            Map m_baseMap;
            Curve m_cltCorrection;
            Curve m_iatCorrection;
    };
}

#endif /* ECU_FUNCTIONS_H */
