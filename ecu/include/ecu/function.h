#ifndef ECU_FUNCTION_H
#define ECU_FUNCTION_H

#include "calibration.h"
#include "measurement.h"
#include "signals.h"

namespace ecu {
    // Rate at which the scheduler runs a function
    enum class Task {
        Step,       // every simulation step (input capture, fast logic)
        Ms10,       // every 10 ms
        Ms100       // every 100 ms
    };

    // One ECU function (fuel, ignition, lambda control...). To add one:
    //   1. derive from EcuFunction, declare its calibration parameters and
    //      measurements in declare(), implement run()
    //   2. add its signals to EcuSignals
    //   3. register it in Ecu::Ecu() (the order there is the execution order)
    class EcuFunction {
        public:
            EcuFunction(const char *name, Task task) : m_name(name), m_task(task) { /* void */ }
            virtual ~EcuFunction() = default;

            const char *name() const { return m_name; }
            Task task() const { return m_task; }

            virtual void declare(CalibrationLayout &cal, MeasurementTable &meas, EcuSignals &s) = 0;

            // Engine attached / ECU reset
            virtual void reset(EcuSignals &s) { /* void */ }

            // cal: calibration snapshot; dt: time since the previous run of
            // this function [s]
            virtual void run(const float *cal, EcuSignals &s, double dt) = 0;

        private:
            const char *m_name;
            Task m_task;
    };
}

#endif /* ECU_FUNCTION_H */
