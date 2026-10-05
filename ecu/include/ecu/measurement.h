#ifndef ECU_MEASUREMENT_H
#define ECU_MEASUREMENT_H

#include <string>
#include <vector>

namespace ecu {
    // ECU variables visible from outside (datalog now, XCP DAQ later). Each
    // function registers the signals it wants to expose.
    struct MeasurementInfo {
        std::string name;
        std::string unit;
        std::string description;
        std::string function;
        const double *value = nullptr;
    };

    class MeasurementTable {
        public:
            void setFunction(const std::string &function) { m_function = function; }

            void add(
                const std::string &name,
                const std::string &unit,
                const std::string &description,
                const double *value)
            {
                m_measurements.push_back({ name, unit, description, m_function, value });
            }

            const std::vector<MeasurementInfo> &list() const { return m_measurements; }

            const MeasurementInfo *find(const std::string &name) const {
                for (const MeasurementInfo &m : m_measurements) {
                    if (m.name == name) return &m;
                }
                return nullptr;
            }

        private:
            std::string m_function;
            std::vector<MeasurementInfo> m_measurements;
    };
}

#endif /* ECU_MEASUREMENT_H */
