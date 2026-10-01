#ifndef ATG_ENGINE_SIM_ECU_CALIBRATION_H
#define ATG_ENGINE_SIM_ECU_CALIBRATION_H

#include <cstddef>
#include <mutex>
#include <string>
#include <vector>

namespace ecu {
    constexpr int MapRpmPoints = 16;
    constexpr int MapLoadPoints = 12;

    // Calibration memory with a fixed layout. Every parameter lives at a fixed
    // offset so that it can later be addressed by XCP (and described in an A2L
    // file). Dimensions never change at runtime; only values and axis
    // breakpoints do. Add new parameters at the end to keep existing offsets.
    struct CalibrationPage {
        float EcuFuelEnable;                                    // 0 = script mixture, 1 = FuelBaseMap
        float EcuIgnEnable;                                     // 0 = script timing curve, 1 = IgnAdvanceMap
        float InjectorFlow;                                     // [cc/min]
        float InjectorDeadTime;                                 // [ms]

        float FuelBaseMap_X[MapRpmPoints];                      // [rpm]
        float FuelBaseMap_Y[MapLoadPoints];                     // [kPa]
        float FuelBaseMap[MapLoadPoints][MapRpmPoints];         // [ms]

        float IgnAdvanceMap_X[MapRpmPoints];                    // [rpm]
        float IgnAdvanceMap_Y[MapLoadPoints];                   // [kPa]
        float IgnAdvanceMap[MapLoadPoints][MapRpmPoints];       // [deg BTDC]

        float CrankingRpm;                                      // [rpm] below this the engine is cranking
        float CrankingFuelFactor;                               // [-] injection time multiplier while cranking
        float AfterStartTime;                                   // [s] time for the cranking factor to fade to 1
    };

    // Description of one calibration parameter (name, dimensions, units,
    // limits and where it lives inside CalibrationPage)
    struct ParameterInfo {
        enum class Type {
            Value,
            Map
        };

        const char *name;
        const char *description;
        Type type;

        const char *unit;
        float min;
        float max;
        size_t offset;

        // Map only
        int sizeX = 0;
        int sizeY = 0;
        const char *unitX = "";
        const char *unitY = "";
        float minX = 0, maxX = 0;
        float minY = 0, maxY = 0;
        size_t offsetX = 0;
        size_t offsetY = 0;
    };

    const std::vector<ParameterInfo> &getParameters();
    const ParameterInfo *findParameter(const std::string &name);

    inline float *parameterValues(CalibrationPage *page, const ParameterInfo &p) {
        return reinterpret_cast<float *>(reinterpret_cast<char *>(page) + p.offset);
    }

    inline const float *parameterValues(const CalibrationPage *page, const ParameterInfo &p) {
        return reinterpret_cast<const float *>(reinterpret_cast<const char *>(page) + p.offset);
    }

    inline float *parameterAxis(CalibrationPage *page, size_t offset) {
        return reinterpret_cast<float *>(reinterpret_cast<char *>(page) + offset);
    }

    inline const float *parameterAxis(const CalibrationPage *page, size_t offset) {
        return reinterpret_cast<const float *>(reinterpret_cast<const char *>(page) + offset);
    }

    // Default page: ECU functions disabled, generic axes and neutral values
    CalibrationPage defaultCalibration();

    // Checks axes (strictly increasing, within limits) and values (within limits)
    bool validate(const CalibrationPage &page, std::string *error);

    struct CellPosition {
        int x = 0;      // index of the nearest X breakpoint
        int y = 0;      // index of the nearest Y breakpoint
    };

    // Finds the segment [axis[*index], axis[*index + 1]] containing v (clamped to
    // the axis ends) and the position t in [0, 1] inside it
    void locateOnAxis(const float *axis, int size, float v, int *index, float *t);

    // Bilinear interpolation, clamped to the axis ends
    float lookupMap(
        const float *axisX, int sizeX,
        const float *axisY, int sizeY,
        const float *values,
        float x, float y,
        CellPosition *cell = nullptr);

    // Working page shared between the calibration sources (file reload now,
    // XCP later) and the simulation. The simulation only reads a snapshot
    // taken at the start of each frame, so a table is never read half-written.
    class Calibration {
        public:
            Calibration();

            CalibrationPage getWorkingPage() const;
            void setWorkingPage(const CalibrationPage &page);

            void updateSnapshot();
            const CalibrationPage &snapshot() const { return m_snapshot; }

        private:
            mutable std::mutex m_mutex;
            CalibrationPage m_working;
            CalibrationPage m_snapshot;
    };
}

#endif /* ATG_ENGINE_SIM_ECU_CALIBRATION_H */
