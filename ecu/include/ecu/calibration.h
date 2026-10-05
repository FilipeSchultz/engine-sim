#ifndef ECU_CALIBRATION_H
#define ECU_CALIBRATION_H

#include <cstddef>
#include <mutex>
#include <string>
#include <vector>

namespace ecu {
    // Calibration memory: one flat block of floats. Each parameter declared by
    // an ECU function gets a fixed offset in it (axes first, then values), so
    // the block can be addressed by XCP and described in an A2L file.
    using CalibrationPage = std::vector<float>;

    // Size of the calibration memory [floats]. It lives in static storage
    // (fixed address inside the executable, as XCP absolute addressing needs)
    constexpr size_t CalibrationCapacity = 8192;

    struct ParameterInfo {
        enum class Type {
            Value,      // DCM FESTWERT
            Curve,      // DCM KENNLINIE: values over an X axis
            Map         // DCM KENNFELD: values over X and Y axes
        };

        std::string name;
        std::string description;
        std::string function;   // ECU function that owns it
        Type type = Type::Value;

        std::string unit;
        float min = 0, max = 0;

        int sizeX = 1;
        int sizeY = 1;
        std::string unitX, unitY;
        float minX = 0, maxX = 0;
        float minY = 0, maxY = 0;

        // Measurements giving the operating point on each axis (shown on the
        // table by the calibration tool)
        std::string inputX, inputY;

        // Offsets in floats from the start of the page
        size_t offset = 0;
        size_t offsetX = 0;
        size_t offsetY = 0;

        int count() const { return sizeX * sizeY; }
    };

    struct CellPosition {
        int x = 0;      // index of the nearest X breakpoint
        int y = 0;      // index of the nearest Y breakpoint
    };

    // Finds the segment [axis[*index], axis[*index + 1]] containing v (clamped
    // to the axis ends) and the position t in [0, 1] inside it
    void locateOnAxis(const float *axis, int size, float v, int *index, float *t);

    // Handles returned when declaring parameters; they read the page passed in
    class Value {
        public:
            float operator()(const float *page) const { return page[m_offset]; }
            float operator()(const CalibrationPage &page) const { return page[m_offset]; }
            size_t m_offset = 0;
    };

    class Curve {
        public:
            float lookup(const float *page, float x) const;
            float lookup(const CalibrationPage &page, float x) const { return lookup(page.data(), x); }
            size_t m_offset = 0, m_offsetX = 0;
            int m_size = 0;
    };

    class Map {
        public:
            float lookup(const float *page, float x, float y, CellPosition *cell = nullptr) const;
            float lookup(const CalibrationPage &page, float x, float y, CellPosition *cell = nullptr) const {
                return lookup(page.data(), x, y, cell);
            }
            size_t m_offset = 0, m_offsetX = 0, m_offsetY = 0;
            int m_sizeX = 0, m_sizeY = 0;
    };

    struct AxisSpec {
        std::string unit;
        float min, max;
        std::vector<float> breakpoints;     // default breakpoints
        std::string input;                  // measurement giving the operating point
    };

    // Parameter declarations of all ECU functions, with their defaults
    class CalibrationLayout {
        public:
            void setFunction(const std::string &function) { m_function = function; }

            Value addValue(
                const std::string &name, const std::string &description,
                const std::string &unit, float min, float max, float value);

            Curve addCurve(
                const std::string &name, const std::string &description,
                const std::string &unit, float min, float max,
                const AxisSpec &x, const std::vector<float> &values);

            // values: row by row (one row per Y breakpoint); a single value
            // fills the whole map
            Map addMap(
                const std::string &name, const std::string &description,
                const std::string &unit, float min, float max,
                const AxisSpec &x, const AxisSpec &y, const std::vector<float> &values);

            const std::vector<ParameterInfo> &parameters() const { return m_parameters; }
            const ParameterInfo *find(const std::string &name) const;
            const CalibrationPage &defaults() const { return m_defaults; }

            // Axes strictly increasing, everything finite and within limits
            bool validate(const CalibrationPage &page, std::string *error) const;

        private:
            size_t allocate(const std::vector<float> &values);

            std::string m_function;
            std::vector<ParameterInfo> m_parameters;
            CalibrationPage m_defaults;
    };

    // Working page shared between the calibration sources (file reload, XCP)
    // and the ECU. The working page is static memory with a fixed address
    // (the page XCP addresses); the ECU only reads a snapshot taken at the
    // start of each frame, so a table is never read half-written.
    class Calibration {
        public:
            // memory: static storage of at least page.size() floats
            void initialize(float *memory, const CalibrationPage &page);

            CalibrationPage getWorkingPage() const;
            void setWorkingPage(const CalibrationPage &page);

            const float *workingMemory() const { return m_working; }
            size_t size() const { return m_size; }

            // Copies a page of size() floats into the working memory (XCP:
            // the tool's active page)
            void mirror(const float *page);

            void updateSnapshot();
            const CalibrationPage &snapshot() const { return m_snapshot; }

        private:
            mutable std::mutex m_mutex;
            float *m_working = nullptr;
            size_t m_size = 0;
            CalibrationPage m_snapshot;
    };
}

#endif /* ECU_CALIBRATION_H */
