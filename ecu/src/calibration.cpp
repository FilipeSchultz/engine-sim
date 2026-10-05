#include "ecu/calibration.h"

#include <algorithm>
#include <cmath>
#include <sstream>

void ecu::locateOnAxis(const float *axis, int size, float v, int *index, float *t) {
    if (size < 2) {
        *index = 0;
        *t = 0;
        return;
    }

    const float c = std::fmin(std::fmax(v, axis[0]), axis[size - 1]);

    int i = 0;
    while (i < size - 2 && c > axis[i + 1]) ++i;

    *index = i;
    *t = (c - axis[i]) / (axis[i + 1] - axis[i]);
}

float ecu::Curve::lookup(const float *page, float x) const {
    if (m_size == 1) return page[m_offset];

    int i;
    float t;
    locateOnAxis(&page[m_offsetX], m_size, x, &i, &t);

    const float *values = &page[m_offset];
    return values[i] * (1 - t) + values[i + 1] * t;
}

float ecu::Map::lookup(const float *page, float x, float y, CellPosition *cell) const {
    int i, j;
    float tx, ty;
    locateOnAxis(&page[m_offsetX], m_sizeX, x, &i, &tx);
    locateOnAxis(&page[m_offsetY], m_sizeY, y, &j, &ty);

    const float *values = &page[m_offset];
    const float v00 = values[j * m_sizeX + i];
    const float v10 = values[j * m_sizeX + i + 1];
    const float v01 = values[(j + 1) * m_sizeX + i];
    const float v11 = values[(j + 1) * m_sizeX + i + 1];

    if (cell != nullptr) {
        cell->x = (tx < 0.5f) ? i : i + 1;
        cell->y = (ty < 0.5f) ? j : j + 1;
    }

    return
        (v00 * (1 - tx) + v10 * tx) * (1 - ty)
        + (v01 * (1 - tx) + v11 * tx) * ty;
}

size_t ecu::CalibrationLayout::allocate(const std::vector<float> &values) {
    const size_t offset = m_defaults.size();
    m_defaults.insert(m_defaults.end(), values.begin(), values.end());
    return offset;
}

ecu::Value ecu::CalibrationLayout::addValue(
    const std::string &name, const std::string &description,
    const std::string &unit, float min, float max, float value)
{
    ParameterInfo p;
    p.name = name;
    p.description = description;
    p.function = m_function;
    p.type = ParameterInfo::Type::Value;
    p.unit = unit;
    p.min = min;
    p.max = max;
    p.offset = allocate({ value });
    m_parameters.push_back(p);

    Value handle;
    handle.m_offset = p.offset;
    return handle;
}

ecu::Curve ecu::CalibrationLayout::addCurve(
    const std::string &name, const std::string &description,
    const std::string &unit, float min, float max,
    const AxisSpec &x, const std::vector<float> &values)
{
    ParameterInfo p;
    p.name = name;
    p.description = description;
    p.function = m_function;
    p.type = ParameterInfo::Type::Curve;
    p.unit = unit;
    p.min = min;
    p.max = max;
    p.sizeX = (int)x.breakpoints.size();
    p.unitX = x.unit;
    p.minX = x.min;
    p.maxX = x.max;
    p.inputX = x.input;
    p.offsetX = allocate(x.breakpoints);
    p.offset = allocate(
        values.size() == 1 ? std::vector<float>(p.sizeX, values[0]) : values);
    m_parameters.push_back(p);

    Curve handle;
    handle.m_offset = p.offset;
    handle.m_offsetX = p.offsetX;
    handle.m_size = p.sizeX;
    return handle;
}

ecu::Map ecu::CalibrationLayout::addMap(
    const std::string &name, const std::string &description,
    const std::string &unit, float min, float max,
    const AxisSpec &x, const AxisSpec &y, const std::vector<float> &values)
{
    ParameterInfo p;
    p.name = name;
    p.description = description;
    p.function = m_function;
    p.type = ParameterInfo::Type::Map;
    p.unit = unit;
    p.min = min;
    p.max = max;
    p.sizeX = (int)x.breakpoints.size();
    p.sizeY = (int)y.breakpoints.size();
    p.unitX = x.unit;
    p.minX = x.min;
    p.maxX = x.max;
    p.unitY = y.unit;
    p.minY = y.min;
    p.maxY = y.max;
    p.inputX = x.input;
    p.inputY = y.input;
    p.offsetX = allocate(x.breakpoints);
    p.offsetY = allocate(y.breakpoints);
    p.offset = allocate(
        values.size() == 1 ? std::vector<float>(p.count(), values[0]) : values);
    m_parameters.push_back(p);

    Map handle;
    handle.m_offset = p.offset;
    handle.m_offsetX = p.offsetX;
    handle.m_offsetY = p.offsetY;
    handle.m_sizeX = p.sizeX;
    handle.m_sizeY = p.sizeY;
    return handle;
}

const ecu::ParameterInfo *ecu::CalibrationLayout::find(const std::string &name) const {
    for (const ParameterInfo &p : m_parameters) {
        if (p.name == name) return &p;
    }

    return nullptr;
}

bool ecu::CalibrationLayout::validate(const CalibrationPage &page, std::string *error) const {
    if (page.size() != m_defaults.size()) {
        *error = "calibration page size does not match the ECU";
        return false;
    }

    for (const ParameterInfo &p : m_parameters) {
        const float *values = &page[p.offset];
        for (int i = 0; i < p.count(); ++i) {
            if (!std::isfinite(values[i]) || values[i] < p.min || values[i] > p.max) {
                std::stringstream ss;
                ss << p.name << ": value " << values[i] << " out of range ["
                    << p.min << ", " << p.max << "]";
                *error = ss.str();
                return false;
            }
        }

        if (p.type == ParameterInfo::Type::Value) continue;

        struct AxisCheck { size_t offset; int size; float min, max; const char *name; };
        std::vector<AxisCheck> axes = { { p.offsetX, p.sizeX, p.minX, p.maxX, "X" } };
        if (p.type == ParameterInfo::Type::Map) {
            axes.push_back({ p.offsetY, p.sizeY, p.minY, p.maxY, "Y" });
        }

        for (const AxisCheck &a : axes) {
            const float *axis = &page[a.offset];
            for (int i = 0; i < a.size; ++i) {
                if (!std::isfinite(axis[i]) || axis[i] < a.min || axis[i] > a.max) {
                    std::stringstream ss;
                    ss << p.name << ": " << a.name << " axis value " << axis[i]
                        << " out of range [" << a.min << ", " << a.max << "]";
                    *error = ss.str();
                    return false;
                }

                if (i > 0 && axis[i] <= axis[i - 1]) {
                    std::stringstream ss;
                    ss << p.name << ": " << a.name << " axis must be strictly increasing ("
                        << axis[i - 1] << " -> " << axis[i] << ")";
                    *error = ss.str();
                    return false;
                }
            }
        }
    }

    return true;
}

void ecu::Calibration::initialize(float *memory, const CalibrationPage &page) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_working = memory;
    m_size = page.size();
    std::copy(page.begin(), page.end(), m_working);
    m_snapshot = page;
}

ecu::CalibrationPage ecu::Calibration::getWorkingPage() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return CalibrationPage(m_working, m_working + m_size);
}

void ecu::Calibration::setWorkingPage(const CalibrationPage &page) {
    std::lock_guard<std::mutex> lock(m_mutex);
    std::copy(page.begin(), page.begin() + std::min(page.size(), m_size), m_working);
}

void ecu::Calibration::mirror(const float *page) {
    std::lock_guard<std::mutex> lock(m_mutex);
    std::copy(page, page + m_size, m_working);
}

void ecu::Calibration::updateSnapshot() {
    std::lock_guard<std::mutex> lock(m_mutex);
    std::copy(m_working, m_working + m_size, m_snapshot.begin());
}
