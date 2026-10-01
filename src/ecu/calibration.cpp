#include "../../include/ecu/calibration.h"

#include <cmath>
#include <sstream>

namespace {
    ecu::ParameterInfo value(
        const char *name,
        const char *description,
        const char *unit,
        float min,
        float max,
        size_t offset)
    {
        ecu::ParameterInfo p;
        p.name = name;
        p.description = description;
        p.type = ecu::ParameterInfo::Type::Value;
        p.unit = unit;
        p.min = min;
        p.max = max;
        p.offset = offset;

        return p;
    }

    ecu::ParameterInfo map(
        const char *name,
        const char *description,
        const char *unit, float min, float max, size_t offset,
        const char *unitX, float minX, float maxX, size_t offsetX,
        const char *unitY, float minY, float maxY, size_t offsetY)
    {
        ecu::ParameterInfo p = value(name, description, unit, min, max, offset);
        p.type = ecu::ParameterInfo::Type::Map;
        p.sizeX = ecu::MapRpmPoints;
        p.sizeY = ecu::MapLoadPoints;
        p.unitX = unitX;
        p.minX = minX;
        p.maxX = maxX;
        p.offsetX = offsetX;
        p.unitY = unitY;
        p.minY = minY;
        p.maxY = maxY;
        p.offsetY = offsetY;

        return p;
    }

}

void ecu::locateOnAxis(const float *axis, int size, float v, int *index, float *t) {
    const float c = std::fmin(std::fmax(v, axis[0]), axis[size - 1]);

    int i = 0;
    while (i < size - 2 && c > axis[i + 1]) ++i;

    *index = i;
    *t = (c - axis[i]) / (axis[i + 1] - axis[i]);
}

const std::vector<ecu::ParameterInfo> &ecu::getParameters() {
    using P = CalibrationPage;

    static const std::vector<ParameterInfo> parameters = {
        value("EcuFuelEnable", "Fuel from FuelBaseMap (0 = engine script mixture)",
            "-", 0, 1, offsetof(P, EcuFuelEnable)),
        value("EcuIgnEnable", "Spark advance from IgnAdvanceMap (0 = engine script curve)",
            "-", 0, 1, offsetof(P, EcuIgnEnable)),
        value("InjectorFlow", "Injector static flow",
            "cc/min", 1, 20000, offsetof(P, InjectorFlow)),
        value("InjectorDeadTime", "Injector dead time",
            "ms", 0, 10, offsetof(P, InjectorDeadTime)),
        map("FuelBaseMap", "Base injection time",
            "ms", 0, 100, offsetof(P, FuelBaseMap),
            "rpm", 0, 20000, offsetof(P, FuelBaseMap_X),
            "kPa", 0, 400, offsetof(P, FuelBaseMap_Y)),
        map("IgnAdvanceMap", "Spark advance",
            "deg", -30, 70, offsetof(P, IgnAdvanceMap),
            "rpm", 0, 20000, offsetof(P, IgnAdvanceMap_X),
            "kPa", 0, 400, offsetof(P, IgnAdvanceMap_Y)),
        value("CrankingRpm", "Engine speed below which the engine is considered cranking",
            "rpm", 0, 2000, offsetof(P, CrankingRpm)),
        value("CrankingFuelFactor", "Injection time multiplier while cranking (0 = start on the script mixture)",
            "-", 0, 5, offsetof(P, CrankingFuelFactor)),
        value("AfterStartTime", "Time for the cranking factor to fade to 1 once the engine runs",
            "s", 0, 60, offsetof(P, AfterStartTime)),
    };

    return parameters;
}

const ecu::ParameterInfo *ecu::findParameter(const std::string &name) {
    for (const ParameterInfo &p : getParameters()) {
        if (name == p.name) return &p;
    }

    return nullptr;
}

ecu::CalibrationPage ecu::defaultCalibration() {
    static const float rpmAxis[MapRpmPoints] = {
        500, 750, 1000, 1250, 1500, 2000, 2500, 3000,
        3500, 4000, 4500, 5000, 5500, 6000, 6500, 7000 };
    // Closed throttle at high RPM pulls the manifold down to ~10 kPa
    static const float loadAxis[MapLoadPoints] = {
        10, 15, 20, 30, 40, 50, 60, 70, 80, 90, 100, 105 };

    CalibrationPage page = {};
    page.EcuFuelEnable = 0;
    page.EcuIgnEnable = 0;
    page.InjectorFlow = 440;
    page.InjectorDeadTime = 1.0f;
    page.CrankingRpm = 400;
    page.CrankingFuelFactor = 1.0f;
    page.AfterStartTime = 0.0f;

    for (int i = 0; i < MapRpmPoints; ++i) {
        page.FuelBaseMap_X[i] = page.IgnAdvanceMap_X[i] = rpmAxis[i];
    }

    for (int j = 0; j < MapLoadPoints; ++j) {
        page.FuelBaseMap_Y[j] = page.IgnAdvanceMap_Y[j] = loadAxis[j];

        for (int i = 0; i < MapRpmPoints; ++i) {
            page.FuelBaseMap[j][i] = 1.0f;
            page.IgnAdvanceMap[j][i] = 10.0f;
        }
    }

    return page;
}

bool ecu::validate(const CalibrationPage &page, std::string *error) {
    for (const ParameterInfo &p : getParameters()) {
        const float *values = parameterValues(&page, p);
        const int count = (p.type == ParameterInfo::Type::Map) ? p.sizeX * p.sizeY : 1;

        for (int i = 0; i < count; ++i) {
            if (!std::isfinite(values[i]) || values[i] < p.min || values[i] > p.max) {
                std::stringstream ss;
                ss << p.name << ": value " << values[i] << " out of range ["
                    << p.min << ", " << p.max << "]";
                *error = ss.str();
                return false;
            }
        }

        if (p.type != ParameterInfo::Type::Map) continue;

        const struct { const float *axis; int size; float min, max; const char *name; } axes[] = {
            { parameterAxis(&page, p.offsetX), p.sizeX, p.minX, p.maxX, "X" },
            { parameterAxis(&page, p.offsetY), p.sizeY, p.minY, p.maxY, "Y" }
        };

        for (const auto &a : axes) {
            for (int i = 0; i < a.size; ++i) {
                if (!std::isfinite(a.axis[i]) || a.axis[i] < a.min || a.axis[i] > a.max) {
                    std::stringstream ss;
                    ss << p.name << ": " << a.name << " axis value " << a.axis[i]
                        << " out of range [" << a.min << ", " << a.max << "]";
                    *error = ss.str();
                    return false;
                }

                if (i > 0 && a.axis[i] <= a.axis[i - 1]) {
                    std::stringstream ss;
                    ss << p.name << ": " << a.name << " axis must be strictly increasing ("
                        << a.axis[i - 1] << " -> " << a.axis[i] << ")";
                    *error = ss.str();
                    return false;
                }
            }
        }
    }

    return true;
}

float ecu::lookupMap(
    const float *axisX, int sizeX,
    const float *axisY, int sizeY,
    const float *values,
    float x, float y,
    CellPosition *cell)
{
    int i, j;
    float tx, ty;
    locateOnAxis(axisX, sizeX, x, &i, &tx);
    locateOnAxis(axisY, sizeY, y, &j, &ty);

    const float v00 = values[j * sizeX + i];
    const float v10 = values[j * sizeX + i + 1];
    const float v01 = values[(j + 1) * sizeX + i];
    const float v11 = values[(j + 1) * sizeX + i + 1];

    if (cell != nullptr) {
        cell->x = (tx < 0.5f) ? i : i + 1;
        cell->y = (ty < 0.5f) ? j : j + 1;
    }

    return
        (v00 * (1 - tx) + v10 * tx) * (1 - ty)
        + (v01 * (1 - tx) + v11 * tx) * ty;
}

ecu::Calibration::Calibration() {
    m_working = m_snapshot = defaultCalibration();
}

ecu::CalibrationPage ecu::Calibration::getWorkingPage() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_working;
}

void ecu::Calibration::setWorkingPage(const CalibrationPage &page) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_working = page;
}

void ecu::Calibration::updateSnapshot() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_snapshot = m_working;
}
