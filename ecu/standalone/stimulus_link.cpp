#include "io_link.h"

#include <cstdio>
#include <cstdlib>
#include <thread>
#include <vector>

namespace {
    struct Field {
        const char *name;
        double ecu::EcuInputs::*value;
    };

    const Field Fields[] = {
        { "rpm", &ecu::EcuInputs::rpm },
        { "map", &ecu::EcuInputs::map_kPa },
        { "iat", &ecu::EcuInputs::iat_C },
        { "clt", &ecu::EcuInputs::clt_C },
        { "tps", &ecu::EcuInputs::tps_percent },
        { "lambda", &ecu::EcuInputs::lambda },
        { "battery", &ecu::EcuInputs::battery_V }
    };
}

StimulusLink::StimulusLink() {
    // Warm engine at idle
    m_inputs.rpm = 900;
    m_inputs.map_kPa = 40;
    m_inputs.iat_C = 25;
    m_inputs.clt_C = 90;
    m_inputs.tps_percent = 5;
    m_inputs.lambda = 1.0;
    m_inputs.battery_V = 14;
}

bool StimulusLink::set(const std::string &assignments, std::string *error) {
    ecu::EcuInputs inputs;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        inputs = m_inputs;
    }

    // Split on commas and blanks
    std::vector<std::string> items;
    std::string item;
    for (const char c : assignments + ",") {
        if (c == ',' || c == ' ' || c == '\t' || c == '\r' || c == '\n') {
            if (!item.empty()) items.push_back(item);
            item.clear();
        }
        else {
            item += c;
        }
    }

    for (const std::string &assignment : items) {
        const size_t equals = assignment.find('=');
        if (equals == std::string::npos) {
            *error = "expected name=value, got '" + assignment + "'";
            return false;
        }

        const std::string name = assignment.substr(0, equals);
        const std::string text = assignment.substr(equals + 1);
        char *end = nullptr;
        const double value = std::strtod(text.c_str(), &end);
        if (text.empty() || *end != '\0') {
            *error = "bad value for " + name + ": '" + text + "'";
            return false;
        }

        bool found = false;
        for (const Field &field : Fields) {
            if (name == field.name) {
                inputs.*field.value = value;
                found = true;
                break;
            }
        }

        if (!found) {
            *error = "unknown input '" + name + "' (rpm, map, iat, clt, tps, lambda, battery)";
            return false;
        }
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    m_inputs = inputs;
    return true;
}

std::string StimulusLink::values() const {
    ecu::EcuInputs inputs;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        inputs = m_inputs;
    }

    std::string text;
    for (const Field &field : Fields) {
        char item[48];
        std::snprintf(item, sizeof(item), "%s%s=%g", text.empty() ? "" : " ", field.name, inputs.*field.value);
        text += item;
    }
    return text;
}

std::string StimulusLink::describe() const {
    return "stimulus (fixed inputs, 10 ms PC clock)";
}

IoLink::Receive StimulusLink::receive(ecu::EcuInputs *inputs, double *dt) {
    using Clock = std::chrono::steady_clock;
    const auto period = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(Period_s));

    const Clock::time_point now = Clock::now();
    if (!m_started) {
        m_next = now;
        m_started = true;
    }
    else if (now - m_next > 10 * period) {
        // Far behind (debugger, sleep): restart the schedule instead of
        // catching up with a burst
        m_next = now;
    }
    else {
        std::this_thread::sleep_until(m_next);
    }
    m_next += period;

    m_time += Period_s;

    std::lock_guard<std::mutex> lock(m_mutex);
    *inputs = m_inputs;
    inputs->time_s = m_time;
    *dt = Period_s;

    return Receive::Sample;
}
