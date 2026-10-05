#include "io_link.h"

bool UdpLink::open(const std::string &address, uint16_t port, std::string *error) {
    m_address = address;
    m_port = port;
    return m_socket.open(address, port, error);
}

std::string UdpLink::describe() const {
    return "UDP " + m_address + ":" + std::to_string(m_port);
}

IoLink::Receive UdpLink::receive(ecu::EcuInputs *inputs, double *dt) {
    using Clock = std::chrono::steady_clock;
    const Clock::time_point deadline = Clock::now() + std::chrono::milliseconds(100);

    ecu::link::Frame frames[ecu::link::MaxFramesPerDatagram];
    while (true) {
        const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now()).count();
        if (left <= 0) return Receive::Timeout;

        const int count = m_socket.receive(frames, ecu::link::MaxFramesPerDatagram, (int)left);
        if (count < 0) return Receive::Closed;

        bool complete = false;
        for (int i = 0; i < count; ++i) {
            if (ecu::link::decodeSample(frames[i], &m_sample)) {
                complete = true;
                break;
            }
        }
        if (complete) break;
    }

    // Time step from the I/O controller's clock (wraps every 71 min); a
    // restart of the controller or a gap starts over at one sample period
    constexpr double Period_s = 0.010;
    double step = Period_s;
    if (m_started) {
        step = (uint32_t)(m_sample.time_us - m_lastTime_us) * 1e-6;
        if (step <= 0 || step > 0.5) {
            step = Period_s;
        }
        else {
            m_lost += (uint8_t)(m_sample.counter - m_lastCounter - 1);
        }
    }
    m_started = true;
    m_lastTime_us = m_sample.time_us;
    m_lastCounter = m_sample.counter;
    m_time += step;

    *inputs = ecu::EcuInputs();
    inputs->time_s = m_time;
    inputs->rpm = m_sample.rpm;
    inputs->map_kPa = m_sample.map_kPa;
    inputs->tps_percent = m_sample.tps_percent;
    inputs->iat_C = m_sample.iat_C;
    inputs->clt_C = m_sample.clt_C;
    inputs->lambda = m_sample.lambda;
    inputs->battery_V = m_sample.battery_V;
    *dt = step;

    return Receive::Sample;
}

void UdpLink::send(const ecu::EcuOutputs &outputs) {
    ecu::link::Command command;
    command.counter = m_sample.counter;
    command.fuelControl = outputs.fuelControl;
    command.sparkControl = outputs.sparkControl;
    command.injection_ms = outputs.injection_ms;
    command.injectionAngle_deg = outputs.injectionAngle_deg;
    command.sparkAdvance_deg = outputs.sparkAdvance_deg;

    const ecu::link::Frame frame = ecu::link::encodeCommand(command);
    m_socket.send(&frame, 1);
}
