#include "../include/eddy_current_brake.h"

#include "../include/dynamometer.h"
#include "../include/constants.h"

#include <algorithm>
#include <cmath>

namespace {
    constexpr double DelaySample = 0.001;      // [s] resolution of the PWM delay line

    // Excitation torque [N m] at 3000 rpm vs duty: the bench's PWM sweeps
    // (0..4095) at 2500 and 3200 rpm minus the drag, interpolated to 3000 rpm
    // (3750 and above extrapolated)
    constexpr int MapSize = 17;
    constexpr double MapDuty[MapSize] = {
        0.0 / 4095, 250.0 / 4095, 500.0 / 4095, 750.0 / 4095, 1000.0 / 4095,
        1250.0 / 4095, 1500.0 / 4095, 1750.0 / 4095, 2000.0 / 4095, 2250.0 / 4095,
        2500.0 / 4095, 2750.0 / 4095, 3000.0 / 4095, 3250.0 / 4095, 3500.0 / 4095,
        3750.0 / 4095, 1.0 };
    constexpr double MapTorque[MapSize] = {
        0.0, 0.0127, 0.1069, 0.3086, 0.6547,
        1.1824, 1.8931, 2.7809, 3.7915, 4.9710,
        6.2330, 7.6061, 9.0054, 10.4506, 11.8537,
        13.189, 15.03 };

    // Speed factor: the data sheet's Md max curve (Zöllner type 1, digitised,
    // relative to 3000 rpm). Zero at standstill, about linear up to 1000 rpm
    constexpr int SpeedSize = 11;
    constexpr double SpeedRpm[SpeedSize] = {
        0, 500, 1000, 1500, 2000, 2500, 3000, 4000, 5000, 6000, 10000 };
    constexpr double SpeedMd[SpeedSize] = {
        0.0, 1.05, 2.0, 2.35, 2.6, 2.82, 3.0, 3.2, 3.35, 3.45, 3.6 };

    double interpolate(const double *x, const double *y, int n, double v) {
        if (v <= x[0]) return y[0];
        if (v >= x[n - 1]) return y[n - 1];
        for (int i = 1; i < n; ++i) {
            if (v <= x[i]) {
                const double s = (v - x[i - 1]) / (x[i] - x[i - 1]);
                return y[i - 1] + s * (y[i] - y[i - 1]);
            }
        }
        return y[n - 1];
    }

    double toRpm(double omega) { return std::abs(omega) * 60.0 / (2 * constants::pi); }
}

EddyCurrentBrake::EddyCurrentBrake() {
    m_rng.seed(std::random_device()());
    reset();
}

void EddyCurrentBrake::configure(const Parameters &params) {
    m_params = params;
    reset();
}

void EddyCurrentBrake::reset() {
    m_duty = m_externalDuty = 0;
    m_externalEnable = m_externalValid = m_external = m_internal = false;
    m_externalAge = m_controlTimer = m_integral = 0;
    m_handOver = false;

    m_delayLine.assign((size_t)std::lround(m_params.delay / DelaySample), 0.0);
    m_delayTimer = 0;
    m_x1 = m_x2 = 0;

    m_torque = m_power = 0;
    m_loadCell = m_measuredTorque = m_measuredSpeed = 0;
    m_measureTimer = m_angle = 0;
}

void EddyCurrentBrake::setExternalCommand(double duty, bool enable) {
    m_externalDuty = std::clamp(duty, 0.0, 1.0);
    m_externalEnable = enable;
    m_externalValid = true;
    m_externalAge = 0;
}

double EddyCurrentBrake::staticTorque(double duty, double rpm) const {
    rpm = std::abs(rpm);
    const double excitation = interpolate(MapDuty, MapTorque, MapSize, std::clamp(duty, 0.0, 1.0));
    const double speedFactor = interpolate(SpeedRpm, SpeedMd, SpeedSize, rpm) / 3.0;
    const double drag = m_params.residualTorque * std::sqrt(rpm / 3000.0);

    return m_params.torqueGain * excitation * speedFactor + drag;
}

double EddyCurrentBrake::dutyForLoad(double load) const {
    // The speed factor multiplies the whole map: the inverse at 3000 rpm
    // holds at every speed
    const double excitation = std::clamp(load, 0.0, 1.0) * MapTorque[MapSize - 1];
    return interpolate(MapTorque, MapDuty, MapSize, excitation);
}

double EddyCurrentBrake::loadForDuty(double duty) const {
    return interpolate(MapDuty, MapTorque, MapSize, std::clamp(duty, 0.0, 1.0)) / MapTorque[MapSize - 1];
}

double EddyCurrentBrake::dutyForTorque(double torque, double rpm) const {
    // Bisection on the (increasing) static characteristic
    double low = 0.0, high = 1.0;
    if (torque <= staticTorque(low, rpm)) return low;
    if (torque >= staticTorque(high, rpm)) return high;
    for (int i = 0; i < 30; ++i) {
        const double mid = 0.5 * (low + high);
        if (staticTorque(mid, rpm) < torque) low = mid;
        else high = mid;
    }
    return 0.5 * (low + high);
}

double EddyCurrentBrake::step(double dt, double omega, const Dynamometer &command) {
    // PWM duty: the external controller while its commands keep coming,
    // else the internal one when the dyno is enabled, else coil off
    m_externalAge += dt;
    m_external = m_externalValid && m_externalAge <= m_params.externalTimeout;
    m_internal = !m_external && command.m_enabled;
    if (m_external) {
        m_duty = m_externalEnable ? m_externalDuty : 0.0;
        m_handOver = true;          // bumpless hand-over to the internal controller
        m_controlTimer = 0;
    }
    else if (m_internal) {
        m_controlTimer += dt;
        if (m_controlTimer >= m_params.controlPeriod) {
            control(command, m_controlTimer);
            m_controlTimer = 0;
        }
    }
    else {
        m_duty = m_integral = m_controlTimer = 0;
        m_handOver = false;
    }

    // Coil current: delay, then the two lags of the magnetic circuit
    m_delayTimer += dt;
    while (m_delayTimer >= DelaySample) {
        m_delayTimer -= DelaySample;
        m_delayLine.push_back(m_duty);
        m_delayLine.pop_front();
    }
    const double delayed = m_delayLine.empty() ? m_duty : m_delayLine.front();
    m_x1 += (delayed - m_x1) * dt / (m_params.timeConstant1 + dt);
    m_x2 += (m_x1 - m_x2) * dt / (m_params.timeConstant2 + dt);

    m_torque = staticTorque(m_x2, toRpm(omega));
    m_power = m_torque * std::abs(omega);

    measure(dt, omega);
    return m_torque;
}

void EddyCurrentBrake::control(const Dynamometer &command, double period) {
    double feedForward, error, kp, ki;
    if (command.m_torqueMode) {
        // In coil current: the loop is then the coil's linear lag, whatever
        // the operating point (torque ~ current^2, speed factor)
        const double rpm = m_measuredSpeed;
        feedForward = dutyForTorque(std::fmax(command.m_torqueSetpoint, 0.0), rpm);
        error = feedForward - dutyForTorque(m_measuredTorque, rpm);
        kp = m_params.torqueKp;
        ki = m_params.torqueKp / m_params.torqueTi;
    }
    else {
        // Hold speed: running faster than the setpoint needs more braking
        feedForward = 0;
        error = m_measuredSpeed - toRpm(command.m_rotationSpeed);
        kp = m_params.speedKp;
        ki = m_params.speedKi;
    }

    // Taking over from the external controller: start from its duty
    if (m_handOver) {
        m_integral = m_duty - feedForward - kp * error;
        m_handOver = false;
    }

    const double integral = std::clamp(m_integral + ki * error * period, -1.0, 1.0);
    const double duty = feedForward + kp * error + integral;

    // Anti-windup: stop integrating while the output is saturated
    const bool saturated = (duty > 1.0 && error > 0) || (duty < 0.0 && error < 0);
    if (!saturated) m_integral = integral;

    m_duty = std::clamp(feedForward + kp * error + m_integral, 0.0, 1.0);
}

void EddyCurrentBrake::measure(double dt, double omega) {
    // Load cell on the housing: the electromagnetic torque (not the shaft
    // torque: the rotor inertia is in between), low-pass filtered
    m_loadCell += (m_torque - m_loadCell) * dt / (m_params.loadCellTimeConstant + dt);

    // Speed: whole pulses counted over the gate time
    m_angle += std::abs(omega) * dt;
    m_measureTimer += dt;
    if (m_measureTimer < m_params.measurementPeriod) return;

    const double pulseAngle = 2 * constants::pi / std::max(m_params.speedPulses, 1);
    const double pulses = std::floor(m_angle / pulseAngle);
    m_angle -= pulses * pulseAngle;
    m_measuredSpeed = pulses / std::max(m_params.speedPulses, 1) / m_measureTimer * 60.0;

    std::normal_distribution<double> noise(0.0, m_params.loadCellNoise);
    m_measuredTorque = m_loadCell + (m_params.loadCellNoise > 0 ? noise(m_rng) : 0.0);

    m_measureTimer = 0;
}
