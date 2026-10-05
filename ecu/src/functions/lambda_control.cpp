#include "ecu/functions.h"

#include <algorithm>

void ecu::LambdaControl::declare(CalibrationLayout &cal, MeasurementTable &meas, EcuSignals &s) {
    m_enable = cal.addValue("LambdaCtrlEnable",
        "Closed loop lambda control (0 = off)",
        "-", 0, 1, 0);
    m_target = cal.addValue("LambdaTarget",
        "Lambda set point",
        "-", 0.5f, 2.0f, 1.0f);
    m_kp = cal.addValue("LambdaCtrlKp",
        "Proportional gain: fuel correction per unit of lambda error",
        "-", 0, 10, 0.2f);
    m_ki = cal.addValue("LambdaCtrlKi",
        "Integral gain: fuel correction per unit of lambda error and second",
        "1/s", 0, 20, 0.5f);
    m_limit = cal.addValue("LambdaCtrlLimit",
        "Maximum fuel correction of the controller (fraction)",
        "-", 0, 0.5f, 0.25f);
    m_startDelay = cal.addValue("LambdaCtrlStartDelay",
        "Time after start before the controller is enabled",
        "s", 0, 120, 3);

    meas.add("lambda_target", "-", "Lambda set point", &s.lambdaTarget);
    meas.add("lambda_ctrl_active", "-", "1 = closed loop lambda active", &s.lambdaControlActive);
    meas.add("fuel_corr_lambda", "-", "Fuel correction of the lambda controller", &s.fuelCorrLambda);
}

void ecu::LambdaControl::reset(EcuSignals &s) {
    m_integral = 0;
    s.fuelCorrLambda = 1;
    s.lambdaControlActive = 0;
}

void ecu::LambdaControl::run(const float *cal, EcuSignals &s, double dt) {
    s.lambdaTarget = m_target(cal);

    const bool active =
        m_enable(cal) >= 0.5f
        && s.fuelControl >= 0.5
        && s.fuelCut < 0.5
        && s.engineState == Running
        && s.runningTime_s >= m_startDelay(cal)
        && s.lambdaSensor > 0;

    if (!active) {
        reset(s);
        return;
    }

    // Lean (lambda above target) -> more fuel
    const double error = s.lambdaSensor - s.lambdaTarget;
    const double limit = m_limit(cal);

    m_integral = std::clamp(m_integral + m_ki(cal) * error * dt, -limit, limit);
    s.fuelCorrLambda = 1.0 + std::clamp(m_kp(cal) * error + m_integral, -limit, limit);
    s.lambdaControlActive = 1;
}
