#include "../include/dynamometer.h"

#include "../include/units.h"

#include <cmath>

Dynamometer::Dynamometer() : atg_scs::Constraint(1, 1) {
    m_rotationSpeed = 0.0;
    m_ks = 10.0;
    m_kd = 1.0;
    m_maxTorque = units::torque(10000.0, units::ft_lb);
    m_eddyCurrent = false;
    m_brakeTorque = 0.0;

    m_torqueSetpoint = 0.0;
    m_torqueMode = false;

    m_enabled = false;
    m_hold = false;
}

Dynamometer::~Dynamometer() {
    /* void */
}

void Dynamometer::connectCrankshaft(Crankshaft *crankshaft) {
    m_bodies[0] = &crankshaft->m_body;
}

void Dynamometer::calculate(Output *output, atg_scs::SystemState *state) {
    output->J[0][0] = 0;
    output->J[0][1] = 0;
    output->J[0][2] = 1;

    output->J_dot[0][0] = 0;
    output->J_dot[0][1] = 0;
    output->J_dot[0][2] = 0;

    output->ks[0] = m_ks;
    output->kd[0] = m_kd;

    output->C[0] = 0;

    if (m_eddyCurrent || m_torqueMode) {
        // Try to stop the shaft, but only with up to m_torqueSetpoint of braking
        // torque: while the engine keeps turning, the constraint stays saturated
        // at the limit, i.e. it applies a constant braking torque
        // (eddy current brake: the brake model's torque, coil on or off)
        const double brakingTorque = m_eddyCurrent
            ? std::fmin(std::fmax(m_brakeTorque, 0.0), m_maxTorque)
            : m_enabled
                ? std::fmin(std::fmax(m_torqueSetpoint, 0.0), m_maxTorque)
                : 0.0;

        output->v_bias[0] = 0;
        if (m_bodies[0]->v_theta < 0) {
            output->limits[0][0] = 0.0;
            output->limits[0][1] = brakingTorque;
        }
        else {
            output->limits[0][0] = -brakingTorque;
            output->limits[0][1] = 0.0;
        }

        return;
    }

    // Engines always run with a negative angular velocity (see the starter
    // motor). In hold mode the dyno can also drive the shaft, so it must always
    // target that direction: picking it from the current sign would spin the
    // engine backwards whenever the shaft crosses zero (stall, low demand).
    if (m_bodies[0]->v_theta < 0 || (m_hold && m_enabled)) {
        output->v_bias[0] = m_rotationSpeed;
        output->limits[0][0] = (m_hold && m_enabled) ? -m_maxTorque : 0.0;
        output->limits[0][1] = m_enabled ? m_maxTorque : 0.0;
    }
    else {
        output->v_bias[0] = -m_rotationSpeed;
        output->limits[0][0] = m_enabled ? -m_maxTorque : 0.0;
        output->limits[0][1] = (m_hold && m_enabled) ? m_maxTorque : 0.0;
    }
}

double Dynamometer::getTorque() const {
    return (m_bodies[0]->v_theta > 0)
        ? -atg_scs::Constraint::F_t[0][0]
        : atg_scs::Constraint::F_t[0][0];
}
