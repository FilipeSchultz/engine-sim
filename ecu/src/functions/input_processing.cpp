#include "ecu/functions.h"

#include <algorithm>

void ecu::InputProcessing::declare(CalibrationLayout &cal, MeasurementTable &meas, EcuSignals &s) {
    meas.add("rpm", "rpm", "Engine speed, averaged over the last firing interval", &s.rpm);
    meas.add("map_kPa", "kPa", "Manifold pressure, averaged over the last firing interval", &s.map_kPa);
    meas.add("rpm_instant", "rpm", "Engine speed, instantaneous", &s.rpmInstant);
    meas.add("map_instant_kPa", "kPa", "Manifold pressure, instantaneous", &s.mapInstant_kPa);
    meas.add("iat_C", "degC", "Intake air temperature", &s.iat_C);
    meas.add("clt_C", "degC", "Coolant temperature", &s.clt_C);
    meas.add("tps_percent", "%", "Throttle position", &s.tps_percent);
    meas.add("lambda_sensor", "-", "Wideband lambda sensor (0 = not ready)", &s.lambdaSensor);
}

void ecu::InputProcessing::reset(EcuSignals &s) {
    m_angle = m_mapTime = m_time = 0;
    m_valid = false;
}

void ecu::InputProcessing::run(const float *cal, EcuSignals &s, double dt) {
    // Remote I/O: the I/O controller already averaged them, synchronously
    // with the crank
    if (s.remoteIo >= 0.5) {
        s.rpm = s.rpmInstant;
        s.map_kPa = s.mapInstant_kPa;
        return;
    }

    // Until the first interval is complete (or with the engine stopped) the
    // instantaneous values are used
    if (s.rpmInstant < 30.0) {
        reset(s);
    }
    else {
        m_angle += s.rpmInstant * 6.0 * dt;
        m_mapTime += s.mapInstant_kPa * dt;
        m_time += dt;

        const double interval = 720.0 / std::max(s.cylinders, 1.0);
        if (m_angle >= interval) {
            s.map_kPa = m_mapTime / m_time;
            s.rpm = m_angle / (6.0 * m_time);
            m_valid = true;
            m_angle = m_mapTime = m_time = 0;
        }
    }

    if (!m_valid) {
        s.rpm = s.rpmInstant;
        s.map_kPa = s.mapInstant_kPa;
    }
}
