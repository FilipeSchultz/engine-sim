#ifndef ATG_ENGINE_SIM_EDDY_CURRENT_BRAKE_H
#define ATG_ENGINE_SIM_EDDY_CURRENT_BRAKE_H

#include <deque>
#include <random>

class Dynamometer;

// Eddy current brake of a small engine test bench (Zöllner type 1, water
// cooled, 6 PS), driven by a PWM coil current. It only absorbs torque: none
// at standstill, rising with speed (the data sheet's Md max curve) and about
// with the square of the coil current, with the slow response of the
// magnetic circuit; a residual drag remains with the coil off.
//
// Static map from the bench's PWM sweeps (0..4095) at 2500 / 3200 rpm, as
// measured (the bench reaches 13 N m at 2500 rpm below full PWM). Dynamics
// identified on the datalog torque0-3 (setpoint step with the bench PID
// active): one lag of 1.8 s on the coil current, the torque's S shape comes
// from the map. That log's PWM scale is uncertain: to be checked with an
// open loop PWM step on the bench.
//
// The PWM duty comes from an external controller (CAN, see dyno_can.h) or,
// without one, from an internal PI controller that follows the Dynamometer's
// commands (enabled, hold speed / torque setpoint), as the keyboard and the
// AVL EMCON remote set them. Also models the bench's measurements: load cell
// on the brake housing and a 720 pulse speed sensor.
class EddyCurrentBrake {
    public:
        struct Parameters {
            double torqueGain = 1.0;            // multiplies the excitation torque (coil voltage, other brake)
            double residualTorque = 0.21;       // [N m] drag at 3000 rpm with the coil off (sweeps)
            double delay = 0.0;                 // [s] PWM -> coil current
            double timeConstant1 = 1.8;         // [s] coil and magnetic circuit
            double timeConstant2 = 0.0;         // [s] second lag (0 = none)
            double maxPower = 4413.0;           // [W] 6 PS, water cooled
            double rotorInertia = 0.02;         // [kg m^2] (estimate)

            double loadCellTimeConstant = 0.05; // [s]
            double loadCellNoise = 0.007;       // [N m] standard deviation
            int speedPulses = 720;              // per revolution
            double measurementPeriod = 0.01;    // [s] speed gate and load cell sample

            double controlPeriod = 0.02;        // [s] internal controller
            // Torque: PI on coil current (set point and load cell through the
            // inverse map), the coil's set point as feed forward
            double torqueKp = 4.0;              // duty / duty
            double torqueTi = 1.8;              // [s] = the coil's time constant
            double speedKp = 0.00015;           // duty / rpm (brake too fast -> more duty)
            double speedKi = 0.00005;           // duty / (rpm s)

            double externalTimeout = 0.5;       // [s] without an external command: internal controller
        };

    public:
        EddyCurrentBrake();

        void configure(const Parameters &params);
        const Parameters &parameters() const { return m_params; }
        void reset();

        // Once per physics step: omega is the shaft speed [rad/s]; returns
        // the braking torque magnitude [N m]
        double step(double dt, double omega, const Dynamometer &command);

        // External controller (CAN): duty 0..1 and coil enable
        void setExternalCommand(double duty, bool enable);

        // Linearisation: duty giving `load` (0..1) of the excitation torque
        // at full duty, at any speed (torque = drag + load x maximum)
        double dutyForLoad(double load) const;
        double loadForDuty(double duty) const;

        double duty() const { return m_duty; }
        double excitation() const { return m_x2; }         // lagged duty (coil current / maximum)
        double torque() const { return m_torque; }          // [N m] true braking torque
        double power() const { return m_power; }            // [W]
        bool isOverPower() const { return m_power > m_params.maxPower; }
        bool isExternal() const { return m_external; }
        bool isExternalTimedOut() const { return m_externalValid && !m_external; }
        bool isInternalControl() const { return m_internal; }

        // Measurements, as the bench sees them
        double measuredTorque() const { return m_measuredTorque; }      // [N m] load cell
        double measuredSpeed_rpm() const { return m_measuredSpeed; }    // pulse count

        // Static characteristic: torque [N m] at duty (0..1) and speed [rpm],
        // and its inverse (duty for a torque, 0..1)
        double staticTorque(double duty, double rpm) const;
        double dutyForTorque(double torque, double rpm) const;

    private:
        void control(const Dynamometer &command, double period);
        void measure(double dt, double omega);

        Parameters m_params;

        // Command
        double m_duty = 0;
        double m_externalDuty = 0;
        bool m_externalEnable = false;
        bool m_externalValid = false;
        double m_externalAge = 0;
        bool m_external = false;
        bool m_internal = false;
        double m_controlTimer = 0;
        double m_integral = 0;
        bool m_handOver = false;

        // Coil: delay line (1 ms samples) and two lags
        std::deque<double> m_delayLine;
        double m_delayTimer = 0;
        double m_x1 = 0;
        double m_x2 = 0;

        double m_torque = 0;
        double m_power = 0;

        // Measurements
        double m_loadCell = 0;
        double m_measuredTorque = 0;
        double m_measuredSpeed = 0;
        double m_measureTimer = 0;
        double m_angle = 0;
        std::mt19937 m_rng;
};

#endif /* ATG_ENGINE_SIM_EDDY_CURRENT_BRAKE_H */
