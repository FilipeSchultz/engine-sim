#ifndef ATG_ENGINE_SIM_DYNAMOMETER_H
#define ATG_ENGINE_SIM_DYNAMOMETER_H

#include "scs.h"

#include "crankshaft.h"

class Dynamometer : public atg_scs::Constraint {
    public:
        Dynamometer();
        virtual ~Dynamometer();

        void connectCrankshaft(Crankshaft *crankshaft);
        virtual void calculate(Output *output, atg_scs::SystemState *state);
        double getTorque() const;

        double m_rotationSpeed;
        double m_ks;
        double m_kd;
        double m_maxTorque;

        // Torque mode: brakes the crankshaft with a constant torque
        // (m_torqueSetpoint) instead of holding m_rotationSpeed
        double m_torqueSetpoint;
        bool m_torqueMode;

        bool m_hold;
        bool m_enabled;
};

#endif /* ATG_ENGINE_SIM_DYNAMOMETER_H */
