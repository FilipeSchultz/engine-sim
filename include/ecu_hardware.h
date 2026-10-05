#ifndef ATG_ENGINE_SIM_ECU_HARDWARE_H
#define ATG_ENGINE_SIM_ECU_HARDWARE_H

#include "ecu/ecu.h"
#include "ecu/link.h"

#include <string>

class Engine;
class EcuLearner;

// The ECU's electrical environment in the simulation, as on a HIL bench:
// sensor models produce ecu::EcuInputs from the engine, actuator models carry
// out ecu::EcuOutputs (port injectors, spark advance). This is the only code
// that touches both the ECU and the engine.
//
// Two ways to run the ECU:
//   - in this process (setEcu): stepped with the physics, as before
//   - external (openLink): the ECU is ecu-standalone, over UDP (ecu/link.h).
//     This class then plays the I/O controller, the microcontroller of a real
//     bench: it averages rpm and MAP over each firing interval, sends a
//     sample every 10 ms, fires the injectors at the commanded angle and cuts
//     the ECU outputs (engine script mixture and timing) when no command came
//     for 0.1 s.
class EcuHardware {
    public:
        struct LinkSettings {
            std::string address = "127.0.0.1";     // where ecu-standalone listens
            uint16_t port = ecu::link::DefaultPort;

            // Lockstep: after each sample the simulation waits (up to 50 ms)
            // for the command, so the external ECU behaves as one in the
            // process. Free: the simulation never waits and applies the last
            // command received (latency and losses as on a real bench).
            bool lockstep = true;
        };

        enum class LinkState {
            Off,            // no link (ECU in the process, or none)
            Waiting,        // no command received yet
            Connected,
            Lost            // commands stopped: fail-safe
        };

        void setEcu(ecu::Ecu *ecu) { m_ecu = ecu; m_attached = nullptr; }
        ecu::Ecu *getEcu() const { return m_ecu; }

        // Optional: records the engine script's behaviour into ECU maps (ECU
        // in the process only)
        void setLearner(EcuLearner *learner) { m_learner = learner; }

        bool openLink(const LinkSettings &settings, std::string *error);
        bool linkOpen() const { return m_link.isOpen(); }
        LinkState linkState() const;
        const LinkSettings &linkSettings() const { return m_linkSettings; }

        // Codes the ECU for the engine (cylinders, firing TDCs) and resets
        // the sensors
        void attach(Engine *engine);

        // Once per physics step, before ignition and gas flow
        void step(Engine *engine, double dt);

        const ecu::EcuInputs &inputs() const { return m_inputs; }
        const ecu::EcuOutputs &outputs() const { return m_outputs; }

        // MAP averaged over the last firing interval, as the ECU sees it
        double averagedMap_kPa() const;

        // Sensor models
        double m_coolantTemperature_C = 90.0;       // no thermal model yet
        double m_lambdaTimeConstant_s = 0.1;        // wideband sensor response
        double m_iatTimeConstant_s = 2.0;           // intake air thermistor

    private:
        void readSensors(Engine *engine, double dt);
        void driveActuators(Engine *engine);

        // External ECU: the I/O controller's work
        void stepIoController(double dt);
        void averageInputs(double dt);
        void sendSample();
        void receiveCommands(int timeout_ms, bool untilAnswer);
        void scheduleInjections(double injection_ms, double injectionAngle_deg);

        ecu::Ecu *m_ecu = nullptr;
        EcuLearner *m_learner = nullptr;
        Engine *m_attached = nullptr;
        ecu::EcuConfig m_config;

        ecu::EcuInputs m_inputs;
        ecu::EcuOutputs m_outputs;

        double m_time = 0;
        double m_lambda = 0;
        double m_iat_C = 25;
        bool m_iatValid = false;

        // External ECU
        ecu::link::UdpEndpoint m_link;
        LinkSettings m_linkSettings;
        ecu::link::Command m_command;
        bool m_commandValid = false;        // a command received since the attach
        double m_commandAge = 0;            // simulated time since the last command [s]
        int m_unanswered = 0;               // samples in a row without an answer
        bool m_failsafe = false;
        uint8_t m_counter = 0;
        double m_sampleTimer = 0;
        double m_pollTimer = 0;
        uint32_t m_cycles = 0;

        // Averages over the last firing interval
        double m_avgAngle = 0;
        double m_avgTime = 0;
        double m_avgMapTime = 0;
        double m_avgRpm = 0;
        double m_avgMap_kPa = 0;
        bool m_avgValid = false;

        // Injection output stage
        double m_lastCycleAngle = 0;
        bool m_angleValid = false;
};

#endif /* ATG_ENGINE_SIM_ECU_HARDWARE_H */
