#ifndef ATG_ENGINE_SIM_ECU_HARDWARE_H
#define ATG_ENGINE_SIM_ECU_HARDWARE_H

#include "ecu/io.h"
#include "ecu/link.h"

#include <deque>
#include <string>

class Engine;

// The ECU's I/O controller, the microcontroller of a real bench, played by
// the simulation. The ECU is ecu-standalone, reached over UDP (ecu/link.h).
// Sensor models read the engine; every 10 ms a sample goes to the ECU, with
// rpm and MAP averaged over the last firing interval; the ECU's command is
// carried out: injectors fired at the commanded angle (the engine's own
// injectors), spark advance. Without a command for 0.1 s the ECU outputs are
// cut and the engine script's mixture and timing take over (fail-safe).
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
            Off,            // link not open
            Waiting,        // no command received yet
            Connected,
            Lost            // commands stopped: fail-safe
        };

        bool openLink(const LinkSettings &settings, std::string *error);
        bool linkOpen() const { return m_link.isOpen(); }
        LinkState linkState() const;
        const LinkSettings &linkSettings() const { return m_linkSettings; }

        // Takes the engine's cylinders and firing TDCs, resets the sensors
        // and the link state
        void attach(Engine *engine);

        // Once per physics step, before ignition and gas flow
        void step(Engine *engine, double dt);

        const ecu::EcuInputs &inputs() const { return m_inputs; }
        const ecu::EcuOutputs &outputs() const { return m_outputs; }

        // MAP averaged over the last firing interval, as the ECU sees it
        double averagedMap_kPa() const;

        // Sensor models (coolant and lambda sensor: engine script)
        double m_iatTimeConstant_s = 2.0;           // intake air thermistor

    private:
        void readSensors(Engine *engine, double dt);
        void readLambdaSensor(Engine *engine, double dt);
        void driveActuators(Engine *engine);

        void stepIoController(double dt);
        void averageInputs(double dt);
        void sendSample();
        void receiveCommands(int timeout_ms, bool untilAnswer);
        void scheduleInjections(double injection_ms, double injectionAngle_deg);

        Engine *m_attached = nullptr;
        ecu::EcuConfig m_config;

        ecu::EcuInputs m_inputs;
        ecu::EcuOutputs m_outputs;

        double m_time = 0;
        double m_iat_C = 25;
        bool m_iatValid = false;

        // Wideband lambda: exhaust gas on its way from the valves to the
        // sensor (plug flow), tagged with the charge it was burned from
        struct ExhaustParcel {
            double volume;      // exhaust volume pushed out when it left [m3]
            double o2;          // O2 and fuel of its charge [mol]
            double fuel;
        };
        std::deque<ExhaustParcel> m_exhaustParcels;
        double m_exhaustVolume = 0;         // exhaust volume pushed out so far [m3]
        double m_sensorGas = 0;             // gas at the sensor, as 1 - 1/lambda
        double m_sensorSignal = 0;          // sensor response, same scale
        bool m_sensorReady = false;         // exhaust gas has reached the sensor

        // Link
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
