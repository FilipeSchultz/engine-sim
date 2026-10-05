#ifndef ECU_STANDALONE_IO_LINK_H
#define ECU_STANDALONE_IO_LINK_H

#include "ecu/io.h"
#include "ecu/link.h"

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>

// Connection to the I/O controller, the microcontroller that runs the angle
// domain (crank and cam decoding, injector and coil timing, ADC). It sends a
// sample of the sensors every 10 ms, which paces the ECU, and receives the
// ECU's command in return.
class IoLink {
    public:
        enum class Receive {
            Sample,     // *inputs and *dt filled
            Timeout,    // nothing received yet: call again
            Closed      // the link is gone: stop
        };

        virtual ~IoLink() = default;

        virtual std::string describe() const = 0;

        // False when the inputs are generated on the PC (no engine)
        virtual bool hasController() const = 0;

        // Waits up to about 0.1 s for the next sample. dt: I/O controller time
        // since the previous sample [s]
        virtual Receive receive(ecu::EcuInputs *inputs, double *dt) = 0;

        virtual void send(const ecu::EcuOutputs &outputs) = 0;

        // Samples missing in the sequence (lost on the way)
        virtual uint64_t lostSamples() const { return 0; }
};

// I/O controller over UDP (ecu/link.h): the simulator playing the
// microcontroller, or a microcontroller with Ethernet. The ECU listens and
// answers whoever sends the samples.
class UdpLink : public IoLink {
    public:
        // false (and *error) if the port can't be opened
        bool open(const std::string &address, uint16_t port, std::string *error);

        virtual std::string describe() const override;
        virtual bool hasController() const override { return true; }
        virtual Receive receive(ecu::EcuInputs *inputs, double *dt) override;
        virtual void send(const ecu::EcuOutputs &outputs) override;
        virtual uint64_t lostSamples() const override { return m_lost; }

    private:
        ecu::link::UdpEndpoint m_socket;
        std::string m_address;
        uint16_t m_port = 0;

        ecu::link::Sample m_sample;
        bool m_started = false;
        uint32_t m_lastTime_us = 0;
        uint8_t m_lastCounter = 0;
        uint64_t m_lost = 0;
        double m_time = 0;
};

// No I/O controller: fixed sensor values, every 10 ms of PC time. For bench
// work on the calibration (maps, corrections, XCP) without hardware; the
// values can be changed while running.
class StimulusLink : public IoLink {
    public:
        StimulusLink();

        // "rpm=900,map=60": rpm, map (kPa), iat, clt (degC), tps (%), lambda,
        // battery (V); separated by commas or spaces. False (and *error) on
        // an unknown name or a bad value; nothing is changed then.
        bool set(const std::string &assignments, std::string *error);
        std::string values() const;

        virtual std::string describe() const override;
        virtual bool hasController() const override { return false; }
        virtual Receive receive(ecu::EcuInputs *inputs, double *dt) override;
        virtual void send(const ecu::EcuOutputs &outputs) override { /* void */ }

    private:
        static constexpr double Period_s = 0.010;

        mutable std::mutex m_mutex;
        ecu::EcuInputs m_inputs;

        std::chrono::steady_clock::time_point m_next;
        bool m_started = false;
        double m_time = 0;
};

#endif /* ECU_STANDALONE_IO_LINK_H */
