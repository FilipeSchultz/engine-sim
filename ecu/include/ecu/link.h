#ifndef ECU_LINK_H
#define ECU_LINK_H

#include <cstddef>
#include <cstdint>
#include <string>

// Link between the ECU and its I/O controller, the microcontroller (or the
// simulator playing it) that runs the angle domain. The messages are CAN
// frames of 8 bytes, little endian, so the same layout runs on a CAN bus
// (ESP32) and, packed into datagrams, over UDP (simulator and ECU on PCs).
//
// Every 10 ms the I/O controller sends one sample: Time, Sensors2, then
// Sensors, which completes it. The ECU answers each sample with a Command
// carrying the sample's counter. The I/O controller applies the last command
// received and cuts injection and ignition when none came for too long.
//
//   0x100 Sensors   rpm        u16  0.25 rpm     (averaged over the last
//                   map        u16  0.01 kPa      firing interval)
//                   tps        u16  0.01 %
//                   status     u8   bit 0 crank synchronised,
//                                   bit 1 fail-safe active (outputs cut)
//                   counter    u8   +1 per sample
//   0x101 Sensors2  iat        i16  0.1 degC
//                   clt        i16  0.1 degC
//                   lambda     u16  0.0001 (0 = sensor not ready)
//                   battery    u16  0.001 V
//   0x102 Time      time       u32  1 us (I/O controller clock, wraps)
//                   cycles     u32  engine cycles since start
//   0x200 Command   injection  u16  1 us (dead time included, 0 = none)
//                   angle      u16  0.1 deg before the firing TDC
//                   advance    i16  0.1 deg before the firing TDC
//                   flags      u8   bit 0 fuel control, bit 1 spark control
//                   counter    u8   counter of the sample answered
namespace ecu {
    namespace link {
        constexpr uint32_t SensorsId = 0x100;
        constexpr uint32_t Sensors2Id = 0x101;
        constexpr uint32_t TimeId = 0x102;
        constexpr uint32_t CommandId = 0x200;

        constexpr uint16_t DefaultPort = 5600;

        struct Frame {
            uint32_t id = 0;
            uint8_t length = 8;
            uint8_t data[8] = {};
        };

        struct Sample {
            uint8_t counter = 0;
            bool synchronized = false;
            bool failsafe = false;
            uint32_t time_us = 0;
            uint32_t cycles = 0;

            double rpm = 0;
            double map_kPa = 0;
            double tps_percent = 0;
            double iat_C = 0;
            double clt_C = 0;
            double lambda = 0;
            double battery_V = 0;
        };

        struct Command {
            uint8_t counter = 0;
            bool fuelControl = false;
            bool sparkControl = false;
            double injection_ms = 0;
            double injectionAngle_deg = 0;
            double sparkAdvance_deg = 0;
        };

        // frames[0..2] = Time, Sensors2, Sensors (send in this order)
        void encodeSample(const Sample &sample, Frame frames[3]);

        // Fills the part of *sample the frame carries; true when the frame
        // completes the sample (Sensors). Other ids are ignored.
        bool decodeSample(const Frame &frame, Sample *sample);

        Frame encodeCommand(const Command &command);
        bool decodeCommand(const Frame &frame, Command *command);

        // Datagram: frames back to back, 13 bytes each (id u32 LE, length,
        // 8 data bytes)
        constexpr size_t PackedFrameSize = 13;
        constexpr int MaxFramesPerDatagram = 16;

        size_t pack(const Frame *frames, int count, uint8_t *buffer);
        int unpack(const uint8_t *buffer, size_t size, Frame *frames, int maxFrames);

        // UDP socket carrying frames. The peer is fixed (setPeer) or the
        // last sender (a server answering whoever talks to it).
        class UdpEndpoint {
            public:
                UdpEndpoint() = default;
                ~UdpEndpoint() { close(); }

                UdpEndpoint(const UdpEndpoint &) = delete;
                UdpEndpoint &operator=(const UdpEndpoint &) = delete;

                // address: local interface ("0.0.0.0" = all); port 0 = any
                bool open(const std::string &address, uint16_t port, std::string *error);
                void close();
                bool isOpen() const { return m_socket != InvalidSocket; }

                bool setPeer(const std::string &address, uint16_t port, std::string *error);
                bool hasPeer() const { return m_hasPeer; }

                bool send(const Frame *frames, int count);

                // Frames of the next datagram: count, 0 on timeout, -1 on
                // error. timeout_ms 0 = don't wait.
                int receive(Frame *frames, int maxFrames, int timeout_ms);

            private:
                static constexpr uintptr_t InvalidSocket = ~uintptr_t(0);

                uintptr_t m_socket = InvalidSocket;
                bool m_fixedPeer = false;
                bool m_hasPeer = false;
                uint8_t m_peer[16] = {};        // sockaddr_in
        };

        // "address:port", "address" or ":port" -> parts (defaults kept);
        // false if malformed
        bool parseEndpoint(const std::string &text, std::string *address, uint16_t *port);
    }
}

#endif /* ECU_LINK_H */
