#include "ecu/link.h"

#include "ecu/host.h"

#include <cmath>
#include <cstdlib>
#include <cstring>

#ifdef _WIN32
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#include <mstcpip.h>
#ifndef SIO_UDP_CONNRESET
#define SIO_UDP_CONNRESET _WSAIOW(IOC_VENDOR, 12)
#endif
using SocketHandle = SOCKET;
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
using SocketHandle = int;
#endif

namespace {
    // Scaled value, rounded and saturated to the field's range
    long scaled(double value, double resolution, long min, long max) {
        if (!std::isfinite(value)) return 0;
        const double raw = std::round(value / resolution);
        if (raw < min) return min;
        if (raw > max) return max;
        return (long)raw;
    }

    void putU16(uint8_t *p, long v) {
        p[0] = (uint8_t)(v & 0xFF);
        p[1] = (uint8_t)((v >> 8) & 0xFF);
    }

    void putU32(uint8_t *p, uint32_t v) {
        for (int i = 0; i < 4; ++i) p[i] = (uint8_t)((v >> (8 * i)) & 0xFF);
    }

    uint16_t getU16(const uint8_t *p) {
        return (uint16_t)(p[0] | (p[1] << 8));
    }

    int16_t getI16(const uint8_t *p) {
        return (int16_t)getU16(p);
    }

    uint32_t getU32(const uint8_t *p) {
        return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    }

    ecu::link::Frame frame(uint32_t id) {
        ecu::link::Frame f;
        f.id = id;
        f.length = 8;
        return f;
    }

#ifdef _WIN32
    bool startSockets() {
        static const bool started = []() {
            WSADATA data;
            return WSAStartup(MAKEWORD(2, 2), &data) == 0;
        }();
        return started;
    }

    void closeSocket(SocketHandle s) { closesocket(s); }
#else
    bool startSockets() { return true; }
    void closeSocket(SocketHandle s) { ::close(s); }
#endif

    bool toAddress(const std::string &address, uint16_t port, sockaddr_in *out) {
        uint8_t ip[4];
        if (!ecu::parseIpv4(address, ip)) return false;

        std::memset(out, 0, sizeof(*out));
        out->sin_family = AF_INET;
        out->sin_port = htons(port);
        std::memcpy(&out->sin_addr, ip, 4);
        return true;
    }
}

void ecu::link::encodeSample(const Sample &sample, Frame frames[3]) {
    Frame &time = frames[0] = frame(TimeId);
    putU32(time.data, sample.time_us);
    putU32(time.data + 4, sample.cycles);

    Frame &sensors2 = frames[1] = frame(Sensors2Id);
    putU16(sensors2.data, scaled(sample.iat_C, 0.1, -32768, 32767));
    putU16(sensors2.data + 2, scaled(sample.clt_C, 0.1, -32768, 32767));
    putU16(sensors2.data + 4, scaled(sample.lambda, 0.0001, 0, 65535));
    putU16(sensors2.data + 6, scaled(sample.battery_V, 0.001, 0, 65535));

    Frame &sensors = frames[2] = frame(SensorsId);
    putU16(sensors.data, scaled(sample.rpm, 0.25, 0, 65535));
    putU16(sensors.data + 2, scaled(sample.map_kPa, 0.01, 0, 65535));
    putU16(sensors.data + 4, scaled(sample.tps_percent, 0.01, 0, 65535));
    sensors.data[6] = (uint8_t)((sample.synchronized ? 0x01 : 0) | (sample.failsafe ? 0x02 : 0));
    sensors.data[7] = sample.counter;
}

bool ecu::link::decodeSample(const Frame &f, Sample *sample) {
    if (f.length < 8) return false;

    switch (f.id) {
        case TimeId:
            sample->time_us = getU32(f.data);
            sample->cycles = getU32(f.data + 4);
            return false;

        case Sensors2Id:
            sample->iat_C = getI16(f.data) * 0.1;
            sample->clt_C = getI16(f.data + 2) * 0.1;
            sample->lambda = getU16(f.data + 4) * 0.0001;
            sample->battery_V = getU16(f.data + 6) * 0.001;
            return false;

        case SensorsId:
            sample->rpm = getU16(f.data) * 0.25;
            sample->map_kPa = getU16(f.data + 2) * 0.01;
            sample->tps_percent = getU16(f.data + 4) * 0.01;
            sample->synchronized = (f.data[6] & 0x01) != 0;
            sample->failsafe = (f.data[6] & 0x02) != 0;
            sample->counter = f.data[7];
            return true;

        default:
            return false;
    }
}

ecu::link::Frame ecu::link::encodeCommand(const Command &command) {
    Frame f = frame(CommandId);
    putU16(f.data, scaled(command.injection_ms * 1000.0, 1.0, 0, 65535));
    putU16(f.data + 2, scaled(command.injectionAngle_deg, 0.1, 0, 65535));
    putU16(f.data + 4, scaled(command.sparkAdvance_deg, 0.1, -32768, 32767));
    f.data[6] = (uint8_t)((command.fuelControl ? 0x01 : 0) | (command.sparkControl ? 0x02 : 0));
    f.data[7] = command.counter;
    return f;
}

bool ecu::link::decodeCommand(const Frame &f, Command *command) {
    if (f.id != CommandId || f.length < 8) return false;

    command->injection_ms = getU16(f.data) / 1000.0;
    command->injectionAngle_deg = getU16(f.data + 2) * 0.1;
    command->sparkAdvance_deg = getI16(f.data + 4) * 0.1;
    command->fuelControl = (f.data[6] & 0x01) != 0;
    command->sparkControl = (f.data[6] & 0x02) != 0;
    command->counter = f.data[7];
    return true;
}

size_t ecu::link::pack(const Frame *frames, int count, uint8_t *buffer) {
    size_t size = 0;
    for (int i = 0; i < count; ++i) {
        putU32(buffer + size, frames[i].id);
        buffer[size + 4] = frames[i].length;
        std::memcpy(buffer + size + 5, frames[i].data, 8);
        size += PackedFrameSize;
    }
    return size;
}

int ecu::link::unpack(const uint8_t *buffer, size_t size, Frame *frames, int maxFrames) {
    int count = 0;
    for (size_t offset = 0; offset + PackedFrameSize <= size && count < maxFrames; offset += PackedFrameSize) {
        Frame &f = frames[count++];
        f.id = getU32(buffer + offset);
        f.length = buffer[offset + 4] > 8 ? 8 : buffer[offset + 4];
        std::memcpy(f.data, buffer + offset + 5, 8);
    }
    return count;
}

bool ecu::link::parseEndpoint(const std::string &text, std::string *address, uint16_t *port) {
    const size_t colon = text.find(':');
    const std::string host = text.substr(0, colon);
    if (!host.empty()) {
        uint8_t ip[4];
        if (!parseIpv4(host, ip)) return false;
        *address = host;
    }

    if (colon != std::string::npos) {
        const std::string number = text.substr(colon + 1);
        char *end = nullptr;
        const long value = std::strtol(number.c_str(), &end, 10);
        if (number.empty() || *end != '\0' || value < 1 || value > 65535) return false;
        *port = (uint16_t)value;
    }

    return true;
}

bool ecu::link::UdpEndpoint::open(const std::string &address, uint16_t port, std::string *error) {
    close();

    sockaddr_in local;
    if (!toAddress(address, port, &local)) {
        *error = "invalid address " + address;
        return false;
    }

    if (!startSockets()) {
        *error = "cannot start Windows sockets";
        return false;
    }

    const SocketHandle s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s == (SocketHandle)InvalidSocket) {
        *error = "cannot create a UDP socket";
        return false;
    }

    if (bind(s, reinterpret_cast<const sockaddr *>(&local), sizeof(local)) != 0) {
        closeSocket(s);
        *error = "cannot open UDP port " + std::to_string(port) + " on " + address + " (in use?)";
        return false;
    }

#ifdef _WIN32
    // A datagram to a port nobody listens on (the other side not started
    // yet) must not make the next receive fail with WSAECONNRESET
    BOOL report = FALSE;
    DWORD returned = 0;
    WSAIoctl(s, SIO_UDP_CONNRESET, &report, sizeof(report), nullptr, 0, &returned, nullptr, nullptr);
#endif

    m_socket = (uintptr_t)s;
    return true;
}

void ecu::link::UdpEndpoint::close() {
    if (m_socket != InvalidSocket) {
        closeSocket((SocketHandle)m_socket);
        m_socket = InvalidSocket;
    }
    m_hasPeer = m_fixedPeer = false;
}

bool ecu::link::UdpEndpoint::setPeer(const std::string &address, uint16_t port, std::string *error) {
    sockaddr_in peer;
    if (!toAddress(address, port, &peer)) {
        *error = "invalid address " + address;
        return false;
    }

    std::memcpy(m_peer, &peer, sizeof(peer));
    m_hasPeer = m_fixedPeer = true;
    return true;
}

bool ecu::link::UdpEndpoint::send(const Frame *frames, int count) {
    if (!isOpen() || !m_hasPeer || count <= 0 || count > MaxFramesPerDatagram) return false;

    uint8_t buffer[MaxFramesPerDatagram * PackedFrameSize];
    const size_t size = pack(frames, count, buffer);
    const int sent = sendto((SocketHandle)m_socket, reinterpret_cast<const char *>(buffer), (int)size, 0,
        reinterpret_cast<const sockaddr *>(m_peer), sizeof(sockaddr_in));
    return sent == (int)size;
}

int ecu::link::UdpEndpoint::receive(Frame *frames, int maxFrames, int timeout_ms) {
    if (!isOpen()) return -1;

    const SocketHandle s = (SocketHandle)m_socket;
    fd_set readable;
    FD_ZERO(&readable);
    FD_SET(s, &readable);
    timeval timeout;
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;

    const int ready = select((int)(s + 1), &readable, nullptr, nullptr, &timeout);
    if (ready < 0) return -1;
    if (ready == 0) return 0;

    uint8_t buffer[MaxFramesPerDatagram * PackedFrameSize];
    sockaddr_in sender;
#ifdef _WIN32
    int senderSize = sizeof(sender);
#else
    socklen_t senderSize = sizeof(sender);
#endif
    const int size = recvfrom(s, reinterpret_cast<char *>(buffer), sizeof(buffer), 0,
        reinterpret_cast<sockaddr *>(&sender), &senderSize);
    if (size <= 0) return 0;

    // Answer whoever talks to us, unless the peer is fixed
    if (!m_fixedPeer) {
        std::memcpy(m_peer, &sender, sizeof(sender));
        m_hasPeer = true;
    }

    return unpack(buffer, (size_t)size, frames, maxFrames);
}
