#ifndef ATG_ENGINE_SIM_UDP_SENDER_H
#define ATG_ENGINE_SIM_UDP_SENDER_H

#include <cstddef>
#include <cstdint>
#include <string>

// Minimal UDP datagram sender (Winsock), used to stream data to external tools
class UdpSender {
    public:
        UdpSender();
        ~UdpSender();

        // address as "host:port", e.g. "127.0.0.1:5005"
        bool open(const std::string &address, std::string *error);
        void close();
        bool isOpen() const { return m_open; }

        bool send(const void *data, size_t size);

    private:
        uintptr_t m_socket;
        bool m_open;
        unsigned char m_destination[16];    // sockaddr_in
};

#endif /* ATG_ENGINE_SIM_UDP_SENDER_H */
