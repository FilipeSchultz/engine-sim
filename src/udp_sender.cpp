#include <winsock2.h>
#include <ws2tcpip.h>

#include "../include/udp_sender.h"

#include <cstring>

#pragma comment(lib, "ws2_32.lib")

static_assert(sizeof(sockaddr_in) <= 16, "sockaddr_in must fit m_destination");

UdpSender::UdpSender() {
    m_socket = INVALID_SOCKET;
    m_open = false;
    std::memset(m_destination, 0, sizeof(m_destination));
}

UdpSender::~UdpSender() {
    close();
}

bool UdpSender::open(const std::string &address, std::string *error) {
    close();

    const size_t colon = address.rfind(':');
    if (colon == std::string::npos) {
        *error = "address must be host:port (" + address + ")";
        return false;
    }

    const std::string host = address.substr(0, colon);
    const int port = std::atoi(address.substr(colon + 1).c_str());

    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        *error = "WSAStartup failed";
        return false;
    }

    sockaddr_in destination = {};
    destination.sin_family = AF_INET;
    destination.sin_port = htons(static_cast<u_short>(port));
    if (inet_pton(AF_INET, host.c_str(), &destination.sin_addr) != 1) {
        *error = "invalid IPv4 address " + host;
        WSACleanup();
        return false;
    }

    const SOCKET s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s == INVALID_SOCKET) {
        *error = "cannot create UDP socket";
        WSACleanup();
        return false;
    }

    m_socket = static_cast<uintptr_t>(s);
    std::memcpy(m_destination, &destination, sizeof(destination));
    m_open = true;

    return true;
}

void UdpSender::close() {
    if (!m_open) return;

    closesocket(static_cast<SOCKET>(m_socket));
    WSACleanup();
    m_socket = INVALID_SOCKET;
    m_open = false;
}

bool UdpSender::send(const void *data, size_t size) {
    if (!m_open) return false;

    const int sent = sendto(
        static_cast<SOCKET>(m_socket),
        static_cast<const char *>(data),
        static_cast<int>(size),
        0,
        reinterpret_cast<const sockaddr *>(m_destination),
        sizeof(sockaddr_in));

    return sent == static_cast<int>(size);
}
