#include "../include/can_interface.h"

#ifdef ATG_ENGINE_SIM_CAN_ENABLED
#include <canlib.h>
#include <timeapi.h>

#pragma comment(lib, "winmm.lib")
#endif /* ATG_ENGINE_SIM_CAN_ENABLED */

#include <algorithm>
#include <cstring>

#ifdef ATG_ENGINE_SIM_CAN_ENABLED
namespace {
    std::once_flag canlibInitialized;

    long toCanlibBitrate(int bitrate) {
        switch (bitrate) {
        case 1000000: return canBITRATE_1M;
        case 500000: return canBITRATE_500K;
        case 250000: return canBITRATE_250K;
        case 125000: return canBITRATE_125K;
        default: return 0;
        }
    }

    std::string canlibError(const char *function, canStatus status) {
        char text[128] = "";
        canGetErrorText(status, text, sizeof(text));
        return std::string(function) + ": " + text + " (" + std::to_string(status) + ")";
    }
}
#endif /* ATG_ENGINE_SIM_CAN_ENABLED */

CanInterface::CanInterface() {
    m_running = false;
    m_open = false;
    m_channel = -1;

    m_received = 0;
    m_transmitted = 0;
    m_rxDropped = 0;
    m_errors = 0;
}

CanInterface::~CanInterface() {
    close();
}

bool CanInterface::open(int channel, int bitrate) {
    close();

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_rxQueue.clear();
        m_txQueue.clear();
        m_cyclic.clear();
        m_lastError.clear();
    }

    m_received = 0;
    m_transmitted = 0;
    m_rxDropped = 0;
    m_errors = 0;

    m_channel = channel;
    m_running = true;

    // The worker opens the channel itself and reports back whether it succeeded
    std::promise<bool> opened;
    std::future<bool> result = opened.get_future();
    m_thread = std::thread(&CanInterface::worker, this, channel, bitrate, &opened);

    if (!result.get()) {
        m_running = false;
        m_thread.join();
        return false;
    }

    return true;
}

void CanInterface::close() {
    m_running = false;
    if (m_thread.joinable()) {
        m_thread.join();
    }

    m_open = false;
}

bool CanInterface::read(Frame *frame) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_rxQueue.empty()) return false;

    *frame = m_rxQueue.front();
    m_rxQueue.pop_front();

    return true;
}

bool CanInterface::send(const Frame &frame) {
    if (!m_open || frame.length > 8) return false;

    std::lock_guard<std::mutex> lock(m_mutex);
    m_txQueue.push_back(frame);

    return true;
}

bool CanInterface::setCyclic(const Frame &frame, int periodMs, BeforeSend beforeSend) {
    if (frame.length > 8 || periodMs <= 0) return false;

    std::lock_guard<std::mutex> lock(m_mutex);
    for (CyclicFrame &cyclic : m_cyclic) {
        if (cyclic.frame.id == frame.id && cyclic.frame.extended == frame.extended) {
            cyclic.frame = frame;
            cyclic.period = std::chrono::milliseconds(periodMs);
            if (beforeSend) cyclic.beforeSend = std::move(beforeSend);
            return true;
        }
    }

    CyclicFrame cyclic;
    cyclic.frame = frame;
    cyclic.period = std::chrono::milliseconds(periodMs);
    cyclic.next = std::chrono::steady_clock::now();
    cyclic.beforeSend = std::move(beforeSend);
    m_cyclic.push_back(cyclic);

    return true;
}

void CanInterface::removeCyclic(uint32_t id, bool extended) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_cyclic.erase(
        std::remove_if(m_cyclic.begin(), m_cyclic.end(),
            [id, extended](const CyclicFrame &cyclic) {
                return cyclic.frame.id == id && cyclic.frame.extended == extended;
            }),
        m_cyclic.end());
}

CanInterface::Statistics CanInterface::getStatistics() const {
    Statistics statistics;
    statistics.received = m_received;
    statistics.transmitted = m_transmitted;
    statistics.rxDropped = m_rxDropped;
    statistics.errors = m_errors;

    return statistics;
}

std::string CanInterface::getLastError() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_lastError;
}

void CanInterface::collectOutgoing(std::vector<Frame> *outgoing) {
    const auto now = std::chrono::steady_clock::now();

    std::lock_guard<std::mutex> lock(m_mutex);
    outgoing->insert(outgoing->end(), m_txQueue.begin(), m_txQueue.end());
    m_txQueue.clear();

    for (CyclicFrame &cyclic : m_cyclic) {
        if (now < cyclic.next) continue;

        outgoing->push_back(cyclic.frame);
        if (cyclic.beforeSend) cyclic.beforeSend(&outgoing->back());

        cyclic.next += cyclic.period;

        // Fell behind by more than a period (e.g. debugger pause): resynchronize
        if (cyclic.next < now) {
            cyclic.next = now + cyclic.period;
        }
    }
}

void CanInterface::pushReceived(const Frame &frame) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_rxQueue.size() >= MaxRxQueueSize) {
        m_rxQueue.pop_front();
        ++m_rxDropped;
    }

    m_rxQueue.push_back(frame);
    ++m_received;
}

void CanInterface::setError(const std::string &error) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_lastError = error;
}

#ifdef ATG_ENGINE_SIM_CAN_ENABLED
void CanInterface::worker(int channel, int bitrate, std::promise<bool> *opened) {
    std::call_once(canlibInitialized, canInitializeLibrary);

    const long canlibBitrate = toCanlibBitrate(bitrate);
    if (canlibBitrate == 0) {
        setError("Unsupported bitrate: " + std::to_string(bitrate));
        opened->set_value(false);
        return;
    }

    const CanHandle handle = canOpenChannel(channel, canOPEN_ACCEPT_VIRTUAL);
    if (handle < 0) {
        setError(canlibError("canOpenChannel", static_cast<canStatus>(handle)));
        opened->set_value(false);
        return;
    }

    canStatus status = canSetBusParams(handle, canlibBitrate, 0, 0, 0, 0, 0);
    if (status == canOK) {
        status = canBusOn(handle);
        if (status != canOK) setError(canlibError("canBusOn", status));
    }
    else {
        setError(canlibError("canSetBusParams", status));
    }

    if (status != canOK) {
        canClose(handle);
        opened->set_value(false);
        return;
    }

    m_open = true;
    opened->set_value(true);

    // Default Windows timer resolution (~15.6 ms) would turn the 1 ms wait
    // below into ~15 ms and add jitter to cyclic frames
    timeBeginPeriod(1);

    std::vector<Frame> outgoing;
    while (m_running) {
        long id = 0;
        uint8_t data[64] = {};
        unsigned int length = 0;
        unsigned int flags = 0;
        unsigned long time = 0;

        // Wait up to 1 ms for traffic, then drain everything already queued
        status = canReadWait(handle, &id, data, &length, &flags, &time, 1);
        while (status == canOK) {
            if ((flags & canMSG_ERROR_FRAME) != 0) {
                ++m_errors;
            }
            else if ((flags & canMSG_RTR) == 0) {
                Frame frame;
                frame.id = static_cast<uint32_t>(id);
                frame.extended = (flags & canMSG_EXT) != 0;
                frame.length = static_cast<uint8_t>((std::min)(length, 8u));
                frame.timestamp = std::chrono::steady_clock::now();
                std::memcpy(frame.data, data, frame.length);

                pushReceived(frame);
            }

            status = canRead(handle, &id, data, &length, &flags, &time);
        }

        if (status != canERR_NOMSG) {
            ++m_errors;
            setError(canlibError("canRead", status));
        }

        outgoing.clear();
        collectOutgoing(&outgoing);
        for (Frame &frame : outgoing) {
            status = canWrite(
                handle,
                static_cast<long>(frame.id),
                frame.data,
                frame.length,
                frame.extended ? canMSG_EXT : canMSG_STD);

            if (status == canOK) {
                ++m_transmitted;
            }
            else {
                ++m_errors;
                setError(canlibError("canWrite", status));
            }
        }
    }

    timeEndPeriod(1);

    canBusOff(handle);
    canClose(handle);
    m_open = false;
}
#else
void CanInterface::worker(int channel, int bitrate, std::promise<bool> *opened) {
    (void)channel;
    (void)bitrate;

    setError("CAN support not compiled in (configure with -DCAN_ENABLED=ON)");
    opened->set_value(false);
}
#endif /* ATG_ENGINE_SIM_CAN_ENABLED */
