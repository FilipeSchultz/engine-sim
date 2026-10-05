#ifndef ECU_STANDALONE_STATUS_H
#define ECU_STANDALONE_STATUS_H

#include "ecu/io.h"
#include "ecu/signals.h"

#include <chrono>
#include <cstdint>
#include <ctime>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

// What the ECU program is doing: published by the ECU loop, shown by the
// status window
struct EcuStatus {
    using Clock = std::chrono::steady_clock;

    enum class State {
        Starting,
        Running,
        Stopped,
        Failed      // could not start (calibration, XCP...): see the messages
    };

    State state = State::Starting;
    std::string software;               // EPK
    Clock::time_point started;

    // I/O link
    std::string link;                   // description
    bool linkHasController = false;     // false: inputs generated here (stimulus)
    bool linkClosed = false;
    uint64_t samples = 0;
    Clock::time_point lastSample;
    double sampleRate_Hz = 0;
    uint64_t lostSamples = 0;

    // XCP
    bool xcpEnabled = false;
    bool xcpRunning = false;
    bool xcpConnected = false;
    std::string xcpEndpoint;

    // Calibration and datalog
    std::string calibrationFile;
    bool calibrationFileOk = true;
    bool fuelEnabled = false;
    bool ignitionEnabled = false;
    std::string logFile;

    // Last sample
    ecu::EcuSignals signals;
    ecu::EcuOutputs outputs;
};

// Shared between the ECU loop (writer) and the status window (reader)
class StatusBoard {
    public:
        EcuStatus snapshot() const {
            std::lock_guard<std::mutex> lock(m_mutex);
            return m_status;
        }

        template <typename Update>
        void update(Update update) {
            std::lock_guard<std::mutex> lock(m_mutex);
            update(m_status);
        }

        // Kept until taken, the last MaxMessages at most
        void addMessage(const std::string &text) {
            const std::time_t now = std::time(nullptr);
            char time[16];
            std::strftime(time, sizeof(time), "%H:%M:%S", std::localtime(&now));

            std::lock_guard<std::mutex> lock(m_mutex);
            m_messages.push_back(std::string(time) + "  " + text);
            if (m_messages.size() > MaxMessages) m_messages.pop_front();
        }

        std::vector<std::string> takeMessages() {
            std::lock_guard<std::mutex> lock(m_mutex);
            std::vector<std::string> messages(m_messages.begin(), m_messages.end());
            m_messages.clear();
            return messages;
        }

    private:
        static constexpr size_t MaxMessages = 200;

        mutable std::mutex m_mutex;
        EcuStatus m_status;
        std::deque<std::string> m_messages;
};

#endif /* ECU_STANDALONE_STATUS_H */
