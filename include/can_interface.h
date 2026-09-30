#ifndef ATG_ENGINE_SIM_CAN_INTERFACE_H
#define ATG_ENGINE_SIM_CAN_INTERFACE_H

#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <future>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// Thin wrapper around a Kvaser CANlib channel.
//
// All bus I/O happens on a dedicated worker thread (CANlib handles must not
// be shared between threads), so the application only exchanges frames
// through thread-safe queues. Cyclic frames are transmitted by the worker
// thread at a fixed period, independent of the render frame rate.
//
// Without ATG_ENGINE_SIM_CAN_ENABLED, open() always fails and the class
// does nothing.
class CanInterface {
    public:
        struct Frame {
            uint32_t id = 0;
            uint8_t data[8] = {};
            uint8_t length = 0;
            bool extended = false;
            std::chrono::steady_clock::time_point timestamp;
        };

        struct Statistics {
            uint64_t received = 0;
            uint64_t transmitted = 0;
            uint64_t rxDropped = 0;
            uint64_t errors = 0;
        };

        // Called on the worker thread with the copy about to be transmitted,
        // e.g. to insert a counter that must advance once per transmission
        using BeforeSend = std::function<void(Frame *frame)>;

        static constexpr size_t MaxRxQueueSize = 1024;

    public:
        CanInterface();
        ~CanInterface();

        // bitrate in bit/s: 125000, 250000, 500000 or 1000000
        bool open(int channel, int bitrate);
        void close();
        bool isOpen() const { return m_open; }
        int getChannel() const { return m_channel; }

        // Pops the oldest received frame, returns false if none is pending
        bool read(Frame *frame);

        // Queues a frame for a single transmission
        bool send(const Frame &frame);

        // Adds a frame that is transmitted every periodMs, or updates the data
        // of an existing cyclic frame with the same id (keeping its schedule
        // and, if beforeSend is empty, its existing beforeSend callback)
        bool setCyclic(const Frame &frame, int periodMs, BeforeSend beforeSend = nullptr);
        void removeCyclic(uint32_t id, bool extended);

        Statistics getStatistics() const;
        std::string getLastError() const;

    private:
        struct CyclicFrame {
            Frame frame;
            std::chrono::microseconds period;
            std::chrono::steady_clock::time_point next;
            BeforeSend beforeSend;
        };

        void worker(int channel, int bitrate, std::promise<bool> *opened);
        void collectOutgoing(std::vector<Frame> *outgoing);
        void pushReceived(const Frame &frame);
        void setError(const std::string &error);

        std::thread m_thread;
        std::atomic<bool> m_running;
        std::atomic<bool> m_open;
        int m_channel;

        mutable std::mutex m_mutex;
        std::deque<Frame> m_rxQueue;
        std::deque<Frame> m_txQueue;
        std::vector<CyclicFrame> m_cyclic;
        std::string m_lastError;

        std::atomic<uint64_t> m_received;
        std::atomic<uint64_t> m_transmitted;
        std::atomic<uint64_t> m_rxDropped;
        std::atomic<uint64_t> m_errors;
};

#endif /* ATG_ENGINE_SIM_CAN_INTERFACE_H */
