// Standalone ECU: the ECU library as a program of its own, for a PC that
// controls a real engine through an I/O controller (see io_link.h). The ECU
// runs as in the simulator, with the same calibration file, XCP server and
// datalog; only its inputs and outputs come from the link. On Windows a
// status window shows what it is doing.

#include "io_link.h"
#include "status.h"

#ifdef _WIN32
#include "status_window.h"
#endif

#include "ecu/ecu.h"
#include "ecu/host.h"
#include "ecu/link.h"

#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#endif

namespace {
    std::atomic<bool> StopRequested{ false };
    std::mutex PrintMutex;
    StatusBoard Board;

    void print(const std::string &text) {
        std::lock_guard<std::mutex> lock(PrintMutex);
        std::printf("%s\n", text.c_str());
        std::fflush(stdout);
    }

    // To the console and the status window
    void report(const std::string &text) {
        print(text);
        Board.addMessage(text);
    }

    void onSignal(int) {
        StopRequested = true;
    }

    struct Options {
        std::string calibration;        // empty: defaultCalibration()
        std::string log;

        // XCP server on by default; --no-xcp turns it off
        bool xcp = true;
        bool xcpRequested = false;      // --xcp given: a build without XCP is an error
        std::string xcpEndpoint;
        std::string xcpClock = "ecu";
        int xcpLog = 2;

        int cylinders = 1;
        std::vector<double> firingTdc;

        // I/O link: UDP (default) or stimulus
        bool useStimulus = false;
        std::string stimulus;
        std::string udpAddress = "127.0.0.1";
        uint16_t udpPort = ecu::link::DefaultPort;

        double duration = 0;
        double status = -1;         // < 0: 1 s without the window, off with it

#ifdef _WIN32
        bool window = true;
#else
        bool window = false;
#endif
    };

    void usage() {
        std::printf(
            "Usage: ecu-standalone [options]\n"
            "\n"
            "  --cal <file.dcm>        calibration file, reloaded whenever it is saved\n"
            "                          (default: the repository's\n"
            "                          assets\\calibration\\calibration.dcm, found from\n"
            "                          the executable's folder up, else calibration.dcm\n"
            "                          here; written with the defaults if missing)\n"
            "  --xcp [address[:port]]  XCP on Ethernet server address (default\n"
            "                          0.0.0.0:5555); the server is always on\n"
            "  --no-xcp                no XCP server\n"
            "  --xcp-clock ecu|pc      DAQ timestamps in I/O controller time (default)\n"
            "                          or PC time\n"
            "  --xcp-log <0..4>        XCPlite debug output (default 2)\n"
            "  --log <file.csv>        datalog of every measurement, 20 Hz\n"
            "  --cylinders <n>         default 1\n"
            "  --firing-tdc <a,b,...>  firing TDC of each cylinder on 0..720 deg\n"
            "                          (default evenly spaced from 0)\n"
            "  --udp <address[:port]>  I/O link over UDP: where the ECU listens for\n"
            "                          the I/O controller (default 127.0.0.1:5600;\n"
            "                          0.0.0.0 = from other computers too)\n"
            "  --stimulus [k=v,...]    I/O link: fixed inputs instead of an I/O\n"
            "                          controller (rpm, map, iat, clt, tps, lambda,\n"
            "                          battery)\n"
            "  --duration <s>          stop after this ECU time (default: Ctrl+C or\n"
            "                          closing the window)\n"
            "  --status <s>            console status line period, 0 = off (default\n"
            "                          1, off with the window)\n"
            "  --no-window             console only (Windows: no status window)\n"
            "\n"
            "I/O link: UDP by default (the simulator or a microcontroller sends a\n"
            "sample every 10 ms and gets the ECU's command back). With --stimulus,\n"
            "type k=v,... and Enter to change the inputs (console or window), an\n"
            "empty line to show them, q to quit.\n");
    }

    // Without --cal: the repository's calibration, looked for from the
    // executable's folder up (build\ecu\Release -> assets\calibration), so
    // a double click opens the lab calibration; else calibration.dcm in the
    // working directory
    std::string defaultCalibration() {
        std::filesystem::path dir;
#ifdef _WIN32
        wchar_t exe[MAX_PATH];
        const DWORD length = GetModuleFileNameW(nullptr, exe, MAX_PATH);
        if (length > 0 && length < MAX_PATH) dir = std::filesystem::path(exe).parent_path();
#endif

        std::error_code ec;
        for (int level = 0; level < 6 && !dir.empty(); ++level) {
            const std::filesystem::path candidate = dir / "assets" / "calibration" / "calibration.dcm";
            if (std::filesystem::exists(candidate, ec)) return candidate.string();

            if (dir == dir.parent_path()) break;
            dir = dir.parent_path();
        }

        return "calibration.dcm";
    }

    bool parseNumber(const std::string &text, double *value) {
        char *end = nullptr;
        *value = std::strtod(text.c_str(), &end);
        return !text.empty() && *end == '\0';
    }

    bool parseList(const std::string &text, std::vector<double> *values) {
        values->clear();
        size_t start = 0;
        while (start <= text.size()) {
            const size_t comma = text.find(',', start);
            const std::string item = text.substr(start, comma == std::string::npos ? std::string::npos : comma - start);
            double value = 0;
            if (!parseNumber(item, &value)) return false;
            values->push_back(value);
            if (comma == std::string::npos) break;
            start = comma + 1;
        }
        return !values->empty();
    }

    bool parseOptions(int argc, char **argv, Options *options) {
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            const bool hasValue = i + 1 < argc && argv[i + 1][0] != '-';
            auto value = [&](std::string *out) {
                if (!hasValue) {
                    std::fprintf(stderr, "%s needs a value\n", arg.c_str());
                    return false;
                }
                *out = argv[++i];
                return true;
            };
            auto number = [&](double *out) {
                std::string text;
                if (!value(&text)) return false;
                if (!parseNumber(text, out)) {
                    std::fprintf(stderr, "%s: bad number '%s'\n", arg.c_str(), text.c_str());
                    return false;
                }
                return true;
            };

            double n = 0;
            std::string text;
            if (arg == "--help" || arg == "-h") {
                usage();
                std::exit(0);
            }
            else if (arg == "--cal") {
                if (!value(&options->calibration)) return false;
            }
            else if (arg == "--log") {
                if (!value(&options->log)) return false;
            }
            else if (arg == "--xcp") {
                options->xcp = true;
                options->xcpRequested = true;
                if (hasValue) options->xcpEndpoint = argv[++i];
            }
            else if (arg == "--no-xcp") {
                options->xcp = false;
            }
            else if (arg == "--xcp-clock") {
                if (!value(&options->xcpClock)) return false;
                if (options->xcpClock != "ecu" && options->xcpClock != "pc") {
                    std::fprintf(stderr, "--xcp-clock: ecu or pc\n");
                    return false;
                }
            }
            else if (arg == "--xcp-log") {
                if (!number(&n)) return false;
                options->xcpLog = (int)n;
            }
            else if (arg == "--cylinders") {
                if (!number(&n)) return false;
                options->cylinders = (int)n;
                if (options->cylinders < 1 || options->cylinders > ecu::EcuOutputs::MaxInjections) {
                    std::fprintf(stderr, "--cylinders: 1..%d\n", ecu::EcuOutputs::MaxInjections);
                    return false;
                }
            }
            else if (arg == "--firing-tdc") {
                if (!value(&text)) return false;
                if (!parseList(text, &options->firingTdc)) {
                    std::fprintf(stderr, "--firing-tdc: bad list '%s'\n", text.c_str());
                    return false;
                }
            }
            else if (arg == "--stimulus") {
                options->useStimulus = true;
                if (hasValue) options->stimulus = argv[++i];
            }
            else if (arg == "--udp") {
                if (!value(&text)) return false;
                if (!ecu::link::parseEndpoint(text, &options->udpAddress, &options->udpPort)) {
                    std::fprintf(stderr, "--udp: bad address '%s'\n", text.c_str());
                    return false;
                }
            }
            else if (arg == "--duration") {
                if (!number(&options->duration)) return false;
            }
            else if (arg == "--status") {
                if (!number(&options->status)) return false;
            }
            else if (arg == "--no-window") {
                options->window = false;
            }
            else {
                std::fprintf(stderr, "unknown option %s (--help for the list)\n", arg.c_str());
                return false;
            }
        }

        if (!options->firingTdc.empty() && (int)options->firingTdc.size() != options->cylinders) {
            std::fprintf(stderr, "--firing-tdc: %d angles for %d cylinders\n",
                (int)options->firingTdc.size(), options->cylinders);
            return false;
        }

        if (options->status < 0) options->status = options->window ? 0 : 1;
        if (options->calibration.empty()) options->calibration = defaultCalibration();

        return true;
    }

    bool xcpSettings(const Options &options, ecu::XcpSettings *settings) {
        settings->simulatedClock = options.xcpClock == "ecu";
        settings->logLevel = options.xcpLog;

        std::string address = options.xcpEndpoint;
        const size_t colon = address.find(':');
        if (colon != std::string::npos) {
            double port = 0;
            if (!parseNumber(address.substr(colon + 1), &port) || port < 1 || port > 65535) {
                report("XCP: bad port in " + address);
                return false;
            }
            settings->port = (uint16_t)port;
            address = address.substr(0, colon);
        }

        if (!address.empty() && !ecu::parseIpv4(address, settings->address)) {
            report("XCP: invalid address " + address);
            return false;
        }

        return true;
    }

    std::string endpoint(const ecu::XcpSettings &settings) {
        char text[32];
        std::snprintf(text, sizeof(text), "%u.%u.%u.%u:%u",
            settings.address[0], settings.address[1], settings.address[2], settings.address[3], settings.port);
        return text;
    }

    const char *engineStateName(double state) {
        switch ((int)state) {
            case ecu::Cranking: return "cranking";
            case ecu::Running: return "running";
            default: return "stopped";
        }
    }

    void printStatus(const ecu::EcuSignals &s, const ecu::EcuOutputs &out) {
        char text[256];
        std::snprintf(text, sizeof(text),
            "t=%8.2f s  %5.0f rpm  MAP %5.1f kPa  %-8s  fuel %s %6.3f ms @ %5.1f deg  spark %s %5.1f deg",
            s.time_s, s.rpm, s.map_kPa, engineStateName(s.engineState),
            out.fuelControl ? "ECU" : "off", out.injection_ms, out.injectionAngle_deg,
            out.sparkControl ? "ECU" : "off", out.sparkAdvance_deg);
        print(text);
    }

    // Console commands for the stimulus link
    void readConsole(StimulusLink *stimulus) {
        std::string line;
        while (!StopRequested && std::getline(std::cin, line)) {
            if (line == "q" || line == "quit") {
                StopRequested = true;
                break;
            }

            std::string error;
            if (line.find_first_not_of(" \t\r") == std::string::npos) {
                report("stimulus: " + stimulus->values());
            }
            else if (stimulus->set(line, &error)) {
                report("stimulus: " + stimulus->values());
            }
            else {
                report("stimulus: " + error);
            }
        }
    }

    // The ECU from start-up to stop; publishes what it does on the board
    int runEcu(const Options &options, IoLink &link) {
        using Clock = EcuStatus::Clock;

        auto fail = []() {
            Board.update([](EcuStatus &s) { s.state = EcuStatus::State::Failed; });
            return 1;
        };

        try {
            ecu::Ecu ecu;
            ecu::EcuHost host(ecu);
            host.setMessageHandler(report);

            // XCP is on by default, but a build without it still runs
            const bool xcp = options.xcp && (ecu::XcpInterface::available() || options.xcpRequested);

            std::error_code ec;
            const std::string calibrationFile = std::filesystem::absolute(options.calibration, ec).string();
            Board.update([&](EcuStatus &s) {
                s.software = ecu.epk();
                s.calibrationFile = calibrationFile;
                s.xcpEnabled = xcp;
                s.logFile = options.log;
                s.link = link.describe();
                s.linkHasController = link.hasController();
            });

            report("ECU software " + ecu.epk());
            host.openCalibration(options.calibration);

            if (options.xcp && !xcp) {
                report("XCP: not in this build (CMake option ECU_XCP_ENABLED), running without it");
            }
            if (xcp) {
                ecu::XcpSettings settings;
                if (!xcpSettings(options, &settings)) return fail();

                Board.update([&](EcuStatus &s) { s.xcpEndpoint = endpoint(settings); });
                if (!host.startXcp(settings)) return fail();
            }

            ecu::EcuConfig config;
            config.cylinders = options.cylinders;
            config.remoteIo = true;
            for (int i = 0; i < options.cylinders; ++i) {
                config.firingTdc_deg.push_back(options.firingTdc.empty()
                    ? 720.0 * i / options.cylinders
                    : options.firingTdc[i]);
            }
            ecu.configure(config);

            if (!options.log.empty()) {
                if (host.openLog(options.log)) report("ECU: datalog " + options.log);
                else Board.update([](EcuStatus &s) { s.logFile.clear(); });
            }

            report("I/O link: " + link.describe());
            Board.update([](EcuStatus &s) {
                s.state = EcuStatus::State::Running;
                s.started = Clock::now();
            });

            ecu::EcuOutputs outputs;
            double time = 0;
            double statusTimer = 0;
            uint64_t samples = 0;
            uint64_t rateSamples = 0;
            double rate = 0;
            Clock::time_point lastSample;
            Clock::time_point rateStart = Clock::now();

            auto publish = [&]() {
                const ecu::XcpStatus xcp = ecu.xcpStatus();
                const bool fuel = ecu.calibrationValue("EcuFuelEnable") >= 0.5f;
                const bool ignition = ecu.calibrationValue("EcuIgnEnable") >= 0.5f;
                Board.update([&](EcuStatus &s) {
                    s.samples = samples;
                    s.lastSample = lastSample;
                    s.sampleRate_Hz = rate;
                    s.lostSamples = link.lostSamples();
                    s.xcpRunning = xcp.running;
                    s.xcpConnected = xcp.connected;
                    s.calibrationFileOk = host.calibrationFileOk();
                    s.fuelEnabled = fuel;
                    s.ignitionEnabled = ignition;
                    s.signals = ecu.signals();
                    s.outputs = outputs;
                });
            };

            bool closed = false;
            while (!StopRequested) {
                ecu::EcuInputs inputs;
                double dt = 0;
                const IoLink::Receive received = link.receive(&inputs, &dt);
                if (received == IoLink::Receive::Closed) {
                    report("I/O link closed");
                    closed = true;
                    break;
                }
                else if (received == IoLink::Receive::Timeout) {
                    // No samples: keep the calibration file and XCP serviced
                    host.update(0.1);
                    publish();
                    continue;
                }

                const Clock::time_point now = Clock::now();
                ++samples;
                lastSample = now;
                ++rateSamples;
                const double rateTime = std::chrono::duration<double>(now - rateStart).count();
                if (rateTime >= 1.0) {
                    rate = rateSamples / rateTime;
                    rateSamples = 0;
                    rateStart = now;
                }

                host.update(dt);
                ecu.step(inputs, dt, &outputs);
                link.send(outputs);
                publish();

                if (host.logDue(dt)) host.writeLogRow();

                time += dt;
                statusTimer -= dt;
                if (options.status > 0 && statusTimer <= 0) {
                    statusTimer = options.status;
                    printStatus(ecu.signals(), outputs);
                }

                if (options.duration > 0 && time >= options.duration - 1e-9) break;
            }

            Board.update([&](EcuStatus &s) {
                s.state = EcuStatus::State::Stopped;
                s.linkClosed = closed;
            });
            report("ECU stopped");
        }
        catch (const std::exception &e) {
            report(e.what());
            return fail();
        }

        return 0;
    }
}

int main(int argc, char **argv) {
    Options options;
    if (!parseOptions(argc, argv, &options)) return 1;

    std::signal(SIGINT, onSignal);
#ifdef _WIN32
    // 1 ms sleep resolution for the 10 ms pacing, kept while the window is
    // in the background or hidden (Windows 11 drops it otherwise), no
    // throttling onto efficiency cores: the simulator waits for the ECU's
    // answers in lockstep
    PROCESS_POWER_THROTTLING_STATE throttling = {};
    throttling.Version = PROCESS_POWER_THROTTLING_CURRENT_VERSION;
    throttling.ControlMask =
        PROCESS_POWER_THROTTLING_EXECUTION_SPEED
        | PROCESS_POWER_THROTTLING_IGNORE_TIMER_RESOLUTION;
    throttling.StateMask = 0;
    SetProcessInformation(GetCurrentProcess(), ProcessPowerThrottling, &throttling, sizeof(throttling));
    SetPriorityClass(GetCurrentProcess(), ABOVE_NORMAL_PRIORITY_CLASS);
    timeBeginPeriod(1);
#endif

    std::string error;
    StimulusLink stimulus;
    UdpLink udp;
    IoLink *link = nullptr;
    if (options.useStimulus) {
        if (!options.stimulus.empty() && !stimulus.set(options.stimulus, &error)) {
            std::fprintf(stderr, "--stimulus: %s\n", error.c_str());
            return 1;
        }
        std::thread(readConsole, &stimulus).detach();
        link = &stimulus;
    }
    else {
        if (!udp.open(options.udpAddress, options.udpPort, &error)) {
            std::fprintf(stderr, "I/O link: %s\n", error.c_str());
            return 1;
        }
        link = &udp;
    }

#ifdef _WIN32
    if (options.window) {
        // The window on this thread, the ECU on its own
        int result = 0;
        std::thread ecuThread([&]() { result = runEcu(options, *link); });
        runStatusWindow(Board, options.useStimulus ? &stimulus : nullptr, StopRequested);
        StopRequested = true;
        ecuThread.join();
        return result;
    }
#endif

    return runEcu(options, *link);
}
