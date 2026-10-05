#include <gtest/gtest.h>

#include "ecu/dcm_file.h"
#include "ecu/ecu.h"
#include "ecu/host.h"
#include "ecu/link.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>

#ifdef _WIN32
#include <windows.h>
#endif

namespace {
    float &parameter(const ecu::Ecu &ecu, ecu::CalibrationPage &page, const char *name, int index = 0) {
        return page[ecu.layout().find(name)->offset + index];
    }

    // Runs the ECU over whole engine cycles at constant speed
    int countInjections(ecu::Ecu &ecu, double rpm, int cycles, double dt = 1.0 / 20000) {
        ecu::EcuInputs in;
        ecu::EcuOutputs out;
        in.rpm = rpm;
        in.map_kPa = 100;

        int injections = 0;
        double angle = 0;
        const double steps = cycles * 720.0 / (rpm * 6.0 * dt);
        for (int i = 0; i < steps; ++i) {
            in.time_s += dt;
            in.cycleAngle_deg = angle;
            ecu.step(in, dt, &out);
            injections += out.injectionCount;
            angle = std::fmod(angle + rpm * 6.0 * dt, 720.0);
        }

        return injections;
    }
}

TEST(EcuTests, DcmRoundTrip) {
    ecu::Ecu ecu;
    ecu::CalibrationPage page = ecu.layout().defaults();
    parameter(ecu, page, "InjectorFlow") = 321.5f;
    parameter(ecu, page, "FuelBaseMap", 17) = 7.25f;
    parameter(ecu, page, "FuelCltCorr", 0) = 1.4f;      // first value of the curve

    const std::string path = "ecu_roundtrip_test.dcm";
    std::string error;
    ASSERT_TRUE(ecu::writeDcm(path, ecu.layout(), page, "test", &error)) << error;

    ecu::CalibrationPage read = ecu.layout().defaults();
    std::vector<std::string> warnings;
    ASSERT_TRUE(ecu::readDcm(path, ecu.layout(), &read, &warnings, &error)) << error;
    EXPECT_TRUE(warnings.empty());
    EXPECT_EQ(read, page);

    std::remove(path.c_str());
}

TEST(EcuTests, DcmMissingParametersKeepValues) {
    ecu::Ecu ecu;
    const std::string path = "ecu_partial_test.dcm";
    {
        std::ofstream file(path);
        file << "KONSERVIERUNG_FORMAT 2.0\n"
            << "FESTWERT InjectorFlow\n   WERT 500\nEND\n"
            << "KENNLINIE FuelIatCorr 6\n   ST/X -20 0 20 40 60 80\n   WERT 1.1 1.05 1 0.98 0.96 0.94\nEND\n";
    }

    ecu::CalibrationPage page = ecu.layout().defaults();
    std::vector<std::string> warnings;
    std::string error;
    ASSERT_TRUE(ecu::readDcm(path, ecu.layout(), &page, &warnings, &error)) << error;
    EXPECT_FLOAT_EQ(parameter(ecu, page, "InjectorFlow"), 500.0f);
    EXPECT_FLOAT_EQ(parameter(ecu, page, "FuelIatCorr", 0), 1.1f);
    EXPECT_FLOAT_EQ(parameter(ecu, page, "CrankingRpm"), ecu.layout().defaults()[ecu.layout().find("CrankingRpm")->offset]);
    EXPECT_EQ(warnings.size(), 1u);     // summary of the parameters not in the file

    std::remove(path.c_str());
}

TEST(EcuTests, CurveInterpolation) {
    ecu::CalibrationLayout layout;
    const ecu::Curve curve = layout.addCurve("C", "", "-", 0, 10, { "x", 0, 100, { 0, 10, 20 } }, { 1, 2, 4 });
    const ecu::CalibrationPage &page = layout.defaults();

    EXPECT_FLOAT_EQ(curve.lookup(page, -5), 1.0f);      // clamped
    EXPECT_FLOAT_EQ(curve.lookup(page, 5), 1.5f);
    EXPECT_FLOAT_EQ(curve.lookup(page, 15), 3.0f);
    EXPECT_FLOAT_EQ(curve.lookup(page, 50), 4.0f);      // clamped
}

TEST(EcuTests, OneInjectionPerCylinderAndCycle) {
    ecu::Ecu ecu;
    ecu::CalibrationPage page = ecu.layout().defaults();
    parameter(ecu, page, "EcuFuelEnable") = 1;
    parameter(ecu, page, "CrankingRpm") = 100;
    for (int i = 0; i < ecu.layout().find("FuelBaseMap")->count(); ++i) {
        parameter(ecu, page, "FuelBaseMap", i) = 5;
    }
    ecu.calibration().setWorkingPage(page);
    ecu.beginFrame();

    ecu::EcuConfig config;
    config.cylinders = 4;
    config.firingTdc_deg = { 0, 180, 360, 540 };
    ecu.configure(config);

    EXPECT_EQ(countInjections(ecu, 1000, 10), 4 * 10);
}

TEST(EcuTests, NoInjectionWhileTheCrankRocks) {
    ecu::Ecu ecu;
    ecu::CalibrationPage page = ecu.layout().defaults();
    parameter(ecu, page, "EcuFuelEnable") = 1;
    ecu.calibration().setWorkingPage(page);
    ecu.beginFrame();

    ecu::EcuConfig config;
    config.cylinders = 1;
    config.firingTdc_deg = { 0 };
    ecu.configure(config);

    ecu::EcuInputs in;
    ecu::EcuOutputs out;
    in.map_kPa = 100;
    in.rpm = 0.001;
    int injections = 0;
    for (int i = 0; i < 1000; ++i) {
        in.cycleAngle_deg = 320.0 + ((i % 2) ? 0.01 : -0.01);
        ecu.step(in, 1.0 / 20000, &out);
        injections += out.injectionCount;
    }

    EXPECT_EQ(injections, 0);
}

TEST(EcuTests, StaticMemoryForXcp) {
    // XCP addresses measurements and calibration by their absolute address:
    // they must not move between ECU instances and must sit inside the
    // executable image (offset from the module base fits in 32 bits)
    const double *rpm = nullptr;
    const float *calibration = nullptr;
    {
        ecu::Ecu ecu;
        rpm = ecu.measurements().find("rpm")->value;
        calibration = ecu.calibration().workingMemory();
    }

    ecu::Ecu ecu;
    EXPECT_EQ(ecu.measurements().find("rpm")->value, rpm);
    EXPECT_EQ(ecu.calibration().workingMemory(), calibration);

#ifdef _WIN32
    const char *base = reinterpret_cast<const char *>(GetModuleHandle(nullptr));
    for (const void *p : { (const void *)rpm, (const void *)calibration }) {
        const ptrdiff_t offset = reinterpret_cast<const char *>(p) - base;
        EXPECT_GT(offset, 0);
        EXPECT_LT(offset, (ptrdiff_t)0xFFFFFFFF);
    }
#endif
}

TEST(EcuTests, EpkIdentifiesTheLayout) {
    ecu::Ecu a;
    const std::string epk = a.epk();
    EXPECT_EQ(epk.rfind("ESIM_ECU_", 0), 0u);
    EXPECT_EQ(epk.size(), 17u);

    ecu::Ecu b;
    EXPECT_EQ(b.epk(), epk);
}

TEST(EcuTests, AxesHaveOperatingPointInputs) {
    ecu::Ecu ecu;
    const ecu::ParameterInfo *fuel = ecu.layout().find("FuelBaseMap");
    EXPECT_EQ(fuel->inputX, "rpm");
    EXPECT_EQ(fuel->inputY, "map_kPa");
    EXPECT_EQ(ecu.layout().find("FuelCltCorr")->inputX, "clt_C");
    EXPECT_EQ(ecu.layout().find("IgnIatCorr")->inputX, "iat_C");
}

TEST(EcuTests, DcmSafeWriteAndBackup) {
    namespace fs = std::filesystem;
    const fs::path directory = fs::temp_directory_path() / "engine_sim_ecu_test";
    fs::remove_all(directory);
    fs::create_directories(directory);
    const std::string path = (directory / "calibration.dcm").string();

    ecu::Ecu ecu;
    std::string error, backup;
    ASSERT_TRUE(ecu::backupFile(path, &backup, &error)) << error;     // nothing to back up yet
    EXPECT_TRUE(backup.empty());

    ASSERT_TRUE(ecu::writeDcm(path, ecu.layout(), ecu.layout().defaults(), "test", &error)) << error;
    EXPECT_TRUE(fs::exists(path));
    EXPECT_FALSE(fs::exists(path + ".tmp"));

    ASSERT_TRUE(ecu::backupFile(path, &backup, &error)) << error;
    EXPECT_TRUE(fs::exists(backup));
    EXPECT_EQ(fs::path(backup).parent_path().filename().string(), "backup");
    EXPECT_EQ(fs::file_size(backup), fs::file_size(path));

    fs::remove_all(directory);
}

TEST(EcuTests, RemoteIoCommandsTheInjection) {
    // ECU on a PC, angle domain on an I/O controller: averaged inputs every
    // 10 ms, the injection goes out as a command
    ecu::Ecu ecu;
    ecu::CalibrationPage page = ecu.layout().defaults();
    parameter(ecu, page, "EcuFuelEnable") = 1;
    parameter(ecu, page, "InjectionAngle") = 380;
    for (int i = 0; i < ecu.layout().find("FuelBaseMap")->count(); ++i) {
        parameter(ecu, page, "FuelBaseMap", i) = 4;
    }
    ecu.calibration().setWorkingPage(page);
    ecu.beginFrame();

    ecu::EcuConfig config;
    config.cylinders = 1;
    config.firingTdc_deg = { 0 };
    config.remoteIo = true;
    ecu.configure(config);

    ecu::EcuInputs in;
    ecu::EcuOutputs out;
    in.rpm = 900;
    in.map_kPa = 55;
    for (int i = 0; i < 100; ++i) {
        in.time_s += 0.010;
        ecu.step(in, 0.010, &out);
        EXPECT_EQ(out.injectionCount, 0);
    }

    // Taken as they come (no second averaging)
    EXPECT_DOUBLE_EQ(ecu.signals().rpm, 900.0);
    EXPECT_DOUBLE_EQ(ecu.signals().map_kPa, 55.0);

    EXPECT_TRUE(out.fuelControl);
    EXPECT_NEAR(out.injection_ms, 4.0, 1e-6);
    EXPECT_DOUBLE_EQ(out.injectionAngle_deg, 380.0);

    // Fuel not under ECU control: no injection commanded
    parameter(ecu, page, "EcuFuelEnable") = 0;
    ecu.calibration().setWorkingPage(page);
    ecu.beginFrame();
    ecu.step(in, 0.010, &out);
    EXPECT_FALSE(out.fuelControl);
    EXPECT_EQ(out.injection_ms, 0.0);
}

TEST(EcuTests, ParseIpv4) {
    uint8_t address[4] = {};
    EXPECT_TRUE(ecu::parseIpv4("192.168.1.20", address));
    EXPECT_EQ(address[0], 192);
    EXPECT_EQ(address[3], 20);
    EXPECT_FALSE(ecu::parseIpv4("192.168.1", address));
    EXPECT_FALSE(ecu::parseIpv4("192.168.1.256", address));
    EXPECT_FALSE(ecu::parseIpv4("192.168.1.2x", address));
}

TEST(EcuTests, HostCalibrationFile) {
    namespace fs = std::filesystem;
    const fs::path directory = fs::temp_directory_path() / "engine_sim_ecu_host_test";
    fs::remove_all(directory);
    const std::string path = (directory / "calibration.dcm").string();

    ecu::Ecu ecu;
    ecu::EcuHost host(ecu);
    std::vector<std::string> messages;
    host.setMessageHandler([&](const std::string &text) { messages.push_back(text); });

    // Missing file: written with the defaults, then loaded
    host.openCalibration(path);
    ASSERT_TRUE(fs::exists(path));
    ASSERT_FALSE(messages.empty());
    EXPECT_EQ(messages.back().rfind("ECU: calibration loaded", 0), 0u);

    // Saved by someone else: reloaded on the next check
    ecu::CalibrationPage page = ecu.layout().defaults();
    parameter(ecu, page, "InjectorFlow") = 512;
    std::string error;
    ASSERT_TRUE(ecu::writeDcm(path, ecu.layout(), page, "edited", &error)) << error;
    fs::last_write_time(path, fs::last_write_time(path) + std::chrono::seconds(1));
    host.update(1.0);
    host.update(1.0);
    EXPECT_FLOAT_EQ(ecu.calibrationValue("InjectorFlow"), 512.0f);

    // Tool's save request (CalSaveRequest 0 -> 1): written with the request
    // cleared, previous file in backup/
    parameter(ecu, page, "InjectorFlow") = 600;
    parameter(ecu, page, "CalSaveRequest") = 1;
    ecu.calibration().setWorkingPage(page);
    host.update(0.01);
    EXPECT_TRUE(fs::exists(directory / "backup"));

    ecu::CalibrationPage saved = ecu.layout().defaults();
    std::vector<std::string> warnings;
    ASSERT_TRUE(ecu::readDcm(path, ecu.layout(), &saved, &warnings, &error)) << error;
    EXPECT_FLOAT_EQ(parameter(ecu, saved, "InjectorFlow"), 600.0f);
    EXPECT_FLOAT_EQ(parameter(ecu, saved, "CalSaveRequest"), 0.0f);

    fs::remove_all(directory);
}

TEST(EcuTests, HostDatalog) {
    const std::string path = "ecu_host_log_test.csv";
    {
        ecu::Ecu ecu;
        ecu::EcuHost host(ecu);
        ASSERT_TRUE(host.openLog(path, { "extra" }));

        int rows = 0;
        for (int i = 0; i < 100; ++i) {         // 1 s at 10 ms
            if (host.logDue(0.010)) {
                host.writeLogRow({ 1.5 });
                ++rows;
            }
        }
        EXPECT_EQ(rows, 1 + 20);    // first sample, then every 50 ms
    }

    std::ifstream file(path);
    std::string header, row;
    std::getline(file, header);
    std::getline(file, row);
    EXPECT_EQ(header.rfind("time_s,", 0), 0u);
    EXPECT_NE(header.find(",rpm,"), std::string::npos);
    EXPECT_EQ(header.substr(header.size() - 6), ",extra");
    EXPECT_EQ(row.substr(row.size() - 4), ",1.5");
    file.close();

    std::remove(path.c_str());
}

TEST(EcuTests, LinkSampleRoundTrip) {
    ecu::link::Sample in;
    in.counter = 201;
    in.synchronized = true;
    in.failsafe = true;
    in.time_us = 4000000123u;
    in.cycles = 77;
    in.rpm = 912.25;
    in.map_kPa = 61.37;
    in.tps_percent = 12.5;
    in.iat_C = -12.3;
    in.clt_C = 88.8;
    in.lambda = 0.9876;
    in.battery_V = 13.812;

    ecu::link::Frame frames[3];
    ecu::link::encodeSample(in, frames);
    EXPECT_EQ(frames[2].id, ecu::link::SensorsId);      // completes the sample, sent last

    // Through a datagram, as over UDP
    uint8_t buffer[3 * ecu::link::PackedFrameSize];
    ASSERT_EQ(ecu::link::pack(frames, 3, buffer), sizeof(buffer));
    ecu::link::Frame unpacked[4];
    ASSERT_EQ(ecu::link::unpack(buffer, sizeof(buffer), unpacked, 4), 3);

    ecu::link::Sample out;
    EXPECT_FALSE(ecu::link::decodeSample(unpacked[0], &out));
    EXPECT_FALSE(ecu::link::decodeSample(unpacked[1], &out));
    EXPECT_TRUE(ecu::link::decodeSample(unpacked[2], &out));

    EXPECT_EQ(out.counter, 201);
    EXPECT_TRUE(out.synchronized);
    EXPECT_TRUE(out.failsafe);
    EXPECT_EQ(out.time_us, 4000000123u);
    EXPECT_EQ(out.cycles, 77u);
    EXPECT_NEAR(out.rpm, 912.25, 0.125);
    EXPECT_NEAR(out.map_kPa, 61.37, 0.005);
    EXPECT_NEAR(out.tps_percent, 12.5, 0.005);
    EXPECT_NEAR(out.iat_C, -12.3, 0.05);
    EXPECT_NEAR(out.clt_C, 88.8, 0.05);
    EXPECT_NEAR(out.lambda, 0.9876, 0.00005);
    EXPECT_NEAR(out.battery_V, 13.812, 0.0005);
}

TEST(EcuTests, LinkCommandRoundTripAndLimits) {
    ecu::link::Command in;
    in.counter = 9;
    in.fuelControl = true;
    in.sparkControl = false;
    in.injection_ms = 8.1234;
    in.injectionAngle_deg = 400.0;
    in.sparkAdvance_deg = -5.5;

    ecu::link::Command out;
    ASSERT_TRUE(ecu::link::decodeCommand(ecu::link::encodeCommand(in), &out));
    EXPECT_EQ(out.counter, 9);
    EXPECT_TRUE(out.fuelControl);
    EXPECT_FALSE(out.sparkControl);
    EXPECT_NEAR(out.injection_ms, 8.123, 0.0005);
    EXPECT_NEAR(out.injectionAngle_deg, 400.0, 0.05);
    EXPECT_NEAR(out.sparkAdvance_deg, -5.5, 0.05);

    // Out of range: saturated, not wrapped
    in.injection_ms = 1000;
    in.sparkAdvance_deg = -1e6;
    ASSERT_TRUE(ecu::link::decodeCommand(ecu::link::encodeCommand(in), &out));
    EXPECT_NEAR(out.injection_ms, 65.535, 1e-9);
    EXPECT_NEAR(out.sparkAdvance_deg, -3276.8, 1e-9);

    ecu::link::Sample sample;
    EXPECT_FALSE(ecu::link::decodeCommand(ecu::link::Frame(), &out));    // id 0
    EXPECT_FALSE(ecu::link::decodeSample(ecu::link::encodeCommand(in), &sample));
}

TEST(EcuTests, LinkUdpLoopback) {
    // ECU side listens; the I/O controller side sends to it and gets the
    // answer back at its own (ephemeral) port
    ecu::link::UdpEndpoint ecuSide, controller;
    std::string error;
    ASSERT_TRUE(controller.open("0.0.0.0", 0, &error)) << error;

    uint16_t port = 0;
    for (uint16_t p = 56000; p < 56100 && port == 0; ++p) {
        if (ecuSide.open("127.0.0.1", p, &error)) port = p;
    }
    ASSERT_NE(port, 0);
    ASSERT_TRUE(controller.setPeer("127.0.0.1", port, &error)) << error;

    ecu::link::Frame none[1];
    EXPECT_EQ(ecuSide.receive(none, 1, 0), 0);          // nothing yet, no wait

    ecu::link::Sample sample;
    sample.counter = 5;
    sample.rpm = 900;
    ecu::link::Frame frames[3];
    ecu::link::encodeSample(sample, frames);
    ASSERT_TRUE(controller.send(frames, 3));

    ecu::link::Frame received[ecu::link::MaxFramesPerDatagram];
    ASSERT_EQ(ecuSide.receive(received, ecu::link::MaxFramesPerDatagram, 1000), 3);
    ecu::link::Sample decoded;
    for (int i = 0; i < 3; ++i) ecu::link::decodeSample(received[i], &decoded);
    EXPECT_EQ(decoded.counter, 5);
    EXPECT_DOUBLE_EQ(decoded.rpm, 900.0);

    // Answer to the last sender
    ecu::link::Command command;
    command.counter = decoded.counter;
    const ecu::link::Frame answer = ecu::link::encodeCommand(command);
    ASSERT_TRUE(ecuSide.send(&answer, 1));
    ASSERT_EQ(controller.receive(received, ecu::link::MaxFramesPerDatagram, 1000), 1);
    ecu::link::Command back;
    ASSERT_TRUE(ecu::link::decodeCommand(received[0], &back));
    EXPECT_EQ(back.counter, 5);
}

TEST(EcuTests, LinkParseEndpoint) {
    std::string address = "127.0.0.1";
    uint16_t port = 5600;
    EXPECT_TRUE(ecu::link::parseEndpoint("192.168.0.10:6000", &address, &port));
    EXPECT_EQ(address, "192.168.0.10");
    EXPECT_EQ(port, 6000);
    EXPECT_TRUE(ecu::link::parseEndpoint(":7000", &address, &port));
    EXPECT_EQ(address, "192.168.0.10");
    EXPECT_EQ(port, 7000);
    EXPECT_FALSE(ecu::link::parseEndpoint("192.168.0.10:0", &address, &port));
    EXPECT_FALSE(ecu::link::parseEndpoint("host:6000", &address, &port));
}

#ifdef ECU_XCP_ENABLED
#include <xcplib.h>

extern "C" uint8_t XcpCalSegWriteMemory(uint32_t dst, uint16_t size, const uint8_t *src);
extern "C" uint8_t XcpCalSegPublishAll(bool wait);

TEST(EcuTests, XcpLibraryLinked) {
    // Built with XCPlite; nothing started, so no client can be connected
    EXPECT_FALSE(XcpIsConnected());
}

TEST(EcuTests, XcpCalibrationSegment) {
    // XCP core without UDP server and A2L: file loads and tool writes both
    // reach the ECU, and the active page is mirrored into the default page
    ecu::Ecu ecu;
    ecu::XcpSettings settings;
    settings.server = false;
    settings.generateA2l = false;
    settings.logLevel = 1;
    std::string error;
    ASSERT_TRUE(ecu.startXcp(settings, &error)) << error;
    ASSERT_TRUE(ecu.xcpStatus().running);

    // Calibration file loaded: written into the XCP working page
    ecu::CalibrationPage page = ecu.layout().defaults();
    parameter(ecu, page, "InjectorFlow") = 333;
    ASSERT_TRUE(ecu.setCalibration(page, &error)) << error;
    ecu.beginFrame();
    EXPECT_FLOAT_EQ(ecu.calibrationValue("InjectorFlow"), 333.0f);

    // Tool DOWNLOAD into the working page (what XCPlite does for VISION)
    const uint16_t segment = XcpFindCalSeg("EcuCalibration");
    ASSERT_NE(segment, XCP_UNDEFINED_CALSEG);
    auto toolWrite = [&](const char *name, float value) {
        const uint32_t offset = (uint32_t)(ecu.layout().find(name)->offset * sizeof(float));
        return XcpCalSegWriteMemory(0x80000000u + ((uint32_t)segment << 16) + offset,
            sizeof(float), reinterpret_cast<const uint8_t *>(&value));
    };

    // Frame with the XCP server's background publishing in between: the ECU
    // picks the new page up at its next lock
    auto frame = [&]() {
        ecu.beginFrame();
        XcpCalSegPublishAll(false);
        ecu.beginFrame();
    };

    ASSERT_EQ(toolWrite("InjectorFlow", 444), 0);
    frame();
    EXPECT_FLOAT_EQ(ecu.calibrationValue("InjectorFlow"), 444.0f);
    const size_t offset = ecu.layout().find("InjectorFlow")->offset;
    EXPECT_FLOAT_EQ(ecu.calibration().workingMemory()[offset], 444.0f);   // mirror

    // CalSaveRequest: rising edge only
    EXPECT_FALSE(ecu.takeSaveRequest());
    ASSERT_EQ(toolWrite("CalSaveRequest", 1), 0);
    frame();
    EXPECT_TRUE(ecu.takeSaveRequest());
    EXPECT_FALSE(ecu.takeSaveRequest());
    ecu.beginFrame();
    EXPECT_FALSE(ecu.takeSaveRequest());
    EXPECT_EQ(ecu.measurements().find("cal_save_count")->value[0], 1.0);

    ecu.stopXcp();
    EXPECT_FALSE(ecu.xcpStatus().running);
}
#endif
