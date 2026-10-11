#include <gtest/gtest.h>

#include "../include/eddy_current_brake.h"
#include "../include/dynamometer.h"
#include "../include/constants.h"

namespace {
    constexpr double Dt = 1.0 / 10000;

    double omega(double rpm) { return rpm * 2 * constants::pi / 60.0; }
}

TEST(EddyCurrentBrakeTests, StaticPointsOfTheBenchSweeps) {
    EddyCurrentBrake brake;

    // PWM sweeps of the bench (0..4095)
    EXPECT_NEAR(brake.staticTorque(2000.0 / 4095, 2500), 3.677, 0.15);
    EXPECT_NEAR(brake.staticTorque(3500.0 / 4095, 2500), 11.398, 0.4);
    EXPECT_NEAR(brake.staticTorque(2000.0 / 4095, 3200), 4.127, 0.15);
    EXPECT_NEAR(brake.staticTorque(3500.0 / 4095, 3200), 12.326, 0.4);
    EXPECT_NEAR(brake.staticTorque(0.0, 3000), 0.21, 0.01);

    // Only absorbs: nothing at standstill, more with speed
    EXPECT_NEAR(brake.staticTorque(1.0, 0), 0.0, 1E-9);
    EXPECT_GT(brake.staticTorque(0.5, 3200), brake.staticTorque(0.5, 2500));
}

TEST(EddyCurrentBrakeTests, LoadDemandIsLinear) {
    // DYNO_Command's load: torque = drag + load x (maximum - drag), any speed
    EddyCurrentBrake brake;
    for (double rpm : { 1000.0, 2500.0, 3600.0 }) {
        const double drag = brake.staticTorque(0.0, rpm);
        const double full = brake.staticTorque(1.0, rpm);
        for (double load : { 0.1, 0.25, 0.5, 0.75 }) {
            EXPECT_NEAR(brake.staticTorque(brake.dutyForLoad(load), rpm), drag + load * (full - drag), 0.01);
            EXPECT_NEAR(brake.loadForDuty(brake.dutyForLoad(load)), load, 1E-6);
        }
    }
}

TEST(EddyCurrentBrakeTests, CoilLag) {
    // PWM step: the coil current reaches 63 % in one time constant (1.8 s)
    EddyCurrentBrake brake;
    Dynamometer command;
    command.m_enabled = false;

    const int steps = (int)(1.8 / Dt);
    for (int i = 0; i < steps; ++i) {
        brake.setExternalCommand(0.5, true);
        brake.step(Dt, omega(3000), command);
    }
    EXPECT_NEAR(brake.excitation(), 0.5 * 0.632, 0.01);
}

TEST(EddyCurrentBrakeTests, ExternalCommandTimesOut) {
    EddyCurrentBrake brake;
    Dynamometer command;
    command.m_enabled = false;

    brake.setExternalCommand(0.5, true);
    brake.step(Dt, omega(3000), command);
    EXPECT_TRUE(brake.isExternal());
    EXPECT_NEAR(brake.duty(), 0.5, 1E-9);

    // No more commands: after the timeout the coil is switched off (dyno off)
    for (int i = 0; i < 6000; ++i) brake.step(Dt, omega(3000), command);
    EXPECT_FALSE(brake.isExternal());
    EXPECT_TRUE(brake.isExternalTimedOut());
    EXPECT_NEAR(brake.duty(), 0.0, 1E-9);
}

TEST(EddyCurrentBrakeTests, InternalTorqueControlReachesSetpoint) {
    EddyCurrentBrake brake;
    Dynamometer command;
    command.m_enabled = true;
    command.m_torqueMode = true;
    command.m_torqueSetpoint = 3.0;

    for (int i = 0; i < 300000; ++i) brake.step(Dt, omega(3000), command);
    EXPECT_TRUE(brake.isInternalControl());
    EXPECT_NEAR(brake.torque(), 3.0, 0.05);
}

TEST(EddyCurrentBrakeTests, SpeedSensorCountsPulses) {
    EddyCurrentBrake brake;
    Dynamometer command;
    command.m_enabled = false;

    for (int i = 0; i < 10000; ++i) brake.step(Dt, omega(3000), command);
    EXPECT_NEAR(brake.measuredSpeed_rpm(), 3000, 60.0 / 720 / 0.01 + 1E-6);
}
