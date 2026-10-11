#include "../include/engine_sim_application.h"

#include <iostream>

#pragma comment(lib, "winmm.lib")

namespace {
    // The physics must keep up with real time even when another window (e.g.
    // the IndiCom or VISION) has the focus or covers this one. Windows
    // otherwise throttles background processes onto efficiency cores (hybrid
    // CPUs) and, since Windows 11, drops the 1 ms timer resolution of a
    // process whose window is hidden, so the frame pacing jitters by ~16 ms.
    void requestRealTimePerformance() {
        PROCESS_POWER_THROTTLING_STATE throttling = {};
        throttling.Version = PROCESS_POWER_THROTTLING_CURRENT_VERSION;
        throttling.ControlMask =
            PROCESS_POWER_THROTTLING_EXECUTION_SPEED
            | PROCESS_POWER_THROTTLING_IGNORE_TIMER_RESOLUTION;
        throttling.StateMask = 0;   // opt out of both
        SetProcessInformation(GetCurrentProcess(), ProcessPowerThrottling, &throttling, sizeof(throttling));

        SetPriorityClass(GetCurrentProcess(), ABOVE_NORMAL_PRIORITY_CLASS);
        timeBeginPeriod(1);
    }
}

int WINAPI WinMain(
    _In_ HINSTANCE hInstance,
    _In_opt_ HINSTANCE hPrevInstance,
    _In_ LPSTR lpCmdLine,
    _In_ int nCmdShow)
{
    (void)nCmdShow;
    (void)lpCmdLine;
    (void)hPrevInstance;

    requestRealTimePerformance();

    EngineSimApplication application;
    application.initialize((void *)&hInstance, ysContextObject::DeviceAPI::DirectX11);
    application.run();
    application.destroy();

    return 0;
}
