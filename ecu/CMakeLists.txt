# Virtual ECU: self-contained library, no dependency on the simulation.
# The simulation drives it through ecu/io.h (sensors in, actuators out).

add_library(engine-sim-ecu STATIC
    # Core
    src/calibration.cpp
    src/dcm_file.cpp
    src/ecu.cpp
    src/host.cpp
    src/link.cpp
    src/xcp_interface.cpp

    # Functions (control logic)
    src/functions/input_processing.cpp
    src/functions/engine_state.cpp
    src/functions/rev_limiter.cpp
    src/functions/lambda_control.cpp
    src/functions/fuel_control.cpp
    src/functions/ignition_control.cpp

    include/ecu/calibration.h
    include/ecu/dcm_file.h
    include/ecu/ecu.h
    include/ecu/function.h
    include/ecu/functions.h
    include/ecu/host.h
    include/ecu/io.h
    include/ecu/link.h
    include/ecu/measurement.h
    include/ecu/signals.h
    include/ecu/xcp.h
)

target_include_directories(engine-sim-ecu PUBLIC include)

if (WIN32)
    # I/O link over UDP (Windows sockets)
    target_link_libraries(engine-sim-ecu PUBLIC ws2_32)
endif ()
target_compile_features(engine-sim-ecu PUBLIC cxx_std_17)

# XCP on Ethernet (measurement and calibration with ATI VISION, CANape...)
# through XCPlite, git submodule dependencies/submodules/XCPlite:
#   git submodule update --init dependencies/submodules/XCPlite
if (ECU_XCP_ENABLED)
    set(XCPLITE_DIR "${CMAKE_SOURCE_DIR}/dependencies/submodules/XCPlite")
    if (NOT EXISTS "${XCPLITE_DIR}/CMakeLists.txt")
        message(FATAL_ERROR "ECU_XCP_ENABLED: XCPlite not found, run "
            "'git submodule update --init dependencies/submodules/XCPlite'")
    endif ()

    # Settings for ATI VISION (absolute addressing on extension 0), for the
    # library and the ECU
    set(XCPLITE_CFG_OVERRIDE "${CMAKE_CURRENT_SOURCE_DIR}/config/xcplib_app_cfg.h")
    add_subdirectory(${XCPLITE_DIR} xcplite)
    set_property(TARGET xcplite PROPERTY FOLDER "xcplite")

    target_link_libraries(engine-sim-ecu PUBLIC xcplite::xcplite)
    target_compile_definitions(engine-sim-ecu PUBLIC ECU_XCP_ENABLED)

    # The generated A2L includes XCP_104.aml from its own directory
    set(ECU_XCP_AML "${XCPLITE_DIR}/XCP_104.aml" CACHE INTERNAL "")

    if (WIN32)
        # timeBeginPeriod(1): XCPlite's sleeps use Sleep()
        target_link_libraries(engine-sim-ecu PUBLIC winmm)
    endif ()
endif ()

# Standalone ECU: the ECU as a program of its own, its inputs and outputs
# through an I/O controller (see standalone/io_link.h)
add_executable(ecu-standalone
    standalone/main.cpp
    standalone/stimulus_link.cpp
    standalone/udp_link.cpp

    standalone/io_link.h
    standalone/status.h
)

target_link_libraries(ecu-standalone engine-sim-ecu)

if (MSVC)
    # Sources are UTF-8 (degree sign in the status window)
    target_compile_options(ecu-standalone PRIVATE /utf-8)
endif ()

if (WIN32)
    # Status window (Win32 + GDI); timeBeginPeriod(1) for the 10 ms pacing
    target_sources(ecu-standalone PRIVATE
        standalone/status_window.cpp
        standalone/status_window.h)
    target_link_libraries(ecu-standalone user32 gdi32 winmm)
endif ()

if (ECU_XCP_ENABLED)
    # The A2L is written to the working directory and includes XCP_104.aml
    add_custom_command(TARGET ecu-standalone POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "${ECU_XCP_AML}" "$<TARGET_FILE_DIR:ecu-standalone>")
endif ()
