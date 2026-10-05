#pragma once

// XCPlite configuration overrides for ATI VISION (applied after xcplib_cfg.h,
// selected with XCPLITE_CFG_OVERRIDE in ecu/CMakeLists.txt). Validated with
// VISION in the Simulation_XCP test project (see its XCPLITE_ENGINE_SIM.md).

// VISION ignores ECU_ADDRESS_EXTENSION in the A2L and always sends address
// extension 0 (WRITE_DAQ, SET_MTA). In XCPlite's default mode (CASDD)
// extension 0 means "relative to the calibration segment" and globals are on
// extension 1, so every measurement's WRITE_DAQ fails with ERR_ACCESS_DENIED.
// ACSDD mode: extension 0 = absolute address; calibration segment parameters
// are addressed by the absolute address of their default page and XCPlite
// redirects the access to the active page.
#define OPTION_CAL_SEGMENTS_ABS

// XCPlite's persistence (.bin) is not supported in ACSDD mode; the ECU keeps
// its calibration in the DCM file instead
#undef OPTION_ENABLE_PERSISTENCE

// Timestamp left at the default (DWORD, 1 ns, wraps every 4.3 s).
// OPTION_CLOCK_TICKS_1US makes the generated A2L inconsistent ("0x1 UNIT_1NS"
// while GET_DAQ_RESOLUTION_INFO answers 1000 ticks).

// Memory for the calibration pages: the ECU's segment (up to
// ecu::CalibrationCapacity floats) needs three pages in ACSDD mode (ECU,
// XCP and free page; the default page is the ECU's own static memory)
#undef OPTION_CAL_MEM_SIZE
#define OPTION_CAL_MEM_SIZE (128 * 1024)

// DAQ list memory: room for every ECU measurement in several DAQ lists
#undef OPTION_DAQ_MEM_SIZE
#define OPTION_DAQ_MEM_SIZE (64 * 1024)
