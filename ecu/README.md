# Virtual ECU

Engine control unit for engine-sim, built as a separate library
(`engine-sim-ecu`) with no dependency on the simulation. It only knows its
sensors, its actuators and its calibration, like a real ECU on a HIL bench.

```
 engine-sim (plant)                       ecu/ (this library)
┌──────────────────────┐   EcuInputs    ┌──────────────────────────────────────────┐
│ EcuHardware          │ ─────────────► │ Ecu::step()                              │
│  sensor models:      │                │  1. inputs → EcuSignals                  │
│  crank/cam, MAP,     │                │  2. scheduler: Step / 10 ms / 100 ms     │
│  IAT, CLT, TPS, λ    │                │     InputProcessing → EngineState →      │
│                      │                │     RevLimiter → LambdaControl →         │
│  actuator models:    │   EcuOutputs   │     FuelControl → IgnitionControl        │
│  port injectors,     │ ◄───────────── │  3. output stage: injector pulses at     │
│  spark advance       │                │     their crank angle, spark advance     │
└──────────────────────┘                └──────────────────────────────────────────┘
                                          calibration (DCM)   measurements (log)
```

| Where | What |
|---|---|
| [include/ecu/io.h](include/ecu/io.h) | The boundary: `EcuConfig` (cylinders, firing TDCs, remote I/O), `EcuInputs` (sensors), `EcuOutputs` (injector pulses or injection command, spark advance) |
| [include/ecu/signals.h](include/ecu/signals.h) | `EcuSignals`: the ECU's internal variables, shared by the functions |
| [include/ecu/functions.h](include/ecu/functions.h) | The functions and their calibration handles |
| [src/functions/](src/functions/) | One file per function: the control logic |
| [include/ecu/calibration.h](include/ecu/calibration.h) | Calibration memory, `Value` / `Curve` / `Map`, interpolation |
| [include/ecu/dcm_file.h](include/ecu/dcm_file.h) | ASAM DCM 2.0 reader / writer (FESTWERT, KENNLINIE, KENNFELD) |
| [src/ecu.cpp](src/ecu.cpp) | Function list (execution order), scheduler, injection output stage |
| [include/ecu/host.h](include/ecu/host.h) | `EcuHost`: the ECU's environment on a PC (calibration file, XCP server, datalog), shared by the simulator and the standalone ECU |
| [standalone/](standalone/) | `ecu-standalone`: the ECU as a program of its own, for a real engine through an I/O controller |

On the simulation side, [EcuHardware](../include/ecu_hardware.h) is the only
code that touches both the engine and the ECU, and
[EcuLearner](../include/ecu_learner.h) is a bench tool that records the engine
script's mixture and timing into ECU maps (`ENGINE_SIM_ECU_LEARN`).

## Functions

| Function | Task | Does | Calibration |
|---|---|---|---|
| InputProcessing | step | rpm and MAP averaged over the last firing interval (removes the intake pulsation) | — |
| EngineState | step | stopped / cranking / running with hysteresis, time since start | `CrankingRpm` |
| RevLimiter | step | fuel cut above a speed | `RevLimitRpm` (0 = off), `RevLimitHysteresis` |
| LambdaControl | 10 ms | PI controller on the wideband sensor → fuel factor | `LambdaCtrlEnable`, `LambdaTarget`, `LambdaCtrlKp`, `LambdaCtrlKi`, `LambdaCtrlLimit`, `LambdaCtrlStartDelay` |
| FuelControl | 10 ms | injection time = base map × corrections | `EcuFuelEnable`, `InjectorFlow`, `InjectorDeadTime`, `FuelBaseMap`, `CrankingFuelFactor`, `AfterStartTime`, `FuelCltCorr`, `FuelIatCorr` |
| IgnitionControl | 10 ms | advance = base map + corrections | `EcuIgnEnable`, `IgnAdvanceMap`, `IgnCltCorr`, `IgnIatCorr` |
| Output stage | step | one injector pulse per cylinder and cycle at `InjectionAngle` before its firing TDC | `InjectionAngle` |

Fuel:

    open time   = (FuelBaseMap − InjectorDeadTime) × start × FuelCltCorr(CLT) × FuelIatCorr(IAT) × λ controller
    injection   = open time + InjectorDeadTime            (0 while the rev limiter cuts fuel)

`start` is `CrankingFuelFactor` while cranking, fading to 1 over
`AfterStartTime` once running. The base map is still the injection time
table (rpm × MAP, dead time included), as on an aftermarket ECU.

Ignition:

    advance = IgnAdvanceMap(rpm, MAP) + IgnCltCorr(CLT) + IgnIatCorr(IAT)

New parameters default to neutral values (corrections 1.0 / 0°, lambda control
and rev limiter off), so an older DCM file behaves as before; the parameters
missing from a file keep their values and the load message says how many.

## Adding a correction to an existing function

Example: a fuel factor against throttle position.

1. `functions.h`: add a handle to `FuelControl`: `Curve m_tpsCorrection;`
2. `fuel_control.cpp`, `declare()`:
   ```cpp
   m_tpsCorrection = cal.addCurve("FuelTpsCorr",
       "Fuel factor vs throttle position",
       "-", 0, 5, { "%", 0, 100, { 0, 25, 50, 75, 100 }, "tps_percent" }, { 1.0f });
   meas.add("fuel_corr_tps", "-", "Throttle position factor", &s.fuelCorrTps);
   ```
3. `signals.h`: add `double fuelCorrTps = 1;` under FuelControl.
4. `fuel_control.cpp`, `run()`: compute `s.fuelCorrTps = m_tpsCorrection.lookup(cal, s.tps_percent);`
   and multiply it into the open time.
5. Rebuild. The new `KENNLINIE FuelTpsCorr` appears in learned / default DCM
   files, can be added to `calibration.dcm` by hand and reloads live; the new
   measurement appears in the `ENGINE_SIM_ECU_LOG` datalog.

## Adding a function

1. Declare a class derived from `EcuFunction` in `functions.h` (name and task
   rate in the constructor, its calibration handles as members).
2. Implement `declare()` (parameters with defaults and limits, measurements),
   optionally `reset()`, and `run()` in a new file in `src/functions/`.
3. Add its signals to `EcuSignals`.
4. Register it in `Ecu::Ecu()` in [src/ecu.cpp](src/ecu.cpp); the list order
   is the execution order within each task.
5. Add the .cpp to [CMakeLists.txt](CMakeLists.txt).

A new output (another actuator) also needs a field in `EcuOutputs` and its
actuator model in `EcuHardware`; a new sensor a field in `EcuInputs` and its
sensor model there.

## Calibration and measurements

- Parameters are declared in code with name, description, unit, limits and
  default; their offsets in the calibration memory follow the declaration
  order. That list is all an A2L file for XCP needs. The last item of an axis
  (`"tps_percent"` above) names the measurement giving the operating point on
  it; it must be a registered measurement.
- The signals (`EcuSignals`) and the working calibration page live in static
  memory, at fixed addresses inside the executable, as XCP absolute
  addressing needs. Hence one ECU per process.
- `Ecu::epk()` identifies the software: `ESIM_ECU_` plus a fingerprint of the
  parameter and measurement layout (names, sizes, offsets). It changes
  whenever a parameter or measurement is added, removed or resized, i.e.
  whenever a calibration tool needs a new A2L; datasets written by the
  simulator record it in their header.
- A declaration error (duplicate name, axis input that is not a measurement,
  calibration larger than `CalibrationCapacity`) stops the ECU at start-up.
- DCM files are written to a temporary file and renamed, so an interrupted
  save can't corrupt them; `backupFile()` keeps dated copies in `backup/`.
- `calibration.dcm` is reloaded whenever it is saved. A file that fails
  validation (out of limits, axis not increasing, wrong size) is rejected and
  the ECU keeps running on the previous calibration.
- Every registered measurement is written to the datalog
  (`ENGINE_SIM_ECU_LOG=<file.csv>`, 20 Hz), followed by plant values the ECU
  can't see (true cylinder λ, AFR, pedal, dyno torque).

## Sensors (EcuHardware)

| Sensor | Model |
|---|---|
| Crank / cam | cycle angle 0–720°, instantaneous speed |
| MAP | plenum pressure, instantaneous (the ECU averages it per firing interval) |
| IAT | plenum gas temperature, first order 2 s |
| CLT | constant 90 °C (no thermal model yet) |
| TPS | throttle plate opening |
| λ (wideband) | λ of the last combustion, first order 0.1 s; 0 until the first combustion |

The injectors are the engine's (`injector_flow` [cc/min] and
`injector_dead_time` [ms] on the engine node of the script): a pulse delivers
`flow × (pulse − dead time)` of fuel to the port, where it waits for the
intake valve to open. `InjectorFlow` and `InjectorDeadTime` in the
calibration describe the same injectors, as in a real ECU.

## Standalone ECU

For a lab engine the ECU can run on a PC as a program of its own, with a
microcontroller (the I/O controller) doing the angle domain: crank and cam
decoding, MAP averaged over each firing interval, injector and coil timing,
the ADC and a fail-safe that cuts the outputs when the PC stops answering.

```
 ecu-standalone (PC)                         I/O controller (microcontroller)
┌──────────────────────────────┐  sensors   ┌───────────────────────────────────┐
│ Ecu: the same functions,     │ ◄───────── │ every 10 ms: rpm and MAP averaged │
│ calibration, XCP, datalog    │  command   │ per firing interval, IAT, CLT ... │
│ (EcuConfig::remoteIo)        │ ─────────► │ injection ms + angle, advance     │
└──────────────────────────────┘            └───────────────────────────────────┘
```

With `EcuConfig::remoteIo` the ECU takes rpm and MAP as they come
(`InputProcessing` does no averaging of its own), ignores `cycleAngle_deg` and
schedules no injector pulses: `EcuOutputs::injection_ms` and
`injectionAngle_deg` are the command, the same for every cylinder, which the
I/O controller carries out. The samples pace the ECU: `Ecu::step()` runs once
per sample with the I/O controller's time step, and the XCP clock follows it
(`--xcp-clock ecu`).

`EcuHost` gives the program what the simulator has: `calibration.dcm`
reloaded when saved, saved on `CalSaveRequest`, XCP server with the A2L, the
datalog. The DCM files are interchangeable with the simulator's (parameters go
by name); the A2L is not (each executable has its own addresses and EPK).

```
build/ecu/Release/ecu-standalone.exe --cal calibration.dcm --xcp --log ecu.csv
```

| Option | Meaning |
|---|---|
| `--cal <file.dcm>` | calibration file (default `calibration.dcm`, written with the defaults if missing) |
| `--xcp [address[:port]]` | XCP on Ethernet server (default `0.0.0.0:5555`) |
| `--xcp-clock ecu\|pc` | DAQ timestamps in I/O controller time (default) or PC time |
| `--log <file.csv>` | datalog of every measurement, 20 Hz |
| `--cylinders <n>`, `--firing-tdc <a,b,...>` | engine coding (default 1 cylinder; TDCs evenly spaced from 0) |
| `--udp <address[:port]>` | where the ECU listens for the I/O controller (default `127.0.0.1:5600`; `0.0.0.0` = other computers too) |
| `--stimulus [k=v,...]` | no I/O controller: fixed inputs `rpm`, `map`, `iat`, `clt`, `tps`, `lambda`, `battery` |
| `--duration <s>`, `--status <s>` | stop after this ECU time; console status line period (0 = off; default 1 s, off with the window) |
| `--no-window` | console only, no status window |

On Windows a status window opens with the program (Win32 and GDI, no extra
library): running / stopped / failed, the engine link (communicating, no data
for some time, stimulus without an engine), XCP (off, waiting, tool
connected), the calibration file (loaded, rejected, owned by the XCP tool;
fuel and ignition enabled), the datalog, the live values, the stimulus inputs
and the messages. Closing it stops the ECU; the ECU loop runs on its own
thread ([standalone/status.h](standalone/status.h) is what it publishes).

The I/O controller is reached through an `IoLink`
([standalone/io_link.h](standalone/io_link.h)):

- `UdpLink` (default): the link protocol over UDP, from the simulator or a
  microcontroller with Ethernet. The ECU answers whoever sends the samples;
  gaps in the sample counter are reported as lost samples.
- `StimulusLink` (`--stimulus`): fixed sensor values every 10 ms of PC time,
  for bench work on the calibration without an engine. While it runs, a line
  such as `rpm=3000 map=85` changes the inputs (console, or the window's
  Stimulus box), an empty line shows them, `q` quits.

### Link protocol

[include/ecu/link.h](include/ecu/link.h): CAN frames of 8 bytes, little
endian, so a microcontroller can use the same layout on a CAN bus; over UDP
the frames are packed back to back in a datagram (13 bytes each). Every 10 ms
the I/O controller sends `Time` (0x102), `Sensors2` (0x101) and `Sensors`
(0x100, completes the sample, carries the counter); the ECU answers with
`Command` (0x200) carrying the counter of the sample it answers. Scalings are
in the header; values out of range saturate.

### The simulator as I/O controller

With `set_ecu_settings(link: true)` (or `ENGINE_SIM_ECU_LINK=1`)
[EcuHardware](../include/ecu_hardware.h) stops stepping the ECU in the process
and plays the microcontroller: it averages rpm and MAP over each firing
interval, sends a sample every 10 ms of simulated time, fires the injectors at
the commanded angle with the engine's own injectors (`injector_flow`,
`injector_dead_time` on the engine node) and applies the advance.

- `link_sync: "lockstep"` (default): after each sample the simulation waits up
  to 50 ms for its command, so the results are those of the ECU in the process
  (checked on the CFR at 600 rpm, WOT: same MAP, lambda and torque; injection
  within the protocol's 1 us). After 3 unanswered samples it stops waiting.
- `link_sync: "free"`: the simulation never waits and applies the last command
  received, as a real bench would.
- Fail-safe: without a command for 0.1 s of simulated time the ECU outputs are
  cut (engine script mixture and timing take over); the next command restores
  them.

The simulator's calibration file, XCP and ECU datalog are then not used;
`ENGINE_SIM_ECU_LOG` logs the I/O controller side instead (link state, the
sample's rpm and MAP, the applied command, plant values).

## XCP (build)

XCP on Ethernet for VISION / CANape comes from
[XCPlite](https://github.com/vectorgrp/XCPlite), a git submodule pinned to
commit `88e3cbe`, and is off by default:

```
git submodule update --init dependencies/submodules/XCPlite
cmake -S . -B build -DECU_XCP_ENABLED=ON
```

With `ECU_XCP_ENABLED` the ECU library links XCPlite (and `winmm`) and gets
the `ECU_XCP_ENABLED` define; [config/xcplib_app_cfg.h](config/xcplib_app_cfg.h)
sets XCPlite up for ATI VISION (absolute addressing on address extension 0, no
XCPlite persistence). `XCP_104.aml`, which the generated A2L includes, is
copied next to the executable and to the build directory. XCPlite needs MSVC
on Windows (the Visual Studio generator already uses it).

## XCP (use)

Turn it on in the engine script (build with `ECU_XCP_ENABLED`), in its
`main` node or in `main.mr`:

```
public node main {
    set_engine(...)
    ...
    set_ecu_settings(xcp: true)     // xcp_address, xcp_port, xcp_clock optional
}
```

It is read when the simulator starts (a change on script reload needs a
restart). The environment variables override the script:

| Variable | Default | Meaning |
|---|---|---|
| `ENGINE_SIM_XCP` | script | `1` / `0` turn the XCP server on / off |
| `ENGINE_SIM_XCP_ADDRESS` | `0.0.0.0` | interface to listen on: every interface (tool on this or another PC); `127.0.0.1` = this PC only |
| `ENGINE_SIM_XCP_PORT` | `5555` | UDP port |
| `ENGINE_SIM_XCP_CLOCK` | `sim` | DAQ timestamps in simulated time (`sim`) or PC time (`real`) |
| `ENGINE_SIM_XCP_LOG` | `2` | XCPlite debug level (4 = every command) |

At start-up the ECU:

1. loads `calibration.dcm` into its calibration memory;
2. creates the calibration segment `EcuCalibration` from it (default page =
   the ECU's static calibration memory, absolute addresses on extension 0);
3. writes the A2L `engine_sim_ecu.a2l` to the working directory (`XCP_104.aml`
   is copied there from the executable's directory if missing): every
   parameter (`FESTWERT` → VALUE, curves and maps with shared editable axes
   `<name>_X` / `<name>_Y` as AXIS_PTS + COM_AXIS, operating point from the
   axis input measurement), every measurement (FLOAT64), groups
   `Cal_<function>` / `Meas_<function>`, EPK = `Ecu::epk()`;
4. creates the cyclic DAQ events `Ecu_10ms` and `Ecu_100ms`, triggered after
   the ECU tasks of that rate. With `ENGINE_SIM_XCP_CLOCK=sim` the XCP clock
   (timestamps, GET_DAQ_CLOCK) is the simulated time, so samples are exactly
   10 ms apart even though the simulation runs in bursts per frame.

The screen shows "XCP: waiting on <address>:<port>, A2L <path>", then
"tool connected" / "disconnected".

Calibration with a tool connected:

- The tool owns the calibration: a change to `calibration.dcm` is ignored
  (save the file again after disconnecting to load it).
- Each frame the ECU copies the active page into its calibration memory (the
  default page), so the values written by the tool also show where the tool
  reads parameters by DAQ (VISION Data List). The default page therefore no
  longer holds the start-up values; the DCM file is the reference.
- To save: set `CalSaveRequest` from 0 to 1. The calibration in use is written
  to `calibration.dcm` (previous file copied to `backup/` with a date) with
  `CalSaveRequest` = 0; `cal_save_count` counts the saves. Set it back to 0
  before the next save. Nothing is saved on exit.

Without a tool connected, a saved `calibration.dcm` is still loaded live:
the XCP server thread writes it into the working page (XCPlite allows a
single writer).

When the ECU code changes (parameter or measurement added, removed or
resized) the EPK and the addresses change: re-import the new A2L in the tool
("Replace existing data items" and "Delete existing Device Settings and
Memory Regions" in VISION) and, if the tool reports different calibration
data, upload ECU → PC.
