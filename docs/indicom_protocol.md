# Indicating stream protocol (engine-sim → IndiCom)

engine-sim plays the role of the indicating hardware: a piezo pressure sensor in
every cylinder plus a crank angle encoder. It streams the raw signals over UDP;
all combustion analysis (volume, IMEP, heat release, CA10/50/90...) is done by
the receiving tool, from the engine geometry entered by the operator.

## Enabling

Set before starting engine-sim:

| Variable | Example | Meaning |
|---|---|---|
| `ENGINE_SIM_INDI_UDP` | `127.0.0.1:5005` | destination (IPv4 host:port) |
| `ENGINE_SIM_INDI_NOISE_BAR` | `0.05` | gaussian sensor noise, standard deviation [bar] (default 0) |
| `ENGINE_SIM_INDI_OFFSET_BAR` | `0.5` | each sensor gets a fixed random offset in ±value [bar] (default 0); the tool has to peg the signal |

## Packets

Little-endian. Every packet starts with a 16-byte header:

| Offset | Type | Field |
|---|---|---|
| 0 | char[4] | magic `"ESIN"` |
| 4 | uint8 | protocol version (1) |
| 5 | uint8 | packet type: 1 = data, 2 = engine info |
| 6 | uint16 | number of cylinders N |
| 8 | uint32 | sequence number (shared by both types, +1 per packet) |
| 12 | uint16 | number of records (data packets), 0 otherwise |
| 14 | uint16 | reserved (0) |

### Type 1: data

`count` records, one per physics step (sample rate = simulation frequency, see
the info packet), each:

| Type | Field |
|---|---|
| float32 | encoder angle [deg], 0 ≤ angle < 720, increasing with rotation |
| float32 × N | pressure of cylinder 1..N [bar], absolute plus the sensor offset/noise |

Packets carry at most 1400 bytes (8 cylinders: 38 records per packet).

### Type 2: engine info (once per second)

Data an operator would normally type in; the tool may use it to pre-fill its
configuration.

| Type | Field |
|---|---|
| float32 | bore [mm] |
| float32 | stroke [mm] |
| float32 | connecting rod length [mm] |
| float32 | compression ratio (0 until the engine has turned) |
| float32 | simulation frequency = sample rate [Hz] |
| float32 × N | encoder angle of each cylinder's firing TDC [deg] (-1 if not yet known) |
| uint8 + bytes | engine name (length-prefixed) |
