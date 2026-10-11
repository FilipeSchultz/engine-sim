# Building an engine for the virtual test cell

How to write an engine script (`.mr`) that uses the test cell additions of this
fork (gas model, wall heat transfer, flame model, intake air temperature...) and how to
calibrate it against real data with the indicating log. The Waukesha CFR
([assets/engines/cfr/cfr_f1.mr](../assets/engines/cfr/cfr_f1.mr)) is the worked
example throughout.

All additions are opt-in: their defaults reproduce the original engine-sim
behaviour, so existing engines are unaffected until you set them.

## 1. Inputs added by this fork

### `engine(...)`

| Input | Default | Meaning |
|---|---|---|
| `turbulence_ratio` | `0.5` | In-cylinder turbulence intensity / mean piston speed. Main burn rate knob. Higher for heads with strong swirl or tumble |
| `flame_model` | `0` | `0` = original: cylindrical flame, speed computed once at the spark. `1` = flame speed updated every step from the unburned gas state, equivalence ratio and residual gas. `2` = as 1, with a spherical flame, entrainment and burn-up (recommended) — see [section 5](#5-model-notes) |
| `spark_plug_offset` | `0.0` | Flame model 2: radial spark plug position / bore radius (0 = central plug, ~0.8 = side plug) |
| `cycle_variation` | `0.0` | Flame model 2: standard deviation (fraction) of the turbulence seen by each cycle's flame kernel. Sets the cycle-to-cycle variation (COV of IMEP) |
| `gas_model` | `0` | `0` = original: constant γ = 1.4. `1` = heat capacities varying with temperature and composition, rich mixture chemistry and mixture non-uniformity (recommended) — see [section 5](#5-model-notes) |
| `mixture_nonuniformity` | `0.1` | Gas model 1: standard deviation of the local equivalence ratio / mean. ~0.1 for a carburettor or port injection |
| `wall_heat_transfer` | `0.0` | Multiplier of the Hohenberg wall heat transfer coefficient. `0` = original constant coefficient (100 W/m²K, almost adiabatic). With gas model 1 use the published correlation, `1.0` |
| `wall_temperature` | `(90 + 273.15) * units.K` | Mean temperature of the combustion chamber surfaces (head, liner, piston crown, valves) |
| `wall_film_fraction` | `0.0` | Port injection (ECU): fraction of each injector pulse deposited on the port walls at 90 °C coolant and 100 kPa (x of the x-τ model). `0` = no film. Typical warm port injection: 0.2–0.3 — see [section 5](#port-wall-film-wall_film_fraction--0) |
| `wall_film_time_constant` | `0.3 * units.sec` | Evaporation time constant of the film at 90 °C and 100 kPa (τ). Typical warm: 0.2–0.5 s |
| `coolant_temperature` | `(90 + 273.15) * units.K` | Port wall temperature (film) and the ECU's coolant sensor. Constant (no thermal model yet) |
| `lambda_sensor_volume` | `0.0 * units.L` | Exhaust volume between the exhaust valves and the ECU's wideband sensor: sets the gas transport delay — see [section 5](#ecu-lambda-sensor) |
| `lambda_sensor_time_constant` | `0.1 * units.sec` | Response time (t63) of the wideband sensor, from its datasheet |

### `fuel(...)`

These existed before, but their defaults are tuned for a rough-sounding
street engine and usually need changing for a realistic one:

| Input | Default | Meaning |
|---|---|---|
| `energy_density` | `48.1 kJ/g` | Lower heating value. Gasoline / iso-octane ≈ 43–44.3 kJ/g, diesel ≈ 42.8, ethanol ≈ 26.8 |
| `max_burning_efficiency` | `0.8` | Fraction of the charge that can burn. Use `1.0` |
| `burning_efficiency_randomness` | `0.5` | Random cycle-to-cycle loss of burned fraction. `0.5` gives misfire-like COV; `0.05` is a well running engine |
| `max_turbulence_effect` | `2.0` | Turbulence [m/s] above which the charge burns completely. Below it part of the fuel stays unburned (and the randomness above applies). Set it under the engine's turbulence at its normal operating speeds |
| `latent_heat` | `0` | Heat of vaporisation the ECU's injected fuel takes from the charge [J/kg]. Iso-octane 308 kJ/kg, gasoline ~350 kJ/kg; use about half for a carburettor or port injection (part evaporates from the walls). Raises the trapped air a few % at rich mixtures |

With flame model 2, use `cycle_variation` for the cycle-to-cycle variation and
keep `burning_efficiency_randomness` low (it only removes fuel at random).

### `intake(...)`

| Input | Default | Meaning |
|---|---|---|
| `air_temperature` | `(25 + 273.15) * units.K` | Temperature of the air drawn in (test cell air conditioning). Temperatures are absolute: `units.K` only, write °C as `(t + 273.15) * units.K` |

## 2. Writing the engine script

Copy an engine with the same layout (the CFR for a single cylinder, `gm_ls` for
a V8) into `assets/engines/<name>/<name>.mr`, then import it in
[assets/main.mr](../assets/main.mr) instead of the current engine:

```
import "engines/<name>/<name>.mr"
```

The file needs a `public node main` calling `set_engine`, `set_vehicle` and
`set_transmission` (see the end of `cfr_f1.mr`).

### Geometry

- `bore`, `stroke`, `rod_length`, `compression_height` from the engine data.
- Compression ratio: expose it as an input and derive the chamber volume,
  `chamber_volume: displacement / (compression_ratio - 1)`, so it can be
  changed in one place. The indicating log and IndiCom report the actual ratio
  (max / min cylinder volume) — check it matches.
- `deck_height: stroke / 2 + rod_length + compression_height`.

### Valve timing

Write cam lobe centres relative to the firing TDC, as the CFR camshaft node
does: firing TDC at 0 / 720 deg, overlap TDC at 360 deg,

```
_exhaust_cam.add_lobe(rot360 - exhaust_lobe_center)
_intake_cam.add_lobe(rot360 + intake_lobe_center)
```

From published seat timing: intake centre = (IVO ATDC + 180 + IVC ABDC) / 2,
exhaust centre = (EVO BBDC + 180 + EVC ATDC) / 2. The `harmonic_cam_lobe`
duration is given at 0.050 in lift, a bit shorter than the seat duration.

Port flow tables (`add_flow_sample(lift_thou, cfm_at_28inH2O)`) are rarely
published; estimate them from valve diameter. At low speed they barely matter
(the CFR showed no VE change with flow cut by 50% at 600 rpm).

### Intake and throttle

- `intake_flow_rate: k_carb(...)` sizes the throttle body. Too large and the
  pedal has no authority until the last few percent: size it for the engine's
  maximum air flow (CFR: `k_carb(15)` for 0.6 L at ≤ 1200 rpm).
- `plenum_volume`: small enough for the manifold pressure to settle within
  about a second at the lowest speed.
- `idle_throttle_plate_position` and `idle_flow_rate` set the closed-pedal air.
- `throttle_gamma` on `engine(...)` spreads the pedal over the useful range.

### Simulation and dyno

- `simulation_frequency`: the angle resolution is `6 * rpm / frequency` deg per
  step. Indicating needs ≤ ~0.5 deg. Real-time limit on this machine: about
  20–25 kHz for 8 cylinders, 40 kHz is fine for 1 cylinder. Results change a
  little with the frequency, so calibrate at the frequency you will run.
- `dyno_min_speed`, `dyno_max_speed`, `dyno_hold_step`: the speed range the
  CAN dyno control can request.

### Combustion block (template)

```
engine engine(
    ...
    fuel: fuel(
        name: "...",
        energy_density: 44.0 * units.kJ / units.g,
        max_burning_efficiency: 1.0,
        burning_efficiency_randomness: 0.05,
        max_turbulence_effect: 1.2,
        latent_heat: 150 * units.kJ / units.kg
    ),
    turbulence_ratio: 1.4,
    flame_model: 2,
    spark_plug_offset: 0.5,
    cycle_variation: 0.2,
    gas_model: 1,
    mixture_nonuniformity: 0.1,
    wall_heat_transfer: 1.0,
    wall_temperature: (200.0 + 273.15) * units.K,
    simulation_frequency: 20000
)
...
intake intake(
    ...
    air_temperature: (25.0 + 273.15) * units.K
)
```

Start from these values (CFR calibration) and tune as in section 3.

## 3. Calibrating against real data

Run with the indicating log enabled:

```
set ENGINE_SIM_INDI_LOG=C:\temp\indi.csv
```

Hold a steady point with the dyno in speed mode over CAN (HOST), wait for the
manifold pressure to settle, and average ~40 cycles. One row per cylinder and
cycle; the columns are listed in [section 6](#6-indicating-log-columns).

Tune in this order — each step only changes what comes after it:

| Step | Knob | Watch | Target (typical SI, WOT) |
|---|---|---|---|
| 1. Air | intake `air_temperature`, `k_carb`, port flow, cam timing | `fresh_charge_mg`, `ve_percent` | VE 80–95 % (from data if available) |
| 2. Fuel energy | `energy_density` | — | Real fuel LHV |
| 3. Burn completeness | `max_turbulence_effect`, `max_burning_efficiency` | `fuel_burned_mg` vs fresh charge / stoichiometric AFR | Full burn when λ ≤ 1 |
| 4. Burn rate | `turbulence_ratio` (with `flame_model: 2`), `spark_plug_offset` | `ca50_atdc_deg`, `burn_10_90_deg` | CA50 from the reference data at its spark advance; 10–90 % burn 20–30 deg |
| 5. Heat loss | `wall_temperature` (keep `wall_heat_transfer` at 1.0 with gas model 1) | `indicated_eff_gross_percent`, `pmax_bar` | Gross indicated efficiency 30–40 % (lower for low speed / low CR) |
| 6. Cycle variation | `cycle_variation` | COV of `imep_net_bar` over ≥ 40 cycles | 1–3 % at WOT near MBT, higher at part load / retarded spark |
| 7. Spark sweep | ECU `IgnAdvanceMap` | IMEP / torque vs advance | MBT at CA50 ≈ 8–10 deg ATDC, flat top |
| 8. Mixture sweep | ECU `FuelBaseMap`, `mixture_nonuniformity`, `latent_heat` | IMEP / torque vs λ | Peak at λ ≈ 0.85–0.9, a few % less at λ = 1, steep loss lean |
| 9. Check | — | the same at part load and other speeds | CA50 later at part load and higher speed |

`spark_plug_offset` mostly sets the burn duration (a side plug gives a longer
burn and a longer tail), `turbulence_ratio` then places CA50. The COV needs
≥ 40 cycles to be meaningful: with 0.2 the CFR shows 1–4 % between runs.

To see the heat release shape, use IndiCom virtual (or any tool reading the
UDP stream, see [indicom_protocol.md](indicom_protocol.md)).

Effects to keep in mind:

- `turbulence_ratio` also changes burn completeness (step 3), and heat loss
  depends on burn phasing — iterate steps 3–5 once.
- IMEP follows air × fuel energy × efficiency. Diagnose an IMEP or Pmax error
  by checking those three columns before touching combustion.
- `wall_heat_transfer` lowers both Pmax and IMEP; `turbulence_ratio` mostly
  moves Pmax and its angle.
- The ECU fuel map is learned or scaled for one engine model: after changing
  the gas model, intake temperature or `latent_heat`, check λ and rescale
  `FuelBaseMap`.

### Worked example: CFR at RON conditions

Validation against Argonne's instrumented CFR (Pal et al., SAE 2018-01-0187:
iso-octane, 600 rpm, 52 °C intake air, CR 7.55, λ 0.89, cycle-averaged
pressure traces and GT-Power estimates):

| | Argonne | Model (gas model 1, flame model 2) |
|---|---|---|
| Trapped mass at IVC | 628 mg (6 % residual) | 615 mg (10 % residual) |
| Fresh air | ≈ 549 mg (from the above) | 552 mg, VE 83 % |
| Pressure at −100° / TDC (spark 5° ATDC) | ≈ 1.5 / 13.8 bar | 1.48 / 12.3 bar |
| Spark 5° ATDC: Pmax | 26 bar at ≈ 37° ATDC | 25.7 bar at 35.7° ATDC |
| Spark 13° BTDC: Pmax | ≈ 47 bar at ≈ 12° ATDC (knocking, 50 KU) | 39.7 bar at 19.7° ATDC (no knock model) |
| Wall temperatures | liner 157 °C, head 252 °C, piston 212 °C | 200 °C (one mean value) |

The compression pressure is ~11 % low: the polytropic exponent from −100° to
TDC is 1.28 in the model and ≈ 1.35 in the measurement. Runs at this point:
without blowby n rises by 0.007, the wall heat transfer changes it by less than
0.003, and the refit of the air heat capacity (c_v was 3 % high at 400 K) added
0.003. A fuel–air mixture at λ 0.89 has γ ≈ 1.33 at 400 K and 1.30 at 650 K
(JANAF), so n ≈ 1.35 with heat losses cannot be reached at CR 7.55 from 1.5 bar
at −100°: the difference is not in the gas model. At CR 8.2 the model gives
1.49 / 13.6 bar. Argonne's engine was probably at an effective compression
ratio of ~8.2–8.3 (the CFR's ratio is set by the cylinder height; clearance and
crevice volumes move it), or the values read from the paper's plots are off.
The calibration stays at the nominal ratio; a motored pressure trace at a known
compression ratio would settle it.

Operating points with the calibrated script (CR 8, spark 13° BTDC, λ 0.88;
ECU fuel, 20 s of cycles each):

| Point | IMEP net | Pmax | CA50 | VE | EGT (flow weighted / port) | COV |
|---|---|---|---|---|---|---|
| 600 rpm WOT | 8.8 bar | 42 bar at 19.4° | 9.4° | 83 % | 834 / 730 °C | 2.0 % |
| 600 rpm, 30 % pedal | 5.8 bar | 27 bar | 11.6° | 58 % | 793 / 721 °C | 2.6 % |
| 900 rpm WOT | 8.7 bar | 38 bar | 11.5° | 79 % | 879 / 776 °C | 1.7 % |
| 1200 rpm WOT | 8.4 bar | 35 bar | 12.7° | 76 % | 891 / 794 °C | 2.3 % |

Spark sweep (600 rpm WOT): MBT at ≈ 13–18° BTDC, CA50 ≈ 8–10° ATDC.

| Spark [deg BTDC] | 3 | 13 | 23 | 33 |
|---|---|---|---|---|
| CA50 [deg ATDC] | 19.1 | 9.4 | 0.1 | −8.8 |
| IMEP net [bar] | 8.51 | **8.82** | 8.70 | 8.19 |
| EGT flow weighted [°C] | 894 | 834 | 796 | 777 |

Mixture sweep (600 rpm WOT, spark 13°):

| λ | 0.50 | 0.55 | 0.60 | 0.65 | 0.70 | **0.88** | 1.01 | 1.25 | 1.43 | 1.55 |
|---|---|---|---|---|---|---|---|---|---|---|
| IMEP net [bar] | −0.2 | 2.18 | 4.86 | 6.89 | 8.42 | **8.84** | 8.57 | 7.04 | 4.29 | 2.27 |
| Misfires [%] | 100 | 70 | 40 | 18 | 1 | 0 | 0 | 4 | 35 | 62 |
| COV of IMEP [%] | — | 175 | 88 | 50 | 10 | 1 | 3 | 22 | 80 | 147 |

Peak torque at λ ≈ 0.85–0.9, −5 % at λ 0.70 and −3 % at λ 1. Half the
cycles misfire at λ ≈ 0.58 rich and ≈ 1.5 lean (ignition limits of flame
model 2); COV passes 10 % below λ 0.70 and above λ ≈ 1.15. EGT peaks near
λ 1 and rises with spark retard and speed, as in a real engine.

Other references: Bestel et al., OSTI 1597460; Waqas et al., OSTI 1606342
(heat release at 600 rpm, PRF90).

## 4. ECU dataset

The virtual ECU (`ecu-standalone --cal ...`) reads
[assets/calibration/calibration.dcm](../assets/calibration/calibration.dcm)
(ASAM DCM, hot reloaded on save). Each engine keeps its own copy in
`assets/calibration/datasets/<name>.dcm`; copy it over `calibration.dcm` when
switching engines.

To create one for a new engine:

1. Start from an existing dataset and set the RPM axis of `FuelBaseMap` and
   `IgnAdvanceMap` to the engine's speed range (the MAP axis usually stays),
   `InjectorFlow`, `InjectorDeadTime`, `CrankingRpm`.
2. Set `EcuFuelEnable` and `EcuIgnEnable` to 0 (engine runs on the script's
   mixture and timing curve).
3. Enable the ECU functions and calibrate the base maps on steady points
   (dyno hold, fixed pedal) with the ECU's datalog or XCP: `FuelBaseMap` from
   the measured λ against the target (with `LambdaCtrlEnable` = 1,
   `fuel_corr_lambda` is the factor the cell is missing), `IgnAdvanceMap`
   from torque (MBT) or knock limit.
4. Copy the result to `datasets/<name>.dcm` and `calibration.dcm`.
5. Then tune the corrections (temperature curves, closed loop λ, rev limiter,
   injection angle) as on a real ECU.

`ENGINE_SIM_ECU_LOG=<file.csv>` logs every ECU measurement plus the true
cylinder λ, pedal and dyno torque. The ECU's structure, its functions and how
to add tables or logic are described in [ecu/README.md](../ecu/README.md).

## 5. Model notes

### Gas model 1 (`gas_model: 1`)

All gas volumes of the engine (intake, runners, cylinders, exhaust) use a mean
heat capacity that depends on temperature and composition. The internal
energy per mol, from 0 K, is

    u = ½ R T (a + b T)

with a, b mole-fraction averages of three components, fitted to tabulated
(JANAF) data. The heat capacity is c_v = R (a/2 + b T):

| Component | a | b [1/K] | Check |
|---|---|---|---|
| air (1 O2 : 3 inert) | 4.559 | 0.000654 | c_v least squares over 300–1000 K (the unburned gas: intake, compression, end gas), within 1 %: γ 1.40 at 300 K, 1.37 at 700 K. c_v +11 % at 2500 K (only the excess air of a lean burned gas) |
| burned gas (N2, CO2, H2O) | 4.98 | 0.00078 | instantaneous γ ≈ 1.22 at 2500 K |
| fuel vapour (iso-octane) | 2.0 | 0.063 | c_v ≈ 22 R at 300 K, 42 R at 700 K |

Temperature from energy is the root of the quadratic; pressure is n R T / V.

Gas moving between volumes (valves, throttle, pipes) carries its enthalpy
h = u + R T per mol, i.e. with the flow work: the gas left in the cylinder
expands and cools during the exhaust blowdown, and the charge entering a
volume heats it. (Gas model 0 transfers the internal energy only, as the
original engine-sim: the exhaust left at the temperature of exhaust valve
opening, ~1200 °C on the CFR, and the intake charge stayed too cold, VE
107 %.) The flow rate equations keep γ = 1.4.

The model 0 gas (c_v = 2.5 R) let
combustion heat the gas by ~4000 K; with model 1 the burned gas rises ~2000 K,
close to real flame temperatures, so the correlations can be used as
published (wall heat transfer × 1.0).

Rich mixtures and mixture preparation:

- Partial oxidation: with φ > 1 all the fuel in the burning gas ends up as
  CO / CO2 and H2 / H2O (as many product moles as complete combustion), but
  the O2 releases less heat: relative to complete combustion of the O2-limited
  fuel, η = 1.28 − 0.28 φ (heats of formation per CH2 unit: 90 kJ to CO + H2,
  ~525 kJ per mol O2 for CO → CO2 and H2 → H2O).
- Mixture non-uniformity: the local φ is normally distributed with standard
  deviation `mixture_nonuniformity` × φ. Lean pockets leave O2 unused, rich
  pockets partially oxidise, so the heat released peaks slightly rich of
  stoichiometric.
- Evaporation (`latent_heat`): the injected fuel takes its heat of
  vaporisation from the charge, which gets denser.

### Wall heat transfer (`wall_heat_transfer` > 0)

Hohenberg (1979), applied every fluid step:

    h = k · 130 · V^-0.06 · p^0.8 · T^-0.4 · (Sp + 1.4)^0.8      [W/(m²K)]
    Q = h · A · (T_wall − T) · dt

V cylinder volume [m³], p [bar], T mean gas temperature [K], Sp mean piston
speed [m/s], A = liner + 2 × bore area, T_wall = `wall_temperature`,
k = `wall_heat_transfer`. Use k = 1 (published) with gas model 1. With the
original gas (γ = 1.4) k ≈ 2.5 was needed to bring the efficiency down, at the
cost of far too much heat loss after combustion.

### Flame model 1 (`flame_model: 1`)

At the spark the chamber records the turbulence u' (= `turbulence_ratio` × mean
piston speed), the equivalence ratio φ, the burned gas fraction x_b, and the
pressure and temperature. Every step after that:

    dT_u / T_u = (γ_u − 1) / γ_u · dp / p              unburned gas, isentropic from the spark
    S_L = (B_m + B_φ (φ − φ_m)²) (T_u/298)^α (p/1 atm)^β (1 − 2.06 x_b^0.77)
    S_T = f(u' / S_L) · S_L                            f = fuel's turbulence_to_flame_speed_ratio

with Metghalchi & Keck gasoline constants (φ_m = 1.21, B_m = 30.5 cm/s,
B_φ = −54.9 cm/s) and α, β as functions of φ (Heywood eq. 9.37–9.38). γ_u is
1.4 with gas model 0 and the instantaneous value of the unburned mixture at
T_u with gas model 1. The
burned fraction is the gas that is neither fuel nor fresh air (fresh air is
1 mol O2 : 3 mol inert, combustion products are inert).

Compared with model 0 this:

- uses φ in the laminar flame speed correlation (model 0 passes λ, so rich
  mixtures burn too slowly and lean ones too fast);
- speeds the flame up as the unburned gas is compressed;
- slows the burn with residual gas (part load, overlap);
- makes CA50 move with load and speed. CFR, spark 13 deg: CA50 17 deg at
  600 rpm WOT, 26 deg at part load, 25 deg at 900 rpm, 31 deg at 1200 rpm
  (model 0: 17 deg everywhere).

`turbulence_ratio` has to be re-tuned when switching models (CFR: model 0
1.0, model 1 0.6, model 2 0.9; model 2 with gas model 1 1.4).

### Flame model 2 (`flame_model: 2`)

Quasi-dimensional entrainment / burn-up model (Blizard & Keck, Tabaczynski)
with a spherical flame. The laminar speed S_L and the unburned temperature
T_u are those of model 1. Per step, with f_e and f_b the fractions of the
charge entrained by the front and burned:

    V_front = V · (f_b·E + f_e − f_b) / (f_b·E + 1 − f_b)     E = burned / unburned density ratio
    r       = flame radius enclosing V_front                  (geometry table)
    A       = free flame front area at r                      (geometry table)
    u_eff   = u' · (1 − exp(−r / L_I))                        kernel: only eddies smaller than the flame
    df_e    = ρ_u · A · (u_eff + S_L) · dt / n_charge         entrainment
    df_b    = (f_e − f_b) · (1 − exp(−dt / τ)),  τ = λ_T / S_L burn-up of the entrained eddies

- Geometry: a sphere centred on the head at the plug (`spark_plug_offset`),
  clipped by the head, the liner and a flat piston at the current chamber
  height h = V / bore area. Volume and front area are tabulated at start-up
  (128 × 128, non-dimensional) and interpolated.
- Turbulence scales: integral scale L_I = 0.2·h, Taylor microscale
  λ_T = L_I·√(15 / Re_t), Re_t = u'·L_I / ν_u.
- E = n_b T_b / (n_u T_u), with T_b from the energy balance at constant
  volume (heat released and moles produced per mol of charge, burned gas
  composition) through the gas model.
- End gas: once the front has swept 98 % of the volume, the remaining charge
  is entrained over L_I / S_L (corners, near-wall layers). This is the tail.
- Cycle-to-cycle variation: each cycle's u' is multiplied by
  (1 + g · `cycle_variation` · N(0, 1)), the flow at the plug when the kernel
  forms. Slow kernels give late, weak cycles, as in a real engine. Slow
  mixtures depend more on it: g = 0.72 / R, between 1 and 3, with R the
  relative laminar speed below.
- Ignition: the kernel forms in the mixture at the plug, whose equivalence
  ratio varies from cycle to cycle, φ_plug = φ · (1 + `mixture_nonuniformity`
  · N(0, 1)). Its relative laminar speed R = S_L,0(φ_plug) · (1 − 1.48·x_r^0.77)
  / S_L,0(1.1) (the mixture term of the correlation; the burned gas weighs
  less on the kernel than the 2.06 of the laminar speed, as turbulence and
  spark help) must be at least 0.418, else the spark fails and the charge
  leaves unburned. This puts the misfire limits at λ ≈ 0.58 rich and
  1.4–1.5 lean with 6–10 % residual gas and at ~28 % burned gas at
  stoichiometric, those of port injection SI engines, with a transition of a
  few hundredths of λ where some cycles misfire. (With 2.06 the dilution
  limit fell at ~21 %: at part load the engine sat on it, misfiring in turns,
  and the misfires hid the effect of the spark advance.)
- Partial burn: a flame still burning when the exhaust valve opens is
  quenched by the blowdown; the rest of the charge leaves unburned (very late
  or very slow combustion).
- Laminar speed outside the fitted range (φ 0.8–1.5): the temperature and
  pressure exponents are held at the end of the range and the pressure
  exponent is kept ≤ 0 (extrapolated, it turns positive beyond φ ≈ 1.7 and
  a rich flame would speed up with pressure).

### Port wall film (`wall_film_fraction` > 0)

x-τ model (Aquino, SAE 810494) per cylinder port, for the ECU's port
injection. Each pulse puts a fraction x on the walls, the rest waits in the
port for the intake valve as before; the film evaporates with time constant τ
into the port:

    dm_film/dt = x · ṁ_inj − m_film / τ

In steady state the cylinder gets all the injected fuel; on a step it gets
(1 − x) at once and the rest over τ (lean on tip-in, rich on tip-out, fuel
still arriving after a fuel cut). x and τ follow the port conditions, so the
plant is richer than an x-τ compensation in the ECU with fixed parameters:

- Evaporation rate r ∝ fuel vapour pressure at the wall (coolant)
  temperature, Clausius-Clapeyron with the heat of vaporisation of
  iso-octane: r = exp(4200 K · (1/363 K − 1/T)).
- τ = τ_ref · (p_port / 100 kPa) / r: slower at high manifold pressure
  (diffusion into the air) and when cold.
- x = x_ref · r^−0.3, at most 0.9.

From 90 to 20 °C, τ grows ×16 and x ×2.3, the order of magnitude identified
on real engines (Guzzella & Onder, *Introduction to Modeling and Control of
Internal Combustion Engine Systems*; Hendricks & Sorenson, SAE 900616). The
fuel on the walls is logged as `wall_film_mg` in `ENGINE_SIM_ECU_LOG`.

Identifying x_ref and τ_ref on a real engine: at fixed speed and load, with
closed loop λ off, step the fuel (e.g. FuelBaseMap ×1.1 through XCP or the
DCM file) and record the command and λ at ≥ 100 Hz. After the transport delay
and the sensor (see below, identify them first), λ jumps by (1 − x) of the
step and settles over τ. Repeat over speed × load, warm and cold.

### ECU lambda sensor

The wideband sensor sees the exhaust gas, not the cylinder. The gas pushed out
of each cylinder is tagged with the O2 and fuel of its charge (taken at intake
valve closing, so a fuel cut reads air and a misfire its charge λ) and moves
to the sensor as plug flow: it arrives once `lambda_sensor_volume` more
exhaust (at the runner's temperature and pressure) has left the cylinders. The
delay therefore shrinks as 1 / exhaust flow, and between exhaust pulses the
gas stands still. The gas at the sensor drives a first order response of
`lambda_sensor_time_constant`, applied to 1 − 1/λ (about linear in the pump
current: excess O2 lean, O2 demand rich). Range 0.65 to 6.5 (air).

From the injection command to the sensor: injection angle → intake valve
opening → one cycle in the cylinder → transport → sensor. With several
cylinders the sensor sees their individual λ in turn, smoothed by its
response. The true cylinder λ stays in `cylinder_lambda` of
`ENGINE_SIM_ECU_LOG`.

Identifying the delay and t63 on a real engine: the same fuel step; the time
from the command to the first λ change minus about one engine cycle is the
transport delay, and it should scale with 1 / exhaust mass flow over the
speed × load grid; `lambda_sensor_volume` ≈ delay × exhaust volume flow.

### Known limits

- Models 0 and 1: the flame is a growing cylinder that stops abruptly at the
  bore and the piston (no tail, sharp Pmax), and cycle-to-cycle variation only
  comes from `burning_efficiency_randomness` (COV ~0.1 % once turbulence
  exceeds `max_turbulence_effect`). Model 2 fixes both.
- Gas model 0: constant γ = 1.4 and no dissociation, so the burned gas gets
  far too hot and the cycle too efficient, torque is flat for every λ < 1.
  Gas model 1 fixes this; dissociation is only implicit in the burned gas
  heat capacity fit.
- The flow rate equations (valves, throttle, pipes) keep γ = 1.4; with gas
  model 0 they transfer internal energy without flow work (see gas model 1).
- engine-sim's air is 1 O2 : 3 inert (real air 3.76): at a given λ the fuel
  mole fraction is ~19 % high and γ of the mixture ~0.01 low.
- No wall heat transfer in the intake and exhaust ports and pipes.
- Flat piston and head only (no bowl, pent-roof or squish).
- No knock model: too much spark advance only loses torque.
- Misfire comes only from the mixture at the plug (rich, lean, residual
  gas); spark energy, plug fouling and wall wetting at cold start are not
  modelled.
- No thermal model: `coolant_temperature` is constant, so cold start and
  warm-up (film, friction, heat loss) are only steady states.
- The fuel evaporated from the wall film cools the charge like the directly
  injected fuel (`latent_heat`).

## 6. Indicating log columns

| Column | Unit | Meaning |
|---|---|---|
| `time_s`, `cycle`, `cylinder`, `rpm` | | Cycle identification, rpm averaged over the cycle |
| `map_kPa` | kPa | Manifold pressure when the cycle was logged |
| `spark_btdc_deg` | deg | Measured spark advance |
| `pmax_bar`, `pmax_atdc_deg` | bar, deg | Peak pressure and its angle |
| `imep_gross_bar`, `imep_net_bar` | bar | Compression + expansion / full 720 deg |
| `ca10/50/90_atdc_deg`, `burn_10_90_deg` | deg | From the apparent heat release (pressure based, γ = 1.33) |
| `ca50_reference_atdc_deg` | deg | From the simulation's burned fuel (validates the above) |
| `misfire` | 0/1 | No fuel burned or no heat released |
| `fresh_charge_mg` | mg | Net gas through the intake valve, gas exchange TDC → firing TDC |
| `trapped_mass_mg` | mg | Cylinder content at intake valve closing (includes ECU fuel) |
| `residual_percent` | % | (trapped − fresh) / trapped (includes ECU fuel, so a few % high) |
| `ve_percent` | % | Fresh charge / (displacement × intake air density at 1 atm and `air_temperature`) |
| `fuel_burned_mg` | mg | Fuel burned in the cycle |
| `indicated_eff_gross_percent` | % | Gross indicated work / energy of the burned fuel |
| `egt_flow_weighted_C` | °C | Temperature of the gas leaving the cylinder, weighted by the exhaust flow (enthalpy-averaged EGT) |
| `egt_port_mean_C` | °C | Time average of the gas temperature in the exhaust port / runner |

Masses use the molar mass of air for all gas (as the simulation does).
