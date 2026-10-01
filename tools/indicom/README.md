# IndiCom virtual

Combustion analysis tool for engine-sim, modelled on a test cell indicating
system (AVL IndiCom, Kistler KiBox). engine-sim acts as the indicating hardware:
a pressure sensor in each cylinder plus a crank angle encoder, streamed over UDP
(see [docs/indicom_protocol.md](../../docs/indicom_protocol.md)). Everything else
is computed here, from the engine geometry entered by the operator.

## Setup (once)

```bat
cd tools\indicom
py -3.12 -m venv .venv
.venv\Scripts\python -m pip install -r requirements.txt
```

## Running

Double-click `iniciar_com_simulador.bat`, or manually:

```bat
set ENGINE_SIM_INDI_UDP=127.0.0.1:5005
cd build && Release\engine-sim-app.exe
tools\indicom\.venv\Scripts\python tools\indicom\indicom.py
```

Then **Carregar do simulador** fills the geometry and the firing TDC of each
cylinder from what engine-sim reports (in a real cell these come from the engine
drawings and a TDC sensor). They can also be typed in or saved/loaded as JSON
(`engines/`).

## What it computes (per cycle and cylinder)

| Result | How |
|---|---|
| Volume | slider-crank from bore, stroke, rod length and compression ratio |
| Pegging | thermodynamic: polytropic compression between -100° and -65° (or none) |
| Pmax, angle of Pmax | within ±180° of firing TDC |
| IMEP net / gross | ∮p dV over 720° / over ±180°, divided by the displacement |
| Heat release, MFB | dQ = γ/(γ-1)·p·dV + 1/(γ-1)·V·dp, normalised from its minimum to its maximum |
| CA10 / CA50 / CA90 | angles of 10/50/90 % mass fraction burned |
| COV IMEP | standard deviation / mean of the net IMEP over the averaged cycles |

Angles are degrees after the firing TDC (negative = before). The table shows the
mean of the last N cycles (setting "Ciclos na média").

The geometry matters as on a real bench: 1° of TDC error changes the IMEP by
a few %, a wrong compression ratio distorts the heat release.

## Sensor imperfections

Set before starting engine-sim:

- `ENGINE_SIM_INDI_OFFSET_BAR=0.5`: each sensor gets a fixed offset within ±0.5 bar
  (piezo sensors only measure pressure changes); thermodynamic pegging removes it.
- `ENGINE_SIM_INDI_NOISE_BAR=0.05`: gaussian noise.

## Check without the UI

`python selftest.py 8` listens for 8 s and prints per-cylinder averages, to be
compared with engine-sim's own per-cycle log (`ENGINE_SIM_INDI_LOG`).
