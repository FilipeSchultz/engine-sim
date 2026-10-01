@echo off
rem Opens engine-sim streaming cylinder pressure over UDP, then the IndiCom.
rem Optional sensor imperfections (piezo offset / noise) for realistic pegging:
rem   set ENGINE_SIM_INDI_OFFSET_BAR=0.5
rem   set ENGINE_SIM_INDI_NOISE_BAR=0.05
cd /d "%~dp0"
set ENGINE_SIM_INDI_UDP=127.0.0.1:5005
pushd "%~dp0..\..\build"
start "" "Release\engine-sim-app.exe"
popd
start "" ".venv\Scripts\pythonw.exe" indicom.py
