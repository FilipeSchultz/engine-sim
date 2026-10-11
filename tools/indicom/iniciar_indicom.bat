@echo off
rem Opens only the IndiCom. The simulator must be started separately with
rem ENGINE_SIM_INDI_UDP=127.0.0.1:5005 to stream the cylinder pressure.
cd /d "%~dp0"
start "" ".venv\Scripts\pythonw.exe" indicom.py
