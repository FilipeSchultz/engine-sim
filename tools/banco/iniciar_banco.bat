@echo off
rem Simulador no papel do banco de motores pequenos: freio de correntes
rem parasitas e motor comandados pelo RCS (LabVIEW) via CAN
rem (Dyno_Eddy_Brake.dbc, mapa de sinais CanConfigRCS_Eddy.canconfig),
rem com a ECU virtual (ecu-standalone) junto.
setlocal
set ROOT=%~dp0..\..

rem Canal Kvaser e taxa do barramento do RCS
set ENGINE_SIM_CAN_CHANNEL=0
set ENGINE_SIM_CAN_BITRATE=500000

rem Opcionais
rem set ENGINE_SIM_DYNO_LOG=C:\temp\freio.csv
rem set ENGINE_SIM_ECU_LOG=C:\temp\ecu.csv
rem set ENGINE_SIM_INDI_UDP=127.0.0.1:5005

rem ECU virtual (sem ela o motor roda com a mistura e o avanco do script)
start "" "%ROOT%\build\ecu\Release\ecu-standalone.exe"

rem O simulador procura os assets em ..\assets: precisa partir da pasta build
pushd "%ROOT%\build"
start "" "Release\engine-sim-app.exe"
popd
