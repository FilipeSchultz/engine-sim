@echo off
rem Simulador no papel do banco de motores pequenos: motor Kawashima GE 700,
rem freio de correntes parasitas e motor comandados pelo RCS (LabVIEW) via CAN
rem (Dyno_Eddy_Brake.dbc, mapa de sinais CanConfigRCS_Eddy.canconfig), com a
rem ECU virtual (ecu-standalone) e o IndiCom (indicacao de combustao) junto.
setlocal
set ROOT=%~dp0..\..

rem Motor: script principal relativo a pasta build
set ENGINE_SIM_SCRIPT=../assets/main_ge700.mr

rem Calibracao da ECU usada no banco: copia de trabalho do dataset do motor
rem (criada na primeira vez; o VISION e o CalSaveRequest gravam nela)
set CAL=%ROOT%\assets\calibration\ge700_banco.dcm
if not exist "%CAL%" copy "%ROOT%\assets\calibration\datasets\ge700.dcm" "%CAL%" > nul

rem Canal Kvaser e taxa do barramento do RCS
set ENGINE_SIM_CAN_CHANNEL=0
set ENGINE_SIM_CAN_BITRATE=500000

rem Pressao no cilindro para o IndiCom
set ENGINE_SIM_INDI_UDP=127.0.0.1:5005

rem Opcionais
rem set ENGINE_SIM_DYNO_LOG=C:\temp\freio.csv
rem set ENGINE_SIM_ECU_LOG=C:\temp\ecu.csv
rem set ENGINE_SIM_INDI_LOG=C:\temp\indi.csv

rem ECU virtual (sem ela o motor roda com a mistura e o avanco do script)
rem (pasta de trabalho ao lado do executavel: o A2L e os arquivos do VISION vao para la)
start "" /D "%ROOT%\build\ecu\Release" "%ROOT%\build\ecu\Release\ecu-standalone.exe" --cal "%CAL%"

rem O simulador procura os assets em ..\assets: precisa partir da pasta build
pushd "%ROOT%\build"
start "" "Release\engine-sim-app.exe"
popd

rem IndiCom
pushd "%ROOT%\tools\indicom"
start "" ".venv\Scripts\pythonw.exe" indicom.py
popd
