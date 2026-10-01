#include "../include/engine_sim_application.h"

#include "../include/piston_object.h"
#include "../include/connecting_rod_object.h"
#include "../include/constants.h"
#include "../include/units.h"
#include "../include/crankshaft_object.h"
#include "../include/cylinder_bank_object.h"
#include "../include/cylinder_head_object.h"
#include "../include/ui_button.h"
#include "../include/combustion_chamber_object.h"
#include "../include/csv_io.h"
#include "../include/exhaust_system.h"
#include "../include/feedback_comb_filter.h"
#include "../include/utilities.h"
#include "../include/avl_emcon.h"
#include "../include/ecu/dcm_file.h"

#include "../scripting/include/compiler.h"

#include <chrono>
#include <stdlib.h>
#include <sstream>
#include <fstream>
#include <windows.h>

#if ATG_ENGINE_SIM_DISCORD_ENABLED
#include "../discord/Discord.h"
#endif

std::string EngineSimApplication::s_buildVersion = "0.1.12a";

EngineSimApplication::EngineSimApplication() {
    m_assetPath = "";

    m_geometryVertexBuffer = nullptr;
    m_geometryIndexBuffer = nullptr;

    m_paused = false;
    m_recording = false;
    m_screenResolutionIndex = 0;
    for (int i = 0; i < ScreenResolutionHistoryLength; ++i) {
        m_screenResolution[i][0] = m_screenResolution[i][1] = 0;
    }

    m_background = ysColor::srgbiToLinear(0x0E1012);
    m_foreground = ysColor::srgbiToLinear(0xFFFFFF);
    m_shadow = ysColor::srgbiToLinear(0x0E1012);
    m_highlight1 = ysColor::srgbiToLinear(0xEF4545);
    m_highlight2 = ysColor::srgbiToLinear(0xFFFFFF);
    m_pink = ysColor::srgbiToLinear(0xF394BE);
    m_red = ysColor::srgbiToLinear(0xEE4445);
    m_orange = ysColor::srgbiToLinear(0xF4802A);
    m_yellow = ysColor::srgbiToLinear(0xFDBD2E);
    m_blue = ysColor::srgbiToLinear(0x77CEE0);
    m_green = ysColor::srgbiToLinear(0xBDD869);

    m_displayHeight = (float)units::distance(2.0, units::foot);
    m_outputAudioBuffer = nullptr;
    m_audioSource = nullptr;

    m_torque = 0;
    m_dynoSpeed = 0;

    m_simulator = nullptr;
    m_engineView = nullptr;
    m_rightGaugeCluster = nullptr;
    m_temperatureGauge = nullptr;
    m_oscCluster = nullptr;
    m_performanceCluster = nullptr;
    m_loadSimulationCluster = nullptr;
    m_mixerCluster = nullptr;
    m_infoCluster = nullptr;
    m_iceEngine = nullptr;
    m_mainRenderTarget = nullptr;

    m_vehicle = nullptr;
    m_transmission = nullptr;

    m_oscillatorSampleOffset = 0;
    m_gameWindowHeight = 256;
    m_screenWidth = 256;
    m_screenHeight = 256;
    m_screen = 0;
    m_viewParameters.Layer0 = 0;
    m_viewParameters.Layer1 = 0;

    m_displayAngle = 0.0f;
}

EngineSimApplication::~EngineSimApplication() {
    /* void */
}

void EngineSimApplication::initialize(void *instance, ysContextObject::DeviceAPI api) {
    dbasic::Path modulePath = dbasic::GetModulePath();
    dbasic::Path confPath = modulePath.Append("delta.conf");

    std::string enginePath = "../dependencies/submodules/delta-studio/engines/basic";
    m_assetPath = "../assets";
    if (confPath.Exists()) {
        std::fstream confFile(confPath.ToString(), std::ios::in);

        std::getline(confFile, enginePath);
        std::getline(confFile, m_assetPath);
        enginePath = modulePath.Append(enginePath).ToString();
        m_assetPath = modulePath.Append(m_assetPath).ToString();

        confFile.close();
    }

    m_engine.GetConsole()->SetDefaultFontDirectory(enginePath + "/fonts/");

    const std::string shaderPath = enginePath + "/shaders/";
    const std::string winTitle = "Engine Sim | AngeTheGreat | v" + s_buildVersion;
    dbasic::DeltaEngine::GameEngineSettings settings;
    settings.API = api;
    settings.DepthBuffer = false;
    settings.Instance = instance;
    settings.ShaderDirectory = shaderPath.c_str();
    settings.WindowTitle = winTitle.c_str();
    settings.WindowPositionX = 0;
    settings.WindowPositionY = 0;
    settings.WindowStyle = ysWindow::WindowStyle::Windowed;
    settings.WindowWidth = 1920;
    settings.WindowHeight = 1080;

    m_engine.CreateGameWindow(settings);

    m_engine.GetDevice()->CreateSubRenderTarget(
        &m_mainRenderTarget,
        m_engine.GetScreenRenderTarget(),
        0,
        0,
        0,
        0);

    m_engine.InitializeShaderSet(&m_shaderSet);
    m_shaders.Initialize(
        &m_shaderSet,
        m_mainRenderTarget,
        m_engine.GetScreenRenderTarget(),
        m_engine.GetDefaultShaderProgram(),
        m_engine.GetDefaultInputLayout());
    m_engine.InitializeConsoleShaders(&m_shaderSet);
    m_engine.SetShaderSet(&m_shaderSet);

    m_shaders.SetClearColor(ysColor::srgbiToLinear(0x34, 0x98, 0xdb));

    m_assetManager.SetEngine(&m_engine);

    m_engine.GetDevice()->CreateIndexBuffer(
        &m_geometryIndexBuffer, sizeof(unsigned short) * 200000, nullptr);
    m_engine.GetDevice()->CreateVertexBuffer(
        &m_geometryVertexBuffer, sizeof(dbasic::Vertex) * 100000, nullptr);

    m_geometryGenerator.initialize(100000, 200000);

    initialize();
}

void EngineSimApplication::initialize() {
    m_shaders.SetClearColor(ysColor::srgbiToLinear(0x34, 0x98, 0xdb));
    m_assetManager.CompileInterchangeFile((m_assetPath + "/assets").c_str(), 1.0f, true);
    m_assetManager.LoadSceneFile((m_assetPath + "/assets").c_str(), true);

    m_textRenderer.SetEngine(&m_engine);
    m_textRenderer.SetRenderer(m_engine.GetUiRenderer());
    m_textRenderer.SetFont(m_engine.GetConsole()->GetFont());

    loadScript();

    m_audioBuffer.initialize(44100, 44100);
    m_audioBuffer.m_writePointer = (int)(44100 * 0.1);

    ysAudioParameters params;
    params.m_bitsPerSample = 16;
    params.m_channelCount = 1;
    params.m_sampleRate = 44100;
    m_outputAudioBuffer =
        m_engine.GetAudioDevice()->CreateBuffer(&params, 44100);

    m_audioSource = m_engine.GetAudioDevice()->CreateSource(m_outputAudioBuffer);
    m_audioSource->SetMode((m_simulator->getEngine() != nullptr)
        ? ysAudioSource::Mode::Loop
        : ysAudioSource::Mode::Stop);

    initializeEcu();
    initializeIndicating();
    openCan();
    m_audioSource->SetPan(0.0f);
    m_audioSource->SetVolume(1.0f);

#ifdef ATG_ENGINE_SIM_DISCORD_ENABLED
    // Create a global instance of discord-rpc
    CDiscord::CreateInstance();

    // Enable it, this needs to be set via a config file of some sort. 
    GetDiscordManager()->SetUseDiscord(true);
    DiscordRichPresence passMe = { 0 };

    std::string engineName = (m_iceEngine != nullptr)
        ? m_iceEngine->getName()
        : "Broken Engine";

    GetDiscordManager()->SetStatus(passMe, engineName, s_buildVersion);
#endif /* ATG_ENGINE_SIM_DISCORD_ENABLED */
}

void EngineSimApplication::process(float frame_dt) {
    frame_dt = static_cast<float>(clamp(frame_dt, 1 / 200.0f, 1 / 30.0f));

    double speed = 1.0 / 1.0;
    if (m_canRemote) {
        // Keep real time while a CAN HOST is in control
    }
    else if (m_engine.IsKeyDown(ysKey::Code::N1)) {
        speed = 1 / 10.0;
    }
    else if (m_engine.IsKeyDown(ysKey::Code::N2)) {
        speed = 1 / 100.0;
    }
    else if (m_engine.IsKeyDown(ysKey::Code::N3)) {
        speed = 1 / 200.0;
    }
    else if (m_engine.IsKeyDown(ysKey::Code::N4)) {
        speed = 1 / 500.0;
    }
    else if (m_engine.IsKeyDown(ysKey::Code::N5)) {
        speed = 1 / 1000.0;
    }

    if (m_engine.IsKeyDown(ysKey::Code::F1)) {
        m_displayAngle += frame_dt * 1.0f;
    }
    else if (m_engine.IsKeyDown(ysKey::Code::F2)) {
        m_displayAngle -= frame_dt * 1.0f;
    }
    else if (m_engine.ProcessKeyDown(ysKey::Code::F3)) {
        m_displayAngle = 0.0f;
    }

    m_simulator->setSimulationSpeed(speed);

    const double avgFramerate = clamp(m_engine.GetAverageFramerate(), 30.0f, 1000.0f);
    m_simulator->startFrame(1 / avgFramerate);

    auto proc_t0 = std::chrono::steady_clock::now();
    const int iterationCount = m_simulator->getFrameIterationCount();
    while (m_simulator->simulateStep()) {
        m_oscCluster->sample();
    }

    auto proc_t1 = std::chrono::steady_clock::now();

    m_simulator->endFrame();

    auto duration = proc_t1 - proc_t0;
    if (iterationCount > 0) {
        m_performanceCluster->addTimePerTimestepSample(
            (duration.count() / 1E9) / iterationCount);
    }

    const SampleOffset safeWritePosition = m_audioSource->GetCurrentWritePosition();
    const SampleOffset writePosition = m_audioBuffer.m_writePointer;

    SampleOffset targetWritePosition =
        m_audioBuffer.getBufferIndex(safeWritePosition, (int)(44100 * 0.1));
    SampleOffset maxWrite = m_audioBuffer.offsetDelta(writePosition, targetWritePosition);

    SampleOffset currentLead = m_audioBuffer.offsetDelta(safeWritePosition, writePosition);
    SampleOffset newLead = m_audioBuffer.offsetDelta(safeWritePosition, targetWritePosition);

    if (currentLead > 44100 * 0.5) {
        m_audioBuffer.m_writePointer = m_audioBuffer.getBufferIndex(safeWritePosition, (int)(44100 * 0.05));
        currentLead = m_audioBuffer.offsetDelta(safeWritePosition, m_audioBuffer.m_writePointer);
        maxWrite = m_audioBuffer.offsetDelta(m_audioBuffer.m_writePointer, targetWritePosition);
    }

    if (currentLead > newLead) {
        maxWrite = 0;
    }

    int16_t *samples = new int16_t[maxWrite];
    const int readSamples = m_simulator->readAudioOutput(maxWrite, samples);

    for (SampleOffset i = 0; i < (SampleOffset)readSamples && i < maxWrite; ++i) {
        const int16_t sample = samples[i];
        if (m_oscillatorSampleOffset % 4 == 0) {
            m_oscCluster->getAudioWaveformOscilloscope()->addDataPoint(
                m_oscillatorSampleOffset,
                sample / (float)(INT16_MAX));
        }

        m_audioBuffer.writeSample(sample, m_audioBuffer.m_writePointer, (int)i);

        m_oscillatorSampleOffset = (m_oscillatorSampleOffset + 1) % (44100 / 10);
    }

    delete[] samples;

    if (readSamples > 0) {
        SampleOffset size0, size1;
        void *data0, *data1;
        m_audioSource->LockBufferSegment(
            m_audioBuffer.m_writePointer, readSamples, &data0, &size0, &data1, &size1);

        m_audioBuffer.copyBuffer(
            reinterpret_cast<int16_t *>(data0), m_audioBuffer.m_writePointer, size0);
        m_audioBuffer.copyBuffer(
            reinterpret_cast<int16_t *>(data1),
            m_audioBuffer.getBufferIndex(m_audioBuffer.m_writePointer, size0),
            size1);

        m_audioSource->UnlockBufferSegments(data0, size0, data1, size1);
        m_audioBuffer.commitBlock(readSamples);
    }

    m_performanceCluster->addInputBufferUsageSample(
        (double)m_simulator->getSynthesizerInputLatency() / m_simulator->getSynthesizerInputLatencyTarget());
    m_performanceCluster->addAudioLatencySample(
        m_audioBuffer.offsetDelta(m_audioSource->GetCurrentWritePosition(), m_audioBuffer.m_writePointer) / (44100 * 0.1));
}

void EngineSimApplication::render() {
    for (SimulationObject *object : m_objects) {
        object->generateGeometry();
    }

    m_viewParameters.Sublayer = 0;
    for (SimulationObject *object : m_objects) {
        object->render(&getViewParameters());
    }

    m_viewParameters.Sublayer = 1;
    for (SimulationObject *object : m_objects) {
        object->render(&getViewParameters());
    }

    m_viewParameters.Sublayer = 2;
    for (SimulationObject *object : m_objects) {
        object->render(&getViewParameters());
    }

    m_uiManager.render();
}

float EngineSimApplication::pixelsToUnits(float pixels) const {
    const float f = m_displayHeight / m_engineView->m_bounds.height();
    return pixels * f;
}

float EngineSimApplication::unitsToPixels(float units) const {
    const float f = m_engineView->m_bounds.height() / m_displayHeight;
    return units * f;
}

void EngineSimApplication::run() {
    while (true) {
        m_engine.StartFrame();

        if (!m_engine.IsOpen()) break;
        if (m_engine.ProcessKeyDown(ysKey::Code::Escape)) {
            break;
        }

        if (m_engine.ProcessKeyDown(ysKey::Code::Return)) {
            m_audioSource->SetMode(ysAudioSource::Mode::Stop);
            loadScript();
            if (m_simulator->getEngine() != nullptr) {
                m_audioSource->SetMode(ysAudioSource::Mode::Loop);
            }
        }

        if (m_engine.ProcessKeyDown(ysKey::Code::Tab)) {
            m_screen++;
            if (m_screen > 2) m_screen = 0;
        }

        if (m_engine.ProcessKeyDown(ysKey::Code::F)) {
            if (m_engine.GetGameWindow()->GetWindowStyle() != ysWindow::WindowStyle::Fullscreen) {
                m_engine.GetGameWindow()->SetWindowStyle(ysWindow::WindowStyle::Fullscreen);
                m_infoCluster->setLogMessage("Entered fullscreen mode");
            }
            else {
                m_engine.GetGameWindow()->SetWindowStyle(ysWindow::WindowStyle::Windowed);
                m_infoCluster->setLogMessage("Exited fullscreen mode");
            }
        }

        m_gameWindowHeight = m_engine.GetGameWindow()->GetGameHeight();
        m_screenHeight = m_engine.GetGameWindow()->GetScreenHeight();
        m_screenWidth = m_engine.GetGameWindow()->GetScreenWidth();

        updateScreenSizeStability();

        updateEcu(m_engine.GetFrameLength());
        updateIndicating(m_engine.GetFrameLength());
        updateCanReceive(m_engine.GetFrameLength());
        processEngineInput();

        if (m_engine.ProcessKeyDown(ysKey::Code::Insert) &&
            m_engine.GetGameWindow()->IsActive()) {
            if (!isRecording() && readyToRecord()) {
                startRecording();
            }
            else if (isRecording()) {
                stopRecording();
            }
        }

        if (isRecording() && !readyToRecord()) {
            stopRecording();
        }

        if (!m_paused || m_engine.ProcessKeyDown(ysKey::Code::Right)) {
            process(m_engine.GetFrameLength());
        }

        updateCanTransmit();

        m_uiManager.update(m_engine.GetFrameLength());

        renderScene();

        m_engine.EndFrame();

        if (isRecording()) {
            recordFrame();
        }
    }

    if (isRecording()) {
        stopRecording();
    }

    m_simulator->endAudioRenderingThread();
}

void EngineSimApplication::destroy() {
    m_can.close();

    if (m_ecu.isLearning()) {
        saveLearnedCalibration();
    }
    m_ecuLog.close();
    m_indicatingLog.close();
    m_indicatingUdp.close();

    m_shaderSet.Destroy();

    m_engine.GetDevice()->DestroyGPUBuffer(m_geometryVertexBuffer);
    m_engine.GetDevice()->DestroyGPUBuffer(m_geometryIndexBuffer);

    m_assetManager.Destroy();
    m_engine.Destroy();

    m_simulator->destroy();
    m_audioBuffer.destroy();
}

void EngineSimApplication::loadEngine(
    Engine *engine,
    Vehicle *vehicle,
    Transmission *transmission)
{
    destroyObjects();

    if (m_simulator != nullptr) {
        m_simulator->releaseSimulation();
        delete m_simulator;
    }

    if (m_vehicle != nullptr) {
        delete m_vehicle;
        m_vehicle = nullptr;
    }

    if (m_transmission != nullptr) {
        delete m_transmission;
        m_transmission = nullptr;
    }

    if (m_iceEngine != nullptr) {
        m_iceEngine->destroy();
        delete m_iceEngine;
    }

    m_iceEngine = engine;
    m_vehicle = vehicle;
    m_transmission = transmission;

    m_simulator = engine->createSimulator(vehicle, transmission);
    if (m_simulator != nullptr) {
        m_simulator->setEcu(&m_ecu);
        attachIndicating();
    }

    if (engine == nullptr || vehicle == nullptr || transmission == nullptr) {
        m_iceEngine = nullptr;
        m_viewParameters.Layer1 = 0;

        return;
    }

    createObjects(engine);

    m_viewParameters.Layer1 = engine->getMaxDepth();
    engine->calculateDisplacement();

    m_simulator->setSimulationFrequency(engine->getSimulationFrequency());

    Synthesizer::AudioParameters audioParams = m_simulator->synthesizer().getAudioParameters();
    audioParams.inputSampleNoise = static_cast<float>(engine->getInitialJitter());
    audioParams.airNoise = static_cast<float>(engine->getInitialNoise());
    audioParams.dF_F_mix = static_cast<float>(engine->getInitialHighFrequencyGain());
    m_simulator->synthesizer().setAudioParameters(audioParams);

    for (int i = 0; i < engine->getExhaustSystemCount(); ++i) {
        ImpulseResponse *response = engine->getExhaustSystem(i)->getImpulseResponse();

        ysWindowsAudioWaveFile waveFile;
        waveFile.OpenFile(response->getFilename().c_str());
        waveFile.InitializeInternalBuffer(waveFile.GetSampleCount());
        waveFile.FillBuffer(0);
        waveFile.CloseFile();

        m_simulator->synthesizer().initializeImpulseResponse(
            reinterpret_cast<const int16_t *>(waveFile.GetBuffer()),
            waveFile.GetSampleCount(),
            response->getVolume(),
            i
        );

        waveFile.DestroyInternalBuffer();
    }

    m_simulator->startAudioRenderingThread();
}

void EngineSimApplication::drawGenerated(
    const GeometryGenerator::GeometryIndices &indices,
    int layer)
{
    drawGenerated(indices, layer, m_shaders.GetRegularFlags());
}

void EngineSimApplication::drawGeneratedUi(
    const GeometryGenerator::GeometryIndices &indices,
    int layer)
{
    drawGenerated(indices, layer, m_shaders.GetUiFlags());
}

void EngineSimApplication::drawGenerated(
    const GeometryGenerator::GeometryIndices &indices,
    int layer,
    dbasic::StageEnableFlags flags)
{
    m_engine.DrawGeneric(
        flags,
        m_geometryIndexBuffer,
        m_geometryVertexBuffer,
        sizeof(dbasic::Vertex),
        indices.BaseIndex,
        indices.BaseVertex,
        indices.FaceCount,
        false,
        layer);
}

void EngineSimApplication::configure(const ApplicationSettings &settings) {
    m_applicationSettings = settings;

    if (settings.startFullscreen) {
        m_engine.GetGameWindow()->SetWindowStyle(ysWindow::WindowStyle::Fullscreen);
    }

    m_background = ysColor::srgbiToLinear(m_applicationSettings.colorBackground);
    m_foreground = ysColor::srgbiToLinear(m_applicationSettings.colorForeground);
    m_shadow = ysColor::srgbiToLinear(m_applicationSettings.colorShadow);
    m_highlight1 = ysColor::srgbiToLinear(m_applicationSettings.colorHighlight1);
    m_highlight2 = ysColor::srgbiToLinear(m_applicationSettings.colorHighlight2);
    m_pink = ysColor::srgbiToLinear(m_applicationSettings.colorPink);
    m_red = ysColor::srgbiToLinear(m_applicationSettings.colorRed);
    m_orange = ysColor::srgbiToLinear(m_applicationSettings.colorOrange);
    m_yellow = ysColor::srgbiToLinear(m_applicationSettings.colorYellow);
    m_blue = ysColor::srgbiToLinear(m_applicationSettings.colorBlue);
    m_green = ysColor::srgbiToLinear(m_applicationSettings.colorGreen);
}

void EngineSimApplication::createObjects(Engine *engine) {
    for (int i = 0; i < engine->getCylinderCount(); ++i) {
        ConnectingRodObject *rodObject = new ConnectingRodObject;
        rodObject->initialize(this);
        rodObject->m_connectingRod = engine->getConnectingRod(i);
        m_objects.push_back(rodObject);

        PistonObject *pistonObject = new PistonObject;
        pistonObject->initialize(this);
        pistonObject->m_piston = engine->getPiston(i);
        m_objects.push_back(pistonObject);

        CombustionChamberObject *ccObject = new CombustionChamberObject;
        ccObject->initialize(this);
        ccObject->m_chamber = m_iceEngine->getChamber(i);
        m_objects.push_back(ccObject);
    }

    for (int i = 0; i < engine->getCrankshaftCount(); ++i) {
        CrankshaftObject *crankshaftObject = new CrankshaftObject;
        crankshaftObject->initialize(this);
        crankshaftObject->m_crankshaft = engine->getCrankshaft(i);
        m_objects.push_back(crankshaftObject);
    }

    for (int i = 0; i < engine->getCylinderBankCount(); ++i) {
        CylinderBankObject *cbObject = new CylinderBankObject;
        cbObject->initialize(this);
        cbObject->m_bank = engine->getCylinderBank(i);
        cbObject->m_head = engine->getHead(i);
        m_objects.push_back(cbObject);

        CylinderHeadObject *chObject = new CylinderHeadObject;
        chObject->initialize(this);
        chObject->m_head = engine->getHead(i);
        chObject->m_engine = engine;
        m_objects.push_back(chObject);
    }
}

void EngineSimApplication::destroyObjects() {
    for (SimulationObject *object : m_objects) {
        object->destroy();
        delete object;
    }

    m_objects.clear();
}

const SimulationObject::ViewParameters &
    EngineSimApplication::getViewParameters() const
{
    return m_viewParameters;
}

void EngineSimApplication::loadScript() {
    Engine *engine = nullptr;
    Vehicle *vehicle = nullptr;
    Transmission *transmission = nullptr;

#ifdef ATG_ENGINE_SIM_PIRANHA_ENABLED
    es_script::Compiler compiler;
    compiler.initialize();
    const bool compiled = compiler.compile("../assets/main.mr");
    if (compiled) {
        const es_script::Compiler::Output output = compiler.execute();
        configure(output.applicationSettings);

        engine = output.engine;
        vehicle = output.vehicle;
        transmission = output.transmission;
    }
    else {
        engine = nullptr;
        vehicle = nullptr;
        transmission = nullptr;
    }

    compiler.destroy();

    if (engine == nullptr) {
        // Show the script errors instead of crashing on a null engine
        std::ifstream errorLog("error_log.log");
        std::stringstream errors;
        errors << errorLog.rdbuf();
        const std::string message =
            "Failed to load ../assets/main.mr\n\n" + errors.str();
        MessageBoxA(nullptr, message.c_str(), "Engine Sim - script error", MB_OK | MB_ICONERROR);

        if (vehicle != nullptr) delete vehicle;
        if (transmission != nullptr) delete transmission;

        // Keep the currently loaded engine when reloading; nothing to fall back to on startup
        if (m_iceEngine == nullptr) ExitProcess(1);
        return;
    }
#endif /* ATG_ENGINE_SIM_PIRANHA_ENABLED */

    if (vehicle == nullptr) {
        Vehicle::Parameters vehParams;
        vehParams.mass = units::mass(1597, units::kg);
        vehParams.diffRatio = 3.42;
        vehParams.tireRadius = units::distance(10, units::inch);
        vehParams.dragCoefficient = 0.25;
        vehParams.crossSectionArea = units::distance(6.0, units::foot) * units::distance(6.0, units::foot);
        vehParams.rollingResistance = 2000.0;
        vehicle = new Vehicle;
        vehicle->initialize(vehParams);
    }

    if (transmission == nullptr) {
        const double gearRatios[] = { 2.97, 2.07, 1.43, 1.00, 0.84, 0.56 };
        Transmission::Parameters tParams;
        tParams.GearCount = 6;
        tParams.GearRatios = gearRatios;
        tParams.MaxClutchTorque = units::torque(1000.0, units::ft_lb);
        transmission = new Transmission;
        transmission->initialize(tParams);
    }

    loadEngine(engine, vehicle, transmission);
    refreshUserInterface();
}

namespace {
    int getEnvironmentInt(const char *name, int defaultValue) {
        char value[32];
        const DWORD length = GetEnvironmentVariableA(name, value, sizeof(value));
        if (length == 0 || length >= sizeof(value)) return defaultValue;

        return std::atoi(value);
    }

    double getEnvironmentDouble(const char *name, double defaultValue) {
        char value[32];
        const DWORD length = GetEnvironmentVariableA(name, value, sizeof(value));
        if (length == 0 || length >= sizeof(value)) return defaultValue;

        return std::atof(value);
    }

    std::string getEnvironmentString(const char *name) {
        char value[512];
        const DWORD length = GetEnvironmentVariableA(name, value, sizeof(value));
        if (length == 0 || length >= sizeof(value)) return "";

        return std::string(value);
    }

    double rampTowards(double current, double target, double maxStep) {
        if (maxStep <= 0) return target;
        else if (target > current) return std::fmin(current + maxStep, target);
        else return std::fmax(current - maxStep, target);
    }
}

void EngineSimApplication::initializeEcu() {
    m_calibrationPath = m_assetPath + "/calibration/calibration.dcm";

    // First run: write the default (disabled) calibration so there is a file to edit
    std::error_code ec;
    if (!std::filesystem::exists(m_calibrationPath, ec)) {
        std::filesystem::create_directories(std::filesystem::path(m_calibrationPath).parent_path(), ec);

        std::string error;
        ecu::writeDcm(
            m_calibrationPath,
            ecu::defaultCalibration(),
            "Default calibration: ECU functions disabled (engine script behaviour)",
            &error);
    }

    reloadCalibration();

    m_ecuLearnPath = getEnvironmentString("ENGINE_SIM_ECU_LEARN");
    if (!m_ecuLearnPath.empty()) {
        m_ecu.setLearning(true);
        m_ecuLearnSaveTimer = 10.0;
    }

    const std::string logPath = getEnvironmentString("ENGINE_SIM_ECU_LOG");
    if (!logPath.empty()) {
        m_ecuLog.open(logPath, std::ios::out);
        m_ecuLog << "time_s,rpm,map_kPa,injection_ms,advance_deg,intake_afr,cylinder_lambda,throttle_percent,"
            "dyno_torque_Nm,fuel_ecu,ignition_ecu\n";
    }
}

void EngineSimApplication::reloadCalibration() {
    std::error_code ec;
    m_calibrationFileTime = std::filesystem::last_write_time(m_calibrationPath, ec);

    ecu::CalibrationPage page = m_ecu.calibration().getWorkingPage();
    std::vector<std::string> warnings;
    std::string error;
    if (!ecu::readDcm(m_calibrationPath, &page, &warnings, &error)) {
        // Keep running with the previous calibration
        m_infoCluster->setLogMessage("ECU: " + error);
        return;
    }

    m_ecu.calibration().setWorkingPage(page);

    std::string message = "ECU: calibration loaded (fuel ";
    message += (page.EcuFuelEnable >= 0.5f) ? "ON" : "OFF";
    message += ", ignition ";
    message += (page.EcuIgnEnable >= 0.5f) ? "ON" : "OFF";
    message += ")";
    if (!warnings.empty()) {
        message += " - " + warnings.front();
    }

    m_infoCluster->setLogMessage(message);
}

void EngineSimApplication::saveLearnedCalibration() {
    int fuelColumns = 0, ignitionColumns = 0;
    const ecu::CalibrationPage page = m_ecu.buildLearnedPage(&fuelColumns, &ignitionColumns);

    std::string error;
    const std::string comment =
        "Learned from the engine script behaviour (steady-state points, linear fit\n"
        "against MAP per RPM column): "
        + std::to_string(fuelColumns) + " fuel and "
        + std::to_string(ignitionColumns) + " ignition RPM columns learned";
    if (!ecu::writeDcm(m_ecuLearnPath, page, comment, &error)) {
        m_infoCluster->setLogMessage("ECU: " + error);
    }
}

void EngineSimApplication::updateEcu(double dt) {
    m_ecu.beginFrame();

    // Reload the calibration file whenever it is saved
    m_calibrationCheckTimer -= dt;
    if (m_calibrationCheckTimer <= 0) {
        m_calibrationCheckTimer = 0.5;

        std::error_code ec;
        const auto fileTime = std::filesystem::last_write_time(m_calibrationPath, ec);
        if (!ec && fileTime != m_calibrationFileTime) {
            reloadCalibration();
        }
    }

    if (m_ecu.isLearning()) {
        m_ecuLearnSaveTimer -= dt;
        if (m_ecuLearnSaveTimer <= 0) {
            m_ecuLearnSaveTimer = 10.0;
            saveLearnedCalibration();
        }
    }

    if (m_ecuLog.is_open() && m_iceEngine != nullptr) {
        m_ecuLogTime += dt;
        m_ecuLogTimer -= dt;
        if (m_ecuLogTimer <= 0) {
            m_ecuLogTimer = 0.05;

            const ecu::Ecu::Measurements &m = m_ecu.getMeasurements();

            m_ecuLog
                << m_ecuLogTime << ","
                << m.rpm << ","
                << m.map_kPa << ","
                << m.injectionTime_ms << ","
                << m.advance_deg << ","
                << m_iceEngine->getIntakeAfr() << ","
                << m_iceEngine->getCylinderLambda() << ","
                << m_speedSetting * 100.0 << ","
                << units::convert(m_simulator->getFilteredDynoTorque(), units::Nm) << ","
                << (m.fuelActive ? 1 : 0) << ","
                << (m.ignitionActive ? 1 : 0) << "\n";

            // Keep the file usable even if the simulator is killed
            if (++m_ecuLogLines % 20 == 0) m_ecuLog.flush();
        }
    }
}

void EngineSimApplication::initializeIndicating() {
    const std::string path = getEnvironmentString("ENGINE_SIM_INDI_LOG");
    if (!path.empty()) {
        m_indicatingLog.open(path, std::ios::out);
        if (m_indicatingLog.is_open()) {
            // Angles in degrees relative to each cylinder's firing TDC (positive = after TDC)
            m_indicatingLog
                << "time_s,cycle,cylinder,rpm,map_kPa,spark_btdc_deg,pmax_bar,pmax_atdc_deg,"
                << "imep_gross_bar,imep_net_bar,ca10_atdc_deg,ca50_atdc_deg,ca90_atdc_deg,"
                << "burn_10_90_deg,ca50_reference_atdc_deg,misfire\n";
        }
        else {
            m_infoCluster->setLogMessage("INDI: cannot write " + path);
        }
    }

    const std::string address = getEnvironmentString("ENGINE_SIM_INDI_UDP");
    if (!address.empty()) {
        std::string error;
        if (m_indicatingUdp.open(address, &error)) {
            m_infoCluster->setLogMessage("INDI: streaming cylinder pressure to " + address);
        }
        else {
            m_infoCluster->setLogMessage("INDI: " + error);
        }
    }

    m_sensorNoise_bar = getEnvironmentDouble("ENGINE_SIM_INDI_NOISE_BAR", 0.0);
    m_sensorRng.seed(std::random_device()());

    // The first engine was loaded before this point
    attachIndicating();
}

void EngineSimApplication::attachIndicating() {
    if (m_simulator == nullptr) return;

    const bool analysis = m_indicatingLog.is_open();
    const bool stream = m_indicatingUdp.isOpen();
    if (!analysis && !stream) return;

    m_indicating.reset();
    m_indicating.setAnalysis(analysis);
    m_indicating.setRawCapture(stream);
    m_simulator->setIndicatingSystem(&m_indicating);

    // Each sensor gets its own fixed offset in +/- ENGINE_SIM_INDI_OFFSET_BAR
    const double offset = getEnvironmentDouble("ENGINE_SIM_INDI_OFFSET_BAR", 0.0);
    std::uniform_real_distribution<double> distribution(-offset, offset);
    m_sensorOffset_bar.clear();
    for (int i = 0; i < m_iceEngine->getCylinderCount(); ++i) {
        m_sensorOffset_bar.push_back(static_cast<float>((offset > 0) ? distribution(m_sensorRng) : 0.0));
    }
}

void EngineSimApplication::updateIndicating(double dt) {
    if (m_iceEngine == nullptr) return;

    if (m_indicatingUdp.isOpen()) {
        sendIndicatingData();

        m_indicatingInfoTimer -= dt;
        if (m_indicatingInfoTimer <= 0) {
            m_indicatingInfoTimer = 1.0;
            sendIndicatingInfo();
        }
    }

    if (!m_indicatingLog.is_open()) return;

    const double map_kPa = units::convert(m_iceEngine->getManifoldPressure(), units::kPa);

    IndicatingSystem::CycleResult r;
    while (m_indicating.popResult(&r)) {
        m_indicatingLog << r.time << "," << r.cycle << "," << r.cylinder << ","
            << r.rpm << "," << map_kPa << ",";

        if (r.sparkValid) m_indicatingLog << r.sparkAdvance_deg;
        m_indicatingLog << ","
            << r.pmax_bar << "," << r.pmaxAngle_deg << ","
            << r.imepGross_bar << "," << r.imepNet_bar << ",";

        if (!r.misfire) {
            m_indicatingLog
                << r.ca10_deg << "," << r.ca50_deg << "," << r.ca90_deg << ","
                << r.ca90_deg - r.ca10_deg << "," << r.ca50Reference_deg;
        }
        else {
            m_indicatingLog << ",,,,";
        }

        m_indicatingLog << "," << (r.misfire ? 1 : 0) << "\n";

        if (++m_indicatingLogLines % 50 == 0) m_indicatingLog.flush();
    }
}

namespace {
    // Indicating stream packet header, see docs/indicom_protocol.md
    constexpr uint8_t IndicatingProtocolVersion = 1;
    constexpr uint8_t IndicatingPacketData = 1;
    constexpr uint8_t IndicatingPacketInfo = 2;
    constexpr size_t IndicatingMaxPacket = 1400;
    constexpr size_t IndicatingHeaderSize = 16;

    template <typename T>
    void put(std::vector<uint8_t> *buffer, T value) {
        const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&value);
        buffer->insert(buffer->end(), bytes, bytes + sizeof(T));
    }

    void putHeader(std::vector<uint8_t> *buffer, uint8_t type, uint16_t cylinders, uint32_t sequence, uint16_t count) {
        buffer->clear();
        buffer->push_back('E');
        buffer->push_back('S');
        buffer->push_back('I');
        buffer->push_back('N');
        buffer->push_back(IndicatingProtocolVersion);
        buffer->push_back(type);
        put<uint16_t>(buffer, cylinders);
        put<uint32_t>(buffer, sequence);
        put<uint16_t>(buffer, count);
        put<uint16_t>(buffer, 0);
    }
}

void EngineSimApplication::sendIndicatingData() {
    std::vector<float> &raw = m_indicating.rawSamples();
    const int cylinders = m_indicating.getCylinderCount();
    if (cylinders == 0 || raw.empty()) {
        raw.clear();
        return;
    }

    const size_t recordSize = 1 + cylinders;
    const size_t records = raw.size() / recordSize;
    const size_t perPacket = (IndicatingMaxPacket - IndicatingHeaderSize) / (recordSize * sizeof(float));

    std::normal_distribution<float> noise(0.0f, static_cast<float>(m_sensorNoise_bar));
    std::vector<uint8_t> packet;
    packet.reserve(IndicatingMaxPacket);

    for (size_t first = 0; first < records; first += perPacket) {
        const size_t count = (std::min)(perPacket, records - first);
        putHeader(&packet, IndicatingPacketData, (uint16_t)cylinders, m_indicatingSequence++, (uint16_t)count);

        for (size_t r = first; r < first + count; ++r) {
            const float *record = &raw[r * recordSize];
            put<float>(&packet, record[0]);
            for (int c = 0; c < cylinders; ++c) {
                float p = record[1 + c];
                if (c < (int)m_sensorOffset_bar.size()) p += m_sensorOffset_bar[c];
                if (m_sensorNoise_bar > 0) p += noise(m_sensorRng);
                put<float>(&packet, p);
            }
        }

        m_indicatingUdp.send(packet.data(), packet.size());
    }

    raw.clear();
}

void EngineSimApplication::sendIndicatingInfo() {
    const IndicatingSystem::EngineInfo &info = m_indicating.getEngineInfo();
    if (info.cylinders == 0) return;

    std::vector<uint8_t> packet;
    putHeader(&packet, IndicatingPacketInfo, (uint16_t)info.cylinders, m_indicatingSequence++, 0);
    put<float>(&packet, (float)info.bore_mm);
    put<float>(&packet, (float)info.stroke_mm);
    put<float>(&packet, (float)info.rodLength_mm);
    put<float>(&packet, (float)info.compressionRatio);
    put<float>(&packet, (float)m_simulator->getSimulationFrequency());
    for (int i = 0; i < info.cylinders; ++i) {
        put<float>(&packet, (i < (int)info.tdcEncoderAngle_deg.size()) ? (float)info.tdcEncoderAngle_deg[i] : -1.0f);
    }

    const std::string name = info.name.substr(0, 255);
    packet.push_back((uint8_t)name.size());
    packet.insert(packet.end(), name.begin(), name.end());

    m_indicatingUdp.send(packet.data(), packet.size());
}

void EngineSimApplication::openCan() {
#ifdef ATG_ENGINE_SIM_CAN_ENABLED
    const int channel = getEnvironmentInt("ENGINE_SIM_CAN_CHANNEL", 0);
    const int bitrate = getEnvironmentInt("ENGINE_SIM_CAN_BITRATE", 500000);

    // Disabled by default to ease testing; the DBC specifies 100 ms
    m_canLifesignalTimeoutMs = getEnvironmentInt("ENGINE_SIM_CAN_LIFESIGNAL_TIMEOUT_MS", 0);

    // Dyno behaviour under CAN control (0 disables the ramp / noise / filter)
    m_canSpeedRampRate = units::rpm(getEnvironmentDouble("ENGINE_SIM_DYNO_SPEED_RAMP_RPM_S", 1000.0));
    m_canTorqueRampRate = units::torque(getEnvironmentDouble("ENGINE_SIM_DYNO_TORQUE_RAMP_NM_S", 2000.0), units::Nm);
    m_canTorqueNoise_Nm = getEnvironmentDouble("ENGINE_SIM_DYNO_TORQUE_NOISE_NM", 1.0);
    m_canTorqueFilterTau = getEnvironmentDouble("ENGINE_SIM_DYNO_TORQUE_FILTER_MS", 50.0) / 1000.0;
    m_canRng.seed(std::random_device()());

    if (!m_can.open(channel, bitrate)) {
        m_infoCluster->setLogMessage("CAN: " + m_can.getLastError());
        return;
    }

    m_can.setCyclic(
        avl_emcon::encodeBmeControl(avl_emcon::BmeControl()),
        avl_emcon::BmeControlPeriodMs);
    m_can.setCyclic(
        avl_emcon::encodeBmeActual1(0.0f, 0.0f),
        avl_emcon::BmeActualPeriodMs);
    m_can.setCyclic(
        avl_emcon::encodeBmeActual2(0.0f),
        avl_emcon::BmeActualPeriodMs,
        avl_emcon::makeLifesignalWriter());

    m_infoCluster->setLogMessage(
        "CAN: AVL EMCON on channel " + std::to_string(channel)
        + " @ " + std::to_string(bitrate / 1000) + " kbit/s");
#endif /* ATG_ENGINE_SIM_CAN_ENABLED */
}

void EngineSimApplication::updateCanReceive(double dt) {
    if (!m_can.isOpen() || m_iceEngine == nullptr) return;

    CanInterface::Frame frame;
    while (m_can.read(&frame)) {
        if (frame.extended) continue;

        switch (frame.id) {
        case avl_emcon::HostControlId:
            avl_emcon::decodeHostControl(frame, &m_hostControl);
            break;
        case avl_emcon::HostDemand1Id:
            avl_emcon::decodeHostDemand1(frame, &m_hostDemand1);
            break;
        case avl_emcon::HostDemand2Id:
            avl_emcon::decodeHostDemand2(frame, &m_hostDemand2);
            break;
        default:
            break;
        }
    }

    // Reset_Key: rising edge clears a latched fault
    if (m_hostControl.resetKey && !m_canPrevResetKey && m_canError) {
        m_canError = false;
        m_canLifesignalAge = 0.0;
        m_infoCluster->setLogMessage("CAN: ERROR RESET");
    }
    m_canPrevResetKey = m_hostControl.resetKey;

    // Lifesignal: the HOST must keep changing LifesignalFeedback while in remote
    if (m_hostDemand2.lifesignalFeedback != m_canLastLifesignalFeedback) {
        m_canLastLifesignalFeedback = m_hostDemand2.lifesignalFeedback;
        m_canLifesignalAge = 0.0;
    }
    else {
        m_canLifesignalAge += dt;
    }

    if (m_canRemote
        && m_canLifesignalTimeoutMs > 0
        && m_canLifesignalAge * 1000.0 > m_canLifesignalTimeoutMs)
    {
        m_canError = true;
        m_infoCluster->setLogMessage("CAN: LIFESIGNAL TIMEOUT - BACK TO MANUAL");
    }

    setCanRemote(m_hostControl.remoteCommand && !m_canError);
}

void EngineSimApplication::setCanRemote(bool remote) {
    if (remote == m_canRemote) return;
    m_canRemote = remote;

    if (remote) {
        m_infoCluster->setLogMessage("CAN: REMOTE CONTROL");
        return;
    }

    // Leaving remote: bring the engine and dyno to a safe state before the
    // keyboard takes over
    m_targetSpeedSetting = 0.0;
    m_simulator->m_starterMotor.m_enabled = false;
    m_simulator->m_dyno.m_enabled = false;
    m_simulator->m_dyno.m_hold = false;
    m_simulator->m_dyno.m_torqueMode = false;
    m_dynoSpeed = 0.0;

    if (!m_canError) {
        m_infoCluster->setLogMessage("CAN: MANUAL CONTROL");
    }
}

void EngineSimApplication::updateCanTransmit() {
    if (!m_can.isOpen() || m_iceEngine == nullptr) return;

    const bool dynoEnabled = m_simulator->m_dyno.m_enabled;

    avl_emcon::BmeControl control;
    control.systemState = m_canRemote
        ? avl_emcon::SystemState::Remote
        : avl_emcon::SystemState::Manual;
    control.controlMode = m_simulator->m_dyno.m_torqueMode
        ? avl_emcon::ControlMode::Torque
        : avl_emcon::ControlMode::Speed;
    control.systemError = m_canError
        ? avl_emcon::SystemError::Trouble
        : avl_emcon::SystemError::NoTrouble;
    control.dynoOnOffState = dynoEnabled
        ? avl_emcon::DynoOnOffState::On
        : avl_emcon::DynoOnOffState::Off;
    control.torqueMode = m_hostControl.torqueMode;

    // The dyno is coupled to the crankshaft, so its speed is the engine speed.
    // Its torque is the braking torque it applies (only meaningful when enabled).
    const float dynoSpeed_rpm = static_cast<float>(std::abs(m_iceEngine->getRpm()));
    const float throttleSet_percent = static_cast<float>(m_speedSetting * 100.0);
    const double trueTorque_Nm = dynoEnabled
        ? units::convert(m_simulator->getFilteredDynoTorque(), units::Nm)
        : 0.0;

    // Load cell: gaussian noise (present even with the dyno off, like a real
    // sensor) followed by a first-order low-pass filter
    double rawTorque_Nm = trueTorque_Nm;
    if (m_canTorqueNoise_Nm > 0) {
        std::normal_distribution<double> noise(0.0, m_canTorqueNoise_Nm);
        rawTorque_Nm += noise(m_canRng);
    }

    const double dt = m_engine.GetFrameLength();
    const double alpha = (m_canTorqueFilterTau > 0)
        ? dt / (dt + m_canTorqueFilterTau)
        : 1.0;
    m_canMeasuredTorque_Nm += alpha * (rawTorque_Nm - m_canMeasuredTorque_Nm);

    m_can.setCyclic(avl_emcon::encodeBmeControl(control), avl_emcon::BmeControlPeriodMs);
    m_can.setCyclic(
        avl_emcon::encodeBmeActual1(dynoSpeed_rpm, throttleSet_percent),
        avl_emcon::BmeActualPeriodMs);
    m_can.setCyclic(
        avl_emcon::encodeBmeActual2(static_cast<float>(m_canMeasuredTorque_Nm)),
        avl_emcon::BmeActualPeriodMs);
}

void EngineSimApplication::processEngineInput() {
    if (m_iceEngine == nullptr) {
        return;
    }

    const float dt = m_engine.GetFrameLength();
    const bool fineControlMode = m_engine.IsKeyDown(ysKey::Code::Space);

    const int mouseWheel = m_engine.GetMouseWheel();
    const int mouseWheelDelta = mouseWheel - m_lastMouseWheel;
    m_lastMouseWheel = mouseWheel;

    bool fineControlInUse = false;
    if (m_engine.IsKeyDown(ysKey::Code::Z)) {
        const double rate = fineControlMode
            ? 0.001
            : 0.01;

        Synthesizer::AudioParameters audioParams = m_simulator->synthesizer().getAudioParameters();
        audioParams.volume = clamp(audioParams.volume + mouseWheelDelta * rate * dt);

        m_simulator->synthesizer().setAudioParameters(audioParams);
        fineControlInUse = true;

        m_infoCluster->setLogMessage("[Z] - Set volume to " + std::to_string(audioParams.volume));
    }
    else if (m_engine.IsKeyDown(ysKey::Code::X)) {
        const double rate = fineControlMode
            ? 0.001
            : 0.01;

        Synthesizer::AudioParameters audioParams = m_simulator->synthesizer().getAudioParameters();
        audioParams.convolution = clamp(audioParams.convolution + mouseWheelDelta * rate * dt);

        m_simulator->synthesizer().setAudioParameters(audioParams);
        fineControlInUse = true;

        m_infoCluster->setLogMessage("[X] - Set convolution level to " + std::to_string(audioParams.convolution));
    }
    else if (m_engine.IsKeyDown(ysKey::Code::C)) {
        const double rate = fineControlMode
            ? 0.00001
            : 0.001;

        Synthesizer::AudioParameters audioParams = m_simulator->synthesizer().getAudioParameters();
        audioParams.dF_F_mix = clamp(audioParams.dF_F_mix + mouseWheelDelta * rate * dt);

        m_simulator->synthesizer().setAudioParameters(audioParams);
        fineControlInUse = true;

        m_infoCluster->setLogMessage("[C] - Set high freq. gain to " + std::to_string(audioParams.dF_F_mix));
    }
    else if (m_engine.IsKeyDown(ysKey::Code::V)) {
        const double rate = fineControlMode
            ? 0.001
            : 0.01;

        Synthesizer::AudioParameters audioParams = m_simulator->synthesizer().getAudioParameters();
        audioParams.airNoise = clamp(audioParams.airNoise + mouseWheelDelta * rate * dt);

        m_simulator->synthesizer().setAudioParameters(audioParams);
        fineControlInUse = true;

        m_infoCluster->setLogMessage("[V] - Set low freq. noise to " + std::to_string(audioParams.airNoise));
    }
    else if (m_engine.IsKeyDown(ysKey::Code::B)) {
        const double rate = fineControlMode
            ? 0.001
            : 0.01;

        Synthesizer::AudioParameters audioParams = m_simulator->synthesizer().getAudioParameters();
        audioParams.inputSampleNoise = clamp(audioParams.inputSampleNoise + mouseWheelDelta * rate * dt);

        m_simulator->synthesizer().setAudioParameters(audioParams);
        fineControlInUse = true;

        m_infoCluster->setLogMessage("[B] - Set high freq. noise to " + std::to_string(audioParams.inputSampleNoise));
    }
    else if (m_engine.IsKeyDown(ysKey::Code::N)) {
        const double rate = fineControlMode
            ? 10.0
            : 100.0;

        const double newSimulationFrequency = clamp(
            m_simulator->getSimulationFrequency() + mouseWheelDelta * rate * dt,
            400.0, 400000.0);

        m_simulator->setSimulationFrequency(newSimulationFrequency);
        fineControlInUse = true;

        m_infoCluster->setLogMessage("[N] - Set simulation freq to " + std::to_string(m_simulator->getSimulationFrequency()));
    }
    else if (m_engine.IsKeyDown(ysKey::Code::G) && m_simulator->m_dyno.m_hold) {
        if (mouseWheelDelta > 0) {
            m_dynoSpeed += m_iceEngine->getDynoHoldStep();
        }
        else if (mouseWheelDelta < 0) {
            m_dynoSpeed -= m_iceEngine->getDynoHoldStep();
        }

        m_dynoSpeed = clamp(m_dynoSpeed, m_iceEngine->getDynoMinSpeed(), m_iceEngine->getDynoMaxSpeed());

        m_infoCluster->setLogMessage("[G] - Set dyno speed to " + std::to_string(units::toRpm(m_dynoSpeed)));
        fineControlInUse = true;
    }

    const double prevTargetThrottle = m_targetSpeedSetting;
    m_targetSpeedSetting = fineControlMode ? m_targetSpeedSetting : 0.0;
    if (m_engine.IsKeyDown(ysKey::Code::Q)) {
        m_targetSpeedSetting = 0.01;
    }
    else if (m_engine.IsKeyDown(ysKey::Code::W)) {
        m_targetSpeedSetting = 0.1;
    }
    else if (m_engine.IsKeyDown(ysKey::Code::E)) {
        m_targetSpeedSetting = 0.2;
    }
    else if (m_engine.IsKeyDown(ysKey::Code::R)) {
        m_targetSpeedSetting = 1.0;
    }
    else if (fineControlMode && !fineControlInUse) {
        m_targetSpeedSetting = clamp(m_targetSpeedSetting + mouseWheelDelta * 0.0001);
    }

    if (m_canRemote) {
        m_targetSpeedSetting = clamp(m_hostDemand2.alphaDemand_percent / 100.0);
    }
    else if (prevTargetThrottle != m_targetSpeedSetting) {
        m_infoCluster->setLogMessage("Speed control set to " + std::to_string(m_targetSpeedSetting));
    }

    m_speedSetting = m_targetSpeedSetting * 0.5 + 0.5 * m_speedSetting;

    m_iceEngine->setSpeedControl(m_speedSetting);
    if (m_engine.ProcessKeyDown(ysKey::Code::M)) {
        const int currentLayer = getViewParameters().Layer0;
        if (currentLayer + 1 < m_iceEngine->getMaxDepth()) {
            setViewLayer(currentLayer + 1);
        }

        m_infoCluster->setLogMessage("[M] - Set render layer to " + std::to_string(getViewParameters().Layer0));
    }

    if (m_engine.ProcessKeyDown(ysKey::Code::OEM_Comma)) {
        if (getViewParameters().Layer0 - 1 >= 0)
            setViewLayer(getViewParameters().Layer0 - 1);

        m_infoCluster->setLogMessage("[,] - Set render layer to " + std::to_string(getViewParameters().Layer0));
    }

    if (m_engine.ProcessKeyDown(ysKey::Code::D) && !m_canRemote) {
        m_simulator->m_dyno.m_enabled = !m_simulator->m_dyno.m_enabled;

        const std::string msg = m_simulator->m_dyno.m_enabled
            ? "DYNOMOMETER ENABLED"
            : "DYNOMOMETER DISABLED";
        m_infoCluster->setLogMessage(msg);
    }

    if (m_engine.ProcessKeyDown(ysKey::Code::H) && !m_canRemote) {
        m_simulator->m_dyno.m_hold = !m_simulator->m_dyno.m_hold;

        const std::string msg = m_simulator->m_dyno.m_hold
            ? m_simulator->m_dyno.m_enabled ? "HOLD ENABLED" : "HOLD ON STANDBY [ENABLE DYNO. FOR HOLD]"
            : "HOLD DISABLED";
        m_infoCluster->setLogMessage(msg);
    }

    if (m_canRemote) {
        // Control_Mode_Command: 0 = hold Speed_Demand, 1 = brake with Torque_Demand
        const bool torqueMode = m_hostControl.controlMode == avl_emcon::ControlMode::Torque;

        const bool speedControl = m_hostControl.dynoOn && !torqueMode;
        const bool torqueControl = m_hostControl.dynoOn && torqueMode;

        // Ramp the active demand; the inactive one tracks the current operating
        // point so switching mode or enabling the dyno doesn't cause a step
        m_canRampedDynoSpeed = speedControl
            ? rampTowards(
                m_canRampedDynoSpeed,
                units::rpm(std::abs(m_hostDemand1.speedDemand_rpm)),
                m_canSpeedRampRate * dt)
            : m_iceEngine->getSpeed();
        m_canRampedDynoTorque = torqueControl
            ? rampTowards(
                m_canRampedDynoTorque,
                units::torque(m_hostDemand1.torqueDemand_Nm, units::Nm),
                m_canTorqueRampRate * dt)
            : std::fmax(m_simulator->getFilteredDynoTorque(), 0.0);

        m_simulator->m_dyno.m_enabled = m_hostControl.dynoOn;
        m_simulator->m_dyno.m_torqueMode = torqueMode;
        m_simulator->m_dyno.m_hold = !torqueMode;
        m_simulator->m_dyno.m_torqueSetpoint = m_canRampedDynoTorque;
        m_dynoSpeed = m_canRampedDynoSpeed;
    }
    else if (m_simulator->m_dyno.m_enabled) {
        if (!m_simulator->m_dyno.m_hold) {
            if (m_simulator->getFilteredDynoTorque() > units::torque(1.0, units::ft_lb)) {
                m_dynoSpeed += units::rpm(500) * dt;
            }
            else {
                m_dynoSpeed *= (1 / (1 + dt));
            }

            if (m_dynoSpeed > m_iceEngine->getRedline()) {
                m_simulator->m_dyno.m_enabled = false;
                m_dynoSpeed = units::rpm(0);
            }
        }
    }
    else {
        if (!m_simulator->m_dyno.m_hold) {
            m_dynoSpeed = units::rpm(0);
        }
    }

    // Under CAN control the HOST owns the speed range; only reject absurd demands.
    // The script limits (dyno_min_speed / dyno_max_speed) apply to the keyboard.
    const double minDynoSpeed = m_canRemote ? 0.0 : m_iceEngine->getDynoMinSpeed();
    const double maxDynoSpeed = m_canRemote
        ? 1.2 * m_iceEngine->getRedline()
        : m_iceEngine->getDynoMaxSpeed();
    m_dynoSpeed = clamp(m_dynoSpeed, minDynoSpeed, maxDynoSpeed);
    m_simulator->m_dyno.m_rotationSpeed = m_dynoSpeed;

    const bool prevStarterEnabled = m_simulator->m_starterMotor.m_enabled;
    if (m_canRemote) {
        m_simulator->m_starterMotor.m_enabled = m_hostControl.starter;
    }
    else if (m_engine.IsKeyDown(ysKey::Code::S)) {
        m_simulator->m_starterMotor.m_enabled = true;
    }
    else {
        m_simulator->m_starterMotor.m_enabled = false;
    }

    if (prevStarterEnabled != m_simulator->m_starterMotor.m_enabled) {
        const std::string msg = m_simulator->m_starterMotor.m_enabled
            ? "STARTER ENABLED"
            : "STARTER DISABLED";
        m_infoCluster->setLogMessage(msg);
    }

    IgnitionModule *ignitionModule = m_simulator->getEngine()->getIgnitionModule();
    const bool prevIgnitionEnabled = ignitionModule->m_enabled;
    const bool ignitionKeyPressed = m_engine.ProcessKeyDown(ysKey::Code::A);
    if (m_canRemote) {
        ignitionModule->m_enabled = m_hostControl.ignitionKey;
    }
    else if (ignitionKeyPressed) {
        ignitionModule->m_enabled = !ignitionModule->m_enabled;
    }

    if (prevIgnitionEnabled != ignitionModule->m_enabled) {
        const std::string msg = ignitionModule->m_enabled
            ? "IGNITION ENABLED"
            : "IGNITION DISABLED";
        m_infoCluster->setLogMessage(msg);
    }

    if (m_engine.ProcessKeyDown(ysKey::Code::Up)) {
        m_simulator->getTransmission()->changeGear(m_simulator->getTransmission()->getGear() + 1);

        m_infoCluster->setLogMessage(
            "UPSHIFTED TO " + std::to_string(m_simulator->getTransmission()->getGear() + 1));
    }
    else if (m_engine.ProcessKeyDown(ysKey::Code::Down)) {
        m_simulator->getTransmission()->changeGear(m_simulator->getTransmission()->getGear() - 1);

        if (m_simulator->getTransmission()->getGear() != -1) {
            m_infoCluster->setLogMessage(
                "DOWNSHIFTED TO " + std::to_string(m_simulator->getTransmission()->getGear() + 1));
        }
        else {
            m_infoCluster->setLogMessage("SHIFTED TO NEUTRAL");
        }
    }

    if (m_engine.IsKeyDown(ysKey::Code::T)) {
        m_targetClutchPressure -= 0.2 * dt;
    }
    else if (m_engine.IsKeyDown(ysKey::Code::U)) {
        m_targetClutchPressure += 0.2 * dt;
    }
    else if (m_engine.IsKeyDown(ysKey::Code::Shift)) {
        m_targetClutchPressure = 0.0;
        m_infoCluster->setLogMessage("CLUTCH DEPRESSED");
    }
    else if (!m_engine.IsKeyDown(ysKey::Code::Y)) {
        m_targetClutchPressure = 1.0;
    }

    m_targetClutchPressure = clamp(m_targetClutchPressure);

    double clutchRC = 0.001;
    if (m_engine.IsKeyDown(ysKey::Code::Space)) {
        clutchRC = 1.0;
    }

    const double clutch_s = dt / (dt + clutchRC);
    m_clutchPressure = m_clutchPressure * (1 - clutch_s) + m_targetClutchPressure * clutch_s;
    m_simulator->getTransmission()->setClutchPressure(m_clutchPressure);
}

void EngineSimApplication::renderScene() {
    getShaders()->ResetBaseColor();
    getShaders()->SetObjectTransform(ysMath::LoadIdentity());

    m_textRenderer.SetColor(ysColor::linearToSrgb(m_foreground));
    m_shaders.SetClearColor(ysColor::linearToSrgb(m_shadow));

    const int screenWidth = m_engine.GetGameWindow()->GetGameWidth();
    const int screenHeight = m_engine.GetGameWindow()->GetGameHeight();
    const float aspectRatio = screenWidth / (float)screenHeight;

    const Point cameraPos = m_engineView->getCameraPosition();
    m_shaders.m_cameraPosition = ysMath::LoadVector(cameraPos.x, cameraPos.y);

    m_shaders.CalculateUiCamera(screenWidth, screenHeight);

    if (m_screen == 0) {
        Bounds windowBounds((float)screenWidth, (float)screenHeight, { 0, (float)screenHeight });
        Grid grid;
        grid.v_cells = 2;
        grid.h_cells = 3;
        Grid grid3x3;
        grid3x3.v_cells = 3;
        grid3x3.h_cells = 3;
        m_engineView->setDrawFrame(true);
        m_engineView->setBounds(grid.get(windowBounds, 1, 0, 1, 1));
        m_engineView->setLocalPosition({ 0, 0 });

        m_rightGaugeCluster->m_bounds = grid.get(windowBounds, 2, 0, 1, 2);
        m_oscCluster->m_bounds = grid.get(windowBounds, 1, 1);
        m_performanceCluster->m_bounds = grid3x3.get(windowBounds, 0, 1);
        m_loadSimulationCluster->m_bounds = grid3x3.get(windowBounds, 0, 2);

        Grid grid1x3;
        grid1x3.v_cells = 3;
        grid1x3.h_cells = 1;
        m_mixerCluster->m_bounds = grid1x3.get(grid3x3.get(windowBounds, 0, 0), 0, 2);
        m_infoCluster->m_bounds = grid1x3.get(grid3x3.get(windowBounds, 0, 0), 0, 0, 1, 2);

        m_engineView->setVisible(true);
        m_rightGaugeCluster->setVisible(true);
        m_oscCluster->setVisible(true);
        m_performanceCluster->setVisible(true);
        m_loadSimulationCluster->setVisible(true);
        m_mixerCluster->setVisible(true);
        m_infoCluster->setVisible(true);

        m_oscCluster->activate();
    }
    else if (m_screen == 1) {
        Bounds windowBounds((float)screenWidth, (float)screenHeight, { 0, (float)screenHeight });
        m_engineView->setDrawFrame(false);
        m_engineView->setBounds(windowBounds);
        m_engineView->setLocalPosition({ 0, 0 });
        m_engineView->activate();

        m_engineView->setVisible(true);
        m_rightGaugeCluster->setVisible(false);
        m_oscCluster->setVisible(false);
        m_performanceCluster->setVisible(false);
        m_loadSimulationCluster->setVisible(false);
        m_mixerCluster->setVisible(false);
        m_infoCluster->setVisible(false);
    }
    else if (m_screen == 2) {
        Bounds windowBounds((float)screenWidth, (float)screenHeight, { 0, (float)screenHeight });
        Grid grid;
        grid.v_cells = 1;
        grid.h_cells = 3;
        m_engineView->setDrawFrame(true);
        m_engineView->setBounds(grid.get(windowBounds, 0, 0, 2, 1));
        m_engineView->setLocalPosition({ 0, 0 });
        m_engineView->activate();

        m_rightGaugeCluster->m_bounds = grid.get(windowBounds, 2, 0, 1, 1);

        m_engineView->setVisible(true);
        m_rightGaugeCluster->setVisible(true);
        m_oscCluster->setVisible(false);
        m_performanceCluster->setVisible(false);
        m_loadSimulationCluster->setVisible(false);
        m_mixerCluster->setVisible(false);
        m_infoCluster->setVisible(false);
    }

    const float cameraAspectRatio =
        m_engineView->m_bounds.width() / m_engineView->m_bounds.height();
    m_engine.GetDevice()->ResizeRenderTarget(
        m_mainRenderTarget,
        m_engineView->m_bounds.width(),
        m_engineView->m_bounds.height(),
        m_engineView->m_bounds.width(),
        m_engineView->m_bounds.height()
    );
    m_engine.GetDevice()->RepositionRenderTarget(
        m_mainRenderTarget,
        m_engineView->m_bounds.getPosition(Bounds::tl).x,
        screenHeight - m_engineView->m_bounds.getPosition(Bounds::tl).y
    );
    m_shaders.CalculateCamera(
        cameraAspectRatio * m_displayHeight / m_engineView->m_zoom,
        m_displayHeight / m_engineView->m_zoom,
        m_engineView->m_bounds,
        m_screenWidth,
        m_screenHeight,
        m_displayAngle);

    m_geometryGenerator.reset();

    render();

    m_engine.GetDevice()->EditBufferDataRange(
        m_geometryVertexBuffer,
        (char *)m_geometryGenerator.getVertexData(),
        sizeof(dbasic::Vertex) * m_geometryGenerator.getCurrentVertexCount(),
        0);

    m_engine.GetDevice()->EditBufferDataRange(
        m_geometryIndexBuffer,
        (char *)m_geometryGenerator.getIndexData(),
        sizeof(unsigned short) * m_geometryGenerator.getCurrentIndexCount(),
        0);
}

void EngineSimApplication::refreshUserInterface() {
    m_uiManager.destroy();
    m_uiManager.initialize(this);

    m_engineView = m_uiManager.getRoot()->addElement<EngineView>();
    m_rightGaugeCluster = m_uiManager.getRoot()->addElement<RightGaugeCluster>();
    m_oscCluster = m_uiManager.getRoot()->addElement<OscilloscopeCluster>();
    m_performanceCluster = m_uiManager.getRoot()->addElement<PerformanceCluster>();
    m_loadSimulationCluster = m_uiManager.getRoot()->addElement<LoadSimulationCluster>();
    m_mixerCluster = m_uiManager.getRoot()->addElement<MixerCluster>();
    m_infoCluster = m_uiManager.getRoot()->addElement<InfoCluster>();

    m_infoCluster->setEngine(m_iceEngine);
    m_rightGaugeCluster->m_simulator = m_simulator;
    m_rightGaugeCluster->setEngine(m_iceEngine);
    m_oscCluster->setSimulator(m_simulator);
    if (m_iceEngine != nullptr) {
        m_oscCluster->setDynoMaxRange(units::toRpm(m_iceEngine->getRedline()));
    }
    m_performanceCluster->setSimulator(m_simulator);
    m_loadSimulationCluster->setSimulator(m_simulator);
    m_mixerCluster->setSimulator(m_simulator);
}

void EngineSimApplication::startRecording() {
    m_recording = true;

#ifdef ATG_ENGINE_SIM_VIDEO_CAPTURE
    atg_dtv::Encoder::VideoSettings settings{};

    // Output filename
    settings.fname = "../workspace/video_capture/engine_sim_video_capture.mp4";
    settings.inputWidth = m_engine.GetScreenWidth();
    settings.inputHeight = m_engine.GetScreenHeight();
    settings.width = settings.inputWidth;
    settings.height = settings.inputHeight;
    settings.hardwareEncoding = true;
    settings.inputAlpha = true;
    settings.bitRate = 40000000;

    m_encoder.run(settings, 2);
#endif /* ATG_ENGINE_SIM_VIDEO_CAPTURE */
}

void EngineSimApplication::updateScreenSizeStability() {
    m_screenResolution[m_screenResolutionIndex][0] = m_engine.GetScreenWidth();
    m_screenResolution[m_screenResolutionIndex][1] = m_engine.GetScreenHeight();

    m_screenResolutionIndex = (m_screenResolutionIndex + 1) % ScreenResolutionHistoryLength;
}

bool EngineSimApplication::readyToRecord() {
    const int w = m_screenResolution[0][0];
    const int h = m_screenResolution[0][1];

    if (w <= 0 && h <= 0) return false;
    if ((w % 2) != 0 || (h % 2) != 0) return false;

    for (int i = 1; i < ScreenResolutionHistoryLength; ++i) {
        if (m_screenResolution[i][0] != w) return false;
        if (m_screenResolution[i][1] != h) return false;
    }

    return true;
}

void EngineSimApplication::stopRecording() {
    m_recording = false;

#ifdef ATG_ENGINE_SIM_VIDEO_CAPTURE
    m_encoder.commit();
    m_encoder.stop();
#endif /* ATG_ENGINE_SIM_VIDEO_CAPTURE */
}

void EngineSimApplication::recordFrame() {
#ifdef ATG_ENGINE_SIM_VIDEO_CAPTURE
    atg_dtv::Frame *frame = m_encoder.newFrame(false);
    if (frame != nullptr && m_encoder.getError() == atg_dtv::Encoder::Error::None) {
        m_engine.GetDevice()->ReadRenderTarget(m_engine.GetScreenRenderTarget(), frame->m_rgb);
    }

    m_encoder.submitFrame();
#endif /* ATG_ENGINE_SIM_VIDEO_CAPTURE */
}
