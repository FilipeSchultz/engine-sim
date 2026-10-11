#ifndef ATG_ENGINE_SIM_ENGINE_SIM_APPLICATION_H
#define ATG_ENGINE_SIM_ENGINE_SIM_APPLICATION_H

#include "geometry_generator.h"
#include "simulator.h"
#include "engine.h"
#include "simulation_object.h"
#include "ui_manager.h"
#include "dynamometer.h"
#include "oscilloscope.h"
#include "audio_buffer.h"
#include "convolution_filter.h"
#include "shaders.h"
#include "engine_view.h"
#include "right_gauge_cluster.h"
#include "cylinder_temperature_gauge.h"
#include "synthesizer.h"
#include "oscilloscope_cluster.h"
#include "performance_cluster.h"
#include "load_simulation_cluster.h"
#include "mixer_cluster.h"
#include "info_cluster.h"
#include "application_settings.h"
#include "transmission.h"
#include "can_interface.h"
#include "avl_emcon.h"
#include "dyno_can.h"
#include "ecu_hardware.h"
#include "indicating_system.h"
#include "udp_sender.h"

#include "delta.h"
#include "dtv.h"

#include <filesystem>
#include <fstream>
#include <random>
#include <vector>

class EngineSimApplication {
    private:
        static std::string s_buildVersion;

    public:
        EngineSimApplication();
        virtual ~EngineSimApplication();

        static std::string getBuildVersion() { return s_buildVersion; }

        void initialize(void *instance, ysContextObject::DeviceAPI api);
        void run();
        void destroy();

        void loadEngine(Engine *engine, Vehicle *vehicle, Transmission *transmission);
        void drawGenerated(
                const GeometryGenerator::GeometryIndices &indices,
                int layer = 0);
        void drawGeneratedUi(
                const GeometryGenerator::GeometryIndices &indices,
                int layer = 0);
        void drawGenerated(
                const GeometryGenerator::GeometryIndices &indices,
                int layer,
                dbasic::StageEnableFlags flags);
        void configure(const ApplicationSettings &settings);
        GeometryGenerator *getGeometryGenerator() { return &m_geometryGenerator; }

        Shaders *getShaders() { return &m_shaders; }
        dbasic::TextRenderer *getTextRenderer() { return &m_textRenderer; }

        void createObjects(Engine *engine);
        void destroyObjects();
        dbasic::DeltaEngine *getEngine() { return &m_engine; }

        float pixelsToUnits(float pixels) const;
        float unitsToPixels(float units) const;

        ysVector getBackgroundColor() const { return m_background; }
        ysVector getForegroundColor() const { return m_foreground; }
        ysVector getHightlight1Color() const { return m_highlight1; }
        ysVector getPink() const { return m_pink; }
        ysVector getGreen() const { return m_green; }
        ysVector getYellow() const { return m_yellow; }
        ysVector getRed() const { return m_red; }
        ysVector getOrange() const { return m_orange; }
        ysVector getBlue() const { return m_blue; }

        const SimulationObject::ViewParameters &getViewParameters() const;
        void setViewLayer(int view) { m_viewParameters.Layer0 = view; }

        dbasic::AssetManager *getAssetManager() { return &m_assetManager; }

        int getScreenWidth() const { return m_screenWidth; }
        int getScreenHeight() const { return m_screenHeight; }

        Simulator *getSimulator() { return m_simulator; }
        InfoCluster *getInfoCluster() { return m_infoCluster; }
        ApplicationSettings* getAppSettings() { return &m_applicationSettings; }

    protected:
        void loadScript();
        void processEngineInput();
        void renderScene();

        void refreshUserInterface();

        void initializeEcu();
        void initializeAutostart();
        void updateEcu(double dt);
        void initializeIndicating();
        void attachIndicating();
        void updateIndicating(double dt);
        void sendIndicatingData();
        void sendIndicatingInfo();
        void onEcuSettings(const EcuScriptSettings &settings);

        void openCan();
        void updateCanReceive(double dt);
        void updateCanTransmit();
        void setCanRemote(bool remote);
        bool isCanRemote() const { return m_canRemote; }

        void configureDyno();
        void updateDyno(double dt);

    protected:
        // The ECU's I/O controller; the ECU is ecu-standalone. Optional CSV
        // datalog of the link (ENGINE_SIM_ECU_LOG)
        EcuHardware m_ecuHardware;
        EcuScriptSettings m_ecuScriptSettings;
        EcuHardware::LinkState m_ecuLinkState = EcuHardware::LinkState::Off;
        std::ofstream m_ecuLinkLog;
        double m_ecuLinkLogTime = 0.0;
        double m_ecuLinkLogTimer = 0.0;
        bool m_ecuSettingsLoaded = false;

        // Combustion analysis per cycle (ENGINE_SIM_INDI_LOG) and raw cylinder
        // pressure stream for the external indicating tool (ENGINE_SIM_INDI_UDP)
        IndicatingSystem m_indicating;
        std::ofstream m_indicatingLog;
        int m_indicatingLogLines = 0;
        UdpSender m_indicatingUdp;
        uint32_t m_indicatingSequence = 0;
        double m_indicatingInfoTimer = 0.0;

        // Pressure sensor model: gaussian noise and a fixed offset per cylinder
        // (piezo sensors only measure pressure changes; the tool must peg them)
        double m_sensorNoise_bar = 0.0;
        std::vector<float> m_sensorOffset_bar;
        std::mt19937 m_sensorRng;

    protected:
        CanInterface m_can;

        // CAN protocol: the bench's (dyno_can.h, default) or AVL EMCON
        // (ENGINE_SIM_CAN_PROTOCOL=emcon)
        bool m_canEmcon = false;

        // Engine commands of the automation in remote, from either protocol
        bool m_remoteIgnition = false;
        bool m_remoteStarter = false;
        double m_remoteThrottle = 0.0;      // 0 .. 1

        // Bench protocol: remote while ENGINE_Command keeps coming
        double m_engineCommandAge = 1e9;    // [s]
        int m_canTimeoutMs = 500;

        // Latest commands received from the HOST (AVL EMCON)
        avl_emcon::HostControl m_hostControl;
        avl_emcon::HostDemand1 m_hostDemand1;
        avl_emcon::HostDemand2 m_hostDemand2;

        // HOST has control of throttle, ignition, starter and dyno
        bool m_canRemote = false;

        // Lifesignal timeout (0 = check disabled) and fault latched until Reset_Key
        int m_canLifesignalTimeoutMs = 0;
        double m_canLifesignalAge = 0.0;
        uint16_t m_canLastLifesignalFeedback = 0;
        bool m_canError = false;
        bool m_canPrevResetKey = false;

        // Dyno demand ramps (0 = no ramp): the dyno moves towards Speed_Demand /
        // Torque_Demand at a limited rate, starting from the current operating point
        double m_canSpeedRampRate = 0.0;    // [rad/s per s]
        double m_canTorqueRampRate = 0.0;   // [N m per s]
        double m_canRampedDynoSpeed = 0.0;
        double m_canRampedDynoTorque = 0.0;

        // Load cell model for the reported Dyno_Torque: gaussian noise followed by
        // a first-order filter
        double m_canTorqueNoise_Nm = 0.0;
        double m_canTorqueFilterTau = 0.0;  // [s]
        double m_canMeasuredTorque_Nm = 0.0;
        std::mt19937 m_canRng;

        // Eddy current brake (default; ENGINE_SIM_DYNO_MODEL=ideal: ideal dyno): PWM from the brake
        // controller's CAN protocol (dyno_can.h) or its internal controller.
        // Optional CSV datalog (ENGINE_SIM_DYNO_LOG)
        bool m_eddyDyno = false;
        uint32_t m_dynoCanBaseId = dyno_can::DefaultBaseId;
        bool m_dynoOverPower = false;
        std::ofstream m_dynoLog;
        double m_dynoLogTime = 0.0;
        double m_dynoLogTimer = 0.0;

    protected:
        double m_speedSetting = 1.0;
        double m_targetSpeedSetting = 1.0;

        double m_clutchPressure = 1.0;
        double m_targetClutchPressure = 1.0;
        int m_lastMouseWheel = 0;

    protected:
        virtual void initialize();
        virtual void process(float dt);
        virtual void render();

        float m_displayAngle;
        float m_displayHeight;
        int m_gameWindowHeight;
        int m_screenWidth;
        int m_screenHeight;
        
        ApplicationSettings m_applicationSettings;
        dbasic::ShaderSet m_shaderSet;
        Shaders m_shaders;

        dbasic::DeltaEngine m_engine;
        dbasic::AssetManager m_assetManager;

        std::string m_assetPath;

        ysRenderTarget *m_mainRenderTarget;
        ysGPUBuffer *m_geometryVertexBuffer;
        ysGPUBuffer *m_geometryIndexBuffer;

        GeometryGenerator m_geometryGenerator;
        dbasic::TextRenderer m_textRenderer;

        std::vector<SimulationObject *> m_objects;
        Engine *m_iceEngine;
        Vehicle *m_vehicle;
        Transmission *m_transmission;
        Simulator *m_simulator;
        double m_dynoSpeed;

        // Unattended start (ENGINE_SIM_AUTOSTART): starter time left [s]
        double m_autostartTimer = 0.0;
        bool m_autostartIgnition = false;
        double m_autostartDynoSpeed = 0.0;
        double m_restingThrottle = 0.0;         // with no throttle key pressed
        double m_torque;

        UiManager m_uiManager;
        EngineView *m_engineView;
        RightGaugeCluster *m_rightGaugeCluster;
        OscilloscopeCluster *m_oscCluster;
        CylinderTemperatureGauge *m_temperatureGauge;
        PerformanceCluster *m_performanceCluster;
        LoadSimulationCluster *m_loadSimulationCluster;
        MixerCluster *m_mixerCluster;
        InfoCluster *m_infoCluster;
        SimulationObject::ViewParameters m_viewParameters;

        bool m_paused;

    protected:
        void startRecording();
        void updateScreenSizeStability();
        bool readyToRecord();
        void stopRecording();
        void recordFrame();
        bool isRecording() const { return m_recording; }

        static constexpr int ScreenResolutionHistoryLength = 5;
        int m_screenResolution[ScreenResolutionHistoryLength][2];
        int m_screenResolutionIndex;
        bool m_recording;

        ysVector m_background;
        ysVector m_foreground;
        ysVector m_shadow;
        ysVector m_highlight1;
        ysVector m_highlight2;

        ysVector m_pink;
        ysVector m_orange;
        ysVector m_yellow;
        ysVector m_red;
        ysVector m_green;
        ysVector m_blue;

        ysAudioBuffer *m_outputAudioBuffer;
        AudioBuffer m_audioBuffer;
        ysAudioSource *m_audioSource;

        int m_oscillatorSampleOffset;
        int m_screen;

#ifdef ATG_ENGINE_SIM_VIDEO_CAPTURE
        atg_dtv::Encoder m_encoder;
#endif /* ATG_ENGINE_SIM_VIDEO_CAPTURE */
};

#endif /* ATG_ENGINE_SIM_ENGINE_SIM_APPLICATION_H */
