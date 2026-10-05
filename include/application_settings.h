#ifndef ATG_ENGINE_SIM_APPLICATION_SETTINGS_H
#define ATG_ENGINE_SIM_APPLICATION_SETTINGS_H
#include <string>

struct ApplicationSettings {
    bool startFullscreen = false;
    std::string powerUnits = "hp";
    std::string torqueUnits = "lb-ft";
    std::string speedUnits = "mph";
    std::string pressureUnits = "inHg";
    std::string boostUnits = "psi";

    int colorBackground = 0x0E1012;
    int colorForeground = 0xFFFFFF;
    int colorShadow = 0x0E1012;
    int colorHighlight1 = 0xEF4545;
    int colorHighlight2 = 0xFFFFFF;
    int colorPink = 0xF394BE;
    int colorRed = 0xEE4445;
    int colorOrange = 0xF4802A;
    int colorYellow = 0xFDBD2E;
    int colorBlue = 0x77CEE0;
    int colorGreen = 0xBDD869;
};

// Virtual ECU settings from the script (set_ecu_settings)
struct EcuScriptSettings {
    bool defined = false;           // set_ecu_settings was called
    bool xcp = false;               // XCP on Ethernet server
    std::string xcpAddress = "0.0.0.0";
    int xcpPort = 5555;
    std::string xcpClock = "sim";   // "sim" or "real"

    // External ECU (ecu-standalone) over UDP instead of the ECU in the
    // simulator
    bool link = false;
    std::string linkAddress = "127.0.0.1";
    int linkPort = 5600;
    std::string linkSync = "lockstep";  // "lockstep" or "free"
};

#endif /* ATG_ENGINE_SIM_APPLICATION_SETTINGS_H */
