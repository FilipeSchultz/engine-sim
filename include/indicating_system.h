#ifndef ATG_ENGINE_SIM_INDICATING_SYSTEM_H
#define ATG_ENGINE_SIM_INDICATING_SYSTEM_H

#include <deque>
#include <string>
#include <vector>

class Engine;

// Combustion analysis as done by a test cell indicating system (AVL IndiCom,
// Kistler KiBox...): records each cylinder's pressure and volume against crank
// angle and, for every cycle, computes peak pressure, IMEP and the mass
// fraction burned (MFB) from the apparent heat release
//
//     dQ = g/(g-1) p dV + 1/(g-1) V dp
//
// Angles are in degrees relative to the cylinder's firing TDC (positive = after).
// The simulation's own burned-fuel counter gives a reference CA50 to validate
// the pressure based one.
class IndicatingSystem {
    public:
        struct CycleResult {
            double time = 0;                // simulation time at the end of the cycle [s]
            int cycle = 0;                  // per cylinder
            int cylinder = 0;               // 1..N
            double rpm = 0;                 // average over the cycle
            double sparkAdvance_deg = 0;    // before TDC
            bool sparkValid = false;
            double pmax_bar = 0;
            double pmaxAngle_deg = 0;
            double imepGross_bar = 0;       // compression + expansion
            double imepNet_bar = 0;         // full 720 deg cycle (includes pumping)
            double ca10_deg = 0;
            double ca50_deg = 0;
            double ca90_deg = 0;
            double ca50Reference_deg = 0;   // from the simulation's burned fuel
            bool misfire = false;

            // Gas exchange and energy balance (simulation truth, not available
            // to a real indicating system)
            double freshCharge_mg = 0;      // gas entering through the intake valve
            double trappedMass_mg = 0;      // cylinder content at intake valve closing (incl. ECU fuel)
            double residual_percent = 0;    // (trapped - fresh) / trapped, includes the ECU fuel
            double volumetricEfficiency_percent = 0;    // fresh charge vs intake air at 1 atm
            double fuelBurned_mg = 0;
            double indicatedEfficiency_percent = 0;     // net indicated work / fuel energy

            // Exhaust gas temperature: of the gas leaving the cylinder,
            // weighted by the exhaust flow (the "mass averaged" EGT), and the
            // time average of the gas in the exhaust port / runner
            double exhaustTempFlowWeighted_C = 0;
            double exhaustPortTemp_C = 0;
        };

        // Engine data an indicating tool needs (normally entered by the operator)
        struct EngineInfo {
            std::string name;
            int cylinders = 0;
            double bore_mm = 0;
            double stroke_mm = 0;
            double rodLength_mm = 0;
            double compressionRatio = 0;
            std::vector<double> tdcEncoderAngle_deg;    // firing TDC per cylinder, < 0 if unknown
        };

        static constexpr double HeatCapacityRatio = 1.33;

    public:
        IndicatingSystem();

        void reset();

        // Per-cycle analysis (CSV log) and raw sensor capture (external tool)
        void setAnalysis(bool enabled) { m_analysis = enabled; }
        void setRawCapture(bool enabled) { m_rawCapture = enabled; }

        // Once per physics step, after combustion and gas exchange
        void sample(Engine *engine, double dt);

        bool popResult(CycleResult *result);

        // Raw "sensor" records since the last call, one per physics step:
        // [encoder angle (deg, 0..720), pressure of cylinder 1..N (bar, absolute)]
        std::vector<float> &rawSamples() { return m_raw; }
        int getCylinderCount() const { return (int)m_cylinders.size(); }

        const EngineInfo &getEngineInfo() const { return m_info; }

    private:
        struct Sample {
            double angle;       // continuous crank angle [deg]
            double encoder;     // crank angle within the 720 deg cycle [deg]
            double time;        // [s]
            double pressure;    // [Pa]
            double volume;      // [m^3]
            double burnedFuel;  // cumulative [kg]
            bool valvesClosed;
            bool intakeOpen;
            double intakeFlow;  // gas entering the cylinder during this step [mol]
            double moles;       // cylinder content [mol]
            double exhaustFlow; // gas leaving the cylinder during this step [mol]
            double gasTemp;     // cylinder gas [K]
            double portTemp;    // exhaust port / runner gas [K]
        };

        struct Cylinder {
            std::deque<Sample> samples;
            std::deque<double> pendingTdc;  // firing TDC angles waiting for the full cycle
            double lastSparkAngle = -1e9;
            int cycles = 0;
            double minVolume = 1e9;
            double maxVolume = 0;
        };

        void analyze(int cylinder, double tdcAngle);
        void updateEngineInfo(Engine *engine);

        std::vector<Cylinder> m_cylinders;
        std::deque<CycleResult> m_results;
        double m_angle;
        double m_time;

        bool m_analysis;
        bool m_rawCapture;
        double m_fuelEnergyDensity;     // [J/kg]
        double m_airTemperature;        // intake air [K]
        std::vector<float> m_raw;
        EngineInfo m_info;
};

#endif /* ATG_ENGINE_SIM_INDICATING_SYSTEM_H */
