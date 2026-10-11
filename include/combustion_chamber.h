#ifndef ATG_ENGINE_SIM_COMBUSTION_CHAMBER_H
#define ATG_ENGINE_SIM_COMBUSTION_CHAMBER_H

#include "scs.h"

#include "piston.h"
#include "gas_system.h"
#include "cylinder_head.h"
#include "units.h"
#include "fuel.h"
#include "flame_geometry.h"

class Engine;
class CombustionChamber : public atg_scs::ForceGenerator {
    public:
        struct Parameters {
            Piston *Piston;
            CylinderHead *Head;
            Fuel *Fuel;
            Function *MeanPistonSpeedToTurbulence;

            double StartingPressure;
            double StartingTemperature;
            double CrankcasePressure;

            // Multiplier of the Hohenberg wall heat transfer coefficient;
            // 0 = original constant coefficient (nearly adiabatic)
            double WallHeatTransfer = 0.0;
            double WallTemperature = units::celcius(90.0);

            // 0 = original flame model (flame speed fixed at the spark)
            // 1 = flame speed updated every step from the unburned gas state
            // 2 = spherical flame with entrainment and burn-up
            int FlameModel = 0;

            // Flame model 2: radial spark plug position / bore radius, and the
            // standard deviation of the turbulence seen by each cycle's flame
            double SparkPlugOffset = 0.0;
            double CycleVariation = 0.0;

            // 1 = heat capacities varying with temperature and composition,
            // rich mixture chemistry and mixture non-uniformity
            int GasModel = 0;
            double MixtureNonuniformity = 0.0;

            // Port injection wall wetting (x-tau): fraction of each pulse
            // deposited on the port walls and the film's evaporation time
            // constant at 90 C and 100 kPa (0 = no film). The walls are at
            // the coolant temperature
            double WallFilmFraction = 0.0;
            double WallFilmTimeConstant = 0.3;
            double CoolantTemperature = units::celcius(90.0);
        };

        struct FlameEvent {
            double lit_n = 0;
            double total_n = 0;
            double percentageLit = 0;
            double efficiency = 1.0;
            double flameSpeed = 0.0;

            double lastVolume = 0.0;
            double travel_x = 0.0;
            double travel_y = 0.0;
            GasSystem::Mix globalMix;

            // Flame models 1 and 2: conditions at the spark
            double turbulence = 0.0;
            double phi = 1.0;
            double residual = 0.0;          // burned gas mole fraction
            double sparkPressure = 0.0;
            double sparkTemperature = 0.0;

            // Flame model 2: fractions of the charge entrained by the flame
            // front and burned, flame radius
            double entrained = 0.0;
            double burned = 0.0;
            double radius = 0.0;

            // Flame models 1 and 2: unburned gas temperature, compressed
            // isentropically since the spark
            double unburnedTemperature = 0.0;
            double lastPressure = 0.0;

            // Flame model 2: heat released and moles produced per mol of
            // charge when it burns, and the burned gas composition (set the
            // burned / unburned density ratio)
            double heatPerMol = 0.0;
            double moleGain = 1.0;
            GasSystem::Mix burnedMix;

            // Heat released / complete combustion of the O2-limited fuel
            // (gas model 1: rich mixture chemistry, mixture non-uniformity)
            double heatFactor = 1.0;
        };

        struct FrictionModelParams {
            double frictionCoeff = 0.06;
            double breakawayFriction = units::force(50, units::N);
            double breakawayFrictionVelocity = units::distance(0.1, units::m);
            double viscousFrictionCoefficient = units::force(20, units::N);
        };

    public:
        CombustionChamber();
        virtual ~CombustionChamber();

        void initialize(const Parameters &params);
        void destroy();
        void setEngine(Engine *engine) { m_engine = engine; }
        virtual void apply(atg_scs::SystemState *system);

        CylinderHead *getCylinderHead() const { return m_head; }
        Piston *getPiston() const { return m_piston; }
        bool isIntakeValveOpen() const { return m_intakeFlowRate > 0; }
        bool isExhaustValveOpen() const { return m_exhaustFlowRate > 0; }

        double getFrictionForce() const;
        double getVolume() const;
        double pistonSpeed() const;
        double calculateMeanPistonSpeed() const;
        double calculateFiringPressure() const;

        bool isLit() const { return m_lit; }
        bool popLitLastFrame();

        void ignite();
        void update(double dt);
        void flow(double dt);

        double lastEventAfr() const;

        double getLastIterationExhaustFlow() const { return m_exhaustFlow; }

        void resetLastTimestepExhaustFlow() { m_lastTimestepTotalExhaustFlow = 0; }
        double getLastTimestepExhaustFlow() const { return m_lastTimestepTotalExhaustFlow; }

        void resetLastTimestepIntakeFlow() { m_lastTimestepTotalIntakeFlow = 0; }
        double getLastTimestepIntakeFlow() const { return m_lastTimestepTotalIntakeFlow; }

        Function *m_meanPistonSpeedToTurbulence;
        GasSystem m_system;
        GasSystem m_intakeRunnerAndManifold;
        GasSystem m_exhaustRunnerAndPrimary;
        FlameEvent m_flameEvent;
        bool m_lit;

        FrictionModelParams m_frictionModel;

        double m_peakTemperature;
        double m_nBurntFuel;

        // Port injection, driven by the ECU hardware: each injector pulse adds
        // fuel to the port, where it waits until the intake valve opens (and
        // the exhaust valve has closed) and enters with the charge, so none of
        // it can flow back into the plenum or out of the exhaust. Part of the
        // pulse wets the port walls and reaches the port later, as the film
        // evaporates
        void setEcuFuelControl(bool active) {
            m_ecuFuelActive = active;
            if (!active) m_ecuPendingFuel = m_wallFilm = 0;
        }
        void injectFuel(double n);

        // Wall film model at the present port conditions
        double wallFilmFraction() const;
        double wallFilmTimeConstant() const;
        double getWallFilm() const { return m_wallFilm; }

        // Charge composition when the intake valve last closed (before
        // combustion): the exhaust gas of this cycle carries its lambda
        const GasSystem::Mix &getChargeMix() const { return m_chargeMix; }
        bool hasCharge() const { return m_chargeValid; }

        bool m_ecuFuelActive;
        double m_ecuPendingFuel;    // [mol]
        double m_wallFilm;          // [mol]

    protected:
        double calculateFrictionForce(double v) const;
        double unburnedTemperature() const { return m_flameEvent.unburnedTemperature; }
        void updateUnburnedTemperature();
        void convertRichFuel(double n_reacting, double fuelBurned);
        double unburnedFlameSpeed() const;
        void updateSphericalFlame(double dt, double volume);
        void updateCycleStates();

        double m_intakeFlowRate;
        double m_exhaustFlowRate;

        double m_manifoldToRunnerFlowRate;
        double m_primaryToCollectorFlowRate;
        double m_cylinderCrossSectionSurfaceArea;
        double m_cylinderWidthApproximation;

        double m_lastTimestepTotalExhaustFlow;
        double m_lastTimestepTotalIntakeFlow;
        double m_exhaustFlow;

        double m_crankcasePressure;
        double m_wallHeatTransfer;
        double m_wallTemperature;
        int m_flameModel;
        int m_gasModel;
        double m_mixtureNonuniformity;
        double m_sparkPlugOffset;
        double m_cycleVariation;
        double m_wallFilmFraction;
        double m_wallFilmTimeConstant;
        double m_coolantTemperature;
        GasSystem::Mix m_chargeMix;
        bool m_chargeValid;
        bool m_intakeWasOpen;
        FlameGeometry m_flameGeometry;
        double m_meanPistonSpeed;

        double *m_pressure;
        double *m_pistonSpeed;
        static constexpr int StateSamples = 256;

        bool m_litLastFrame;

        Piston *m_piston;
        CylinderHead *m_head;
        Engine *m_engine;
        Fuel *m_fuel;
};

#endif /* ATG_ENGINE_SIM_COMBUSTION_CHAMBER_H */
