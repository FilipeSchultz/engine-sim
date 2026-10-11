#ifndef ATG_ENGINE_SIM_ENGINE_NODE_H
#define ATG_ENGINE_SIM_ENGINE_NODE_H

#include "object_reference_node.h"

#include "crankshaft_node.h"
#include "cylinder_bank_node.h"
#include "ignition_module_node.h"
#include "engine_context.h"
#include "fuel_node.h"
#include "throttle_nodes.h"

#include "engine_sim.h"

#include <map>
#include <set>

namespace es_script {

    class EngineNode : public ObjectReferenceNode<EngineNode> {
    public:
        EngineNode() { /* void */ }
        virtual ~EngineNode() { /* void */ }

        void buildEngine(Engine *engine) {
            int cylinderCount = 0;
            for (const CylinderBankNode *bank : m_cylinderBanks) {
                cylinderCount += bank->getCylinderCount();
            }

            std::set<ExhaustSystemNode *> exhaustSystems;
            std::set<IntakeNode *> intakes;
            for (const CylinderBankNode *bank : m_cylinderBanks) {
                const int n = bank->getCylinderCount();
                for (int i = 0; i < n; ++i) {
                    exhaustSystems.insert(bank->getCylinder(i).exhaust);
                    intakes.insert(bank->getCylinder(i).intake);
                }
            }

            EngineContext context;
            context.setEngine(engine);

            Engine::Parameters parameters = m_parameters;
            parameters.crankshaftCount = (int)m_crankshafts.size();
            parameters.cylinderBanks = (int)m_cylinderBanks.size();
            parameters.cylinderCount = cylinderCount;
            parameters.exhaustSystemCount = (int)exhaustSystems.size();
            parameters.intakeCount = (int)intakes.size();
            parameters.throttle = m_throttle->generate();
            engine->initialize(parameters);

            {
                int i = 0;
                for (ExhaustSystemNode *exhaust : exhaustSystems) {
                    context.addExhaust(
                        exhaust, engine->getExhaustSystem(i++));
                }
            }

            {
                int i = 0;
                for (IntakeNode *intake : intakes) {
                    context.addIntake(
                        intake, engine->getIntake(i++));
                }
            }

            {
                int i = 0;
                for (const CylinderBankNode *bank : m_cylinderBanks) {
                    context.addHead(bank->getCylinderHead(), engine->getHead(i++));
                }
            }

            for (const CylinderBankNode *bank : m_cylinderBanks) {
                const int n = bank->getCylinderCount();
                for (int i = 0; i < n; ++i) {
                    exhaustSystems.insert(bank->getCylinder(i).exhaust);
                    intakes.insert(bank->getCylinder(i).intake);
                }
            }

            for (int i = 0; i < parameters.crankshaftCount; ++i) {
                m_crankshafts[i]->generate(engine->getCrankshaft(i), &context);
            }

            for (int i = 0; i < parameters.cylinderBanks; ++i) {
                m_cylinderBanks[i]->indexSlaveJournals(&context);
            }

            int cylinderIndex = 0;
            for (int i = 0; i < parameters.cylinderBanks; ++i) {
                m_cylinderBanks[i]->generate(
                    i,
                    cylinderIndex,
                    engine->getCylinderBank(i),
                    engine->getCrankshaft(0),
                    engine,
                    &context);
                cylinderIndex += m_cylinderBanks[i]->getCylinderCount();
            }

            for (int i = 0; i < parameters.cylinderBanks; ++i) {
                m_cylinderBanks[i]->connectRodAssemblies(&context);
            }

            m_ignitionModule->generate(engine, &context);
            
            Function *meanPistonSpeedToTurbulence = new Function;
            meanPistonSpeedToTurbulence->initialize(30, 1);
            for (int i = 0; i < 30; ++i) {
                const double s = (double)i;
                meanPistonSpeedToTurbulence->addSample(s, s * m_turbulenceRatio);
            }

            Fuel *fuel = engine->getFuel();
            m_fuel->generate(fuel, &context);

            CombustionChamber::Parameters ccParams;
            ccParams.CrankcasePressure = units::pressure(1.0, units::atm);
            ccParams.Fuel = fuel;
            ccParams.StartingPressure = units::pressure(1.0, units::atm);
            ccParams.StartingTemperature = units::celcius(25.0);
            ccParams.MeanPistonSpeedToTurbulence = meanPistonSpeedToTurbulence;
            ccParams.WallHeatTransfer = m_wallHeatTransfer;
            ccParams.WallTemperature = m_wallTemperature;
            ccParams.FlameModel = m_flameModel;
            ccParams.SparkPlugOffset = m_sparkPlugOffset;
            ccParams.CycleVariation = m_cycleVariation;
            ccParams.GasModel = m_gasModel;
            ccParams.MixtureNonuniformity = m_mixtureNonuniformity;
            ccParams.WallFilmFraction = m_wallFilmFraction;
            ccParams.WallFilmTimeConstant = m_wallFilmTimeConstant;
            ccParams.CoolantTemperature = m_parameters.coolantTemperature;

            for (int i = 0; i < engine->getCylinderCount(); ++i) {
                ccParams.Piston = engine->getPiston(i);
                ccParams.Head = engine->getHead(ccParams.Piston->getCylinderBank()->getIndex());
                engine->getChamber(i)->initialize(ccParams);
            }

            for (int i = 0; i < engine->getIntakeCount(); ++i) {
                engine->getIntake(i)->setHeatCapacityModel(m_gasModel);
            }
            for (int i = 0; i < engine->getExhaustSystemCount(); ++i) {
                engine->getExhaustSystem(i)->setHeatCapacityModel(m_gasModel);
            }
        }

        void addCrankshaft(CrankshaftNode *crankshaft) {
            m_crankshafts.push_back(crankshaft);
        }

        void addCylinderBank(CylinderBankNode *bank) {
            m_cylinderBanks.push_back(bank);
        }

        int getIgnitionModuleCount() const {
            return m_ignitionModule == nullptr
                ? 0
                : 1;
        }

        void addIgnitionModule(IgnitionModuleNode *ignitionModule) {
            m_ignitionModule = ignitionModule;
        }

    protected:
        virtual void registerInputs() {
            addInput("name", &m_parameters.name);
            addInput("starter_torque", &m_parameters.starterTorque);
            addInput("starter_speed", &m_parameters.starterSpeed);
            addInput("dyno_min_speed", &m_parameters.dynoMinSpeed);
            addInput("dyno_max_speed", &m_parameters.dynoMaxSpeed);
            addInput("dyno_hold_step", &m_parameters.dynoHoldStep);
            addInput("redline", &m_parameters.redline);
            addInput("fuel", &m_fuel, InputTarget::Type::Object);
            addInput("throttle", &m_throttle, InputTarget::Type::Object);
            addInput("simulation_frequency", &m_parameters.initialSimulationFrequency);
            addInput("hf_gain", &m_parameters.initialHighFrequencyGain);
            addInput("jitter", &m_parameters.initialJitter);
            addInput("noise", &m_parameters.initialNoise);
            addInput("injector_flow", &m_parameters.injectorFlow_ccmin);
            addInput("injector_dead_time", &m_parameters.injectorDeadTime_ms);
            addInput("wall_film_fraction", &m_wallFilmFraction);
            addInput("wall_film_time_constant", &m_wallFilmTimeConstant);
            addInput("coolant_temperature", &m_parameters.coolantTemperature);
            addInput("lambda_sensor_volume", &m_parameters.lambdaSensorVolume);
            addInput("lambda_sensor_time_constant", &m_parameters.lambdaSensorTimeConstant);
            addInput("turbulence_ratio", &m_turbulenceRatio);
            addInput("wall_heat_transfer", &m_wallHeatTransfer);
            addInput("wall_temperature", &m_wallTemperature);
            addInput("flame_model", &m_flameModel);
            addInput("spark_plug_offset", &m_sparkPlugOffset);
            addInput("cycle_variation", &m_cycleVariation);
            addInput("gas_model", &m_gasModel);
            addInput("mixture_nonuniformity", &m_mixtureNonuniformity);

            ObjectReferenceNode<EngineNode>::registerInputs();
        }

        virtual void _evaluate() {
            setOutput(this);

            // Read inputs
            readAllInputs();
        }

        ThrottleNode *m_throttle = nullptr;
        IgnitionModuleNode *m_ignitionModule = nullptr;
        FuelNode *m_fuel = nullptr;

        Engine::Parameters m_parameters;
        double m_turbulenceRatio = 0.5;     // turbulence intensity / mean piston speed
        double m_wallHeatTransfer = 0.0;    // Hohenberg multiplier, 0 = original model
        double m_wallTemperature = units::celcius(90.0);    // mean combustion chamber surface
        int m_flameModel = 0;               // 0 = original, 1 = flame speed updated every step, 2 = spherical
        double m_sparkPlugOffset = 0.0;     // flame model 2: radial plug position / bore radius
        double m_cycleVariation = 0.0;      // flame model 2: std. deviation of the turbulence per cycle
        int m_gasModel = 0;                 // 0 = constant gamma, 1 = variable heat capacities + rich chemistry
        double m_mixtureNonuniformity = 0.1;    // gas model 1: std. deviation of local phi / phi
        double m_wallFilmFraction = 0.0;        // port injection: fraction deposited on the walls (90 C, 100 kPa)
        double m_wallFilmTimeConstant = 0.3;    // port injection: film evaporation time constant [s] (90 C, 100 kPa)
        std::vector<CrankshaftNode *> m_crankshafts;
        std::vector<CylinderBankNode *> m_cylinderBanks;
    };

} /* namespace es_script */

#endif /* ATG_ENGINE_SIM_ENGINE_NODE_H */
