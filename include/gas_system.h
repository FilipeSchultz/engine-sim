#ifndef ATG_ENGINE_SIM_GAS_SYSTEM_H
#define ATG_ENGINE_SIM_GAS_SYSTEM_H

#include "constants.h"
#include "units.h"

#include <cfloat>
#include <cmath>

class GasSystem {
    public:
        struct Mix {
            double p_fuel = 0.0;
            double p_inert = 1.0;
            double p_o2 = 0.0;
        };

        struct State {
            double n_mol = 0.0;
            double E_k = 0.0;
            double V = 0.0;
            double momentum[2] = { 0.0, 0.0 };

            Mix mix;
        };

        struct FlowParameters {
            double k_flow;
            double dt;
            double direction_x, direction_y;
            double crossSectionArea_0, crossSectionArea_1;
            GasSystem *system_0, *system_1;
        };

    public:
        GasSystem() { /* void */ }
        ~GasSystem() { /* void */ }

        void setGeometry(double width, double height, double dx, double dy);
        void initialize(double P, double V, double T, const Mix &mix = {}, int degreesOfFreedom = 5);
        void reset(double P, double T, const Mix &mix = {});

        void setVolume(double V);
        void setN(double n);

        void changeVolume(double dV);
        void changePressure(double pressure);
        void changeTemperature(double dT);
        void changeTemperature(double dT, double n);
        void changeEnergy(double dE);
        void changeMix(const Mix &mix);
        void injectFuel(double n);

        double react(double n, const Mix &mix);

        // Rich mixture: once the O2 is used up the remaining fuel is not left
        // as vapour, the O2 is shared out and every fuel molecule ends up as
        // CO / CO2 and H2 / H2O. Converts n mol of fuel into products (inert),
        // internal energy unchanged.
        void convertFuel(double n, double productsPerFuel);
        static double flowConstant(double flowRate, double P, double pressureDrop, double T, double hcr);
        static double k_28inH2O(double flowRateScfm);
        static double k_carb(double flowRateScfm);
        static double flowRate(
            double k_flow,
            double P0,
            double P1,
            double T0,
            double T1,
            double hcr,
            double chokedFlowLimit,
            double chokedFlowRateCached);
        double loseN(double dn, double E_k_per_mol);
        double gainN(double dn, double E_k_per_mol, const Mix &mix = {});
        void dissipateExcessVelocity();

        void updateVelocity(double dt, double beta = 1.0);
        void dissipateVelocity(double dt, double timeConstant);

        static double flow(const FlowParameters &params);
        double flow(double k_flow, double dt, double P_env, double T_env, const Mix &mix = {});

        double pressureEquilibriumMaxFlow(const GasSystem *b) const;
        double pressureEquilibriumMaxFlow(double P_env, double T_env, const Mix &mix = {}) const;

        inline static constexpr double kineticEnergyPerMol(double T, int degreesOfFreedom);
        inline static constexpr double heatCapacityRatio(int degreesOfFreedom);
        inline static double chokedFlowLimit(int degreesOfFreedom);
        inline static double chokedFlowRate(int degreesOfFreedom);

        inline double approximateDensity() const;
        inline int degreesOfFreedom() const { return m_degreesOfFreedom; }
        inline double n() const;
        inline double n(double V) const;
        inline double kineticEnergy() const;
        inline double kineticEnergy(double n) const;
        inline double kineticEnergyPerMol() const { return kineticEnergy(1.0); }
        inline double totalEnergy() const;
        inline double bulkKineticEnergy() const;
        inline double c() const;
        inline double dynamicPressure(double dx, double dy) const;
        inline double mass() const;
        inline double pressure() const;
        inline double temperature() const;
        inline double velocity_x() const;
        inline double velocity_y() const;
        inline double volume() const;
        inline double volume(double n) const;
        inline double n_fuel() const;
        inline double n_inert() const;
        inline double n_o2() const;
        inline double heatCapacityRatio() const;
        inline Mix mix() const { return m_state.mix; }

        // Heat capacity model: 0 = constant, set by the degrees of freedom
        // (original); 1 = mean heat capacity varying with temperature and
        // composition (air, fuel vapour, burned gas). Pressure and temperature
        // follow the model; the flow equations keep the constant ratio of
        // specific heats of the degrees of freedom in both cases.
        void setHeatCapacityModel(int model);
        inline int heatCapacityModel() const { return m_heatCapacityModel; }

        // Internal energy per mol at temperature T (from 0 K), and its inverse
        static double energyPerMol(double T, const Mix &mix, int model, int degreesOfFreedom);
        static double temperatureFromEnergy(double energyPerMol, const Mix &mix, int model, int degreesOfFreedom);

        // Model 1: instantaneous ratio of specific heats c_p / c_v
        static double variableHeatCapacityRatio(double T, const Mix &mix);

        // Energy carried per mol of gas leaving a volume: model 0 the
        // internal energy (original), model 1 the enthalpy u + R T, i.e.
        // with the flow work: the gas left behind expands and cools (exhaust
        // blowdown), the gas entering a volume heats it (intake charge)
        inline double flowEnergyPerMol() const;
        static double flowEnergyPerMol(double T, const Mix &mix, int model, int degreesOfFreedom);

        // E = 0.5 f n R T
        inline double effectiveDegreesOfFreedom() const;

    protected:
        // Model 1: mean degrees of freedom f = a + b T (2 c_v,mean / R)
        static void meanHeatCapacity(const Mix &mix, double *a, double *b);

        State m_state;

        int m_degreesOfFreedom = 5;
        int m_heatCapacityModel = 0;

        double m_chokedFlowLimit = 0;
        double m_chokedFlowFactorCached = 0;

        double m_width = 0.0;
        double m_height = 0.0;
        double m_dx = 0.0;
        double m_dy = 0.0;
};

inline constexpr double GasSystem::kineticEnergyPerMol(double T, int degreesOfFreedom) {
    return 0.5 * T * constants::R * degreesOfFreedom;
}

inline constexpr double GasSystem::heatCapacityRatio(int degreesOfFreedom) {
    return 1.0 + (2.0 / degreesOfFreedom);
}

inline double GasSystem::chokedFlowLimit(int degreesOfFreedom) {
    const double hcr = heatCapacityRatio(degreesOfFreedom);
    return std::pow((2.0 / (hcr + 1)), hcr / (hcr - 1));
}

inline double GasSystem::chokedFlowRate(int degreesOfFreedom) {
    const double hcr = heatCapacityRatio(degreesOfFreedom);
    double flowRate =
        std::sqrt(hcr) * std::pow(2 / (hcr + 1), (hcr + 1) / (2 * (hcr - 1)));

    return flowRate;
}

inline double GasSystem::approximateDensity() const {
    return (units::AirMolecularMass * n()) / volume();
}

inline double GasSystem::n() const {
    return m_state.n_mol;
}

inline double GasSystem::n(double V) const {
    return (V / volume()) * n();
}

inline double GasSystem::kineticEnergy() const {
    return m_state.E_k;
}

inline double GasSystem::kineticEnergy(double n) const {
    return (kineticEnergy() / this->n()) * n;
}

inline double GasSystem::c() const {
    if (n() == 0 || kineticEnergy() == 0) return 0;

    const double hcr = heatCapacityRatio();
    const double staticPressure = pressure();
    const double density = approximateDensity();
    const double c = std::sqrt(staticPressure * hcr / density);

    return c;
}

inline double GasSystem::totalEnergy() const {
    if (n() == 0) return 0;

    const double invMass = 1 / mass();
    const double v_x = m_state.momentum[0] * invMass;
    const double v_y = m_state.momentum[1] * invMass;
    const double v_squared = v_x * v_x + v_y * v_y;

    return kineticEnergy() + 0.5 * mass() * v_squared;
}

inline double GasSystem::bulkKineticEnergy() const {
    const double m = mass();
    if (m == 0) return 0;

    const double v_x = m_state.momentum[0] / m;
    const double v_y = m_state.momentum[1] / m;
    const double v_squared = v_x * v_x + v_y * v_y;
    return 0.5 * m * v_squared;
}

inline double GasSystem::dynamicPressure(double dx, double dy) const {
    if (n() == 0 || kineticEnergy() == 0) return 0;

    const double inverseMass = 1 / this->mass();
    const double v = inverseMass * (dx * m_state.momentum[0] + dy * m_state.momentum[1]);

    if (v <= 0) {
        return 0;
    }

    const double hcr = heatCapacityRatio();
    const double staticPressure = pressure();
    const double density = approximateDensity();
    const double c_squared = staticPressure * hcr / density;
    const double machNumber_squared = v * v / c_squared;

    // Below is equivalent to:
    // staticPressure * pow(1 + ((hcr - 1) / 2) * machNumber * machNumber, hcr / (hcr - 1)) - 1)

    const double x = 1 + ((hcr - 1) / 2) * machNumber_squared;
    double x_d;
    switch (m_degreesOfFreedom) {
    case 3:
        x_d = x * x * x * x * x;
        break;
    case 5:
    {
        const double x_2 = x * x;
        const double x_3 = x_2 * x;
        x_d = x_3 * x_3 * x;
        break;
    }
    default:
        x_d = x;
    }

    return staticPressure * (std::sqrt(x_d) - 1);
}

inline double GasSystem::mass() const {
    return units::AirMolecularMass * n();
}

inline double GasSystem::pressure() const {
    const double volume = this->volume();
    if (volume == 0) return 0;
    else if (m_heatCapacityModel == 0) return kineticEnergy() / (0.5 * m_degreesOfFreedom * volume);
    else return n() * constants::R * temperature() / volume;
}

inline double GasSystem::temperature() const {
    if (n() == 0) return 0;
    else if (m_heatCapacityModel == 0) return kineticEnergy() / (0.5 * m_degreesOfFreedom * n() * constants::R);
    else return temperatureFromEnergy(kineticEnergy() / n(), m_state.mix, m_heatCapacityModel, m_degreesOfFreedom);
}

inline double GasSystem::flowEnergyPerMol() const {
    return (m_heatCapacityModel == 0)
        ? kineticEnergyPerMol()
        : kineticEnergyPerMol() + constants::R * temperature();
}

inline double GasSystem::effectiveDegreesOfFreedom() const {
    if (m_heatCapacityModel == 0 || n() == 0) return m_degreesOfFreedom;

    const double T = temperature();
    return (T > 0)
        ? kineticEnergy() / (0.5 * n() * constants::R * T)
        : m_degreesOfFreedom;
}

inline double GasSystem::velocity_x() const {
    if (n() == 0) return 0;
    else return m_state.momentum[0] / mass();
}

inline double GasSystem::velocity_y() const {
    if (n() == 0) return 0;
    else return m_state.momentum[1] / mass();
}

inline double GasSystem::volume() const {
    return m_state.V;
}

inline double GasSystem::volume(double n) const {
    return n * this->n() / volume();
}

inline double GasSystem::n_fuel() const {
    return m_state.mix.p_fuel * n();
}

inline double GasSystem::n_inert() const {
    return m_state.mix.p_inert * n();
}

inline double GasSystem::n_o2() const {
    return m_state.mix.p_o2 * n();
}

inline double GasSystem::heatCapacityRatio() const {
    return heatCapacityRatio(m_degreesOfFreedom);
}

#endif /* ATG_ENGINE_SIM_GAS_SYSTEM_H */
