#include "../include/fuel.h"

#include "../include/units.h"

#include <cmath>

Fuel::Fuel() {
    m_molecularMass = 0.0;
    m_energyDensity = 0.0;
    m_density = 0.0;
    m_turbulenceToFlameSpeedRatio = nullptr;
    m_molecularAfr = 0.0;
    m_maxBurningEfficiency = 0.0;
    m_maxDilutionEffect = 0.0;
    m_maxTurbulenceEffect = 0.0;
    m_burningEfficiencyRandomness = 0.0;
    m_lowEfficiencyAttenuation = 0.0;
    m_latentHeat = 0.0;
}

Fuel::~Fuel() {
    /* void */
}

void Fuel::initialize(const Parameters &params) {
    m_molecularMass = params.molecularMass;
    m_energyDensity = params.energyDensity;
    m_density = params.density;
    m_turbulenceToFlameSpeedRatio = params.turbulenceToFlameSpeedRatio;
    m_molecularAfr = params.molecularAfr;
    m_burningEfficiencyRandomness = params.burningEfficiencyRandomness;
    m_maxBurningEfficiency = params.maxBurningEfficiency;
    m_maxDilutionEffect = params.maxDilutionEffect;
    m_maxTurbulenceEffect = params.maxTurbulenceEffect;
    m_lowEfficiencyAttenuation = params.lowEfficiencyAttenuation;
    m_latentHeat = params.latentHeat;
}

double Fuel::flameSpeed(
    double turbulence,
    double molecularAfr,
    double T,
    double P,
    double firingPressure,
    double motoringPressure) const
{
    const double S_L = laminarBurningVelocity(molecularAfr, T, P);
    const double p_adjustment = 1.0;

    return m_turbulenceToFlameSpeedRatio->sampleTriangle((turbulence / S_L) * p_adjustment) * S_L;
}

double Fuel::laminarBurningVelocity(double molecularAfr, double T, double P) const {
    // Assuming fuel is gasoline
    constexpr double er_m = 1.21;
    constexpr double B_m = units::distance(30.5, units::cm) / units::sec;
    constexpr double B_er = -units::distance(54.9, units::cm) / units::sec;
    const double er = molecularAfr / m_molecularAfr;
    const double alpha = 2.4 - 0.271 * std::pow(er, 3.51);
    const double beta = -0.357 + 0.14 * std::pow(er, 2.77);

    const double S_L_0 = B_m + B_er * (er - er_m) * (er - er_m);
    const double T_ratio = T / units::kelvin(298);
    const double P_ratio = P / units::pressure(1.0, units::atm);

    return S_L_0 * std::pow(T_ratio, alpha) * std::pow(P_ratio, beta);
}

double Fuel::turbulentFlameSpeed(
    double turbulence,
    double phi,
    double T_unburned,
    double P,
    double burnedFraction) const
{
    const double S_L = laminarBurningVelocity(phi, T_unburned, P, burnedFraction);
    return m_turbulenceToFlameSpeedRatio->sampleTriangle(turbulence / S_L) * S_L;
}

double Fuel::relativeLaminarSpeed(double phi, double burnedFraction) {
    // Mixture terms of laminarBurningVelocity(phi, ...) over their value at
    // phi 1.1 without residual gas
    constexpr double phi_m = 1.21;
    constexpr double B_m = 30.5, B_phi = -54.9;
    const double reference = B_m + B_phi * (1.1 - phi_m) * (1.1 - phi_m);
    const double S_L_0 = B_m + B_phi * (phi - phi_m) * (phi - phi_m);
    const double dilution = 1.0 - 2.06 * std::pow(std::fmax(burnedFraction, 0.0), 0.77);

    return std::fmax(S_L_0 * dilution / reference, 0.0);
}

double Fuel::laminarBurningVelocity(
    double phi,
    double T_unburned,
    double P,
    double burnedFraction) const
{
    // Metghalchi & Keck (gasoline / indolene) with the temperature and pressure
    // exponents and the residual dilution term from Heywood, eq. 9.37 - 9.38.
    // Unlike laminarBurningVelocity(afr, ...) the correlation gets the
    // equivalence ratio it was fitted with. The exponents are fitted for
    // phi 0.8 .. 1.5: outside they are held at the end of the range (beyond
    // ~1.7 the pressure exponent turns positive, a rich flame would get faster
    // with pressure), and the pressure exponent is kept <= 0
    constexpr double phi_m = 1.21;
    constexpr double B_m = units::distance(30.5, units::cm) / units::sec;
    constexpr double B_phi = -units::distance(54.9, units::cm) / units::sec;
    constexpr double S_L_min = units::distance(0.1, units::cm) / units::sec;

    const double phi_fit = std::fmin(std::fmax(phi, 0.8), 1.5);
    const double alpha = 2.4 - 0.271 * std::pow(phi_fit, 3.51);
    const double beta = std::fmin(-0.357 + 0.14 * std::pow(phi_fit, 2.77), 0.0);

    const double S_L_0 = B_m + B_phi * (phi - phi_m) * (phi - phi_m);
    const double T_ratio = T_unburned / units::kelvin(298);
    const double P_ratio = P / units::pressure(1.0, units::atm);
    const double dilution = 1.0 - 2.06 * std::pow(std::fmax(burnedFraction, 0.0), 0.77);

    return std::fmax(
        S_L_0 * std::pow(T_ratio, alpha) * std::pow(P_ratio, beta) * dilution,
        S_L_min);
}
