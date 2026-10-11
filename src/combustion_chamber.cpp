#include "../include/combustion_chamber.h"

#include "../include/constants.h"
#include "../include/units.h"
#include "../include/piston.h"
#include "../include/connecting_rod.h"
#include "../include/utilities.h"
#include "../include/exhaust_system.h"
#include "../include/cylinder_bank.h"
#include "../include/engine.h"

#include <cmath>

namespace {
    // Port wall film (x-tau): the evaporation rate relative to the reference
    // follows the fuel's vapour pressure (Clausius-Clapeyron with the heat of
    // vaporisation of iso-octane, 35 kJ/mol / R = 4200 K). The time constant
    // scales with 1 / rate and with the port pressure (diffusion into the
    // air), the deposited fraction with rate^-0.3: from 90 to 20 C tau grows
    // x16 and x x2.3, as identified on real engines
    constexpr double FilmReferenceTemperature = units::celcius(90.0);
    constexpr double FilmReferencePressure = units::pressure(100.0, units::kPa);
    constexpr double FilmVaporisationTemperature = 4200.0 * units::K;
    constexpr double FilmFractionExponent = 0.3;
    constexpr double MaxFilmFraction = 0.9;
    constexpr double MinFilmTimeConstant = 0.005;

    // Flame model 2, ignition and cycle-to-cycle variation: the relative
    // laminar speed (Fuel::relativeLaminarSpeed) of the mixture at the plug
    // decides whether the kernel survives. Below this the spark fails: with
    // 6-10 % residual gas the misfire limits fall at lambda ~0.57-0.59 rich
    // and ~1.4-1.5 lean, those of port injection SI engines (Heywood)
    constexpr double MinIgnitionSpeed = 0.37;
    // ...and the turbulence of slower mixtures varies more from cycle to
    // cycle, relative to stoichiometric with 6 % residual gas
    constexpr double ReferenceSpeed = 0.72;
    constexpr double MaxVariationGain = 3.0;

    double normalSample() {
        // Box-Muller
        const double u1 = (rand() + 1.0) / (RAND_MAX + 2.0);
        const double u2 = (rand() + 1.0) / (RAND_MAX + 2.0);
        return std::sqrt(-2.0 * std::log(u1)) * std::cos(2 * constants::pi * u2);
    }

    // Rich mixture: the O2 oxidises all the fuel partially (CO, H2) instead
    // of part of it completely. Heat released relative to complete combustion
    // of the O2-limited fuel, from the heats of formation per CH2 unit:
    // 90 kJ to CO + H2, then ~525 kJ per mol of O2 for CO -> CO2 / H2 -> H2O.
    double partialOxidation(double phi) {
        return (phi <= 1.0) ? 1.0 : std::fmax(1.2805 - 0.2805 * phi, 0.5);
    }

    // The mixture is not uniform: the local equivalence ratio is normally
    // distributed around phi. Lean pockets leave O2, rich pockets leave fuel
    // partially oxidised, so the heat released peaks slightly rich of
    // stoichiometric. Relative to complete combustion of the O2-limited fuel.
    double combustionHeatFactor(double phi, double nonuniformity) {
        if (phi <= 0) return 1.0;
        if (nonuniformity <= 0) return partialOxidation(phi);

        constexpr int Points = 31;
        double sum = 0, weights = 0;
        for (int i = 0; i < Points; ++i) {
            const double z = -3.0 + 6.0 * i / (Points - 1);
            const double w = std::exp(-0.5 * z * z);
            const double phi_local = std::fmax(phi * (1 + nonuniformity * z), 0.0);
            sum += w * ((phi_local <= 1.0) ? phi_local : partialOxidation(phi_local));
            weights += w;
        }

        return (sum / weights) / std::fmin(phi, 1.0);
    }
}

CombustionChamber::CombustionChamber() {
    m_crankcasePressure = 0.0;
    m_wallHeatTransfer = 0.0;
    m_wallTemperature = units::celcius(90.0);
    m_flameModel = 0;
    m_gasModel = 0;
    m_mixtureNonuniformity = 0.0;
    m_sparkPlugOffset = 0.0;
    m_cycleVariation = 0.0;
    m_wallFilmFraction = 0.0;
    m_wallFilmTimeConstant = 0.3;
    m_coolantTemperature = units::celcius(90.0);
    m_chargeValid = false;
    m_intakeWasOpen = false;
    m_meanPistonSpeed = 0.0;
    m_piston = nullptr;
    m_head = nullptr;
    m_engine = nullptr;
    m_pistonSpeed = nullptr;
    m_pressure = nullptr;
    m_lit = false;
    m_litLastFrame = false;
    m_peakTemperature = 0;

    m_meanPistonSpeedToTurbulence = nullptr;
    m_nBurntFuel = 0;

    m_manifoldToRunnerFlowRate = 0;
    m_primaryToCollectorFlowRate = 0;
    m_cylinderWidthApproximation = 0;
    m_cylinderCrossSectionSurfaceArea = 0;

    m_lastTimestepTotalExhaustFlow = 0;
    m_lastTimestepTotalIntakeFlow = 0;
    m_exhaustFlow = 0;
    m_exhaustFlowRate = 0;
    m_intakeFlowRate = 0;
    m_ecuFuelActive = false;
    m_ecuPendingFuel = 0;
    m_wallFilm = 0;

    m_fuel = nullptr;
}

CombustionChamber::~CombustionChamber() {
    assert(m_pistonSpeed == nullptr);
    assert(m_pressure == nullptr);
}

void CombustionChamber::initialize(const Parameters &params) {
    m_piston = params.Piston;
    m_head = params.Head;
    m_fuel = params.Fuel;
    m_crankcasePressure = params.CrankcasePressure;
    m_wallHeatTransfer = params.WallHeatTransfer;
    m_wallTemperature = params.WallTemperature;
    m_flameModel = params.FlameModel;
    m_gasModel = params.GasModel;
    m_mixtureNonuniformity = params.MixtureNonuniformity;
    m_sparkPlugOffset = params.SparkPlugOffset;
    m_cycleVariation = params.CycleVariation;
    m_wallFilmFraction = params.WallFilmFraction;
    m_wallFilmTimeConstant = params.WallFilmTimeConstant;
    m_coolantTemperature = params.CoolantTemperature;
    m_meanPistonSpeedToTurbulence = params.MeanPistonSpeedToTurbulence;

    m_pistonSpeed = new double[StateSamples];
    m_pressure = new double[StateSamples];
    for (int i = 0; i < StateSamples; ++i) {
        m_pistonSpeed[i] = 0;
        m_pressure[i] = 0;
    }

    Intake *intake = m_head->getIntake(m_piston->getCylinderIndex());
    ExhaustSystem *exhaust = m_head->getExhaustSystem(m_piston->getCylinderIndex());

    m_manifoldToRunnerFlowRate = intake->getRunnerFlowRate();
    m_primaryToCollectorFlowRate = exhaust->getPrimaryFlowRate();

    const double bore_r = m_head->getCylinderBank()->getBore() / 2.0;
    m_cylinderCrossSectionSurfaceArea = constants::pi * bore_r * bore_r;
    if (m_flameModel == 2) {
        m_flameGeometry.initialize(bore_r, m_sparkPlugOffset);
    }
    m_cylinderWidthApproximation = std::sqrt(m_cylinderCrossSectionSurfaceArea);

    const double height = getVolume() / m_cylinderCrossSectionSurfaceArea;
    m_system.setGeometry(
        m_cylinderWidthApproximation,
        height,
        1.0,
        0.0);

    const double intakeRunnerCrossSection = m_head->getIntakeRunnerCrossSectionArea();
    const double intakeRunnerWidth = std::sqrt(intakeRunnerCrossSection);
    const double manifoldRunnerLength = intake->getRunnerLength();
    const double manifoldRunnerVolume = intakeRunnerCrossSection * manifoldRunnerLength;
    const double totalIntakeRunnerVolume = m_head->getIntakeRunnerVolume() + manifoldRunnerVolume;
    const double overallIntakeRunnerLength = totalIntakeRunnerVolume / intakeRunnerCrossSection;
    m_intakeRunnerAndManifold.initialize(
        units::pressure(1.0, units::atm),
        totalIntakeRunnerVolume,
        intake->getAirTemperature());
    m_intakeRunnerAndManifold.setGeometry(
        overallIntakeRunnerLength,
        intakeRunnerWidth,
        1.0,
        0.0);

    const double exhaustRunnerCrossSection = m_head->getExhaustRunnerCrossSectionArea();
    const double exhaustRunnerWidth = std::sqrt(exhaustRunnerCrossSection);
    const double exhaustTubeLength =
        exhaust->getPrimaryTubeLength() + m_head->getHeaderPrimaryLength(m_piston->getCylinderIndex());
    const double exhaustTubeVolume = exhaustRunnerCrossSection * exhaustTubeLength;
    const double totalExhaustRunnerVolume = m_head->getExhaustRunnerVolume() + exhaustTubeVolume;
    const double overallExhaustRunnerLength = totalExhaustRunnerVolume / exhaustRunnerCrossSection;
    m_exhaustRunnerAndPrimary.initialize(
        units::pressure(1.0, units::atm),
        totalExhaustRunnerVolume,
        units::celcius(25.0));
    m_exhaustRunnerAndPrimary.setGeometry(
        overallExhaustRunnerLength,
        exhaustRunnerWidth,
        1.0,
        0.0);

    // m_system is (re)initialised by the simulator and keeps the model
    m_system.setHeatCapacityModel(m_gasModel);
    m_intakeRunnerAndManifold.setHeatCapacityModel(m_gasModel);
    m_exhaustRunnerAndPrimary.setHeatCapacityModel(m_gasModel);
}

void CombustionChamber::destroy() {
    if (m_pistonSpeed != nullptr) delete[] m_pistonSpeed;
    if (m_pressure != nullptr) delete[] m_pressure;

    m_pistonSpeed = nullptr;
    m_pressure = nullptr;
}

double CombustionChamber::getVolume() const {
    const double combustionPortVolume = m_head->getCombustionChamberVolume();
    const CylinderBank *bank = m_head->getCylinderBank();

    const double area = bank->boreSurfaceArea();
    const double s =
        m_piston->relativeX() * bank->getDx()
        + m_piston->relativeY() * bank->getDy();
    const double sweep =
        area * (bank->getDeckHeight() - s - m_piston->getCompressionHeight());

    return sweep + combustionPortVolume - m_piston->getDisplacement();
}

double CombustionChamber::pistonSpeed() const {
    const CylinderBank *bank = m_head->getCylinderBank();
    return
        m_piston->m_body.v_x * bank->getDx()
        + m_piston->m_body.v_y * bank->getDy();
}

double CombustionChamber::calculateMeanPistonSpeed() const {
    double avg = 0;
    for (int i = 0; i < StateSamples; ++i) {
        avg += m_pistonSpeed[i];
    }

    avg /= StateSamples;
    return avg;
}

double CombustionChamber::calculateFiringPressure() const {
    double firingPressure = 0;
    for (int i = 0; i < StateSamples; ++i) {
        if (m_pressure[i] > firingPressure) {
            firingPressure = m_pressure[i];
        }
    }

    return firingPressure;
}

bool CombustionChamber::popLitLastFrame() {
    const bool lit = m_litLastFrame;
    m_litLastFrame = false;

    return lit;
}

void CombustionChamber::injectFuel(double n) {
    if (!m_ecuFuelActive || n <= 0) return;

    const double x = wallFilmFraction();
    m_wallFilm += x * n;
    m_ecuPendingFuel += (1 - x) * n;
}

double CombustionChamber::wallFilmFraction() const {
    if (m_wallFilmFraction <= 0) return 0;

    const double rate = std::exp(FilmVaporisationTemperature
        * (1 / FilmReferenceTemperature - 1 / m_coolantTemperature));
    return std::fmin(m_wallFilmFraction * std::pow(rate, -FilmFractionExponent), MaxFilmFraction);
}

double CombustionChamber::wallFilmTimeConstant() const {
    const double rate = std::exp(FilmVaporisationTemperature
        * (1 / FilmReferenceTemperature - 1 / m_coolantTemperature));
    const double portPressure =
        m_head->getIntake(m_piston->getCylinderIndex())->m_system.pressure();
    return std::fmax(
        m_wallFilmTimeConstant * (portPressure / FilmReferencePressure) / rate,
        MinFilmTimeConstant);
}

void CombustionChamber::ignite() {
    if (!m_lit) {
        if (m_system.mix().p_fuel == 0) return;

        const double afr = m_system.mix().p_o2 / m_system.mix().p_fuel;
        const double equivalenceRatio = afr / m_fuel->getMolecularAfr();
        if (equivalenceRatio < 0.5) return;
        else if (equivalenceRatio > 1.9) return;

        // Flame model 2: the kernel forms in the mixture at the plug, whose
        // equivalence ratio varies from cycle to cycle (mixture
        // non-uniformity). Too slow a mixture there (rich, lean or diluted
        // by residual gas): misfire, the charge leaves unburned
        double relativeSpeed = 1.0;
        if (m_flameModel == 2) {
            const GasSystem::Mix &mix = m_system.mix();
            const double phi = m_fuel->getMolecularAfr() / afr;
            const double residual = clamp(1.0 - mix.p_fuel - 4.0 * mix.p_o2);
            const double phiPlug = phi * std::fmax(1.0 + m_mixtureNonuniformity * normalSample(), 0.1);
            if (Fuel::relativeLaminarSpeed(phiPlug, residual) < MinIgnitionSpeed) return;

            relativeSpeed = Fuel::relativeLaminarSpeed(phi, residual);
        }

        const double idealInert = m_system.mix().p_o2 / 0.7;
        const double dilution = (m_system.mix().p_inert / idealInert) - 1;

        m_flameEvent.lastVolume = getVolume();
        m_flameEvent.travel_x = 0;
        m_flameEvent.travel_y = 0;
        m_flameEvent.lit_n = 0;
        m_flameEvent.total_n = m_system.n();
        m_flameEvent.percentageLit = 0;
        m_flameEvent.globalMix = m_system.mix();
        m_lit = true;
        m_litLastFrame = true;

        const double randomness =
            m_fuel->getBurningEfficiencyRandomness();
        const double lowEfficiencyAttenuation =
            m_fuel->getLowEfficiencyAttenuation();
        const double maxBurningEfficiency =
            m_fuel->getMaxBurningEfficiency();
        const double maxTurbulenceEffect =
            m_fuel->getMaxTurbulenceEffect();
        const double maxDilutionEffect =
            m_fuel->getMaxDilutionEffect();

        const double turbulence =
            m_meanPistonSpeedToTurbulence->sampleTriangle(
                calculateMeanPistonSpeed());
        const double mixingFactor =
            1.0 - (
                clamp(turbulence / maxTurbulenceEffect)
                * clamp(1 - dilution / maxDilutionEffect));
        const double rand_s =
            lowEfficiencyAttenuation
            * ((1 - randomness) + randomness * ((double)rand() / RAND_MAX));
        const double efficiencyAttenuation =
            (mixingFactor * rand_s + (1 - mixingFactor));
        m_flameEvent.efficiency =
            efficiencyAttenuation * maxBurningEfficiency;
        m_flameEvent.heatFactor = (m_gasModel == 1)
            ? combustionHeatFactor(m_fuel->getMolecularAfr() / afr, m_mixtureNonuniformity)
            : 1.0;

        if (m_flameModel == 1 || m_flameModel == 2) {
            // Fresh air carries 3 mol of inert gas per mol of O2 and the
            // combustion products are inert, so whatever is neither fuel nor
            // fresh air is burned gas (residual)
            const GasSystem::Mix &mix = m_system.mix();
            m_flameEvent.turbulence = turbulence;
            m_flameEvent.phi = m_fuel->getMolecularAfr() / afr;
            m_flameEvent.residual = clamp(1.0 - mix.p_fuel - 4.0 * mix.p_o2);
            m_flameEvent.sparkPressure = m_system.pressure();
            m_flameEvent.sparkTemperature = m_system.temperature();
            m_flameEvent.unburnedTemperature = m_flameEvent.sparkTemperature;
            m_flameEvent.lastPressure = m_flameEvent.sparkPressure;
            m_flameEvent.entrained = 0;
            m_flameEvent.burned = 0;
            m_flameEvent.radius = 0;

            // Per mol of charge: fuel that can burn (O2 limited), the heat it
            // releases at constant volume, the moles it adds and the burned
            // gas composition, as in GasSystem::react()
            // (gas model 1: in a rich mixture all the fuel ends up as products)
            const double molecularAfr = m_fuel->getMolecularAfr();
            const double efficiency = m_flameEvent.efficiency;
            const double burnableFuel = efficiency * std::fmin(mix.p_fuel, mix.p_o2 / molecularAfr);
            const double fuelReacted = (m_gasModel == 1) ? efficiency * mix.p_fuel : burnableFuel;
            const double o2Reacted = burnableFuel * molecularAfr;
            const double products = (34.0 / 27.0) * (1 + molecularAfr) * fuelReacted;
            m_flameEvent.heatPerMol = burnableFuel * m_fuel->getMolecularMass()
                * m_fuel->getEnergyDensity() * m_flameEvent.heatFactor;
            m_flameEvent.moleGain = 1.0 + products - fuelReacted - o2Reacted;

            GasSystem::Mix &burnedMix = m_flameEvent.burnedMix;
            burnedMix.p_fuel = (mix.p_fuel - fuelReacted) / m_flameEvent.moleGain;
            burnedMix.p_o2 = (mix.p_o2 - o2Reacted) / m_flameEvent.moleGain;
            burnedMix.p_inert = 1.0 - burnedMix.p_fuel - burnedMix.p_o2;

            if (m_flameModel == 2 && m_cycleVariation > 0) {
                // Cycle-to-cycle variation: the flow field at the plug when
                // the kernel forms; slow mixtures depend more on it
                const double gain = clamp(
                    ReferenceSpeed / std::fmax(relativeSpeed, 1E-3), 1.0, MaxVariationGain);
                m_flameEvent.turbulence *=
                    std::fmax(1.0 + gain * m_cycleVariation * normalSample(), 0.2);
            }

            m_flameEvent.flameSpeed = unburnedFlameSpeed();
        }
        else {
            m_flameEvent.flameSpeed = m_fuel->flameSpeed(
                turbulence,
                afr,
                m_system.temperature(),
                m_system.pressure(),
                calculateFiringPressure(),
                units::pressure(160, units::psi));
        }
    }
}

void CombustionChamber::convertRichFuel(double n_reacting, double fuelBurned) {
    // Gas model 1: the fuel of the burning parcel that found no O2 ends up as
    // CO and H2 (heat accounted by heatFactor), not as hot fuel vapour
    if (m_gasModel != 1) return;

    const double leftover = n_reacting * m_flameEvent.globalMix.p_fuel - fuelBurned;
    if (leftover > 0) {
        m_system.convertFuel(leftover, (34.0 / 27.0) * (1 + m_fuel->getMolecularAfr()));
    }
}

void CombustionChamber::updateUnburnedTemperature() {
    // The unburned gas ahead of the flame is compressed (isentropically) by
    // the piston and by the expanding burned gas since the spark
    const double p = m_system.pressure();
    double &T_u = m_flameEvent.unburnedTemperature;
    if (m_flameEvent.lastPressure > 0 && p > 0 && T_u > 0) {
        const double g = (m_system.heatCapacityModel() == 0)
            ? m_system.heatCapacityRatio()
            : GasSystem::variableHeatCapacityRatio(T_u, m_flameEvent.globalMix);
        T_u *= std::pow(p / m_flameEvent.lastPressure, (g - 1) / g);
    }

    m_flameEvent.lastPressure = p;
}

double CombustionChamber::unburnedFlameSpeed() const {
    return m_fuel->turbulentFlameSpeed(
        m_flameEvent.turbulence,
        m_flameEvent.phi,
        unburnedTemperature(),
        m_system.pressure(),
        m_flameEvent.residual);
}

void CombustionChamber::updateSphericalFlame(double dt, double volume) {
    // Quasi-dimensional entrainment / burn-up model (Blizard & Keck,
    // Tabaczynski): the turbulent flame front entrains unburned mixture at
    // u' + S_L, the entrained eddies then burn at the laminar speed over the
    // Taylor microscale. The flame is a sphere around the plug, clipped by the
    // head, the liner and the piston.
    constexpr double KernelRadius = units::distance(1.0, units::mm);

    const double h = volume / m_cylinderCrossSectionSurfaceArea;
    const double p = m_system.pressure();
    const double T_u = unburnedTemperature();
    const double n_charge = m_flameEvent.total_n;
    if (p <= 0 || T_u <= 0 || n_charge <= 0) {
        m_lit = false;
        return;
    }

    // A flame still burning when the exhaust valve opens is quenched by the
    // blowdown: the rest of the charge leaves unburned (partial burn)
    if (m_exhaustFlowRate > 0) {
        m_lit = false;
        return;
    }

    const double S_L = m_fuel->laminarBurningVelocity(
        m_flameEvent.phi, T_u, p, m_flameEvent.residual);
    const double unburnedDensity = p / (constants::R * T_u);    // [mol/m^3]

    // Volume behind the flame front: burned gas (expanded by the density
    // ratio) plus the entrained gas not burned yet. Burned gas temperature
    // from the energy balance at constant volume.
    const int model = m_system.heatCapacityModel();
    const int dof = m_system.degreesOfFreedom();
    const double u_unburned = GasSystem::energyPerMol(T_u, m_flameEvent.globalMix, model, dof);
    const double u_burned = (u_unburned + m_flameEvent.heatPerMol) / m_flameEvent.moleGain;
    const double T_b = GasSystem::temperatureFromEnergy(u_burned, m_flameEvent.burnedMix, model, dof);
    const double expansion = m_flameEvent.moleGain * T_b / T_u;
    const double f_e = m_flameEvent.entrained;
    const double f_b = m_flameEvent.burned;
    const double V_front =
        volume * (f_b * expansion + (f_e - f_b)) / (f_b * expansion + (1 - f_b));

    const double r = std::fmax(m_flameGeometry.radius(V_front, h), KernelRadius);
    const double A_front = m_flameGeometry.frontArea(r, h);
    m_flameEvent.radius = r;
    m_flameEvent.travel_x = std::fmin(r, m_head->getCylinderBank()->getBore() / 2);
    m_flameEvent.travel_y = std::fmin(r, h);

    // Turbulence scales: integral scale ~0.2 x chamber height, Taylor
    // microscale from the turbulent Reynolds number (air viscosity from
    // Heywood). The kernel only feels the eddies smaller than itself.
    const double L_I = 0.2 * h;
    const double viscosity = 3.3E-7 * std::pow(T_u, 0.7);
    const double density = unburnedDensity * units::AirMolecularMass;
    const double u = m_flameEvent.turbulence;
    const double Re = std::fmax(u * L_I * density / viscosity, 1.0);
    const double taylorScale = L_I * std::sqrt(15.0 / Re);
    const double u_effective = u * (1 - std::exp(-r / L_I));

    m_flameEvent.flameSpeed = u_effective + S_L;
    m_flameEvent.entrained = std::fmin(
        m_flameEvent.entrained
            + unburnedDensity * A_front * (u_effective + S_L) * dt / n_charge,
        1.0);

    // End gas: once the front has swept nearly the whole chamber, the last
    // pockets (corners, near the walls) are consumed in about the time the
    // flame needs to cross an integral scale at the laminar speed
    if (V_front > 0.98 * volume) {
        m_flameEvent.entrained +=
            (1 - m_flameEvent.entrained) * (1 - std::exp(-dt * S_L / L_I));
        if (1 - m_flameEvent.entrained < 1E-5) m_flameEvent.entrained = 1.0;
    }

    const double tau = taylorScale / S_L;
    const double burnFraction =
        (m_flameEvent.entrained - m_flameEvent.burned) * (1 - std::exp(-dt / tau));
    m_flameEvent.burned += burnFraction;
    m_flameEvent.percentageLit = m_flameEvent.burned;

    const double fuelBurned = m_system.react(
        burnFraction * n_charge * m_flameEvent.efficiency, m_flameEvent.globalMix);
    convertRichFuel(burnFraction * n_charge * m_flameEvent.efficiency, fuelBurned);
    const double massFuelBurned = fuelBurned * m_fuel->getMolecularMass();
    m_system.changeEnergy(massFuelBurned * m_fuel->getEnergyDensity() * m_flameEvent.heatFactor);
    m_nBurntFuel += massFuelBurned;

    if (m_flameEvent.entrained >= 1.0
        && m_flameEvent.entrained - m_flameEvent.burned < 1E-4)
    {
        m_lit = false;
    }

    m_flameEvent.lastVolume = volume;
}

void CombustionChamber::update(double dt) {
    m_system.setVolume(getVolume());

    updateCycleStates();

    m_intakeFlowRate = m_head->intakeFlowRate(m_piston->getCylinderIndex());
    m_exhaustFlowRate = m_head->exhaustFlowRate(m_piston->getCylinderIndex());

    // Mean piston speed = 2 * stroke * revolutions per second
    const Crankshaft *crankshaft = m_engine->getOutputCrankshaft();
    m_meanPistonSpeed =
        4 * crankshaft->getThrow() * std::abs(crankshaft->m_body.v_theta) / (2 * constants::pi);
}

void CombustionChamber::flow(double dt) {
    if (m_system.temperature() > m_peakTemperature) {
        m_peakTemperature = m_system.temperature();
    }

    const double volume = getVolume();
    const double cylinderHeight = volume / m_cylinderCrossSectionSurfaceArea;
    const double cylinderSurfaceArea =
        cylinderHeight * constants::pi * m_head->getCylinderBank()->getBore()
        + m_cylinderCrossSectionSurfaceArea * 2;

    const double T = m_system.temperature();
    const double dT = m_wallTemperature - T;

    // Heat transfer coefficient [W/(m^2 K)]
    double h = 100.0;
    if (m_wallHeatTransfer > 0 && T > 0 && volume > 0) {
        // Hohenberg (1979), V [m^3], p [bar], T [K], mean piston speed [m/s]
        const double p = units::convert(m_system.pressure(), units::bar);
        h = m_wallHeatTransfer * 130.0
            * std::pow(volume, -0.06)
            * std::pow(std::fmax(p, 0.0), 0.8)
            * std::pow(T, -0.4)
            * std::pow(m_meanPistonSpeed + 1.4, 0.8);
    }

    m_system.changeEnergy(dT * cylinderSurfaceArea * h * dt);
    m_system.flow(m_piston->getBlowbyK(), dt, m_crankcasePressure, units::celcius(25.0));

    Intake *intake = m_head->getIntake(m_piston->getCylinderIndex());
    ExhaustSystem *exhaust = m_head->getExhaustSystem(m_piston->getCylinderIndex());

    const double start_n = m_system.n();

    GasSystem::FlowParameters flowParams;
    flowParams.dt = dt;

    flowParams.k_flow = m_manifoldToRunnerFlowRate;
    flowParams.crossSectionArea_0 = intake->getPlenumCrossSectionArea();
    flowParams.crossSectionArea_1 = m_head->getIntakeRunnerCrossSectionArea();
    flowParams.direction_x = 1.0;
    flowParams.direction_y = 0.0;
    flowParams.system_0 = &intake->m_system;
    flowParams.system_1 = &m_intakeRunnerAndManifold;
    GasSystem::flow(flowParams);

    m_intakeRunnerAndManifold.dissipateExcessVelocity();

    // Wall film: the evaporated fuel joins the fuel waiting in the port
    if (m_wallFilm > 0) {
        const double evaporated = m_wallFilm * (1 - std::exp(-dt / wallFilmTimeConstant()));
        m_wallFilm -= evaporated;
        m_ecuPendingFuel += evaporated;
    }

    // ECU sequential injection: delivered into the cylinder with the intake
    // charge, after the exhaust valve has closed (during valve overlap it would
    // be scavenged straight into the exhaust)
    if (m_ecuFuelActive && m_intakeFlowRate > 0 && m_exhaustFlowRate <= 0 && m_ecuPendingFuel > 0) {
        GasSystem::Mix pureFuel;
        pureFuel.p_fuel = 1.0;
        pureFuel.p_inert = 0.0;
        pureFuel.p_o2 = 0.0;

        m_system.gainN(m_ecuPendingFuel, m_system.kineticEnergyPerMol(), pureFuel);

        // Evaporation cools the charge (capped: the fuel is delivered at once,
        // possibly into little residual gas)
        const double latentHeat =
            m_ecuPendingFuel * m_fuel->getMolecularMass() * m_fuel->getLatentHeat();
        if (latentHeat > 0) {
            m_system.changeEnergy(-std::fmin(latentHeat, 0.5 * m_system.kineticEnergy()));
        }
        intake->m_totalFuelInjected += m_ecuPendingFuel;
        m_ecuPendingFuel = 0;
    }

    flowParams.k_flow = m_intakeFlowRate;
    flowParams.crossSectionArea_0 = m_head->getIntakeRunnerCrossSectionArea();
    flowParams.crossSectionArea_1 = volume / cylinderHeight;
    flowParams.direction_x = 1.0;
    flowParams.direction_y = 0.0;
    flowParams.system_0 = &m_intakeRunnerAndManifold;
    flowParams.system_1 = &m_system;
    const double intakeFlow = GasSystem::flow(flowParams);

    m_intakeRunnerAndManifold.dissipateExcessVelocity();
    m_system.dissipateExcessVelocity();

    flowParams.k_flow = m_exhaustFlowRate;
    flowParams.crossSectionArea_0 = volume / cylinderHeight;
    flowParams.crossSectionArea_1 = m_head->getExhaustRunnerCrossSectionArea();
    flowParams.direction_x = 1.0;
    flowParams.direction_y = 0.0;
    flowParams.system_0 = &m_system;
    flowParams.system_1 = &m_exhaustRunnerAndPrimary;
    const double exhaustFlow = GasSystem::flow(flowParams);

    m_system.dissipateExcessVelocity();
    m_exhaustRunnerAndPrimary.dissipateExcessVelocity();

    flowParams.k_flow = m_primaryToCollectorFlowRate;
    flowParams.crossSectionArea_0 = m_head->getExhaustRunnerCrossSectionArea();
    flowParams.crossSectionArea_1 = exhaust->getCollectorCrossSectionArea();
    flowParams.direction_x = 1.0;
    flowParams.direction_y = 0.0;
    flowParams.system_0 = &m_exhaustRunnerAndPrimary;
    flowParams.system_1 = exhaust->getSystem();
    GasSystem::flow(flowParams);

    m_intakeRunnerAndManifold.updateVelocity(dt, intake->getVelocityDecay());
    m_system.updateVelocity(dt, 0.5);
    m_exhaustRunnerAndPrimary.updateVelocity(dt, exhaust->getVelocityDecay());

    if (std::abs(intakeFlow) > 1E-9 && m_lit) {
        m_lit = false;
    }

    // Intake valve closing: the charge of this cycle
    const bool intakeOpen = m_intakeFlowRate > 0;
    if (m_intakeWasOpen && !intakeOpen) {
        m_chargeMix = m_system.mix();
        m_chargeValid = true;
    }
    m_intakeWasOpen = intakeOpen;

    m_exhaustFlow = exhaustFlow;
    m_lastTimestepTotalExhaustFlow += exhaustFlow;
    m_lastTimestepTotalIntakeFlow += intakeFlow;

    if (m_lit && (m_flameModel == 1 || m_flameModel == 2)) {
        updateUnburnedTemperature();
    }

    if (m_lit && m_flameModel == 2) {
        updateSphericalFlame(dt, volume);
    }
    else if (m_lit) {
        CylinderBank *bank = m_head->getCylinderBank();
        const double totalTravel_x = bank->getBore() / 2;
        const double totalTravel_y = volume / bank->boreSurfaceArea();
        const double expansion = volume / m_flameEvent.lastVolume;
        if (m_flameModel == 1) {
            m_flameEvent.flameSpeed = unburnedFlameSpeed();
        }

        const double lastTravel_x = m_flameEvent.travel_x;
        const double lastTravel_y = m_flameEvent.travel_y * expansion;
        const double flameSpeed = m_flameEvent.flameSpeed;

        m_flameEvent.travel_x =
            std::fmin(lastTravel_x + dt * flameSpeed, totalTravel_x);
        m_flameEvent.travel_y =
            std::fmin(lastTravel_y + dt * flameSpeed, totalTravel_y);

        if (lastTravel_x < m_flameEvent.travel_x || lastTravel_y < m_flameEvent.travel_y) {
            const double burnedVolume =
                m_flameEvent.travel_x * m_flameEvent.travel_x
                * constants::pi * m_flameEvent.travel_y;
            const double prevBurnedVolume =
                lastTravel_x * lastTravel_x * constants::pi * lastTravel_y;
            const double litVolume = burnedVolume - prevBurnedVolume;
            const double n = (litVolume / volume) * m_system.n();

            const double fuelBurned =
                m_system.react(n * m_flameEvent.efficiency, m_flameEvent.globalMix);
            convertRichFuel(n * m_flameEvent.efficiency, fuelBurned);
            const double massFuelBurned = fuelBurned * m_fuel->getMolecularMass();
            m_system.changeEnergy(
                massFuelBurned * m_fuel->getEnergyDensity() * m_flameEvent.heatFactor);

            m_flameEvent.lit_n += n;
            m_flameEvent.percentageLit += litVolume / volume;

            m_nBurntFuel += massFuelBurned;
        }
        else {
            m_lit = false;
        }

        m_flameEvent.lastVolume = volume;
    }
}

double CombustionChamber::lastEventAfr() const {
    const double totalFuel = m_flameEvent.globalMix.p_fuel * m_flameEvent.total_n;
    const double totalOxygen = m_flameEvent.globalMix.p_o2 * m_flameEvent.total_n;
    const double totalInert = m_flameEvent.globalMix.p_inert * m_flameEvent.total_n;

    constexpr double octaneMolarMass = units::mass(114.23, units::g);
    constexpr double oxygenMolarMass = units::mass(31.9988, units::g);
    constexpr double nitrogenMolarMass = units::mass(28.014, units::g);

    if (totalFuel == 0) return 0;
    else {
        return
            (oxygenMolarMass * totalOxygen + totalInert * nitrogenMolarMass)
            / (totalFuel * octaneMolarMass);
    }
}

double CombustionChamber::calculateFrictionForce(double v_s) const {
    const double cylinderWallForce = m_piston->calculateCylinderWallForce();

    const double F_coul = m_frictionModel.frictionCoeff * cylinderWallForce;
    const double v_st = m_frictionModel.breakawayFrictionVelocity * constants::root_2;
    const double v_coul = m_frictionModel.breakawayFrictionVelocity / 10;
    const double F_brk = m_frictionModel.breakawayFriction;
    const double v = std::abs(v_s);

    const double F_0 = constants::root_2 * constants::e * (F_brk - F_coul);
    const double F_1 = v / v_st;
    const double F_2 = std::exp(-F_1 * F_1) * F_1;
    const double F_3 = F_coul * std::tanh(v / v_coul);
    const double F_4 = m_frictionModel.viscousFrictionCoefficient * v;

    return F_0 * F_2 + F_3 + F_4;
}

void CombustionChamber::updateCycleStates() {
    double crankAngle = m_engine->getOutputCrankshaft()->getCycleAngle();
    if (std::isnan(crankAngle) || std::isinf(crankAngle)) {
        crankAngle = 0.0;
    }

    const int i = (int)std::round((crankAngle / (4 * constants::pi)) * (StateSamples - 1.0));

    m_pistonSpeed[i] = std::abs(pistonSpeed());
    m_pressure[i] = m_system.pressure();
}

void CombustionChamber::apply(atg_scs::SystemState *system) {
    CylinderBank *bank = m_head->getCylinderBank();
    const double area = (bank->getBore() * bank->getBore() / 4.0) * constants::pi;
    const double v_x = system->v_x[m_piston->m_body.index];
    const double v_y = system->v_y[m_piston->m_body.index];

    const double v_s =
        v_x * bank->getDx() + v_y * bank->getDy();

    const double pressureDifferential = m_system.pressure() - m_crankcasePressure;
    const double force = -area * pressureDifferential;

    if (std::isnan(force) || std::isinf(force)) {
        assert(false);
    }

    constexpr double limit = 1E-3;
    const double abs_v_s = std::fmin(std::abs(v_s), limit);
    const double attenuation = abs_v_s / limit;

    const double F = calculateFrictionForce(v_s) * attenuation;
    const double F_fric = (v_s > 0)
        ? -F
        : F;

    system->applyForce(
        0.0,
        0.0,
        (force + F_fric) * bank->getDx(),
        (force + F_fric) * bank->getDy(),
        m_piston->m_body.index);
}

double CombustionChamber::getFrictionForce() const {
    CylinderBank *bank = m_head->getCylinderBank();
    const double v_x = m_piston->m_body.v_x;
    const double v_y = m_piston->m_body.v_y;

    const double v_s =
        v_x * bank->getDx() + v_y * bank->getDy();

    return calculateFrictionForce(v_s);
}
