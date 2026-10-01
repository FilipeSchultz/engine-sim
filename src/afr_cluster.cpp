#include "../include/afr_cluster.h"

#include "../include/units.h"
#include "../include/gauge.h"
#include "../include/constants.h"
#include "../include/engine_sim_application.h"

#include <sstream>

AfrCluster::AfrCluster() {
    m_engine = nullptr;
    m_intakeAfrGauge = nullptr;
    m_exhaustAfrGauge = nullptr;
}

AfrCluster::~AfrCluster() {
    /* void */
}

void AfrCluster::initialize(EngineSimApplication *app) {
    UiElement::initialize(app);

    m_intakeAfrGauge = addElement<LabeledGauge>();
    m_exhaustAfrGauge = addElement<LabeledGauge>();

    constexpr float shortenAngle = (float)units::angle(1.0, units::deg);

    // In-cylinder lambda. With ECU port injection the intake manifold only holds
    // air, so the old intake AFR reading is meaningless; this works in both modes.
    // The gauge scale is lambda x 100 (0.50 .. 1.50)
    m_intakeAfrGauge->m_title = "LAMBDA";
    m_intakeAfrGauge->m_unit = "";
    m_intakeAfrGauge->m_precision = 2;
    m_intakeAfrGauge->m_displayScale = 0.01;
    m_intakeAfrGauge->setLocalPosition({ 0, 0 });
    m_intakeAfrGauge->m_gauge->m_min = 50;
    m_intakeAfrGauge->m_gauge->m_max = 150;
    m_intakeAfrGauge->m_gauge->m_minorStep = 5;
    m_intakeAfrGauge->m_gauge->m_majorStep = 25;
    m_intakeAfrGauge->m_gauge->m_maxMinorTick = 7000;
    m_intakeAfrGauge->m_gauge->m_renderText = false;
    m_intakeAfrGauge->m_gauge->m_thetaMin = (float)constants::pi * 1.2f;
    m_intakeAfrGauge->m_gauge->m_thetaMax = -(float)constants::pi * 0.2f;
    m_intakeAfrGauge->m_gauge->m_needleWidth = 4.0f;
    m_intakeAfrGauge->m_gauge->m_gamma = 1.0f;
    m_intakeAfrGauge->m_gauge->m_needleKs = 1000.0f;
    m_intakeAfrGauge->m_gauge->m_needleKd = 20.0f;

    // Green: best power mixture of the engine scripts (~0.8); red: close to
    // the rich limit below which the cylinders no longer ignite (0.5)
    m_intakeAfrGauge->m_gauge->setBandCount(2);
    m_intakeAfrGauge->m_gauge->setBand(
        { app->getGreen(), 75, 85, 3.0f, 6.0f, shortenAngle, -shortenAngle }, 0);
    m_intakeAfrGauge->m_gauge->setBand(
        { app->getRed(), 50, 55, 3.0f, 6.0f, shortenAngle, -shortenAngle }, 1);

    m_exhaustAfrGauge->m_title = "EX. O2";
    m_exhaustAfrGauge->m_unit = "";
    m_exhaustAfrGauge->m_precision = 1;
    m_exhaustAfrGauge->setLocalPosition({ 0, 0 });
    m_exhaustAfrGauge->m_gauge->m_min = 0;
    m_exhaustAfrGauge->m_gauge->m_max = 100;
    m_exhaustAfrGauge->m_gauge->m_minorStep = 5;
    m_exhaustAfrGauge->m_gauge->m_majorStep = 10;
    m_exhaustAfrGauge->m_gauge->m_maxMinorTick = 200;
    m_exhaustAfrGauge->m_gauge->m_thetaMin = (float)constants::pi * 1.2f;
    m_exhaustAfrGauge->m_gauge->m_thetaMax = -(float)constants::pi * 0.2f;
    m_exhaustAfrGauge->m_gauge->m_needleWidth = 4.0f;
    m_exhaustAfrGauge->m_gauge->m_gamma = 1.0f;
    m_exhaustAfrGauge->m_gauge->m_needleKs = 1000.0f;
    m_exhaustAfrGauge->m_gauge->m_needleKd = 20.0f;
    m_exhaustAfrGauge->m_gauge->setBandCount(0);
}

void AfrCluster::destroy() {
    UiElement::destroy();
}

void AfrCluster::update(float dt) {
    UiElement::update(dt);
}

void AfrCluster::render() {
    const Bounds top = m_bounds.verticalSplit(0.5f, 1.0f);
    const Bounds bottom = m_bounds.verticalSplit(0.0f, 0.5f);

    m_intakeAfrGauge->m_bounds = top;
    m_intakeAfrGauge->m_gauge->m_value = (m_engine != nullptr)
        ? static_cast<float>(m_engine->getCylinderLambda() * 100.0)
        : 0.0f;

    m_exhaustAfrGauge->m_bounds = bottom;
    m_exhaustAfrGauge->m_gauge->m_value = (m_engine != nullptr)
        ? static_cast<float>(m_engine->getExhaustO2()) * 100.0f
        : 0.0f;

    UiElement::render();
}
