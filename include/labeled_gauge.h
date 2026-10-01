#ifndef ATG_ENGINE_SIM_LABELED_GAUGE_H
#define ATG_ENGINE_SIM_LABELED_GAUGE_H

#include "ui_element.h"

#include "gauge.h"

#include <string>

class LabeledGauge : public UiElement {
    public:
        LabeledGauge();
        virtual ~LabeledGauge();

        virtual void initialize(EngineSimApplication *app);
        virtual void destroy();

        virtual void update(float dt);
        virtual void render();

        Gauge *m_gauge;
        std::string m_title;

        int m_precision;
        bool m_spaceBeforeUnit;

        // The gauge scale is integer; fractional quantities (e.g. lambda) are
        // drawn scaled up and shown as m_value * m_displayScale
        double m_displayScale = 1.0;
        std::string m_unit;

        float m_margin = 10.0f;
        float m_needleInnerRadius = 0.1f;
        float m_needleOuterRadius = 0.7f;
};

#endif /* ATG_ENGINE_SIM_LABELED_GAUGE_H */
