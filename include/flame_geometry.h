#ifndef ATG_ENGINE_SIM_FLAME_GEOMETRY_H
#define ATG_ENGINE_SIM_FLAME_GEOMETRY_H

#include <vector>

// Spherical flame in a flat cylindrical chamber (flat head, flat piston). The
// flame is centred on the head at the spark plug, which sits at a radial
// offset from the bore axis. For a flame radius r and chamber height h the
// table gives the volume enclosed by the flame and the area of its free front
// (the part of the sphere not touching the head, liner or piston).
//
// Tables are non-dimensional (lengths / bore radius), one row per chamber
// height, each row on its own radius grid from 0 to the farthest chamber
// corner, so the whole chamber is always covered.
class FlameGeometry {
    public:
        FlameGeometry();

        // boreRadius [m], plugOffset = radial plug position / bore radius (0..1)
        void initialize(double boreRadius, double plugOffset);

        // Flame radius enclosing the given volume, chamber height h
        double radius(double volume, double h) const;
        double frontArea(double r, double h) const;

    private:
        static constexpr int RadiusSteps = 128;
        static constexpr int HeightSteps = 128;
        static constexpr double MinHeight = 0.05;
        static constexpr double MaxHeight = 6.0;

        static void evaluate(double rho, double eta, double plugOffset, double *volume, double *area);

        double rowHeight(int i) const;
        double rowMaxRadius(int i) const;
        void locateHeight(double eta, int *i, double *t) const;
        double sampleRow(const std::vector<double> &table, int i, double rho) const;
        double invertRow(int i, double volume) const;

        double m_boreRadius;
        double m_plugOffset;
        std::vector<double> m_volume;   // [HeightSteps][RadiusSteps], / R^3
        std::vector<double> m_area;     // [HeightSteps][RadiusSteps], / R^2
};

#endif /* ATG_ENGINE_SIM_FLAME_GEOMETRY_H */
