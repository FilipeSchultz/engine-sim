#include "../include/flame_geometry.h"

#include "../include/constants.h"

#include <algorithm>
#include <cmath>

FlameGeometry::FlameGeometry() {
    m_boreRadius = 0;
    m_plugOffset = 0;
}

void FlameGeometry::initialize(double boreRadius, double plugOffset) {
    m_boreRadius = boreRadius;
    m_plugOffset = std::fmin(std::fmax(plugOffset, 0.0), 0.95);

    m_volume.assign(HeightSteps * RadiusSteps, 0.0);
    m_area.assign(HeightSteps * RadiusSteps, 0.0);

    for (int i = 0; i < HeightSteps; ++i) {
        const double eta = rowHeight(i);
        const double rhoMax = rowMaxRadius(i);
        for (int k = 0; k < RadiusSteps; ++k) {
            const double rho = rhoMax * k / (RadiusSteps - 1);
            evaluate(rho, eta, m_plugOffset, &m_volume[i * RadiusSteps + k], &m_area[i * RadiusSteps + k]);
        }

        // Keep the volume strictly monotonic for the inversion
        for (int k = 1; k < RadiusSteps; ++k) {
            double &v = m_volume[i * RadiusSteps + k];
            v = std::fmax(v, m_volume[i * RadiusSteps + k - 1] + 1E-12);
        }
    }
}

void FlameGeometry::evaluate(double rho, double eta, double d, double *volume, double *area) {
    // Polar coordinates around the plug on the head plane: along direction phi
    // the chamber extends to the liner at distance L(phi). At distance s the
    // sphere reaches down to sqrt(rho^2 - s^2), cut by the piston at eta.
    // Integrated analytically in s, numerically in phi (symmetric: 0..pi, x2).
    constexpr int Steps = 180;
    const double dphi = constants::pi / Steps;
    const double rho2 = rho * rho;
    const double s_h = std::sqrt(std::fmax(rho2 - eta * eta, 0.0));

    double v = 0, a = 0;
    for (int j = 0; j < Steps; ++j) {
        const double phi = (j + 0.5) * dphi;
        const double sinPhi = std::sin(phi);
        const double L = -d * std::cos(phi) + std::sqrt(1 - d * d * sinPhi * sinPhi);

        const double s0 = std::fmin(s_h, L);    // below: flame touches the piston
        const double s1 = std::fmin(rho, L);    // above: no flame / liner

        v += 0.5 * eta * s0 * s0;
        if (s1 > s0) {
            const double c0 = rho2 - s0 * s0;
            const double c1 = std::fmax(rho2 - s1 * s1, 0.0);
            v += (c0 * std::sqrt(c0) - c1 * std::sqrt(c1)) / 3.0;
            a += rho * (std::sqrt(c0) - std::sqrt(c1));
        }
    }

    *volume = 2 * v * dphi;
    *area = 2 * a * dphi;
}

double FlameGeometry::rowHeight(int i) const {
    return MinHeight + (MaxHeight - MinHeight) * i / (HeightSteps - 1.0);
}

double FlameGeometry::rowMaxRadius(int i) const {
    const double eta = rowHeight(i);
    const double reach = 1 + m_plugOffset;
    return std::sqrt(reach * reach + eta * eta);
}

void FlameGeometry::locateHeight(double eta, int *i, double *t) const {
    const double x = (eta - MinHeight) / (MaxHeight - MinHeight) * (HeightSteps - 1.0);
    const double xc = std::fmin(std::fmax(x, 0.0), HeightSteps - 1.000001);
    *i = (int)xc;
    *t = xc - *i;
}

double FlameGeometry::sampleRow(const std::vector<double> &table, int i, double rho) const {
    const double x = rho / rowMaxRadius(i) * (RadiusSteps - 1.0);
    if (x >= RadiusSteps - 1) return table[i * RadiusSteps + RadiusSteps - 1];

    const int k = (int)std::fmax(x, 0.0);
    const double t = std::fmax(x, 0.0) - k;
    return table[i * RadiusSteps + k] * (1 - t) + table[i * RadiusSteps + k + 1] * t;
}

double FlameGeometry::invertRow(int i, double volume) const {
    const double *row = &m_volume[i * RadiusSteps];
    if (volume <= 0) return 0.0;
    if (volume >= row[RadiusSteps - 1]) return rowMaxRadius(i);

    const int k = (int)(std::upper_bound(row, row + RadiusSteps, volume) - row) - 1;
    const double t = (volume - row[k]) / (row[k + 1] - row[k]);
    return rowMaxRadius(i) * (k + t) / (RadiusSteps - 1);
}

double FlameGeometry::radius(double volume, double h) const {
    if (m_boreRadius <= 0) return 0.0;

    const double R = m_boreRadius;
    int i;
    double t;
    locateHeight(h / R, &i, &t);

    const double v = volume / (R * R * R);
    return R * (invertRow(i, v) * (1 - t) + invertRow(i + 1, v) * t);
}

double FlameGeometry::frontArea(double r, double h) const {
    if (m_boreRadius <= 0) return 0.0;

    const double R = m_boreRadius;
    int i;
    double t;
    locateHeight(h / R, &i, &t);

    const double rho = r / R;
    return R * R * (sampleRow(m_area, i, rho) * (1 - t) + sampleRow(m_area, i + 1, rho) * t);
}
