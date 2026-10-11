#include <gtest/gtest.h>

#include "../include/function.h"
#include "../include/gaussian_filter.h"

#include <cmath>
#include <stdlib.h>

TEST(FunctionTests, FunctionSanityCheck) {
    Function f;
    f.initialize(16, 1.0);
    f.destroy();
}

TEST(FunctionTests, FunctionTriangleFilterTest) {
    Function f;
    f.initialize(0, 1.0);
    for (int i = 0; i < 10; ++i) {
        f.addSample((double)i, (double)i * 2);
    }

    EXPECT_NEAR(f.sampleTriangle(-1.0), 0.0, 1E-6);
    EXPECT_NEAR(f.sampleTriangle(11.0), 18.0, 1E-6);

    for (int i = 0; i < 10; ++i) {
        EXPECT_NEAR(f.sampleTriangle((double)i), (double)i * 2, 1E-6);
    }

    f.destroy();
}

TEST(FunctionTests, FunctionClosestTest) {
    Function f;
    f.initialize(0, 1.0);
    f.addSample(0.0, 1.0);
    f.addSample(2.0, 1.0);
    f.addSample(3.0, 1.1);
    f.addSample(1.0, 1.0);
    f.addSample(5.0, 10.0);
    f.addSample(4.0, 9.0);

    EXPECT_EQ(f.closestSample(2.4), 2);
    EXPECT_EQ(f.closestSample(6.0), 5);

    f.destroy();
}

TEST(FunctionTests, FunctionRandomAddTest) {
    Function f;
    f.initialize(0, 1.0);

    for (int i = 0; i < 1000; ++i) {
        f.addSample(rand() % 1000, i);
    }

    EXPECT_TRUE(f.isOrdered());

    f.destroy();
}

TEST(FunctionTests, FunctionGaussianTest) {
    // Samples (unordered on purpose)
    const double x[] = { 0.0, 2.0, 3.0, 1.0, 5.0, 4.0 };
    const double y[] = { 1.0, 1.0, 5.0, 1.0, 10.0, 9.0 };

    // A filter narrower than the sample spacing returns the samples
    GaussianFilter narrow;
    narrow.initialize(4.0, 1.0);

    Function f;
    f.initialize(0, 1.0, &narrow);
    for (int i = 0; i < 6; ++i) f.addSample(x[i], y[i]);

    for (int i = 0; i < 6; ++i) {
        EXPECT_NEAR(f.sampleGaussian(x[i]), y[i], 1E-3);
    }

    // Beyond the samples: the end values
    EXPECT_NEAR(f.sampleGaussian(100.0), 10.0, 1E-3);
    EXPECT_NEAR(f.sampleGaussian(-100.0), 1.0, 1E-3);
    f.destroy();

    // Default filter (alpha 1, radius 3 x the filter radius): the weighted
    // average with w(d) = exp(-d^2) - exp(-9)
    Function g;
    g.initialize(0, 1.0);
    for (int i = 0; i < 6; ++i) g.addSample(x[i], y[i]);

    for (double s : { 2.0, 2.5, 4.0 }) {
        double sum = 0, weights = 0;
        for (int i = 0; i < 6; ++i) {
            const double d = x[i] - s;
            const double w = std::fmax(std::exp(-d * d) - std::exp(-9.0), 0.0);
            sum += w * y[i];
            weights += w;
        }
        EXPECT_NEAR(g.sampleGaussian(s), sum / weights, 1E-3);
    }

    // Smooth and bounded between the samples
    for (double s = 2.0; s <= 3.0; s += 0.001) {
        const double v = g.sampleGaussian(s);
        EXPECT_GE(v, 1.0);
        EXPECT_LE(v, 10.0);
    }

    g.destroy();
}
