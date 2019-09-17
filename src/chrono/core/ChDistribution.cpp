// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/core/ChDistribution.h"
#include "chrono/core/ChMathematics.h"

namespace chrono {

ChMinMaxDistribution::ChMinMaxDistribution(double mmin, double mmax) : min(mmin), max(mmax) {}

double ChMinMaxDistribution::GetRandom() {
    return min + (::chrono::ChRandom() * (max - min));
}

// -----------------------------------------------------------------------------

ChNormalDistribution::ChNormalDistribution(double mvariance, double mmean)
    : variance(mvariance), mean(mmean), hasSpare(false) {}

double ChNormalDistribution::GetRandom() {
    if (hasSpare) {
        hasSpare = false;
        return (mean + sqrt(variance * rand1) * sin(rand2));
    }

    hasSpare = true;

    rand1 = ::chrono::ChRandom();
    if (rand1 < 1e-100)
        rand1 = 1e-100;
    rand1 = -2.0 * log(rand1);
    rand2 = ::chrono::ChRandom() * CH_C_2PI;

    return (mean + sqrt(variance * rand1) * cos(rand2));
}

// -----------------------------------------------------------------------------

ChWeibullDistribution::ChWeibullDistribution(double mlambda, double mk) : lambda(mlambda), k(mk) {}

double ChWeibullDistribution::GetRandom() {
    double rand = ::chrono::ChRandom();
    if (rand < 1e-100)
        rand = 1e-100;
    return lambda * pow(-log(rand), (1.0 / k));
}

// -----------------------------------------------------------------------------

ChZhangDistribution::ChZhangDistribution(double average_size, double minimum_size) : minsize(minimum_size) {
    lambdar = 1.0 / (average_size - minimum_size);
}

double ChZhangDistribution::GetRandom() {
    double rand = ::chrono::ChRandom();
    if (rand < 1e-100)
        rand = 1e-100;
    return (minsize + (1.0 / lambdar) * (-log(rand)));
}

// -----------------------------------------------------------------------------

ChContinuumDistribution::ChContinuumDistribution(ChVectorDynamic<>& mx, ChVectorDynamic<>& my) : x(mx), y(my) {
    if (mx.size() != my.size())
        throw ChException("Probability curve must have same number of elements in abscysse and ordinates");

    cdf_x = mx;
    cdf_y = my;

    // compute CDF
    double integral = 0;
    for (int i = 0; i < x.size() - 1; i++) {
        integral += 0.5 * (y(i) + y(i + 1)) * (x(i + 1) - x(i));
        cdf_y(i) = integral;
        cdf_x(i) = 0.5 * (x(i + 1) + x(i));
    }

    // normalize if P(x) had not unit integral
    double totintegral = cdf_y(x.size() - 2);
    if (totintegral != 1.0) {
        for (int i = 0; i < x.size() - 1; i++) {
            cdf_y(i) *= 1. / totintegral;
        }
    }

    cdf_x(x.size() - 1) = x(x.size() - 1);
    cdf_y(x.size() - 1) = 1.0;
}

double ChContinuumDistribution::GetRandom() {
    double mx1 = x(0);
    double mx2 = cdf_x(0);
    double my1 = 0;
    double my2 = cdf_y(0);

    double rand = ChRandom();
    for (int i = 0; i < x.size() - 1; i++) {
        if ((rand <= cdf_y(i + 1)) && (rand > cdf_y(i))) {
            mx1 = cdf_x(i);
            mx2 = cdf_x(i + 1);
            my1 = cdf_y(i);
            my2 = cdf_y(i + 1);
            break;
        }
    }
    // linear interp
    double val = mx1 + ((rand - my1) / (my2 - my1)) * (mx2 - mx1);
    return val;
}

// -----------------------------------------------------------------------------

ChDiscreteDistribution::ChDiscreteDistribution(ChVectorDynamic<>& mx, ChVectorDynamic<>& my) : x(mx), y(my) {
    if (mx.size() != my.size())
        throw ChException("Probability values and percentages must have the same size");

    cdf_y = my;

    // compute CDF
    double integral = 0;
    for (int i = 0; i < x.size(); i++) {
        integral += y(i);
        cdf_y(i) = integral;
    }
    // normalize if P(x) had not unit integral
    double totintegral = cdf_y(x.size() - 1);
    if (totintegral != 1.0) {
        for (int i = 0; i < x.size(); i++) {
            cdf_y(i) *= 1. / totintegral;
        }
    }
    cdf_y(x.size() - 1) = 1.0;
}

double ChDiscreteDistribution::GetRandom() {
    double rand = ChRandom();
    double lastval = 0;
    for (int i = 0; i < x.size(); i++) {
        if ((rand <= cdf_y(i)) && (rand > lastval)) {
            return x(i);
        }
        lastval = cdf_y(i);
    }
    return 0;
}

}  // end namespace chrono
