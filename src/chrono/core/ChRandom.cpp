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

#include "chrono/core/ChRandom.h"

namespace chrono {

double ChRandom::Get() {
    return GetInstance().m_distribution(GetInstance().m_generator);
}

void ChRandom::SetSeed(double seed) {
    GetInstance().m_generator.seed(seed);
}

ChRandom::ChRandom() : m_generator(m_rand_device()) {}

ChRandom& ChRandom::GetInstance() {
    static ChRandom chrandom_instance;
    return chrandom_instance;
}

// -----------------------------------------------------------------------------

ChUniformDistribution::ChUniformDistribution(double min, double max)
    : m_generator(m_rand_device()), m_distribution(min, max) {}

double ChUniformDistribution::GetRandom() {
    return m_distribution(m_generator);
}

// -----------------------------------------------------------------------------

ChNormalDistribution::ChNormalDistribution(double mean, double std_dev)
    : m_generator(m_rand_device()), m_distribution(mean, std_dev), m_mean(mean), m_std_dev(std_dev) {}

double ChNormalDistribution::GetRandom() {
    return m_distribution(m_generator);
}

// -----------------------------------------------------------------------------

ChWeibullDistribution::ChWeibullDistribution(double shape_param, double scale_param)
    : m_generator(m_rand_device()),
      m_distribution(shape_param, scale_param),
      m_shape_param(shape_param),
      m_scale_param(scale_param) {}

double ChWeibullDistribution::GetRandom() {
    return m_distribution(m_generator);
}

// -----------------------------------------------------------------------------

ChZhangDistribution::ChZhangDistribution(double average_size, double minimum_size)
    : m_generator(m_rand_device()), m_min_size(minimum_size) {
    m_lambda_r = 1.0 / (average_size - minimum_size);
}

double ChZhangDistribution::GetRandom() {
    double rand = m_distribution(m_generator);
    if (rand < 1e-100)
        rand = 1e-100;
    return (m_min_size + (1.0 / m_lambda_r) * (-std::log(rand)));
}

// -----------------------------------------------------------------------------

ChContinuumDistribution::ChContinuumDistribution(ChVectorDynamic<>& x, ChVectorDynamic<>& y)
    : m_generator(m_rand_device()), m_x(x), m_y(y) {
    if (x.size() != y.size())
        throw std::runtime_error("Probability curve must have same number of elements in abscysse and ordinates");

    m_cdf_x = x;
    m_cdf_y = y;

    // compute CDF
    double integral = 0;
    for (int i = 0; i < m_x.size() - 1; i++) {
        integral += 0.5 * (m_y(i) + m_y(i + 1)) * (m_x(i + 1) - m_x(i));
        m_cdf_y(i) = integral;
        m_cdf_x(i) = 0.5 * (m_x(i + 1) + m_x(i));
    }

    // normalize if P(x) had not unit integral
    double totintegral = m_cdf_y(m_x.size() - 2);
    if (totintegral != 1.0) {
        for (int i = 0; i < m_x.size() - 1; i++) {
            m_cdf_y(i) *= 1. / totintegral;
        }
    }

    m_cdf_x(m_x.size() - 1) = m_x(m_x.size() - 1);
    m_cdf_y(m_x.size() - 1) = 1.0;
}

double ChContinuumDistribution::GetRandom() {
    double x1 = m_x(0);
    double x2 = m_cdf_x(0);
    double y1 = 0;
    double y2 = m_cdf_y(0);

    double rand = m_distribution(m_generator);
    for (int i = 0; i < m_x.size() - 1; i++) {
        if ((rand <= m_cdf_y(i + 1)) && (rand > m_cdf_y(i))) {
            x1 = m_cdf_x(i);
            x2 = m_cdf_x(i + 1);
            y1 = m_cdf_y(i);
            y2 = m_cdf_y(i + 1);
            break;
        }
    }
    // linear interp
    double val = x1 + ((rand - y1) / (y2 - y1)) * (x2 - x1);
    return val;
}

// -----------------------------------------------------------------------------

ChDiscreteDistribution::ChDiscreteDistribution(ChVectorDynamic<>& x, ChVectorDynamic<>& y)
    : m_generator(m_rand_device()), m_x(x), m_y(y) {
    if (x.size() != y.size())
        throw std::runtime_error("Probability values and percentages must have the same size");

    m_cdf_y = y;

    // compute CDF
    double integral = 0;
    for (int i = 0; i < m_x.size(); i++) {
        integral += m_y(i);
        m_cdf_y(i) = integral;
    }
    // normalize if P(x) had not unit integral
    double totintegral = m_cdf_y(m_x.size() - 1);
    if (totintegral != 1.0) {
        for (int i = 0; i < m_x.size(); i++) {
            m_cdf_y(i) *= 1. / totintegral;
        }
    }
    m_cdf_y(m_x.size() - 1) = 1.0;
}

double ChDiscreteDistribution::GetRandom() {
    double rand = m_distribution(m_generator);
    double lastval = 0;
    for (int i = 0; i < m_x.size(); i++) {
        if ((rand <= m_cdf_y(i)) && (rand > lastval)) {
            return m_x(i);
        }
        lastval = m_cdf_y(i);
    }
    return 0;
}

}  // end namespace chrono
