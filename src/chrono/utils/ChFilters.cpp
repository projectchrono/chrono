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
// Authors: Radu Serban
// =============================================================================
//
// A collection of various data filters
//
// =============================================================================

#include <algorithm>

#include "chrono/utils/ChFilters.h"
#include "chrono/core/ChMathematics.h"

namespace chrono {
namespace utils {

// -----------------------------------------------------------------------------
ChRunningAverage::ChRunningAverage(int n) : m_n(n), m_index(0), m_std(0) {
    m_data.resize(n, 0.0);
}

double ChRunningAverage::Add(double val) {
    m_data[(m_index++) % m_n] = val;
    int size = std::min(m_index, m_n);
    double mean = m_data.sum() / size;
    m_std = std::sqrt(std::pow(m_data - mean, 2.0).sum() / (size - 1));
    return mean;
}

// -----------------------------------------------------------------------------
ChMovingAverage::ChMovingAverage(const std::valarray<double>& data, int n) : m_n(n) {
    int np = (int) data.size();
    m_out.resize(np);

    // Start and end of data
    int lim = ChMin(n, np);
    for (int i = 0; i < lim; i++) {
        m_out[i] = data[i];
        for (int j = 1; j <= i; j++)
            m_out[i] += data[i - j] + data[i + j];
        m_out[i] /= (2 * i + 1);
    }

    for (int i = 0; i < lim; i++) {
        m_out[np - 1 - i] = data[np - 1 - i];
        for (int j = 1; j <= i; j++)
            m_out[np - 1 - i] += data[np - 1 - i - j] + data[np - 1 - i + j];
        m_out[np - 1 - i] /= (2 * i + 1);
    }

    // Middle values
    for (int i = lim; i < np - lim; i++) {
        m_out[i] = data[i];
        for (int j = 1; j <= n; j++)
            m_out[i] += data[i - j] + data[i + j];
        m_out[i] /= (2 * n + 1);
    }
}

}  // end namespace utils
}  // end namespace chrono
