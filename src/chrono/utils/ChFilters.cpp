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
    m_std = (size == 1) ? 0 : std::sqrt(std::pow(m_data - mean, 2.0).sum() / (size - 1));
    return mean;
}

void ChRunningAverage::Reset() {
    m_index = 0;
    m_std = 0;
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

//******************************************************
// Integral filter implementation

ChFilterI::ChFilterI(double step, double Ti) {
    Config(step, Ti);
}

void ChFilterI::Reset() {
    m_u_old = 0.0;
    m_y_old = 0.0;
}

void ChFilterI::Config(double step, double Ti) {
    m_step = step;
    m_Ti   = Ti;
    Reset();
}

double ChFilterI::Filter(double u) {
     m_y_old += (u + m_u_old) * m_step / (2.0 * m_Ti); 
     m_u_old = u; 
     
     return m_y_old; 
}

//******************************************************
// Differential filter implementation

ChFilterD::ChFilterD(double step, double Td) {
    Config(step, Td);
}

void ChFilterD::Reset() {
    m_u_old = 0.0;
    m_y_old = 0.0;
}

void ChFilterD::Config(double step, double Td) {
    m_Td   = Td;
    m_step = step;
    Reset();
}

double ChFilterD::Filter(double u) {
     m_y_old = (u - m_u_old) * m_Td / m_step; 
     m_u_old = u; 
     
     return m_y_old; 
}

// Time delay implementation
ChFilterPT1::ChFilterPT1(double step, double Td, double Kpt1) {
    Config(step, Td, Kpt1);
}

void ChFilterPT1::Reset() {
    m_u_old = 0.0;
    m_y_old = 0.0;
}

void ChFilterPT1::Config(double step, double T1, double Kpt1) {
    m_T1   = T1;
    m_Kpt1 = Kpt1;
    m_step = step;
    Reset();
}

double ChFilterPT1::Filter(double u) {
     m_y_old += (m_Kpt1 * u - m_y_old) * m_step / (m_T1 + m_step); 
     m_u_old = u; 
     
     return m_y_old; 
}

// PD1 controller implementation
ChFilterPD1::ChFilterPD1(double step, double Td1, double Kdt1) {
    Config(step, Td1, Kdt1);
}

void ChFilterPD1::Reset() {
    m_u_old = 0.0;
    m_y_old = 0.0;
}

void ChFilterPD1::Config(double step, double Td1, double Kdt1) {
    m_Td1  = Td1;
    m_Kdt1 = Kdt1;
    m_step = step;
    Reset();
}

double ChFilterPD1::Filter(double u) {
     m_y_old = m_Kdt1 * (u + (u - m_u_old) * m_Td1 / m_step); 
     m_u_old = u; 
     
     return m_y_old; 
}

// PDT1 controller implementation
ChFilterPDT1::ChFilterPDT1(double step, double Td1, double T1, double Kp) {
    Config(step, Td1, T1, Kp);
}

void ChFilterPDT1::Reset() {
    m_u_old = 0.0;
    m_y_old = 0.0;
    pd1.Reset();
    pt1.Reset();
}

void ChFilterPDT1::Config(double step, double Td1, double T1, double Kp) {
    m_step = step;
    pd1.Config(step, Td1);
    pt1.Config(step, T1, Kp);
    Reset();
}

double ChFilterPDT1::Filter(double u) {
     double u1 = pd1.Filter(u);
     
     return pt1.Filter(u1); 
}


}  // end namespace utils
}  // end namespace chrono
