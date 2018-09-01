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
    int np = (int)data.size();
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
    m_Ti = Ti;
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
    m_Td = Td;
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
    m_T1 = T1;
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
    m_Td1 = Td1;
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

//--------------------------------------- IIR filters ----------------------------------
ChButterworth_Lowpass::ChButterworth_Lowpass() {}

ChButterworth_Lowpass::ChButterworth_Lowpass(unsigned int nPoles, double step, double fc) {
    Config(nPoles, step, fc);
}

void ChButterworth_Lowpass::Reset() {
    if (m_n_single == 1) {
        m_u_hist1 = 0.0;
        m_y_hist1 = 0.0;
    }
    for (unsigned int i = 0; i < m_n_biquad; i++) {
        m_biq_u_hist1[i] = m_biq_u_hist2[i] = 0.0;
        m_biq_y_hist1[i] = m_biq_y_hist2[i] = 0.0;
    }
}

void ChButterworth_Lowpass::Config(unsigned int nPoles, double step, double fc) {
    const unsigned int nPolesMin = 1;
    const unsigned int nPolesMax = 6;

    m_Ts = step;
    double wc = fc * CH_C_2PI;

    if (nPoles < nPolesMin) {
        nPoles = 1;
    }
    if (nPoles > nPolesMax) {
        nPoles = nPolesMax;
    }

    if (nPoles == 1) {
        m_n_single = 1;
        m_n_biquad = 0;
    } else {
        m_n_biquad = nPoles / 2;
        m_n_single = nPoles % 2;
    }
    m_Q.resize(m_n_biquad);
    if (m_n_single == 1) {
        for (unsigned int k = 1; k <= (nPoles - 1) / 2; k++) {
            m_Q[k - 1] = -2.0 * cos((2.0 * double(k) + nPoles - 1) * CH_C_PI / (2.0 * nPoles));
        }
    } else {
        for (unsigned int k = 1; k <= nPoles / 2; k++) {
            m_Q[k - 1] = -2.0 * cos(CH_C_PI * (2.0 * double(k) + nPoles - 1) / (2.0 * nPoles));
        }
    }

    m_biq_a0.resize(m_n_biquad);
    m_biq_a1.resize(m_n_biquad);
    m_biq_a2.resize(m_n_biquad);
    m_biq_b0.resize(m_n_biquad);
    m_biq_b1.resize(m_n_biquad);
    m_biq_b2.resize(m_n_biquad);

    double T = m_Ts;

    if (m_n_single == 1) {
        m_b0 = T * wc;
        m_b1 = T * wc;

        m_a0 = T * wc + 2.0;
        m_a1 = T * wc - 2.0;

        m_b0 /= m_a0;
        m_b1 /= m_a0;
        m_a1 /= m_a0;
        m_a0 = 1.0;
    }
    for (size_t i = 0; i < m_n_biquad; i++) {
        m_biq_b0[i] = T * T * wc * wc;
        m_biq_b1[i] = 2.0 * T * T * wc * wc;
        m_biq_b2[i] = T * T * wc * wc;

        m_biq_a0[i] = T * T * wc * wc + 2.0 * m_Q[i] * T * wc + 4.0;
        m_biq_a1[i] = (2.0 * T * T * wc * wc - 8.0);
        m_biq_a2[i] = T * T * wc * wc - 2.0 * m_Q[i] * T * wc + 4.0;

        m_biq_b0[i] /= m_biq_a0[i];
        m_biq_b1[i] /= m_biq_a0[i];
        m_biq_b2[i] /= m_biq_a0[i];

        m_biq_a1[i] /= m_biq_a0[i];
        m_biq_a2[i] /= m_biq_a0[i];
        m_biq_a0[i] = 1.0;
    }
    m_biq_u_hist1.resize(m_n_biquad);
    m_biq_u_hist2.resize(m_n_biquad);
    m_biq_y_hist1.resize(m_n_biquad);
    m_biq_y_hist2.resize(m_n_biquad);

    Reset();
}

double ChButterworth_Lowpass::Filter(double u) {
    double y = 0.0;
    if (m_n_single == 1) {
        y = m_b0 * u + m_b1 * m_u_hist1 - m_a1 * m_y_hist1;
        m_u_hist1 = u;
        m_y_hist1 = y;
        u = y;  // output might be input for the first biquad
    }
    for (size_t i = 0; i < m_n_biquad; i++) {
        y = m_biq_b0[i] * u + m_biq_b1[i] * m_biq_u_hist1[i] + m_biq_b2[i] * m_biq_u_hist2[i] -
            m_biq_a1[i] * m_biq_y_hist1[i] - m_biq_a2[i] * m_biq_y_hist2[i];
        m_biq_u_hist2[i] = m_biq_u_hist1[i];
        m_biq_y_hist2[i] = m_biq_y_hist1[i];
        m_biq_u_hist1[i] = u;
        m_biq_y_hist1[i] = y;
        u = y;  // output might be input for the next biquad, if present
    }
    return y;
}

ChButterworth_Highpass::ChButterworth_Highpass() {}

ChButterworth_Highpass::ChButterworth_Highpass(unsigned int nPoles, double step, double fc) {
    Config(nPoles, step, fc);
}

void ChButterworth_Highpass::Reset() {
    if (m_n_single == 1) {
        m_u_hist1 = 0.0;
        m_y_hist1 = 0.0;
    }
    for (unsigned int i = 0; i < m_n_biquad; i++) {
        m_biq_u_hist1[i] = m_biq_u_hist2[i] = 0.0;
        m_biq_y_hist1[i] = m_biq_y_hist2[i] = 0.0;
    }
}

void ChButterworth_Highpass::Config(unsigned int nPoles, double step, double fc) {
    const unsigned int nPolesMin = 1;
    const unsigned int nPolesMax = 6;

    m_Ts = step;
    double wc = fc * CH_C_2PI;

    if (nPoles < nPolesMin) {
        nPoles = 1;
    }
    if (nPoles > nPolesMax) {
        nPoles = nPolesMax;
    }

    if (nPoles == 1) {
        m_n_single = 1;
        m_n_biquad = 0;
    } else {
        m_n_biquad = nPoles / 2;
        m_n_single = nPoles % 2;
    }
    m_Q.resize(m_n_biquad);
    if (m_n_single == 1) {
        for (unsigned int k = 1; k <= (nPoles - 1) / 2; k++) {
            m_Q[k - 1] = -2.0 * cos((2.0 * double(k) + nPoles - 1) * CH_C_PI / (2.0 * nPoles));
        }
    } else {
        for (unsigned int k = 1; k <= nPoles / 2; k++) {
            m_Q[k - 1] = -2.0 * cos(CH_C_PI * (2.0 * double(k) + nPoles - 1) / (2.0 * nPoles));
        }
    }

    m_biq_a0.resize(m_n_biquad);
    m_biq_a1.resize(m_n_biquad);
    m_biq_a2.resize(m_n_biquad);
    m_biq_b0.resize(m_n_biquad);
    m_biq_b1.resize(m_n_biquad);
    m_biq_b2.resize(m_n_biquad);

    if (m_n_single == 1) {
        m_b0 = 2.0;
        m_b1 = -2.0;

        m_a0 = m_Ts * wc + 2.0;
        m_a1 = m_Ts * wc - 2.0;

        m_b0 /= m_a0;
        m_b1 /= m_a0;
        m_a1 /= m_a0;
        m_a0 = 1.0;
    }
    for (size_t i = 0; i < m_n_biquad; i++) {
        m_biq_b0[i] = 4.0;
        m_biq_b1[i] = -8.0;
        ;
        m_biq_b2[i] = 4.0;
        ;

        m_biq_a0[i] = m_Ts * m_Ts * wc * wc + 2.0 * m_Q[i] * m_Ts * wc + 4.0;
        m_biq_a1[i] = 2.0 * m_Ts * m_Ts * wc * wc - 8.0;
        m_biq_a2[i] = m_Ts * m_Ts * wc * wc - 2.0 * m_Q[i] * m_Ts * wc + 4.0;

        m_biq_b0[i] /= m_biq_a0[i];
        m_biq_b1[i] /= m_biq_a0[i];
        m_biq_b2[i] /= m_biq_a0[i];

        m_biq_a1[i] /= m_biq_a0[i];
        m_biq_a2[i] /= m_biq_a0[i];
        m_biq_a0[i] = 1.0;
    }
    m_biq_u_hist1.resize(m_n_biquad);
    m_biq_u_hist2.resize(m_n_biquad);
    m_biq_y_hist1.resize(m_n_biquad);
    m_biq_y_hist2.resize(m_n_biquad);

    Reset();
}

double ChButterworth_Highpass::Filter(double u) {
    double y = 0.0;
    if (m_n_single == 1) {
        y = m_b0 * u + m_b1 * m_u_hist1 - m_a1 * m_y_hist1;
        m_u_hist1 = u;
        m_y_hist1 = y;
        u = y;  // output might be input for the first biquad
    }
    for (size_t i = 0; i < m_n_biquad; i++) {
        y = m_biq_b0[i] * u + m_biq_b1[i] * m_biq_u_hist1[i] + m_biq_b2[i] * m_biq_u_hist2[i] -
            m_biq_a1[i] * m_biq_y_hist1[i] - m_biq_a2[i] * m_biq_y_hist2[i];
        m_biq_u_hist2[i] = m_biq_u_hist1[i];
        m_biq_y_hist2[i] = m_biq_y_hist1[i];
        m_biq_u_hist1[i] = u;
        m_biq_y_hist1[i] = y;
        u = y;  // output might be input for the next biquad, if present
    }
    return y;
}

ChISO2631_1_AVTransition::ChISO2631_1_AVTransition() {
    Reset();
}

ChISO2631_1_AVTransition::ChISO2631_1_AVTransition(double step, double f4, double Q4) {
    Config(step, f4, Q4);
}

ChISO2631_1_AVTransition::ChISO2631_1_AVTransition(double step, double f3, double f4, double Q4) {
    Config(step, f3, f4, Q4);
}

void ChISO2631_1_AVTransition::Config(double step, double f4, double Q4) {
    // Standard Bilinear Transform
    m_Ts = step;
    m_wc3 = 0.0;
    m_wc4 = CH_C_2PI * f4;
    m_Q4 = Q4;

    m_b0 = m_Q4 * m_Ts * m_Ts * m_wc4 * m_wc4;
    m_b1 = 2.0 * m_Q4 * m_Ts * m_Ts * m_wc4 * m_wc4;
    m_b2 = m_Q4 * m_Ts * m_Ts * m_wc4 * m_wc4;

    m_a0 = m_Q4 * m_Ts * m_Ts * m_wc4 * m_wc4 + 2.0 * m_Ts * m_wc4 + 4.0 * m_Q4;
    m_a1 = 2.0 * m_Q4 * m_Ts * m_Ts * m_wc4 * m_wc4 - 8.0 * m_Q4;
    m_a2 = m_Q4 * m_Ts * m_Ts * m_wc4 * m_wc4 - 2.0 * m_Ts * m_wc4 + 4.0 * m_Q4;

    m_b0 /= m_a0;
    m_b1 /= m_a0;
    m_b2 /= m_a0;

    m_a1 /= m_a0;
    m_a2 /= m_a0;
    m_a0 = 1.0;

    Reset();
}

void ChISO2631_1_AVTransition::Config(double step, double f3, double f4, double Q4) {
    m_Ts = step;
    m_wc3 = CH_C_2PI * f3;
    m_wc4 = CH_C_2PI * f4;
    m_Q4 = Q4;

    m_b0 = m_Q4 * m_Ts * m_wc4 * m_wc4 * (m_Ts * m_wc3 + 2.0);
    m_b1 = m_Q4 * m_Ts * m_wc4 * m_wc4 * (m_Ts * m_wc3 - 2.0) + m_Q4 * m_Ts * m_wc4 * m_wc4 * (m_Ts * m_wc3 + 2.0);
    m_b2 = m_Q4 * m_Ts * m_wc4 * m_wc4 * (m_Ts * m_wc3 - 2.0);

    m_a0 = m_wc3 * (m_Q4 * m_Ts * m_Ts * m_wc4 * m_wc4 + 2.0 * m_Ts * m_wc4 + 4.0 * m_Q4);
    m_a1 = -m_wc3 * (8.0 * m_Q4 - 2.0 * m_Q4 * m_Ts * m_Ts * m_wc4 * m_wc4);
    m_a2 = m_wc3 * (m_Q4 * m_Ts * m_Ts * m_wc4 * m_wc4 - 2.0 * m_Ts * m_wc4 + 4.0 * m_Q4);

    m_b0 /= m_a0;
    m_b1 /= m_a0;
    m_b2 /= m_a0;

    m_a1 /= m_a0;
    m_a2 /= m_a0;
    m_a0 = 1.0;

    Reset();
}

double ChISO2631_1_AVTransition::Filter(double u) {
    double y = m_b0 * u + m_b1 * m_u_hist1 + m_b2 * m_u_hist2 - m_a1 * m_y_hist1 - m_a2 * m_y_hist2;

    m_u_hist2 = m_u_hist1;
    m_u_hist1 = u;

    m_y_hist2 = m_y_hist1;
    m_y_hist1 = y;

    return y;
}

void ChISO2631_1_AVTransition::Reset() {
    m_u_hist1 = m_u_hist2 = 0.0;
    m_y_hist1 = m_y_hist2 = 0.0;
}

ChISO2631_1_UpwardStep::ChISO2631_1_UpwardStep() {
    Reset();
}

ChISO2631_1_UpwardStep::ChISO2631_1_UpwardStep(double step, double f5, double f6, double Q5, double Q6) {
    Config(step, f5, f6, Q5, Q6);
}

void ChISO2631_1_UpwardStep::Config(double step, double f5, double f6, double Q5, double Q6) {
    m_Ts = step;

    m_wc5 = CH_C_2PI * f5;
    m_wc6 = CH_C_2PI * f6;
    m_Q5 = Q5;
    m_Q6 = Q6;

    m_b0 = m_Q6 * (m_Q5 * m_Ts * m_Ts * m_wc5 * m_wc5 + 2.0 * m_Ts * m_wc5 + 4.0 * m_Q5);
    m_b1 = -m_Q6 * (8.0 * m_Q5 - 2.0 * m_Q5 * m_Ts * m_Ts * m_wc5 * m_wc5);
    m_b2 = m_Q6 * (m_Q5 * m_Ts * m_Ts * m_wc5 * m_wc5 - 2.0 * m_Ts * m_wc5 + 4.0 * m_Q5);

    m_a0 = m_Q5 * (m_Q6 * m_Ts * m_Ts * m_wc6 * m_wc6 + 2.0 * m_Ts * m_wc6 + 4.0 * m_Q6);
    m_a1 = -m_Q5 * (8.0 * m_Q6 - 2.0 * m_Q6 * m_Ts * m_Ts * m_wc6 * m_wc6);
    m_a2 = m_Q5 * (m_Q6 * m_Ts * m_Ts * m_wc6 * m_wc6 - 2.0 * m_Ts * m_wc6 + 4.0 * m_Q6);

    m_b0 /= m_a0;
    m_b1 /= m_a0;
    m_b2 /= m_a0;

    m_a1 /= m_a0;
    m_a2 /= m_a0;
    m_a0 = 1.0;

    Reset();
}

void ChISO2631_1_UpwardStep::Reset() {
    m_u_hist1 = m_u_hist2 = 0.0;
    m_y_hist1 = m_y_hist2 = 0.0;
}

double ChISO2631_1_UpwardStep::Filter(double u) {
    double y = m_b0 * u + m_b1 * m_u_hist1 + m_b2 * m_u_hist2 - m_a1 * m_y_hist1 - m_a2 * m_y_hist2;

    m_u_hist2 = m_u_hist1;
    m_u_hist1 = u;

    m_y_hist2 = m_y_hist1;
    m_y_hist1 = y;

    return y;
}

// ----------------------------- Combined filter Wk --------------------------------

const double ChISO2631_1_Wk::f1 = 0.4;
const double ChISO2631_1_Wk::f2 = 100.0;
const double ChISO2631_1_Wk::f3 = 12.5;
const double ChISO2631_1_Wk::f4 = 12.5;
const double ChISO2631_1_Wk::f5 = 2.37;
const double ChISO2631_1_Wk::f6 = 3.35;
const double ChISO2631_1_Wk::Q4 = 0.63;
const double ChISO2631_1_Wk::Q5 = 0.91;
const double ChISO2631_1_Wk::Q6 = 0.91;

ChISO2631_1_Wk::ChISO2631_1_Wk() {}

ChISO2631_1_Wk::ChISO2631_1_Wk(double step) {
    Config(step);
}

void ChISO2631_1_Wk::Config(double step) {
    hp.Config(2, step, f1);
    lp.Config(2, step, f2);
    avt.Config(step, f3, f4, Q4);
    ups.Config(step, f5, f6, Q5, Q6);

    Reset();
}

double ChISO2631_1_Wk::Filter(double u) {
    double y = hp.Filter(u);
    y = lp.Filter(y);
    y = avt.Filter(y);
    y = ups.Filter(y);
    return y;
}

void ChISO2631_1_Wk::Reset() {
    lp.Reset();
    hp.Reset();
    avt.Reset();
    ups.Reset();
}

// ----------------------------- Combined filter Wd --------------------------------

const double ChISO2631_1_Wd::f1 = 0.4;
const double ChISO2631_1_Wd::f2 = 100.0;
const double ChISO2631_1_Wd::f3 = 2.0;
const double ChISO2631_1_Wd::f4 = 2.0;
const double ChISO2631_1_Wd::Q4 = 0.63;

ChISO2631_1_Wd::ChISO2631_1_Wd() {}

ChISO2631_1_Wd::ChISO2631_1_Wd(double step) {
    Config(step);
}

void ChISO2631_1_Wd::Config(double step) {
    hp.Config(2, step, f1);
    lp.Config(2, step, f2);
    avt.Config(step, f3, f4, Q4);

    Reset();
}

double ChISO2631_1_Wd::Filter(double u) {
    double y = hp.Filter(u);
    y = lp.Filter(y);
    y = avt.Filter(y);
    return y;
}

void ChISO2631_1_Wd::Reset() {
    lp.Reset();
    hp.Reset();
    avt.Reset();
}

// ----------------------------- Combined filter Wf --------------------------------

const double ChISO2631_1_Wf::f1 = 0.08;
const double ChISO2631_1_Wf::f2 = 0.63;
const double ChISO2631_1_Wf::f4 = 0.25;
const double ChISO2631_1_Wf::f5 = 0.0625;
const double ChISO2631_1_Wf::f6 = 0.1;
const double ChISO2631_1_Wf::Q4 = 0.86;
const double ChISO2631_1_Wf::Q5 = 0.8;
const double ChISO2631_1_Wf::Q6 = 0.8;

ChISO2631_1_Wf::ChISO2631_1_Wf() {}

ChISO2631_1_Wf::ChISO2631_1_Wf(double step) {
    Config(step);
}

void ChISO2631_1_Wf::Config(double step) {
    hp.Config(2, step, f1);
    lp.Config(2, step, f2);
    avt.Config(step, f4, Q4);
    ups.Config(step, f5, f6, Q5, Q6);

    Reset();
}

double ChISO2631_1_Wf::Filter(double u) {
    double y = hp.Filter(u);
    y = lp.Filter(y);
    y = avt.Filter(y);
    y = ups.Filter(y);
    return y;
}

void ChISO2631_1_Wf::Reset() {
    lp.Reset();
    hp.Reset();
    avt.Reset();
    ups.Reset();
}

// ISO 2631-5 Horizontal weighting

// internal sample step
const double ChISO2631_5_Wxy::m_step = 1.0 / 160.0;

// discrete filter coefficients
const double ChISO2631_5_Wxy::m_b0 = 0.0192752;
const double ChISO2631_5_Wxy::m_b1 = 0.00433451;
const double ChISO2631_5_Wxy::m_b2 = -0.0167763;
const double ChISO2631_5_Wxy::m_a0 = 1.0;
const double ChISO2631_5_Wxy::m_a1 = -1.957115;
const double ChISO2631_5_Wxy::m_a2 = 0.963949;

ChISO2631_5_Wxy::ChISO2631_5_Wxy() {
    Reset();
}

void ChISO2631_5_Wxy::Reset() {
    m_u_hist1 = m_u_hist2 = 0.0;
    m_y_hist1 = m_y_hist2 = 0.0;
}

double ChISO2631_5_Wxy::Filter(double u) {
    double y = m_b0 * u + m_b1 * m_u_hist1 + m_b2 * m_u_hist2 - m_a1 * m_y_hist1 - m_a2 * m_y_hist2;

    m_u_hist2 = m_u_hist1;
    m_u_hist1 = u;
    m_y_hist2 = m_y_hist1;
    m_y_hist1 = y;

    return y;
}

// vertical shock weighting filter ISO 2631-5
const double ChISO2631_5_Wz::m_w[13][7] = {{0.00130, 0.01841, -0.00336, 0.01471, 0.00174, 0.00137, 0.00145},
                                           {-0.00646, -0.00565, -0.00539, 0.01544, -0.00542, 0.00381, 0.00497},
                                           {-0.00091, -0.02073, 0.00708, -0.00091, 0.00255, -0.00216, 0.01001},
                                           {0.00898, -0.02626, 0.00438, -0.00595, -0.00774, -0.00034, 0.0128},
                                           {0.00201, 0.00579, 0.00330, -0.00065, -0.00459, -0.00417, -0.00468},
                                           {0.00158, 0.00859, 0.00166, 0.00490, -0.00546, 0.00057, -0.00797},
                                           {0.00361, 0.00490, 0.00452, 0.00079, -0.00604, -0.00638, -0.00529},
                                           {0.00167, -0.00098, 0.00743, 0.00795, -0.01095, 0.00627, -0.00341},
                                           {-0.00078, -0.00261, 0.00771, 0.00600, -0.00908, 0.00504, 0.00135},
                                           {-0.00405, -0.00210, 0.00520, 0.00176, -0.00465, -0.00198, 0.00451},
                                           {-0.00563, 0.00218, -0.00105, 0.00195, 0.00296, -0.00190, 0.00306},
                                           {-0.00372, 0.00037, -0.00045, -0.00197, 0.00289, -0.00448, 0.00216},
                                           {-0.31088, -0.95883, -0.67105, 0.14423, 0.04063, 0.07029, 1.03300}};
const double ChISO2631_5_Wz::m_W[8] = {57.96539, 52.32773, 49.78227, 53.16885, 56.02619, -27.79550, 72.34446, 21.51959};

ChISO2631_5_Wz::ChISO2631_5_Wz() {}

void ChISO2631_5_Wz::Filter(std::vector<double>& u, std::vector<double>& y) {
    std::vector<double> asz(8);
    for (size_t i = 0; i < u.size(); i++) {
        asz.push_back(u[i]);
    }
    y.resize(asz.size());
    for (size_t i = 0; i < y.size(); i++) {
        y[i] = 0.0;
    }
    ChMatrixDynamic<> x(asz.size(), 7);
    // indices run from 1 to N to make it compatible to Matlab
    for (size_t t = 9; t <= asz.size(); t++) {
        for (size_t j = 1; j <= 7; j++) {
            double S1 = 0.0;
            for (size_t k = 1; k <= 4; k++) {
                S1 += y[t - k - 1] * m_w[k - 1][j - 1];
            }
            double S2 = 0.0;
            size_t kk = 1;
            for (size_t k = 5; k <= 12; k++) {
                S2 += asz[t - kk - 1] * m_w[k - 1][j - 1];
                kk++;
            }
            x(t - 1, j - 1) = tanh(S1 + S2 + m_w[12][j - 1]);
        }
        y[t - 1] = 0.0;
        for (size_t k = 1; k <= 7; k++) {
            y[t - 1] += m_W[k - 1] * x(t - 1, k - 1);
        }
        y[t - 1] += m_W[7];
    }
    y.erase(y.begin(), y.begin() + 8);  // get rid of leading zeros
}

// ISO 2661-1 Seat cushion logger class
ChISO2631_Vibration_SeatCushionLogger::ChISO2631_Vibration_SeatCushionLogger() {}

ChISO2631_Vibration_SeatCushionLogger::ChISO2631_Vibration_SeatCushionLogger(double step) {
    Config(step);
}

void ChISO2631_Vibration_SeatCushionLogger::Config(double step) {
    m_step = step;
    m_logging_time = 0.0;

    // prepare all filters for 1st usage
    m_filter_wd_x.Config(step);
    m_filter_wd_y.Config(step);
    m_filter_wk_z.Config(step);

    m_filter_int_aw_x.Config(step);
    m_filter_int_aw_y.Config(step);
    m_filter_int_aw_z.Config(step);

    m_filter_int_vdv_x.Config(step);
    m_filter_int_vdv_y.Config(step);
    m_filter_int_vdv_z.Config(step);

    Reset();
}

void ChISO2631_Vibration_SeatCushionLogger::AddData(double speed, double acc_x, double acc_y, double acc_z) {
    double startFactor = ChSineStep(m_logging_time, m_tstart1, 0.0, m_tstart2, 1.0);

    m_data_speed.push_back(speed);

    m_data_acc_x.push_back(startFactor * acc_x);
    m_data_acc_y.push_back(startFactor * acc_y);
    m_data_acc_z.push_back(startFactor * acc_z);

    m_data_acc_x_wd.push_back(m_filter_wd_x.Filter(m_data_acc_x.back()));
    m_data_acc_y_wd.push_back(m_filter_wd_y.Filter(m_data_acc_y.back()));
    m_data_acc_z_wk.push_back(m_filter_wk_z.Filter(m_data_acc_z.back()));

    m_data_aw_x_i.push_back(m_filter_int_aw_x.Filter(pow(m_data_acc_x_wd.back(), 2.0)));
    m_data_aw_y_i.push_back(m_filter_int_aw_y.Filter(pow(m_data_acc_y_wd.back(), 2.0)));
    m_data_aw_z_i.push_back(m_filter_int_aw_z.Filter(pow(m_data_acc_z_wk.back(), 2.0)));

    m_data_vdv_x_i.push_back(m_filter_int_vdv_x.Filter(pow(m_data_acc_x_wd.back(), 4.0)));
    m_data_vdv_y_i.push_back(m_filter_int_vdv_y.Filter(pow(m_data_acc_y_wd.back(), 4.0)));
    m_data_vdv_z_i.push_back(m_filter_int_vdv_z.Filter(pow(m_data_acc_z_wk.back(), 4.0)));

    if (m_data_aw_x_avg.size() == 0) {
        m_data_aw_x_avg.push_back(0.0);
        m_data_aw_y_avg.push_back(0.0);
        m_data_aw_z_avg.push_back(0.0);

        m_data_vdv_x_avg.push_back(0.0);
        m_data_vdv_y_avg.push_back(0.0);
        m_data_vdv_z_avg.push_back(0.0);
    } else {
        m_data_aw_x_avg.push_back(sqrt(m_data_aw_x_i.back() / m_logging_time));
        m_data_aw_y_avg.push_back(sqrt(m_data_aw_y_i.back() / m_logging_time));
        m_data_aw_z_avg.push_back(sqrt(m_data_aw_z_i.back() / m_logging_time));

        m_data_vdv_x_avg.push_back(pow(m_data_vdv_x_i.back() / m_logging_time, 0.25));
        m_data_vdv_y_avg.push_back(pow(m_data_vdv_y_i.back() / m_logging_time, 0.25));
        m_data_vdv_z_avg.push_back(pow(m_data_vdv_z_i.back() / m_logging_time, 0.25));
    }

    m_logging_time += m_step;
}

double ChISO2631_Vibration_SeatCushionLogger::GetVDV() {
    const double kx = 1.4;
    const double ky = 1.4;
    const double kz = 1.0;

    return sqrt(pow(kx * m_data_vdv_x_avg.back(), 2.0) + pow(ky * m_data_vdv_y_avg.back(), 2.0) +
                pow(kz * m_data_vdv_z_avg.back(), 2.0));
}

void ChISO2631_Vibration_SeatCushionLogger::Reset() {
    // Reset() prepares the instance for reusing with the same configuration, but new input data
    m_logging_time = 0.0;

    // clear data containers
    m_data_speed.clear();

    m_data_acc_x.clear();
    m_data_acc_y.clear();
    m_data_acc_z.clear();

    m_data_acc_x_wd.clear();
    m_data_acc_y_wd.clear();
    m_data_acc_z_wk.clear();

    m_data_aw_x_i.clear();
    m_data_aw_y_i.clear();
    m_data_aw_z_i.clear();

    m_data_vdv_x_i.clear();
    m_data_vdv_y_i.clear();
    m_data_vdv_z_i.clear();

    m_data_aw_x_avg.clear();
    m_data_aw_y_avg.clear();
    m_data_aw_z_avg.clear();

    m_data_vdv_x_avg.clear();
    m_data_vdv_y_avg.clear();
    m_data_vdv_z_avg.clear();

    // reset the filter classes
    m_filter_wd_x.Reset();
    m_filter_wd_y.Reset();
    m_filter_wk_z.Reset();

    m_filter_int_aw_x.Reset();
    m_filter_int_aw_y.Reset();
    m_filter_int_aw_z.Reset();

    m_filter_int_vdv_x.Reset();
    m_filter_int_vdv_y.Reset();
    m_filter_int_vdv_z.Reset();
}

double ChISO2631_Vibration_SeatCushionLogger::mean(std::vector<double>& v) {
    double s = 0.0;
    for (size_t i = 0; i < v.size(); i++) {
        s += v[i];
    }
    return s / double(v.size());
}

double ChISO2631_Vibration_SeatCushionLogger::rms(std::vector<double>& v) {
    double s = 0.0;
    double m = mean(v);
    for (size_t i = 0; i < v.size(); i++) {
        s += (v[i] - m) * (v[i] - m);
    }
    return sqrt(s / double(v.size()));
}

double ChISO2631_Vibration_SeatCushionLogger::GetAW_V() {
    const double kx = 1.4;
    const double ky = 1.4;
    const double kz = 1.0;

    return sqrt(pow(kx * GetAW_X(), 2.0) + pow(ky * GetAW_Y(), 2.0) + pow(kz * GetAW_Z(), 2.0));
}

double ChISO2631_Vibration_SeatCushionLogger::maxval(std::vector<double>& v) {
    double vm = 0.0;
    for (size_t i = 0; i < v.size(); i++) {
        if (v[i] > vm) {
            vm = v[i];
        }
    }
    return vm;
}

void ChISO2631_Vibration_SeatCushionLogger::GeneratePlotFile(std::string fName, std::string testInfo) {
    std::ofstream plt(fName);
    if (!plt.is_open()) {
        std::cout << "ChISO2631_SeatCushionLogger: Connot generate file '" << fName << "', bailing out!" << std::endl;
        return;
    }
    plt << "$ALLDATA << EOD" << std::endl;
    for (size_t i = 0; i < m_data_acc_x.size(); i++) {
        double t = m_step * double(i);
        plt << t << "\t" << m_data_acc_x[i] << "\t" << m_data_acc_y[i] << "\t" << m_data_acc_z[i] << "\t"
            << m_data_acc_x_wd[i] << "\t" << m_data_acc_y_wd[i] << "\t" << m_data_acc_z_wk[i] << "\t"
            << m_data_aw_x_i[i] << "\t" << m_data_aw_y_i[i] << "\t" << m_data_aw_z_i[i] << "\t" << m_data_aw_x_avg[i]
            << "\t" << m_data_aw_y_avg[i] << "\t" << m_data_aw_z_avg[i] << std::endl;
    }
    plt << "EOD" << std::endl;
    plt << "set title 'ISO2631 Vibration Test: " << testInfo << "'" << std::endl;
    plt << "set xlabel 'Time [s]'" << std::endl;
    plt << "set ylabel 'A [m/s^2]'" << std::endl;
    plt << "plot $ALLDATA u 1:2  t 'Ax' with lines, \\" << std::endl;
    plt << "     $ALLDATA u 1:3  t 'Ay' with lines, \\" << std::endl;
    plt << "     $ALLDATA u 1:4  t 'Az' with lines" << std::endl;
    plt << "pause -1" << std::endl;
    plt << "set title 'ISO2631 Vibration Test: " << testInfo << "'" << std::endl;
    plt << "set xlabel 'Time [s]'" << std::endl;
    plt << "set ylabel 'AW [m/s^2]'" << std::endl;
    plt << "plot $ALLDATA u 1:5  t 'AWx' with lines, \\" << std::endl;
    plt << "     $ALLDATA u 1:6  t 'AWy' with lines, \\" << std::endl;
    plt << "     $ALLDATA u 1:7  t 'AWz' with lines" << std::endl;
    plt << "pause -1" << std::endl;
    plt << "set title 'ISO2631 Vibration Test: " << testInfo << "'" << std::endl;
    plt << "set xlabel 'Time [s]'" << std::endl;
    plt << "set ylabel 'AWi [m/s^2]'" << std::endl;
    plt << "plot $ALLDATA u 1:8  t 'AWxi' with lines, \\" << std::endl;
    plt << "     $ALLDATA u 1:9  t 'AWyi' with lines, \\" << std::endl;
    plt << "     $ALLDATA u 1:10  t 'AWzi' with lines" << std::endl;
    plt << "pause -1" << std::endl;
    plt << "set title 'ISO2631 Vibration Test: " << testInfo << "'" << std::endl;
    plt << "set xlabel 'Time [s]'" << std::endl;
    plt << "set ylabel 'AWavg [m/s^2]'" << std::endl;
    plt << "plot $ALLDATA u 1:11  t 'AWxavg' with lines, \\" << std::endl;
    plt << "     $ALLDATA u 1:12  t 'AWyavg' with lines, \\" << std::endl;
    plt << "     $ALLDATA u 1:13  t 'AWzavz' with lines" << std::endl;
    plt.close();
}

// ISO2631-5 shock data logger

ChISO2631_Shock_SeatCushionLogger::ChISO2631_Shock_SeatCushionLogger() : m_speed(0) {}

ChISO2631_Shock_SeatCushionLogger::ChISO2631_Shock_SeatCushionLogger(double step) {
    Config(step);
}

void ChISO2631_Shock_SeatCushionLogger::AddData(double speed, double ax, double ay, double az) {
    double startFactor = ChSineStep(m_logging_time, m_tstart1, 0.0, m_tstart2, 1.0);

    // speed of a vehicle changes very much during obstacle crossing, we take the first value as significant
    // instead of an average value
    if (m_speed == 0.0) {
        m_speed = speed;
    }
    m_raw_inp_x.AddPoint(m_logging_time, m_lpx.Filter(startFactor * ax));
    m_raw_inp_y.AddPoint(m_logging_time, m_lpy.Filter(startFactor * ay));
    m_raw_inp_z.AddPoint(m_logging_time, m_lpz.Filter(startFactor * az));

    m_logging_time += m_step_inp;
}

void ChISO2631_Shock_SeatCushionLogger::Config(double step) {
    m_step_inp = step;

    m_lpx.Config(4, m_step_inp, 75.0);
    m_lpy.Config(4, m_step_inp, 75.0);
    m_lpz.Config(4, m_step_inp, 75.0);

    Reset();
}

double ChISO2631_Shock_SeatCushionLogger::GetSe() {
    // generate filter input with fs = 160 Hz
    size_t nInDat = m_logging_time / m_step;

    m_inp_x.resize(nInDat);
    m_inp_y.resize(nInDat);
    m_inp_z.resize(nInDat);

    for (size_t i = 0; i < nInDat; i++) {
        double t = m_step * double(i);
        m_inp_x[i] = m_raw_inp_x.Get_y(t);
        m_inp_y[i] = m_raw_inp_y.Get_y(t);
        m_inp_z[i] = m_raw_inp_z.Get_y(t);
    }

    m_out_x.resize(nInDat);
    m_out_y.resize(nInDat);
    m_out_z.resize(nInDat);

    for (size_t i = 0; i < nInDat; i++) {
        double t = m_step * double(i);
        m_out_x[i] = m_weighting_x.Filter(m_inp_x[i]);
        m_out_y[i] = m_weighting_y.Filter(m_inp_y[i]);
    }

    m_weighting_z.Filter(m_inp_z, m_out_z);

    m_dkx = CalcPeaks(m_inp_x, false);
    m_dky = CalcPeaks(m_inp_y, false);
    m_dkz = CalcPeaks(m_inp_z, true);

    return pow(pow(m_mx * m_dkx, 6.0) + pow(m_my * m_dky, 6.0) + pow(m_mz * m_dkz, 6.0), 1.0 / 6.0);
}

double ChISO2631_Shock_SeatCushionLogger::CalcPeaks(std::vector<double>& v, bool vertical) {
    double d = 0.0;
    size_t id1 = 0;
    size_t id2 = 0;
    double mx;
    for (size_t i = 0; i < v.size() - 1; i++) {
        id2 = i;
        if ((v[i] > 0 && v[i + 1] < 0) || (v[i] < 0 && v[i + 1] > 0)) {
            if (v[id2] < 0 && !vertical) {
                mx = 0.0;
                for (size_t k = id1; k <= id2; k++) {
                    if (v[k] < mx) {
                        mx = v[k];
                    }
                }
            } else {
                mx = 0.0;
                for (size_t k = id1; k <= id2; k++) {
                    if (v[k] > mx) {
                        mx = v[k];
                    }
                }
            }
            d += pow(mx, 6.0);
            id1 = id2;
        }
    }

    return pow(d, 1.0 / 6.0);
}

void ChISO2631_Shock_SeatCushionLogger::Reset() {
    m_logging_time = 0.0;

    m_speed = 0;

    m_lpx.Reset();
    m_lpy.Reset();
    m_lpz.Reset();

    m_weighting_x.Reset();
    m_weighting_y.Reset();

    m_raw_inp_x.Reset();
    m_raw_inp_y.Reset();
    m_raw_inp_z.Reset();

    m_inp_x.clear();
    m_inp_y.clear();
    m_inp_z.clear();

    m_out_x.clear();
    m_out_y.clear();
    m_out_z.clear();
}

}  // end namespace utils
}  // end namespace chrono
