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
// Authors: Rainer Gericke
// =============================================================================
//
// =============================================================================

#include <chrono/core/ChChrono.h>
#include <chrono/core/ChMath.h>
#include <chrono/core/ChMatrixDynamic.h>
#include <chrono/motion_functions/ChFunction_Recorder.h>
#include <chrono/utils/ChFilters.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

using namespace chrono;
using namespace chrono::utils;

double mean(std::vector<double>& v) {
    double s = 0;
    for (size_t i = 0; i < v.size(); i++) {
        s += v[i];
    }
    return s / double(v.size());
}

double rms(std::vector<double>& v) {
    double avg = mean(v);
    double s = 0;
    for (size_t i = 0; i < v.size(); i++) {
        s += (v[i] - avg) * (v[i] - avg);
    }
    return sqrt(s / double(v.size()));
}

double rms(ChMatrixDynamic<>& m, size_t iRow) {
    size_t nCol = m.GetColumns();
    double avg = 0.0;
    for (size_t i = 0; i < nCol; i++) {
        avg += m(iRow, i);
    }
    avg /= double(nCol);
    double s = 0;
    for (size_t i = 0; i < nCol; i++) {
        s += (m(iRow, i) - avg) * (m(iRow, i) - avg);
    }
    return sqrt(s / double(nCol));
}

int test_iso2631_1_wk() {
    int rcode_wk = 0;
    // transtion frequencies for frequency weighting tolerances (ISO 8041)
    const double ftol_wk1 = pow(10.0, -6.0 / 10.0);
    const double ftol_wk2 = pow(10.0, -2.0 / 10.0);
    const double ftol_wk3 = pow(10.0, 18.0 / 10.0);
    const double ftol_wk4 = pow(10.0, 22.0 / 10.0);

    // expected frequncy weightings for filter Wk (ISO 2631-1)
    double expect_wk_data[] = {31.2e-3, 48.6e-3, 79e-3,   121e-3,  182e-3, 263e-3,  352e-3, 418e-3,  459e-3,  477e-3,
                               482e-3,  484e-3,  494e-3,  531e-3,  631e-3, 804e-3,  967e-3, 1039e-3, 1054e-3, 1036e-3,
                               988e-3,  902e-3,  768e-3,  636e-3,  513e-3, 405e-3,  314e-3, 246e-3,  186e-3,  132e-3,
                               88.7e-3, 54e-3,   28.5e-3, 15.2e-3, 7.9e-3, 3.98e-3, 1.95e-3};

    std::vector<double> expect_wk(expect_wk_data, expect_wk_data + sizeof(expect_wk_data) / sizeof(double));
    std::vector<double> expect_wk_upper;
    std::vector<double> expect_wk_lower;

    // test frequencies in one-third octaves
    std::vector<double> tst_freq_wk;
    for (int i = -17; i <= 26; i++) {
        double f = pow(10.0, double(i) / 10.0);
        if (i >= -10) {
            tst_freq_wk.push_back(f);
        }
    }

    // calculate tolerance limits
    for (int i = 0; i < tst_freq_wk.size(); i++) {
        if (tst_freq_wk[i] <= ftol_wk1) {
            expect_wk_upper.push_back(1.26 * expect_wk[i]);
            expect_wk_lower.push_back(0.0002);
        } else if (tst_freq_wk[i] > ftol_wk1 && tst_freq_wk[i] < ftol_wk2) {
            expect_wk_upper.push_back(1.26 * expect_wk[i]);
            expect_wk_lower.push_back(0.79 * expect_wk[i]);
        } else if (tst_freq_wk[i] >= ftol_wk2 && tst_freq_wk[i] <= ftol_wk3) {
            expect_wk_upper.push_back(1.12 * expect_wk[i]);
            expect_wk_lower.push_back(0.89 * expect_wk[i]);
        } else if (tst_freq_wk[i] > ftol_wk3 && tst_freq_wk[i] < ftol_wk4) {
            expect_wk_upper.push_back(1.26 * expect_wk[i]);
            expect_wk_lower.push_back(0.79 * expect_wk[i]);
        } else {
            expect_wk_upper.push_back(1.26 * expect_wk[i]);
            expect_wk_lower.push_back(0.0002);
        }
    }
    int rcode = 0;
    double dT = 0.001;
    size_t nFreqWk = tst_freq_wk.size();
    size_t nTvals = 1 + (1.0 / tst_freq_wk[0] * 10 / dT);

    ChMatrixDynamic<> sig_in_wk(nFreqWk, nTvals);

    std::vector<double> rms_in(nFreqWk);
    std::vector<double> rms_wk(nFreqWk);

    std::vector<double> t(nTvals);
    for (size_t j = 0; j < nTvals; j++) {
        t[j] = dT * double(j);
    }
    for (size_t i = 0; i < nFreqWk; i++) {
        for (size_t j = 0; j < nTvals; j++) {
            sig_in_wk(i, j) = sin(CH_C_2PI * tst_freq_wk[i] * t[j]);
        }
        rms_in[i] = rms(sig_in_wk, i);
    }

    ChISO2631_1_Wk wk(dT);
    for (size_t i = 0; i < nFreqWk; i++) {
        std::vector<double> sig_out;
        for (size_t j = 0; j < nTvals; j++) {
            sig_out.push_back(wk.Filter(sig_in_wk[i][j]));
        }
        rms_wk[i] = rms(sig_out) / rms_in[i];
        wk.Reset();
    }

    std::ofstream plt("filtertest_wk.plt");
    plt << "$FILTERWK << EOD" << std::endl;
    for (size_t i = 0; i < nFreqWk; i++) {
        plt << tst_freq_wk[i] << "\t" << expect_wk_upper[i] << "\t" << rms_wk[i] << "\t" << expect_wk_lower[i]
            << std::endl;
    }
    plt << "EOD" << std::endl;
    plt << "set title 'Test ISO2631 Wk Filter in Chrono'" << std::endl;
    plt << "set xlabel 'Frequency [Hz]'" << std::endl;
    plt << "set ylabel 'Weighting []'" << std::endl;
    plt << "set logscale xy" << std::endl;
    plt << "plot $FILTERWK t 'ISO2631 Wk Upper Limit' with lines, \\" << std::endl;
    plt << "     $FILTERWK u 1:3 t 'ISO2631 Wk Filtertest' with lines, \\" << std::endl;
    plt << "     $FILTERWK u 1:4 t 'ISO2631 Wk Lower Limits' with lines" << std::endl;
    plt.close();

    // Correction of the first and the last values of expect_wk_lower to zero (it could not be plotted in logscale!)
    expect_wk_lower.front() = 0.0;
    expect_wk_lower.back() = 0.0;
    int n_wk_errors = 0;
    for (size_t i = 0; i < nFreqWk; i++) {
        if (rms_wk[i] < expect_wk_lower[i] || rms_wk[i] > expect_wk_upper[i]) {
            n_wk_errors++;
        }
    }
    std::cout << "Errors Wk = " << n_wk_errors << std::endl;
    if (n_wk_errors > 0) {
        rcode_wk = 1;
    }
    return rcode_wk;
}

int test_iso2631_1_wd() {
    int rcode_wd = 0;
    // transtion frequencies for frequency weighting tolerances (ISO 8041)
    const double ftol_wd1 = pow(10.0, -6.0 / 10.0);
    const double ftol_wd2 = pow(10.0, -2.0 / 10.0);
    const double ftol_wd3 = pow(10.0, 18.0 / 10.0);
    const double ftol_wd4 = pow(10.0, 22.0 / 10.0);

    // expected frequncy weightings for filter Wd (ISO 2631-1)
    double expect_wd_data[] = {62.4e-3, 97.3e-3, 158e-3,  243e-3,  365e-3,  530e-3,  713e-3,  853e-3,  944e-3,  992e-3,
                               1011e-3, 1008e-3, 968e-3,  890e-3,  776e-3,  642e-3,  512e-3,  409e-3,  323e-3,  253e-3,
                               202e-3,  161e-3,  125e-3,  100e-3,  80e-3,   63.2e-3, 49.4e-3, 38.8e-3, 29.5e-3, 21.1e-3,
                               14.1e-3, 8.63e-3, 4.55e-3, 2.43e-3, 1.26e-3, 0.64e-3, 0.31e-3};

    std::vector<double> expect_wd(expect_wd_data, expect_wd_data + sizeof(expect_wd_data) / sizeof(double));
    std::vector<double> expect_wd_upper;
    std::vector<double> expect_wd_lower;

    // test frequencies in one-third octaves
    std::vector<double> tst_freq_wd;
    for (int i = -17; i <= 26; i++) {
        double f = pow(10.0, double(i) / 10.0);
        if (i >= -10) {
            tst_freq_wd.push_back(f);
        }
    }

    // calculate tolerance limits
    for (int i = 0; i < tst_freq_wd.size(); i++) {
        if (tst_freq_wd[i] <= ftol_wd1) {
            expect_wd_upper.push_back(1.26 * expect_wd[i]);
            expect_wd_lower.push_back(0.00002);
        } else if (tst_freq_wd[i] > ftol_wd1 && tst_freq_wd[i] < ftol_wd2) {
            expect_wd_upper.push_back(1.26 * expect_wd[i]);
            expect_wd_lower.push_back(0.79 * expect_wd[i]);
        } else if (tst_freq_wd[i] >= ftol_wd2 && tst_freq_wd[i] <= ftol_wd3) {
            expect_wd_upper.push_back(1.12 * expect_wd[i]);
            expect_wd_lower.push_back(0.89 * expect_wd[i]);
        } else if (tst_freq_wd[i] > ftol_wd3 && tst_freq_wd[i] < ftol_wd4) {
            expect_wd_upper.push_back(1.26 * expect_wd[i]);
            expect_wd_lower.push_back(0.79 * expect_wd[i]);
        } else {
            expect_wd_upper.push_back(1.26 * expect_wd[i]);
            expect_wd_lower.push_back(0.00002);
        }
    }
    int rcode = 0;
    double dT = 0.001;

    size_t nFreqWd = tst_freq_wd.size();
    size_t nTvals = 1 + (1.0 / tst_freq_wd[0] * 10 / dT);

    ChMatrixDynamic<> sig_in_wd(nFreqWd, nTvals);

    std::vector<double> rms_in(nFreqWd);
    std::vector<double> rms_wd(nFreqWd);

    std::vector<double> t(nTvals);
    for (size_t j = 0; j < nTvals; j++) {
        t[j] = dT * double(j);
    }

    for (size_t i = 0; i < nFreqWd; i++) {
        for (size_t j = 0; j < nTvals; j++) {
            sig_in_wd(i, j) = sin(CH_C_2PI * tst_freq_wd[i] * t[j]);
        }
        rms_in[i] = rms(sig_in_wd, i);
    }

    ChISO2631_1_Wd wd(dT);
    for (size_t i = 0; i < nFreqWd; i++) {
        std::vector<double> sig_out;
        for (size_t j = 0; j < nTvals; j++) {
            sig_out.push_back(wd.Filter(sig_in_wd[i][j]));
        }
        rms_wd[i] = rms(sig_out) / rms_in[i];
        wd.Reset();
    }

    std::ofstream plt("filtertest_wd.plt");
    plt << "$FILTERWD << EOD" << std::endl;
    for (size_t i = 0; i < nFreqWd; i++) {
        plt << tst_freq_wd[i] << "\t" << expect_wd_upper[i] << "\t" << rms_wd[i] << "\t" << expect_wd_lower[i]
            << std::endl;
    }
    plt << "EOD" << std::endl;
    plt << "set title 'Test ISO2631 Wd Filter in Chrono'" << std::endl;
    plt << "set xlabel 'Frequency [Hz]'" << std::endl;
    plt << "set ylabel 'Weighting []'" << std::endl;
    plt << "set logscale xy" << std::endl;
    plt << "plot $FILTERWD t 'ISO2631 Wd Upper Limit' with lines, \\" << std::endl;
    plt << "     $FILTERWD u 1:3 t 'ISO2631 Wd Filtertest' with lines, \\" << std::endl;
    plt << "     $FILTERWD u 1:4 t 'ISO2631 Wd Lower Limits' with lines" << std::endl;
    plt.close();

    // Correction of the first and the last values of expect_wd_lower to zero (it could not be plotted in logscale!)
    expect_wd_lower.front() = 0.0;
    expect_wd_lower.back() = 0.0;
    int n_wd_errors = 0;
    for (size_t i = 0; i < nFreqWd; i++) {
        if (rms_wd[i] < expect_wd_lower[i] || rms_wd[i] > expect_wd_upper[i]) {
            n_wd_errors++;
        }
    }
    std::cout << "Errors Wd = " << n_wd_errors << std::endl;
    if (n_wd_errors > 0) {
        rcode_wd = 1;
    }

    return rcode_wd;
}

int test_iso2631_1_wf() {
    int rcode_wf = 0;

    // transtion frequencies for frequency weighting tolerances (ISO 8041)
    const double ftol_wf1 = pow(10.0, -13.0 / 10.0);
    const double ftol_wf2 = pow(10.0, -8.0 / 10.0);
    const double ftol_wf3 = pow(10.0, -4.0 / 10.0);
    const double ftol_wf4 = pow(10.0, 0.0 / 10.0);

    // expected frequncy weightings for filter Wf (ISO 2631-1)
    double expect_wf_data[] = {24.2e-3, 37.7e-3, 57.7e-3, 97.1e-3, 157e-3,  267e-3,  461e-3,  695e-3,
                               895e-3,  1006e-3, 992e-3,  854e-3,  619e-3,  384e-3,  224e-3,  116e-3,
                               53e-3,   23.5e-3, 9.98e-3, 3.77e-3, 1.55e-3, 0.64e-3, 0.25e-3, 0.097e-3};

    std::vector<double> expect_wf(expect_wf_data, expect_wf_data + sizeof(expect_wf_data) / sizeof(double));
    std::vector<double> expect_wf_upper;
    std::vector<double> expect_wf_lower;

    // test frequencies in one-third octaves
    std::vector<double> tst_freq_wf;
    for (int i = -17; i <= 6; i++) {
        double f = pow(10.0, double(i) / 10.0);
        tst_freq_wf.push_back(f);
    }
    if (tst_freq_wf.size() != expect_wf.size()) {
        std::cout << "ill configured Wf tables!" << std::endl;
    }

    // calculate tolerance limits
    for (int i = 0; i < tst_freq_wf.size(); i++) {
        if (tst_freq_wf[i] <= ftol_wf1) {
            expect_wf_upper.push_back(1.26 * expect_wf[i]);
            expect_wf_lower.push_back(0.00002);
        } else if (tst_freq_wf[i] > ftol_wf1 && tst_freq_wf[i] < ftol_wf2) {
            expect_wf_upper.push_back(1.26 * expect_wf[i]);
            expect_wf_lower.push_back(0.79 * expect_wf[i]);
        } else if (tst_freq_wf[i] >= ftol_wf2 && tst_freq_wf[i] <= ftol_wf3) {
            expect_wf_upper.push_back(1.12 * expect_wf[i]);
            expect_wf_lower.push_back(0.89 * expect_wf[i]);
        } else if (tst_freq_wf[i] > ftol_wf3 && tst_freq_wf[i] < ftol_wf4) {
            expect_wf_upper.push_back(1.26 * expect_wf[i]);
            expect_wf_lower.push_back(0.79 * expect_wf[i]);
        } else {
            expect_wf_upper.push_back(1.26 * expect_wf[i]);
            expect_wf_lower.push_back(0.00002);
        }
    }

    double dT = 0.1;

    size_t nFreqWf = tst_freq_wf.size();
    size_t nTvals = 1 + (1.0 / tst_freq_wf[0] * 10 / dT);

    ChMatrixDynamic<> sig_in_wf(nFreqWf, nTvals);

    std::vector<double> rms_in(nFreqWf);
    std::vector<double> rms_wf(nFreqWf);

    std::vector<double> t(nTvals);
    for (size_t j = 0; j < nTvals; j++) {
        t[j] = dT * double(j);
    }

    for (size_t i = 0; i < nFreqWf; i++) {
        for (size_t j = 0; j < nTvals; j++) {
            sig_in_wf(i, j) = sin(CH_C_2PI * tst_freq_wf[i] * t[j]);
        }
        rms_in[i] = rms(sig_in_wf, i);
    }

    ChISO2631_1_Wf wf(dT);
    for (size_t i = 0; i < nFreqWf; i++) {
        std::vector<double> sig_out;
        for (size_t j = 0; j < nTvals; j++) {
            sig_out.push_back(wf.Filter(sig_in_wf[i][j]));
        }
        rms_wf[i] = rms(sig_out) / rms_in[i];
        wf.Reset();
    }

    std::ofstream plt("filtertest_wf.plt");
    plt << "$FILTERWF << EOD" << std::endl;
    for (size_t i = 0; i < nFreqWf; i++) {
        plt << tst_freq_wf[i] << "\t" << expect_wf_upper[i] << "\t" << rms_wf[i] << "\t" << expect_wf_lower[i]
            << std::endl;
    }
    plt << "EOD" << std::endl;
    plt << "set title 'Test ISO2631 Wf Filter in Chrono'" << std::endl;
    plt << "set xlabel 'Frequency [Hz]'" << std::endl;
    plt << "set ylabel 'Weighting []'" << std::endl;
    plt << "set logscale xy" << std::endl;
    plt << "plot $FILTERWF t 'ISO2631 Wf Upper Limit' with lines, \\" << std::endl;
    plt << "     $FILTERWF u 1:3 t 'ISO2631 Wf Filtertest' with lines, \\" << std::endl;
    plt << "     $FILTERWF u 1:4 t 'ISO2631 Wf Lower Limits' with lines" << std::endl;
    plt.close();

    int n_wf_errors = 0;
    // Correction of the first and the last values of expect_wd_lower to zero (it could not be plotted in logscale!)
    expect_wf_lower.front() = 0.0;
    expect_wf_lower.back() = 0.0;
    for (size_t i = 0; i < nFreqWf; i++) {
        if (rms_wf[i] < expect_wf_lower[i] || rms_wf[i] > expect_wf_upper[i]) {
            if (rms_wf[i] > 0.01) {
                n_wf_errors++;
                std::cout << "Wf: Out of range at f = " << tst_freq_wf[i] << " Hz, Weighting = " << rms_wf[i]
                          << std::endl;
            } else {
                std::cout << "Wf: Acceptable deviation at f = " << tst_freq_wf[i] << " Hz, Weighting = " << rms_wf[i]
                          << std::endl;
            }
        }
    }

    std::cout << "Errors Wf = " << n_wf_errors << std::endl;
    if (n_wf_errors > 0) {
        rcode_wf = 1;
    }

    return rcode_wf;
}

int test_iso2631_5_wxy() {
    int rcode_wxy = 0;
    int n_wxy_errors = 0;
    const double fs = 160.0;
    const double ts = 1.0 / fs;
    const double Tlen = 2.0;

    std::vector<double> t;
    std::vector<double> sig_in_wxy;
    size_t nTvals = Tlen / ts + 1;
    t.resize(nTvals);
    sig_in_wxy.resize(nTvals);
    for (size_t i = 0; i < nTvals; i++) {
        t[i] = ts * double(i);
        if (t[i] <= 0.05) {
            sig_in_wxy[i] = 40.0 * t[i] + 0.0;
        } else if (t[i] > 0.05 && t[i] <= 0.2) {
            sig_in_wxy[i] = 0.0 * t[i] + 2.0;
        } else if (t[i] > 0.2 && t[i] <= 0.4) {
            sig_in_wxy[i] = -20.0 * t[i] + 6.0;
        } else if (t[i] > 0.4 && t[i] <= 0.5) {
            sig_in_wxy[i] = 0.0 * t[i] - 2.0;
        } else if (t[i] > 0.5 && t[i] <= 0.55) {
            sig_in_wxy[i] = 40.0 * t[i] - 22.0;
        } else if (t[i] > 0.55 && t[i] <= 2.0) {
            sig_in_wxy[i] = 0.0 * t[i] + 0.0;
        }
    }

    ChISO2631_5_Wxy filter_wxy;
    std::vector<double> resp_wxy;
    for (size_t i = 0; i < nTvals; i++) {
        resp_wxy.push_back(filter_wxy.Filter(sig_in_wxy[i]));
    }

    // tolerances from ISO 8041 +12%/-11%

    const double rMax1_exp = 3.025;
    const double rMax1_high = 1.12 * rMax1_exp;
    const double rMax1_low = 0.89 * rMax1_exp;
    size_t iMax1 = 0;
    double rMax1 = 0.0;

    const double rMin1_exp = -4.048;
    const double rMin1_high = rMin1_exp + 0.12 * fabs(rMin1_exp);
    const double rMin1_low = rMin1_exp - 0.11 * fabs(rMin1_exp);
    size_t iMin1 = 0;
    double rMin1 = 0.0;

    const double rMax2_exp = 2.065;
    const double rMax2_high = 1.12 * rMax2_exp;
    const double rMax2_low = 0.89 * rMax2_exp;
    size_t iMax2 = 0;
    double rMax2 = 0.0;

    for (size_t i = 0; i < nTvals - 1; i++) {
        if (resp_wxy[i] > rMax1) {
            rMax1 = resp_wxy[i];
        }
        if (resp_wxy[i + 1] < rMax1) {
            iMax1 = i;
            break;
        }
    }
    if (rMax1 > rMax1_high || rMax1 < rMax1_low) {
        n_wxy_errors++;
        std::cout << "Wxy: Out of Range Maximum 1 = " << rMax1 << std::endl;
    }

    rMin1 = rMax1;
    for (size_t i = iMax1; i < nTvals - 1; i++) {
        if (resp_wxy[i] < rMin1) {
            rMin1 = resp_wxy[i];
        }
        if (resp_wxy[i + 1] > rMin1) {
            iMin1 = i;
            break;
        }
    }
    if (rMin1 > rMin1_high || rMin1 < rMin1_low) {
        n_wxy_errors++;
        std::cout << "Wxy: Out of Range Minimum 1 = " << rMin1 << std::endl;
    }

    rMax2 = rMin1;
    for (size_t i = iMin1; i < nTvals - 1; i++) {
        if (resp_wxy[i] > rMax2) {
            rMax2 = resp_wxy[i];
        }
        if (resp_wxy[i + 1] < rMax2) {
            iMax2 = i;
            break;
        }
    }
    if (rMax2 > rMax2_high || rMax2 < rMax2_low) {
        n_wxy_errors++;
        std::cout << "Wxy: Out of Range Maximum 1 = " << rMax1 << std::endl;
    }

    std::ofstream plt("filtertest_wxy.plt");
    plt << "$FILTERWXY << EOD" << std::endl;
    for (size_t i = 0; i < nTvals; i++) {
        plt << t[i] << "\t" << sig_in_wxy[i] << "\t" << resp_wxy[i] << std::endl;
    }
    plt << "EOD" << std::endl;
    plt << "set title 'Test ISO2631-5 Wxy Filter in Chrono'" << std::endl;
    plt << "set xlabel 'Time [s]'" << std::endl;
    plt << "set ylabel 'Amplitude []'" << std::endl;
    plt << "plot $FILTERWXY t 'ISO2631-5 Wxy Test Signal' with lines, \\" << std::endl;
    plt << "     $FILTERWXY u 1:3 t 'ISO2631-5 Wxy Filter Response' with lines" << std::endl;
    plt.close();

    std::cout << "Errors Wxy = " << n_wxy_errors << std::endl;
    if (n_wxy_errors > 0) {
        rcode_wxy = 1;
    }
    return rcode_wxy;
}

int test_iso2631_5_wz() {
    int rcode_wz = 0;
    int n_wz_errors = 0;

    const double fs = 160.0;
    const double ts = 1.0 / fs;
    const double Tlen = 2.0;

    std::vector<double> t;
    std::vector<double> sig_in_wz;
    size_t nTvals = Tlen / ts + 1;
    t.resize(nTvals);
    sig_in_wz.resize(nTvals);
    for (size_t i = 0; i < nTvals; i++) {
        t[i] = ts * double(i);
        if (t[i] <= 0.05) {
            sig_in_wz[i] = 40.0 * t[i] + 0.0;
        } else if (t[i] > 0.05 && t[i] <= 0.2) {
            sig_in_wz[i] = 0.0 * t[i] + 2.0;
        } else if (t[i] > 0.2 && t[i] <= 0.4) {
            sig_in_wz[i] = -20.0 * t[i] + 6.0;
        } else if (t[i] > 0.4 && t[i] <= 0.5) {
            sig_in_wz[i] = 0.0 * t[i] - 2.0;
        } else if (t[i] > 0.5 && t[i] <= 0.55) {
            sig_in_wz[i] = 40.0 * t[i] - 22.0;
        } else if (t[i] > 0.55 && t[i] <= 2.0) {
            sig_in_wz[i] = 0.0 * t[i] + 0.0;
        }
    }

    std::vector<double> resp_wz;

    ChISO2631_5_Wz filter_wz;
    filter_wz.Filter(sig_in_wz, resp_wz);

    const double rMax1_exp = 1.661;
    const double rMax1_high = rMax1_exp + 0.11 * rMax1_exp;
    const double rMax1_low = rMax1_exp - 0.11 * rMax1_exp;
    size_t iMax1 = 0;
    double rMax1 = 0.0;

    const double rMin1_exp = -1.512;
    const double rMin1_high = rMin1_exp + 0.11 * fabs(rMin1_exp);
    const double rMin1_low = rMin1_exp - 0.11 * fabs(rMin1_exp);
    size_t iMin1 = 0;
    double rMin1 = 0.0;

    const double rMax2_exp = 0.308;
    const double rMax2_high = rMax2_exp + 0.11 * rMax2_exp;
    const double rMax2_low = rMax2_exp - 0.11 * rMax2_exp;
    size_t iMax2 = 0;
    double rMax2 = 0.0;

    for (size_t i = 0; i < nTvals - 1; i++) {
        if (resp_wz[i] > rMax1 && resp_wz[i] > 0.0) {
            rMax1 = resp_wz[i];
        }
        if (resp_wz[i + 1] < rMax1 && resp_wz[i] > 0.0) {
            iMax1 = i;
            break;
        }
    }
    if (rMax1 > rMax1_high || rMax1 < rMax1_low) {
        n_wz_errors++;
        std::cout << "Wz: Out of Range Maximum 1 = " << rMax1 << std::endl;
    }

    rMin1 = rMax1;
    for (size_t i = iMax1; i < nTvals - 1; i++) {
        if (resp_wz[i] < rMin1) {
            rMin1 = resp_wz[i];
        }
        if (resp_wz[i + 1] > rMin1 && resp_wz[i] < 0.0) {
            iMin1 = i;
            break;
        }
    }
    if (rMin1 > rMin1_high || rMin1 < rMin1_low) {
        n_wz_errors++;
        std::cout << "Wz: Out of Range Minimum 1 = " << rMin1 << std::endl;
    }

    rMax2 = rMin1;
    for (size_t i = iMin1; i < nTvals - 1; i++) {
        if (resp_wz[i] > rMax2) {
            rMax2 = resp_wz[i];
        }
        if (resp_wz[i + 1] < rMax2) {
            iMax2 = i;
            break;
        }
    }
    if (rMax2 > rMax2_high || rMax2 < rMax2_low) {
        n_wz_errors++;
        std::cout << "Wz: Out of Range Maximum 2 = " << rMax2 << std::endl;
    }

    std::ofstream plt("filtertest_wz.plt");
    plt << "$FILTERWZ << EOD" << std::endl;
    for (size_t i = 0; i < nTvals; i++) {
        plt << t[i] << "\t" << sig_in_wz[i] << "\t" << resp_wz[i] << std::endl;
    }
    plt << "EOD" << std::endl;
    plt << "set title 'Test ISO2631-5 Wz Filter in Chrono'" << std::endl;
    plt << "set xlabel 'Time [s]'" << std::endl;
    plt << "set ylabel 'Amplitude []'" << std::endl;
    plt << "plot $FILTERWZ t 'ISO2631-5 Wz Test Signal' with lines, \\" << std::endl;
    plt << "     $FILTERWZ u 1:3 t 'ISO2631-5 Wz Filter Response' with lines" << std::endl;
    plt.close();

    std::cout << "Errors Wz = " << n_wz_errors << std::endl;
    return rcode_wz;
}

int main(int argc, char* argv[]) {
    int rcode = 0;
    int res_wk = test_iso2631_1_wk();
    int res_wd = test_iso2631_1_wd();
    int res_wf = test_iso2631_1_wf();
    int res_wxy = test_iso2631_5_wxy();
    int res_wz = test_iso2631_5_wz();

    if (res_wk > 0 || res_wd > 0 || res_wf > 0 || res_wxy > 0 || res_wz > 0) {
        rcode = 1;
    }
    return rcode;
}
