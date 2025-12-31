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

#include "gtest/gtest.h"
#include "chrono/utils/ChFilters.h"
#include "chrono/utils/ChConstants.h"

using namespace chrono;

// Test classes for shock signal weighting filters ISO 2631-5, error limits from ISO 8041

class ShockTestbed {
  public:
    ShockTestbed();
    double Test1();
    double Test2();
    double Test3();

  protected:
    double m_tDuration;  // duration of the complete testsignal
    double m_fSample;    // sample rate of input
    std::vector<double> input_signal;
    std::vector<double> output_signal;
};

ShockTestbed::ShockTestbed() : m_tDuration(2), m_fSample(160) {
    size_t nPoints = (size_t)(m_tDuration * m_fSample + 1);
    input_signal.resize(nPoints, 0.0);
    output_signal.resize(nPoints, 0.0);
    for (size_t i = 0; i < input_signal.size(); i++) {
        double t = double(i) / m_fSample;
        if (t <= 0.05) {
            input_signal[i] = 40.0 * t + 0.0;
        } else if (t > 0.05 && t <= 0.2) {
            input_signal[i] = 0.0 * t + 2.0;
        } else if (t > 0.2 && t <= 0.4) {
            input_signal[i] = -20.0 * t + 6.0;
        } else if (t > 0.4 && t <= 0.5) {
            input_signal[i] = 0.0 * t - 2.0;
        } else if (t > 0.5 && t <= 0.55) {
            input_signal[i] = 40.0 * t - 22.0;
        } else if (t > 0.55 && t <= 2.0) {
            input_signal[i] = 0.0 * t + 0.0;
        }
    }
}

double ShockTestbed::Test1() {
    // Search the global maximum
    double r = output_signal[0];
    for (size_t i = 1; i < output_signal.size(); i++) {
        if (output_signal[i] > r) {
            r = output_signal[i];
        }
    }
    return r;
}

double ShockTestbed::Test2() {
    // Search the global minimum
    double r = output_signal[0];
    for (size_t i = 1; i < output_signal.size(); i++) {
        if (output_signal[i] < r) {
            r = output_signal[i];
        }
    }
    return r;
}

double ShockTestbed::Test3() {
    // Search the second maximum
    size_t istart = (size_t)(0.5 * m_fSample);
    double r = output_signal[istart];
    for (size_t i = istart + 1; i < output_signal.size(); i++) {
        if (output_signal[i] > r) {
            r = output_signal[i];
        }
    }
    return r;
}

class ShockTestWxy : public ShockTestbed {
  public:
    ShockTestWxy();

  private:
    chrono::utils::ChISO2631_5_Wxy filter;
};

ShockTestWxy::ShockTestWxy() {
    std::ofstream shock("shock.txt");
    for (size_t i = 0; i < input_signal.size(); i++) {
        double t = double(i) / m_fSample;
        output_signal[i] = filter.Filter(input_signal[i]);
        shock << t << "\t" << input_signal[i] << "\t" << output_signal[i] << std::endl;
    }
    shock.close();
}

class ShockTestWz : public ShockTestbed {
  public:
    ShockTestWz();

  private:
    chrono::utils::ChISO2631_5_Wz filter;
};

ShockTestWz::ShockTestWz() {
    filter.Filter(input_signal, output_signal);
}

// Test classes for vibration weighting filters ISO 2631-1
// Actually considered: Whole Body Vibration (Bandfilter 0.4 ... 100 Hz, Wk, Wd)
// Sawtooth Burst input signals and error limits taken from ISO 8041

class SawtoothTestbed {
  public:
    SawtoothTestbed(size_t nSawTeeth);
    virtual double Test() { return 0; };

  protected:
    unsigned int m_Nsaw;  // # of periods of sawtooth
    double m_wSaw;        // angular frequency of the sawtooth
    double m_aSaw;        // period length of sawtooth
    double m_tDuration;   // duration of the complete testsignal
    double m_fSample;     // sample rate of input
    std::vector<double> input_signal;
    std::vector<double> output_signal;
    double rms();

  private:
    double sawtooth(double t_ofs, double t);
};

SawtoothTestbed::SawtoothTestbed(size_t nSawTeeth) : m_wSaw(100), m_tDuration(60), m_fSample(1000) {
    switch (nSawTeeth) {
        case 1:
            m_Nsaw = 1;
            break;
        case 2:
            m_Nsaw = 2;
            break;
        case 4:
            m_Nsaw = 4;
            break;
        case 8:
            m_Nsaw = 8;
            break;
        case 16:
            m_Nsaw = 16;
            break;
        default:
            m_Nsaw = 0;  // continous sawtooth signal
    }
    m_aSaw = CH_2PI / m_wSaw;
    size_t nPoints = (size_t)(m_tDuration * m_fSample + 1);
    input_signal.resize(nPoints, 0.0);
    output_signal.resize(nPoints, 0.0);
    const double t_ofs1 = 1.0;
    const double t_ofs_rep = 10.0;
    for (size_t i = 0; i < input_signal.size(); i++) {
        double t = double(i) / m_fSample;
        if (m_Nsaw > 0) {
            input_signal[i] = sawtooth(t_ofs1, t) + sawtooth(t_ofs1 + t_ofs_rep, t) +
                              sawtooth(t_ofs1 + 2.0 * t_ofs_rep, t) + sawtooth(t_ofs1 + 3.0 * t_ofs_rep, t) +
                              sawtooth(t_ofs1 + 4.0 * t_ofs_rep, t) + sawtooth(t_ofs1 + 5.0 * t_ofs_rep, t);
        } else {
            input_signal[i] = sawtooth(t_ofs1, t);
        }
    }
}

double SawtoothTestbed::rms() {
    double r = 0.0;
    for (size_t i = 0; i < output_signal.size(); i++) {
        double e = output_signal[i];
        r += e * e;
    }
    return std::sqrt(r / double(output_signal.size()));
}

double SawtoothTestbed::sawtooth(double t_ofs, double t) {
    double y = 0;
    if (m_Nsaw > 0) {
        if ((t >= t_ofs) && (t <= t_ofs + m_Nsaw * m_aSaw)) {
            y = 2.0 * ((t - t_ofs) / m_aSaw - floor(0.5 + (t - t_ofs) / m_aSaw));
        }
    } else {
        y = 2.0 * ((t - t_ofs) / m_aSaw - floor(0.5 + (t - t_ofs) / m_aSaw));
    }
    return y;
}

class SawtoothTestBandfilter : public SawtoothTestbed {
  public:
    SawtoothTestBandfilter(size_t nSawTeeth);
    double Test();

  private:
    chrono::utils::ChButterworthHighpass hp;
    chrono::utils::ChButterworthLowpass lp;
};

SawtoothTestBandfilter::SawtoothTestBandfilter(size_t nSawTeeth) : SawtoothTestbed(nSawTeeth) {
    hp.Config(2, 1.0 / m_fSample, 0.4);
    lp.Config(2, 1.0 / m_fSample, 100.0);
}

double SawtoothTestBandfilter::Test() {
    for (size_t i = 0; i < input_signal.size(); i++) {
        double y = hp.Filter(input_signal[i]);
        output_signal[i] = lp.Filter(y);
    }
    return rms();
}

class SawtoothTestWkfilter : public SawtoothTestbed {
  public:
    SawtoothTestWkfilter(size_t nSawTeeth);
    void GenerateOutput();
    double Test();

  private:
    chrono::utils::ChISO2631_1_Wk wk;
};

SawtoothTestWkfilter::SawtoothTestWkfilter(size_t nSawTeeth) : SawtoothTestbed(nSawTeeth) {
    wk.Config(1.0 / m_fSample);
}

double SawtoothTestWkfilter::Test() {
    for (size_t i = 0; i < input_signal.size(); i++) {
        output_signal[i] = wk.Filter(input_signal[i]);
    }
    return rms();
}

class SawtoothTestWdfilter : public SawtoothTestbed {
  public:
    SawtoothTestWdfilter(size_t nSawTeeth);
    double Test();

  private:
    chrono::utils::ChISO2631_1_Wd wd;
};

SawtoothTestWdfilter::SawtoothTestWdfilter(size_t nSawTeeth) : SawtoothTestbed(nSawTeeth) {
    wd.Config(1.0 / m_fSample);
}

double SawtoothTestWdfilter::Test() {
    for (size_t i = 0; i < input_signal.size(); i++) {
        output_signal[i] = wd.Filter(input_signal[i]);
    }
    return rms();
}

TEST(WholeBodyBandfilter, SawtoothBurstRMS) {
    const double band_res_1 = 0.0433;
    const double band_res_1_err = 0.1 * band_res_1;
    SawtoothTestBandfilter bf_1(1);
    ASSERT_NEAR(bf_1.Test(), band_res_1, band_res_1_err);

    const double band_res_2 = 0.0612;
    const double band_res_2_err = 0.1 * band_res_2;
    SawtoothTestBandfilter bf_2(2);
    ASSERT_NEAR(bf_2.Test(), band_res_2, band_res_2_err);

    const double band_res_4 = 0.0865;
    const double band_res_4_err = 0.1 * band_res_4;
    SawtoothTestBandfilter bf_4(4);
    ASSERT_NEAR(bf_4.Test(), band_res_4, band_res_4_err);

    const double band_res_8 = 0.122;
    const double band_res_8_err = 0.1 * band_res_8;
    SawtoothTestBandfilter bf_8(8);
    ASSERT_NEAR(bf_8.Test(), band_res_8, band_res_8_err);

    const double band_res_16 = 0.173;
    const double band_res_16_err = 0.1 * band_res_16;
    SawtoothTestBandfilter bf_16(16);
    ASSERT_NEAR(bf_16.Test(), band_res_16, band_res_16_err);

    const double band_res_c = 0.546;
    const double band_res_c_err = 0.1 * band_res_c;
    SawtoothTestBandfilter bf_c(0);
    ASSERT_NEAR(bf_c.Test(), band_res_c, band_res_c_err);
}

TEST(WholeBodyWkFilter, SawtoothBurstRMS) {
    const double wk_res_1 = 0.0299;
    const double wk_res_1_err = 0.1 * wk_res_1;
    SawtoothTestWkfilter wk_1(1);
    ASSERT_NEAR(wk_1.Test(), wk_res_1, wk_res_1_err);

    const double wk_res_2 = 0.0411;
    const double wk_res_2_err = 0.1 * wk_res_2;
    SawtoothTestWkfilter wk_2(2);
    ASSERT_NEAR(wk_2.Test(), wk_res_2, wk_res_2_err);

    const double wk_res_4 = 0.0577;
    const double wk_res_4_err = 0.1 * wk_res_4;
    SawtoothTestWkfilter wk_4(4);
    ASSERT_NEAR(wk_4.Test(), wk_res_4, wk_res_4_err);

    const double wk_res_8 = 0.0814;
    const double wk_res_8_err = 0.1 * wk_res_8;
    SawtoothTestWkfilter wk_8(8);
    ASSERT_NEAR(wk_8.Test(), wk_res_8, wk_res_8_err);

    const double wk_res_16 = 0.115;
    const double wk_res_16_err = 0.1 * wk_res_16;
    SawtoothTestWkfilter wk_16(16);
    ASSERT_NEAR(wk_16.Test(), wk_res_16, wk_res_16_err);

    const double wk_res_c = 0.362;
    const double wk_res_c_err = 0.1 * wk_res_c;
    SawtoothTestWkfilter wk_c(0);
    ASSERT_NEAR(wk_c.Test(), wk_res_c, wk_res_c_err);
}

TEST(WholeBodyWdFilter, SawtoothBurstRMS) {
    const double wd_res_1 = 0.00669;
    const double wd_res_1_err = 0.1 * wd_res_1;
    SawtoothTestWdfilter wd_1(1);
    ASSERT_NEAR(wd_1.Test(), wd_res_1, wd_res_1_err);

    const double wd_res_2 = 0.00906;
    const double wd_res_2_err = 0.1 * wd_res_2;
    SawtoothTestWdfilter wd_2(2);
    ASSERT_NEAR(wd_2.Test(), wd_res_2, wd_res_2_err);

    const double wd_res_4 = 0.0116;
    const double wd_res_4_err = 0.1 * wd_res_4;
    SawtoothTestWdfilter wd_4(4);
    ASSERT_NEAR(wd_4.Test(), wd_res_4, wd_res_4_err);

    const double wd_res_8 = 0.0148;
    const double wd_res_8_err = 0.1 * wd_res_8;
    SawtoothTestWdfilter wd_8(8);
    ASSERT_NEAR(wd_8.Test(), wd_res_8, wd_res_8_err);

    const double wd_res_16 = 0.0197;
    const double wd_res_16_err = 0.1 * wd_res_16;
    SawtoothTestWdfilter wd_16(16);
    ASSERT_NEAR(wd_16.Test(), wd_res_16, wd_res_16_err);

    const double wd_res_c = 0.059;
    const double wd_res_c_err = 0.1 * wd_res_c;
    SawtoothTestWdfilter wd_c(0);
    ASSERT_NEAR(wd_c.Test(), wd_res_c, wd_res_c_err);
}

TEST(ShockWxyFilter, StandardSignal) {
    ShockTestWxy wxy;

    const double wxy_res_1 = 3.025;
    const double wxy_res_1_err = 0.1 * wxy_res_1;
    ASSERT_NEAR(wxy.Test1(), wxy_res_1, wxy_res_1_err);

    const double wxy_res_2 = -4.048;
    const double wxy_res_2_err = std::abs(0.1 * wxy_res_2);
    ASSERT_NEAR(wxy.Test2(), wxy_res_2, wxy_res_2_err);

    const double wxy_res_3 = 2.065;
    const double wxy_res_3_err = std::abs(0.1 * wxy_res_3);
    ASSERT_NEAR(wxy.Test3(), wxy_res_3, wxy_res_3_err);
}

TEST(ShockWzFilter, StandardSignal) {
    ShockTestWz wz;

    const double wz_res_1 = 1.661;
    const double wz_res_1_err = 0.1 * wz_res_1;
    ASSERT_NEAR(wz.Test1(), wz_res_1, wz_res_1_err);

    const double wz_res_2 = -1.512;
    const double wz_res_2_err = std::abs(0.1 * wz_res_2);
    ASSERT_NEAR(wz.Test2(), wz_res_2, wz_res_2_err);

    const double wz_res_3 = 0.308;
    const double wz_res_3_err = std::abs(0.1 * wz_res_3);
    ASSERT_NEAR(wz.Test3(), wz_res_3, wz_res_3_err);
}