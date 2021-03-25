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

#ifndef CH_FILTERS_H
#define CH_FILTERS_H

#include <cmath>
#include <valarray>
#include <vector>

#include <chrono/motion_functions/ChFunction_Recorder.h>
#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChMathematics.h"
#include "chrono/core/ChVector.h"

namespace chrono {

/// Chrono core utilities
namespace utils {

/// @addtogroup chrono_utils
/// @{

/// Moving average filter for smoothing running data.
class ChApi ChRunningAverage {
  public:
    /// Construct a running moving average filter (backward).
    /// The filter only uses previous data (as specified by the filter span)
    ChRunningAverage(int n  ///< filter span
    );

    ~ChRunningAverage() {}

    /// Insert the specified value and return the current moving average.
    double Add(double val);

    /// Return the standard deviation of data in current filter span.
    double GetStdDev() const { return m_std; }

    /// Reset the filter.
    void Reset();

  private:
    int m_n;
    int m_index;
    double m_std;
    std::valarray<double> m_data;
};

/// Moving average filter for smoothing a data array.
class ChApi ChMovingAverage {
  public:
    /// Construct a moving average filter (centered).
    /// The average is calculated over 2*n+1 points
    ChMovingAverage(const std::valarray<double>& data,  ///< input data
                    int n                               ///< filter half-span
    );

    ~ChMovingAverage() {}

    /// Return the filtered output.
    const std::valarray<double>& Get() const { return m_out; }

    /// Return the specified component of the filtered output.
    double Get(int i) const { return m_out[i]; }

  private:
    std::valarray<double> m_out;
};

/// Base class for simulated analogue filters in the time domain.
class ChApi ChAnalogueFilter {
  public:
    ChAnalogueFilter() {}
    virtual ~ChAnalogueFilter() {}
    virtual void Reset() = 0;
    virtual double Filter(double u) = 0;

  protected:
    double m_step;   // solver step width
    double m_u_old;  // last input value
    double m_y_old;  // last output value
};

/// Calculate the integral of an input signal in the time domain:
/// H(s) = 1 / ( Ti * s)
class ChApi ChFilterI : public ChAnalogueFilter {
  public:
    ChFilterI() {}
    ChFilterI(double step, double Ti = 1.0);
    ~ChFilterI() {}
    virtual void Reset() override;
    void Config(double step, double Ti = 1.0);
    virtual double Filter(double u) override;

  private:
    double m_Ti;
};

/// Caclulate the time derivation of an input signal:
/// H(s) = Td * s
class ChApi ChFilterD : public ChAnalogueFilter {
  public:
    ChFilterD() {}
    ChFilterD(double step, double Td = 1.0);
    ~ChFilterD() {}
    virtual void Reset() override;
    void Config(double step, double Td = 1.0);
    virtual double Filter(double u) override;

  private:
    double m_Td;
};

/// Delay an input signal:
/// H(s) = Kpt1 / ( T1 * s + 1 )
class ChApi ChFilterPT1 : public ChAnalogueFilter {
  public:
    ChFilterPT1() {}
    ChFilterPT1(double step, double T1 = 1.0, double Kpt1 = 1.0);
    ~ChFilterPT1() {}
    virtual void Reset() override;
    void Config(double step, double T1 = 1.0, double Kpt1 = 1.0);
    virtual double Filter(double u) override;

  private:
    double m_T1;
    double m_Kpt1;
};

/// PD1 controller:
/// H(s) = Kdt1 * ( Td1 * s + 1 )
class ChApi ChFilterPD1 : public ChAnalogueFilter {
  public:
    ChFilterPD1() {}
    ChFilterPD1(double step, double Td1 = 1.0, double Kdt1 = 1.0);
    ~ChFilterPD1() {}
    virtual void Reset() override;
    void Config(double step, double Td1 = 1.0, double Kdt1 = 1.0);
    virtual double Filter(double u) override;

  private:
    double m_Td1;
    double m_Kdt1;
};

/// PDT1 controller:
/// H(s) = Kp * ( Td1 * s + 1 ) / ( T1 * s + 1)
class ChApi ChFilterPDT1 : public ChAnalogueFilter {
  public:
    ChFilterPDT1() {}
    ChFilterPDT1(double step, double Td1 = 1.0, double T1 = 1.0, double Kp = 1.0);
    ~ChFilterPDT1() {}
    virtual void Reset() override;
    void Config(double step, double Td1 = 1.0, double T1 = 1.0, double Kp = 1.0);
    virtual double Filter(double u) override;

  private:
    ChFilterPD1 pd1;
    ChFilterPT1 pt1;
};

// Collection of useful recursive filters (IIR) for signal processing.
// For the sake of stability and numerical efficiency they are based
// on the bilinear transform (Tustin method)

/// Butterworth low-pass filter.
class ChApi ChButterworth_Lowpass {
  public:
    ChButterworth_Lowpass();
    ChButterworth_Lowpass(unsigned int nPoles, double step, double fc);
    void Reset();
    void Config(unsigned int nPoles, double step, double fc);
    double Filter(double u);

  private:
    double m_Ts;
    unsigned int m_n_single;
    unsigned int m_n_biquad;

    // Coefficients for a possible single pole lowpass
    double m_b0, m_b1;
    double m_a0, m_a1;

    // state buffers for a possible single pole lowpass
    double m_u_hist1;
    double m_y_hist1;

    std::vector<double> m_Q;

    // Coefficients for possible biquad lowpasses
    std::vector<double> m_biq_b0, m_biq_b1, m_biq_b2;
    std::vector<double> m_biq_a0, m_biq_a1, m_biq_a2;

    // state buffers for a possible biquad lowpasses
    std::vector<double> m_biq_u_hist1, m_biq_u_hist2;
    std::vector<double> m_biq_y_hist1, m_biq_y_hist2;
};

/// Butterworth high-pass filter.
class ChApi ChButterworth_Highpass {
  public:
    ChButterworth_Highpass();
    ChButterworth_Highpass(unsigned int nPoles, double step, double fc);
    void Reset();
    void Config(unsigned int nPoles, double step, double fc);
    double Filter(double u);

  private:
    double m_Ts;
    unsigned int m_n_single;
    unsigned int m_n_biquad;

    // Coefficients for a possible single pole lowpass
    double m_b0, m_b1;
    double m_a0, m_a1;

    // state buffers for a possible single pole lowpass
    double m_u_hist1;
    double m_y_hist1;

    std::vector<double> m_Q;

    // Coefficients for possible biquad lowpasses
    std::vector<double> m_biq_b0, m_biq_b1, m_biq_b2;
    std::vector<double> m_biq_a0, m_biq_a1, m_biq_a2;

    // state buffers for a possible biquad lowpasses
    std::vector<double> m_biq_u_hist1, m_biq_u_hist2;
    std::vector<double> m_biq_y_hist1, m_biq_y_hist2;
};

/// Filter for vertical absorbed power.
class ChApi ChAbsorbed_Power_Vertical {
  public:
    ChAbsorbed_Power_Vertical();
    ChAbsorbed_Power_Vertical(double step);
    void Reset();
    void Config(double step);
    double Filter(double u);

  private:
    double m_Ts;

    // digital filter coefficients
    double m_b0, m_b1, m_b2, m_b3;
    double m_a0, m_a1, m_a2, m_a3;

    // history buffers
    double m_u_hist1, m_u_hist2, m_u_hist3;
    double m_y_hist1, m_y_hist2, m_y_hist3;
};

class ChApi ChISO2631_1_AVTransition {
  public:
    ChISO2631_1_AVTransition();
    ChISO2631_1_AVTransition(double step, double f4, double Q4);
    ChISO2631_1_AVTransition(double step, double f3, double f4, double Q4);
    void Config(double step, double f4, double Q4);
    void Config(double step, double f3, double f4, double Q4);
    void Reset();
    double Filter(double u);

  private:
    double m_Ts;
    double m_wc3;
    double m_wc4;
    double m_Q4;

    double m_b0, m_b1, m_b2;
    double m_a0, m_a1, m_a2;

    double m_u_hist1, m_u_hist2;
    double m_y_hist1, m_y_hist2;
};

class ChApi ChISO2631_1_UpwardStep {
  public:
    ChISO2631_1_UpwardStep();
    ChISO2631_1_UpwardStep(double step, double f5, double f6, double Q5, double Q6);
    void Config(double step, double f5, double f6, double Q5, double Q6);
    void Reset();
    double Filter(double u);

  private:
    double m_Ts;
    double m_wc5;
    double m_wc6;
    double m_Q5;
    double m_Q6;

    double m_b0, m_b1, m_b2;
    double m_a0, m_a1, m_a2;

    double m_u_hist1, m_u_hist2;
    double m_y_hist1, m_y_hist2;
};

/// Combined filter Wk.
class ChApi ChISO2631_1_Wk {
  public:
    ChISO2631_1_Wk();
    ChISO2631_1_Wk(double step);
    void Config(double step);
    void Reset();
    double Filter(double u);

  private:
    static const double f1;
    static const double f2;
    static const double f3;
    static const double f4;
    static const double f5;
    static const double f6;
    static const double Q4;
    static const double Q5;
    static const double Q6;

    ChButterworth_Highpass hp;
    ChButterworth_Lowpass lp;
    ChISO2631_1_AVTransition avt;
    ChISO2631_1_UpwardStep ups;
};

/// Combined filter Wd.
class ChApi ChISO2631_1_Wd {
  public:
    ChISO2631_1_Wd();
    ChISO2631_1_Wd(double step);
    void Config(double step);
    void Reset();
    double Filter(double u);

  private:
    static const double f1;
    static const double f2;
    static const double f3;
    static const double f4;
    static const double Q4;

    ChButterworth_Highpass hp;
    ChButterworth_Lowpass lp;
    ChISO2631_1_AVTransition avt;
};

/// Combined filter Wf.
class ChApi ChISO2631_1_Wf {
  public:
    ChISO2631_1_Wf();
    ChISO2631_1_Wf(double step);
    void Config(double step);
    void Reset();
    double Filter(double u);

  private:
    static const double f1;
    static const double f2;
    static const double f4;
    static const double f5;
    static const double f6;
    static const double Q4;
    static const double Q5;
    static const double Q6;

    ChButterworth_Highpass hp;
    ChButterworth_Lowpass lp;
    ChISO2631_1_AVTransition avt;
    ChISO2631_1_UpwardStep ups;
};

/// ISO2631-5 weighting filter for shock like signal in horizontal direction. 
/// Works with sample rate = 160 Hz only.
class ChApi ChISO2631_5_Wxy {
  public:
    ChISO2631_5_Wxy();
    void Reset();
    double Filter(double u);

  private:
    static const double m_step;

    // discrete filter coefficients
    static const double m_b0;
    static const double m_b1;
    static const double m_b2;
    static const double m_a0;
    static const double m_a1;
    static const double m_a2;

    // data history
    double m_u_hist1, m_u_hist2;
    double m_y_hist1, m_y_hist2;
};

/// ISO2631-5 weighting filter for shock like signal in vertical direction.
/// Unlike ChISO2631_5_Wxy this filter is nonlinear and works with a sample rate of fs = 160 Hz (step = 1/160 s) only.
/// unfiltered input u -> filtered output y
class ChApi ChISO2631_5_Wz {
  public:
    ChISO2631_5_Wz();
    void Filter(std::vector<double>& u, std::vector<double>& y);

  private:
    static const double m_w[13][7];
    static const double m_W[8];
};

/// Easy to use class for evaluation of ISO 2361-1 vibration load on sitting vehicle occupants
/// Input: 3 seat accelerations x,y,z in [m/s^2].
/// Sample rate should be >= 250 Hz (i.e., step <= 1/250 s).
class ChApi ChISO2631_Vibration_SeatCushionLogger {
  public:
    ChISO2631_Vibration_SeatCushionLogger();
    ChISO2631_Vibration_SeatCushionLogger(double step);
    void Config(double step);
    void AddData(double speed, double acc_x, double acc_y, double acc_z);
    void AddData(double speed, ChVector<>& acc_v) { AddData(speed, acc_v.x(), acc_v.y(), acc_v.z()); }
    void Reset();
    double GetExposureTime() const { return m_logging_time; }
    double GetInputRMS_X() const;
    double GetInputRMS_Y() const;
    double GetInputRMS_Z() const;
    double GetAW_X() const;
    double GetAW_Y() const;
    double GetAW_Z() const;
    double GetAW_V() const;
    double GetCrestFactor() const;
    double GetVDV() const;
    double GetAVGSpeed() const;
    double GetSeverityVDV() const;
    double GetAbsorbedPowerVertical();
    void GeneratePlotFile(std::string fName, std::string testInfo);

  private:
    // running time of data loging
    const double m_tstart1 = 0.2;
    const double m_tstart2 = 0.5;
    double m_logging_time;
    double m_step;

    // raw input data series
    std::vector<double> m_data_speed;

    std::vector<double> m_data_acc_x;
    std::vector<double> m_data_acc_y;
    std::vector<double> m_data_acc_z;

    std::vector<double> m_data_acc_ap_z;  // vertical acceleration in ft/s^2 for absorbed power calculation

    // freqency weighted data series
    std::vector<double> m_data_acc_x_wd;
    std::vector<double> m_data_acc_y_wd;
    std::vector<double> m_data_acc_z_wk;

    // integrated squared freqency weighted data series
    std::vector<double> m_data_aw_x_i;
    std::vector<double> m_data_aw_y_i;
    std::vector<double> m_data_aw_z_i;

    // integrated vibration dose values data series
    std::vector<double> m_data_vdv_x_i;
    std::vector<double> m_data_vdv_y_i;
    std::vector<double> m_data_vdv_z_i;

    // integral averaged freqency weighted data series
    std::vector<double> m_data_aw_x_avg;
    std::vector<double> m_data_aw_y_avg;
    std::vector<double> m_data_aw_z_avg;

    // integral averaged vdv data series
    std::vector<double> m_data_vdv_x_avg;
    std::vector<double> m_data_vdv_y_avg;
    std::vector<double> m_data_vdv_z_avg;

    // filter classes for frequency weighting
    ChISO2631_1_Wd m_filter_wd_x;
    ChISO2631_1_Wd m_filter_wd_y;
    ChISO2631_1_Wk m_filter_wk_z;

    ChAbsorbed_Power_Vertical m_filter_abspow;

    // filter classes for time integral aw
    ChFilterI m_filter_int_aw_x;
    ChFilterI m_filter_int_aw_y;
    ChFilterI m_filter_int_aw_z;

    // filter classes for time integral vdv
    ChFilterI m_filter_int_vdv_x;
    ChFilterI m_filter_int_vdv_y;
    ChFilterI m_filter_int_vdv_z;
};

/// Easy to use class for evaluation of ISO 2361-5 shock load on sitting vehicle occupants.
/// Input: 3 seat accelerations x,y,z in [m/s^2].
/// Sample rate of the input signal should be >= 250 Hz (i.e., step <= 1/250 s).
/// Internal filtering works with 160 Hz sample rate as demanded by ISO 2631-5.
/// Downsampling is done automatically antialiasing included
class ChApi ChISO2631_Shock_SeatCushionLogger {
  public:
    ChISO2631_Shock_SeatCushionLogger();
    ChISO2631_Shock_SeatCushionLogger(double step);
    void Config(double step);
    void AddData(double acc_x, double acc_y, double acc_z);
    void AddData(ChVector<>& acc_v) { AddData(acc_v.x(), acc_v.y(), acc_v.z()); }
    void Reset();
    // Se = equivalent static spine compressive stress [MPa]
    // Se < 0.5 MPa : low risk of severe health effect
    // Se > 0.8 Mpa : high risk of severe health effect
    double GetSe();
    // Legacy Method for Military Vehicles from NRMM
    // Calculates the max. Value of Az filtered by 30 Hz Butterworth Lowpass
    // Result should not exceed 2.5 g
    double GetLegacyAz();

  private:
    // time step = 1/fs for the input data
    double m_step_inp;
    // running time of data loging
    const double m_tstart1 = 0.2;
    const double m_tstart2 = 0.5;
    const double m_step = 1.0 / 160.0;

    double m_logging_time;

    // results
    double m_dkx;
    double m_dky;
    double m_dkz;

    const double m_mx = 0.015;
    const double m_my = 0.035;
    const double m_mz = 0.032;

    // antialiasing filters
    ChButterworth_Lowpass m_lpx;
    ChButterworth_Lowpass m_lpy;
    ChButterworth_Lowpass m_lpz;

    // legacy lowpass
    ChButterworth_Lowpass m_legacy_lpz;

    // buffers for raw but antialiased input data
    ChFunction_Recorder m_raw_inp_x;
    ChFunction_Recorder m_raw_inp_y;
    ChFunction_Recorder m_raw_inp_z;

    // buffers for resampled input data
    std::vector<double> m_inp_x;
    std::vector<double> m_inp_y;
    std::vector<double> m_inp_z;

    // weighting filters
    ChISO2631_5_Wxy m_weighting_x;
    ChISO2631_5_Wxy m_weighting_y;
    ChISO2631_5_Wz m_weighting_z;

    // buffers for weighted output
    std::vector<double> m_out_x;
    std::vector<double> m_out_y;
    std::vector<double> m_out_z;

    double CalcPeaks(std::vector<double>& v, bool vertical);
};

/// @} chrono_utils

}  // end namespace utils
}  // end namespace chrono

#endif
