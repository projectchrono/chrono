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

#include <valarray>

#include "chrono/core/ChApiCE.h"

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
    int m_n;
    std::valarray<double> m_out;
};

/// Abstract Base class for simulated analogue filters in the time domain
class ChApi ChAnalogueFilter {
  public:
    ChAnalogueFilter() {};
    ~ChAnalogueFilter() {};
    virtual void Reset() = 0;
    virtual double Filter(double u) = 0;
  protected:
    double m_step;  // solver step width
    double m_u_old; // last input value
    double m_y_old; // last output value
};

// Calculate the integral of an input signal in the time domain
// H(s) = 1 / ( Ti * s)
class ChApi ChFilterI : public ChAnalogueFilter {
  public:
    ChFilterI() {};
    ChFilterI(double step, double Ti=1.0);
    ~ChFilterI() {};
    virtual void Reset() override;
    void Config(double step, double Ti=1.0);
    virtual double Filter(double u) override;
    
  private:
    double m_Ti;
};

// Caclulate the time derivation of an input signal
// H(s) = Td * s
class ChApi ChFilterD : public ChAnalogueFilter {
  public:
    ChFilterD() {};
    ChFilterD(double step, double Td=1.0);
    ~ChFilterD() {};
    virtual void Reset() override;
    void Config(double step, double Td=1.0);
    virtual double Filter(double u) override;
    
  private:
    double m_Td;
};

// Delay an input signal
// H(s) = Kpt1 / ( T1 * s + 1 )
class ChApi ChFilterPT1 : public ChAnalogueFilter {
  public:
    ChFilterPT1() {};
    ChFilterPT1(double step, double T1=1.0, double Kpt1=1.0);
    ~ChFilterPT1() {};
    virtual void Reset() override;
    void Config(double step, double T1=1.0, double Kpt1=1.0);
    virtual double Filter(double u) override;
    
  private:
    double m_T1;
    double m_Kpt1;
};

// PD1 Controller 
// H(s) = Kdt1 * ( Td1 * s + 1 )
class ChApi ChFilterPD1 : public ChAnalogueFilter {
  public:
    ChFilterPD1() {};
    ChFilterPD1(double step, double Td1=1.0, double Kdt1=1.0);
    ~ChFilterPD1() {};
    virtual void Reset() override;
    void Config(double step, double Td1=1.0, double Kdt1=1.0);
    virtual double Filter(double u) override;
    
  private:
    double m_Td1;
    double m_Kdt1;
};

// PDT1 Controller 
// H(s) = Kp * ( Td1 * s + 1 ) / ( T1 * s + 1)
class ChApi ChFilterPDT1 : public ChAnalogueFilter {
  public:
    ChFilterPDT1() {};
    ChFilterPDT1(double step, double Td1=1.0, double T1=1.0, double Kp=1.0);
    ~ChFilterPDT1() {};
    virtual void Reset() override;
    void Config(double step, double Td1=1.0, double T1=1.0, double Kp=1.0);
    virtual double Filter(double u) override;
    
  private:
    ChFilterPD1 pd1;
    ChFilterPT1 pt1;
};

/// @} chrono_utils

}  // end namespace utils
}  // end namespace chrono

#endif
