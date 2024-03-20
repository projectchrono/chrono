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

#ifndef CHFUNCT_INTEGRAL_H
#define CHFUNCT_INTEGRAL_H

#include "chrono/functions/ChFunctionBase.h"

namespace chrono {

/// @addtogroup chrono_functions
/// @{

/// Integral of a function.
/// `    y(x) = offset + \int{f(s) ds}_{x_start}^{x}    `.
/// The integral is computed using trapezoidal integration over a given number of samples.
/// The function returns a value equal to the offset for any value x<= x_start, while returns y(x_end) for any value x>=
/// x_end. x_start and x_end are set through the SetStartArg() and SetEndArg() methods.
class ChApi ChFunctionIntegral : public ChFunction {
  private:
    std::shared_ptr<ChFunction> m_integrand_fun;
    int m_integration_order;     ///< integration order
    double m_offset;             ///< initial value of the integral at x=m_x_start
    double m_x_start;            ///< start of integration interval
    double m_x_end;              ///< end of integration interval
    unsigned int m_num_samples;  ///< number of samples for the numerical quadrature
    ChArray<> m_cumintegral;     ///< precomputed integral values

  public:
    ChFunctionIntegral();
    ChFunctionIntegral(const ChFunctionIntegral& other);
    ~ChFunctionIntegral() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChFunctionIntegral* Clone() const override { return new ChFunctionIntegral(*this); }

    virtual Type GetType() const override { return ChFunction::Type::INTEGRAL; }

    virtual double GetVal(double x) const override;

    /// Precompute the integral values.
    /// Need to be called after the settings are changed.
    void Setup();

    /// Set the order of integration.
    void SetOrder(int int_order) { m_integration_order = int_order; }

    /// Get the order of integration.
    int GetOrder() const { return m_integration_order; }

    /// Set the number of samples used for the numerical quadrature.
    void SetNumSamples(int m_samples);

    /// Get the number of samples used for the numerical quadrature.
    unsigned int GetNumSamples() const { return m_num_samples; }

    /// Set the initial value of the integral.
    void SetOffsetVal(double offset) { m_offset = offset; }

    /// Get the initial value of the integral.
    double GetOffsetVal() const { return m_offset; }

    /// Set the integration interval.
    void SetInterval(double xstart, double xend);

    /// Set the integration interval starting point.
    void SetStartArg(double x_start) { m_x_start = x_start; }

    /// Set the integration interval ending point.
    void SetEndArg(double x_end) { m_x_end = x_end; }

    /// Get the integration interval starting point.
    double GetStart() const { return m_x_start; }

    /// Get the integration interval ending point.
    double GetEnd() const { return m_x_end; }

    /// Set the function to be integrated.
    void SetIntegrandFunction(std::shared_ptr<ChFunction> integrand_fun) { m_integrand_fun = integrand_fun; }

    /// Get the function to be integrated.
    std::shared_ptr<ChFunction> GetIntegrandFunction() const { return m_integrand_fun; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;
};

/// @} chrono_functions

CH_CLASS_VERSION(ChFunctionIntegral, 0)

}  // end namespace chrono

#endif
