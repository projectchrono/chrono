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

#ifndef CHFUNCT_INTEGRATE_H
#define CHFUNCT_INTEGRATE_H

#include "chrono/motion_functions/ChFunction_Base.h"

namespace chrono {

/// @addtogroup chrono_functions
/// @{

/// Integral of a function: `y = int{ f(x) dx`
///
/// Uses a numerical quadrature method to compute the definite integral.
class ChApi ChFunction_Integrate : public ChFunction {
  private:
    std::shared_ptr<ChFunction> fa;
    int order;  // 1= Integrate one time, 2= two times, etc.
    double C_start;
    double x_start;
    double x_end;
    int num_samples;
    ChArray<> array_x;

  public:
    ChFunction_Integrate();
    ChFunction_Integrate(const ChFunction_Integrate& other);
    ~ChFunction_Integrate() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChFunction_Integrate* Clone() const override { return new ChFunction_Integrate(*this); }

    virtual FunctionType Get_Type() const override { return FUNCT_INTEGRATE; }

    virtual double Get_y(double x) const override;

    void ComputeIntegral();

    void Set_order(int m_order) { order = m_order; }
    int Get_order() const { return order; }

    void Set_num_samples(int m_samples) {
        num_samples = m_samples;
        array_x.setZero(num_samples, 1);
        ComputeIntegral();
    }
    int Get_num_samples() const { return num_samples; }

    void Set_C_start(double m_val) {
        C_start = m_val;
        ComputeIntegral();
    }
    double Get_C_start() const { return C_start; }

    void Set_x_start(double m_val) {
        x_start = m_val;
        ComputeIntegral();
    }
    double Get_x_start() const { return x_start; }

    void Set_x_end(double m_val) {
        x_end = m_val;
        ComputeIntegral();
    }
    double Get_x_end() const { return x_end; }

    /// Set the function to be integrated
    void Set_fa(std::shared_ptr<ChFunction> m_fa) {
        fa = m_fa;
        ComputeIntegral();
    }
    std::shared_ptr<ChFunction> Get_fa() const { return fa; }

    virtual void Estimate_x_range(double& xmin, double& xmax) const override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;
};

/// @} chrono_functions

CH_CLASS_VERSION(ChFunction_Integrate, 0)

}  // end namespace chrono

#endif
