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

#ifndef CHFUNCT_DERIVE_H
#define CHFUNCT_DERIVE_H

#include "chrono/motion_functions/ChFunctionBase.h"
#include "chrono/motion_functions/ChFunctionConst.h"

namespace chrono {

/// @addtogroup chrono_functions
/// @{

/// Derivative of a function: `y = df/dx`
///
/// Uses a numerical differentiation method to compute the derivative
/// of a generic function.
class ChApi ChFunctionDerive : public ChFunction {
  private:
    std::shared_ptr<ChFunction> fa;
    int order;  ///< 1= derive one time, 2= two times, etc.

  public:
    ChFunctionDerive() : order(1) {}
    ChFunctionDerive(const ChFunctionDerive& other);
    ~ChFunctionDerive() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChFunctionDerive* Clone() const override { return new ChFunctionDerive(*this); }

    virtual FunctionType Get_Type() const override { return FUNCT_DERIVE; }

    virtual double Get_y(double x) const override;

    void Set_order(int m_order) { order = m_order; }
    int Get_order() { return order; }

    void Set_fa(std::shared_ptr<ChFunction> m_fa) { fa = m_fa; }
    std::shared_ptr<ChFunction> Get_fa() { return fa; }

    virtual void Estimate_x_range(double& xmin, double& xmax) const override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;
};

/// @} chrono_functions

CH_CLASS_VERSION(ChFunctionDerive, 0)

}  // namespace chrono

#endif
