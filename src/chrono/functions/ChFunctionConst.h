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

#ifndef CHFUNCT_CONST_H
#define CHFUNCT_CONST_H

#include "chrono/functions/ChFunctionBase.h"

namespace chrono {

/// @addtogroup chrono_functions
/// @{

/// Function returnin a constant value.
class ChApi ChFunctionConst : public ChFunction {
  public:
    ChFunctionConst();
    ChFunctionConst(double y_constant);
    ChFunctionConst(const ChFunctionConst& other);

    virtual ~ChFunctionConst() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChFunctionConst* Clone() const override { return new ChFunctionConst(*this); }

    /// Returns the function type.
    virtual Type GetType() const override { return ChFunction::Type::CONSTANT; }

    /// Returns the function value at \a x (constant in this case).
    virtual double GetVal(double x) const override { return m_constant; }

    /// Returns the function first derivative (i.e. 0)
    virtual double GetDer(double x) const override { return 0; }

    /// Returns the function second derivative (i.e. 0)
    virtual double GetDer2(double x) const override { return 0; }

    /// Returns the function third derivative (i.e. 0)
    virtual double GetDer3(double x) const override { return 0; }

    /// Set the constant value of the function.
    void SetConstant(double y_constant) { m_constant = y_constant; }

    /// Get the constant value of the function
    double GetConstant() const { return m_constant; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    double m_constant;
};

/// @} chrono_functions

CH_CLASS_VERSION(ChFunctionConst, 0)

}  // end namespace chrono

#endif
