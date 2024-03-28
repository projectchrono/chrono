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

#ifndef CHFUNCT_DERIVATIVE_H
#define CHFUNCT_DERIVATIVE_H

#include "chrono/functions/ChFunctionBase.h"
#include "chrono/functions/ChFunctionConst.h"

namespace chrono {

/// @addtogroup chrono_functions
/// @{

/// Derivative of a function.
///
/// Uses a numerical differentiation method to compute the derivative
/// of a generic function.
class ChApi ChFunctionDerivative : public ChFunction {
  private:
    std::shared_ptr<ChFunction> m_operand_fun;
    int m_der_order;  ///< derivative order

  public:
    ChFunctionDerivative() : m_der_order(1) {}
    ChFunctionDerivative(const ChFunctionDerivative& other);
    ~ChFunctionDerivative() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChFunctionDerivative* Clone() const override { return new ChFunctionDerivative(*this); }

    virtual Type GetType() const override { return ChFunction::Type::DERIVATIVE; }

    /// Get function output at \a x.
    virtual double GetVal(double x) const override;

    /// Set the derivative order.
    void SetOrder(int m_order) { m_der_order = m_order; }

    /// Get the derivative order.
    int GetOrder() const { return m_der_order; }

    /// Set the function to be differentiated.
    void SetOperandFunction(std::shared_ptr<ChFunction> operand_function) { m_operand_fun = operand_function; }

    /// Get the function to be differentiated.
    std::shared_ptr<ChFunction> GetOperandFunction() { return m_operand_fun; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;
};

/// @} chrono_functions

CH_CLASS_VERSION(ChFunctionDerivative, 0)

}  // namespace chrono

#endif
