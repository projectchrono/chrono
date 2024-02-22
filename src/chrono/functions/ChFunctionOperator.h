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

#ifndef CHFUNCT_OPERATOR_H
#define CHFUNCT_OPERATOR_H

#include "chrono/functions/ChFunctionBase.h"
#include "chrono/functions/ChFunctionConst.h"

namespace chrono {

/// @addtogroup chrono_functions
/// @{

/// Operation between functions:
///
/// Math operation between two operand functions
class ChApi ChFunctionOperator : public ChFunction {
  public:
    ChFunctionOperator();
    ChFunctionOperator(const ChFunctionOperator& other);
    ~ChFunctionOperator() {}

    /// Type of operation.
    enum eChOperation {
        ADD = 0,
        SUB,
        MUL,
        DIV,
        POW,
        MAX,
        MIN,
        MODULO,
        FABS,
        FUNCT,
    };

    /// "Virtual" copy constructor (covariant return type).
    virtual ChFunctionOperator* Clone() const override { return new ChFunctionOperator(*this); }

    virtual Type GetType() const override { return ChFunction::Type::OPERATOR; }

    virtual double GetVal(double x) const override;

    /// Set the operation type between the two operands.
    void SetOperationType(eChOperation m_op) { m_op_type = m_op; }

    /// Get the operation type between the two operands.
    eChOperation GetOperationType() { return m_op_type; }

    /// Set the first operand function.
    void SetFirstOperandFunction(std::shared_ptr<ChFunction> m_m_first_fun) { m_first_fun = m_m_first_fun; }

    /// Get the first operand function.
    std::shared_ptr<ChFunction> GetFirstOperandFunction() { return m_first_fun; }

    /// Set the second operand function.
    void SetSecondOperandFunction(std::shared_ptr<ChFunction> m_m_second_fun) { m_second_fun = m_m_second_fun; }

    /// Get the second operand function.
    std::shared_ptr<ChFunction> GetSecondOperandFunction() { return m_second_fun; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

    /// @cond
    CH_ENUM_MAPPER_BEGIN(eChOperation);
    CH_ENUM_VAL(ADD);
    CH_ENUM_VAL(SUB);
    CH_ENUM_VAL(MUL);
    CH_ENUM_VAL(DIV);
    CH_ENUM_VAL(POW);
    CH_ENUM_VAL(MAX);
    CH_ENUM_VAL(MIN);
    CH_ENUM_VAL(MODULO);
    CH_ENUM_VAL(FABS);
    CH_ENUM_VAL(FUNCT);
    CH_ENUM_MAPPER_END(eChOperation);
    /// @endcond

  private:
    std::shared_ptr<ChFunction> m_first_fun;
    std::shared_ptr<ChFunction> m_second_fun;
    eChOperation m_op_type;
};

/// @} chrono_functions

CH_CLASS_VERSION(ChFunctionOperator, 0)

}  // end namespace chrono

#endif
