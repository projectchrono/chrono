// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHFUNCT_OPERATION_H
#define CHFUNCT_OPERATION_H

#include "chrono/motion_functions/ChFunction_Base.h"
#include "chrono/motion_functions/ChFunction_Const.h"

namespace chrono {

enum eChOperation {
    ChOP_ADD = 0,
    ChOP_SUB,
    ChOP_MUL,
    ChOP_DIV,
    ChOP_POW,
    ChOP_MAX,
    ChOP_MIN,
    ChOP_MODULO,
    ChOP_FABS,
    ChOP_FUNCT,
};

CH_ENUM_MAPPER_BEGIN(eChOperation);
CH_ENUM_VAL(ChOP_ADD);
CH_ENUM_VAL(ChOP_SUB);
CH_ENUM_VAL(ChOP_MUL);
CH_ENUM_VAL(ChOP_DIV);
CH_ENUM_VAL(ChOP_POW);
CH_ENUM_VAL(ChOP_MAX);
CH_ENUM_VAL(ChOP_MIN);
CH_ENUM_VAL(ChOP_MODULO);
CH_ENUM_VAL(ChOP_FABS);
CH_ENUM_VAL(ChOP_FUNCT);
CH_ENUM_MAPPER_END(eChOperation);

/// Operation between functions:
///
/// math operation between A and  B operands
///   - fa = first operand function
///   - fb = second operand function

class ChApi ChFunction_Operation : public ChFunction {

    CH_FACTORY_TAG(ChFunction_Operation)

  private:
    std::shared_ptr<ChFunction> fa;
    std::shared_ptr<ChFunction> fb;
    eChOperation op_type;

  public:
    ChFunction_Operation();
    ChFunction_Operation(const ChFunction_Operation& other);
    ~ChFunction_Operation() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChFunction_Operation* Clone() const override { return new ChFunction_Operation(*this); }

    virtual FunctionType Get_Type() const override { return FUNCT_OPERATION; }

    virtual double Get_y(double x) const override;

    void Set_optype(eChOperation m_op) { op_type = m_op; }
    eChOperation Get_optype() { return op_type; }

    void Set_fa(std::shared_ptr<ChFunction> m_fa) { fa = m_fa; }
    std::shared_ptr<ChFunction> Get_fa() { return fa; }

    void Set_fb(std::shared_ptr<ChFunction> m_fb) { fb = m_fb; }
    std::shared_ptr<ChFunction> Get_fb() { return fb; }

    virtual void Estimate_x_range(double& xmin, double& xmax) const override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override {
        // version number
        marchive.VersionWrite<ChFunction_Operation>();
        // serialize parent class
        ChFunction::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(fa);
        marchive << CHNVP(fb);
        eChOperation_mapper mmapper;
        marchive << CHNVP(mmapper(op_type), "operation_type");
    }

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override {
        // version number
        int version = marchive.VersionRead<ChFunction_Operation>();
        // deserialize parent class
        ChFunction::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(fa);
        marchive >> CHNVP(fb);
        eChOperation_mapper mmapper;
        marchive >> CHNVP(mmapper(op_type), "operation_type");
    }
};

CH_CLASS_VERSION(ChFunction_Operation,0)

}  // end namespace chrono

#endif
