//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011-2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHFUNCT_OPERATION_H
#define CHFUNCT_OPERATION_H

//////////////////////////////////////////////////
//
//   ChFunction_Operation.h
//
//   Function objects,
//   as scalar functions of scalar variable y=f(t)
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "ChFunction_Base.h"
#include "ChFunction_Const.h"

namespace chrono {

#define FUNCT_OPERATION 12

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

/// OPERATION BETWEEN FUNCTIONS
/// (math operation between A and  B operands
///   - fa = first operand function
///   - fb = second operand function

class ChApi ChFunction_Operation : public ChFunction {
    CH_RTTI(ChFunction_Operation, ChFunction);

  private:
    std::shared_ptr<ChFunction> fa;
    std::shared_ptr<ChFunction> fb;
    eChOperation op_type;  // see operation type IDS

  public:
    ChFunction_Operation() {
        op_type = ChOP_ADD;
        fa = std::make_shared<ChFunction_Const>();
        fb = std::make_shared<ChFunction_Const>();
    }
    ~ChFunction_Operation(){};
    void Copy(ChFunction_Operation* source);
    ChFunction* new_Duplicate();

    void Set_optype(eChOperation m_op) { op_type = m_op; }
    eChOperation Get_optype() { return op_type; }

    void Set_fa(std::shared_ptr<ChFunction> m_fa) { fa = m_fa; }
    std::shared_ptr<ChFunction> Get_fa() { return fa; }

    void Set_fb(std::shared_ptr<ChFunction> m_fb) { fb = m_fb; }
    std::shared_ptr<ChFunction> Get_fb() { return fb; }

    double Get_y(double x);
    //	double Get_y_dx   (double x) ;
    //	double Get_y_dxdx (double x) ;

    void Estimate_x_range(double& xmin, double& xmax);

    int Get_Type() { return (FUNCT_OPERATION); }

    int MakeOptVariableTree(ChList<chjs_propdata>* mtree);

    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive)
    {
        // version number
        marchive.VersionWrite(1);
        // serialize parent class
        ChFunction::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(fa);
        marchive << CHNVP(fb);
        eChOperation_mapper mmapper;
        marchive << CHNVP(mmapper(op_type),"operation_type");
    }

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) 
    {
        // version number
        int version = marchive.VersionRead();
        // deserialize parent class
        ChFunction::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(fa);
        marchive >> CHNVP(fb);
        eChOperation_mapper mmapper;
        marchive >> CHNVP(mmapper(op_type),"operation_type");
    }

};

}  // END_OF_NAMESPACE____

#endif
