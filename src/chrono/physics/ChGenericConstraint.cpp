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

#include <cmath>

#include "chrono/physics/ChGenericConstraint.h"

namespace chrono {

// -----------------------------------------------------------------------------

ChGenericConstraint::ChGenericConstraint() : valid(false), disabled(false), C(NULL) {
    Reset_Cn(Get_Cn());
}

ChGenericConstraint::~ChGenericConstraint() {
    if (C)
        delete C;
    C = NULL;
}

// Sets the number of equations in this constraints (reset the
// size of the C residual vector).
int ChGenericConstraint::Reset_Cn(int mCn) {
    if (mCn > 0) {
        Cn = mCn;
        if (C) {
            delete C;
            C = NULL;
        }
        C = new ChMatrixDynamic<>(Cn, 1);
    } else {
        Cn = 0;
        if (C) {
            delete C;
            C = NULL;
        }
        C = NULL;
    }
    return Cn;
}

// -----------------------------------------------------------------------------

ChGenericConstraint_Chf::ChGenericConstraint_Chf() {
    root_function = NULL;
}

ChGenericConstraint_Chf::ChGenericConstraint_Chf(ChFunction* mRootFunct, char* mTreeIDs) {
    this->root_function = mRootFunct;
    this->target_function.SetTreeIDs(mTreeIDs);
    this->target_function.RestoreReference(this->root_function);
}

bool ChGenericConstraint_Chf::RestoreReferences(ChFunction* mroot) {
    root_function = mroot;
    if (mroot) {
        valid = this->target_function.RestoreReference(mroot);
        return (valid);
    } else {
        return (valid = false);
    }
}

// -----------------------------------------------------------------------------

ChGenericConstraint_Chf_ImposeVal::ChGenericConstraint_Chf_ImposeVal(ChFunction* mRootFunct,
                                                                     char* mTreeIDs,
                                                                     double mval,
                                                                     double mtime) {
    this->target_function.SetTreeIDs(mTreeIDs);
    this->RestoreReferences(mRootFunct);

    this->SetT(mtime);
    this->SetValue(mval);
    this->Reset_Cn(Get_Cn());
    this->Update();
}

bool ChGenericConstraint_Chf_ImposeVal::Update() {
    // INHERIT parent behaviour:
    if (!ChGenericConstraint_Chf::Update())
        return false;

    // Implement method:

    // CONSTRAINT EQUATION (residual of mc=0);
    double mc = this->Get_target_function()->Get_y(this->T) - this->value;

    if (C)
        C->SetElement(0, 0, mc);

    return true;
}

// -----------------------------------------------------------------------------

ChGenericConstraint_Chf_Continuity::ChGenericConstraint_Chf_Continuity(ChFunction* mRootFunct,
                                                                       char* mTreeIDs,
                                                                       int cont_ord,
                                                                       int interf_num) {
    this->target_function.SetTreeIDs(mTreeIDs);
    this->RestoreReferences(mRootFunct);

    this->SetContinuityOrder(cont_ord);
    this->SetInterfaceNum(interf_num);

    this->Reset_Cn(Get_Cn());
    this->Update();
}

bool ChGenericConstraint_Chf_Continuity::Update() {
    // INHERIT parent behaviour:
    if (!ChGenericConstraint_Chf::Update())
        return false;

    // Implement method:

    // a- cast target function to sequence, if correct
    if (this->Get_target_function()->Get_Type() != ChFunction::FUNCT_SEQUENCE)
        return false;

    ChFunction_Sequence* mfun = (ChFunction_Sequence*)this->Get_target_function();

    // b- computes the time instant of discontinuity
    double mt;
    double mc = 0;
    if ((this->interface_num > mfun->Get_list().size()) || (this->interface_num < 0))  // NO!out of range...
    {
        if (C)
            C->SetElement(0, 0, 0.0);
        return false;
    }
    if (this->interface_num == mfun->Get_list().size())  // ok, last discontinuity
    {
        mt = mfun->GetNthNode(this->interface_num)->t_end;
    }

    mt = mfun->GetNthNode(this->interface_num)->t_start;  // ok, inner interface

    // CONSTRAINT EQUATION (residual of mc=0);
    double mstep = 1e-9;
    double v_a, v_b;
    switch (this->continuity_order) {
        case 0:  // C0
            v_a = mfun->Get_y(mt - mstep);
            v_b = mfun->Get_y(mt + mstep);
            break;
        case 1:  // C1
            v_a = mfun->Get_y_dx(mt - mstep);
            v_b = mfun->Get_y_dx(mt + mstep);
            break;
        case 2:  // C2
            v_a = mfun->Get_y_dxdx(mt - mstep);
            v_b = mfun->Get_y_dxdx(mt + mstep);
            break;
        default:
            v_a = v_b = 0;
    }
    mc = v_b - v_a;

    if (C)
        C->SetElement(0, 0, mc);

    return true;
}

// -----------------------------------------------------------------------------

ChGenericConstraint_Chf_HorDistance::ChGenericConstraint_Chf_HorDistance(ChFunction* mRootFunct,
                                                                         char* mTreeIDs,
                                                                         int mhA,
                                                                         int mhB) {
    this->target_function.SetTreeIDs(mTreeIDs);
    this->RestoreReferences(mRootFunct);

    this->SetHandleA(mhA);
    this->SetHandleB(mhB);

    this->Reset_Cn(Get_Cn());
    this->Update();
}

bool ChGenericConstraint_Chf_HorDistance::Update() {
    // INHERIT parent behaviour:
    if (!ChGenericConstraint_Chf::Update())
        return false;

    // Implement method:

    // a- cast target function to sequence, if correct
    if (this->Get_target_function()->Get_Type() != ChFunction::FUNCT_SEQUENCE)
        return false;

    ChFunction_Sequence* mfun = (ChFunction_Sequence*)this->Get_target_function();

    // b- computes the time instants of the two handles

    double x_a = mfun->GetNthNode(this->handleA)->t_start;  // ok, first  handle X value
    double x_b = mfun->GetNthNode(this->handleB)->t_start;  // ok, second handle X value

    // CONSTRAINT EQUATION (residual of mc=0);

    double mc = x_b - x_a;

    if (C)
        C->SetElement(0, 0, mc);

    return true;
}

// -----------------------------------------------------------------------------

ChGenericConstraint_Chf_VertDistance::ChGenericConstraint_Chf_VertDistance(ChFunction* mRootFunct,
                                                                           char* mTreeIDs,
                                                                           int mhA,
                                                                           int mhB) {
    this->target_function.SetTreeIDs(mTreeIDs);
    this->RestoreReferences(mRootFunct);

    this->SetHandleA(mhA);
    this->SetHandleB(mhB);

    this->Reset_Cn(Get_Cn());
    this->Update();
}

bool ChGenericConstraint_Chf_VertDistance::Update() {
    // INHERIT parent behaviour:
    if (!ChGenericConstraint_Chf::Update())
        return false;

    // Implement method:

    // a- cast target function to sequence, if correct
    if (this->Get_target_function()->Get_Type() != ChFunction::FUNCT_SEQUENCE)
        return false;

    ChFunction_Sequence* mfun = (ChFunction_Sequence*)this->Get_target_function();

    // b- computes the time instants of the two handles

    double x_a = mfun->GetNthNode(this->handleA)->t_start;  // ok, first  handle X value
    double x_b = mfun->GetNthNode(this->handleB)->t_start;  // ok, second handle X value

    double y_a = mfun->Get_y(x_a);
    double y_b = mfun->Get_y(x_b);

    // CONSTRAINT EQUATION (residual of mc=0);

    double mc = y_b - y_a;

    if (C)
        C->SetElement(0, 0, mc);

    return true;
}

}  // end namespace chrono
