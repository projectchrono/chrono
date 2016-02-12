//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChFunction_Operation.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "ChFunction_Operation.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction_Operation> a_registration_operation;

void ChFunction_Operation::Copy(ChFunction_Operation* source) {
    op_type = source->op_type;
    // fa = source->fa;		//***? shallow copy (now sharing same object)...
    fa = std::shared_ptr<ChFunction>(source->fa->new_Duplicate());  //***? ..or deep copy? make optional with flag?
    // fb = source->fb;		//***? shallow copy (now sharing same object)...
    fb = std::shared_ptr<ChFunction>(source->fb->new_Duplicate());  //***? ..or deep copy? make optional with flag?
}

ChFunction* ChFunction_Operation::new_Duplicate() {
    ChFunction_Operation* m_func;
    m_func = new ChFunction_Operation;
    m_func->Copy(this);
    return (m_func);
}

double ChFunction_Operation::Get_y(double x) {
    double res;

    switch (op_type) {
        case ChOP_ADD:
            res = fa->Get_y(x) + fb->Get_y(x);
            break;
        case ChOP_SUB:
            res = fa->Get_y(x) - fb->Get_y(x);
            break;
        case ChOP_MUL:
            res = fa->Get_y(x) * fb->Get_y(x);
            break;
        case ChOP_DIV:
            res = fa->Get_y(x) / fb->Get_y(x);
            break;
        case ChOP_POW:
            res = pow(fa->Get_y(x), fb->Get_y(x));
            break;
        case ChOP_MAX:
            res = ChMax(fa->Get_y(x), fb->Get_y(x));
            break;
        case ChOP_MIN:
            res = ChMin(fa->Get_y(x), fb->Get_y(x));
            break;
        case ChOP_MODULO:
            res = fmod(fa->Get_y(x), fb->Get_y(x));
            break;
        case ChOP_FABS:
            res = fabs(fa->Get_y(x));
            break;
        case ChOP_FUNCT:
            res = fa->Get_y(fb->Get_y(x));
            break;
        default:
            res = 0;
            break;
    }
    return res;
}
/*
double ChFunction_Operation::Get_y_dx   (double x)
{
    double res = 0;
    res = ChFunction::Get_y_dx(x); // default: numerical differentiation
    return res;
}

double ChFunction_Operation::Get_y_dxdx (double x)
{
    double res = 0;
    res = ChFunction::Get_y_dxdx(x); // default: numerical differentiation
    return res;
}
*/
void ChFunction_Operation::Estimate_x_range(double& xmin, double& xmax) {
    double amin, amax, bmin, bmax;
    fa->Estimate_x_range(amin, amax);
    fb->Estimate_x_range(bmin, bmax);
    xmin = ChMin(amin, bmin);
    xmax = ChMax(amax, bmax);
}

int ChFunction_Operation::MakeOptVariableTree(ChList<chjs_propdata>* mtree) {
    int i = 0;

    // inherit parent behaviour
    ChFunction::MakeOptVariableTree(mtree);

    // expand tree for the two children..

    chjs_propdata* mdataA = new chjs_propdata;
    strcpy(mdataA->propname, "fa");
    strcpy(mdataA->label, mdataA->propname);
    mdataA->haschildren = TRUE;
    mtree->AddTail(mdataA);

    i += this->fa->MakeOptVariableTree(&mdataA->children);

    chjs_propdata* mdataB = new chjs_propdata;
    strcpy(mdataB->propname, "fb");
    strcpy(mdataB->label, mdataB->propname);
    mdataB->haschildren = TRUE;
    mtree->AddTail(mdataB);

    i += this->fb->MakeOptVariableTree(&mdataB->children);

    return i;
}


}  // END_OF_NAMESPACE____

// eof
