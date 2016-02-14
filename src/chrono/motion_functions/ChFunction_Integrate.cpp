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
//   ChFunction_Integrate.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "ChFunction_Integrate.h"
#include "ChFunction_Const.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction_Integrate> a_registration_integrate;

ChFunction_Integrate::ChFunction_Integrate() {
    order = 1;
    fa = std::make_shared<ChFunction_Const>(); // default
    C_start = x_start = 0;
    x_end = 1;
    num_samples = 2000;
    array_x = new ChMatrixDynamic<>(num_samples, 1);
}

void ChFunction_Integrate::Copy(ChFunction_Integrate* source) {
    // fa = source->fa;		//***? shallow copy (now sharing same object)...
    fa = std::shared_ptr<ChFunction>(source->fa->new_Duplicate());  //***? ..or deep copy? make optional with flag?
    order = source->order;
    C_start = source->C_start;
    x_start = source->x_start;
    x_end = source->x_end;
    num_samples = source->num_samples;
    array_x->CopyFromMatrix(*source->array_x);
}

ChFunction* ChFunction_Integrate::new_Duplicate() {
    ChFunction_Integrate* m_func;
    m_func = new ChFunction_Integrate;
    m_func->Copy(this);
    return (m_func);
}

void ChFunction_Integrate::ComputeIntegral() {
    double mstep = (x_end - x_start) / ((double)(num_samples - 1));
    double x_a, x_b, y_a, y_b, F_b;

    double F_sum = this->Get_C_start();

    this->array_x->SetElement(0, 0, this->Get_C_start());

    for (int i = 1; i < this->num_samples; i++) {
        x_b = x_start + ((double)i) * (mstep);
        x_a = x_b - mstep;
        y_a = this->fa->Get_y(x_a);
        y_b = this->fa->Get_y(x_b);
        // trapezoidal rule..
        F_b = F_sum + mstep * (y_a + y_b) * 0.5;
        this->array_x->SetElement(i, 0, F_b);
        F_sum = F_b;
    }
}

double ChFunction_Integrate::Get_y(double x) {
    if ((x < x_start) || (x > x_end))
        return 0.0;
    int i_a, i_b;
    double position = (double)(num_samples - 1) * ((x - x_start) / (x_end - x_start));
    i_a = (int)(floor(position));
    i_b = i_a + 1;

    if (i_a == num_samples - 1)
        return array_x->GetElement(num_samples - 1, 0);

    if ((i_a < 0) || (i_b >= num_samples))
        return 0.0;

    double weightB = position - (double)i_a;
    double weightA = 1 - weightB;

    return (weightA * (array_x->GetElement(i_a, 0)) + weightB * (array_x->GetElement(i_b, 0)));
}

void ChFunction_Integrate::Estimate_x_range(double& xmin, double& xmax) {
    xmin = x_start;
    xmax = x_end;
}

int ChFunction_Integrate::MakeOptVariableTree(ChList<chjs_propdata>* mtree) {
    int i = 0;

    // inherit parent behaviour
    ChFunction::MakeOptVariableTree(mtree);

    // expand tree for children..

    chjs_propdata* mdataA = new chjs_propdata;
    strcpy(mdataA->propname, "fa");
    strcpy(mdataA->label, mdataA->propname);
    mdataA->haschildren = TRUE;
    mtree->AddTail(mdataA);

    i += this->fa->MakeOptVariableTree(&mdataA->children);

    return i;
}


}  // END_OF_NAMESPACE____

// eof
