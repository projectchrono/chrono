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

#include "chrono/motion_functions/ChFunction_Operation.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunction_Operation)

ChFunction_Operation::ChFunction_Operation() {
    op_type = ChOP_ADD;
    fa = chrono_types::make_shared<ChFunction_Const>();
    fb = chrono_types::make_shared<ChFunction_Const>();
}

ChFunction_Operation::ChFunction_Operation(const ChFunction_Operation& other) {
    op_type = other.op_type;
    fa = std::shared_ptr<ChFunction>(other.fa->Clone());
    fb = std::shared_ptr<ChFunction>(other.fb->Clone());
}

double ChFunction_Operation::Get_y(double x) const {
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
void ChFunction_Operation::Estimate_x_range(double& xmin, double& xmax) const {
    double amin, amax, bmin, bmax;
    fa->Estimate_x_range(amin, amax);
    fb->Estimate_x_range(bmin, bmax);
    xmin = ChMin(amin, bmin);
    xmax = ChMax(amax, bmax);
}

void ChFunction_Operation::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChFunction_Operation>();
    // serialize parent class
    ChFunction::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(fa);
    marchive << CHNVP(fb);
    eChOperation_mapper mmapper;
    marchive << CHNVP(mmapper(op_type), "operation_type");
}

void ChFunction_Operation::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChFunction_Operation>();
    // deserialize parent class
    ChFunction::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(fa);
    marchive >> CHNVP(fb);
    eChOperation_mapper mmapper;
    marchive >> CHNVP(mmapper(op_type), "operation_type");
}

}  // end namespace chrono
