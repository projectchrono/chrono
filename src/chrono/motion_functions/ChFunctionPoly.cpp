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

#include "chrono/motion_functions/ChFunctionPoly.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunctionPoly)

ChFunctionPoly::ChFunctionPoly() {
    order = 0;
    for (int i = 0; i < POLY_COEFF_ARRAY; i++) {
        coeff[i] = 0;
    }
}

ChFunctionPoly::ChFunctionPoly(const ChFunctionPoly& other) {
    order = other.order;
    for (int i = 0; i < POLY_COEFF_ARRAY; i++) {
        coeff[i] = other.coeff[i];
    }
}

double ChFunctionPoly::Get_y(double x) const {
    double total = 0;
    for (int i = 0; i <= order; i++) {
        total += (coeff[i] * pow(x, (double)i));
    }
    return total;
}

double ChFunctionPoly::Get_y_dx(double x) const {
    double total = 0;
    for (int i = 1; i <= order; i++) {
        total += ((double)i * coeff[i] * pow(x, ((double)(i - 1))));
    }
    return total;
}

double ChFunctionPoly::Get_y_dxdx(double x) const {
    double total = 0;
    for (int i = 2; i <= order; i++) {
        total += ((double)(i * (i - 1)) * coeff[i] * pow(x, ((double)(i - 2))));
    }
    return total;
}

void ChFunctionPoly::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChFunctionPoly>();
    // serialize parent class
    ChFunction::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(coeff);
    marchive << CHNVP(order);
}

void ChFunctionPoly::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChFunctionPoly>();
    // deserialize parent class
    ChFunction::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(coeff);
    marchive >> CHNVP(order);
}

}  // end namespace chrono
