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

#include "chrono/motion_functions/ChFunction_Poly.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunction_Poly)

ChFunction_Poly::ChFunction_Poly() {
    order = 0;
    for (int i = 0; i < POLY_COEFF_ARRAY; i++) {
        coeff[i] = 0;
    }
}

ChFunction_Poly::ChFunction_Poly(const ChFunction_Poly& other) {
    order = other.order;
    for (int i = 0; i < POLY_COEFF_ARRAY; i++) {
        coeff[i] = other.coeff[i];
    }
}

double ChFunction_Poly::Get_y(double x) const {
    double total = 0;
    for (int i = 0; i <= order; i++) {
        total += (coeff[i] * pow(x, (double)i));
    }
    return total;
}

double ChFunction_Poly::Get_y_dx(double x) const {
    double total = 0;
    for (int i = 1; i <= order; i++) {
        total += ((double)i * coeff[i] * pow(x, ((double)(i - 1))));
    }
    return total;
}

double ChFunction_Poly::Get_y_dxdx(double x) const {
    double total = 0;
    for (int i = 2; i <= order; i++) {
        total += ((double)(i * (i - 1)) * coeff[i] * pow(x, ((double)(i - 2))));
    }
    return total;
}

void ChFunction_Poly::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChFunction_Poly>();
    // serialize parent class
    ChFunction::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(coeff);
    marchive << CHNVP(order);
}

void ChFunction_Poly::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChFunction_Poly>();
    // deserialize parent class
    ChFunction::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(coeff);
    marchive >> CHNVP(order);
}

}  // end namespace chrono
