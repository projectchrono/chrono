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

#include "chrono/motion_functions/ChFunction_Sigma.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunction_Sigma)

ChFunction_Sigma::ChFunction_Sigma(const ChFunction_Sigma& other) {
    amp = other.amp;
    start = other.start;
    end = other.end;
}

void ChFunction_Sigma::Estimate_x_range(double& xmin, double& xmax) const {
    double mdx = end - start;
    xmin = start + mdx * 0.1;
    xmax = end - mdx * 0.1;
}

double ChFunction_Sigma::Get_y(double x) const {
    double ret;
    double A = (end - start);
    if (x < start)
        return 0;
    if (x > end)
        return amp;
    else {
        ret = amp * ((3 * (pow(((x - start) / A), 2))) - 2 * (pow(((x - start) / A), 3)));
    }
    return ret;
}

double ChFunction_Sigma::Get_y_dx(double x) const {
    double ret;
    double A = (end - start);
    if ((x < start) || (x > end))
        ret = 0;
    else {
        ret = amp * (6 * ((x - start) / pow(A, 2)) - 6 * (pow((x - start), 2) / pow(A, 3)));
    }
    return ret;
}

double ChFunction_Sigma::Get_y_dxdx(double x) const {
    double ret;
    double A = (end - start);
    if ((x < start) || (x > end))
        ret = 0;
    else {
        ret = amp * (6 * (1 / pow(A, 2)) - 12 * ((x - start) / pow(A, 3)));
    }
    return ret;
}

void ChFunction_Sigma::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChFunction_Sigma>();
    // serialize parent class
    ChFunction::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(amp);
    marchive << CHNVP(start);
    marchive << CHNVP(end);
}

void ChFunction_Sigma::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChFunction_Sigma>();
    // deserialize parent class
    ChFunction::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(amp);
    marchive >> CHNVP(start);
    marchive >> CHNVP(end);
}

}  // end namespace chrono
