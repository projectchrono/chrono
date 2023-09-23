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

#include "chrono/motion_functions/ChFunction_Poly345.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunction_Poly345)

ChFunction_Poly345::ChFunction_Poly345(double m_h, double m_end) : h(m_h) {
    Set_end(m_end);
}

ChFunction_Poly345::ChFunction_Poly345(const ChFunction_Poly345& other) {
    h = other.h;
    end = other.end;
}

double ChFunction_Poly345::Get_y(double x) const {
    double ret = 0;
    if (x <= 0)
        return 0;
    if (x >= end)
        return h;
    double a = x / end;
    ret = h * (10 * pow(a, 3) - 15 * pow(a, 4) + 6 * pow(a, 5));
    return ret;
}

double ChFunction_Poly345::Get_y_dx(double x) const {
    double ret = 0;
    if (x <= 0)
        return 0;
    if (x >= end)
        return 0;
    double a = x / end;
    ret = h * (1 / end) * (30 * pow(a, 2) - 60 * pow(a, 3) + 30 * pow(a, 4));
    return ret;
}

double ChFunction_Poly345::Get_y_dxdx(double x) const {
    double ret = 0;
    if (x <= 0)
        return 0;
    if (x >= end)
        return 0;
    double a = x / end;
    ret = h * (1 / (end * end)) * (60 * a - 180 * pow(a, 2) + 120 * pow(a, 3));
    return ret;
}

double ChFunction_Poly345::Get_y_dxdxdx(double x) const {
    double ret = 0;
    if (x <= 0)
        return 0;
    if (x >= end)
        return 0;
    double a = x / end;
    ret = h / pow(end, 3) * (60 - 360 * a + 360 * pow(a, 2));
    return ret;
}

void ChFunction_Poly345::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChFunction_Poly345>();
    // serialize parent class
    ChFunction::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(h);
    marchive << CHNVP(end);
}

void ChFunction_Poly345::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChFunction_Poly345>();
    // deserialize parent class
    ChFunction::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(h);
    marchive >> CHNVP(end);
}

}  // end namespace chrono
