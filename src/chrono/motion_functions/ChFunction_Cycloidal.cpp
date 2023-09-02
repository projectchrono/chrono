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
// Authors: Dario Fusai
// =============================================================================

#include "chrono/motion_functions/ChFunction_Cycloidal.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunction_Cycloidal)

ChFunction_Cycloidal::ChFunction_Cycloidal(double m_h, double m_end)
    : h(m_h)
{
    Set_end(m_end);
}

ChFunction_Cycloidal::ChFunction_Cycloidal(const ChFunction_Cycloidal& other) {
    h = other.h;
    end = other.end;
}

double ChFunction_Cycloidal::Get_y(double x) const {
    if (x <= 0)
        return 0;
    else if (x >= end)
        return h;
    double ret = h * (x / end - 1. / CH_C_2PI * sin(CH_C_2PI * x / end));
    return ret;
}

double ChFunction_Cycloidal::Get_y_dx(double x) const {
    if (x <= 0)
        return 0;
    else if (x >= end)
        return 0;
    double ret = h / end * (1 - cos(CH_C_2PI * x / end));
    return ret;
}

double ChFunction_Cycloidal::Get_y_dxdx(double x) const {
    if (x <= 0)
        return 0;
    else if (x >= end)
        return 0;
    double ret = CH_C_2PI * h / end / end * sin(CH_C_2PI * x / end);
    return ret;
}

double ChFunction_Cycloidal::Get_y_dxdxdx(double x) const {
    if (x <= 0)
        return 0;
    else if (x >= end)
        return 0;
    double ret = pow(CH_C_2PI, 2) * h / pow(end, 3) * cos(CH_C_2PI * x / end);
    return ret;
}

void ChFunction_Cycloidal::Set_end(double m_end) {
    if (m_end < 0)
        m_end = 0;
    end = m_end;
}


void ChFunction_Cycloidal::Estimate_x_range(double& xmin, double& xmax) const {
    xmin = 0.0;
    xmax = end;
}

void ChFunction_Cycloidal::ArchiveOut(ChArchiveOut& marchive) {
    // serialize parent class
    ChFunction::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(h);
    marchive << CHNVP(end);
}

void ChFunction_Cycloidal::ArchiveIn(ChArchiveIn& marchive) {
    // deserialize parent class
    ChFunction::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(h);
    marchive >> CHNVP(end);
}

}  // end namespace chrono
