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

#include "chrono/motion_functions/ChFunction_ConstAcc.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunction_ConstAcc)

ChFunction_ConstAcc::ChFunction_ConstAcc(double m_h, double m_av, double m_aw, double m_end) : h(m_h) {
    Set_end(m_end);
    Set_avw(m_av, m_aw);
}

ChFunction_ConstAcc::ChFunction_ConstAcc(const ChFunction_ConstAcc& other) {
    h = other.h;
    av = other.av;
    aw = other.aw;
    end = other.end;
}

double ChFunction_ConstAcc::Get_y(double x) const {
    double ret = 0;
    if (x <= 0)
        return 0;
    if (x >= end)
        return h;
    double ev = av * end;
    double ew = aw * end;
    double A = 2 * h / ((ev) * (end - ev + ew));
    double B = 2 * h / ((end - ew) * (end - ev + ew));
    if ((x > 0) && (x < ev)) {
        ret = 0.5 * A * x * x;
    }
    if ((x >= ev) && (x <= ew)) {
        ret = A * ev * (x - ev * 0.5);
    }
    if ((x > ew) && (x < end)) {
        ret = A * ev * (x - ev * 0.5) - B * 0.5 * pow((x - ew), 2);
    }
    return ret;
}

double ChFunction_ConstAcc::Get_y_dx(double x) const {
    double ret = 0;
    double ev = av * end;
    double ew = aw * end;
    double A = 2 * h / ((ev) * (end - ev + ew));
    double B = 2 * h / ((end - ew) * (end - ev + ew));
    if ((x > 0) && (x < ev)) {
        ret = A * x;
    }
    if ((x >= ev) && (x <= ew)) {
        ret = A * ev;
    }
    if ((x > ew) && (x < end)) {
        ret = A * ev - B * (x - ew);
    }
    return ret;
}

double ChFunction_ConstAcc::Get_y_dxdx(double x) const {
    double ret = 0;
    double ev = av * end;
    double ew = aw * end;
    double A = 2 * h / ((ev) * (end - ev + ew));
    double B = 2 * h / ((end - ew) * (end - ev + ew));
    if ((x > 0) && (x < ev)) {
        ret = A;
    }
    if ((x >= ev) && (x <= ew)) {
        ret = 0;
    }
    if ((x > ew) && (x < end)) {
        ret = -B;
    }
    return ret;
}

double ChFunction_ConstAcc::Get_Ca_pos() const {
    return 2 * (end * end) / (av * end * (end - av * end + aw * end));
}

double ChFunction_ConstAcc::Get_Ca_neg() const {
    return 2 * (end * end) / ((end - aw * end) * (end - av * end + aw * end));
}

double ChFunction_ConstAcc::Get_Cv() const {
    return 2 * (end) / (end - av * end + aw * end);
}

void ChFunction_ConstAcc::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChFunction_ConstAcc>();
    // serialize parent class
    ChFunction::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(h);
    marchive << CHNVP(end);
    marchive << CHNVP(aw);
    marchive << CHNVP(av);
}

void ChFunction_ConstAcc::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChFunction_ConstAcc>();
    // deserialize parent class
    ChFunction::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(h);
    marchive >> CHNVP(end);
    marchive >> CHNVP(aw);
    marchive >> CHNVP(av);
}

}  // end namespace chrono
