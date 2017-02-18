// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/physics/ChLimit.h"

namespace chrono {

ChLinkLimit::ChLinkLimit()
    : active(false),
      penalty_only(false),
      polar(false),
      rotation(false),
      max(1),
      min(-1),
      maxCushion(0),
      minCushion(0),
      Kmax(1000),
      Kmin(1000),
      Rmax(100),
      Rmin(100),
      minElastic(0),
      maxElastic(0) {
    modul_Kmax = new ChFunction_Const(1);  // default: const.modulation of K
    modul_Kmin = new ChFunction_Const(1);  // default: const.modulation of K
    modul_Rmax = new ChFunction_Const(1);  // default: const.modulation of K
    modul_Rmin = new ChFunction_Const(1);  // default: const.modulation of K
    polar_Max = new ChFunction_Const(1);   // default function for polar limits
    constr_upper.SetMode(CONSTRAINT_UNILATERAL);
    constr_lower.SetMode(CONSTRAINT_UNILATERAL);
}

ChLinkLimit::ChLinkLimit(const ChLinkLimit& other) {
    active = other.active;
    penalty_only = other.penalty_only;
    polar = other.polar;
    rotation = other.rotation;
    max = other.max;
    min = other.min;
    maxCushion = other.maxCushion;
    minCushion = other.minCushion;
    Kmax = other.Kmax;
    Kmin = other.Kmin;
    Rmax = other.Rmax;
    Rmin = other.Rmin;
    minElastic = other.minElastic;
    maxElastic = other.maxElastic;
    // replace functions:
    modul_Kmax = other.modul_Kmax->Clone();
    modul_Kmin = other.modul_Kmin->Clone();
    modul_Rmax = other.modul_Rmax->Clone();
    modul_Rmin = other.modul_Rmin->Clone();
    polar_Max = other.polar_Max->Clone();
}

ChLinkLimit::~ChLinkLimit() {
    delete modul_Kmax;
    delete modul_Kmin;
    delete modul_Rmax;
    delete modul_Rmin;
    delete polar_Max;
}

void ChLinkLimit::Set_max(double m_max) {
    max = m_max;
    if (max < min)
        min = max;
    if ((max - maxCushion) < min)
        maxCushion = max - min;
    if ((max - maxCushion) < (min + minCushion))
        minCushion = max - min - maxCushion;
    constr_upper.SetActive(true);
}

void ChLinkLimit::Set_min(double m_min) {
    min = m_min;
    if (min > max)
        max = min;
    if ((min + minCushion) > max)
        minCushion = max - min;
    if ((min + minCushion) > (max - maxCushion))
        maxCushion = max - min - minCushion;
    constr_lower.SetActive(true);
}

void ChLinkLimit::Set_maxCushion(double m_maxCushion) {
    maxCushion = m_maxCushion;
    if ((max - maxCushion) < min)
        maxCushion = max - min;
    if ((max - maxCushion) < (min + minCushion))
        minCushion = max - min - maxCushion;
}

void ChLinkLimit::Set_minCushion(double m_minCushion) {
    minCushion = m_minCushion;
    if ((min + minCushion) > max)
        minCushion = max - min;
    if ((min + minCushion) > (max - maxCushion))
        maxCushion = max - min - minCushion;
}

void ChLinkLimit::SetModul_Kmax(ChFunction* m_funct) {
    if (modul_Kmax)
        delete modul_Kmax;
    modul_Kmax = m_funct;
}
void ChLinkLimit::SetModul_Kmin(ChFunction* m_funct) {
    if (modul_Kmin)
        delete modul_Kmin;
    modul_Kmin = m_funct;
}
void ChLinkLimit::SetModul_Rmax(ChFunction* m_funct) {
    if (modul_Rmax)
        delete modul_Rmax;
    modul_Rmax = m_funct;
}
void ChLinkLimit::SetModul_Rmin(ChFunction* m_funct) {
    if (modul_Rmin)
        delete modul_Rmin;
    modul_Rmin = m_funct;
}
void ChLinkLimit::SetPolar_Max(ChFunction* m_funct) {
    if (polar_Max)
        delete polar_Max;
    polar_Max = m_funct;
}

// file parsing / dumping

void ChLinkLimit::ArchiveOUT(ChArchiveOut& marchive) {
    // class version number
    marchive.VersionWrite<ChLinkLimit>();
    // serialize parent class too

    // stream out all member data
    marchive << CHNVP(active);
    marchive << CHNVP(penalty_only);
    marchive << CHNVP(polar);
    marchive << CHNVP(rotation);
    marchive << CHNVP(max);
    marchive << CHNVP(min);
    marchive << CHNVP(maxCushion);
    marchive << CHNVP(minCushion);
    marchive << CHNVP(Kmax);
    marchive << CHNVP(Kmin);
    marchive << CHNVP(Rmax);
    marchive << CHNVP(Rmin);
    marchive << CHNVP(maxElastic);
    marchive << CHNVP(minElastic);
    marchive << CHNVP(modul_Kmax);
    marchive << CHNVP(modul_Kmin);
    marchive << CHNVP(modul_Rmax);
    marchive << CHNVP(modul_Rmin);
    marchive << CHNVP(polar_Max);
}

void ChLinkLimit::ArchiveIN(ChArchiveIn& marchive) {
    // class version number
    int version = marchive.VersionRead<ChLinkLimit>();
    // deserialize parent class too

    // stream in all member data
    marchive >> CHNVP(active);
    marchive >> CHNVP(penalty_only);
    marchive >> CHNVP(polar);
    marchive >> CHNVP(rotation);
    marchive >> CHNVP(max);
    marchive >> CHNVP(min);
    marchive >> CHNVP(maxCushion);
    marchive >> CHNVP(minCushion);
    marchive >> CHNVP(Kmax);
    marchive >> CHNVP(Kmin);
    marchive >> CHNVP(Rmax);
    marchive >> CHNVP(Rmin);
    marchive >> CHNVP(maxElastic);
    marchive >> CHNVP(minElastic);
    marchive >> CHNVP(modul_Kmax);
    marchive >> CHNVP(modul_Kmin);
    marchive >> CHNVP(modul_Rmax);
    marchive >> CHNVP(modul_Rmin);
    marchive >> CHNVP(polar_Max);
}

///////////////////

double ChLinkLimit::GetViolation(double x) const {
    if (!active || penalty_only) {
        return 0;
    } else {
        if ((x > min) && (x < max))
            return 0;
        if (x <= min)
            return (x - min);
        if (x >= max)
            return (x - max);
    }
    return 0;
}

double ChLinkLimit::GetForce(double x, double x_dt) const {
    double cush_coord;
    double cush_coord_norm;
    double force;
    double m_min, m_max;

    if (!penalty_only) {
        m_min = min;
        m_max = max;
    } else {
        m_min = -999999999;
        m_max = 999999999;
    }

    if ((x > m_min) && (x < (min + minCushion))) {
        cush_coord = (min + minCushion) - x;

        if (minCushion >= 0.0000001)
            cush_coord_norm = cush_coord / minCushion;
        else
            cush_coord_norm = 1;

        if (cush_coord_norm > 1)
            cush_coord_norm = 1;  // clip cushion forces at stopper limit

        force = cush_coord * Kmin * modul_Kmin->Get_y(cush_coord_norm);
        force += (-x_dt) * Rmin * modul_Rmin->Get_y(cush_coord_norm);
        if (force < 0) {
            force = 0;
        }  // damping could cause neg force while going away,
           // so -as the limit is not "sticky"- clip force sign.

        return (force);
    }

    if ((x < m_max) && (x > (max - maxCushion))) {
        cush_coord = x - (max - maxCushion);

        if (maxCushion >= 0.0000001)
            cush_coord_norm = cush_coord / maxCushion;
        else
            cush_coord_norm = 1;

        if (cush_coord_norm > 1)
            cush_coord_norm = 1;  // clip cushion forces at stopper limit

        force = (-cush_coord) * Kmax * modul_Kmax->Get_y(cush_coord_norm);
        force += (-x_dt) * Rmax * modul_Rmax->Get_y(cush_coord_norm);
        if (force > 0) {
            force = 0;
        }  // damping could cause pos force while going away,
           // so -as the limit is not "sticky"- clip force sign.
        return (force);
    }
    return 0;
}

////

double ChLinkLimit::Get_polar_max(double pol_ang) const {
    if (!polar_Max)
        return 0.001;
    return polar_Max->Get_y(pol_ang);
}

//// The same, but for conical limits, in polar coordinates

double ChLinkLimit::GetPolarForce(double x, double x_dt, double pol_ang) const {
    double cush_coord;
    double cush_coord_norm;
    double cushion_thick;
    double force;
    double m_max;
    double ang_max;

    if (!polar_Max)
        return 0;

    if (!penalty_only) {
        m_max = max;
    } else {
        m_max = 999999999;
    }

    ang_max = polar_Max->Get_y(pol_ang);

    if ((x < m_max) && (x > (ang_max - maxCushion))) {
        cushion_thick = maxCushion;
        if (cushion_thick > ang_max)
            cushion_thick = ang_max;

        cush_coord = x - (ang_max - maxCushion);

        if (cushion_thick >= 0.0000001)
            cush_coord_norm = cush_coord / cushion_thick;
        else
            cush_coord_norm = 1;

        if (cush_coord_norm > 1)
            cush_coord_norm = 1;  // clip cushion forces at stopper limit

        force = (-cush_coord) * Kmax * modul_Kmax->Get_y(cush_coord_norm);
        force += (-x_dt) * Rmax * modul_Rmax->Get_y(cush_coord_norm);
        if (force > 0) {
            force = 0;
        }  // damping could cause pos force while going away,
           // so -as the limit is not "sticky"- clip force sign.
        return (force);
    }
    return 0;
}

}  // end namespace chrono
