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

#ifndef CHLINKLIMIT_H
#define CHLINKLIMIT_H

#include <cmath>
#include <cfloat>

#include "chrono/core/ChMath.h"
#include "chrono/motion_functions/ChFunction.h"
#include "chrono/physics/ChLinkMask.h"
#include "chrono/solver/ChConstraintTwoBodies.h"

namespace chrono {

/// Class for limits in link joints (for example limits on elbow or knee
/// rotations, etc.)
/// Old code: Must be improved..

class ChApi ChLinkLimit {
  private:
    bool active;  // true/false
    bool penalty_only;
    bool polar;
    bool rotation;
    double max;
    double min;
    double maxCushion;
    double minCushion;
    double Kmax;
    double Kmin;
    double Rmax;
    double Rmin;
    double maxElastic;
    double minElastic;
    ChFunction* modul_Kmax;
    ChFunction* modul_Kmin;
    ChFunction* modul_Rmax;
    ChFunction* modul_Rmin;
    ChFunction* polar_Max;

  public:
    ChConstraintTwoBodies constr_upper;
    ChConstraintTwoBodies constr_lower;

    ChLinkLimit();
    ChLinkLimit(const ChLinkLimit& other);
    ~ChLinkLimit();

    ChLinkLimit* Clone() const { return new ChLinkLimit(*this); }

    bool Get_active() const { return active; }
    bool Get_penalty() const { return penalty_only; }
    bool Get_polar() const { return polar; }
    bool Get_rotation() const { return rotation; }
    double Get_max() const { return max; }
    double Get_min() const { return min; }
    double Get_maxCushion() const { return maxCushion; }
    double Get_minCushion() const { return minCushion; }
    double Get_Kmax() const { return Kmax; }
    double Get_Kmin() const { return Kmin; }
    double Get_Rmax() const { return Rmax; }
    double Get_Rmin() const { return Rmin; }
    double Get_maxElastic() const { return maxElastic; }
    double Get_minElastic() const { return minElastic; }
    ChFunction* GetModul_Kmax() const { return modul_Kmax; }
    ChFunction* GetModul_Kmin() const { return modul_Kmin; }
    ChFunction* GetModul_Rmax() const { return modul_Rmax; }
    ChFunction* GetModul_Rmin() const { return modul_Rmin; }
    ChFunction* GetPolar_Max() const { return polar_Max; }
    double Get_polar_max(double pol_ang) const;

    void Set_active(bool m_active) { active = m_active; }
    void Set_penalty(bool m_active) { penalty_only = m_active; }
    void Set_polar(bool m_pol) { polar = m_pol; }
    void Set_rotation(bool m_rot) { rotation = m_rot; }
    void Set_max(double m_max);
    void Set_min(double m_min);
    void Set_maxCushion(double m_maxCushion);
    void Set_minCushion(double m_minCushion);
    void Set_Kmax(double m_K) { Kmax = m_K; }
    void Set_Kmin(double m_K) { Kmin = m_K; }
    void Set_Rmax(double m_R) { Rmax = m_R; }
    void Set_Rmin(double m_R) { Rmin = m_R; }
    void Set_maxElastic(double m_e) { maxElastic = m_e; }
    void Set_minElastic(double m_e) { minElastic = m_e; }
    void SetModul_Kmax(ChFunction* m_funct);
    void SetModul_Kmin(ChFunction* m_funct);
    void SetModul_Rmax(ChFunction* m_funct);
    void SetModul_Rmin(ChFunction* m_funct);
    void SetPolar_Max(ChFunction* m_funct);

    /// Return negative violation when x<min, or positive if x>max
    double GetViolation(double x) const;

    double GetForce(double x, double x_dt) const;
    double GetPolarForce(double x, double x_dt, double pol_ang) const;

    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    void ArchiveOUT(ChArchiveOut& marchive);

    /// Method to allow deserialization of transient data from archives.
    void ArchiveIN(ChArchiveIn& marchive);
};

CH_CLASS_VERSION(ChLinkLimit,0)


}  // end namespace chrono

#endif
