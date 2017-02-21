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

#ifndef CHLINKFORCE_H
#define CHLINKFORCE_H

#include <cfloat>
#include <cmath>

#include "chrono/core/ChMath.h"
#include "chrono/motion_functions/ChFunction.h"

namespace chrono {

/// Class for forces in link joints of type ChLink().

class ChApi ChLinkForce {
  private:
    bool active;  ///< true/false

    double iforce;             ///< impressed force
    ChFunction* modul_iforce;  ///< time-modulation of imp. force

    double K;             ///< stiffness of the dof
    ChFunction* modul_K;  ///< modulation of K along the dof coord

    double R;             ///< damping of the dof
    ChFunction* modul_R;  ///< modulation of R along the dof coord

  public:
    ChLinkForce();
    ChLinkForce(const ChLinkForce& other);
    ~ChLinkForce();

    /// "Virtual" copy constructor (covariant return type).
    ChLinkForce* Clone() const { return new ChLinkForce(*this); }

    bool Get_active() const { return active; }
    void Set_active(bool m_a) { active = m_a; }

    double Get_iforce() const { return iforce; }
    void Set_iforce(double m_f) { iforce = m_f; }

    double Get_K() const { return K; }
    void Set_K(double m_K) { K = m_K; }

    double Get_R() const { return R; }
    void Set_R(double m_R) { R = m_R; }

    ChFunction* Get_modul_iforce() const { return modul_iforce; }
    ChFunction* Get_modul_K() const { return modul_K; }
    ChFunction* Get_modul_R() const { return modul_R; }

    void Set_modul_iforce(ChFunction* m_funct);
    void Set_modul_K(ChFunction* m_funct);
    void Set_modul_R(ChFunction* m_funct);

    double Get_Kcurrent(double x, double x_dt, double t) const;
    double Get_Rcurrent(double x, double x_dt, double t) const;
    double Get_iFcurrent(double x, double x_dt, double t) const;

    // This is the most important function: it is called to evaluate
    // the internal force at instant t, position x and speed x_dt
    double Get_Force(double x, double x_dt, double t) const;

    /// Method to allow serialization of transient data to archives.
    void ArchiveOUT(ChArchiveOut& marchive);

    /// Method to allow deserialization of transient data from archives.
    void ArchiveIN(ChArchiveIn& marchive);
};

CH_CLASS_VERSION(ChLinkForce,0)

}  // end namespace chrono

#endif
