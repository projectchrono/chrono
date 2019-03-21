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

#ifndef CHLINKFORCE_H
#define CHLINKFORCE_H

#include <cfloat>
#include <cmath>

#include "chrono/core/ChMath.h"
#include "chrono/motion_functions/ChFunction.h"

namespace chrono {

/// Class for forces in link joints of type ChLinkLock.
class ChApi ChLinkForce {
  public:
    ChLinkForce();
    ChLinkForce(const ChLinkForce& other);
    ~ChLinkForce() {}

    /// "Virtual" copy constructor (covariant return type).
    ChLinkForce* Clone() const { return new ChLinkForce(*this); }

    bool IsActive() const { return m_active; }
    void SetActive(bool val) { m_active = val; }

    double GetF() const { return m_F; }
    void SetF(double F) { m_F = F; }

    double GetK() const { return m_K; }
    void SetK(double K) { m_K = K; }

    double GetR() const { return m_R; }
    void SetR(double R) { m_R = R; }

    std::shared_ptr<ChFunction> GetModulationF() const { return m_F_modul; }
    std::shared_ptr<ChFunction> GetModulationK() const { return m_K_modul; }
    std::shared_ptr<ChFunction> GetModulationR() const { return m_R_modul; }

    void SetModulationF(std::shared_ptr<ChFunction> funct) { m_F_modul = funct; }
    void SetModulationK(std::shared_ptr<ChFunction> funct) { m_K_modul = funct; }
    void SetModulationR(std::shared_ptr<ChFunction> funct) { m_R_modul = funct; }

    double GetFcurrent(double x, double x_dt, double t) const;
    double GetKcurrent(double x, double x_dt, double t) const;
    double GetRcurrent(double x, double x_dt, double t) const;

    // This is the most important function: it is called to evaluate
    // the internal force at instant t, position x and speed x_dt
    double GetForce(double x, double x_dt, double t) const;

    /// Method to allow serialization of transient data to archives.
    void ArchiveOUT(ChArchiveOut& marchive);

    /// Method to allow de-serialization of transient data from archives.
    void ArchiveIN(ChArchiveIn& marchive);

  private:
    bool m_active;  ///< true/false

    double m_F;  ///< impressed force
    double m_K;  ///< stiffness of the dof
    double m_R;  ///< damping of the dof

    std::shared_ptr<ChFunction> m_F_modul;  ///< time-modulation of imp. force
    std::shared_ptr<ChFunction> m_K_modul;  ///< modulation of K along the dof coord
    std::shared_ptr<ChFunction> m_R_modul;  ///< modulation of R along the dof coord
};

CH_CLASS_VERSION(ChLinkForce, 0)

}  // end namespace chrono

#endif
