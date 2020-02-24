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

#ifndef CHLINKLIMIT_H
#define CHLINKLIMIT_H

#include "chrono/core/ChMath.h"
#include "chrono/motion_functions/ChFunction.h"
#include "chrono/solver/ChConstraintTwoBodies.h"

namespace chrono {

/// Class for limits in ChLinkLock joints.
class ChApi ChLinkLimit {
  public:
    ChConstraintTwoBodies constr_upper;
    ChConstraintTwoBodies constr_lower;

    ChLinkLimit();
    ChLinkLimit(const ChLinkLimit& other);
    ~ChLinkLimit() {}

    ChLinkLimit* Clone() const { return new ChLinkLimit(*this); }

    bool IsActive() const { return m_active; }
    void SetActive(bool val) { m_active = val; }

    bool IsPenalty() const { return m_penalty_only; }
    bool IsPolar() const { return m_polar; }
    bool IsRotation() const { return m_rotation; }

    void SetPenalty(bool val) { m_penalty_only = val; }
    void SetPolar(bool val) { m_polar = val; }
    void SetRotation(bool val) { m_rotation = val; }

    double GetMax() const { return m_max; }
    double GetMin() const { return m_min; }
    double GetMaxCushion() const { return m_maxCushion; }
    double GetMinCushion() const { return m_minCushion; }
    double GetKmax() const { return m_Kmax; }
    double GetKmin() const { return m_Kmin; }
    double GetRmax() const { return m_Rmax; }
    double GetRmin() const { return m_Rmin; }
    double GetMaxElastic() const { return m_maxElastic; }
    double GetMinElastic() const { return m_minElastic; }
    double GetMaxPolarAngle(double pol_ang) const;

    void SetMax(double val);
    void SetMin(double val);
    void SetMaxCushion(double val);
    void SetMinCushion(double val);
    void SetKmax(double val) { m_Kmax = val; }
    void SetKmin(double val) { m_Kmin = val; }
    void SetRmax(double val) { m_Rmax = val; }
    void SetRmin(double val) { m_Rmin = val; }
    void SetMaxElastic(double val) { m_maxElastic = val; }
    void SetMinElastic(double val) { m_minElastic = val; }

    void SetModulationKmax(std::shared_ptr<ChFunction> funct) { m_Kmax_modul = funct; }
    void SetModulationKmin(std::shared_ptr<ChFunction> funct) { m_Kmin_modul = funct; }
    void SetModulationRmax(std::shared_ptr<ChFunction> funct) { m_Rmax_modul = funct; }
    void SetModulationRmin(std::shared_ptr<ChFunction> funct) { m_Rmin_modul = funct; }
    void SetPolarMax(std::shared_ptr<ChFunction> funct) { m_polarMax_funct = funct; }

    std::shared_ptr<ChFunction> GetModulationKmax() const { return m_Kmax_modul; }
    std::shared_ptr<ChFunction> GetModulationKmin() const { return m_Kmin_modul; }
    std::shared_ptr<ChFunction> GetModulationRmax() const { return m_Rmax_modul; }
    std::shared_ptr<ChFunction> GetModulationRmin() const { return m_Rmin_modul; }
    std::shared_ptr<ChFunction> GetPolarMax() const { return m_polarMax_funct; }

    /// Return negative violation when x<min, or positive if x>max
    double GetViolation(double x) const;

    double GetForce(double x, double x_dt) const;
    double GetPolarForce(double x, double x_dt, double pol_ang) const;

    /// Method to allow serialization of transient data to archives.
    void ArchiveOUT(ChArchiveOut& marchive);

    /// Method to allow deserialization of transient data from archives.
    void ArchiveIN(ChArchiveIn& marchive);

  private:
    bool m_active;
    bool m_penalty_only;
    bool m_polar;
    bool m_rotation;

    double m_max;
    double m_min;
    double m_maxCushion;
    double m_minCushion;
    double m_Kmax;
    double m_Kmin;
    double m_Rmax;
    double m_Rmin;
    double m_maxElastic;
    double m_minElastic;

    std::shared_ptr<ChFunction> m_Kmax_modul;
    std::shared_ptr<ChFunction> m_Kmin_modul;
    std::shared_ptr<ChFunction> m_Rmax_modul;
    std::shared_ptr<ChFunction> m_Rmin_modul;
    std::shared_ptr<ChFunction> m_polarMax_funct;
};

CH_CLASS_VERSION(ChLinkLimit, 0)

}  // end namespace chrono

#endif
