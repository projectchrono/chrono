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

#include "chrono/core/ChFrame.h"
#include "chrono/functions/ChFunction.h"

namespace chrono {

/// Class for forces in link joints of type ChLinkLock.
/// Force is applied to specific degrees of freedom of the ChLinkLock through ChLinkLock::Force functions.
/// The resulting force (or torque) is computed as the sum of actuator, spring force and a damping force.
/// Each term consists of a constant part (set through SetActuatorForce, SetSpringCoefficient, SetDampingCoefficient)
/// and an optional modulation part (set through SetActuatorModulation, SetSpringModulation, SetDamperModulation).
/// The resulting actuation force, the spring and damping coefficient are given as the constant part multiplied by the
/// value of the modulation function at the current time.
/// The final force is obtained as the sum of all the contributions:
/// `    F(x, x_dt, t) = F_act_const * F_act_modulation - K_const * K_modulation * x - R_const * R_modulation * x_dt`
class ChApi ChLinkForce {
  public:
    ChLinkForce();
    ChLinkForce(const ChLinkForce& other);
    ~ChLinkForce() {}

    /// "Virtual" copy constructor (covariant return type).
    ChLinkForce* Clone() const { return new ChLinkForce(*this); }

    bool IsActive() const { return m_active; }
    void SetActive(bool val) { m_active = val; }

    /// Get the force or torque applied to the link.
    double GetActuatorForceTorque() const { return m_F; }

    /// Set the force or torque applied to the link.
    void SetActuatorForceTorque(double F) { m_F = F; }

    double GetSpringCoefficient() const { return m_K; }
    void SetSpringCoefficient(double K) { m_K = K; }

    double GetDampingCoefficient() const { return m_R; }
    void SetDampingCoefficient(double R) { m_R = R; }

    std::shared_ptr<ChFunction> GetActuatorModulation() const { return m_F_modul; }
    std::shared_ptr<ChFunction> GetSpringModulation() const { return m_K_modul; }
    std::shared_ptr<ChFunction> GetDamperModulation() const { return m_R_modul; }

    void SetActuatorModulation(std::shared_ptr<ChFunction> funct) { m_F_modul = funct; }
    void SetSpringModulation(std::shared_ptr<ChFunction> funct) { m_K_modul = funct; }
    void SetDamperModulation(std::shared_ptr<ChFunction> funct) { m_R_modul = funct; }

    double GetCurrentActuatorForceTorque(double x, double x_dt, double t) const;
    double GetCurrentSpringCoefficient(double x, double x_dt, double t) const;
    double GetCurrentDampingCoefficient(double x, double x_dt, double t) const;

    // This is the most important function: it is called to evaluate
    // the internal force at instant t, position x and speed x_dt
    double GetForceTorque(double x, double x_dt, double t) const;

    /// Method to allow serialization of transient data to archives.
    void ArchiveOut(ChArchiveOut& archive_out);

    /// Method to allow de-serialization of transient data from archives.
    void ArchiveIn(ChArchiveIn& archive_in);

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
