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
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CH_SHAFTS_LOADS_H
#define CH_SHAFTS_LOADS_H

#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChLoad.h"

namespace chrono {

/// Base class for defining loads between a couple of two one-degree-of-freedom parts.
class ChApi ChShaftsLoad : public ChLoadCustomMultiple {
  public:
    ChShaftsLoad(std::shared_ptr<ChShaft> shaft1,  ///< shaft A
                 std::shared_ptr<ChShaft> shaft2   ///< shaft B
    );

    virtual ~ChShaftsLoad() {}

    /// Compute the torque between the two shafts, given relative rotation and velocity.
    /// Torque is assumed applied to shaft B, and its opposite to A.
    /// Inherited classes MUST IMPLEMENT THIS.
    virtual void ComputeShaftShaftTorque(const double rel_rot, const double rel_rot_dt, double& result_torque) = 0;

    // Optional: inherited classes could implement this to avoid the
    // default numerical computation of Jacobians:
    //   virtual void ComputeJacobian(...) // see ChLoad

    /// Return the actual last computed value of the applied torque, assumed applied to 2ns shaft.
    double GetTorque() const { return torque; }

    std::shared_ptr<ChShaft> GetShaft1() const;
    std::shared_ptr<ChShaft> GetShaft2() const;

  protected:
    double torque;  ///< store computed values here

    /// Compute Q, the generalized load. It calls ComputeBodyBodyForceTorque, so in
    /// children classes you do not need to implement it.
    /// Called automatically at each Update().
    virtual void ComputeQ(ChState* state_x,      ///< state position to evaluate Q
                          ChStateDelta* state_w  ///< state speed to evaluate Q
                          ) override;
};

CH_CLASS_VERSION(ChShaftsLoad, 0)

/// Class for defining a torsional spring-damper between two 1D parts.
/// This supersedes the old ChShaftsTorsionSpring (which cannot handle extremely stiff spring values).
class ChApi ChShaftsTorsionSpringDamper : public ChShaftsLoad {
  public:
    ChShaftsTorsionSpringDamper(std::shared_ptr<ChShaft> shaft1,  ///< first shaft
                                std::shared_ptr<ChShaft> shaft2,  ///< second shaft
                                const double stiffness,           ///< torsional stiffness
                                const double damping              ///< torsional damping
    );

    /// "Virtual" copy constructor (covariant return type).
    virtual ChShaftsTorsionSpringDamper* Clone() const override { return new ChShaftsTorsionSpringDamper(*this); }

    /// Set torsional stiffness.
    void SetTorsionalStiffness(const double stiffness) { m_stiffness = stiffness; }

    double GetTorsionalStiffness() const { return m_stiffness; }

    /// Set torsional damping.
    void SetTorsionalDamping(const double damping) { m_damping = damping; }

    double GetTorsionalDamping() const { return m_damping; }

    /// Set the phase shaft1-shaft2 for zero torsion of the spring (default = 0).
    void SetRestPhase(const double phase) { m_rest_phase = phase; }

    double GetRestPhase() const { return m_rest_phase; }

  protected:
    double m_stiffness;
    double m_damping;
    double m_rest_phase;

    virtual bool IsStiff() override { return true; }

    /// Implement the computation of the torque between the two shafts, given relative rotation and velocity.
    /// Torque is assumed applied to shaft B, and opposite to A.
    virtual void ComputeShaftShaftTorque(const double rel_rot, const double rel_rot_dt, double& result_torque) override;
};

/// Elastic gear coupling between two shafts.
class ChApi ChShaftsElasticGear : public ChLoadCustomMultiple {
  public:
    ChShaftsElasticGear(
        std::shared_ptr<ChShaft> shaft1,  ///< first shaft
        std::shared_ptr<ChShaft> shaft2,  ///< second shaft
        const double stiffness,           ///< normal stiffness at teeth contact, tangent direction to primitive
        const double damping,             ///< normal damping at teeth contact, tangent direction to primitive
        const double Ra,                  ///< primitive radius of the gear on shaft A (the radius of B is not needed)
        const double ratio = -1           ///< transmission ratio (negative for outer gear, positive for inner gears)
    );

    /// "Virtual" copy constructor (covariant return type).
    virtual ChShaftsElasticGear* Clone() const override { return new ChShaftsElasticGear(*this); }

    /// Set teeth stiffness, at contact point, in tangent direction to the two primitives.
    void SetTeethStiffness(const double stiffness) { m_stiffness = stiffness; }

    double GetTeethStiffness() const { return m_stiffness; }

    /// Set teeth damping, at contact point, in tangent direction to the two primitives.
    void SetTeethDamping(const double damping) { m_damping = damping; }

    double GetTeethDamping() const { return m_damping; }

    /// Set the phase shaft1-tau*shaft2 for zero compression of the spring (default = 0).
    void SetRestPhase(const double phase) { m_rest_phase = phase; }

    double GetRestPhase() const { return m_rest_phase; }

    /// Set the transmission ratio t, as in w2=t*w1, or t=w2/w1 , or  t*w1 - w2 = 0.
    /// For example, t=1 for a rigid joint; t=-0.5 for representing a couple of spur gears with teeth z1=20 & z2=40;
    /// t=0.1 for a gear with inner teeth (or epicycloidal reducer), etc. Differently from the ideal ChShaftsGear
    /// constraint, this model includes elasticity, so at least the radius of one of the two gear wheels is needed,
    /// namely Ra.
    void SetTransmissionRatioAndRadiusA(double ratio, double Ra);

    /// Set the transmission ratio t, as in w2=t*w1, using the two primitive radii Ra and Rb.
    /// It will be computed as t=Ra/Rb by default, otherwise t=-Ra/Rb if the bigger wheel has inner teeth.
    void SetTransmissionRatioFromRadii(double Ra, double Rb, bool internal = false);

    /// Get the transmission ratio t, as in w2=t*w1, or t=w2/w1.
    double GetTransmissionRatio() const { return m_ratio; }

    /// Get the primitive radius of the gear wheel of shaft1.
    double GetGearRadius1() const { return m_Ra; }

    /// Get the primitive radius of the gear wheel of shaft2.
    double GetGearRadius2() const { return m_Ra * fabs(1.0 / m_ratio); }

    /// Get the last computed contact force, for diagnostics.
    /// The force is assumed tangent to the primitives of the gear wheels,
    /// it can be positive or negative depending on clock/counterclock effect.
    double GetContactForce() const { return m_contact_force; }

  protected:
    double m_stiffness;
    double m_damping;
    double m_rest_phase;
    double m_Ra;
    double m_ratio;
    double m_contact_force;

    virtual bool IsStiff() override { return true; }

    /// Compute Q, the generalized load.
    /// Called automatically at each Update().
    virtual void ComputeQ(ChState* state_x,      ///< state position to evaluate Q
                          ChStateDelta* state_w  ///< state speed to evaluate Q
                          ) override;
};

}  // end namespace chrono

#endif
