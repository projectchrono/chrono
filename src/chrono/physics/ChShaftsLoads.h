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

#ifndef CHSHAFTSLOADS_H
#define CHSHAFTSLOADS_H

#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChLoad.h"

namespace chrono {




/// Base class for defining loads between a couple of two one-degree-of-freedom
/// parts; i.e., shafts that can be used to build 1D models of powertrains.

class ChApi ChShaftsLoad : public ChLoadCustomMultiple {
  public:
    ChShaftsLoad(std::shared_ptr<ChShaft> shaftA,  ///< shaft A
                     std::shared_ptr<ChShaft> shaftB   ///< shaft B
    );

    /// Compute the torque between the two shafts, given relative rotation and velocity.
    /// Torque is assumed applied to shaft B, and its opposite to A.
    /// Inherited classes MUST IMPLEMENT THIS.
    virtual void ComputeShaftShaftTorque(     const double rel_rot,
                                              const double rel_rot_dt,
											  double& result_torque) = 0;

    // Optional: inherited classes could implement this to avoid the
    // default numerical computation of jacobians:
    //   virtual void ComputeJacobian(...) // see ChLoad

    /// For diagnosis purposes, this can return the actual last computed value of
    /// the applied torque, assumed applied to body B
    double GetTorque() const { return torque; }

    std::shared_ptr<ChShaft> GetShaftA() const;
    std::shared_ptr<ChShaft> GetShaftB() const;

  protected:
    double torque;        ///< store computed values here

    /// Compute Q, the generalized load. It calls ComputeBodyBodyForceTorque, so in
    /// children classes you do not need to implement it.
    /// Called automatically at each Update().
    virtual void ComputeQ(ChState* state_x,      ///< state position to evaluate Q
                          ChStateDelta* state_w  ///< state speed to evaluate Q
                          ) override;
};

CH_CLASS_VERSION(ChShaftsLoad, 0)



/// Class for defining a torsional spring-damper between two 1D parts;
/// i.e., shafts that can be used to build 1D models of powertrains. This is
/// more efficient than simulating power trains modeled with full 3D ChBody
/// objects.
/// This supersedes the old ChShaftsTorsionSpring (which cannot handle
/// extremely stiff spring values)

class ChApi ChShaftsTorsionSpringDamper : public ChShaftsLoad {
  public:
    ChShaftsTorsionSpringDamper(
        std::shared_ptr<ChShaft> mbodyA,    ///< shaft A
        std::shared_ptr<ChShaft> mbodyB,    ///< shaft B
        const double mstiffness,      ///< torsional stiffness
        const double mdamping         ///< torsional damping
    );

    /// "Virtual" copy constructor (covariant return type).
    virtual ChShaftsTorsionSpringDamper* Clone() const override { return new ChShaftsTorsionSpringDamper(*this); }

    /// Set torsional stiffness, es. [Nm/rad]
    void SetTorsionalStiffness(const double mstiffness) { stiffness = mstiffness; }
    double GetTorsionalStiffness() const { return stiffness; }

    /// Set torsional damping, es [Nm s/rad]
    void SetTorsionalDamping(const double mdamping) { damping = mdamping; }
    double GetTorsionalDamping() const { return damping; }

	/// Set the phase shaftA-shaftB for zero torsion of the spring (default = 0 [rad])
    void SetRestPhase(const double mphase) { rest_phase = mphase; }
    double GetRestPhase() const { return rest_phase; }

  protected:
    double stiffness;
    double damping;
    double rest_phase;

    virtual bool IsStiff() override { return true; }

    /// Implement the computation of the torque between the two shafts, 
	/// given relative rotation and velocity.
    /// Torque is assumed applied to shaft B, and it's opposite to A.
    virtual void ComputeShaftShaftTorque(const double rel_rot, const double rel_rot_dt, double& result_torque) override;
};


/// Class for defining a torsional spring-damper between two 1D parts;
/// i.e., shafts that can be used to build 1D models of powertrains. This is
/// more efficient than simulating power trains modeled with full 3D ChBody
/// objects.
/// This supersedes the old ChShaftsTorsionSpring (which cannot handle
/// extremely stiff spring values)

class ChApi ChShaftsElasticGear : public ChLoadCustomMultiple {
  public:
    ChShaftsElasticGear(std::shared_ptr<ChShaft> mbodyA,  ///< shaft A
                                std::shared_ptr<ChShaft> mbodyB,  ///< shaft B
                              const double mstiffness,      ///< normal stiffness at teeth contact, tangent direction to primitive
                              const double mdamping,        ///< normal damping at teeth contact, tangent direction to primitive
							  const double mRa,				///< primitive radius of the gear on shaft A (the radius of B is not needed)
							  const double mratio = -1       ///< transmission ratio (negative for outer gear, positive for inner gears)
    );

    /// "Virtual" copy constructor (covariant return type).
    virtual ChShaftsElasticGear* Clone() const override { return new ChShaftsElasticGear(*this); }

    /// Set teeth stiffness, at contact point, in tangent direction to the two primitives. Es. [N/m]
    void SetTeethStiffness(const double mstiffness) { stiffness = mstiffness; }
    double GetTeethStiffness() const { return stiffness; }

    /// Set teeth damping, at contact point, in tangent direction to the two primitives. Es. [Ns/m]
    void SetTeethDamping(const double mdamping) { damping = mdamping; }
    double GetTeethDamping() const { return damping; }

	/// Set the phase shaftA-tau*shaftB for zero compression of the spring (default = 0 [rad])
    void SetRestPhase(const double mphase) { rest_phase = mphase; }
    double GetRestPhase() const { return rest_phase; }


	/// Set the transmission ratio t, as in w2=t*w1, or t=w2/w1 , or  t*w1 - w2 = 0.
    /// For example, t=1 for a rigid joint; t=-0.5 for representing
    /// a couple of spur gears with teeth z1=20 & z2=40; t=0.1 for
    /// a gear with inner teeth (or epicycloidal reducer), etc.
	/// Differently from the ideal ChShaftsGear constraint, this model includes 
	/// elasticity, so at least the radius of one of the two gear wheels is needed, namely Ra.
    void SetTransmissionRatioAndRadiusA(double mt, double mRa) { ratio = mt; Ra = mRa; }

	/// Set the transmission ratio t, as in w2=t*w1, using the two primitive radii Ra and Rb.
	/// It will be computed as t=Ra/Rb by default, otherwise t=-Ra/Rb if the bigger wheel has inner teeth.
    void SetTransmissionRatioFromRadii(double mRa, double mRb, bool minternal = false) {
        Ra = mRa;
        ratio = fabs(mRa/mRb);
        if (minternal)
            ratio = -ratio;
    }

    /// Get the transmission ratio t, as in w2=t*w1, or t=w2/w1
    double GetTransmissionRatio() const { return ratio; }

	/// Get the primitive radius of the gear wheel of shaftA
    double GetGearRadiusA() const { return Ra; }

	/// Get the primitive radius of the gear wheel of shaftB
    double GetGearRadiusB() const { return Ra * fabs(1.0/ratio); }

	/// Get the last computed contact force, for diagnostics.
	/// The force is assumed tangent to the primitives of the gear wheels,
	/// it can be positive or negative depending on clock/counterclock effect.
    double GetContactForce() const { return contact_force; }


  protected:
    double stiffness;
    double damping;
    double rest_phase;
    double Ra;
    double ratio;
    double contact_force;

    virtual bool IsStiff() override { return true; }


	/// Compute Q, the generalized load.
    /// Called automatically at each Update().
    virtual void ComputeQ(ChState* state_x,      ///< state position to evaluate Q
                          ChStateDelta* state_w  ///< state speed to evaluate Q
                          ) override;
};







}  // end namespace chrono

#endif
