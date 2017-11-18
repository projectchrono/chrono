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
//
// This file contains a number of ready-to-use loads (ChLoad inherited classes and
// their embedded ChLoader classes) that can be applied to objects of ChBody (and
// inherited classes) type.
// These are 'simplified' tools, that save you from inheriting your custom loads.
// Or just look at these classes and learn how to implement some special type of load.
//
// Example:
//    std::shared_ptr<ChBodyEasyBox> body_test(new ChBodyEasyBox(8,4,4,1000));
//    mphysicalSystem.Add(body_test);
//
//    std::shared_ptr<ChLoadContainer> mforcecontainer (new ChLoadContainer);
//    mphysicalSystem.Add(mforcecontainer);
//
//    std::shared_ptr<ChLoadBodyForce> mforce (new ChLoadBodyForce(body_test, ChVector<>(0,80000,0), false,
//    ChVector<>(8,0,0),true)); mforcecontainer->Add(mforce);
//
//    std::shared_ptr<ChLoadBodyTorque> mtorque (new ChLoadBodyTorque(body_test, ChVector<>(0,0,-80000*8), true));
//    mforcecontainer->Add(mtorque);
//
// =============================================================================

#ifndef CHLOADSBODY_H
#define CHLOADSBODY_H

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLoad.h"

namespace chrono {

/// Load representing a concentrated force acting on a rigid body.
/// The force can rotate together with the body (if in body local coordinates) or not.
/// The application point can follow the body (if in body local coordinates) or not.
class ChApi ChLoadBodyForce : public ChLoadCustom {
  public:
    ChLoadBodyForce(std::shared_ptr<ChBody> mbody,   ///< object to apply load to
                    const ChVector<>& mforce,        ///< force to apply
                    bool mlocal_force,               ///< force is in body local coords
                    const ChVector<>& mapplication,  ///< application point for the force
                    bool mlocal_application = true   ///< application point is in body local coords
    );

    /// Set force assumed to be constant.
    /// It can be expressed in absolute coordinates or body local coordinates
    void SetForce(const ChVector<>& mf, const bool is_local) {
        force = mf;
        local_force = is_local;
    }

    /// Return the force value
    ChVector<> GetForce() const { return force; }

    /// Set the application point of force, assumed to be constant.
    /// It can be expressed in absolute coordinates or body local coordinates
    void SetApplicationPoint(const ChVector<>& ma, const bool is_local) {
        application = ma;
        local_application = is_local;
    }

    /// Return the location of the application point.
    ChVector<> GetApplicationPoint() const { return application; }

    /// Compute Q, the generalized load.
    /// Called automatically at each Update().
    virtual void ComputeQ(ChState* state_x,      ///< state position to evaluate Q
                          ChStateDelta* state_w  ///< state speed to evaluate Q
                          ) override;

  private:
    ChVector<> force;
    ChVector<> application;
    bool local_force;
    bool local_application;

    virtual bool IsStiff() override { return false; }
};

//------------------------------------------------------------------------------------------------

/// Load representing a torque applied to a rigid body.
/// Torque direction does not rotate with the body.
class ChApi ChLoadBodyTorque : public ChLoadCustom {
  public:
    ChLoadBodyTorque(std::shared_ptr<ChBody> mbody,  ///< object to apply load to
                     const ChVector<>& torque,       ///< torque to apply
                     bool mlocal_torque              ///< torque is in body local coords
    );

    /// Set torque, assumed to be constant in space and time.
    void SetTorque(const ChVector<>& mf) { torque = mf; }

    /// Return the current torque value.
    ChVector<> GetTorque() const { return torque; }

    /// Compute Q, the generalized load.
    /// Called automatically at each Update().
    virtual void ComputeQ(ChState* state_x,      ///< state position to evaluate Q
                          ChStateDelta* state_w  ///< state speed to evaluate Q
                          ) override;

  private:
    ChVector<> torque;
    bool local_torque;

    virtual bool IsStiff() override { return false; }
};

//------------------------------------------------------------------------------------------------

/// Base class for wrench loads (a force + a torque) acting between two bodies.
/// See children classes for concrete implementations.
class ChApi ChLoadBodyBody : public ChLoadCustomMultiple {
  public:
    ChLoadBodyBody(std::shared_ptr<ChBody> mbodyA,   ///< body A
                   std::shared_ptr<ChBody> mbodyB,   ///< body B
                   const ChFrame<>& abs_application  ///< location of load element (in abs. coordinates)
    );

    /// Compute the force between the two bodies, in local reference loc_application_B,
    /// given rel_AB, i.e. the position and speed of loc_application_A respect to loc_application_B.
    /// Force is assumed applied to body B, and its opposite to A.
    /// Inherited classes MUST IMPLEMENT THIS.
    virtual void ComputeBushingForceTorque(const ChFrameMoving<>& rel_AB,
                                           ChVector<>& loc_force,
                                           ChVector<>& loc_torque) = 0;

    // Optional: inherited classes could implement this to avoid the
    // default numerical computation of jacobians:
    //   virtual void ComputeJacobian(...) // see ChLoad

    /// For diagnosis purposes, this can return the actual last computed value of
    /// the applied force, expressed in coordinate system of loc_application_B, assumed applied to body B
    ChVector<> GetBushingForce() const { return locB_force; }

    /// For diagnosis purposes, this can return the actual last computed value of
    /// the applied torque, expressed in coordinate system of loc_application_B, assumed applied to body B
    ChVector<> GetBushingTorque() const { return locB_torque; }

    /// Set the application frame of bushing on bodyA
    void SetApplicationFrameA(const ChFrame<>& mpA) { loc_application_A = mpA; }
    ChFrame<> GetApplicationFrameA() const { return loc_application_A; }

    /// Set the application frame of bushing on bodyB
    void SetApplicationFrameB(const ChFrame<>& mpB) { loc_application_B = mpB; }
    ChFrame<> GetApplicationFrameB() const { return loc_application_B; }

    /// Get absolute coordinate of frame A (last computed)
    ChFrameMoving<> GetAbsoluteFrameA() const { return frame_Aw; }

    /// Get absolute coordinate of frame B (last computed)
    ChFrameMoving<> GetAbsoluteFrameB() const { return frame_Bw; }

    std::shared_ptr<ChBody> GetBodyA() const;
    std::shared_ptr<ChBody> GetBodyB() const;

  protected:
    ChFrame<> loc_application_A;  ///< application point on body A (local)
    ChFrame<> loc_application_B;  ///< application point on body B (local)
    ChVector<> locB_force;        ///< store computed values here
    ChVector<> locB_torque;       ///< store computed values here
    ChFrameMoving<> frame_Aw;     ///< for results
    ChFrameMoving<> frame_Bw;     ///< for results

    /// Compute Q, the generalized load. It calls ComputeBushingForceTorque, so in
    /// children classes you do not need to implement it.
    /// Called automatically at each Update().
    virtual void ComputeQ(ChState* state_x,      ///< state position to evaluate Q
                          ChStateDelta* state_w  ///< state speed to evaluate Q
                          ) override;
};

//------------------------------------------------------------------------------------------------

/// Load for a visco-elastic bushing acting between two bodies.
/// It uses three values for stiffness along the X Y Z axes of a coordinate system attached
/// to the second body. This is equivalent to having a bushing with 3x3 diagonal local stiffness matrix.
class ChApi ChLoadBodyBodyBushingSpherical : public ChLoadBodyBody {
  public:
    ChLoadBodyBodyBushingSpherical(
        std::shared_ptr<ChBody> mbodyA,    ///< body A
        std::shared_ptr<ChBody> mbodyB,    ///< body B
        const ChFrame<>& abs_application,  ///< bushing location, in abs. coordinates.
        const ChVector<>& mstiffness,      ///< stiffness, along x y z axes of the abs_application
        const ChVector<>& mdamping         ///< damping, along x y z axes of the abs_application
    );

    /// Set stiffness, along the x y z axes of loc_application_B, es [N/m]
    void SetStiffness(const ChVector<> mstiffness) { stiffness = mstiffness; }
    ChVector<> GetStiffness() const { return stiffness; }

    /// Set damping, along the x y z axes of loc_application_B, es [Ns/m]
    void SetDamping(const ChVector<> mdamping) { damping = mdamping; }
    ChVector<> GetDamping() const { return damping; }

  protected:
    ChVector<> stiffness;
    ChVector<> damping;

    virtual bool IsStiff() override { return true; }

    /// Implement the computation of bushing force, in local
    /// coordinates of the loc_application_B.
    /// Force is assumed applied to body B, and its opposite to A.
    virtual void ComputeBushingForceTorque(const ChFrameMoving<>& rel_AB,
                                           ChVector<>& loc_force,
                                           ChVector<>& loc_torque) override;
};

//------------------------------------------------------------------------------------------------

/// Load for a visco-elasto-plastic bushing acting between two bodies.
/// It uses three values for stiffness along the X Y Z axes of a coordinate system attached
/// to the second body. This is equivalent to having a bushing with 3x3 diagonal local stiffness matrix.
/// Also, it allows a very simple plasticity model, to cap the plastic force on x,y,z given three yelds.
class ChApi ChLoadBodyBodyBushingPlastic : public ChLoadBodyBodyBushingSpherical {
  public:
    ChLoadBodyBodyBushingPlastic(
        std::shared_ptr<ChBody> mbodyA,    ///< body A
        std::shared_ptr<ChBody> mbodyB,    ///< body B
        const ChFrame<>& abs_application,  ///< create the bushing here, in abs. coordinates.
        const ChVector<>& mstiffness,      ///< stiffness, along the x y z axes of the abs_application
        const ChVector<>& mdamping,        ///< damping, along the x y z axes of the abs_application
        const ChVector<>& myield           ///< plastic yield, along the x y z axes of the abs_application
    );

    /// Set plastic yield, forces beyond this limit will be capped.
    /// Expressed along the x y z axes of loc_application_B.
    void SetYeld(const ChVector<> myeld) { yield = myeld; }
    ChVector<> GetYeld() const { return yield; }

    /// Get the current accumulated plastic deformation.
    /// This could become nonzero if forces went beyond the plastic yield.
    ChVector<> GetPlasticDeformation() const { return plastic_def; }

  protected:
    ChVector<> yield;
    ChVector<> plastic_def;

    /// Implement the computation of bushing force, in local
    /// coordinates of the loc_application_B.
    /// Force is assumed applied to body B, and its opposite to A.
    virtual void ComputeBushingForceTorque(const ChFrameMoving<>& rel_AB,
                                           ChVector<>& loc_force,
                                           ChVector<>& loc_torque) override;
};

//------------------------------------------------------------------------------------------------

/// Load for a visco-elastic translational/rotational bushing acting between two bodies.
/// It uses three values for stiffness along the X Y Z axes of a coordinate system attached
/// to the second body , and three rotational stiffness values for (small) rotations about X Y Z of the
/// same coordinate system.
/// This is equivalent to having a bushing with 6x6 diagonal local stiffness matrix.
class ChApi ChLoadBodyBodyBushingMate : public ChLoadBodyBodyBushingSpherical {
  public:
    ChLoadBodyBodyBushingMate(
        std::shared_ptr<ChBody> mbodyA,    ///< body A
        std::shared_ptr<ChBody> mbodyB,    ///< body B
        const ChFrame<>& abs_application,  ///< create the bushing here, in abs. coordinates.
        const ChVector<>& mstiffness,      ///< stiffness, along x y z axes of the abs_application
        const ChVector<>& mdamping,        ///< damping, along x y z axes of the abs_application
        const ChVector<>& mrotstiffness,   ///< rotational stiffness, about x y z axes of the abs_application
        const ChVector<>& mrotdamping      ///< rotational damping, about x y z axes of the abs_application
    );

    /// Set radial stiffness, along the x y z axes of loc_application_B, es [N/m]
    void SetRotationalStiffness(const ChVector<> mstiffness) { rot_stiffness = mstiffness; }
    ChVector<> GetRotationalStiffness() const { return rot_stiffness; }

    /// Set radial damping, along the x y z axes of loc_application_B, es [Ns/m]
    void SetRotationalDamping(const ChVector<> mdamping) { rot_damping = mdamping; }
    ChVector<> GetRotationalDamping() const { return rot_damping; }

  protected:
    ChVector<> rot_stiffness;
    ChVector<> rot_damping;

    /// Implement the computation of bushing force, in local
    /// coordinates of the loc_application_B.
    /// Force is assumed applied to body B, and its opposite to A.
    virtual void ComputeBushingForceTorque(const ChFrameMoving<>& rel_AB,
                                           ChVector<>& loc_force,
                                           ChVector<>& loc_torque) override;
};

//------------------------------------------------------------------------------------------------

/// Load for a visco-elastic translational/rotational bushing acting between two bodies.
/// It uses a full user-defined 6x6 matrix [K] to express the local stiffness of the
/// bushing, assumed expressed in the bushing coordinate system  attached
/// to the second body. A user-defined 6x6 matrix [D] can be defined for damping, as well.
/// Note that this assumes small rotations.
/// Differently from the simpler ChLoadBodyBodyBushingMate and ChLoadBodyBodyBushingSpherical
/// this can represent coupled effects, by using extra-diagonal terms in [K] and/or [D].
class ChApi ChLoadBodyBodyBushingGeneric : public ChLoadBodyBody {
  public:
    ChLoadBodyBodyBushingGeneric(
        std::shared_ptr<ChBody> mbodyA,    ///< body A
        std::shared_ptr<ChBody> mbodyB,    ///< body B
        const ChFrame<>& abs_application,  ///< create the bushing here, in abs. coordinates.
        const ChMatrix<>& mstiffness,      ///< stiffness as a 6x6 matrix, local in the abs_application frame
        const ChMatrix<>& mdamping         ///< damping as a 6x6 matrix, local in the abs_application frame
    );

    /// Set a generic 6x6 stiffness matrix, expressed in local
    /// coordinate system of loc_application_B.
    void SetStiffnessMatrix(const ChMatrix<>& mstiffness) { stiffness = mstiffness; }
    const ChMatrix<>& GetStiffnessMatrix() const { return stiffness; }

    /// Set a generic 6x6 damping matrix, expressed in local
    /// coordinate system of loc_application_B.
    void SetDampingMatrix(const ChMatrix<>& mdamping) { damping = mdamping; }
    const ChMatrix<>& GetDampingMatrix() const { return damping; }

    /// Set the initial pre-load of the bushing, applied to loc_application_A,
    /// expressed in local coordinate system of loc_application_B.
    /// By default it is zero.
    void SetNeutralForce(const ChVector<> mf) { neutral_force = mf; }
    ChVector<> GetNeutralForce() const { return neutral_force; }

    /// Set the initial pre-load torque of the bushing, applied to loc_application_A,
    /// expressed in local coordinate system of loc_application_B.
    /// By default it is zero.
    void SetNeutralTorque(const ChVector<> mt) { neutral_torque = mt; }
    ChVector<> GetNeutralTorque() const { return neutral_torque; }

    /// Set/get the initial pre-displacement of the bushing, as the pre-displacement
    /// of A, expressed in local coordinate system of loc_application_B.
    /// Default behavior is no initial pre-displacement.
    ChFrame<>& NeutralDisplacement() { return neutral_displacement; }

  protected:
    ChMatrixNM<double, 6, 6> stiffness;
    ChMatrixNM<double, 6, 6> damping;

    ChVector<> neutral_force;
    ChVector<> neutral_torque;
    ChFrame<> neutral_displacement;

    virtual bool IsStiff() override { return true; }

    /// Implement the computation of bushing force, in local
    /// coordinates of the loc_application_B.
    /// Force is assumed applied to body B, and its opposite to A.
    virtual void ComputeBushingForceTorque(const ChFrameMoving<>& rel_AB,
                                           ChVector<>& loc_force,
                                           ChVector<>& loc_torque) override;
};

}  // end namespace chrono

#endif
