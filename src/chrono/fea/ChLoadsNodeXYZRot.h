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
// This file contains a number of ready-to-use loads (ChLoad inherited classes
// and their embedded ChLoader classes) that can be applied to objects of
// ChNodeFEAxyzrot (and inherited classes) type.
// These are 'simplified' tools, that save from inheriting custom loads.
// =============================================================================

#ifndef CH_LOADS_NODE_XYZROT_H
#define CH_LOADS_NODE_XYZROT_H

#include "chrono/physics/ChLoad.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"
#include "chrono/physics/ChBody.h"
#include "chrono/functions/ChFunction.h"

namespace chrono {
namespace fea {

//  LOADS ON:   ChNodeFEAxyzrot

/// Base class for loads representing a concentrated wrench (force + torque) acting on a ChNodeFEAxyzrot.
/// Users should inherit from this and implement custom ComputeForceTorque(), this is enough to have the load working.
/// Note: some predefined examples are provided, e.g. ChLoadNodeXYZRotForceAbs.
class ChApi ChLoadNodeXYZRot : public ChLoadCustom {
  public:
    ChLoadNodeXYZRot(std::shared_ptr<ChNodeFEAxyzrot> node  ///< node to apply load to
    );

    /// Compute the force and torque on the node, in absolute coordsystem,
    /// given absolute position and speed of node passed via node_frame_abs_pos_vel.
    virtual void ComputeForceTorque(const ChFrameMoving<>& node_frame_abs_pos_vel,
                                    ChVector3d& abs_force,
                                    ChVector3d& abs_torque) = 0;

    // Optional: inherited classes could implement this to avoid the
    // default numerical computation of Jacobians:
    //   virtual void ComputeJacobian(...) // see ChLoad

    /// Compute Q, the generalized load.
    /// Called automatically at each Update().
    virtual void ComputeQ(ChState* state_x,      ///< state position to evaluate Q
                          ChStateDelta* state_w  ///< state speed to evaluate Q
                          ) override;

    /// For diagnosis purposes, this can return the actual last computed value of
    /// the applied force, expressed in absolute coordinate system, assumed applied to node
    ChVector3d GetForce() const { return computed_abs_force; }

    /// For diagnosis purposes, this can return the actual last computed value of
    /// the applied torque, expressed in absolute coordinate system, assumed applied to node
    ChVector3d GetTorque() const { return computed_abs_torque; }

  protected:
    /// Inherited classes could override this and return true, if the load benefits from a Jacobian
    /// when using implicit integrators.
    virtual bool IsStiff() override { return false; }

    virtual void Update(double time, bool update_assets) override;

    ChVector3d computed_abs_force;
    ChVector3d computed_abs_torque;
};

/// Load representing a concentrated force acting on a ChNodeFEAxyzrot.
/// The load can be provided as a constant force or as a function to modulate it with time.
/// As it is constant in space, IsStiff() is set to false.
class ChApi ChLoadNodeXYZRotForceAbs : public ChLoadNodeXYZRot {
  public:
    ChLoadNodeXYZRotForceAbs(std::shared_ptr<ChNodeFEAxyzrot> node,  ///< node to apply load to
                             const ChVector3d& force                 ///< applied force in absolute coordsys
    );

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLoadNodeXYZRotForceAbs* Clone() const override { return new ChLoadNodeXYZRotForceAbs(*this); }

    /// Compute the force on the node, in absolute coordsystem,
    /// given absolute position and speed of node passed via node_frame_abs_pos_vel.
    virtual void ComputeForceTorque(const ChFrameMoving<>& node_frame_abs_pos_vel,
                                    ChVector3d& abs_force,
                                    ChVector3d& abs_torque) override;

    /// Set the applied force vector, expressed in absolute coordinates.
    /// This is assumed constant, unless a non-constant scaling time function is provided.
    void SetForceBase(const ChVector3d& force);

    /// Return the current force vector (scaled by the current modulation value).
    ChVector3d GetForce() const;

    /// Set modulation function.
    /// This is a function of time which (optionally) modulates the specified applied force.
    /// By default the modulation is a constant function, always returning a value of 1.
    void SetModulationFunction(std::shared_ptr<ChFunction> modulation) { m_modulation = modulation; }

  protected:
    ChVector3d m_force_base;  ///< base force value

    std::shared_ptr<ChFunction> m_modulation;  ///< modulation function of time
    double m_scale;                            ///< scaling factor (current modulation value)

    virtual bool IsStiff() override { return false; }

    virtual void Update(double time, bool update_assets) override;
};

////////////////////////////////////////////////////////////////////////////
//
//  LOADS ON:   ChNodeFEAxyzrot ---- ChNodeFEAxyzrot

/// Base class for loads representing a concentrated wrench (force & torque) acting between two ChNodeFEAxyzrot.
/// The force & torque is applied between two local references attached to the two nodes, loc_application_A and
/// loc_application_B, not necessarily centered in the respective nodes. Users should inherit from this and implement a
/// custom ComputeForceTorque(), this is enough to have the load working. Note: there are already some predefined
/// examples, ie. children classes with common concrete implementations, such as
/// ChLoadNodeXYZRotNodeXYZRotBushingSpherical ChLoadNodeXYZRotNodeXYZRotBushingMate, etc.
class ChApi ChLoadNodeXYZRotNodeXYZRot : public ChLoadCustomMultiple {
  public:
    ChLoadNodeXYZRotNodeXYZRot(std::shared_ptr<ChNodeFEAxyzrot> nodeA,  ///< node A to apply load to
                               std::shared_ptr<ChNodeFEAxyzrot> nodeB,  ///< node B to apply load to, as reaction
                               const ChFrame<>& abs_application  ///< location of load element in abs. coordinates
    );

    /// Compute the wrench (force & torque) between the two nodes, expressed in local frame of loc_application_B,
    /// given rel_AB, i.e. the position and speed of loc_application_A respect to loc_application_B, expressed in frame
    /// of loc_application_B. Force is assumed applied to loc_application_B, and its opposite reaction to A.
    virtual void ComputeForceTorque(const ChFrameMoving<>& rel_AB, ChVector3d& loc_force, ChVector3d& loc_torque) = 0;

    /// Compute Q, the generalized load.
    /// Called automatically at each Update().
    virtual void ComputeQ(ChState* state_x,      ///< state position to evaluate Q
                          ChStateDelta* state_w  ///< state speed to evaluate Q
                          ) override;

    // Optional: inherited classes could implement this to avoid the
    // default numerical computation of Jacobians:
    //   virtual void ComputeJacobian(...) // see ChLoad

    /// For diagnosis purposes, this can return the actual last computed value of
    /// the applied force, expressed in coordinate system of loc_application_B, assumed applied to body B
    ChVector3d GetForce() const { return locB_force; }

    /// For diagnosis purposes, this can return the actual last computed value of
    /// the applied torque, expressed in coordinate system of loc_application_B, assumed applied to body B
    ChVector3d GetTorque() const { return locB_torque; }

    /// Set the application frame of bushing on bodyA
    void SetApplicationFrameA(const ChFrame<>& pA) { loc_application_A = pA; }
    ChFrame<> GetApplicationFrameA() const { return loc_application_A; }

    /// Set the application frame of bushing on bodyB
    void SetApplicationFrameB(const ChFrame<>& pB) { loc_application_B = pB; }
    ChFrame<> GetApplicationFrameB() const { return loc_application_B; }

    /// Get absolute coordinate of frame A (last computed)
    ChFrameMoving<> GetAbsoluteFrameA() const { return frame_Aw; }

    /// Get absolute coordinate of frame B (last computed)
    ChFrameMoving<> GetAbsoluteFrameB() const { return frame_Bw; }

    std::shared_ptr<ChNodeFEAxyzrot> GetNodeA() const;
    std::shared_ptr<ChNodeFEAxyzrot> GetNodeB() const;

  protected:
    ChFrame<> loc_application_A;  ///< application point on body A (local)
    ChFrame<> loc_application_B;  ///< application point on body B (local)
    ChVector3d locB_force;        ///< store computed values here
    ChVector3d locB_torque;       ///< store computed values here
    ChFrameMoving<> frame_Aw;     ///< for results
    ChFrameMoving<> frame_Bw;     ///< for results
};

//------------------------------------------------------------------------------------------------

/// Load for a visco-elastic bushing acting between two bodies.
/// The bushing is between two local references attached to the two nodes, loc_application_A and loc_application_B,
/// not necessarily centered in the respective nodes.
/// It uses three values for stiffness along the X Y Z axes of a coordinate system attached
/// to the second body. This is equivalent to having a bushing with 3x3 diagonal local stiffness matrix.
class ChApi ChLoadNodeXYZRotNodeXYZRotBushingSpherical : public ChLoadNodeXYZRotNodeXYZRot {
  public:
    ChLoadNodeXYZRotNodeXYZRotBushingSpherical(
        std::shared_ptr<ChNodeFEAxyzrot> nodeA,  ///< node A
        std::shared_ptr<ChNodeFEAxyzrot> nodeB,  ///< node B
        const ChFrame<>& abs_application,        ///< create bushing here, in abs. coordinates
        const ChVector3d& stiffness_coefs,       ///< stiffness, along x y z axes of the abs_application
        const ChVector3d& damping_coefs          ///< damping, along x y z axes of the abs_application
    );

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLoadNodeXYZRotNodeXYZRotBushingSpherical* Clone() const override {
        return new ChLoadNodeXYZRotNodeXYZRotBushingSpherical(*this);
    }

    /// Set stiffness, along the x y z axes of frame of loc_application_B.
    void SetStiffness(const ChVector3d stiffness_coefs) { stiffness = stiffness_coefs; }

    ChVector3d GetStiffness() const { return stiffness; }

    /// Set damping, along the x y z axes of frame of loc_application_B.
    void SetDamping(const ChVector3d damping_coefs) { damping = damping_coefs; }

    ChVector3d GetDamping() const { return damping; }

  protected:
    ChVector3d stiffness;
    ChVector3d damping;

    virtual bool IsStiff() override { return true; }

    /// Compute the wrench (force & torque) between the two nodes, expressed in local frame of loc_application_B, given
    /// rel_AB. rel_AB is the position and speed of loc_application_A respect to loc_application_B, expressed in frame
    /// of loc_application_B. Force is assumed applied to loc_application_B, and its opposite reaction to A.
    virtual void ComputeForceTorque(const ChFrameMoving<>& rel_AB,
                                    ChVector3d& loc_force,
                                    ChVector3d& loc_torque) override;
};

//------------------------------------------------------------------------------------------------

/// Load for a visco-elasto-plastic bushing acting between two ChNodeFEAxyzrot.
/// It uses three values for stiffness along the X Y Z axes of a coordinate system attached
/// to the second node. This is equivalent to having a bushing with 3x3 diagonal local stiffness matrix.
/// Also, it allows a very simple plasticity model, to cap the plastic force on x,y,z given three yelds.
class ChApi ChLoadNodeXYZRotNodeXYZRotBushingPlastic : public ChLoadNodeXYZRotNodeXYZRotBushingSpherical {
  public:
    ChLoadNodeXYZRotNodeXYZRotBushingPlastic(
        std::shared_ptr<ChNodeFEAxyzrot> nodeA,  ///< node A
        std::shared_ptr<ChNodeFEAxyzrot> nodeB,  ///< node B
        const ChFrame<>& abs_application,        ///< create the bushing here, in abs. coordinates
        const ChVector3d& stiffness_coefs,       ///< stiffness, along the x y z axes of the abs_application
        const ChVector3d& damping_coefs,         ///< damping, along the x y z axes of the abs_application
        const ChVector3d& yield_coefs            ///< plastic yield, along the x y z axes of the abs_application
    );

    /// Set plastic yield, forces beyond this limit will be capped.
    /// Expressed along the x y z axes of loc_application_B.
    void SetYield(const ChVector3d yield_coefs) { yield = yield_coefs; }
    ChVector3d GetYield() const { return yield; }

    /// Get the current accumulated plastic deformation.
    /// This could become nonzero if forces went beyond the plastic yield.
    ChVector3d GetPlasticDeformation() const { return plastic_def; }

  protected:
    ChVector3d yield;
    ChVector3d plastic_def;

    /// Compute the wrench (force & torque) between the two nodes, expressed in local frame of loc_application_B,
    /// given rel_AB, i.e. the position and speed of loc_application_A respect to loc_application_B, expressed in frame
    /// of loc_application_B. Force is assumed applied to loc_application_B, and its opposite reaction to A.
    virtual void ComputeForceTorque(const ChFrameMoving<>& rel_AB,
                                    ChVector3d& loc_force,
                                    ChVector3d& loc_torque) override;
};

//------------------------------------------------------------------------------------------------

/// Load for a visco-elastic translational/rotational bushing acting between two ChNodeFEAxyzrot nodes.
/// It uses three values for stiffness along the X Y Z axes of a coordinate system attached
/// to the second body , and three rotational stiffness values for (small) rotations about X Y Z of the
/// same coordinate system.
/// This is equivalent to having a bushing with 6x6 diagonal local stiffness matrix.
class ChApi ChLoadNodeXYZRotNodeXYZRotBushingMate : public ChLoadNodeXYZRotNodeXYZRotBushingSpherical {
  public:
    ChLoadNodeXYZRotNodeXYZRotBushingMate(
        std::shared_ptr<ChNodeFEAxyzrot> nodeA,  ///< node A
        std::shared_ptr<ChNodeFEAxyzrot> nodeB,  ///< node B
        const ChFrame<>& abs_application,        ///< create the bushing here, in abs. coordinates
        const ChVector3d& stiffness_coefs,       ///< stiffness, along x y z axes of the abs_application
        const ChVector3d& damping_coefs,         ///< damping, along x y z axes of the abs_application
        const ChVector3d& rotstiffness_coefs,    ///< rotational stiffness, about x y z axes of the abs_application
        const ChVector3d& rotdamping_coefs       ///< rotational damping, about x y z axes of the abs_application
    );

    /// Set radial stiffness, along the x y z axes of loc_application_B.
    void SetRotationalStiffness(const ChVector3d rotstiffness_coefs) { rot_stiffness = rotstiffness_coefs; }
    ChVector3d GetRotationalStiffness() const { return rot_stiffness; }

    /// Set radial damping, along the x y z axes of loc_application_B.
    void SetRotationalDamping(const ChVector3d rotdamping_coefs) { rot_damping = rotdamping_coefs; }
    ChVector3d GetRotationalDamping() const { return rot_damping; }

  protected:
    ChVector3d rot_stiffness;
    ChVector3d rot_damping;

    /// Compute the wrench (force & torque) between the two nodes, expressed in local frame of loc_application_B,
    /// given rel_AB, i.e. the position and speed of loc_application_A respect to loc_application_B, expressed in frame
    /// of loc_application_B. Force is assumed applied to loc_application_B, and its opposite reaction to A.
    virtual void ComputeForceTorque(const ChFrameMoving<>& rel_AB,
                                    ChVector3d& loc_force,
                                    ChVector3d& loc_torque) override;
};

//------------------------------------------------------------------------------------------------

/// Load for a visco-elastic translational/rotational bushing acting between two bodies.
/// It uses a full user-defined 6x6 matrix [K] to express the local stiffness of the
/// bushing, assumed expressed in the bushing coordinate system  attached
/// to the second body. A user-defined 6x6 matrix [D] can be defined for damping, as well.
/// Note that this assumes small rotations.
/// Differently from the simpler ChLoadBodyBodyBushingMate and ChLoadBodyBodyBushingSpherical
/// this can represent coupled effects, by using extra-diagonal terms in [K] and/or [D].
class ChApi ChLoadNodeXYZRotNodeXYZRotBushingGeneric : public ChLoadNodeXYZRotNodeXYZRot {
  public:
    ChLoadNodeXYZRotNodeXYZRotBushingGeneric(
        std::shared_ptr<ChNodeFEAxyzrot> nodeA,  ///< node A
        std::shared_ptr<ChNodeFEAxyzrot> nodeB,  ///< node B
        const ChFrame<>& abs_application,        ///< create the bushing here, in abs. coordinates
        ChMatrixConstRef stiffness_coefs,        ///< stiffness as a 6x6 matrix, local in the abs_application frame
        ChMatrixConstRef damping_coefs           ///< damping as a 6x6 matrix, local in the abs_application frame
    );

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLoadNodeXYZRotNodeXYZRotBushingGeneric* Clone() const override {
        return new ChLoadNodeXYZRotNodeXYZRotBushingGeneric(*this);
    }

    /// Set a generic 6x6 stiffness matrix, expressed in local coordinate system of loc_application_B.
    void SetStiffnessMatrix(ChMatrixConstRef stiffness_coefs) { stiffness = stiffness_coefs; }
    const ChMatrix66d& GetStiffnessMatrix() const { return stiffness; }

    /// Set a generic 6x6 damping matrix, expressed in local coordinate system of loc_application_B.
    void SetDampingMatrix(ChMatrixConstRef damping_coefs) { damping = damping_coefs; }
    const ChMatrix66d& GetDampingMatrix() const { return damping; }

    /// Set the initial pre-load of the bushing, applied to loc_application_A,
    /// expressed in local coordinate system of loc_application_B.
    /// By default it is zero.
    void SetNeutralForce(const ChVector3d force) { neutral_force = force; }
    ChVector3d GetNeutralForce() const { return neutral_force; }

    /// Set the initial pre-load torque of the bushing (default: 0).
    /// The pre-load is assumed applied to loc_application_A, expressed in local coordinate system of loc_application_B.
    void SetNeutralTorque(const ChVector3d torque) { neutral_torque = torque; }
    ChVector3d GetNeutralTorque() const { return neutral_torque; }

    /// Set/get the initial pre-displacement of the bushing, as the pre-displacement
    /// of A, expressed in local coordinate system of loc_application_B.
    /// Default behavior is no initial pre-displacement.
    ChFrame<>& NeutralDisplacement() { return neutral_displacement; }

  protected:
    ChMatrix66d stiffness;
    ChMatrix66d damping;

    ChVector3d neutral_force;
    ChVector3d neutral_torque;
    ChFrame<> neutral_displacement;

    virtual bool IsStiff() override { return true; }

    /// Compute the wrench (force & torque) between the two nodes, expressed in local frame of loc_application_B,
    /// given rel_AB, i.e. the position and speed of loc_application_A respect to loc_application_B, expressed in frame
    /// of loc_application_B. Force is assumed applied to loc_application_B, and its opposite reaction to A.
    virtual void ComputeForceTorque(const ChFrameMoving<>& rel_AB,
                                    ChVector3d& loc_force,
                                    ChVector3d& loc_torque) override;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

////////////////////////////////////////////////////////////////////////////
//
//  LOADS ON:   ChNodeFEAxyzrot ---- ChBody

/// Base class for loads representing a concentrated wrench (force & torque) acting between a ChNodeFEAxyzrot and a
/// ChBody The force & torque is applied between two local references attached to the two nodes, loc_application_A and
/// loc_application_B, not necessarily centered in the respective nodes. Users should inherit from this and implement a
/// custom ComputeForceTorque(), this is enough to have the load working. Note: there are already some predefined
/// examples, ie. children classes with common concrete implementations, such as ChLoadNodeXYZRotBodyBushingSpherical
/// ChLoadNodeXYZRotBodyBushingMate, etc.
class ChApi ChLoadNodeXYZRotBody : public ChLoadCustomMultiple {
  public:
    ChLoadNodeXYZRotBody(std::shared_ptr<ChNodeFEAxyzrot> node,  ///< node to apply load to as reaction
                         std::shared_ptr<ChBody> body,           ///< body B to apply load to
                         const ChFrame<>& abs_application        ///< location of load element (in abs. coordinates)
    );

    /// Compute the wrench (force & torque) between the two nodes, expressed in local frame of loc_application_B,
    /// given rel_AB, i.e. the position and speed of loc_application_A respect to loc_application_B, expressed in frame
    /// of loc_application_B. Force is assumed applied to loc_application_B, and its opposite reaction to A.
    virtual void ComputeForceTorque(const ChFrameMoving<>& rel_AB, ChVector3d& loc_force, ChVector3d& loc_torque) = 0;

    /// Compute Q, the generalized load.
    /// Called automatically at each Update().
    virtual void ComputeQ(ChState* state_x,      ///< state position to evaluate Q
                          ChStateDelta* state_w  ///< state speed to evaluate Q
                          ) override;

    // Optional: inherited classes could implement this to avoid the
    // default numerical computation of Jacobians:
    //   virtual void ComputeJacobian(...) // see ChLoad

    /// For diagnosis purposes, this can return the actual last computed value of
    /// the applied force, expressed in coordinate system of loc_application_B, assumed applied to body B
    ChVector3d GetForce() const { return locB_force; }

    /// For diagnosis purposes, this can return the actual last computed value of
    /// the applied torque, expressed in coordinate system of loc_application_B, assumed applied to body B
    ChVector3d GetTorque() const { return locB_torque; }

    /// Set the application frame of bushing on bodyA
    void SetApplicationFrameA(const ChFrame<>& pA) { loc_application_A = pA; }
    ChFrame<> GetApplicationFrameA() const { return loc_application_A; }

    /// Set the application frame of bushing on bodyB
    void SetApplicationFrameB(const ChFrame<>& pB) { loc_application_B = pB; }
    ChFrame<> GetApplicationFrameB() const { return loc_application_B; }

    /// Get absolute coordinate of frame A (last computed)
    ChFrameMoving<> GetAbsoluteFrameA() const { return frame_Aw; }

    /// Get absolute coordinate of frame B (last computed)
    ChFrameMoving<> GetAbsoluteFrameB() const { return frame_Bw; }

    std::shared_ptr<ChNodeFEAxyzrot> GetNode() const;
    std::shared_ptr<ChBody> GetBody() const;

  protected:
    ChFrame<> loc_application_A;  ///< application point on body A (local)
    ChFrame<> loc_application_B;  ///< application point on body B (local)
    ChVector3d locB_force;        ///< store computed values here
    ChVector3d locB_torque;       ///< store computed values here
    ChFrameMoving<> frame_Aw;     ///< for results
    ChFrameMoving<> frame_Bw;     ///< for results
};

//------------------------------------------------------------------------------------------------

/// Load for a visco-elastic bushing acting between a ChNodeFEAxyzrot and a ChBody.
/// It uses three values for stiffness along the X Y Z axes of a coordinate system attached
/// to the second body. This is equivalent to having a bushing with 3x3 diagonal local stiffness matrix.
class ChApi ChLoadNodeXYZRotBodyBushingSpherical : public ChLoadNodeXYZRotBody {
  public:
    ChLoadNodeXYZRotBodyBushingSpherical(
        std::shared_ptr<ChNodeFEAxyzrot> node,  ///< node A
        std::shared_ptr<ChBody> body,           ///< body B
        const ChFrame<>& abs_application,       ///< bushing location, in abs. coordinates.
        const ChVector3d& stiffness_coefs,      ///< stiffness, along x y z axes of the abs_application
        const ChVector3d& damping_coefs         ///< damping, along x y z axes of the abs_application
    );

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLoadNodeXYZRotBodyBushingSpherical* Clone() const override {
        return new ChLoadNodeXYZRotBodyBushingSpherical(*this);
    }

    /// Set stiffness, along the x y z axes of loc_application_B, es [N/m]
    void SetStiffness(const ChVector3d stiffness_coefs) { stiffness = stiffness_coefs; }
    ChVector3d GetStiffness() const { return stiffness; }

    /// Set damping, along the x y z axes of loc_application_B, es [Ns/m]
    void SetDamping(const ChVector3d damping_coefs) { damping = damping_coefs; }
    ChVector3d GetDamping() const { return damping; }

  protected:
    ChVector3d stiffness;
    ChVector3d damping;

    virtual bool IsStiff() override { return true; }

    /// Compute the wrench (force & torque) between the two nodes, expressed in local frame of loc_application_B,
    /// given rel_AB, i.e. the position and speed of loc_application_A respect to loc_application_B, expressed in frame
    /// of loc_application_B. Force is assumed applied to loc_application_B, and its opposite reaction to A.
    virtual void ComputeForceTorque(const ChFrameMoving<>& rel_AB,
                                    ChVector3d& loc_force,
                                    ChVector3d& loc_torque) override;
};

//------------------------------------------------------------------------------------------------

/// Load for a visco-elasto-plastic bushing acting between a ChNodeFEAxyzrot and a ChBody.
/// It uses three values for stiffness along the X Y Z axes of a coordinate system attached
/// to the second body. This is equivalent to having a bushing with 3x3 diagonal local stiffness matrix.
/// Also, it allows a very simple plasticity model, to cap the plastic force on x,y,z given three yelds.
class ChApi ChLoadNodeXYZRotBodyBushingPlastic : public ChLoadNodeXYZRotBodyBushingSpherical {
  public:
    ChLoadNodeXYZRotBodyBushingPlastic(
        std::shared_ptr<ChNodeFEAxyzrot> node,  ///< node A
        std::shared_ptr<ChBody> body,           ///< body B
        const ChFrame<>& abs_application,       ///< create the bushing here, in abs. coordinates.
        const ChVector3d& stiffness_coefs,      ///< stiffness, along the x y z axes of the abs_application
        const ChVector3d& damping_coefs,        ///< damping, along the x y z axes of the abs_application
        const ChVector3d& myield                ///< plastic yield, along the x y z axes of the abs_application
    );

    /// Set plastic yield, forces beyond this limit will be capped.
    /// Expressed along the x y z axes of loc_application_B.
    void SetYield(const ChVector3d yield_coefs) { yield = yield_coefs; }
    ChVector3d GetYield() const { return yield; }

    /// Get the current accumulated plastic deformation.
    /// This could become nonzero if forces went beyond the plastic yield.
    ChVector3d GetPlasticDeformation() const { return plastic_def; }

  protected:
    ChVector3d yield;
    ChVector3d plastic_def;

    /// Compute the wrench (force & torque) between the two nodes, expressed in local frame of loc_application_B,
    /// given rel_AB, i.e. the position and speed of loc_application_A respect to loc_application_B, expressed in frame
    /// of loc_application_B. Force is assumed applied to loc_application_B, and its opposite reaction to A.
    virtual void ComputeForceTorque(const ChFrameMoving<>& rel_AB,
                                    ChVector3d& loc_force,
                                    ChVector3d& loc_torque) override;
};

//------------------------------------------------------------------------------------------------

/// Load for a visco-elastic translational/rotational bushing acting between a ChNodeFEAxyzrot and a ChBody.
/// It uses three values for stiffness along the X Y Z axes of a coordinate system attached
/// to the second body , and three rotational stiffness values for (small) rotations about X Y Z of the
/// same coordinate system.
/// This is equivalent to having a bushing with 6x6 diagonal local stiffness matrix.
class ChApi ChLoadNodeXYZRotBodyBushingMate : public ChLoadNodeXYZRotBodyBushingSpherical {
  public:
    ChLoadNodeXYZRotBodyBushingMate(
        std::shared_ptr<ChNodeFEAxyzrot> mnodeA,  ///< node A
        std::shared_ptr<ChBody> mbodyB,           ///< body B
        const ChFrame<>& abs_application,         ///< create the bushing here, in abs. coordinates.
        const ChVector3d& stiffness_coefs,        ///< stiffness, along x y z axes of the abs_application
        const ChVector3d& damping_coefs,          ///< damping, along x y z axes of the abs_application
        const ChVector3d& rotstiffness_coefs,     ///< rotational stiffness, about x y z axes of the abs_application
        const ChVector3d& rotdamping_coefs        ///< rotational damping, about x y z axes of the abs_application
    );

    /// Set radial stiffness, along the x y z axes of loc_application_B, es [N/m]
    void SetRotationalStiffness(const ChVector3d rotstiffness_coefs) { rot_stiffness = rotstiffness_coefs; }
    ChVector3d GetRotationalStiffness() const { return rot_stiffness; }

    /// Set radial damping, along the x y z axes of loc_application_B, es [Ns/m]
    void SetRotationalDamping(const ChVector3d rotdamping_coefs) { rot_damping = rotdamping_coefs; }
    ChVector3d GetRotationalDamping() const { return rot_damping; }

  protected:
    ChVector3d rot_stiffness;
    ChVector3d rot_damping;

    /// Compute the wrench (force & torque) between the two nodes, expressed in local frame of loc_application_B,
    /// given rel_AB, i.e. the position and speed of loc_application_A respect to loc_application_B, expressed in frame
    /// of loc_application_B. Force is assumed applied to loc_application_B, and its opposite reaction to A.
    virtual void ComputeForceTorque(const ChFrameMoving<>& rel_AB,
                                    ChVector3d& loc_force,
                                    ChVector3d& loc_torque) override;
};

//------------------------------------------------------------------------------------------------

/// Load for a visco-elastic translational/rotational bushing acting between a ChNodeFEAxyzrot and a ChBody.
/// It uses a full user-defined 6x6 matrix [K] to express the local stiffness of the
/// bushing, assumed expressed in the bushing coordinate system  attached
/// to the second body. A user-defined 6x6 matrix [D] can be defined for damping, as well.
/// Note that this assumes small rotations.
/// Differently from the simpler ChLoadBodyBodyBushingMate and ChLoadBodyBodyBushingSpherical
/// this can represent coupled effects, by using extra-diagonal terms in [K] and/or [D].
class ChApi ChLoadNodeXYZRotBodyBushingGeneric : public ChLoadNodeXYZRotBody {
  public:
    ChLoadNodeXYZRotBodyBushingGeneric(
        std::shared_ptr<ChNodeFEAxyzrot> mnodeA,  ///< node A
        std::shared_ptr<ChBody> mbodyB,           ///< body B
        const ChFrame<>& abs_application,         ///< create the bushing here, in abs. coordinates.
        ChMatrixConstRef stiffness_coefs,         ///< stiffness as a 6x6 matrix, local in the abs_application frame
        ChMatrixConstRef damping_coefs            ///< damping as a 6x6 matrix, local in the abs_application frame
    );

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLoadNodeXYZRotBodyBushingGeneric* Clone() const override {
        return new ChLoadNodeXYZRotBodyBushingGeneric(*this);
    }

    /// Set a generic 6x6 stiffness matrix, expressed in local coordinate system of loc_application_B.
    void SetStiffnessMatrix(ChMatrixConstRef stiffness_coefs) { stiffness = stiffness_coefs; }
    const ChMatrix66d& GetStiffnessMatrix() const { return stiffness; }

    /// Set a generic 6x6 damping matrix, expressed in local coordinate system of loc_application_B.
    void SetDampingMatrix(ChMatrixConstRef damping_coefs) { damping = damping_coefs; }
    const ChMatrix66d& GetDampingMatrix() const { return damping; }

    /// Set the initial pre-load of the bushing, applied to loc_application_A,
    /// expressed in local coordinate system of loc_application_B.
    /// By default it is zero.
    void SetNeutralForce(const ChVector3d force) { neutral_force = force; }
    ChVector3d GetNeutralForce() const { return neutral_force; }

    /// Set the initial pre-load torque of the bushing, applied to loc_application_A,
    /// expressed in local coordinate system of loc_application_B.
    /// By default it is zero.
    void SetNeutralTorque(const ChVector3d torque) { neutral_torque = torque; }
    ChVector3d GetNeutralTorque() const { return neutral_torque; }

    /// Set/get the initial pre-displacement of the bushing, as the pre-displacement
    /// of A, expressed in local coordinate system of loc_application_B.
    /// Default behavior is no initial pre-displacement.
    ChFrame<>& NeutralDisplacement() { return neutral_displacement; }

  protected:
    ChMatrix66d stiffness;
    ChMatrix66d damping;

    ChVector3d neutral_force;
    ChVector3d neutral_torque;
    ChFrame<> neutral_displacement;

    virtual bool IsStiff() override { return true; }

    /// Compute the wrench (force & torque) between the two nodes, expressed in local frame of loc_application_B,
    /// given rel_AB, i.e. the position and speed of loc_application_A respect to loc_application_B, expressed in frame
    /// of loc_application_B. Force is assumed applied to loc_application_B, and its opposite reaction to A.
    virtual void ComputeForceTorque(const ChFrameMoving<>& rel_AB,
                                    ChVector3d& loc_force,
                                    ChVector3d& loc_torque) override;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace fea
}  // namespace chrono

#endif
