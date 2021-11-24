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

#ifndef CHLOADSXYZROTNODE_H
#define CHLOADSXYZROTNODE_H

#include "chrono/physics/ChLoad.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"
#include "chrono/physics/ChBody.h"
#include "chrono/motion_functions/ChFunction.h"

namespace chrono {
namespace fea {

////////////////////////////////////////////////////////////////////////////
//
//  LOADS ON:   ChNodeFEAxyzrot

/// Base class for loads representing a concentrated wrench (force + torque) acting on a ChNodeFEAxyzrot.
/// Users should inherit from this and implement custom ComputeForceTorque(), this is enough to have the load working.
/// Note: there are already some predefined examples, i.e. children classes with common concrete implementations, such
/// as ChLoadXYZROTnodeForceAbsolute; take inspiration from those example to inherit your own load if they are not
/// enough.
class ChApi ChLoadXYZROTnode : public ChLoadCustom {
  public:
    ChLoadXYZROTnode(std::shared_ptr<ChNodeFEAxyzrot> body  ///< node to apply load to
    );

    /// Compute the force and torque on the node, in absolute coordsystem,
    /// given absolute position and speed of node passed via node_frame_abs_pos_vel.
    /// Inherited classes MUST IMPLEMENT THIS.
    virtual void ComputeForceTorque(const ChFrameMoving<>& node_frame_abs_pos_vel,
                                    ChVector<>& abs_force,
                                    ChVector<>& abs_torque) = 0;

    // Optional: inherited classes could implement this to avoid the
    // default numerical computation of jacobians:
    //   virtual void ComputeJacobian(...) // see ChLoad

    /// Compute Q, the generalized load.
    /// Called automatically at each Update().
    virtual void ComputeQ(ChState* state_x,      ///< state position to evaluate Q
                          ChStateDelta* state_w  ///< state speed to evaluate Q
                          ) override;

    /// For diagnosis purposes, this can return the actual last computed value of
    /// the applied force, expressed in absolute coordinate system, assumed applied to node
    ChVector<> GetForce() const { return computed_abs_force; }

    /// For diagnosis purposes, this can return the actual last computed value of
    /// the applied torque, expressed in absolute coordinate system, assumed applied to node
    ChVector<> GetTorque() const { return computed_abs_torque; }

  protected:
    /// Inherited classes could override this and return true, if the load benefits from a jacobian
    /// when using implicit integrators.
    virtual bool IsStiff() override { return false; }

    virtual void Update(double time) override;

    ChVector<> computed_abs_force;
    ChVector<> computed_abs_torque;
};

/// Load representing a concentrated force acting on a ChNodeXYZ, as a constant force, or it provides a function to
/// modulate it with time. As it is constant in space, IsStiff() is false, as default.
class ChApi ChLoadXYZROTnodeForceAbsolute : public ChLoadXYZROTnode {
  public:
    ChLoadXYZROTnodeForceAbsolute(std::shared_ptr<ChNodeFEAxyzrot> node,  ///< node to apply load to
                                  const ChVector<>& force  ///< force to apply, assumed in absolute coordsys,
    );

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLoadXYZROTnodeForceAbsolute* Clone() const override { return new ChLoadXYZROTnodeForceAbsolute(*this); }

    /// Compute the force on the node, in absolute coordsystem,
    /// given absolute position and speed of node passed via node_frame_abs_pos_vel.
    virtual void ComputeForceTorque(const ChFrameMoving<>& node_frame_abs_pos_vel,
                                    ChVector<>& abs_force,
                                    ChVector<>& abs_torque) override;

    /// Set the applied force vector: it is assumed constant, unless a
    /// non-constant scaling time function is provided.
    /// It is expressed in absolute coordinates.
    void SetForceBase(const ChVector<>& force);

    /// Return the current force vector (scaled by the current modulation value).
    ChVector<> GetForce() const;

    /// Set modulation function.
    /// This is a function of time which (optionally) modulates the specified applied force.
    /// By default the modulation is a constant function, always returning a value of 1.
    void SetModulationFunction(std::shared_ptr<ChFunction> modulation) { m_modulation = modulation; }

  protected:
    ChVector<> m_force_base;  ///< base force value

    std::shared_ptr<ChFunction> m_modulation;  ///< modulation function of time
    double m_scale;                            ///< scaling factor (current modulation value)

    virtual bool IsStiff() override { return false; }

    virtual void Update(double time) override;
};

////////////////////////////////////////////////////////////////////////////
//
//  LOADS ON:   ChNodeFEAxyzrot ---- ChNodeFEAxyzrot

/// Base class for loads representing a concentrated wrench (force & torque) acting between two ChNodeFEAxyzrot.
/// The force & torque is applied between two local references attached to the two nodes, loc_application_A and
/// loc_application_B, not necessarily centered in the respective nodes. Users should inherit from this and implement a
/// custom ComputeForceTorque(), this is enough to have the load working. Note: there are already some predefined
/// examples, ie. children classes with common concrete implementations, such as
/// ChLoadXYZROTnodeXYZROTnodeBushingSpherical ChLoadXYZROTnodeXYZROTnodeBushingMate, etc.; take inspiration from those
/// example to inherit your own load if they are not enough.
class ChApi ChLoadXYZROTnodeXYZROTnode : public ChLoadCustomMultiple {
  public:
    ChLoadXYZROTnodeXYZROTnode(std::shared_ptr<ChNodeFEAxyzrot> mnodeA,  ///< node A to apply load to
                               std::shared_ptr<ChNodeFEAxyzrot> mnodeB,  ///< node B to apply load to, as reaction
                               const ChFrame<>& abs_application  ///< location of load element (in abs. coordinates)
    );

    /// Compute the wrench (force & torque) between the two nodes, expressed in local frame of loc_application_B,
    /// given rel_AB, i.e. the position and speed of loc_application_A respect to loc_application_B, expressed in frame
    /// of loc_application_B. Force is assumed applied to loc_application_B, and its opposite reaction to A. Inherited
    /// classes MUST IMPLEMENT THIS.
    virtual void ComputeForceTorque(const ChFrameMoving<>& rel_AB, ChVector<>& loc_force, ChVector<>& loc_torque) = 0;

    /// Compute Q, the generalized load.
    /// Called automatically at each Update().
    virtual void ComputeQ(ChState* state_x,      ///< state position to evaluate Q
                          ChStateDelta* state_w  ///< state speed to evaluate Q
                          ) override;

    // Optional: inherited classes could implement this to avoid the
    // default numerical computation of jacobians:
    //   virtual void ComputeJacobian(...) // see ChLoad

    /// For diagnosis purposes, this can return the actual last computed value of
    /// the applied force, expressed in coordinate system of loc_application_B, assumed applied to body B
    ChVector<> GetForce() const { return locB_force; }

    /// For diagnosis purposes, this can return the actual last computed value of
    /// the applied torque, expressed in coordinate system of loc_application_B, assumed applied to body B
    ChVector<> GetTorque() const { return locB_torque; }

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

    std::shared_ptr<ChNodeFEAxyzrot> GetNodeA() const;
    std::shared_ptr<ChNodeFEAxyzrot> GetNodeB() const;

  protected:
    ChFrame<> loc_application_A;  ///< application point on body A (local)
    ChFrame<> loc_application_B;  ///< application point on body B (local)
    ChVector<> locB_force;        ///< store computed values here
    ChVector<> locB_torque;       ///< store computed values here
    ChFrameMoving<> frame_Aw;     ///< for results
    ChFrameMoving<> frame_Bw;     ///< for results
};

//------------------------------------------------------------------------------------------------

/// Load for a visco-elastic bushing acting between two bodies.
/// The bushing is between two local references attached to the two nodes, loc_application_A and loc_application_B,
/// not necessarily centered in the respective nodes.
/// It uses three values for stiffness along the X Y Z axes of a coordinate system attached
/// to the second body. This is equivalent to having a bushing with 3x3 diagonal local stiffness matrix.
class ChApi ChLoadXYZROTnodeXYZROTnodeBushingSpherical : public ChLoadXYZROTnodeXYZROTnode {
  public:
    ChLoadXYZROTnodeXYZROTnodeBushingSpherical(
        std::shared_ptr<ChNodeFEAxyzrot> mnodeA,  ///< node A
        std::shared_ptr<ChNodeFEAxyzrot> mnodeB,  ///< node B
        const ChFrame<>&
            abs_application,  ///< create bushing here, in abs. coordinates. Will define loc_application_A and B
        const ChVector<>& mstiffness,  ///< stiffness, along x y z axes of the abs_application
        const ChVector<>& mdamping     ///< damping, along x y z axes of the abs_application
    );

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLoadXYZROTnodeXYZROTnodeBushingSpherical* Clone() const override {
        return new ChLoadXYZROTnodeXYZROTnodeBushingSpherical(*this);
    }

    /// Set stiffness, along the x y z axes of frame of loc_application_B, es [N/m]
    void SetStiffness(const ChVector<> mstiffness) { stiffness = mstiffness; }
    ChVector<> GetStiffness() const { return stiffness; }

    /// Set damping, along the x y z axes of frame of loc_application_B, es [Ns/m]
    void SetDamping(const ChVector<> mdamping) { damping = mdamping; }
    ChVector<> GetDamping() const { return damping; }

  protected:
    ChVector<> stiffness;
    ChVector<> damping;

    virtual bool IsStiff() override { return true; }

    /// Compute the wrench (force & torque) between the two nodes, expressed in local frame of loc_application_B,
    /// given rel_AB, i.e. the position and speed of loc_application_A respect to loc_application_B, expressed in frame
    /// of loc_application_B. Force is assumed applied to loc_application_B, and its opposite reaction to A.
    virtual void ComputeForceTorque(const ChFrameMoving<>& rel_AB,
                                    ChVector<>& loc_force,
                                    ChVector<>& loc_torque) override;
};

//------------------------------------------------------------------------------------------------

/// Load for a visco-elasto-plastic bushing acting between two ChNodeFEAxyzrot.
/// It uses three values for stiffness along the X Y Z axes of a coordinate system attached
/// to the second node. This is equivalent to having a bushing with 3x3 diagonal local stiffness matrix.
/// Also, it allows a very simple plasticity model, to cap the plastic force on x,y,z given three yelds.
class ChApi ChLoadXYZROTnodeXYZROTnodeBushingPlastic : public ChLoadXYZROTnodeXYZROTnodeBushingSpherical {
  public:
    ChLoadXYZROTnodeXYZROTnodeBushingPlastic(
        std::shared_ptr<ChNodeFEAxyzrot> mnodeA,  ///< node A
        std::shared_ptr<ChNodeFEAxyzrot> mnodeB,  ///< node B
        const ChFrame<>&
            abs_application,  ///< create the bushing here, in abs. coordinates. Will define loc_application_A and B
        const ChVector<>& mstiffness,  ///< stiffness, along the x y z axes of the abs_application
        const ChVector<>& mdamping,    ///< damping, along the x y z axes of the abs_application
        const ChVector<>& myield       ///< plastic yield, along the x y z axes of the abs_application
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

    /// Compute the wrench (force & torque) between the two nodes, expressed in local frame of loc_application_B,
    /// given rel_AB, i.e. the position and speed of loc_application_A respect to loc_application_B, expressed in frame
    /// of loc_application_B. Force is assumed applied to loc_application_B, and its opposite reaction to A.
    virtual void ComputeForceTorque(const ChFrameMoving<>& rel_AB,
                                    ChVector<>& loc_force,
                                    ChVector<>& loc_torque) override;
};

//------------------------------------------------------------------------------------------------

/// Load for a visco-elastic translational/rotational bushing acting between two ChNodeFEAxyzrot nodes.
/// It uses three values for stiffness along the X Y Z axes of a coordinate system attached
/// to the second body , and three rotational stiffness values for (small) rotations about X Y Z of the
/// same coordinate system.
/// This is equivalent to having a bushing with 6x6 diagonal local stiffness matrix.
class ChApi ChLoadXYZROTnodeXYZROTnodeBushingMate : public ChLoadXYZROTnodeXYZROTnodeBushingSpherical {
  public:
    ChLoadXYZROTnodeXYZROTnodeBushingMate(
        std::shared_ptr<ChNodeFEAxyzrot> mnodeA,  ///< node A
        std::shared_ptr<ChNodeFEAxyzrot> mnodeB,  ///< node B
        const ChFrame<>&
            abs_application,  ///< create the bushing here, in abs. coordinates. Will define loc_application_A and B
        const ChVector<>& mstiffness,     ///< stiffness, along x y z axes of the abs_application
        const ChVector<>& mdamping,       ///< damping, along x y z axes of the abs_application
        const ChVector<>& mrotstiffness,  ///< rotational stiffness, about x y z axes of the abs_application
        const ChVector<>& mrotdamping     ///< rotational damping, about x y z axes of the abs_application
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

    /// Compute the wrench (force & torque) between the two nodes, expressed in local frame of loc_application_B,
    /// given rel_AB, i.e. the position and speed of loc_application_A respect to loc_application_B, expressed in frame
    /// of loc_application_B. Force is assumed applied to loc_application_B, and its opposite reaction to A.
    virtual void ComputeForceTorque(const ChFrameMoving<>& rel_AB,
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
class ChApi ChLoadXYZROTnodeXYZROTnodeBushingGeneric : public ChLoadXYZROTnodeXYZROTnode {
  public:
    ChLoadXYZROTnodeXYZROTnodeBushingGeneric(
        std::shared_ptr<ChNodeFEAxyzrot> mnodeA,  ///< node A
        std::shared_ptr<ChNodeFEAxyzrot> mnodeB,  ///< node B
        const ChFrame<>&
            abs_application,  ///< create the bushing here, in abs. coordinates. Will define loc_application_A and B
        ChMatrixConstRef mstiffness,  ///< stiffness as a 6x6 matrix, local in the abs_application frame
        ChMatrixConstRef mdamping     ///< damping as a 6x6 matrix, local in the abs_application frame
    );

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLoadXYZROTnodeXYZROTnodeBushingGeneric* Clone() const override {
        return new ChLoadXYZROTnodeXYZROTnodeBushingGeneric(*this);
    }

    /// Set a generic 6x6 stiffness matrix, expressed in local
    /// coordinate system of loc_application_B.
    void SetStiffnessMatrix(ChMatrixConstRef mstiffness) { stiffness = mstiffness; }
    const ChMatrixNM<double, 6, 6>& GetStiffnessMatrix() const { return stiffness; }

    /// Set a generic 6x6 damping matrix, expressed in local
    /// coordinate system of loc_application_B.
    void SetDampingMatrix(ChMatrixConstRef mdamping) { damping = mdamping; }
    const ChMatrixNM<double, 6, 6>& GetDampingMatrix() const { return damping; }

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

    /// Compute the wrench (force & torque) between the two nodes, expressed in local frame of loc_application_B,
    /// given rel_AB, i.e. the position and speed of loc_application_A respect to loc_application_B, expressed in frame
    /// of loc_application_B. Force is assumed applied to loc_application_B, and its opposite reaction to A.
    virtual void ComputeForceTorque(const ChFrameMoving<>& rel_AB,
                                    ChVector<>& loc_force,
                                    ChVector<>& loc_torque) override;

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
/// examples, ie. children classes with common concrete implementations, such as ChLoadXYZROTnodeBodyBushingSpherical
/// ChLoadXYZROTnodeBodyBushingMate, etc.; take inspiration from those example to inherit your
/// own load if they are not enough.
class ChApi ChLoadXYZROTnodeBody : public ChLoadCustomMultiple {
  public:
    ChLoadXYZROTnodeBody(std::shared_ptr<ChNodeFEAxyzrot> mnodeA,  ///< node A to apply load to as reaction,
                         std::shared_ptr<ChBody> mbodyB,           ///< body B to apply load to,
                         const ChFrame<>& abs_application          ///< location of load element (in abs. coordinates)
    );

    /// Compute the wrench (force & torque) between the two nodes, expressed in local frame of loc_application_B,
    /// given rel_AB, i.e. the position and speed of loc_application_A respect to loc_application_B, expressed in frame
    /// of loc_application_B. Force is assumed applied to loc_application_B, and its opposite reaction to A. Inherited
    /// classes MUST IMPLEMENT THIS.
    virtual void ComputeForceTorque(const ChFrameMoving<>& rel_AB, ChVector<>& loc_force, ChVector<>& loc_torque) = 0;

    /// Compute Q, the generalized load.
    /// Called automatically at each Update().
    virtual void ComputeQ(ChState* state_x,      ///< state position to evaluate Q
                          ChStateDelta* state_w  ///< state speed to evaluate Q
                          ) override;

    // Optional: inherited classes could implement this to avoid the
    // default numerical computation of jacobians:
    //   virtual void ComputeJacobian(...) // see ChLoad

    /// For diagnosis purposes, this can return the actual last computed value of
    /// the applied force, expressed in coordinate system of loc_application_B, assumed applied to body B
    ChVector<> GetForce() const { return locB_force; }

    /// For diagnosis purposes, this can return the actual last computed value of
    /// the applied torque, expressed in coordinate system of loc_application_B, assumed applied to body B
    ChVector<> GetTorque() const { return locB_torque; }

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

    std::shared_ptr<ChNodeFEAxyzrot> GetNodeA() const;
    std::shared_ptr<ChBody> GetBodyB() const;

  protected:
    ChFrame<> loc_application_A;  ///< application point on body A (local)
    ChFrame<> loc_application_B;  ///< application point on body B (local)
    ChVector<> locB_force;        ///< store computed values here
    ChVector<> locB_torque;       ///< store computed values here
    ChFrameMoving<> frame_Aw;     ///< for results
    ChFrameMoving<> frame_Bw;     ///< for results
};

//------------------------------------------------------------------------------------------------

/// Load for a visco-elastic bushing acting between a ChNodeFEAxyzrot and a ChBody.
/// It uses three values for stiffness along the X Y Z axes of a coordinate system attached
/// to the second body. This is equivalent to having a bushing with 3x3 diagonal local stiffness matrix.
class ChApi ChLoadXYZROTnodeBodyBushingSpherical : public ChLoadXYZROTnodeBody {
  public:
    ChLoadXYZROTnodeBodyBushingSpherical(
        std::shared_ptr<ChNodeFEAxyzrot> mnodeA,  ///< node A
        std::shared_ptr<ChBody> mbodyB,           ///< body B
        const ChFrame<>& abs_application,         ///< bushing location, in abs. coordinates.
        const ChVector<>& mstiffness,             ///< stiffness, along x y z axes of the abs_application
        const ChVector<>& mdamping                ///< damping, along x y z axes of the abs_application
    );

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLoadXYZROTnodeBodyBushingSpherical* Clone() const override {
        return new ChLoadXYZROTnodeBodyBushingSpherical(*this);
    }

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

    /// Compute the wrench (force & torque) between the two nodes, expressed in local frame of loc_application_B,
    /// given rel_AB, i.e. the position and speed of loc_application_A respect to loc_application_B, expressed in frame
    /// of loc_application_B. Force is assumed applied to loc_application_B, and its opposite reaction to A.
    virtual void ComputeForceTorque(const ChFrameMoving<>& rel_AB,
                                    ChVector<>& loc_force,
                                    ChVector<>& loc_torque) override;
};

//------------------------------------------------------------------------------------------------

/// Load for a visco-elasto-plastic bushing acting between a ChNodeFEAxyzrot and a ChBody.
/// It uses three values for stiffness along the X Y Z axes of a coordinate system attached
/// to the second body. This is equivalent to having a bushing with 3x3 diagonal local stiffness matrix.
/// Also, it allows a very simple plasticity model, to cap the plastic force on x,y,z given three yelds.
class ChApi ChLoadXYZROTnodeBodyBushingPlastic : public ChLoadXYZROTnodeBodyBushingSpherical {
  public:
    ChLoadXYZROTnodeBodyBushingPlastic(
        std::shared_ptr<ChNodeFEAxyzrot> mnodeA,  ///< node A
        std::shared_ptr<ChBody> mbodyB,           ///< body B
        const ChFrame<>& abs_application,         ///< create the bushing here, in abs. coordinates.
        const ChVector<>& mstiffness,             ///< stiffness, along the x y z axes of the abs_application
        const ChVector<>& mdamping,               ///< damping, along the x y z axes of the abs_application
        const ChVector<>& myield                  ///< plastic yield, along the x y z axes of the abs_application
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

    /// Compute the wrench (force & torque) between the two nodes, expressed in local frame of loc_application_B,
    /// given rel_AB, i.e. the position and speed of loc_application_A respect to loc_application_B, expressed in frame
    /// of loc_application_B. Force is assumed applied to loc_application_B, and its opposite reaction to A.
    virtual void ComputeForceTorque(const ChFrameMoving<>& rel_AB,
                                    ChVector<>& loc_force,
                                    ChVector<>& loc_torque) override;
};

//------------------------------------------------------------------------------------------------

/// Load for a visco-elastic translational/rotational bushing acting between a ChNodeFEAxyzrot and a ChBody.
/// It uses three values for stiffness along the X Y Z axes of a coordinate system attached
/// to the second body , and three rotational stiffness values for (small) rotations about X Y Z of the
/// same coordinate system.
/// This is equivalent to having a bushing with 6x6 diagonal local stiffness matrix.
class ChApi ChLoadXYZROTnodeBodyBushingMate : public ChLoadXYZROTnodeBodyBushingSpherical {
  public:
    ChLoadXYZROTnodeBodyBushingMate(
        std::shared_ptr<ChNodeFEAxyzrot> mnodeA,  ///< node A
        std::shared_ptr<ChBody> mbodyB,           ///< body B
        const ChFrame<>& abs_application,         ///< create the bushing here, in abs. coordinates.
        const ChVector<>& mstiffness,             ///< stiffness, along x y z axes of the abs_application
        const ChVector<>& mdamping,               ///< damping, along x y z axes of the abs_application
        const ChVector<>& mrotstiffness,          ///< rotational stiffness, about x y z axes of the abs_application
        const ChVector<>& mrotdamping             ///< rotational damping, about x y z axes of the abs_application
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

    /// Compute the wrench (force & torque) between the two nodes, expressed in local frame of loc_application_B,
    /// given rel_AB, i.e. the position and speed of loc_application_A respect to loc_application_B, expressed in frame
    /// of loc_application_B. Force is assumed applied to loc_application_B, and its opposite reaction to A.
    virtual void ComputeForceTorque(const ChFrameMoving<>& rel_AB,
                                    ChVector<>& loc_force,
                                    ChVector<>& loc_torque) override;
};

//------------------------------------------------------------------------------------------------

/// Load for a visco-elastic translational/rotational bushing acting between a ChNodeFEAxyzrot and a ChBody.
/// It uses a full user-defined 6x6 matrix [K] to express the local stiffness of the
/// bushing, assumed expressed in the bushing coordinate system  attached
/// to the second body. A user-defined 6x6 matrix [D] can be defined for damping, as well.
/// Note that this assumes small rotations.
/// Differently from the simpler ChLoadBodyBodyBushingMate and ChLoadBodyBodyBushingSpherical
/// this can represent coupled effects, by using extra-diagonal terms in [K] and/or [D].
class ChApi ChLoadXYZROTnodeBodyBushingGeneric : public ChLoadXYZROTnodeBody {
  public:
    ChLoadXYZROTnodeBodyBushingGeneric(
        std::shared_ptr<ChNodeFEAxyzrot> mnodeA,  ///< node A
        std::shared_ptr<ChBody> mbodyB,           ///< body B
        const ChFrame<>& abs_application,         ///< create the bushing here, in abs. coordinates.
        ChMatrixConstRef mstiffness,              ///< stiffness as a 6x6 matrix, local in the abs_application frame
        ChMatrixConstRef mdamping                 ///< damping as a 6x6 matrix, local in the abs_application frame
    );

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLoadXYZROTnodeBodyBushingGeneric* Clone() const override {
        return new ChLoadXYZROTnodeBodyBushingGeneric(*this);
    }

    /// Set a generic 6x6 stiffness matrix, expressed in local
    /// coordinate system of loc_application_B.
    void SetStiffnessMatrix(ChMatrixConstRef mstiffness) { stiffness = mstiffness; }
    const ChMatrixNM<double, 6, 6>& GetStiffnessMatrix() const { return stiffness; }

    /// Set a generic 6x6 damping matrix, expressed in local
    /// coordinate system of loc_application_B.
    void SetDampingMatrix(ChMatrixConstRef mdamping) { damping = mdamping; }
    const ChMatrixNM<double, 6, 6>& GetDampingMatrix() const { return damping; }

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

    /// Compute the wrench (force & torque) between the two nodes, expressed in local frame of loc_application_B,
    /// given rel_AB, i.e. the position and speed of loc_application_A respect to loc_application_B, expressed in frame
    /// of loc_application_B. Force is assumed applied to loc_application_B, and its opposite reaction to A.
    virtual void ComputeForceTorque(const ChFrameMoving<>& rel_AB,
                                    ChVector<>& loc_force,
                                    ChVector<>& loc_torque) override;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace fea
}  // namespace chrono

#endif
