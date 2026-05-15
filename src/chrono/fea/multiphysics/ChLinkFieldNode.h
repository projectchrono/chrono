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

#ifndef CHLINKFIELD_H
#define CHLINKFIELD_H

#include "chrono/physics/ChLinkBase.h"
#include "chrono/fea/multiphysics/ChField.h"
#include "chrono/solver/ChConstraintTwoGeneric.h"
#include "chrono/solver/ChConstraintNgeneric.h"


namespace chrono {

class ChIndexedNodes;  // forward ref

namespace fea {

class ChFieldScalar;  // forward ref

/// @addtogroup fea_constraints
/// @{



/// Constraint between on a scalar field state, (ex. temperature, or voltage, etc.)
/// to enformce that the state at node must be equal to some "offset" value.
/// The offset can be either a constant value, or a function of time.

class ChApi ChLinkField : public ChLinkBase {
  public:
    ChLinkField();
    ChLinkField(const ChLinkField& other);
    ~ChLinkField() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkField* Clone() const override { return new ChLinkField(*this); }

    /// Get the number of scalar variables affected by constraints in this link.
    virtual unsigned int GetNumAffectedCoords() override { return 1; }

    /// Number of scalar constraints.
    virtual unsigned int GetNumConstraintsBilateral() override { return 1; }

    // Get constraint violations.
    virtual ChVectorDynamic<> GetConstraintViolation() const override;

    /// Initialize the constraint, given a node, and a scalar
    /// field to whom the node belong
    virtual int Initialize(std::shared_ptr<ChNodeFEAbase> node,  ///< node, associated to some scalar field state
                           std::shared_ptr<ChFieldBase> field  ///< scalar field with the field state to constrain (must contain node).
    );

    /// Set the offset for the field state at node. The constraint is assumed satisfied when
    ///  state1 = offset;
    /// Default is 0 offset.
    virtual void SetOffset(double offset);

    /// Set the offset for the field state at node, where the offset is a function of time.
    /// The constraint is assumed satisfied when
    ///  state1 = offset(t);
    virtual void SetOffset(std::shared_ptr<ChFunction> offset_f);

    /// Get the connected node.
    std::shared_ptr<fea::ChNodeFEAbase> GetNode() const { return m_node; }

    /// Get the reaction force considered as applied to the 1st node.
    ChVector3d GetReactionOnNode() const { return -react; }

    /// Update all auxiliary data of the gear transmission at given time.
    virtual void Update(double time, UpdateFlags update_flags) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

    // Override/implement interfaces for global state vectors, see ChPhysicsItem for comments.

    virtual void IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) override;
    virtual void IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) override;
    virtual void IntLoadResidual_CqL(const unsigned int off_L,
                                     ChVectorDynamic<>& R,
                                     const ChVectorDynamic<>& L,
                                     const double c) override;
    virtual void IntLoadConstraint_C(const unsigned int off,
                                     ChVectorDynamic<>& Qc,
                                     const double c,
                                     const double c_vel,  ///< the scaling factor if the constraint is at speed level
                                     bool do_clamp,
                                     double recovery_clamp) override;
    virtual void IntLoadConstraint_Ct(const unsigned int off, 
                                    ChVectorDynamic<>& Qc, 
                                    const double c, 
                                    const double c_vel) override;
    virtual void IntToDescriptor(const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const ChVectorDynamic<>& R,
                                 const unsigned int off_L,
                                 const ChVectorDynamic<>& L,
                                 const ChVectorDynamic<>& Qc) override;
    virtual void IntFromDescriptor(const unsigned int off_v,
                                   ChStateDelta& v,
                                   const unsigned int off_L,
                                   ChVectorDynamic<>& L) override;

    // Override/implement system functions of ChPhysicsItem
    // (to assemble/manage data for system solver)

    virtual void InjectConstraints(ChSystemDescriptor& descriptor) override;
    virtual void ConstraintsBiReset() override;
    virtual void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false) override;
    virtual void ConstraintsBiLoad_Ct(double factor = 1) override;
    virtual void LoadConstraintJacobians() override;
    virtual void ConstraintsFetch_react(double factor = 1) override;

  private:
    double react;

    std::shared_ptr<ChFunction> offset_function;

    // used as an interface to the solver.
    ChConstraintNgeneric m_constraint1;

    std::shared_ptr<fea::ChNodeFEAbase> m_node;
    std::shared_ptr<fea::ChFieldBase> m_field;

    // Provide dummy implementation for the following 4 members tht are meaningless anyway for this kind of link:

    /// Get the link frame 1, on the 1st node, expressed in the absolute frame.
    virtual ChFramed GetFrame1Abs() const override { return ChFramed(); }
    /// Get the link frame 2, on the 2nd node, expressed in the absolute frame.
    virtual ChFramed GetFrame2Abs() const override { return ChFramed(); }
    /// Get reaction force and torque on 1st node, expressed on link frame 1.
    virtual ChWrenchd GetReaction1() const override { return {ChVector3d(react, 0, 0), VNULL}; }
    /// Get reaction force and torque on 2nd node, expressed on link frame 2.
    virtual ChWrenchd GetReaction2() const override { return {ChVector3d(-react, 0, 0), VNULL}; }
};



/////////////////////////////////////////////////////////////////////////////////////////////////


/// Constraint between two field states, (ex. temperature-temperature, or voltage-voltage, etc.)
/// to enformce that the two states are equal.
/// This supports an offset, so that you can impose a temperature difference between two nodes, or
/// a voltage difference between two nodes.
/// The offset can be either a constant value, or a function of time.

class ChApi ChLinkFieldField : public ChLinkBase {
public:
    ChLinkFieldField();
    ChLinkFieldField(const ChLinkFieldField& other);
    ~ChLinkFieldField() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkFieldField* Clone() const override { return new ChLinkFieldField(*this); }

    /// Get the number of scalar variables affected by constraints in this link.
    virtual unsigned int GetNumAffectedCoords() override { return 1 + 1; }

    /// Number of scalar constraints.
    virtual unsigned int GetNumConstraintsBilateral() override { return 1; }

    // Get constraint violations.
    virtual ChVectorDynamic<> GetConstraintViolation() const override;

    /// Initialize the constraint, given the two nodes, and scalar 
    /// fields to whom the two nodes belong
    virtual int Initialize(std::shared_ptr<ChNodeFEAbase> node1,  ///< node 1, associated to some scalar field state
                           std::shared_ptr<ChFieldBase> field1, ///< scalar field with the 1st field state to constrain (must contain node 1).
                           std::shared_ptr<ChNodeFEAbase> node2,  ///< node 2, associated to some scalar field state
                           std::shared_ptr<ChFieldBase> field2 ///< scalar field with the 2nd field state to constrain (must contain node 2).
    );

    /// Set the offset between the two field states. The constraint is assumed satisfied when  
    ///  state1 - state2 = offset;
    /// Default is 0 offset, for enforcing  state2 = state 1.
    virtual void SetOffset(double offset);

    /// Set the offset between the two field states, where the offset is a function of time. 
    /// The constraint is assumed satisfied when  
    ///  state1 - state2 = offset(t);
    virtual void SetOffset(std::shared_ptr<ChFunction> offset_f);


    /// Get the 1st connected xyz node.
    std::shared_ptr<fea::ChNodeFEAbase> GetNode1() const { return m_node1; }

    /// Get the 2nd connected xyz node.
    std::shared_ptr<fea::ChNodeFEAbase> GetNode2() const { return m_node2; }

    /// Get the reaction force considered as applied to the 1st node.
    ChVector3d GetReactionOnNode1() const { return +react; }

    /// Get the reaction force considered as applied to the 2nd node.
    ChVector3d GetReactionOnNode2() const { return -react; }

    /// Update all auxiliary data of the gear transmission at given time.
    virtual void Update(double time, UpdateFlags update_flags) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

    // Override/implement interfaces for global state vectors, see ChPhysicsItem for comments.

    virtual void IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) override;
    virtual void IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) override;
    virtual void IntLoadResidual_CqL(const unsigned int off_L,
                                     ChVectorDynamic<>& R,
                                     const ChVectorDynamic<>& L,
                                     const double c) override;
    virtual void IntLoadConstraint_C(const unsigned int off,
                                     ChVectorDynamic<>& Qc,
                                     const double c,
                                     const double c_vel,  ///< the scaling factor if the constraint is at speed level
                                     bool do_clamp,
                                     double recovery_clamp) override;
    virtual void IntLoadConstraint_Ct(const unsigned int off, 
                                    ChVectorDynamic<>& Qc,
                                    const double c, 
                                    const double c_vel) override;
    virtual void IntToDescriptor(const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const ChVectorDynamic<>& R,
                                 const unsigned int off_L,
                                 const ChVectorDynamic<>& L,
                                 const ChVectorDynamic<>& Qc) override;
    virtual void IntFromDescriptor(const unsigned int off_v,
                                   ChStateDelta& v,
                                   const unsigned int off_L,
                                   ChVectorDynamic<>& L) override;

    // Override/implement system functions of ChPhysicsItem
    // (to assemble/manage data for system solver)

    virtual void InjectConstraints(ChSystemDescriptor& descriptor) override;
    virtual void ConstraintsBiReset() override;
    virtual void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false) override;
    virtual void ConstraintsBiLoad_Ct(double factor = 1) override;
    virtual void LoadConstraintJacobians() override;
    virtual void ConstraintsFetch_react(double factor = 1) override;

  private:
    double react;

    std::shared_ptr<ChFunction> offset_function;

    // used as an interface to the solver.
    ChConstraintTwoGeneric m_constraint1;

    std::shared_ptr<fea::ChNodeFEAbase> m_node1;
    std::shared_ptr<fea::ChNodeFEAbase> m_node2;
    std::shared_ptr<fea::ChFieldBase> m_field_1;
    std::shared_ptr<fea::ChFieldBase> m_field_2;


    // Provide dummy implementation for the following 4 members tht are meaningless anyway for this kind of link:

    /// Get the link frame 1, on the 1st node, expressed in the absolute frame.
    virtual ChFramed GetFrame1Abs() const override { return ChFramed(); }
    /// Get the link frame 2, on the 2nd node, expressed in the absolute frame.
    virtual ChFramed GetFrame2Abs() const override { return ChFramed(); }
    /// Get reaction force and torque on 1st node, expressed on link frame 1.
    virtual ChWrenchd GetReaction1() const override { return { ChVector3d(react,0,0), VNULL}; }
    /// Get reaction force and torque on 2nd node, expressed on link frame 2.
    virtual ChWrenchd GetReaction2() const override { return { ChVector3d(-react,0,0), VNULL}; }
    
};






/////////////////////////////////////////////////////////////////////////////////////////////////

/// Constraint between a node with displacement field and a ChBodyFrame (ex a ChBody).

class ChApi ChLinkFieldFrame : public ChLinkBase {
  public:
    ChLinkFieldFrame();
    ChLinkFieldFrame(const ChLinkFieldFrame& other);
    ~ChLinkFieldFrame() {} 
    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkFieldFrame* Clone() const override { return new ChLinkFieldFrame(*this); }

    /// Get the number of scalar variables affected by constraints in this link.
    virtual unsigned int GetNumAffectedCoords() override { return 3 + 7; }

    /// Number of scalar constraints.
    virtual unsigned int GetNumConstraintsBilateral() override { return 3; }

    // Get constraint violations.
    virtual ChVectorDynamic<> GetConstraintViolation() const override;

    /// Initialize the constraint, given the a multiphysics node, its displacement field,
    /// and a ChBodyFrame
    virtual int Initialize(std::shared_ptr<ChNodeFEAbase> node,         ///< node, associated to the displacement field
                           std::shared_ptr<ChFieldDisplacement3D> field,///< displacement field of node 
                           std::shared_ptr<ChBodyFrame> body,           ///< body (frame) to join
                           const ChVector3d* pos = 0                    ///< optional attachment position in absolute coordinates.
    );

    /// Get the connected xyz node (point).
    std::shared_ptr<fea::ChNodeFEAbase> GetConstrainedNode() { return m_node; }

    /// Get the connected body (frame).
    std::shared_ptr<ChBodyFrame> GetConstrainedBodyFrame() { return m_body; }

    /// Return the link frame, expressed in absolute coordinates.
    ChFrame<> GetFrameNodeAbs() const;

    /// Get the attachment position, in the coordinates of the body.
    const ChVector3d& GetAttachPosition() const { return m_csys.pos; }

    /// Get the attachment reference, in the coordinates of the body.
    const ChCoordsys<>& GetAttachReference() const { return m_csys; }

    /// Set the attachment position, expressed in the coordinates of the body.
    /// This function may be called only after initialization.
    void SetAttachPositionInBodyCoords(const ChVector3d& pos_loc) { m_csys.pos = pos_loc; }

    /// Set the attachment position, expressed in absolute coordinates.
    /// This function may be called only after initialization.
    void SetAttachPositionInAbsoluteCoords(const ChVector3d& pos_abs) { m_csys.pos = m_body->TransformPointParentToLocal(pos_abs); }

    /// Set the attachment reference, expressed in the coordinates of the body.
    /// This function may be called only after initialization.
    void SetAttachReferenceInBodyCoords(const ChCoordsys<>& csys_loc) { m_csys = csys_loc; }

    /// Set the attachment position, expressed in absolute coordinates.
    /// This function may be called only after initialization.
    void SetAttachReferenceInAbsoluteCoords(const ChCoordsys<>& csys_abs) { m_csys = m_body->GetCoordsys().TransformParentToLocal(csys_abs); }

    /// Get the reaction force on the node, expressed in the link coordinate system.
    ChVector3d GetReactionOnNode() const { return m_react; }

    /// Get the reaction force on the body, at the attachment point, expressed in the link coordinate system.
    ChVector3d GetReactionOnBody() const { return -m_react; }

    /// Update all auxiliary data of the gear transmission at given time
    virtual void Update(double time, UpdateFlags update_flags) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

    // Override/implement interfaces for global state vectors, see ChPhysicsItem for comments.

    virtual void IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) override;
    virtual void IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) override;
    virtual void IntLoadResidual_CqL(const unsigned int off_L, ChVectorDynamic<>& R, const ChVectorDynamic<>& L, const double c) override;
    virtual void IntLoadConstraint_C(const unsigned int off,
                                     ChVectorDynamic<>& Qc,
                                     const double c,
                                     const double c_vel,  ///< the scaling factor if the constraint is at speed level
                                     bool do_clamp,
                                     double recovery_clamp) override;
    virtual void IntToDescriptor(const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const ChVectorDynamic<>& R,
                                 const unsigned int off_L,
                                 const ChVectorDynamic<>& L,
                                 const ChVectorDynamic<>& Qc) override;
    virtual void IntFromDescriptor(const unsigned int off_v, ChStateDelta& v, const unsigned int off_L, ChVectorDynamic<>& L) override;

    // Override/implement system functions of ChPhysicsItem
    // (to assemble/manage data for system solver)

    virtual void InjectConstraints(ChSystemDescriptor& descriptor) override;
    virtual void ConstraintsBiReset() override;
    virtual void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false) override;
    virtual void ConstraintsBiLoad_Ct(double factor = 1) override;
    virtual void LoadConstraintJacobians() override;
    virtual void ConstraintsFetch_react(double factor = 1) override;

  private:
    ChVector3d m_react;

    // used as an interface to the solver.
    ChConstraintTwoGeneric m_constraint1;
    ChConstraintTwoGeneric m_constraint2;
    ChConstraintTwoGeneric m_constraint3;

    std::shared_ptr<fea::ChNodeFEAbase> m_node;
    std::shared_ptr<fea::ChFieldDisplacement3D> m_field;
    std::shared_ptr<ChBodyFrame> m_body;

    // Coordinate system, attached to the body, whose origin is
    // constrained to coincide with the node's position.
    ChCoordsys<> m_csys;

    // Provide dummy implementation for the following 4 members tht are meaningless anyway for this kind of link:

    /// Get the link frame 1, on the connected node, expressed in the absolute frame.
    virtual ChFramed GetFrame1Abs() const override { return GetFrameNodeAbs(); }

    /// Get the link frame 2, on the body, expressed in the absolute frame.
    virtual ChFramed GetFrame2Abs() const override { return ChFramed(); }  //// TODO

    /// Get reaction force and torque on node, expressed on link frame 1.
    virtual ChWrenchd GetReaction1() const override { return {GetReactionOnNode(), VNULL}; }

    /// Get reaction force and torque on frame, expressed on link frame 2.
    virtual ChWrenchd GetReaction2() const override { return {GetReactionOnBody(), VNULL}; }
};




/// @} fea_constraints

}  // end namespace fea
}  // end namespace chrono

#endif
