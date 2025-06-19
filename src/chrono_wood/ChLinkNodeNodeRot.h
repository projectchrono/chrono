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

#ifndef CHLINKNODENODEROT_H
#define CHLINKNODENODEROT_H

#include "chrono_wood/ChWoodApi.h"
#include "chrono/physics/ChLinkBase.h"
#include "chrono/solver/ChConstraintTwoGeneric.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"

using namespace chrono::fea;

namespace chrono {

class ChIndexedNodes;  // forward ref

namespace wood {

/// @addtogroup fea_constraints
/// @{

/// Constraint between two xyz FEA nodes (points).
/// The two nodes will be joined, as overlapping. Nodes are 3-DOF points that are used in point-based primitives, such
/// as finite elements.
class ChWoodApi ChLinkNodeNodeRot : public ChLinkBase {
  public:
    ChLinkNodeNodeRot();
    ChLinkNodeNodeRot(const ChLinkNodeNodeRot& other);
    ~ChLinkNodeNodeRot() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkNodeNodeRot* Clone() const override { return new ChLinkNodeNodeRot(*this); }

    /// Get the number of scalar variables affected by constraints in this link.
    virtual unsigned int GetNumAffectedCoords() override { return 6 + 6; }

    /// Number of scalar constraints.
    virtual unsigned int GetNumConstraintsBilateral() override { return 6; }

    // Get constraint violations.
    virtual ChVectorDynamic<> GetConstraintViolation() const override;

    /// Return the link frame at node 1, expressed in absolute coordinates.
    ChFrame<> GetFrameNode1Abs() const { return ChFrame<>(m_node1->GetPos(), QUNIT); }

    /// Return the link frame at node 2, expressed in absolute coordinates.
    ChFrame<> GetFrameNode2Abs() const { return ChFrame<>(m_node2->GetPos(), QUNIT); }

    /// Initialize the constraint, given the two nodes to join.
    /// The attachment position is the actual position of the node.
    /// Note that nodes must belong to the same ChSystem.
    virtual int Initialize(std::shared_ptr<ChNodeFEAxyzrot> node1,  ///< xyz node (point) to join
                           std::shared_ptr<ChNodeFEAxyzrot> node2   ///< xyz node (point) to join
    );

    /// Get the 1st connected xyz node.
    std::shared_ptr<fea::ChNodeFEAxyzrot> GetNode1() const { return m_node1; }

    /// Get the 2nd connected xyz node.
    std::shared_ptr<fea::ChNodeFEAxyzrot> GetNode2() const { return m_node2; }

    /// Get the reaction force considered as applied to the 1st node.
    ChVector3d GetReactionOnNode1() const { return +react; }

    /// Get the reaction force considered as applied to the 2nd node.
    ChVector3d GetReactionOnNode2() const { return -react; }

    /// Update all auxiliary data of the gear transmission at given time.
    virtual void Update(double mytime, bool update_assets = true) override;

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
                                     bool do_clamp,
                                     double recovery_clamp) override;
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
    ChVector3d react;
    ChVector3d torque;

    // used as an interface to the solver.
    ChConstraintTwoGeneric m_constraint1;
    ChConstraintTwoGeneric m_constraint2;
    ChConstraintTwoGeneric m_constraint3;
    //
    ChConstraintTwoGeneric m_constraint4;
    ChConstraintTwoGeneric m_constraint5;
    ChConstraintTwoGeneric m_constraint6;
    

    std::shared_ptr<fea::ChNodeFEAxyzrot> m_node1;
    std::shared_ptr<fea::ChNodeFEAxyzrot> m_node2;

    /// Get the link frame 1, on the 1st node, expressed in the absolute frame.
    virtual ChFramed GetFrame1Abs() const override { return GetFrameNode1Abs(); }

    /// Get the link frame 2, on the 2nd node, expressed in the absolute frame.
    virtual ChFramed GetFrame2Abs() const override { return GetFrameNode2Abs(); }

    /// Get reaction force and torque on 1st node, expressed on link frame 1.
    virtual ChWrenchd GetReaction1() const override { return {GetReactionOnNode1(), VNULL}; }

    /// Get reaction force and torque on 2nd node, expressed on link frame 2.
    virtual ChWrenchd GetReaction2() const override { return {GetReactionOnNode2(), VNULL}; }
};

/// @} fea_constraints

}  // end namespace wood
}  // end namespace chrono

#endif
