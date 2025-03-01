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

#ifndef CHLINKPOINTTRIFACE_H
#define CHLINKPOINTTRIFACE_H

#include <array>

#include "chrono/physics/ChLinkBase.h"
#include "chrono/solver/ChConstraintTwoTuplesContactN.h"

#include "chrono/fea/ChNodeFEAxyz.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"

namespace chrono {

class ChIndexedNodes;  // forward ref

namespace fea {

/// @addtogroup fea_constraints
/// @{

/// Utility class for using the ChLinkNodeFace constraint.
class ChApi ChTriangleNodesXYZ : public ChVariableTupleCarrier_3vars<3, 3, 3> {
  public:
    std::shared_ptr<fea::ChNodeFEAxyz> node1;
    std::shared_ptr<fea::ChNodeFEAxyz> node2;
    std::shared_ptr<fea::ChNodeFEAxyz> node3;

    virtual ChVariables* GetVariables1() override { return &node1->Variables(); }
    virtual ChVariables* GetVariables2() override { return &node2->Variables(); }
    virtual ChVariables* GetVariables3() override { return &node3->Variables(); }
};

/// Constraint between an xyz FEA node (point) and a triangular face given by three xyz FEA nodes
/// The triangular face element is assumed to have linear shape function (e.g., the face of a tetrahedron or a
/// triangular shell). The node can be offset respect to the face.
class ChApi ChLinkNodeFace : public ChLinkBase {
  public:
    ChLinkNodeFace();
    ChLinkNodeFace(const ChLinkNodeFace& other);
    ~ChLinkNodeFace() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkNodeFace* Clone() const override { return new ChLinkNodeFace(*this); }

    /// Get the number of scalar variables affected by constraints in this link.
    virtual unsigned int GetNumAffectedCoords() override { return 3 + 3 + 3 + 3; }

    /// Number of scalar constraints.
    virtual unsigned int GetNumConstraintsBilateral() override { return 3; }

    /// Return the link frame, expressed in absolute coordinates.
    ChFrame<> GetFrameNodeAbs() const { return ChFrame<>(m_node->GetPos(), QUNIT); }

    /// Initialize this constraint, given the node and the triangle to join.
    /// The attachment position is the actual position of the node.
    /// All nodes must belong to the same ChSystem.
    virtual int Initialize(std::shared_ptr<ChNodeFEAxyz> anodeA,   ///< xyz node (point) to join
                           std::shared_ptr<ChNodeFEAxyz> anodeB1,  ///< triangle: corner n.1
                           std::shared_ptr<ChNodeFEAxyz> anodeB2,  ///< triangle: corner n.2
                           std::shared_ptr<ChNodeFEAxyz> anodeB3   ///< triangle: corner n.3
    );

    /// Set the area coordinates to specify where the point A is connected on triangle.
    /// These are 0..1 values, one respect to point B2, the other respect to B3
    /// (the third area coord follows automatically as 1-s2-s3).
    /// The Initialize() function initialize this automatically.
    virtual void SetAreaCoords(const double ms2, const double ms3) {
        s2 = ms2;
        s3 = ms3;
    }

    /// Get the area coordinates, as set with respect to B2 and B3.
    /// (the third area coord follows automatically as 1-s2-s3).
    virtual void GetAreaCoords(double& ms2, double& ms3) const {
        ms2 = s2;
        ms3 = s3;
    }

    /// Set an optional offset of the point with respect to triangle, along triangle normal.
    /// Note that it is better to avoid large offsets. Better if offset is zero.
    /// The Initialize() function initialize this automatically.
    virtual void SetOffset(const double md) { d = md; }

    /// Get the imposed offset of the point with respect to triangle.
    virtual double GetOffset(double md) const { return d; }

    /// Get the connected xyz node (point).
    std::shared_ptr<fea::ChNodeFEAxyz> GetNode() const { return this->m_node; }

    /// Get the connected triangle.
    std::array<std::shared_ptr<fea::ChNodeFEAxyz>, 3> GetTriangle() const {
        return std::array<std::shared_ptr<fea::ChNodeFEAxyz>, 3>{
            {this->m_triangle.node1, this->m_triangle.node2, this->m_triangle.node3}};
    }

    /// Get the reaction force considered as applied to the node, in abs coords.
    ChVector3d GetReactionOnNode() const { return -react; }

    /// Get the reaction force considered as applied to the triangle, in abs coords.
    ChVector3d GetReactionOnTriangle() const { return -react; }

    /// Update all auxiliary data of the gear transmission at given time.
    virtual void Update(double time, bool update_assets) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

    // STATE FUNCTIONS

    // (override/implement interfaces for global state vectors, see ChPhysicsItem for comments.)
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

    // used as an interface to the solver.
    ChConstraintTwoTuples<ChNodeFEAxyz, ChTriangleNodesXYZ> constraint1;
    ChConstraintTwoTuples<ChNodeFEAxyz, ChTriangleNodesXYZ> constraint2;
    ChConstraintTwoTuples<ChNodeFEAxyz, ChTriangleNodesXYZ> constraint3;

    std::shared_ptr<ChNodeFEAxyz> m_node;
    ChTriangleNodesXYZ m_triangle;

    double s2, s3;
    double d;

    /// Get the link frame 1, on the connected node, expressed in the absolute frame.
    virtual ChFramed GetFrame1Abs() const override { return GetFrameNodeAbs(); }

    /// Get the link frame 2, on the face, expressed in the absolute frame.
    virtual ChFramed GetFrame2Abs() const override { return ChFramed(); }  //// TODO

    /// Get reaction force and torque on node, expressed on link frame 1.
    virtual ChWrenchd GetReaction1() const override {
        return {GetFrame1Abs().TransformDirectionParentToLocal(GetReactionOnNode()), VNULL};
    }

    /// Get reaction force and torque on triangle, expressed on link frame 2.
    virtual ChWrenchd GetReaction2() const override {
        return {GetFrame2Abs().TransformDirectionParentToLocal(GetReactionOnTriangle()), VNULL};
        //// TODO: check torque
    }
};

// ---------------------------------------------------------------------------------

// The following classes might be removed if ChNodeFEAxyzrot were inherited from ChNodeFEAxys.
// Planned for future

/// Utility class for using the ChLinkNodeFaceRot constraint.
class ChApi ChTriangleNodesXYZrot : public ChVariableTupleCarrier_3vars<6, 6, 6> {
  public:
    std::shared_ptr<fea::ChNodeFEAxyzrot> node1;
    std::shared_ptr<fea::ChNodeFEAxyzrot> node2;
    std::shared_ptr<fea::ChNodeFEAxyzrot> node3;

    virtual ChVariables* GetVariables1() { return &node1->Variables(); };
    virtual ChVariables* GetVariables2() { return &node2->Variables(); };
    virtual ChVariables* GetVariables3() { return &node3->Variables(); };
};

/// Constraint between an xyz FEA node (point) and a triangular face given by three xyzrot FEA nodes.
/// The triangular face element is assumed to have linear shape function.
/// The node can be offset respect to the face.
class ChApi ChLinkNodeFaceRot : public ChLinkBase {
  public:
    ChLinkNodeFaceRot();
    ChLinkNodeFaceRot(const ChLinkNodeFaceRot& other);
    ~ChLinkNodeFaceRot() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkNodeFaceRot* Clone() const override { return new ChLinkNodeFaceRot(*this); }

    /// Get the number of scalar variables affected by constraints in this link.
    virtual unsigned int GetNumAffectedCoords() override { return 3 + 6 + 6 + 6; }

    /// Number of scalar constraints.
    virtual unsigned int GetNumConstraintsBilateral() override { return 3; }

    /// Return the link frame, expressed in absolute coordinates.
    ChFrame<> GetFrameNodeAbs() const { return ChFrame<>(m_node->GetPos(), QUNIT); }

    /// Initialize this constraint, given the node and the triangle to join.
    /// The attachment position is the actual position of the node.
    /// All nodes must belong to the same ChSystem.
    virtual int Initialize(std::shared_ptr<ChNodeFEAxyz> nodeA,      ///< xyz node (point) to join
                           std::shared_ptr<ChNodeFEAxyzrot> nodeB1,  ///< triangle: corner n.1
                           std::shared_ptr<ChNodeFEAxyzrot> nodeB2,  ///< triangle: corner n.2
                           std::shared_ptr<ChNodeFEAxyzrot> nodeB3   ///< triangle: corner n.3
    );

    /// Set the area coordinates to specify where the point A is connected on triangle.
    /// These are 0..1 values, one respect to point B2, the other respect to B3
    /// (the third area coord follows automatically as 1-s2-s3).
    /// The Initialize() function initialize this automatically.
    virtual void SetAreaCoords(const double ms2, const double ms3) {
        s2 = ms2;
        s3 = ms3;
    }

    /// Get the area coordinates, as set with respect to B2 and B3.
    /// (the third area coord follows automatically as 1-s2-s3).
    virtual void GetAreaCoords(double& ms2, double& ms3) const {
        ms2 = s2;
        ms3 = s3;
    }

    /// Set an optional offset of point A respect to triangle, along triangle normal.
    /// Note that it is better to avoid large offsets. Better if offset is zero.
    /// The Initialize() function initialize this automatically.
    virtual void SetOffset(const double md) { d = md; }

    /// Get the imposed offset of point A respect to triangle.
    virtual double GetOffset(double md) const { return d; }

    /// Get the connected xyz node (point).
    std::shared_ptr<fea::ChNodeFEAxyz> GetNode() const { return this->m_node; }

    /// Get the connected triangle.
    std::array<std::shared_ptr<fea::ChNodeFEAxyzrot>, 3> GetTriangle() const {
        return std::array<std::shared_ptr<fea::ChNodeFEAxyzrot>, 3>{
            {this->m_triangle.node1, this->m_triangle.node2, this->m_triangle.node3}};
    }

    /// Get the reaction force considered as applied to node A, in abs coords.
    ChVector3d GetReactionOnNode() const { return -react; }

    /// Get the reaction force considered as applied to the triangle, in abs coords.
    ChVector3d GetReactionOnTriangle() const { return -react; }

    /// Update all auxiliary data of the gear transmission at given time
    virtual void Update(double time, bool update_assets) override;

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

    // used as an interface to the solver.
    ChConstraintTwoTuples<ChNodeFEAxyz, ChTriangleNodesXYZrot> constraint1;
    ChConstraintTwoTuples<ChNodeFEAxyz, ChTriangleNodesXYZrot> constraint2;
    ChConstraintTwoTuples<ChNodeFEAxyz, ChTriangleNodesXYZrot> constraint3;

    std::shared_ptr<ChNodeFEAxyz> m_node;
    ChTriangleNodesXYZrot m_triangle;

    double s2, s3;
    double d;

    /// Get the link frame 1, on the connected node, expressed in the absolute frame.
    virtual ChFramed GetFrame1Abs() const override { return GetFrameNodeAbs(); }

    /// Get the link frame 2, on the face, expressed in the absolute frame.
    virtual ChFramed GetFrame2Abs() const override { return ChFramed(); }  //// TODO

    /// Get reaction force and torque on node, expressed on link frame 1.
    virtual ChWrenchd GetReaction1() const override {
        return {GetFrame1Abs().TransformDirectionParentToLocal(GetReactionOnNode()), VNULL};
    }

    /// Get reaction force and torque on triangle, expressed on link frame 2.
    virtual ChWrenchd GetReaction2() const override {
        return {GetFrame2Abs().TransformDirectionParentToLocal(GetReactionOnTriangle()), VNULL};
        //// TODO: check torque
    }
};

/// @} fea_constraints

}  // end namespace fea
}  // end namespace chrono

#endif
