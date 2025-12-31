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

#ifndef CHLINK_NODE_FACE_H
#define CHLINK_NODE_FACE_H

#include "chrono/physics/ChLinkBase.h"
#include "chrono/solver/ChConstraintTwoTuples.h"

#include "chrono/fea/ChNodeFEAxyz.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_constraints
/// @{

/// Constraint between an FEA node (point) and a triangular face given by three FEA nodes.
/// The triangular face element is assumed to have linear shape function.
/// The node can be offset with respect to the face.
class ChApi ChLinkNodeFace : public ChLinkBase {
  public:
    ChLinkNodeFace();
    ChLinkNodeFace(const ChLinkNodeFace& other);
    ~ChLinkNodeFace() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkNodeFace* Clone() const override { return new ChLinkNodeFace(*this); }

    /// Get the number of scalar variables affected by constraints in this link.
    virtual unsigned int GetNumAffectedCoords() override {
        return m_point->GetNumCoords() + m_triangle->GetNumCoords();
    }

    /// Number of scalar constraints.
    virtual unsigned int GetNumConstraintsBilateral() override { return 3; }

    /// Return the link frame, expressed in absolute coordinates.
    ChFrame<> GetFrameNodeAbs() const { return ChFrame<>(m_point->GetPos(), QUNIT); }

    /// Initialize this constraint, given an XYZ node and an XYZ triangle.
    /// The attachment position is the actual position of the node.
    /// All nodes must belong to the same ChSystem.
    virtual void Initialize(std::shared_ptr<ChNodeFEAxyz> nodeA,   ///< node (point) to join
                            std::shared_ptr<ChNodeFEAxyz> nodeB1,  ///< triangle corner 1
                            std::shared_ptr<ChNodeFEAxyz> nodeB2,  ///< triangle corner 2
                            std::shared_ptr<ChNodeFEAxyz> nodeB3   ///< triangle corner 3
    );

    /// Initialize this constraint, given an XYZ node and an XYZrot triangle.
    /// The attachment position is the actual position of the node.
    /// All nodes must belong to the same ChSystem.
    virtual void Initialize(std::shared_ptr<ChNodeFEAxyz> nodeA,      ///< node (point) to join
                            std::shared_ptr<ChNodeFEAxyzrot> nodeB1,  ///< triangle corner 1
                            std::shared_ptr<ChNodeFEAxyzrot> nodeB2,  ///< triangle corner 2
                            std::shared_ptr<ChNodeFEAxyzrot> nodeB3   ///< triangle corner 3
    );

    /// Initialize this constraint, given an XYZrot node and an XYZ triangle.
    /// The attachment position is the actual position of the node.
    /// All nodes must belong to the same ChSystem.
    virtual void Initialize(std::shared_ptr<ChNodeFEAxyzrot> nodeA,  ///< node (point) to join
                            std::shared_ptr<ChNodeFEAxyz> nodeB1,    ///< triangle corner 1
                            std::shared_ptr<ChNodeFEAxyz> nodeB2,    ///< triangle corner 2
                            std::shared_ptr<ChNodeFEAxyz> nodeB3     ///< triangle corner 3
    );

    /// Initialize this constraint, given an XYZrot node and an XYZrot triangle.
    /// The attachment position is the actual position of the node.
    /// All nodes must belong to the same ChSystem.
    virtual void Initialize(std::shared_ptr<ChNodeFEAxyzrot> nodeA,   ///< node (point) to join
                            std::shared_ptr<ChNodeFEAxyzrot> nodeB1,  ///< triangle corner 1
                            std::shared_ptr<ChNodeFEAxyzrot> nodeB2,  ///< triangle corner 2
                            std::shared_ptr<ChNodeFEAxyzrot> nodeB3   ///< triangle corner 3
    );

    /// Set the area coordinates to specify where the point A is connected on triangle.
    /// These are 0..1 values, one with respect to point B2, the other with respect to B3
    /// (the third area coord follows automatically as 1-s2-s3).
    virtual void SetAreaCoords(double s2, double s3) {
        m_s2 = s2;
        m_s3 = s3;
    }

    /// Get the area coordinates, as set with respect to B2 and B3.
    /// (the third area coord follows automatically as 1-s2-s3).
    virtual void GetAreaCoords(double& s2, double& s3) const {
        s2 = m_s2;
        s3 = m_s3;
    }

    /// Set an optional offset of the point with respect to triangle, along triangle normal.
    /// Note that it is better to avoid large offsets. Better if offset is zero.
    virtual void SetOffset(const double d) { m_d = d; }

    /// Get the imposed offset of the point with respect to the triangle.
    virtual double GetOffset() const { return m_d; }

    /// Get the reaction force considered as applied to the node, in abs coords.
    ChVector3d GetReactionOnNode() const { return -m_react; }

    /// Get the reaction force considered as applied to the triangle, in abs coords.
    ChVector3d GetReactionOnTriangle() const { return -m_react; }

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
    /// Complete initialization.
    void CompleteInitialization();

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

  private:
    enum class FeaNodeType { XYZ, XYZrot };

    /// Base class for a point given by an FEA node.
    struct PointNode {
        virtual ~PointNode() {}
        virtual FeaNodeType GetType() const = 0;
        virtual unsigned int GetNumCoords() const = 0;
        virtual const ChVector3d& GetPos() const = 0;
        virtual ChConstraintTuple* CreateConstraintTuple() = 0;
    };

    /// Base class for a triangle given by three FEA nodes.
    struct TriangleNodes {
        virtual ~TriangleNodes() {}
        virtual FeaNodeType GetType() const = 0;
        virtual unsigned int GetNumCoords() const = 0;
        virtual const ChVector3d& GetPos1() const = 0;
        virtual const ChVector3d& GetPos2() const = 0;
        virtual const ChVector3d& GetPos3() const = 0;
        virtual ChConstraintTuple* CreateConstraintTuple() = 0;
    };

    bool m_initialized;

    ChVector3d m_react;

    // used as an interface to the solver
    ChConstraintTwoTuples constraint1;
    ChConstraintTwoTuples constraint2;
    ChConstraintTwoTuples constraint3;

    std::unique_ptr<PointNode> m_point;
    std::unique_ptr<TriangleNodes> m_triangle;

    double m_s1;
    double m_s2;
    double m_s3;
    double m_d;

    friend struct PointNodeXYZ;
    friend struct PointNodeXYZrot;
    friend struct TriangleNodesXYZ;
    friend struct TriangleNodesXYZrot;
};

/// @} fea_constraints

}  // end namespace fea
}  // end namespace chrono

#endif
