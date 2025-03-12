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

#ifndef CHLINKDIRFRAME_H
#define CHLINKDIRFRAME_H

#include "chrono/physics/ChBodyFrame.h"
#include "chrono/physics/ChLinkBase.h"
#include "chrono/solver/ChConstraintTwoGeneric.h"
#include "chrono/fea/ChNodeFEAxyzD.h"

namespace chrono {

class ChIndexedNodes;  // forward ref

namespace fea {

/// @addtogroup fea_constraints
/// @{

/// Constraint between the direction of a FEA node of ChNodeFEAxyzD class, and a ChBodyFrame (frame).
/// The D direction of the ChNodeFEAxyzD is enforced to stay parallel to a given direction associated to the
/// ChBodyFrame.
class ChApi ChLinkNodeSlopeFrame : public ChLinkBase {
  public:
    ChLinkNodeSlopeFrame();
    ChLinkNodeSlopeFrame(const ChLinkNodeSlopeFrame& other);
    ~ChLinkNodeSlopeFrame() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkNodeSlopeFrame* Clone() const override { return new ChLinkNodeSlopeFrame(*this); }

    /// Get the number of scalar variables affected by constraints in this link.
    virtual unsigned int GetNumAffectedCoords() override { return 3 + 4; }

    /// Number of scalar constraints.
    virtual unsigned int GetNumConstraintsBilateral() override { return 2; }

    // Get constraint violations
    virtual ChVectorDynamic<> GetConstraintViolation() const override;

    // Get the link coordinate system, expressed in the absolute frame.
    ChFrame<> GetFrameNodeAbs() const;

    /// Initialize this constraint, given the node and body frame to join.
    /// The constrained direction is the actual direction of the node (unless
    /// otherwise defined, using the optional 'dir' parameter).
    /// Note: the node and body must belong to the same ChSystem.
    virtual int Initialize(std::shared_ptr<ChNodeFEAxyzD> node,  ///< xyzD node to join (with the direction)
                           std::shared_ptr<ChBodyFrame> body,    ///< body (frame) to join
                           ChVector3d* dir = nullptr             ///< direction in absolute coordinates
    );

    /// Get the connected xyzD node (point).
    virtual std::shared_ptr<ChNodeFEAxyzD> GetConstrainedNode() { return m_node; }

    /// Get the connected body (frame).
    virtual std::shared_ptr<ChBodyFrame> GetConstrainedBodyFrame() { return m_body; }

    /// Get the constrained direction, expressed in the reference coordinates of the body.
    ChVector3d GetDirection() const { return m_csys.rot.GetAxisX(); }

    /// Set the constrained direction, expressed in the reference coordinates of the body.
    /// This function may be called only after initialization.
    void SetDirectionInBodyCoords(const ChVector3d& dir_loc);

    /// Set the constrained direction, expressed in absolute coordinates.
    /// This function may be called only after initialization.
    void SetDirectionInAbsoluteCoords(const ChVector3d& dir_abs);

    /// Get the reaction torque on the node, expressed in the link coordinate system.
    ChVector3d GetReactionOnNode() const { return -GetReactionOnBody(); }

    /// Get the reaction torque on the body, expressed in the link coordinate system.
    ChVector3d GetReactionOnBody() const;

    /// Update all auxiliary data of the gear transmission at given time.
    virtual void Update(double time, bool update_assets) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

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
    virtual void ConstraintsBiLoad_C(double factor = 1., double recovery_clamp = 0.1, bool do_clamp = false) override;
    virtual void ConstraintsBiLoad_Ct(double factor = 1.) override;
    virtual void LoadConstraintJacobians() override;
    virtual void ConstraintsFetch_react(double factor = 1.) override;

  private:
    ChVector3d m_react;

    // used as an interface to the solver.
    ChConstraintTwoGeneric constraint1;
    ChConstraintTwoGeneric constraint2;

    std::shared_ptr<fea::ChNodeFEAxyzD> m_node;
    std::shared_ptr<ChBodyFrame> m_body;

    // Coordinate system, attached to the body, whose X direction is
    // constrained to remain parallel to the node's D direction.
    ChCoordsys<> m_csys;

    /// Get the link frame 1, on the connected node, expressed in the absolute frame.
    virtual ChFramed GetFrame1Abs() const override { return GetFrameNodeAbs(); }

    /// Get the link frame 2, on the body, expressed in the absolute frame.
    virtual ChFramed GetFrame2Abs() const override { return ChFramed(); }  //// TODO

    /// Get reaction force and torque on node, expressed on link frame 1.
    virtual ChWrenchd GetReaction1() const override { return {VNULL, GetReactionOnNode()}; }

    /// Get reaction force and torque on frame, expressed on link frame 2.
    virtual ChWrenchd GetReaction2() const override { return {VNULL, GetReactionOnBody()}; }
};

/// @} fea_constraints

}  // end namespace fea
}  // end namespace chrono

#endif
