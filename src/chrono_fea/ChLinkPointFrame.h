// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHLINKPOINTFRAME_H
#define CHLINKPOINTFRAME_H

#include "chrono/physics/ChBodyFrame.h"
#include "chrono/physics/ChLinkBase.h"
#include "chrono/solver/ChConstraintTwoGeneric.h"
#include "chrono_fea/ChNodeFEAxyz.h"

namespace chrono {

class ChIndexedNodes;  // forward ref

namespace fea {

/// @addtogroup fea_constraints
/// @{

/// Class for creating a constraint between a xyz FEA node (point)
/// and a ChBodyFrame (frame) object (that is, it fixes a 3-DOF point
/// to a 6-DOF frame).
/// Nodes are 3-DOF points that are used in point-based
/// primitives, such as ChMatterSPH or finite elements.
class ChApiFea ChLinkPointFrame : public ChLinkBase {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChLinkPointFrame, ChLinkBase);

  private:
    ChVector<> react;

    // used as an interface to the solver.
    ChConstraintTwoGeneric constraint1;
    ChConstraintTwoGeneric constraint2;
    ChConstraintTwoGeneric constraint3;

    std::shared_ptr<fea::ChNodeFEAxyz> mnode;
    std::shared_ptr<ChBodyFrame> body;

    ChCoordsys<> attach_reference;

  public:
    ChLinkPointFrame();
    ChLinkPointFrame(const ChLinkPointFrame& other);
    ~ChLinkPointFrame() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkPointFrame* Clone() const override { return new ChLinkPointFrame(*this); }

    /// Get the number of scalar variables affected by constraints in this link
    virtual int GetNumCoords() override { return 3 + 7; }

    /// Number of scalar costraints
    virtual int GetDOC_c() override { return 3; }

    /// To get reaction force, expressed in link coordinate system:
    virtual ChVector<> Get_react_force() override { return GetReactionOnBody(); }

    // Get constraint violations
    ChMatrixNM<double, 3, 1> GetC() const;

    //
    // STATE FUNCTIONS
    //

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

    virtual void InjectConstraints(ChSystemDescriptor& mdescriptor) override;
    virtual void ConstraintsBiReset() override;
    virtual void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false) override;
    virtual void ConstraintsBiLoad_Ct(double factor = 1) override;
    virtual void ConstraintsLoadJacobians() override;
    virtual void ConstraintsFetch_react(double factor = 1) override;

    // Other functions

    virtual ChCoordsys<> GetLinkAbsoluteCoords() override;

    /// Use this function after object creation, to initialize it, given
    /// the node and body to join.
    /// The attachment position is the actual position of the node (unless
    /// otherwise defines, using the optional 'mattach' parameter).
    /// Note, mnodes and mbody must belong to the same ChSystem.
    virtual int Initialize(std::shared_ptr<ChIndexedNodes> mnodes,  ///< nodes container
                           unsigned int mnode_index,                ///< index of the xyz node (point) to join
                           std::shared_ptr<ChBodyFrame> mbody,      ///< body (frame) to join
                           ChVector<>* mattach = 0  ///< optional: if not null, sets the attachment position in absolute coordinates
                           );
    /// Use this function after object creation, to initialize it, given
    /// the node and body frame to join.
    /// The attachment position is the actual position of the node (unless
    /// otherwise defines, using the optional 'mattach' parameter).
    /// Note, mnodes and mbody must belong to the same ChSystem.
    virtual int Initialize(std::shared_ptr<ChNodeFEAxyz> anode,  ///< xyz node (point) to join
                           std::shared_ptr<ChBodyFrame> mbody,   ///< body (frame) to join
                           ChVector<>* mattach = 0  ///< optional: if not null, sets the attachment position in absolute coordinates
                           );

    /// Get the connected xyz node (point)
    std::shared_ptr<fea::ChNodeFEAxyz> GetConstrainedNode() { return mnode; }

    /// Get the connected body (frame)
    std::shared_ptr<ChBodyFrame> GetConstrainedBodyFrame() { return body; }

    /// Get the attachment position, in the coordinates of the body.
    const ChVector<>& GetAttachPosition() const { return attach_reference.pos; }
    /// Set the attachment position, in the coordinates of the body
    void SetAttachPositionInBodyCoords(ChVector<> mattach) { attach_reference.pos = mattach; }
    /// Set the attachment position, in the absolute coordinates
    void SetAttachPositionInAbsoluteCoords(ChVector<> mattach) {
        attach_reference.pos = body->TransformPointParentToLocal(mattach);
    }

    /// Get the attachment reference, in the coordinates of the body.
    const ChCoordsys<>& GetAttachReference() const { return attach_reference; }
    /// Set the attachment reference, in the coordinates of the body
    void SetAttachReferenceInBodyCoords(ChCoordsys<> mattach) { attach_reference = mattach; }
    /// Set the attachment position, in the absolute coordinates
    void SetAttachReferenceInAbsoluteCoords(ChCoordsys<> mattach) {
        attach_reference = body->coord.TransformParentToLocal(mattach);
    }

    /// Get the reaction force considered as applied to ChShaft.
    ChVector<> GetReactionOnNode() const { return -react; }

    /// Get the reaction force considered as applied to ChBody.
    ChVector<> GetReactionOnBody() const { return react; }

    //
    // UPDATE FUNCTIONS
    //

    /// Update all auxiliary data of the gear transmission at given time
    virtual void Update(double mytime, bool update_assets = true) override;

    //
    // STREAMING
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

/// @} fea_constraints

}  // end namespace fea
}  // end namespace chrono

#endif
