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

#ifndef CHLINKBEAMIGASLIDER_H
#define CHLINKBEAMIGASLIDER_H

#include "chrono/physics/ChBodyFrame.h"
#include "chrono/physics/ChLinkBase.h"
#include "chrono/solver/ChConstraintNgeneric.h"

#include "chrono/fea/ChElementBeamIGA.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"

namespace chrono {

class ChIndexedNodes;  // forward ref

namespace fea {

/// @addtogroup fea_constraints
/// @{

/// Class for allowin an IGA beam to slide inside a 'outlet' represented
/// by the x axis of a coordinate system floating with a ChBodyFrame.
/// The parameteric coordinate of the point of the spline that correspond
/// to the outlet is automatically updated during the sliding motion.

class ChApi ChLinkBeamIGAslider : public ChLinkBase {
  private:
    ChVector<> m_react;

    // used as an interface to the solver.
    // ChConstraintNgeneric constraint1;
    ChConstraintNgeneric constraint2;
    ChConstraintNgeneric constraint3;

    std::vector<std::shared_ptr<fea::ChElementBeamIGA>> m_beams;
    std::shared_ptr<ChBodyFrame> m_body;

    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> m_nodes;

    int order;
    size_t active_element;
    double tau;

    // Coordinate system, attached to the body, whose origin is
    // constrained to coincide with the node's position.
    ChCoordsys<> m_csys;

  public:
    ChLinkBeamIGAslider();
    ChLinkBeamIGAslider(const ChLinkBeamIGAslider& other);
    ~ChLinkBeamIGAslider() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkBeamIGAslider* Clone() const override { return new ChLinkBeamIGAslider(*this); }

    /// Get the number of scalar variables affected by constraints in this link
    virtual int GetNumCoords() override { return (int)m_nodes.size() * 3 + 7; }

    /// Number of scalar constraints.
    virtual int GetDOC_c() override { return 2; }

    /// Reaction force on the body, at the attachment point, expressed in the link coordinate frame.
    virtual ChVector<> Get_react_force() override { return GetReactionOnBody(); }

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

    /// Initialize this constraint, given the node and element(s).
    /// The attachment position is the actual position of the node (unless
    /// otherwise defined, using the optional 'pos' parameter).
    /// Note: the node and body must belong to the same ChSystem.
    virtual int Initialize(
        std::vector<std::shared_ptr<fea::ChElementBeamIGA>>& melements,  ///< elements that must slide
        std::shared_ptr<ChBodyFrame> body,  ///< body (frame) representing the slider outlet
        ChVector<>* pos = 0                 ///< attachment position in absolute coordinates (X axis is outlet dir)
    );

    /// Get the connected elements
    std::vector<std::shared_ptr<ChElementBeamIGA>>& GetConstrainedElements() { return m_beams; }

    /// Get the connected body (frame).
    std::shared_ptr<ChBodyFrame> GetConstrainedBodyFrame() { return m_body; }

    /// Get the attachment position, in the coordinates of the body.
    const ChVector<>& GetAttachPosition() const { return m_csys.pos; }

    /// Get the attachment reference, in the coordinates of the body.
    const ChCoordsys<>& GetAttachReference() const { return m_csys; }

    /// Set the attachment position, expressed in the coordinates of the body.
    /// This function may be called only after initialization.
    void SetAttachPositionInBodyCoords(const ChVector<>& pos_loc) { m_csys.pos = pos_loc; }

    /// Set the attachment position, expressed in absolute coordinates.
    /// This function may be called only after initialization.
    void SetAttachPositionInAbsoluteCoords(const ChVector<>& pos_abs) {
        m_csys.pos = m_body->TransformPointParentToLocal(pos_abs);
    }

    /// Set the attachment reference, expressed in the coordinates of the body.
    /// This function may be called only after initialization.
    void SetAttachReferenceInBodyCoords(const ChCoordsys<>& csys_loc) { m_csys = csys_loc; }

    /// Set the attachment position, expressed in absolute coordinates.
    /// This function may be called only after initialization.
    void SetAttachReferenceInAbsoluteCoords(const ChCoordsys<>& csys_abs) {
        m_csys = m_body->coord.TransformParentToLocal(csys_abs);
    }

    /// Get the reaction force on the node, expressed in the link coordinate system.
    ChVector<> GetReactionOnSpline() const { return m_react; }

    /// Get the reaction force on the body, at the attachment point, expressed in the link coordinate system.
    ChVector<> GetReactionOnBody() const { return -m_react; }

    //
    // UPDATE FUNCTIONS
    //

    /// Update all auxiliary data of the gear transmission at given time
    virtual void Update(double mytime, bool update_assets = true) override;

    //
    // STREAMING
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

  private:
    virtual void UpdateNodes();
};

/// @} fea_constraints

}  // end namespace fea
}  // end namespace chrono

#endif
