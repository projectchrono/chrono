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

#ifndef CHLINKPOINTFRAME_H
#define CHLINKPOINTFRAME_H

#include "chrono/physics/ChBodyFrame.h"
#include "chrono/physics/ChLinkBase.h"
#include "chrono/solver/ChConstraintTwoGeneric.h"
#include "chrono/fea/ChNodeFEAxyz.h"

namespace chrono {

class ChIndexedNodes;  // forward ref

namespace fea {

/// @addtogroup fea_constraints
/// @{

/// Class for creating a constraint between an FEA node of ChNodeFEAxyz type
/// and a ChBodyFrame (frame) object.
/// The node position is enforced to coincide to a given position associated
/// with the ChBodyFrame.

class ChApi ChLinkPointFrame : public ChLinkBase {

  private:
    ChVector<> m_react;

    // used as an interface to the solver.
    ChConstraintTwoGeneric constraint1;
    ChConstraintTwoGeneric constraint2;
    ChConstraintTwoGeneric constraint3;

    std::shared_ptr<fea::ChNodeFEAxyz> m_node;
    std::shared_ptr<ChBodyFrame> m_body;

    // Coordinate system, attached to the body, whose origin is
    // constrained to coincide with the node's position.
    ChCoordsys<> m_csys;

  public:
    ChLinkPointFrame();
    ChLinkPointFrame(const ChLinkPointFrame& other);
    ~ChLinkPointFrame() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkPointFrame* Clone() const override { return new ChLinkPointFrame(*this); }

    /// Get the number of scalar variables affected by constraints in this link
    virtual int GetNumCoords() override { return 3 + 7; }

    /// Number of scalar constraints.
    virtual int GetDOC_c() override { return 3; }

    /// Reaction force on the body, at the attachment point, expressed in the link coordinate frame.
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


    /// Initialize this constraint, given the node and body frame to join.
    /// The attachment position is the actual position of the node (unless
    /// otherwise defined, using the optional 'pos' parameter).
    /// Note: the node and body must belong to the same ChSystem.
    virtual int Initialize(std::shared_ptr<ChNodeFEAxyz> node,  ///< xyz node (point) to join
                           std::shared_ptr<ChBodyFrame> body,   ///< body (frame) to join
                           ChVector<>* pos = 0                  ///< attachment position in absolute coordinates
                           );

    /// Get the connected xyz node (point).
    std::shared_ptr<fea::ChNodeFEAxyz> GetConstrainedNode() { return m_node; }

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
    ChVector<> GetReactionOnNode() const { return m_react; }

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
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};


/// Class for creating a constraint between an FEA node of ChNodeFEAxyz type
/// and a ChBodyFrame (frame) object.
/// The node position is constrained to a given coordinate system CSYS
/// that moves with the ChBodyFrame. The movements of the node respect
/// to X, Y, Z  axes of such CSYS can be costrained or not, 
/// depending on three boolean toggles. 
/// By default, XYZ are all constrained and the node follows the center of CSYS, 
/// s it is completely locked to it, but other options are, for example, that
/// you just enable the X constraint (so the node moves on the flat YZ plane)
/// or you just enable XY constraints (so the node moves along the Z direction) 
/// etc. 

class ChApi ChLinkPointFrameGeneric : public ChLinkBase {

  private:
    ChVector<> m_react;

	bool c_x;
    bool c_y;
    bool c_z;

    // used as an interface to the solver.
    ChConstraintTwoGeneric constraint1;
    ChConstraintTwoGeneric constraint2;
    ChConstraintTwoGeneric constraint3;

    std::shared_ptr<fea::ChNodeFEAxyz> m_node;
    std::shared_ptr<ChBodyFrame> m_body;

    // Coordinate system, attached to the body, whose origin is
    // constrained to coincide with the node's position.
    ChCoordsys<> m_csys;

  public:
    ChLinkPointFrameGeneric(bool mc_x = true, bool mc_y = true, bool mc_z = true);
    ChLinkPointFrameGeneric(const ChLinkPointFrameGeneric& other);
    ~ChLinkPointFrameGeneric() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkPointFrameGeneric* Clone() const override { return new ChLinkPointFrameGeneric(*this); }

    /// Get the number of scalar variables affected by constraints in this link
    virtual int GetNumCoords() override { return 3 + 7; }

    /// Number of scalar constraints.
    virtual int GetDOC_c() override { return ((int)c_x+(int)c_y+(int)c_z); }

    /// Reaction force on the body, at the attachment point, expressed in the link coordinate frame.
    virtual ChVector<> Get_react_force() override { return GetReactionOnBody(); }

    // Get constraint violations
    //ChMatrixNM<double, 3, 1> GetC() const;

	bool IsConstrainedX() { return c_x; }
    bool IsConstrainedY() { return c_y; }
    bool IsConstrainedZ() { return c_z; }

	/// Sets which movements (of frame 1 respect to frame 2) are constrained
    void SetConstrainedCoords(bool mc_x, bool mc_y, bool mc_z);

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


    /// Initialize this constraint, given the node and body frame to join.
    /// The attachment position is the actual position of the node (unless
    /// otherwise defined, using the optional 'pos' parameter).
    /// Note: the node and body must belong to the same ChSystem.
    virtual int Initialize(std::shared_ptr<ChNodeFEAxyz> node,  ///< xyz node (point) to join
                           std::shared_ptr<ChBodyFrame> body,   ///< body (frame) to join
                           ChVector<>* pos = 0                  ///< attachment position in absolute coordinates
                           );

    /// Initialize this constraint, given the node and body frame to join.
    /// The constraint frame is a coordinate system passed with the csys parameter,
	/// where csys is expressed in absolute coordinates
    virtual int Initialize(std::shared_ptr<ChNodeFEAxyz> node,  ///< xyz node (point) to join
                           std::shared_ptr<ChBodyFrame> body,   ///< body (frame) to join
                           chrono::ChCoordsys<>  csys_abs       ///< attachment position in absolute coordinates
                           );

    /// Get the connected xyz node (point).
    std::shared_ptr<fea::ChNodeFEAxyz> GetConstrainedNode() { return m_node; }

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
    ChVector<> GetReactionOnNode() const { return m_react; }

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
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

/// @} fea_constraints

}  // end namespace fea
}  // end namespace chrono

#endif
