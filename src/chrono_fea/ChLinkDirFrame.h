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

#ifndef CHLINKDIRFRAME_H
#define CHLINKDIRFRAME_H

#include "chrono/physics/ChBodyFrame.h"
#include "chrono/physics/ChLinkBase.h"
#include "chrono/solver/ChConstraintTwoGeneric.h"
#include "chrono_fea/ChNodeFEAxyzD.h"

namespace chrono {

class ChIndexedNodes;  // forward ref

namespace fea {

/// @addtogroup fea_constraints
/// @{

/// Class for creating a constraint between the direction of a FEA node
/// of ChNodeFEAxyzD class, and a ChBodyFrame (frame).
/// The D direction of the ChNodeFEAxyzD is enforced to stay parallel
/// to a given direction associated to the ChBodyFrame.

class ChApiFea ChLinkDirFrame : public ChLinkBase {
    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChLinkDirFrame)

  private:
    ChVector<> m_react;

    // used as an interface to the solver.
    ChConstraintTwoGeneric constraint1;
    ChConstraintTwoGeneric constraint2;

    std::shared_ptr<fea::ChNodeFEAxyzD> m_node;
    std::shared_ptr<ChBodyFrame> m_body;

    // Coordinate system, attached to the body, whose X direction is
    // constrained to remain parallel to the node's D direction.
    ChCoordsys<> m_csys;

  public:
    ChLinkDirFrame();
    ChLinkDirFrame(const ChLinkDirFrame& other);
    ~ChLinkDirFrame() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkDirFrame* Clone() const override { return new ChLinkDirFrame(*this); }

    /// Get the number of scalar variables affected by constraints in this link.
    virtual int GetNumCoords() override { return 3 + 4; }

    /// Number of scalar constraints.
    virtual int GetDOC_c() override { return 2; }

    /// Get the reaction torque on the body, expressed in the link coordinate system.
    virtual ChVector<> Get_react_torque() override { return GetReactionOnBody(); }

    // Get constraint violations
    ChMatrixNM<double, 2, 1> GetC() const;

    //
    // STATE FUNCTIONS
    //

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
    virtual void ConstraintsBiLoad_C(double factor = 1., double recovery_clamp = 0.1, bool do_clamp = false) override;
    virtual void ConstraintsBiLoad_Ct(double factor = 1.) override;
    virtual void ConstraintsLoadJacobians() override;
    virtual void ConstraintsFetch_react(double factor = 1.) override;

    // Other functions

    // Get the link coordinate system, expressed in the absolute frame.
    virtual ChCoordsys<> GetLinkAbsoluteCoords() override;

    /// Initialize this constraint, given the node and body frame to join.
    /// The constrained direction is the actual direction of the node (unless
    /// otherwise defined, using the optional 'dir' parameter).
    /// Note: the node and body must belong to the same ChSystem.
    virtual int Initialize(std::shared_ptr<ChNodeFEAxyzD> node,  ///< xyzD node to join (with the direction)
                           std::shared_ptr<ChBodyFrame> body,    ///< body (frame) to join
                           ChVector<>* dir = nullptr             ///< direction in absolute coordinates
                           );

    /// Get the connected xyzD node (point).
    virtual std::shared_ptr<ChNodeFEAxyzD> GetConstrainedNode() { return m_node; }

    /// Get the connected body (frame).
    virtual std::shared_ptr<ChBodyFrame> GetConstrainedBodyFrame() { return m_body; }

    /// Get the constrained direction, expressed in the reference coordinates of the body.
    ChVector<> GetDirection() const { return m_csys.rot.GetXaxis(); }

    /// Set the constrained direction, expressed in the reference coordinates of the body.
    /// This function may be called only after initialization.
    void SetDirectionInBodyCoords(const ChVector<>& dir_loc);

    /// Set the constrained direction, expressed in absolute coordinates.
    /// This function may be called only after initialization.
    void SetDirectionInAbsoluteCoords(const ChVector<>& dir_abs);

    /// Get the reaction torque on the node, expressed in the link coordinate system.
    ChVector<> GetReactionOnNode() const { return -GetReactionOnBody(); }

    /// Get the reaction torque on the body, expressed in the link coordinate system.
    ChVector<> GetReactionOnBody() const;

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
