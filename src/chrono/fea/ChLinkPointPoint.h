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

#ifndef CHLINKPOINTPOINT_H
#define CHLINKPOINTPOINT_H

#include "chrono/physics/ChLinkBase.h"
#include "chrono/solver/ChConstraintTwoGeneric.h"
#include "chrono/fea/ChNodeFEAxyz.h"

namespace chrono {

class ChIndexedNodes;  // forward ref

namespace fea {

/// @addtogroup fea_constraints
/// @{

/// Class for creating a constraint between two xyz FEA nodes (points).
/// The two nodes will be joined, as overlapping. Nodes are 3-DOF points that are used in point-based primitives, such
/// as finite elements.
class ChApi ChLinkPointPoint : public ChLinkBase {
  private:
    ChVector<> react;

    // used as an interface to the solver.
    ChConstraintTwoGeneric constraint1;
    ChConstraintTwoGeneric constraint2;
    ChConstraintTwoGeneric constraint3;

    std::shared_ptr<fea::ChNodeFEAxyz> mnodeA;
    std::shared_ptr<fea::ChNodeFEAxyz> mnodeB;

  public:
    ChLinkPointPoint();
    ChLinkPointPoint(const ChLinkPointPoint& other);
    ~ChLinkPointPoint() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkPointPoint* Clone() const override { return new ChLinkPointPoint(*this); }

    /// Get the number of scalar variables affected by constraints in this link
    virtual int GetNumCoords() override { return 3 + 3; }

    /// Number of scalar constraints
    virtual int GetDOC_c() override { return 3; }

    /// To get reaction force, expressed in link coordinate system:
    virtual ChVector<> Get_react_force() override { return GetReactionOnNode(); }

    // Get constraint violations
    virtual ChVectorDynamic<> GetConstraintViolation() const override;

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

    virtual ChCoordsys<> GetLinkAbsoluteCoords() override { return CSYSNORM; }

    /// Use this function after object creation, to initialize it, given
    /// the two nodes join.
    /// The attachment position is the actual position of the node.
    /// Note, mnodes must belong to the same ChSystem.
    virtual int Initialize(std::shared_ptr<ChNodeFEAxyz> anodeA,  ///< xyz node (point) to join
                           std::shared_ptr<ChNodeFEAxyz> anodeB   ///< xyz node (point) to join
                           );

    /// Get the 1st connected xyz node (point)
    std::shared_ptr<fea::ChNodeFEAxyz> GetConstrainedNodeA() const { return this->mnodeA; }

    /// Get the 2nd connected xyz node (point)
    std::shared_ptr<fea::ChNodeFEAxyz> GetConstrainedNodeB() const { return this->mnodeB; }

    /// Get the reaction force considered as applied to ChShaft.
    ChVector<> GetReactionOnNode() const { return -react; }

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
};

/// @} fea_constraints

}  // end namespace fea
}  // end namespace chrono

#endif
