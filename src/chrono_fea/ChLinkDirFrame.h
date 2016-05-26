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
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChLinkDirFrame, ChLinkBase);

  private:
    ChVector<> react;

    // used as an interface to the solver.
    ChConstraintTwoGeneric constraint1;
    ChConstraintTwoGeneric constraint2;

    std::shared_ptr<fea::ChNodeFEAxyzD> mnode;
    std::shared_ptr<ChBodyFrame> body;

    ChVector<> direction;
    ChCoordsys<> csys_direction;

  public:
    ChLinkDirFrame();
    ChLinkDirFrame(const ChLinkDirFrame& other);
    ~ChLinkDirFrame() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkDirFrame* Clone() const override { return new ChLinkDirFrame(*this); }

    /// Get the number of scalar variables affected by constraints in this link
    virtual int GetNumCoords() override { return 3 + 4; }

    /// Number of scalar costraints
    virtual int GetDOC_c() override { return 2; }

    /// To get reaction force, expressed in link coordinate system:
    virtual ChVector<> Get_react_torque() override { return GetReactionOnBody(); }

    // Get constraint violations
    ChMatrixNM<double, 2, 1> GetC() const;

    //
    // STATE FUNCTIONS
    //

    virtual void IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L);
    virtual void IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L);
    virtual void IntLoadResidual_CqL(const unsigned int off_L,
                                     ChVectorDynamic<>& R,
                                     const ChVectorDynamic<>& L,
                                     const double c);
    virtual void IntLoadConstraint_C(const unsigned int off,
                                     ChVectorDynamic<>& Qc,
                                     const double c,
                                     bool do_clamp,
                                     double recovery_clamp);
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

    virtual void InjectConstraints(ChSystemDescriptor& mdescriptor);
    virtual void ConstraintsBiReset();
    virtual void ConstraintsBiLoad_C(double factor = 1., double recovery_clamp = 0.1, bool do_clamp = false);
    virtual void ConstraintsBiLoad_Ct(double factor = 1.);
    virtual void ConstraintsLoadJacobians();
    virtual void ConstraintsFetch_react(double factor = 1.);

    // Other functions

    virtual ChCoordsys<> GetLinkAbsoluteCoords();

    /// Use this function after object creation, to initialize it, given
    /// the node and body frame to join.
    /// The attachment position is the actual position of the node (unless
    /// otherwise defines, using the optional 'mattach' parameter).
    /// Note, mnodes and mbody must belong to the same ChSystem.
    virtual int Initialize(std::shared_ptr<ChNodeFEAxyzD> anode,  ///< xyzD node to join (with the direction)
                           std::shared_ptr<ChBodyFrame> mbody,    ///< body (frame) to join
                           ChVector<>* dir = 0  ///< optional: if not null, sets the direction in absolute coordinates
                           );

    /// Get the connected xyz node (point)
    virtual std::shared_ptr<ChNodeFEAxyzD> GetConstrainedNode() { return this->mnode; }

    /// Get the connected body (frame)
    virtual std::shared_ptr<ChBodyFrame> GetConstrainedBodyFrame() { return this->body; }

    /// Get the attachment position, in the reference coordinates of the body.
    ChVector<> GetDirection() { return direction; }
    /// Set the attachment position, in the reference coordinates of the body
    void SetDirectionInBodyCoords(ChVector<> mattach);
    /// Set the attachment position, in the absolute coordinates
    void SetDirectionInAbsoluteCoords(ChVector<> mattach);

    /// Get the reaction torque considered as applied to the FEA node.
    ChVector<> GetReactionOnNode() { return -(react); }

    /// Get the reaction torque considered as applied to ChBody.
    ChVector<> GetReactionOnBody() { return react; }

    //
    // UPDATE FUNCTIONS
    //

    /// Update all auxiliary data of the gear transmission at given time
    virtual void Update(double mytime, bool update_assets = true);

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
