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

#ifndef CHLINKFIELD_H
#define CHLINKFIELD_H

#include "chrono/physics/ChLinkBase.h"
#include "chrono/solver/ChConstraintTwoGeneric.h"
#include "chrono/fea/ChNodeFEAxyz.h"

namespace chrono {

class ChIndexedNodes;  // forward ref

namespace fea {

class ChFieldScalar;  // forward ref

/// @addtogroup fea_constraints
/// @{


/// Constraint between two field states, (ex. temperature-temperature, or voltage-voltage, etc.)
/// to enformce that the two states are equal.
/// This supports an offset, so that you can impose a temperature difference between two nodes, or
/// a voltage difference between two nodes.
/// The offset can be either a constant value, or a function of time.

class ChApi ChLinkFieldField : public ChLinkBase {
public:
    ChLinkFieldField();
    ChLinkFieldField(const ChLinkFieldField& other);
    ~ChLinkFieldField() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkFieldField* Clone() const override { return new ChLinkFieldField(*this); }

    /// Get the number of scalar variables affected by constraints in this link.
    virtual unsigned int GetNumAffectedCoords() override { return 1 + 1; }

    /// Number of scalar constraints.
    virtual unsigned int GetNumConstraintsBilateral() override { return 1; }

    // Get constraint violations.
    virtual ChVectorDynamic<> GetConstraintViolation() const override;

    /// Initialize the constraint, given the two nodes, and scalar 
    /// fields to whom the two nodes belong
    virtual int Initialize(std::shared_ptr<ChNodeFEAxyz> node1,  ///< node 1, associated to some scalar field state
                           std::shared_ptr<ChFieldScalar> field1, ///< scalar field with the 1st field state to constrain (must contain node 1).
                           std::shared_ptr<ChNodeFEAxyz> node2,   ///< node 2, associated to some scalar field state
                           std::shared_ptr<ChFieldScalar> field2 ///< scalar field with the 2nd field state to constrain (must contain node 2).
    );

    /// Set the offset between the two field states. The constraint is assumed satisfied when  
    ///  state1 - state2 = offset;
    /// Default is 0 offset, for enforcing  state2 = state 1.
    virtual void SetOffset(double offset);

    /// Set the offset between the two field states, where the offset is a function of time. 
    /// The constraint is assumed satisfied when  
    ///  state1 - state2 = offset(t);
    virtual void SetOffset(std::shared_ptr<ChFunction> offset_f);


    /// Get the 1st connected xyz node.
    std::shared_ptr<fea::ChNodeFEAbase> GetNode1() const { return m_node1; }

    /// Get the 2nd connected xyz node.
    std::shared_ptr<fea::ChNodeFEAbase> GetNode2() const { return m_node2; }

    /// Get the reaction force considered as applied to the 1st node.
    ChVector3d GetReactionOnNode1() const { return +react; }

    /// Get the reaction force considered as applied to the 2nd node.
    ChVector3d GetReactionOnNode2() const { return -react; }

    /// Update all auxiliary data of the gear transmission at given time.
    virtual void Update(double time, UpdateFlags update_flags) override;

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
    double react;

    std::shared_ptr<ChFunction> offset_function;

    // used as an interface to the solver.
    ChConstraintTwoGeneric m_constraint1;

    std::shared_ptr<fea::ChNodeFEAbase> m_node1;
    std::shared_ptr<fea::ChNodeFEAbase> m_node2;
    std::shared_ptr<fea::ChFieldScalar> m_field_1;
    std::shared_ptr<fea::ChFieldScalar> m_field_2;


    // Provide dummy implementation for the following 4 members tht are meaningless anyway for this kind of link:

    /// Get the link frame 1, on the 1st node, expressed in the absolute frame.
    virtual ChFramed GetFrame1Abs() const override { return ChFramed(); }
    /// Get the link frame 2, on the 2nd node, expressed in the absolute frame.
    virtual ChFramed GetFrame2Abs() const override { return ChFramed(); }
    /// Get reaction force and torque on 1st node, expressed on link frame 1.
    virtual ChWrenchd GetReaction1() const override { return { ChVector3d(react,0,0), VNULL}; }
    /// Get reaction force and torque on 2nd node, expressed on link frame 2.
    virtual ChWrenchd GetReaction2() const override { return { ChVector3d(-react,0,0), VNULL}; }
    
};




/// @} fea_constraints

}  // end namespace fea
}  // end namespace chrono

#endif
