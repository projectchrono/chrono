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

#include "chrono/physics/ChIndexedNodes.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/fea/ChLinkFieldNode.h"
#include "chrono/fea/ChField.h"

namespace chrono {
namespace fea {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkFieldField)

ChLinkFieldField::ChLinkFieldField() : react(0) {
    auto mfun = chrono_types::make_shared<ChFunctionConst>(0);
    this->offset_function = mfun;
}

ChLinkFieldField::ChLinkFieldField(const ChLinkFieldField& other) : ChLinkBase(other) {
    react = other.react;
    offset_function = other.offset_function;
}

int ChLinkFieldField::Initialize        (std::shared_ptr<ChNodeFEAxyz> node1,   ///< node 1, associated to some scalar field state
                                         std::shared_ptr<ChFieldScalar> field1, ///< scalar field with the 1st field state to constrain (must contain node 1).
                                         std::shared_ptr<ChNodeFEAxyz> node2,   ///< node 2, associated to some scalar field state
                                         std::shared_ptr<ChFieldScalar> field2 ///< scalar field with the 2nd field state to constrain (must contain node 2).
) {
    assert(node1 && node2);
    assert(field1 && field2);
    assert(field1->GetNodeDataMap().find(node1) != field1->GetNodeDataMap().end());
    assert(field2->GetNodeDataMap().find(node2) != field2->GetNodeDataMap().end());

    m_node1 = node1;
    m_node2 = node2;
    m_field_1 = field1;
    m_field_2 = field2;

    m_constraint1.SetVariables(&(field1->NodeData(node1).GetVariable()), &(field2->NodeData(node2).GetVariable()));

    return true;
}


void ChLinkFieldField::SetOffset(double offset) {
    auto mfun = chrono_types::make_shared<ChFunctionConst>(offset);
    offset_function = mfun;
}

void ChLinkFieldField::SetOffset(std::shared_ptr<ChFunction> offset_f) {
    offset_function = offset_f;
}

void ChLinkFieldField::Update(double time, UpdateFlags update_flags) {
    // Inherit time changes of parent class
    ChPhysicsItem::Update(time, update_flags);

    // update class data
    // ...
}

ChVectorDynamic<> ChLinkFieldField::GetConstraintViolation() const {
    double res = m_field_1->NodeData(m_node1).State()(0) - m_field_2->NodeData(m_node2).State()(0) - this->offset_function->GetVal(this->ChTime);
    ChVectorN<double, 1> C;
    C(0) = res;
    return C;
}

//// STATE BOOKKEEPING FUNCTIONS

void ChLinkFieldField::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    L(off_L + 0) = react;
}

void ChLinkFieldField::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    react = L(off_L + 0);
}

void ChLinkFieldField::IntLoadResidual_CqL(const unsigned int off_L,    // offset in L multipliers
                                         ChVectorDynamic<>& R,        // result: the R residual, R += c*Cq'*L
                                         const ChVectorDynamic<>& L,  // the L vector
                                         const double c               // a scaling factor
) {
    if (!IsActive())
        return;

    m_constraint1.AddJacobianTransposedTimesScalarInto(R, L(off_L + 0) * c);
}

void ChLinkFieldField::IntLoadConstraint_C(const unsigned int off_L,  // offset in Qc residual
                                         ChVectorDynamic<>& Qc,     // result: the Qc residual, Qc += c*C
                                         const double c,            // a scaling factor
                                         bool do_clamp,             // apply clamping to c*C?
                                         double recovery_clamp      // value for min/max clamping of c*C
) {
    if (!IsActive())
        return;

    double res = m_field_1->NodeData(m_node1).State()(0) - m_field_2->NodeData(m_node2).State()(0) - this->offset_function->GetVal(this->ChTime);
    double cres = res * c;

    if (do_clamp) {
        cres = std::min(std::max(cres, -recovery_clamp), recovery_clamp);
    }
    Qc(off_L + 0) += cres;
}

void ChLinkFieldField::IntToDescriptor(const unsigned int off_v,
                                     const ChStateDelta& v,
                                     const ChVectorDynamic<>& R,
                                     const unsigned int off_L,
                                     const ChVectorDynamic<>& L,
                                     const ChVectorDynamic<>& Qc) {
    if (!IsActive())
        return;

    m_constraint1.SetLagrangeMultiplier(L(off_L + 0));

    m_constraint1.SetRightHandSide(Qc(off_L + 0));
}

void ChLinkFieldField::IntFromDescriptor(const unsigned int off_v,
                                       ChStateDelta& v,
                                       const unsigned int off_L,
                                       ChVectorDynamic<>& L) {
    if (!IsActive())
        return;

    L(off_L + 0) = m_constraint1.GetLagrangeMultiplier();
}

// SOLVER INTERFACES

void ChLinkFieldField::InjectConstraints(ChSystemDescriptor& descriptor) {
    if (!IsActive())
        return;

    descriptor.InsertConstraint(&m_constraint1);
}

void ChLinkFieldField::ConstraintsBiReset() {
    m_constraint1.SetRightHandSide(0.);
}

void ChLinkFieldField::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    double res = m_field_1->NodeData(m_node1).State()(0) - m_field_2->NodeData(m_node2).State()(0) - this->offset_function->GetVal(this->ChTime);

    m_constraint1.SetRightHandSide(m_constraint1.GetRightHandSide() + factor * res);
}

void ChLinkFieldField::ConstraintsBiLoad_Ct(double factor) {
    // if (!IsActive())
    //	return;

    // nothing
}

void ChLinkFieldField::LoadConstraintJacobians() {
    // compute jacobians
    ChMatrix33<> Jxa(1.0);

    ChMatrix33<> Jxb(-1.0);

    m_constraint1.Get_Cq_a()(0) = 1.0;

    m_constraint1.Get_Cq_b()(0) = -1.0;
}

void ChLinkFieldField::ConstraintsFetch_react(double factor) {
    // From constraints to react vector:
    react = m_constraint1.GetLagrangeMultiplier() * factor;
}

// FILE I/O

void ChLinkFieldField::ArchiveOut(ChArchiveOut& archive_out) {
    //// TODO
}

void ChLinkFieldField::ArchiveIn(ChArchiveIn& archive_in) {
    //// TODO
}

}  // end namespace fea
}  // end namespace chrono
