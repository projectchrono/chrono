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
#include "chrono/fea/multiphysics/ChLinkFieldNode.h"
#include "chrono/fea/multiphysics/ChField.h"

namespace chrono {
namespace fea {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkField)

ChLinkField::ChLinkField() : react(0) {
    auto mfun = chrono_types::make_shared<ChFunctionConst>(0);
    this->offset_function = mfun;
}

ChLinkField::ChLinkField(const ChLinkField& other) : ChLinkBase(other) {
    react = other.react;
    offset_function = other.offset_function;
}

int ChLinkField::Initialize(std::shared_ptr<ChNodeFEAbase> node,  ///< node 1, associated to some scalar field state
                            std::shared_ptr<ChFieldBase> field) {
    assert(node);
    assert(field);
    assert(field->IsNodeAdded(node));

    m_node = node;

    m_field = field;

    std::vector<ChVariables*> mvars;
    mvars.push_back(&(field->GetNodeDataPointer(node)->GetVariable()));
    m_constraint1.SetVariables(mvars);

    return true;
}

void ChLinkField::SetOffset(double offset) {
    auto mfun = chrono_types::make_shared<ChFunctionConst>(offset);
    offset_function = mfun;
}

void ChLinkField::SetOffset(std::shared_ptr<ChFunction> offset_f) {
    offset_function = offset_f;
}

void ChLinkField::Update(double time, UpdateFlags update_flags) {
    // Inherit time changes of parent class
    ChPhysicsItem::Update(time, update_flags);

    // update class data
    // ...
}

ChVectorDynamic<> ChLinkField::GetConstraintViolation() const {

    double res = 0;

    if (m_field->IsFirstOrderField()) {
        // for 1st order fields, the constraint is at speed level, so use c_vel as scaling factor
        res = m_field->GetNodeDataPointer(m_node)->StateDt()(0) - this->offset_function->GetVal(this->ChTime);
    } else {
        // for 2nd order fields, the constraint is at position level, so use c as scaling factor
        res = m_field->GetNodeDataPointer(m_node)->State()(0) - this->offset_function->GetVal(this->ChTime);
    }

    ChVectorN<double, 1> C;
    C(0) = res;
    return C;
}

//// STATE BOOKKEEPING FUNCTIONS

void ChLinkField::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    L(off_L + 0) = react;
}

void ChLinkField::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    react = L(off_L + 0);
}

void ChLinkField::IntLoadResidual_CqL(const unsigned int off_L,    // offset in L multipliers
                                      ChVectorDynamic<>& R,        // result: the R residual, R += c*Cq'*L
                                      const ChVectorDynamic<>& L,  // the L vector
                                      const double c               // a scaling factor
) {
    if (!IsActive())
        return;

    m_constraint1.AddJacobianTransposedTimesScalarInto(R, L(off_L + 0) * c);
}

void ChLinkField::IntLoadConstraint_C(const unsigned int off_L,  // offset in Qc residual
                                      ChVectorDynamic<>& Qc,     // result: the Qc residual, Qc += c*C
                                      const double c,            // a scaling factor
                                      const double c_vel,        // the scaling factor if the constraint is at speed level
                                      bool do_clamp,             // apply clamping to c*C?
                                      double recovery_clamp      // value for min/max clamping of c*C
) {
    if (!IsActive())
        return;

    double cres = 0;

    if (m_field->IsFirstOrderField()) {
        // for 1st order fields, the constraint is at speed level, so use c_vel as scaling factor
        double res = m_field->GetNodeDataPointer(m_node)->StateDt()(0) - this->offset_function->GetVal(this->ChTime);
        cres = res * c_vel;
    } else {
        // for 2nd order fields, the constraint is at position level, so use c as scaling factor
        double res = m_field->GetNodeDataPointer(m_node)->State()(0) - this->offset_function->GetVal(this->ChTime);
        cres = res * c;
    }

    //if (do_clamp) {
    //    cres = std::min(std::max(cres, -recovery_clamp), recovery_clamp);
    //}
    Qc(off_L + 0) += cres;
}

void ChLinkField::IntLoadConstraint_Ct(const unsigned int off_L, 
    ChVectorDynamic<>& Qc, 
    const double c, 
    const double c_vel) {
    
    double mCt = -offset_function->GetDer(this->GetChTime());

    if (m_field->IsFirstOrderField()) {
        // for 1st order fields, the constraint is at speed level, so use c_vel as scaling factor
        Qc(off_L) += c_vel * mCt;
    } else {
        // for 2nd order fields, the constraint is at position level, so use c as scaling factor
        Qc(off_L) += c * mCt;
    }

}

void ChLinkField::IntToDescriptor(const unsigned int off_v,
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

void ChLinkField::IntFromDescriptor(const unsigned int off_v, ChStateDelta& v, const unsigned int off_L, ChVectorDynamic<>& L) {
    if (!IsActive())
        return;

    L(off_L + 0) = m_constraint1.GetLagrangeMultiplier();
}

// SOLVER INTERFACES

void ChLinkField::InjectConstraints(ChSystemDescriptor& descriptor) {
    if (!IsActive())
        return;

    descriptor.InsertConstraint(&m_constraint1);
}

void ChLinkField::ConstraintsBiReset() {
    m_constraint1.SetRightHandSide(0.);
}

// OBSOLETE***
void ChLinkField::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    if (!IsActive())
        return;
    double res = this->GetConstraintViolation()(0);
    m_constraint1.SetRightHandSide(m_constraint1.GetRightHandSide() + factor * res);
}

// OBSOLETE***
void ChLinkField::ConstraintsBiLoad_Ct(double factor) {
    if (!IsActive())
        return;
    double mCt = -offset_function->GetDer(this->GetChTime());
    m_constraint1.SetRightHandSide(m_constraint1.GetRightHandSide() + factor * mCt);
}

void ChLinkField::LoadConstraintJacobians() {
    // compute jacobians

    m_constraint1.Get_Cq_N(0)(0) = 1.0;
}

void ChLinkField::ConstraintsFetch_react(double factor) {
    // From constraints to react vector:
    react = m_constraint1.GetLagrangeMultiplier() * factor;
}

// FILE I/O

void ChLinkField::ArchiveOut(ChArchiveOut& archive_out) {
    //// TODO
}

void ChLinkField::ArchiveIn(ChArchiveIn& archive_in) {
    //// TODO
}

////////////////////////////////////////////////////////////////////////////////////////////////////

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

int ChLinkFieldField::Initialize(std::shared_ptr<ChNodeFEAbase> node1,  ///< node 1, associated to some scalar field state
                                 std::shared_ptr<ChFieldBase> field1,   ///< scalar field with the 1st field state to constrain (must contain node 1).
                                 std::shared_ptr<ChNodeFEAbase> node2,  ///< node 2, associated to some scalar field state
                                 std::shared_ptr<ChFieldBase> field2    ///< scalar field with the 2nd field state to constrain (must contain node 2).
) {
    assert(node1 && node2);
    assert(field1 && field2);
    assert(field1->IsNodeAdded(node1));
    assert(field2->IsNodeAdded(node2));

    m_node1 = node1;
    m_node2 = node2;
    m_field_1 = field1;
    m_field_2 = field2;

    m_constraint1.SetVariables(&(field1->GetNodeDataPointer(node1)->GetVariable()), &(field2->GetNodeDataPointer(node2)->GetVariable()));

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
    double res = this->offset_function->GetVal(this->ChTime);

    if (this->m_field_1->IsFirstOrderField() && this->m_field_2->IsFirstOrderField()) {
        res += m_field_1->GetNodeDataPointer(m_node1)->StateDt()(0) - m_field_2->GetNodeDataPointer(m_node2)->StateDt()(0);
    } else {
        res += m_field_1->GetNodeDataPointer(m_node1)->State()(0) - m_field_2->GetNodeDataPointer(m_node2)->State()(0);
    }

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
                                           const double c_vel,        // the scaling factor if the constraint is at speed level
                                           bool do_clamp,             // apply clamping to c*C?
                                           double recovery_clamp      // value for min/max clamping of c*C
) {
    if (!IsActive())
        return;

    double res = - this->offset_function->GetVal(this->ChTime);
    double cres = 0;

    if (this->m_field_1->IsFirstOrderField() && this->m_field_2->IsFirstOrderField()) {
        res += m_field_1->GetNodeDataPointer(m_node1)->StateDt()(0) - m_field_2->GetNodeDataPointer(m_node2)->StateDt()(0);
        cres = res * c_vel;
    } else {
        res += m_field_1->GetNodeDataPointer(m_node1)->State()(0) - m_field_2->GetNodeDataPointer(m_node2)->State()(0);
        cres = res * c;
    }

    if (do_clamp) {
        cres = std::min(std::max(cres, -recovery_clamp), recovery_clamp);
    }
    Qc(off_L + 0) += cres;
}

void ChLinkFieldField::IntLoadConstraint_Ct(const unsigned int off_L, 
    ChVectorDynamic<>& Qc, 
    const double c, 
    const double c_vel) {

    double mCt = -offset_function->GetDer(this->GetChTime());

    if (this->m_field_1->IsFirstOrderField() && this->m_field_2->IsFirstOrderField()) {
        // for 1st order fields, the constraint is at speed level, so use c_vel as scaling factor
        Qc(off_L) += c_vel * mCt;
    } else {
        // for 2nd order fields, the constraint is at position level, so use c as scaling factor
        Qc(off_L) += c * mCt;
    }

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

void ChLinkFieldField::IntFromDescriptor(const unsigned int off_v, ChStateDelta& v, const unsigned int off_L, ChVectorDynamic<>& L) {
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

// OBSOLETE***
void ChLinkFieldField::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    if (!IsActive())
        return;
    double res = this->GetConstraintViolation()[0];
    m_constraint1.SetRightHandSide(m_constraint1.GetRightHandSide() + factor * res);
}

// OBSOLETE***
void ChLinkFieldField::ConstraintsBiLoad_Ct(double factor) {
    if (!IsActive())
        return;
    double mCt = -offset_function->GetDer(this->GetChTime());
    m_constraint1.SetRightHandSide(m_constraint1.GetRightHandSide() + factor * mCt);
}

void ChLinkFieldField::LoadConstraintJacobians() {
    // compute jacobians

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
