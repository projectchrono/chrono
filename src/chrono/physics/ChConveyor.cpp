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

#include <cstdlib>
#include <algorithm>

#include "chrono/physics/ChConveyor.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/collision/ChCollisionSystem.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChConveyor)

ChConveyor::ChConveyor(double xlength, double ythick, double zwidth) : conveyor_speed(1) {
    conveyor_truss = new ChBody;
    conveyor_plate = new ChBody;

    conveyor_mat = chrono_types::make_shared<ChContactMaterialNSC>();

    auto cshape = chrono_types::make_shared<ChCollisionShapeBox>(conveyor_mat, xlength, ythick, zwidth);
    conveyor_plate->AddCollisionShape(cshape);
    conveyor_plate->EnableCollision(true);

    internal_link = new ChLinkLockLock;
    internal_link->SetMotionX(chrono_types::make_shared<ChFunctionRamp>());

    std::shared_ptr<ChMarker> mmark1(new ChMarker);
    std::shared_ptr<ChMarker> mmark2(new ChMarker);
    conveyor_truss->AddMarker(mmark1);
    conveyor_plate->AddMarker(mmark2);

    internal_link->SetupMarkers(mmark1.get(), mmark2.get());
}

ChConveyor::ChConveyor(const ChConveyor& other) : ChPhysicsItem(other) {
    conveyor_speed = other.conveyor_speed;
    internal_link = other.internal_link->Clone();
    conveyor_plate = other.conveyor_plate->Clone();
    conveyor_truss = other.conveyor_truss->Clone();

    //// RADU: more to do here
}

ChConveyor::~ChConveyor() {
    if (internal_link)
        delete internal_link;
    if (conveyor_plate)
        delete conveyor_plate;
    if (conveyor_truss)
        delete conveyor_truss;
}

//// STATE BOOKKEEPING FUNCTIONS

void ChConveyor::IntStateGather(const unsigned int off_x,  // offset in x state vector
                                ChState& x,                // state vector, position part
                                const unsigned int off_v,  // offset in v state vector
                                ChStateDelta& v,           // state vector, speed part
                                double& T                  // time
) {
    conveyor_truss->IntStateGather(off_x, x, off_v, v, T);
    conveyor_plate->IntStateGather(off_x + 7, x, off_v + 6, v, T);
}

void ChConveyor::IntStateScatter(const unsigned int off_x,  // offset in x state vector
                                 const ChState& x,          // state vector, position part
                                 const unsigned int off_v,  // offset in v state vector
                                 const ChStateDelta& v,     // state vector, speed part
                                 const double T,            // time
                                 bool full_update           // perform complete update
) {
    conveyor_truss->IntStateScatter(off_x, x, off_v, v, T, full_update);
    conveyor_plate->IntStateScatter(off_x + 7, x, off_v + 6, v, T, full_update);
    this->Update(T, full_update);
}

void ChConveyor::IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    conveyor_truss->IntStateGatherAcceleration(off_a, a);
    conveyor_plate->IntStateGatherAcceleration(off_a + 6, a);
}

void ChConveyor::IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    conveyor_truss->IntStateScatterAcceleration(off_a, a);
    conveyor_plate->IntStateScatterAcceleration(off_a + 6, a);
}

void ChConveyor::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    internal_link->IntStateGatherReactions(off_L, L);
}

void ChConveyor::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    internal_link->IntStateScatterReactions(off_L, L);
}

void ChConveyor::IntStateIncrement(const unsigned int off_x,  // offset in x state vector
                                   ChState& x_new,            // state vector, position part, incremented result
                                   const ChState& x,          // state vector, initial position part
                                   const unsigned int off_v,  // offset in v state vector
                                   const ChStateDelta& Dv     // state vector, increment
) {
    conveyor_truss->IntStateIncrement(off_x, x_new, x, off_v, Dv);
    conveyor_plate->IntStateIncrement(off_x + 7, x_new, x, off_v + 6, Dv);
}

void ChConveyor::IntStateGetIncrement(const unsigned int off_x,  // offset in x state vector
                                      const ChState& x_new,      // state vector, position part, incremented result
                                      const ChState& x,          // state vector, initial position part
                                      const unsigned int off_v,  // offset in v state vector
                                      ChStateDelta& Dv           // state vector, increment
) {
    conveyor_truss->IntStateGetIncrement(off_x, x_new, x, off_v, Dv);
    conveyor_plate->IntStateGetIncrement(off_x + 7, x_new, x, off_v + 6, Dv);
}

void ChConveyor::IntLoadResidual_F(const unsigned int off,  // offset in R residual
                                   ChVectorDynamic<>& R,    // result: the R residual, R += c*F
                                   const double c           // a scaling factor
) {
    conveyor_truss->IntLoadResidual_F(off, R, c);
    conveyor_plate->IntLoadResidual_F(off + 6, R, c);
}

void ChConveyor::IntLoadResidual_Mv(const unsigned int off,      // offset in R residual
                                    ChVectorDynamic<>& R,        // result: the R residual, R += c*M*v
                                    const ChVectorDynamic<>& w,  // the w vector
                                    const double c               // a scaling factor
) {
    conveyor_truss->IntLoadResidual_Mv(off, R, w, c);
    conveyor_plate->IntLoadResidual_Mv(off + 6, R, w, c);
}

void ChConveyor::IntLoadLumpedMass_Md(const unsigned int off, ChVectorDynamic<>& Md, double& err, const double c) {
    conveyor_truss->IntLoadLumpedMass_Md(off, Md, err, c);
    conveyor_plate->IntLoadLumpedMass_Md(off + 6, Md, err, c);
}

void ChConveyor::IntToDescriptor(const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const ChVectorDynamic<>& R,
                                 const unsigned int off_L,
                                 const ChVectorDynamic<>& L,
                                 const ChVectorDynamic<>& Qc) {
    conveyor_truss->IntToDescriptor(off_v, v, R, off_L, L, Qc);
    conveyor_plate->IntToDescriptor(off_v + 6, v, R, off_L, L, Qc);
    internal_link->IntToDescriptor(off_v, v, R, off_L, L, Qc);
}

void ChConveyor::IntFromDescriptor(const unsigned int off_v,  // offset in v
                                   ChStateDelta& v,
                                   const unsigned int off_L,  // offset in L
                                   ChVectorDynamic<>& L) {
    conveyor_truss->IntFromDescriptor(off_v, v, off_L, L);
    conveyor_plate->IntFromDescriptor(off_v + 6, v, off_L, L);
    internal_link->IntFromDescriptor(off_v, v, off_L, L);
}

void ChConveyor::IntLoadResidual_CqL(const unsigned int off_L,
                                     ChVectorDynamic<>& R,
                                     const ChVectorDynamic<>& L,
                                     const double c) {
    internal_link->IntLoadResidual_CqL(off_L, R, L, c);
}

void ChConveyor::IntLoadConstraint_C(const unsigned int off,
                                     ChVectorDynamic<>& Qc,
                                     const double c,
                                     bool do_clamp,
                                     double recovery_clamp) {
    internal_link->IntLoadConstraint_C(off, Qc, c, do_clamp, recovery_clamp);
}

void ChConveyor::IntLoadConstraint_Ct(const unsigned int off, ChVectorDynamic<>& Qc, const double c) {
    internal_link->IntLoadConstraint_Ct(off, Qc, c);
}

// SOLVER INTERFACE

void ChConveyor::InjectVariables(ChSystemDescriptor& descriptor) {
    conveyor_truss->InjectVariables(descriptor);
    conveyor_plate->InjectVariables(descriptor);
}

void ChConveyor::VariablesFbReset() {
    conveyor_truss->VariablesFbReset();
    conveyor_plate->VariablesFbReset();
}

void ChConveyor::VariablesFbLoadForces(double factor) {
    conveyor_truss->VariablesFbLoadForces(factor);
    conveyor_plate->VariablesFbLoadForces(factor);
}

void ChConveyor::VariablesFbIncrementMq() {
    conveyor_truss->VariablesFbIncrementMq();
    conveyor_plate->VariablesFbIncrementMq();
}

void ChConveyor::VariablesQbLoadSpeed() {
    conveyor_truss->VariablesFbIncrementMq();
    conveyor_plate->VariablesQbLoadSpeed();
}

void ChConveyor::VariablesQbSetSpeed(double step) {
    conveyor_truss->VariablesQbSetSpeed(step);
    conveyor_plate->VariablesQbSetSpeed(step);
}

void ChConveyor::VariablesQbIncrementPosition(double dt_step) {
    conveyor_truss->VariablesQbIncrementPosition(dt_step);
    conveyor_plate->VariablesQbIncrementPosition(dt_step);
}

void ChConveyor::InjectConstraints(ChSystemDescriptor& descriptor) {
    internal_link->InjectConstraints(descriptor);
}

void ChConveyor::ConstraintsBiReset() {
    internal_link->ConstraintsBiReset();
}

void ChConveyor::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    internal_link->ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);
}

void ChConveyor::ConstraintsBiLoad_Ct(double factor) {
    internal_link->ConstraintsBiLoad_Ct(factor);
}

void ChConveyor::ConstraintsBiLoad_Qc(double factor) {
    internal_link->ConstraintsBiLoad_Qc(factor);
}

void ChConveyor::LoadConstraintJacobians() {
    internal_link->LoadConstraintJacobians();
}

void ChConveyor::ConstraintsFetch_react(double factor) {
    internal_link->ConstraintsFetch_react(factor);
}

void ChConveyor::SetSystem(ChSystem* m_system) {
    system = m_system;
    conveyor_truss->SetSystem(m_system);
    conveyor_plate->SetSystem(m_system);
    internal_link->SetSystem(m_system);
}

void ChConveyor::Update(double time, bool update_assets) {
    // inherit parent class function
    ChPhysicsItem::Update(time, update_assets);

    conveyor_truss->Update(time, update_assets);

    if (conveyor_truss->IsFixed()) {
        double largemass = 100000;
        conveyor_plate->SetMass(largemass);
        conveyor_plate->SetInertiaXX(ChVector3d(largemass, largemass, largemass));
        conveyor_plate->SetInertiaXY(ChVector3d(0, 0, 0));
    } else {
        conveyor_plate->SetMass(conveyor_truss->GetMass());
        conveyor_plate->SetInertiaXX(conveyor_truss->GetInertiaXX());
        conveyor_plate->SetInertiaXY(conveyor_truss->GetInertiaXY());
    }

    // keep the plate always at the same position of the main reference
    conveyor_plate->SetCoordsys(conveyor_truss->GetCoordsys());
    conveyor_plate->SetCoordsysDt(conveyor_truss->GetCoordsysDt());
    // keep the plate always at the same speed of the main reference, plus the conveyor speed on X local axis
    conveyor_plate->SetPosDt(conveyor_truss->GetPosDt() + (ChVector3d(conveyor_speed, 0, 0) >> (*conveyor_truss)));

    conveyor_plate->Update(time, update_assets);

    std::static_pointer_cast<ChFunctionRamp>(internal_link->GetMotionX())->SetAngularCoeff(-conveyor_speed);
    // always zero pos. offset (trick):
    std::static_pointer_cast<ChFunctionRamp>(internal_link->GetMotionX())->SetStartVal(+conveyor_speed * GetChTime());

    internal_link->Update(time, update_assets);
}

void ChConveyor::AddCollisionModelsToSystem(ChCollisionSystem* coll_sys) const {
    if (conveyor_truss->GetCollisionModel())
        coll_sys->Add(conveyor_truss->GetCollisionModel());

    if (conveyor_plate->GetCollisionModel())
        coll_sys->Add(conveyor_plate->GetCollisionModel());
}

void ChConveyor::RemoveCollisionModelsFromSystem(ChCollisionSystem* coll_sys) const {
    if (conveyor_truss->GetCollisionModel())
        coll_sys->Remove(conveyor_truss->GetCollisionModel());

    if (conveyor_plate->GetCollisionModel())
        coll_sys->Remove(conveyor_plate->GetCollisionModel());
}

void ChConveyor::SyncCollisionModels() {
    // inherit parent class
    ChPhysicsItem::SyncCollisionModels();

    conveyor_truss->SyncCollisionModels();
    conveyor_plate->SyncCollisionModels();
}

// FILE I/O

void ChConveyor::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChConveyor>();

    // serialize parent class
    ChPhysicsItem::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(conveyor_speed);
    archive_out << CHNVP(conveyor_truss);
    archive_out << CHNVP(conveyor_plate);
    archive_out << CHNVP(internal_link);
}

/// Method to allow de serialization of transient data from archives.
void ChConveyor::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChConveyor>();

    // deserialize parent class
    ChPhysicsItem::ArchiveIn(archive_in);

    // stream in all member data:
    archive_in >> CHNVP(conveyor_speed);
    archive_in >> CHNVP(conveyor_truss);
    archive_in >> CHNVP(conveyor_plate);
    archive_in >> CHNVP(internal_link);
}

}  // end namespace chrono
