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

#include <cstdlib>
#include <algorithm>

#include "chrono/collision/ChCModelBullet.h"
#include "chrono/core/ChTransform.h"
#include "chrono/physics/ChConveyor.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

using namespace collision;
using namespace geometry;

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChConveyor)

ChConveyor::ChConveyor(double xlength, double ythick, double zwidth) : conveyor_speed(1) {
    conveyor_truss = new ChBody;
    conveyor_plate = new ChBody;

    // conveyor_plate->SetMaterialSurface(GetMaterialSurface());

    conveyor_plate->GetCollisionModel()->ClearModel();
    conveyor_plate->GetCollisionModel()->AddBox(xlength * 0.5, ythick * 0.5, zwidth * 0.5);
    conveyor_plate->GetCollisionModel()->BuildModel();
    conveyor_plate->SetCollide(true);

    internal_link = new ChLinkLockLock;
    internal_link->SetMotion_X(std::make_shared<ChFunction_Ramp>());

    std::shared_ptr<ChMarker> mmark1(new ChMarker);
    std::shared_ptr<ChMarker> mmark2(new ChMarker);
    conveyor_truss->AddMarker(mmark1);
    conveyor_plate->AddMarker(mmark2);

    internal_link->ReferenceMarkers(mmark1.get(), mmark2.get());
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
                                 const double T             // time
                                 ) {
    conveyor_truss->IntStateScatter(off_x, x, off_v, v, T);
    conveyor_plate->IntStateScatter(off_x + 7, x, off_v + 6, v, T);
    this->Update(T);
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

void ChConveyor::IntToDescriptor(const unsigned int off_v,  // offset in v, R
                                 const ChStateDelta& v,
                                 const ChVectorDynamic<>& R,
                                 const unsigned int off_L,  // offset in L, Qc
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

void ChConveyor::InjectVariables(ChSystemDescriptor& mdescriptor) {
    conveyor_truss->InjectVariables(mdescriptor);
    conveyor_plate->InjectVariables(mdescriptor);
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

void ChConveyor::InjectConstraints(ChSystemDescriptor& mdescriptor) {
    internal_link->InjectConstraints(mdescriptor);
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

void ChConveyor::ConstraintsLoadJacobians() {
    internal_link->ConstraintsLoadJacobians();
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

void ChConveyor::Update(double mytime, bool update_assets) {
    // inherit parent class function
    ChPhysicsItem::Update(mytime, update_assets);

    conveyor_truss->Update(mytime, update_assets);

    if (conveyor_truss->GetBodyFixed()) {
        double largemass = 100000;
        conveyor_plate->SetMass(largemass);
        conveyor_plate->SetInertiaXX(ChVector<>(largemass, largemass, largemass));
        conveyor_plate->SetInertiaXY(ChVector<>(0, 0, 0));
    } else {
        conveyor_plate->SetMass(conveyor_truss->GetMass());
        conveyor_plate->SetInertiaXX(conveyor_truss->GetInertiaXX());
        conveyor_plate->SetInertiaXY(conveyor_truss->GetInertiaXY());
    }

    // keep the plate always at the same position of the main reference
    conveyor_plate->SetCoord(conveyor_truss->GetCoord());
    conveyor_plate->SetCoord_dt(conveyor_truss->GetCoord_dt());
    // keep the plate always at the same speed of the main reference, plus the conveyor speed on X local axis
    conveyor_plate->SetPos_dt(conveyor_truss->GetPos_dt() + (ChVector<>(conveyor_speed, 0, 0) >> (*conveyor_truss)));

    conveyor_plate->Update(mytime, update_assets);

    std::static_pointer_cast<ChFunction_Ramp>(internal_link->GetMotion_X())->Set_ang(-conveyor_speed);
    // always zero pos. offset (trick):
    std::static_pointer_cast<ChFunction_Ramp>(internal_link->GetMotion_X())->Set_y0(+conveyor_speed * GetChTime());

    internal_link->Update(mytime, update_assets);
}

void ChConveyor::SyncCollisionModels() {
    // inherit parent class
    ChPhysicsItem::SyncCollisionModels();

    conveyor_truss->SyncCollisionModels();
    conveyor_plate->SyncCollisionModels();
}

void ChConveyor::AddCollisionModelsToSystem() {
    // inherit parent class
    ChPhysicsItem::AddCollisionModelsToSystem();
    // conveyor_truss->AddCollisionModelsToSystem();
    // conveyor_plate->AddCollisionModelsToSystem();
}

void ChConveyor::RemoveCollisionModelsFromSystem() {
    // inherit parent class
    ChPhysicsItem::RemoveCollisionModelsFromSystem();
    // conveyor_plate->RemoveCollisionModelsFromSystem();
    // conveyor_truss->RemoveCollisionModelsFromSystem();
}

// FILE I/O

void ChConveyor::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChConveyor>();

    // serialize parent class
    ChPhysicsItem::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(conveyor_speed);
    marchive << CHNVP(conveyor_truss);
    marchive << CHNVP(conveyor_plate);
    marchive << CHNVP(internal_link);
}

/// Method to allow de serialization of transient data from archives.
void ChConveyor::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChConveyor>();

    // deserialize parent class
    ChPhysicsItem::ArchiveIN(marchive);

    // stream in all member data:
    marchive >> CHNVP(conveyor_speed);
    marchive >> CHNVP(conveyor_truss);
    marchive >> CHNVP(conveyor_plate);
    marchive >> CHNVP(internal_link);
}

}  // end namespace chrono
