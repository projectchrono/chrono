//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChConveyor.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include <stdlib.h>
#include <algorithm>

#include "core/ChTransform.h"
#include "physics/ChConveyor.h"
#include "physics/ChSystem.h"

#include "collision/ChCModelBullet.h"

namespace chrono {

using namespace collision;
using namespace geometry;

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChConveyor> a_registration_ChConveyor;

//////////////////////////////////////
//////////////////////////////////////

/// CLASS FOR CONVEYOR

ChConveyor::ChConveyor(double xlength, double ythick, double zwidth) {
    conveyor_speed = 1.0;

    conveyor_truss = new ChBody;

    conveyor_plate = new ChBody;

    //conveyor_plate->SetMaterialSurface(this->GetMaterialSurface());

    conveyor_plate->GetCollisionModel()->ClearModel();
    conveyor_plate->GetCollisionModel()->AddBox(xlength * 0.5, ythick * 0.5, zwidth * 0.5);
    conveyor_plate->GetCollisionModel()->BuildModel();
    conveyor_plate->SetCollide(true);

    internal_link = new ChLinkLockLock;
    internal_link->SetMotion_X(new ChFunction_Ramp);

    std::shared_ptr<ChMarker> mmark1(new ChMarker);
    std::shared_ptr<ChMarker> mmark2(new ChMarker);
    this->conveyor_truss->AddMarker(mmark1);
    this->conveyor_plate->AddMarker(mmark2);

    internal_link->ReferenceMarkers(mmark1.get(), mmark2.get());
}

ChConveyor::~ChConveyor() {
    if (internal_link)
        delete internal_link;
    if (conveyor_plate)
        delete conveyor_plate;
    if (conveyor_truss)
        delete conveyor_truss;
}

void ChConveyor::Copy(ChConveyor* source) {
    // copy the parent class data...
    ChPhysicsItem::Copy(source);

    this->conveyor_speed = source->conveyor_speed;

    this->internal_link->Copy(source->internal_link);
    this->conveyor_plate->Copy(source->conveyor_plate);
    this->conveyor_truss->Copy(source->conveyor_truss);
}

//// STATE BOOKKEEPING FUNCTIONS

void ChConveyor::IntStateGather(const unsigned int off_x,  ///< offset in x state vector
                                ChState& x,                ///< state vector, position part
                                const unsigned int off_v,  ///< offset in v state vector
                                ChStateDelta& v,           ///< state vector, speed part
                                double& T)                 ///< time
{
    this->conveyor_truss->IntStateGather(off_x    , x, off_v    , v, T);
    this->conveyor_plate->IntStateGather(off_x + 7, x, off_v + 6, v, T);
}

void ChConveyor::IntStateScatter(const unsigned int off_x,  ///< offset in x state vector
                                 const ChState& x,          ///< state vector, position part
                                 const unsigned int off_v,  ///< offset in v state vector
                                 const ChStateDelta& v,     ///< state vector, speed part
                                 const double T)            ///< time
{
    this->conveyor_truss->IntStateScatter(off_x    , x, off_v    , v, T);
    this->conveyor_plate->IntStateScatter(off_x + 7, x, off_v + 6, v, T);
}

void ChConveyor::IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    this->conveyor_truss->IntStateGatherAcceleration(off_a    , a);
    this->conveyor_plate->IntStateGatherAcceleration(off_a + 6, a);
}

void ChConveyor::IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    this->conveyor_truss->IntStateScatterAcceleration(off_a    , a);
    this->conveyor_plate->IntStateScatterAcceleration(off_a + 6, a);
}

void ChConveyor::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    this->internal_link->IntStateGatherReactions(off_L, L);
}

void ChConveyor::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    this->internal_link->IntStateScatterReactions(off_L, L);
}

void ChConveyor::IntStateIncrement(const unsigned int off_x,  ///< offset in x state vector
                                   ChState& x_new,            ///< state vector, position part, incremented result
                                   const ChState& x,          ///< state vector, initial position part
                                   const unsigned int off_v,  ///< offset in v state vector
                                   const ChStateDelta& Dv)    ///< state vector, increment
{
    this->conveyor_truss->IntStateIncrement(off_x    , x_new, x, off_v    , Dv);
    this->conveyor_plate->IntStateIncrement(off_x + 7, x_new, x, off_v + 6, Dv);
}

void ChConveyor::IntLoadResidual_F(const unsigned int off,  ///< offset in R residual
                                   ChVectorDynamic<>& R,    ///< result: the R residual, R += c*F
                                   const double c           ///< a scaling factor
                                   ) {
    this->conveyor_truss->IntLoadResidual_F(off    , R, c);
    this->conveyor_plate->IntLoadResidual_F(off + 6, R, c);
}

void ChConveyor::IntLoadResidual_Mv(const unsigned int off,      ///< offset in R residual
                                    ChVectorDynamic<>& R,        ///< result: the R residual, R += c*M*v
                                    const ChVectorDynamic<>& w,  ///< the w vector
                                    const double c               ///< a scaling factor
                                    ) {
    this->conveyor_truss->IntLoadResidual_Mv(off    , R, w, c);
    this->conveyor_plate->IntLoadResidual_Mv(off + 6, R, w, c);
}

void ChConveyor::IntToLCP(const unsigned int off_v,  ///< offset in v, R
                          const ChStateDelta& v,
                          const ChVectorDynamic<>& R,
                          const unsigned int off_L,  ///< offset in L, Qc
                          const ChVectorDynamic<>& L,
                          const ChVectorDynamic<>& Qc) {
    this->conveyor_truss->IntToLCP(off_v    , v, R, off_L, L, Qc);
    this->conveyor_plate->IntToLCP(off_v + 6, v, R, off_L, L, Qc);
    this->internal_link->IntToLCP(off_v, v, R, off_L, L, Qc);
}

void ChConveyor::IntFromLCP(const unsigned int off_v,  ///< offset in v
                            ChStateDelta& v,
                            const unsigned int off_L,  ///< offset in L
                            ChVectorDynamic<>& L) {
    this->conveyor_truss->IntFromLCP(off_v    , v, off_L, L);
    this->conveyor_plate->IntFromLCP(off_v + 6, v, off_L, L);
    this->internal_link->IntFromLCP(off_v, v, off_L, L);
}

void ChConveyor::IntLoadResidual_CqL(const unsigned int off_L,
                                     ChVectorDynamic<>& R,
                                     const ChVectorDynamic<>& L,
                                     const double c) {
    this->internal_link->IntLoadResidual_CqL(off_L, R, L, c);
}

void ChConveyor::IntLoadConstraint_C(const unsigned int off,
                                     ChVectorDynamic<>& Qc,
                                     const double c,
                                     bool do_clamp,
                                     double recovery_clamp) {
    this->internal_link->IntLoadConstraint_C(off, Qc, c, do_clamp, recovery_clamp);
}

void ChConveyor::IntLoadConstraint_Ct(const unsigned int off, ChVectorDynamic<>& Qc, const double c) {
    this->internal_link->IntLoadConstraint_Ct(off, Qc, c);
}

//// LCP INTERFACE

void ChConveyor::InjectVariables(ChLcpSystemDescriptor& mdescriptor) {
    this->conveyor_truss->InjectVariables(mdescriptor);
    this->conveyor_plate->InjectVariables(mdescriptor);
}

void ChConveyor::VariablesFbReset() {
    this->conveyor_truss->VariablesFbReset();
    this->conveyor_plate->VariablesFbReset();
}

void ChConveyor::VariablesFbLoadForces(double factor) {
    this->conveyor_truss->VariablesFbLoadForces(factor);
    this->conveyor_plate->VariablesFbLoadForces(factor);
}

void ChConveyor::VariablesFbIncrementMq() {
    this->conveyor_truss->VariablesFbIncrementMq();
    this->conveyor_plate->VariablesFbIncrementMq();
}

void ChConveyor::VariablesQbLoadSpeed() {
    this->conveyor_truss->VariablesFbIncrementMq();
    this->conveyor_plate->VariablesQbLoadSpeed();
}

void ChConveyor::VariablesQbSetSpeed(double step) {
    this->conveyor_truss->VariablesQbSetSpeed(step);
    this->conveyor_plate->VariablesQbSetSpeed(step);
}

void ChConveyor::VariablesQbIncrementPosition(double dt_step) {
    this->conveyor_truss->VariablesQbIncrementPosition(dt_step);
    this->conveyor_plate->VariablesQbIncrementPosition(dt_step);
}

void ChConveyor::InjectConstraints(ChLcpSystemDescriptor& mdescriptor) {
    this->internal_link->InjectConstraints(mdescriptor);
}

void ChConveyor::ConstraintsBiReset() {
    this->internal_link->ConstraintsBiReset();
}

void ChConveyor::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    this->internal_link->ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);
}

void ChConveyor::ConstraintsBiLoad_Ct(double factor) {
    this->internal_link->ConstraintsBiLoad_Ct(factor);
}

void ChConveyor::ConstraintsBiLoad_Qc(double factor) {
    this->internal_link->ConstraintsBiLoad_Qc(factor);
}

void ChConveyor::ConstraintsLoadJacobians() {
    this->internal_link->ConstraintsLoadJacobians();
}

void ChConveyor::ConstraintsFetch_react(double factor) {
    this->internal_link->ConstraintsFetch_react(factor);
}

void ChConveyor::ConstraintsLiLoadSuggestedSpeedSolution() {
    this->internal_link->ConstraintsLiLoadSuggestedSpeedSolution();
}

void ChConveyor::ConstraintsLiLoadSuggestedPositionSolution() {
    this->internal_link->ConstraintsLiLoadSuggestedPositionSolution();
}

void ChConveyor::ConstraintsLiFetchSuggestedSpeedSolution() {
    this->internal_link->ConstraintsLiFetchSuggestedSpeedSolution();
}

void ChConveyor::ConstraintsLiFetchSuggestedPositionSolution() {
    this->internal_link->ConstraintsLiFetchSuggestedPositionSolution();
}

void ChConveyor::SetSystem(ChSystem* m_system) {
    this->system = m_system;
    this->conveyor_truss->SetSystem(m_system);
    this->conveyor_plate->SetSystem(m_system);
    this->internal_link->SetSystem(m_system);
}


void ChConveyor::Update(double mytime, bool update_assets) {
    // inherit parent class function
    ChPhysicsItem::Update(mytime, update_assets);

    this->conveyor_truss->Update(mytime, update_assets);

    if (this->conveyor_truss->GetBodyFixed()) {
        double largemass = 100000;
        this->conveyor_plate->SetMass(largemass);
        this->conveyor_plate->SetInertiaXX(ChVector<>(largemass, largemass, largemass));
        this->conveyor_plate->SetInertiaXY(ChVector<>(0, 0, 0));
    } else {
        this->conveyor_plate->SetMass(this->conveyor_truss->GetMass());
        this->conveyor_plate->SetInertiaXX(this->conveyor_truss->GetInertiaXX());
        this->conveyor_plate->SetInertiaXY(this->conveyor_truss->GetInertiaXY());
    }


    // keep the plate always at the same position of the main reference
    this->conveyor_plate->SetCoord(this->conveyor_truss->GetCoord());
    this->conveyor_plate->SetCoord_dt(this->conveyor_truss->GetCoord_dt());
    // keep the plate always at the same speed of the main reference, plus the conveyor speed on X local axis
    this->conveyor_plate->SetPos_dt(this->conveyor_truss->GetPos_dt() + (ChVector<>(conveyor_speed, 0, 0) >> (*this->conveyor_truss)));

    conveyor_plate->Update(mytime, update_assets);


    ((ChFunction_Ramp*)internal_link->GetMotion_X())->Set_ang(-conveyor_speed);
    // always zero pos. offset (trick):
    ((ChFunction_Ramp*)internal_link->GetMotion_X())->Set_y0(+conveyor_speed * this->GetChTime());  

    this->internal_link->Update(mytime, update_assets);
}

void ChConveyor::SyncCollisionModels() {
    // inherit parent class
    ChPhysicsItem::SyncCollisionModels();

    this->conveyor_truss->SyncCollisionModels();
    this->conveyor_plate->SyncCollisionModels();
}

void ChConveyor::AddCollisionModelsToSystem() {
    // inherit parent class
    ChPhysicsItem::AddCollisionModelsToSystem();
    //this->conveyor_truss->AddCollisionModelsToSystem();
    //this->conveyor_plate->AddCollisionModelsToSystem();
}

void ChConveyor::RemoveCollisionModelsFromSystem() {
    // inherit parent class
    ChPhysicsItem::RemoveCollisionModelsFromSystem();
    //this->conveyor_plate->RemoveCollisionModelsFromSystem();
    //this->conveyor_truss->RemoveCollisionModelsFromSystem();
}

//////// FILE I/O


void ChConveyor::ArchiveOUT(ChArchiveOut& marchive)
{
    // version number
    marchive.VersionWrite(1);

    // serialize parent class
    ChPhysicsItem::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(conveyor_speed);
    marchive << CHNVP(conveyor_truss);
    marchive << CHNVP(conveyor_plate);
    marchive << CHNVP(internal_link);
}

/// Method to allow de serialization of transient data from archives.
void ChConveyor::ArchiveIN(ChArchiveIn& marchive) 
{
    // version number
    int version = marchive.VersionRead();

    // deserialize parent class
    ChPhysicsItem::ArchiveIN(marchive);

    // stream in all member data:
    marchive >> CHNVP(conveyor_speed);
    marchive >> CHNVP(conveyor_truss);
    marchive >> CHNVP(conveyor_plate);
    marchive >> CHNVP(internal_link);
}




}  // END_OF_NAMESPACE____

/////////////////////
