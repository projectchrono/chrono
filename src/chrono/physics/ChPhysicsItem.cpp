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

#include "chrono/physics/ChPhysicsItem.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChPhysicsItem)

ChPhysicsItem::ChPhysicsItem(const ChPhysicsItem& other) : ChObj(other) {
    // Do not copy the system; this is initialized at insertion time
    system = NULL;
    offset_x = other.offset_x;
    offset_w = other.offset_w;
    offset_L = other.offset_L;
    // A copy is not initialized; it records its own reference configuration
    m_init_stage = InitPhase::NONE;
}

ChPhysicsItem::~ChPhysicsItem() {
    SetSystem(NULL);  // note that this might remove collision model from system
}

void ChPhysicsItem::SetSystem(ChSystem* m_system) {
    system = m_system;
}

ChAABB ChPhysicsItem::GetTotalAABB() const {
    return ChAABB();
}

ChVector3d ChPhysicsItem::GetCenter() const {
    auto bbox = GetTotalAABB();
    return (bbox.min + bbox.max) * 0.5;
}

void ChPhysicsItem::InitializePhase(InitPhase phase) {
    if (phase == InitPhase::STATE && m_init_stage == InitPhase::NONE)
        InitializePhase(InitPhase::STRUCTURE);

    if (m_init_stage < phase) {
        if (phase == InitPhase::STRUCTURE)
            SetupInitial();
        else if (phase == InitPhase::STATE)
            SetupInitialState();
        m_init_stage = phase;
    }

    InitializeChildren(phase);
}

void ChPhysicsItem::ResetInitialization() {
    m_init_stage = InitPhase::NONE;
    SetInitializationPending();
}

void ChPhysicsItem::SetInitializationPending() {
    if (system)
        system->m_items_pending = true;
}

/// Takes the F force term, scale by c and scale by i-th weight in Wd at node offset, and adds to R at given offset:
///    R += c*F* Wi
/// This is a weighted version of IntLoadResidual_F, where some items are scaled, ex. to simulate splitting a mass.
/// Many children classes do not need to implement this: here a default fallback to IntLoadResidual_F is implemented.
void ChPhysicsItem::IntLoadResidual_F_weighted(const unsigned int off,  ///< offset in R residual
    ChVectorDynamic<>& R,    ///< result: the R residual, R += c*F
    const double c,          ///< a scaling factor
    ChVectorDynamic<>& Wd    ///< vector with weights, the node effect is "scaled".
) {
    // default use first Wd value at offset off for scaling the entire physics item. Child classes 
    // should override for finer behavior if contain more items etc.
    if (this->GetNumCoordsVelLevel())
        this->IntLoadResidual_F(off, R, c * Wd(off));
    else
        this->IntLoadResidual_F(off, R, c);
}


/// Takes the M*v term,  multiplying mass by a vector, scale by c and and scale by i-th weight in Wd at node offset
/// and adds to R at given offset:
///    R += c*M*v * Wi
/// This is a weighted version of IntLoadResidual_Mv, where some items are scaled, ex. to simulate splitting a mass.
/// Many children classes do not need to implement this: here a default fallback to IntLoadResidual_Mv is implemented.
void ChPhysicsItem::IntLoadResidual_Mv_weighted(const unsigned int off,  ///< offset in R residual
    ChVectorDynamic<>& R,        ///< result: the R residual, R += c*M*v
    const ChVectorDynamic<>& w,  ///< the w vector
    const double c,              ///< a scaling factor
    ChVectorDynamic<>& Wd        ///< vector with weights, the node mass is "scaled".
) {
    // default use first Wd value at offset off for scaling the entire physics item. Child classes 
    // should override for finer behavior if contain more items etc.
    if (this->GetNumCoordsVelLevel())
        this->IntLoadResidual_Mv(off, R, w, c * Wd(off));
    else
        this->IntLoadResidual_Mv(off, R, w, c);
}


/// Adds the lumped mass to a Md vector, representing a mass diagonal matrix. Used by lumped explicit integrators.
/// If mass lumping is impossible or approximate, adds scalar error to "error" parameter.
///    Md += c*diag(M)* Wi
/// This is a weighted version of IntLoadLumpedMass_Md, where some items are scaled, ex. to simulate splitting a mass.
void ChPhysicsItem::IntLoadLumpedMass_Md_weighted(const unsigned int off,  ///< offset in Md vector
    ChVectorDynamic<>& Md,  ///< result: Md vector, diagonal of the lumped mass matrix
    double& err,            ///< result: not touched if lumping does not introduce errors
    const double c,         ///< a scaling factor
    ChVectorDynamic<>& Wd   ///< vector with weights, the node mass is "scaled".
) {
    // default use first Wd value at offset off for scaling the entire physics item. Child classes 
    // should override for finer behavior if contain more items etc.
    if (this->GetNumCoordsVelLevel())
        this->IntLoadLumpedMass_Md(off, Md, err, c * Wd(off));
    else
        this->IntLoadLumpedMass_Md(off, Md, err, c);
}


void ChPhysicsItem::Update(double time, UpdateFlags update_flags) {
    ChObj::Update(time, update_flags);
}

void ChPhysicsItem::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChPhysicsItem>();

    // serialize parent class
    ChObj::ArchiveOut(archive_out);

    // serialize all member data:
    // archive_out << CHNVP(system); ***TODO***
    // archive_out << CHNVP(offset_x);
    // archive_out << CHNVP(offset_w);
    // archive_out << CHNVP(offset_L);
}

/// Method to allow de serialization of transient data from archives.
void ChPhysicsItem::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChPhysicsItem>();

    // deserialize parent class
    ChObj::ArchiveIn(archive_in);

    // A deserialized item is not initialized
    m_init_stage = InitPhase::NONE;

    // stream in all member data:
    // archive_in >> CHNVP(system); ***TODO***
    // archive_in >> CHNVP(offset_x);
    // archive_in >> CHNVP(offset_w);
    // archive_in >> CHNVP(offset_L);
}

}  // end namespace chrono
