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

#include <algorithm>
#include <cstdlib>

#include "chrono/core/ChLinearAlgebra.h"
#include "chrono/core/ChTransform.h"
#include "chrono/physics/ChAssembly.h"
#include "chrono/physics/ChGlobal.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

using namespace fea;
using namespace collision;
using namespace geometry;

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChAssembly)

ChAssembly::ChAssembly()
    : nbodies(0),
      nlinks(0),
      nmeshes(0),
      nphysicsitems(0),
      ndof(0),
      ndoc(0),
      ndoc_w(0),
      ndoc_w_C(0),
      ndoc_w_D(0),
      ncoords(0),
      ncoords_w(0),
      nsysvars(0),
      nsysvars_w(0),
      nbodies_sleep(0),
      nbodies_fixed(0) {}

ChAssembly::ChAssembly(const ChAssembly& other) : ChPhysicsItem(other) {
    nbodies = other.nbodies;
    nlinks = other.nlinks;
    nmeshes = other.nmeshes;
    nphysicsitems = other.nphysicsitems;
    ncoords = other.ncoords;
    ncoords_w = other.ncoords_w;
    ndoc = other.ndoc;
    ndoc_w = other.ndoc_w;
    ndoc_w_C = other.ndoc_w_C;
    ndoc_w_D = other.ndoc_w_D;
    ndof = other.ndof;
    nsysvars = other.nsysvars;
    nsysvars_w = other.nsysvars_w;
    nbodies_sleep = other.nbodies_sleep;
    nbodies_fixed = other.nbodies_fixed;

    //// RADU
    //// TODO:  deep copy of the object lists (bodylist, linklist, meshlist,  otherphysicslist)
}

ChAssembly::~ChAssembly() {
    RemoveAllBodies();
    RemoveAllLinks();
    RemoveAllMeshes();
    RemoveAllOtherPhysicsItems();
}

void ChAssembly::Clear() {
    RemoveAllLinks();
    RemoveAllBodies();
    RemoveAllMeshes();
    RemoveAllOtherPhysicsItems();

    nbodies = 0;
    nlinks = 0;
    nmeshes = 0;
    nphysicsitems = 0;
    ndof = 0;
    ndoc = 0;
    ndoc_w = 0;
    ndoc_w_C = 0;
    ndoc_w_D = 0;
    nsysvars_w = 0;
    ncoords = 0;
    ncoords_w = 0;
    nsysvars = 0;
    ncoords_w = 0;
    nbodies_sleep = 0;
    nbodies_fixed = 0;
}

// Note: removing items from the assembly incurs linear time cost

void ChAssembly::AddBody(std::shared_ptr<ChBody> body) {
    assert(std::find(std::begin(bodylist), std::end(bodylist), body) == bodylist.end());
    assert(body->GetSystem() == nullptr);  // should remove from other system before adding here

    // set system and also add collision models to system
    body->SetSystem(system);
    bodylist.push_back(body);
}

void ChAssembly::RemoveBody(std::shared_ptr<ChBody> body) {
    auto itr = std::find(std::begin(bodylist), std::end(bodylist), body);
    assert(itr != bodylist.end());

    bodylist.erase(itr);
    body->SetSystem(nullptr);
}

void ChAssembly::AddLink(std::shared_ptr<ChLinkBase> link) {
    assert(std::find(std::begin(linklist), std::end(linklist), link) == linklist.end());

    link->SetSystem(system);
    linklist.push_back(link);
}

void ChAssembly::RemoveLink(std::shared_ptr<ChLinkBase> link) {
    auto itr = std::find(std::begin(linklist), std::end(linklist), link);
    assert(itr != linklist.end());

    linklist.erase(itr);
    link->SetSystem(nullptr);
}

void ChAssembly::AddMesh(std::shared_ptr<fea::ChMesh> mesh) {
    assert(std::find(std::begin(meshlist), std::end(meshlist), mesh) == meshlist.end());

    mesh->SetSystem(system);
    meshlist.push_back(mesh);
}

void ChAssembly::RemoveMesh(std::shared_ptr<fea::ChMesh> mesh) {
    auto itr = std::find(std::begin(meshlist), std::end(meshlist), mesh);
    assert(itr != meshlist.end());

    meshlist.erase(itr);
    mesh->SetSystem(nullptr);
}

void ChAssembly::AddOtherPhysicsItem(std::shared_ptr<ChPhysicsItem> item) {
    assert(!std::dynamic_pointer_cast<ChBody>(item));
    assert(!std::dynamic_pointer_cast<ChLinkBase>(item));
    assert(!std::dynamic_pointer_cast<ChMesh>(item));
    assert(std::find(std::begin(otherphysicslist), std::end(otherphysicslist), item) == otherphysicslist.end());
    // assert(item->GetSystem()==nullptr); // should remove from other system before adding here

    // set system and also add collision models to system
    item->SetSystem(system);
    otherphysicslist.push_back(item);
}

void ChAssembly::RemoveOtherPhysicsItem(std::shared_ptr<ChPhysicsItem> item) {
    auto itr = std::find(std::begin(otherphysicslist), std::end(otherphysicslist), item);
    assert(itr != otherphysicslist.end());

    otherphysicslist.erase(itr);
    item->SetSystem(nullptr);
}

void ChAssembly::Add(std::shared_ptr<ChPhysicsItem> item) {
    if (auto body = std::dynamic_pointer_cast<ChBody>(item)) {
        AddBody(body);
        return;
    }

    if (auto link = std::dynamic_pointer_cast<ChLinkBase>(item)) {
        AddLink(link);
        return;
    }

    if (auto mesh = std::dynamic_pointer_cast<fea::ChMesh>(item)) {
        AddMesh(mesh);
        return;
    }

    AddOtherPhysicsItem(item);
}

void ChAssembly::AddBatch(std::shared_ptr<ChPhysicsItem> item) {
    batch_to_insert.push_back(item);
}

void ChAssembly::FlushBatch() {
    for (auto& item : batch_to_insert) {
        Add(item);
    }
    batch_to_insert.clear();
}

void ChAssembly::Remove(std::shared_ptr<ChPhysicsItem> item) {
    if (auto body = std::dynamic_pointer_cast<ChBody>(item)) {
        RemoveBody(body);
        return;
    }

    if (auto link = std::dynamic_pointer_cast<ChLinkBase>(item)) {
        RemoveLink(link);
        return;
    }

    if (auto mesh = std::dynamic_pointer_cast<fea::ChMesh>(item)) {
        RemoveMesh(mesh);
        return;
    }

    RemoveOtherPhysicsItem(item);
}

void ChAssembly::RemoveAllBodies() {
    for (auto& body : bodylist) {
        body->SetSystem(nullptr);
    }
    bodylist.clear();
}

void ChAssembly::RemoveAllLinks() {
    for (auto& link : linklist) {
        link->SetSystem(nullptr);
    }
    linklist.clear();
}

void ChAssembly::RemoveAllMeshes() {
    for (auto& mesh : meshlist) {
        mesh->SetSystem(nullptr);
    }
    meshlist.clear();
}

void ChAssembly::RemoveAllOtherPhysicsItems() {
    for (auto& item : otherphysicslist) {
        item->SetSystem(nullptr);
    }
    otherphysicslist.clear();
}

std::shared_ptr<ChBody> ChAssembly::SearchBody(const char* name) {
    return ChContainerSearchFromName<std::shared_ptr<ChBody>, std::vector<std::shared_ptr<ChBody>>::iterator>(
        name, bodylist.begin(), bodylist.end());
}

std::shared_ptr<ChLinkBase> ChAssembly::SearchLink(const char* name) {
    return ChContainerSearchFromName<std::shared_ptr<ChLinkBase>, std::vector<std::shared_ptr<ChLinkBase>>::iterator>(
        name, linklist.begin(), linklist.end());
}

std::shared_ptr<fea::ChMesh> ChAssembly::SearchMesh(const char* name) {
    return ChContainerSearchFromName<std::shared_ptr<fea::ChMesh>, std::vector<std::shared_ptr<fea::ChMesh>>::iterator>(
        name, meshlist.begin(), meshlist.end());
}

std::shared_ptr<ChPhysicsItem> ChAssembly::SearchOtherPhysicsItem(const char* name) {
    return ChContainerSearchFromName<std::shared_ptr<ChPhysicsItem>,
                                     std::vector<std::shared_ptr<ChPhysicsItem>>::iterator>(
        name, otherphysicslist.begin(), otherphysicslist.end());
}

std::shared_ptr<ChPhysicsItem> ChAssembly::Search(const char* name) {
    if (auto mbo = SearchBody(name))
        return mbo;

    if (auto mli = SearchLink(name))
        return mli;

    if (auto mesh = SearchMesh(name))
        return mesh;

    if (auto mph = SearchOtherPhysicsItem(name))
        return mph;

    return std::shared_ptr<ChPhysicsItem>();  // not found; return an empty shared_ptr
}

std::shared_ptr<ChMarker> ChAssembly::SearchMarker(const char* name) {
    // Iterate over all bodies and search in the body's marker list
    for (auto& body : bodylist) {
        if (auto mmark = body->SearchMarker(name))
            return mmark;
    }

    return (std::shared_ptr<ChMarker>());  // not found; return an empty shared_ptr
}

std::shared_ptr<ChMarker> ChAssembly::SearchMarker(int markID) {
    // Iterate over all bodies and search in the body's marker list
    for (auto& body : bodylist) {
        if (auto res = ChContainerSearchFromID<std::shared_ptr<ChMarker>,
                                               std::vector<std::shared_ptr<ChMarker>>::const_iterator>(
                markID, body->GetMarkerList().begin(), body->GetMarkerList().end()))
            return res;
    }

    return (std::shared_ptr<ChMarker>());  // not found; return an empty shared_ptr
}

// -----------------------------------------------------------------------------

void ChAssembly::SetSystem(ChSystem* m_system) {
    system = m_system;

    for (auto& body : bodylist) {
        body->SetSystem(m_system);
    }
    for (auto& link : linklist) {
        link->SetSystem(m_system);
    }
    for (auto& mesh : meshlist) {
        mesh->SetSystem(m_system);
    }
    for (auto& item : otherphysicslist) {
        item->SetSystem(m_system);
    }
}

void ChAssembly::SyncCollisionModels() {
    for (auto& body : bodylist) {
        body->SyncCollisionModels();
    }
    for (auto& link : linklist) {
        link->SyncCollisionModels();
    }
    for (auto& mesh : meshlist) {
        mesh->SyncCollisionModels();
    }
    for (auto& item : otherphysicslist) {
        item->SyncCollisionModels();
    }
}

// -----------------------------------------------------------------------------
// UPDATING ROUTINES

// Count all bodies, links, meshes, and other physics items.
// Set counters (DOF, num constraints, etc) and offsets.
void ChAssembly::Setup() {
    nbodies = 0;
    nbodies_sleep = 0;
    nbodies_fixed = 0;
    ncoords = 0;
    ncoords_w = 0;
    ndoc = 0;
    ndoc_w = 0;
    ndoc_w_C = 0;
    ndoc_w_D = 0;
    nlinks = 0;
    nmeshes = 0;
    nphysicsitems = 0;

    // Add any items queued for insertion in the assembly's lists.
    this->FlushBatch();

    for (auto& body : bodylist) {
        if (body->GetBodyFixed())
            nbodies_fixed++;
        else if (body->GetSleeping())
            nbodies_sleep++;
        else {
            nbodies++;

            body->SetOffset_x(this->offset_x + ncoords);
            body->SetOffset_w(this->offset_w + ncoords_w);
            body->SetOffset_L(this->offset_L + ndoc_w);

            // body->Setup(); // not needed since in bodies does nothing

            ncoords += body->GetDOF();
            ncoords_w += body->GetDOF_w();
            ndoc_w += body->GetDOC();      // not really needed since ChBody introduces no constraints
            ndoc_w_C += body->GetDOC_c();  // not really needed since ChBody introduces no constraints
            ndoc_w_D += body->GetDOC_d();  // not really needed since ChBody introduces no constraints
        }
    }

    for (auto& link : linklist) {
        if (link->IsActive()) {
            nlinks++;

            link->SetOffset_x(this->offset_x + ncoords);
            link->SetOffset_w(this->offset_w + ncoords_w);
            link->SetOffset_L(this->offset_L + ndoc_w);

            link->Setup();  // compute DOFs etc. and sets the offsets also in child items, if any

            ncoords += link->GetDOF();
            ncoords_w += link->GetDOF_w();
            ndoc_w += link->GetDOC();
            ndoc_w_C += link->GetDOC_c();
            ndoc_w_D += link->GetDOC_d();
        }
    }

    for (auto& mesh : meshlist) {
        nmeshes++;

        mesh->SetOffset_x(this->offset_x + ncoords);
        mesh->SetOffset_w(this->offset_w + ncoords_w);
        mesh->SetOffset_L(this->offset_L + ndoc_w);

        mesh->Setup();  // compute DOFs and iteratively call Setup for child items

        ncoords += mesh->GetDOF();
        ncoords_w += mesh->GetDOF_w();
        ndoc_w += mesh->GetDOC();
        ndoc_w_C += mesh->GetDOC_c();
        ndoc_w_D += mesh->GetDOC_d();
    }

    for (auto& item : otherphysicslist) {
        nphysicsitems++;

        item->SetOffset_x(this->offset_x + ncoords);
        item->SetOffset_w(this->offset_w + ncoords_w);
        item->SetOffset_L(this->offset_L + ndoc_w);

        item->Setup();

        ncoords += item->GetDOF();
        ncoords_w += item->GetDOF_w();
        ndoc_w += item->GetDOC();
        ndoc_w_C += item->GetDOC_c();
        ndoc_w_D += item->GetDOC_d();
    }

    ndoc = ndoc_w + nbodies;          // number of constraints including quaternion constraints.
    nsysvars = ncoords + ndoc;        // total number of variables (coordinates + lagrangian multipliers)
    nsysvars_w = ncoords_w + ndoc_w;  // total number of variables (with 6 dof per body)

    // number of degrees of freedom (approximate - does not consider constr. redundancy, etc)
    ndof = ncoords_w - ndoc_w;
}

// Update assembly's own properties first (ChTime and assets, if any).
// Then update all contents of this assembly.
void ChAssembly::Update(double mytime, bool update_assets) {
    ChPhysicsItem::Update(mytime, update_assets);
    Update(update_assets);
}

// Update all physical items (bodies, links, meshes, etc), including their auxiliary variables.
// Updates all forces (automatic, as children of bodies)
// Updates all markers (automatic, as children of bodies).
void ChAssembly::Update(bool update_assets) {
    //// NOTE: do not switch these to range for loops (may want to use OMP for)
    for (int ip = 0; ip < (int)bodylist.size(); ++ip) {
        bodylist[ip]->Update(ChTime, update_assets);
    }
    for (int ip = 0; ip < (int)otherphysicslist.size(); ++ip) {
        otherphysicslist[ip]->Update(ChTime, update_assets);
    }
    for (int ip = 0; ip < (int)linklist.size(); ++ip) {
        linklist[ip]->Update(ChTime, update_assets);
    }
    for (int ip = 0; ip < (int)meshlist.size(); ++ip) {
        meshlist[ip]->Update(ChTime, update_assets);
    }
}

void ChAssembly::SetNoSpeedNoAcceleration() {
    for (auto& body : bodylist) {
        body->SetNoSpeedNoAcceleration();
    }
    for (auto& link : linklist) {
        link->SetNoSpeedNoAcceleration();
    }
    for (auto& mesh : meshlist) {
        mesh->SetNoSpeedNoAcceleration();
    }
    for (auto& item : otherphysicslist) {
        item->SetNoSpeedNoAcceleration();
    }
}

void ChAssembly::IntStateGather(const unsigned int off_x,
                                ChState& x,
                                const unsigned int off_v,
                                ChStateDelta& v,
                                double& T) {
    unsigned int displ_x = off_x - this->offset_x;
    unsigned int displ_v = off_v - this->offset_w;

    for (auto& body : bodylist) {
        if (body->IsActive())
            body->IntStateGather(displ_x + body->GetOffset_x(), x, displ_v + body->GetOffset_w(), v, T);
    }
    for (auto& link : linklist) {
        if (link->IsActive())
            link->IntStateGather(displ_x + link->GetOffset_x(), x, displ_v + link->GetOffset_w(), v, T);
    }
    for (auto& mesh : meshlist) {
        mesh->IntStateGather(displ_x + mesh->GetOffset_x(), x, displ_v + mesh->GetOffset_w(), v, T);
    }
    for (auto& item : otherphysicslist) {
        item->IntStateGather(displ_x + item->GetOffset_x(), x, displ_v + item->GetOffset_w(), v, T);
    }
    T = GetChTime();
}

void ChAssembly::IntStateScatter(const unsigned int off_x,
                                 const ChState& x,
                                 const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const double T) {
    unsigned int displ_x = off_x - this->offset_x;
    unsigned int displ_v = off_v - this->offset_w;

    for (auto& body : bodylist) {
        if (body->IsActive())
            body->IntStateScatter(displ_x + body->GetOffset_x(), x, displ_v + body->GetOffset_w(), v, T);
    }
    for (auto& link : linklist) {
        if (link->IsActive())
            link->IntStateScatter(displ_x + link->GetOffset_x(), x, displ_v + link->GetOffset_w(), v, T);
    }
    for (auto& mesh : meshlist) {
        mesh->IntStateScatter(displ_x + mesh->GetOffset_x(), x, displ_v + mesh->GetOffset_w(), v, T);
    }
    for (auto& item : otherphysicslist) {
        item->IntStateScatter(displ_x + item->GetOffset_x(), x, displ_v + item->GetOffset_w(), v, T);
    }
    SetChTime(T);

    // Note: all those IntStateScatter() above should call Update() automatically
    // for each object in the loop, therefore:
    // -do not call Update() on this.
    // -do not call ChPhysicsItem::IntStateScatter() -it calls this->Update() anyway-
    // because this would cause redundant updates.
}

void ChAssembly::IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    unsigned int displ_a = off_a - this->offset_w;

    for (auto& body : bodylist) {
        if (body->IsActive())
            body->IntStateGatherAcceleration(displ_a + body->GetOffset_w(), a);
    }
    for (auto& link : linklist) {
        if (link->IsActive())
            link->IntStateGatherAcceleration(displ_a + link->GetOffset_w(), a);
    }
    for (auto& mesh : meshlist) {
        mesh->IntStateGatherAcceleration(displ_a + mesh->GetOffset_w(), a);
    }
    for (auto& item : otherphysicslist) {
        item->IntStateGatherAcceleration(displ_a + item->GetOffset_w(), a);
    }
}

// From state derivative (acceleration) to system, sometimes might be needed
void ChAssembly::IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    unsigned int displ_a = off_a - this->offset_w;

    for (auto& body : bodylist) {
        if (body->IsActive())
            body->IntStateScatterAcceleration(displ_a + body->GetOffset_w(), a);
    }
    for (auto& link : linklist) {
        if (link->IsActive())
            link->IntStateScatterAcceleration(displ_a + link->GetOffset_w(), a);
    }
    for (auto& mesh : meshlist) {
        mesh->IntStateScatterAcceleration(displ_a + mesh->GetOffset_w(), a);
    }
    for (auto& item : otherphysicslist) {
        item->IntStateScatterAcceleration(displ_a + item->GetOffset_w(), a);
    }
}

// From system to reaction forces (last computed) - some timestepper might need this
void ChAssembly::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    unsigned int displ_L = off_L - this->offset_L;

    for (auto& body : bodylist) {
        if (body->IsActive())
            body->IntStateGatherReactions(displ_L + body->GetOffset_L(), L);
    }
    for (auto& link : linklist) {
        if (link->IsActive())
            link->IntStateGatherReactions(displ_L + link->GetOffset_L(), L);
    }
    for (auto& mesh : meshlist) {
        mesh->IntStateGatherReactions(displ_L + mesh->GetOffset_L(), L);
    }
    for (auto& item : otherphysicslist) {
        item->IntStateGatherReactions(displ_L + item->GetOffset_L(), L);
    }
}

// From reaction forces to system, ex. store last computed reactions in ChLinkBase objects for plotting etc.
void ChAssembly::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    unsigned int displ_L = off_L - this->offset_L;

    for (auto& body : bodylist) {
        if (body->IsActive())
            body->IntStateScatterReactions(displ_L + body->GetOffset_L(), L);
    }
    for (auto& link : linklist) {
        if (link->IsActive())
            link->IntStateScatterReactions(displ_L + link->GetOffset_L(), L);
    }
    for (auto& mesh : meshlist) {
        mesh->IntStateScatterReactions(displ_L + mesh->GetOffset_L(), L);
    }
    for (auto& item : otherphysicslist) {
        item->IntStateScatterReactions(displ_L + item->GetOffset_L(), L);
    }
}

void ChAssembly::IntStateIncrement(const unsigned int off_x,
                                   ChState& x_new,
                                   const ChState& x,
                                   const unsigned int off_v,
                                   const ChStateDelta& Dv) {
    unsigned int displ_x = off_x - this->offset_x;
    unsigned int displ_v = off_v - this->offset_w;

    for (auto& body : bodylist) {
        if (body->IsActive())
            body->IntStateIncrement(displ_x + body->GetOffset_x(), x_new, x, displ_v + body->GetOffset_w(), Dv);
    }

    for (auto& link : linklist) {
        if (link->IsActive())
            link->IntStateIncrement(displ_x + link->GetOffset_x(), x_new, x, displ_v + link->GetOffset_w(), Dv);
    }

    for (auto& mesh : meshlist) {
        mesh->IntStateIncrement(displ_x + mesh->GetOffset_x(), x_new, x, displ_v + mesh->GetOffset_w(), Dv);
    }

    for (auto& item : otherphysicslist) {
        item->IntStateIncrement(displ_x + item->GetOffset_x(), x_new, x, displ_v + item->GetOffset_w(), Dv);
    }
}

void ChAssembly::IntLoadResidual_F(const unsigned int off,  ///< offset in R residual
                                   ChVectorDynamic<>& R,    ///< result: the R residual, R += c*F
                                   const double c)          ///< a scaling factor
{
    unsigned int displ_v = off - this->offset_w;

    for (auto& body : bodylist) {
        if (body->IsActive())
            body->IntLoadResidual_F(displ_v + body->GetOffset_w(), R, c);
    }
    for (auto& link : linklist) {
        if (link->IsActive())
            link->IntLoadResidual_F(displ_v + link->GetOffset_w(), R, c);
    }
    for (auto& mesh : meshlist) {
        mesh->IntLoadResidual_F(displ_v + mesh->GetOffset_w(), R, c);
    }
    for (auto& item : otherphysicslist) {
        item->IntLoadResidual_F(displ_v + item->GetOffset_w(), R, c);
    }
}

void ChAssembly::IntLoadResidual_Mv(const unsigned int off,      ///< offset in R residual
                                    ChVectorDynamic<>& R,        ///< result: the R residual, R += c*M*v
                                    const ChVectorDynamic<>& w,  ///< the w vector
                                    const double c               ///< a scaling factor
) {
    unsigned int displ_v = off - this->offset_w;

    for (auto& body : bodylist) {
        if (body->IsActive())
            body->IntLoadResidual_Mv(displ_v + body->GetOffset_w(), R, w, c);
    }
    for (auto& link : linklist) {
        if (link->IsActive())
            link->IntLoadResidual_Mv(displ_v + link->GetOffset_w(), R, w, c);
    }
    for (auto& mesh : meshlist) {
        mesh->IntLoadResidual_Mv(displ_v + mesh->GetOffset_w(), R, w, c);
    }
    for (auto& item : otherphysicslist) {
        item->IntLoadResidual_Mv(displ_v + item->GetOffset_w(), R, w, c);
    }
}

void ChAssembly::IntLoadResidual_CqL(const unsigned int off_L,    ///< offset in L multipliers
                                     ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
                                     const ChVectorDynamic<>& L,  ///< the L vector
                                     const double c               ///< a scaling factor
) {
    unsigned int displ_L = off_L - this->offset_L;

    for (auto& body : bodylist) {
        if (body->IsActive())
            body->IntLoadResidual_CqL(displ_L + body->GetOffset_L(), R, L, c);
    }
    for (auto& link : linklist) {
        if (link->IsActive())
            link->IntLoadResidual_CqL(displ_L + link->GetOffset_L(), R, L, c);
    }
    for (auto& mesh : meshlist) {
        mesh->IntLoadResidual_CqL(displ_L + mesh->GetOffset_L(), R, L, c);
    }
    for (auto& item : otherphysicslist) {
        item->IntLoadResidual_CqL(displ_L + item->GetOffset_L(), R, L, c);
    }
}

void ChAssembly::IntLoadConstraint_C(const unsigned int off_L,  ///< offset in Qc residual
                                     ChVectorDynamic<>& Qc,     ///< result: the Qc residual, Qc += c*C
                                     const double c,            ///< a scaling factor
                                     bool do_clamp,             ///< apply clamping to c*C?
                                     double recovery_clamp      ///< value for min/max clamping of c*C
) {
    unsigned int displ_L = off_L - this->offset_L;

    for (auto& body : bodylist) {
        if (body->IsActive())
            body->IntLoadConstraint_C(displ_L + body->GetOffset_L(), Qc, c, do_clamp, recovery_clamp);
    }
    for (auto& link : linklist) {
        if (link->IsActive())
            link->IntLoadConstraint_C(displ_L + link->GetOffset_L(), Qc, c, do_clamp, recovery_clamp);
    }
    for (auto& mesh : meshlist) {
        mesh->IntLoadConstraint_C(displ_L + mesh->GetOffset_L(), Qc, c, do_clamp, recovery_clamp);
    }
    for (auto& item : otherphysicslist) {
        item->IntLoadConstraint_C(displ_L + item->GetOffset_L(), Qc, c, do_clamp, recovery_clamp);
    }
}

void ChAssembly::IntLoadConstraint_Ct(const unsigned int off_L,  ///< offset in Qc residual
                                      ChVectorDynamic<>& Qc,     ///< result: the Qc residual, Qc += c*Ct
                                      const double c             ///< a scaling factor
) {
    unsigned int displ_L = off_L - this->offset_L;

    for (auto& body : bodylist) {
        if (body->IsActive())
            body->IntLoadConstraint_Ct(displ_L + body->GetOffset_L(), Qc, c);
    }
    for (auto& link : linklist) {
        if (link->IsActive())
            link->IntLoadConstraint_Ct(displ_L + link->GetOffset_L(), Qc, c);
    }
    for (auto& mesh : meshlist) {
        mesh->IntLoadConstraint_Ct(displ_L + mesh->GetOffset_L(), Qc, c);
    }
    for (auto& item : otherphysicslist) {
        item->IntLoadConstraint_Ct(displ_L + item->GetOffset_L(), Qc, c);
    }
}

void ChAssembly::IntToDescriptor(const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const ChVectorDynamic<>& R,
                                 const unsigned int off_L,
                                 const ChVectorDynamic<>& L,
                                 const ChVectorDynamic<>& Qc) {
    unsigned int displ_L = off_L - this->offset_L;
    unsigned int displ_v = off_v - this->offset_w;

    for (auto& body : bodylist) {
        if (body->IsActive())
            body->IntToDescriptor(displ_v + body->GetOffset_w(), v, R, displ_L + body->GetOffset_L(), L, Qc);
    }

    for (auto& link : linklist) {
        if (link->IsActive())
            link->IntToDescriptor(displ_v + link->GetOffset_w(), v, R, displ_L + link->GetOffset_L(), L, Qc);
    }

    for (auto& mesh : meshlist) {
        mesh->IntToDescriptor(displ_v + mesh->GetOffset_w(), v, R, displ_L + mesh->GetOffset_L(), L, Qc);
    }

    for (auto& item : otherphysicslist) {
        item->IntToDescriptor(displ_v + item->GetOffset_w(), v, R, displ_L + item->GetOffset_L(), L, Qc);
    }
}

void ChAssembly::IntFromDescriptor(const unsigned int off_v,
                                   ChStateDelta& v,
                                   const unsigned int off_L,
                                   ChVectorDynamic<>& L) {
    unsigned int displ_L = off_L - this->offset_L;
    unsigned int displ_v = off_v - this->offset_w;

    for (auto& body : bodylist) {
        if (body->IsActive())
            body->IntFromDescriptor(displ_v + body->GetOffset_w(), v, displ_L + body->GetOffset_L(), L);
    }

    for (auto& link : linklist) {
        if (link->IsActive())
            link->IntFromDescriptor(displ_v + link->GetOffset_w(), v, displ_L + link->GetOffset_L(), L);
    }

    for (auto& mesh : meshlist) {
        mesh->IntFromDescriptor(displ_v + mesh->GetOffset_w(), v, displ_L + mesh->GetOffset_L(), L);
    }

    for (auto& item : otherphysicslist) {
        item->IntFromDescriptor(displ_v + item->GetOffset_w(), v, displ_L + item->GetOffset_L(), L);
    }
}

// -----------------------------------------------------------------------------

void ChAssembly::InjectVariables(ChSystemDescriptor& mdescriptor) {
    for (auto& body : bodylist) {
        body->InjectVariables(mdescriptor);
    }
    for (auto& link : linklist) {
        link->InjectVariables(mdescriptor);
    }
    for (auto& mesh : meshlist) {
        mesh->InjectVariables(mdescriptor);
    }
    for (auto& item : otherphysicslist) {
        item->InjectVariables(mdescriptor);
    }
}

void ChAssembly::VariablesFbReset() {
    for (auto& body : bodylist) {
        body->VariablesFbReset();
    }
    for (auto& link : linklist) {
        link->VariablesFbReset();
    }
    for (auto& mesh : meshlist) {
        mesh->VariablesFbReset();
    }
    for (auto& item : otherphysicslist) {
        item->VariablesFbReset();
    }
}

void ChAssembly::VariablesFbLoadForces(double factor) {
    for (auto& body : bodylist) {
        body->VariablesFbLoadForces(factor);
    }
    for (auto& link : linklist) {
        link->VariablesFbLoadForces(factor);
    }
    for (auto& mesh : meshlist) {
        mesh->VariablesFbLoadForces(factor);
    }
    for (auto& item : otherphysicslist) {
        item->VariablesFbLoadForces(factor);
    }
}

void ChAssembly::VariablesFbIncrementMq() {
    for (auto& body : bodylist) {
        body->VariablesFbIncrementMq();
    }
    for (auto& link : linklist) {
        link->VariablesFbIncrementMq();
    }
    for (auto& mesh : meshlist) {
        mesh->VariablesFbIncrementMq();
    }
    for (auto& item : otherphysicslist) {
        item->VariablesFbIncrementMq();
    }
}

void ChAssembly::VariablesQbLoadSpeed() {
    for (auto& body : bodylist) {
        body->VariablesQbLoadSpeed();
    }
    for (auto& link : linklist) {
        link->VariablesQbLoadSpeed();
    }
    for (auto& mesh : meshlist) {
        mesh->VariablesQbLoadSpeed();
    }
    for (auto& item : otherphysicslist) {
        item->VariablesQbLoadSpeed();
    }
}

void ChAssembly::VariablesQbSetSpeed(double step) {
    for (auto& body : bodylist) {
        body->VariablesQbSetSpeed(step);
    }
    for (auto& link : linklist) {
        link->VariablesQbSetSpeed(step);
    }
    for (auto& mesh : meshlist) {
        mesh->VariablesQbSetSpeed(step);
    }
    for (auto& item : otherphysicslist) {
        item->VariablesQbSetSpeed(step);
    }
}

void ChAssembly::VariablesQbIncrementPosition(double dt_step) {
    for (auto& body : bodylist) {
        body->VariablesQbIncrementPosition(dt_step);
    }
    for (auto& link : linklist) {
        link->VariablesQbIncrementPosition(dt_step);
    }
    for (auto& mesh : meshlist) {
        mesh->VariablesQbIncrementPosition(dt_step);
    }
    for (auto& item : otherphysicslist) {
        item->VariablesQbIncrementPosition(dt_step);
    }
}

void ChAssembly::InjectConstraints(ChSystemDescriptor& mdescriptor) {
    for (auto& body : bodylist) {
        body->InjectConstraints(mdescriptor);
    }
    for (auto& link : linklist) {
        link->InjectConstraints(mdescriptor);
    }
    for (auto& mesh : meshlist) {
        mesh->InjectConstraints(mdescriptor);
    }
    for (auto& item : otherphysicslist) {
        item->InjectConstraints(mdescriptor);
    }
}

void ChAssembly::ConstraintsBiReset() {
    for (auto& body : bodylist) {
        body->ConstraintsBiReset();
    }
    for (auto& link : linklist) {
        link->ConstraintsBiReset();
    }
    for (auto& mesh : meshlist) {
        mesh->ConstraintsBiReset();
    }
    for (auto& item : otherphysicslist) {
        item->ConstraintsBiReset();
    }
}

void ChAssembly::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    for (auto& body : bodylist) {
        body->ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);
    }
    for (auto& link : linklist) {
        link->ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);
    }
    for (auto& mesh : meshlist) {
        mesh->ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);
    }
    for (auto& item : otherphysicslist) {
        item->ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);
    }
}

void ChAssembly::ConstraintsBiLoad_Ct(double factor) {
    for (auto& body : bodylist) {
        body->ConstraintsBiLoad_Ct(factor);
    }
    for (auto& link : linklist) {
        link->ConstraintsBiLoad_Ct(factor);
    }
    for (auto& mesh : meshlist) {
        mesh->ConstraintsBiLoad_Ct(factor);
    }
    for (auto& item : otherphysicslist) {
        item->ConstraintsBiLoad_Ct(factor);
    }
}

void ChAssembly::ConstraintsBiLoad_Qc(double factor) {
    for (auto& body : bodylist) {
        body->ConstraintsBiLoad_Qc(factor);
    }
    for (auto& link : linklist) {
        link->ConstraintsBiLoad_Qc(factor);
    }
    for (auto& mesh : meshlist) {
        mesh->ConstraintsBiLoad_Qc(factor);
    }
    for (auto& item : otherphysicslist) {
        item->ConstraintsBiLoad_Qc(factor);
    }
}

void ChAssembly::ConstraintsFbLoadForces(double factor) {
    for (auto& body : bodylist) {
        body->ConstraintsFbLoadForces(factor);
    }
    for (auto& link : linklist) {
        link->ConstraintsFbLoadForces(factor);
    }
    for (auto& mesh : meshlist) {
        mesh->ConstraintsFbLoadForces(factor);
    }
    for (auto& item : otherphysicslist) {
        item->ConstraintsFbLoadForces(factor);
    }
}

void ChAssembly::ConstraintsLoadJacobians() {
    for (auto& body : bodylist) {
        body->ConstraintsLoadJacobians();
    }
    for (auto& link : linklist) {
        link->ConstraintsLoadJacobians();
    }
    for (auto& mesh : meshlist) {
        mesh->ConstraintsLoadJacobians();
    }
    for (auto& item : otherphysicslist) {
        item->ConstraintsLoadJacobians();
    }
}

void ChAssembly::ConstraintsFetch_react(double factor) {
    for (auto& body : bodylist) {
        body->ConstraintsFetch_react(factor);
    }
    for (auto& link : linklist) {
        link->ConstraintsFetch_react(factor);
    }
    for (auto& mesh : meshlist) {
        mesh->ConstraintsFetch_react(factor);
    }
    for (auto& item : otherphysicslist) {
        item->ConstraintsFetch_react(factor);
    }
}

void ChAssembly::InjectKRMmatrices(ChSystemDescriptor& mdescriptor) {
    for (auto& body : bodylist) {
        body->InjectKRMmatrices(mdescriptor);
    }
    for (auto& link : linklist) {
        link->InjectKRMmatrices(mdescriptor);
    }
    for (auto& mesh : meshlist) {
        mesh->InjectKRMmatrices(mdescriptor);
    }
    for (auto& item : otherphysicslist) {
        item->InjectKRMmatrices(mdescriptor);
    }
}

void ChAssembly::KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor) {
    for (auto& body : bodylist) {
        body->KRMmatricesLoad(Kfactor, Rfactor, Mfactor);
    }
    for (auto& link : linklist) {
        link->KRMmatricesLoad(Kfactor, Rfactor, Mfactor);
    }
    for (auto& mesh : meshlist) {
        mesh->KRMmatricesLoad(Kfactor, Rfactor, Mfactor);
    }
    for (auto& item : otherphysicslist) {
        item->KRMmatricesLoad(Kfactor, Rfactor, Mfactor);
    }
}

// -----------------------------------------------------------------------------
//  STREAMING - FILE HANDLING

void ChAssembly::ShowHierarchy(ChStreamOutAscii& m_file, int level) {
    std::string mtabs;
    for (int i = 0; i < level; ++i)
        mtabs += "  ";

    m_file << "\n" << mtabs << "List of the " << (int)bodylist.size() << " added rigid bodies: \n";
    for (auto& body : bodylist) {
        m_file << mtabs << "  BODY:       " << body->GetName() << "\n";

        for (auto& marker : body->GetMarkerList()) {
            m_file << mtabs << "    MARKER:  " << marker->GetName() << "\n";
        }

        for (auto& force : body->GetForceList()) {
            m_file << mtabs << "    FORCE:  " << force->GetName() << "\n";
        }
    }

    m_file << "\n" << mtabs << "List of the " << (int)linklist.size() << " added links: \n";
    for (auto& link : linklist) {
        m_file << mtabs << "  LINK:  " << link->GetName() << " [" << typeid(link.get()).name() << "]\n";
        if (auto malink = std::dynamic_pointer_cast<ChLinkMarkers>(link)) {
            if (malink->GetMarker1())
                m_file << mtabs << "    marker1:  " << malink->GetMarker1()->GetName() << "\n";
            if (malink->GetMarker2())
                m_file << mtabs << "    marker2:  " << malink->GetMarker2()->GetName() << "\n";
        }
    }

    m_file << "\n" << mtabs << "List of the " << (int)meshlist.size() << " added meshes: \n";
    for (auto& mesh : meshlist) {
        m_file << mtabs << "  MESH :   " << mesh->GetName() << "\n";
    }

    m_file << "\n" << mtabs << "List of other " << (int)otherphysicslist.size() << " added physic items: \n";
    for (auto& item : otherphysicslist) {
        m_file << mtabs << "  PHYSIC ITEM :   " << item->GetName() << " [" << typeid(item.get()).name() << "]\n";

        // recursion:
        if (auto assem = std::dynamic_pointer_cast<ChAssembly>(item))
            assem->ShowHierarchy(m_file, level + 1);
    }

    m_file << "\n\n";
}

void ChAssembly::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChAssembly>();

    // serialize parent class
    ChPhysicsItem::ArchiveOUT(marchive);

    // serialize all member data:

    marchive << CHNVP(bodylist, "bodies");
    marchive << CHNVP(linklist, "links");
    marchive << CHNVP(meshlist, "meshes");
    marchive << CHNVP(otherphysicslist, "other_physics_items");
}

void ChAssembly::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChAssembly>();

    // deserialize parent class
    ChPhysicsItem::ArchiveIN(marchive);

    // stream in all member data:
    std::vector<std::shared_ptr<ChBody>> tempbodies;
    std::vector<std::shared_ptr<ChLinkBase>> templinks;
    std::vector<std::shared_ptr<ChMesh>> tempmeshes;
    std::vector<std::shared_ptr<ChPhysicsItem>> tempitems;
    marchive >> CHNVP(tempbodies, "bodies");
    marchive >> CHNVP(templinks, "links");
    marchive >> CHNVP(tempmeshes, "meshes");
    marchive >> CHNVP(tempitems, "other_physics_items");
    // trick needed because the "Add...()" functions are required
    RemoveAllBodies();
    for (auto& body : tempbodies) {
        AddBody(body);
    }
    RemoveAllLinks();
    for (auto& link : templinks) {
        AddLink(link);
    }
    RemoveAllMeshes();
    for (auto& mesh : tempmeshes) {
        AddMesh(mesh);
    }
    RemoveAllOtherPhysicsItems();
    for (auto& item : tempitems) {
        AddOtherPhysicsItem(item);
    }

    // Recompute statistics, offsets, etc.
    Setup();
}

}  // end namespace chrono
