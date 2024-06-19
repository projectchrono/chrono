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

#include "chrono/core/ChGlobal.h"
#include "chrono/physics/ChAssembly.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

using namespace fea;

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChAssembly)

ChAssembly::ChAssembly()
    : m_num_bodies_active(0),
      m_num_bodies_sleep(0),
      m_num_bodies_fixed(0),
      m_num_shafts(0),
      m_num_shafts_sleep(0),
      m_num_shafts_fixed(0),
      m_num_links_active(0),
      m_num_meshes(0),
      m_num_otherphysicsitems_active(0),
      m_num_coords_pos(0),
      m_num_coords_vel(0),
      m_num_constr(0),
      m_num_constr_bil(0),
      m_num_constr_uni(0) {}

ChAssembly::ChAssembly(const ChAssembly& other) : ChPhysicsItem(other) {
    m_num_bodies_active = other.m_num_bodies_active;
    m_num_bodies_sleep = other.m_num_bodies_sleep;
    m_num_bodies_fixed = other.m_num_bodies_fixed;
    m_num_shafts = other.m_num_shafts;
    m_num_shafts_sleep = other.m_num_shafts_sleep;
    m_num_shafts_fixed = other.m_num_shafts_fixed;
    m_num_links_active = other.m_num_links_active;
    m_num_meshes = other.m_num_meshes;
    m_num_otherphysicsitems_active = other.m_num_otherphysicsitems_active;
    m_num_coords_pos = other.m_num_coords_pos;
    m_num_coords_vel = other.m_num_coords_vel;
    m_num_constr = other.m_num_constr;
    m_num_constr_bil = other.m_num_constr_bil;
    m_num_constr_uni = other.m_num_constr_uni;

    //// RADU
    //// TODO:  deep copy of the object lists (bodylist, shaftlist, linklist, meshlist,  otherphysicslist)
}

ChAssembly::~ChAssembly() {
    RemoveAllBodies();
    RemoveAllShafts();
    RemoveAllLinks();
    RemoveAllMeshes();
    RemoveAllOtherPhysicsItems();
}

ChAssembly& ChAssembly::operator=(ChAssembly other) {
    ChAssembly tmp(other);
    swap(*this, other);
    return *this;
}

// Note: implement this as a friend function (instead of a member function swap(ChAssembly& other)) so that other
// classes that have a ChAssembly member (currently only ChSystem) could use it, the same way we use std::swap here.
void swap(ChAssembly& first, ChAssembly& second) {
    using std::swap;
    swap(first.m_num_bodies_active, second.m_num_bodies_active);
    swap(first.m_num_bodies_sleep, second.m_num_bodies_sleep);
    swap(first.m_num_bodies_fixed, second.m_num_bodies_fixed);
    swap(first.m_num_shafts, second.m_num_shafts);
    swap(first.m_num_shafts_sleep, second.m_num_shafts_sleep);
    swap(first.m_num_shafts_fixed, second.m_num_shafts_fixed);
    swap(first.m_num_links_active, second.m_num_links_active);
    swap(first.m_num_meshes, second.m_num_meshes);
    swap(first.m_num_otherphysicsitems_active, second.m_num_otherphysicsitems_active);
    swap(first.m_num_coords_pos, second.m_num_coords_pos);
    swap(first.m_num_coords_vel, second.m_num_coords_vel);
    swap(first.m_num_constr, second.m_num_constr);
    swap(first.m_num_constr_bil, second.m_num_constr_bil);
    swap(first.m_num_constr_uni, second.m_num_constr_uni);

    //// RADU
    //// TODO: deal with all other member variables...
}

void ChAssembly::Clear() {
    RemoveAllLinks();
    RemoveAllBodies();
    RemoveAllShafts();
    RemoveAllMeshes();
    RemoveAllOtherPhysicsItems();

    m_num_bodies_active = 0;
    m_num_bodies_sleep = 0;
    m_num_bodies_fixed = 0;
    m_num_shafts = 0;
    m_num_shafts_sleep = 0;
    m_num_shafts_fixed = 0;
    m_num_links_active = 0;
    m_num_meshes = 0;
    m_num_otherphysicsitems_active = 0;
    m_num_constr = 0;
    m_num_constr_bil = 0;
    m_num_constr_uni = 0;
    m_num_coords_pos = 0;
    m_num_coords_vel = 0;
    m_num_coords_vel = 0;
}

// Note: removing items from the assembly incurs linear time cost

void ChAssembly::AddBody(std::shared_ptr<ChBody> body) {
    assert(std::find(std::begin(bodylist), std::end(bodylist), body) == bodylist.end());
    assert(body->GetSystem() == nullptr);  // should remove from other system before adding here

    // set system and also add collision models to system
    body->SetSystem(system);
    bodylist.push_back(body);

    ////system->is_initialized = false;  // Not needed, unless/until ChBody::SetupInitial does something
    system->is_updated = false;
}

void ChAssembly::RemoveBody(std::shared_ptr<ChBody> body) {
    auto itr = std::find(std::begin(bodylist), std::end(bodylist), body);
    assert(itr != bodylist.end());

    bodylist.erase(itr);
    body->SetSystem(nullptr);

    system->is_updated = false;
}

void ChAssembly::AddShaft(std::shared_ptr<ChShaft> shaft) {
    assert(std::find(std::begin(shaftlist), std::end(shaftlist), shaft) == shaftlist.end());
    assert(shaft->GetSystem() == nullptr);  // should remove from other system before adding here

    shaft->SetSystem(system);
    shaftlist.push_back(shaft);

    ////system->is_initialized = false;  // Not needed, unless/until ChShaft::SetupInitial does something
    system->is_updated = false;
}

void ChAssembly::RemoveShaft(std::shared_ptr<ChShaft> shaft) {
    auto itr = std::find(std::begin(shaftlist), std::end(shaftlist), shaft);
    assert(itr != shaftlist.end());

    shaftlist.erase(itr);
    shaft->SetSystem(nullptr);

    system->is_updated = false;
}

void ChAssembly::AddLink(std::shared_ptr<ChLinkBase> link) {
    assert(std::find(std::begin(linklist), std::end(linklist), link) == linklist.end());
    assert(link->GetSystem() == nullptr || link->GetSystem() == system);

    link->SetSystem(system);
    linklist.push_back(link);

    ////system->is_initialized = false;  // Not needed, unless/until ChLink::SetupInitial does something
    system->is_updated = false;
}

void ChAssembly::RemoveLink(std::shared_ptr<ChLinkBase> link) {
    auto itr = std::find(std::begin(linklist), std::end(linklist), link);
    assert(itr != linklist.end());

    linklist.erase(itr);
    link->SetSystem(nullptr);

    system->is_updated = false;
}

void ChAssembly::AddMesh(std::shared_ptr<fea::ChMesh> mesh) {
    assert(std::find(std::begin(meshlist), std::end(meshlist), mesh) == meshlist.end());

    mesh->SetSystem(system);
    meshlist.push_back(mesh);

    system->is_initialized = false;
    system->is_updated = false;
}

void ChAssembly::RemoveMesh(std::shared_ptr<fea::ChMesh> mesh) {
    auto itr = std::find(std::begin(meshlist), std::end(meshlist), mesh);
    assert(itr != meshlist.end());

    meshlist.erase(itr);
    mesh->SetSystem(nullptr);

    system->is_updated = false;
}

void ChAssembly::AddOtherPhysicsItem(std::shared_ptr<ChPhysicsItem> item) {
    assert(!std::dynamic_pointer_cast<ChBody>(item));
    assert(!std::dynamic_pointer_cast<ChShaft>(item));
    assert(!std::dynamic_pointer_cast<ChLinkBase>(item));
    assert(!std::dynamic_pointer_cast<ChMesh>(item));
    assert(std::find(std::begin(otherphysicslist), std::end(otherphysicslist), item) == otherphysicslist.end());
    assert(item->GetSystem() == nullptr || item->GetSystem() == system);

    // set system and also add collision models to system
    item->SetSystem(system);
    otherphysicslist.push_back(item);

    ////system->is_initialized = false;  // Not needed, unless/until ChPhysicsItem::SetupInitial does something
    system->is_updated = false;
}

void ChAssembly::RemoveOtherPhysicsItem(std::shared_ptr<ChPhysicsItem> item) {
    auto itr = std::find(std::begin(otherphysicslist), std::end(otherphysicslist), item);
    assert(itr != otherphysicslist.end());

    otherphysicslist.erase(itr);
    item->SetSystem(nullptr);

    system->is_updated = false;
}

void ChAssembly::Add(std::shared_ptr<ChPhysicsItem> item) {
    if (auto body = std::dynamic_pointer_cast<ChBody>(item)) {
        AddBody(body);
        return;
    }

    if (auto shaft = std::dynamic_pointer_cast<ChShaft>(item)) {
        AddShaft(shaft);
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

    system->is_initialized = false;  // Needed, as the list may include a ChMesh
    system->is_updated = false;
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

    if (auto shaft = std::dynamic_pointer_cast<ChShaft>(item)) {
        RemoveShaft(shaft);
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

    if (system)
        system->is_updated = false;
}

void ChAssembly::RemoveAllShafts() {
    for (auto& shaft : shaftlist) {
        shaft->SetSystem(nullptr);
    }
    shaftlist.clear();

    if (system)
        system->is_updated = false;
}

void ChAssembly::RemoveAllLinks() {
    for (auto& link : linklist) {
        link->SetSystem(nullptr);
    }
    linklist.clear();

    if (system)
        system->is_updated = false;
}

void ChAssembly::RemoveAllMeshes() {
    for (auto& mesh : meshlist) {
        mesh->SetSystem(nullptr);
    }
    meshlist.clear();

    if (system)
        system->is_updated = false;
}

void ChAssembly::RemoveAllOtherPhysicsItems() {
    for (auto& item : otherphysicslist) {
        item->SetSystem(nullptr);
    }
    otherphysicslist.clear();

    if (system)
        system->is_updated = false;
}

std::shared_ptr<ChBody> ChAssembly::SearchBody(const std::string& name) const {
    auto body = std::find_if(std::begin(bodylist), std::end(bodylist),
                             [name](std::shared_ptr<ChBody> body) { return body->GetName() == name; });
    return (body != std::end(bodylist)) ? *body : nullptr;
}

std::shared_ptr<ChBody> ChAssembly::SearchBodyID(int id) const {
    auto body = std::find_if(std::begin(bodylist), std::end(bodylist),
                             [id](std::shared_ptr<ChBody> body) { return body->GetIdentifier() == id; });
    return (body != std::end(bodylist)) ? *body : nullptr;
}

std::shared_ptr<ChShaft> ChAssembly::SearchShaft(const std::string& name) const {
    auto shaft = std::find_if(std::begin(shaftlist), std::end(shaftlist),
                              [name](std::shared_ptr<ChShaft> shaft) { return shaft->GetName() == name; });
    return (shaft != std::end(shaftlist)) ? *shaft : nullptr;
}

std::shared_ptr<ChLinkBase> ChAssembly::SearchLink(const std::string& name) const {
    auto link = std::find_if(std::begin(linklist), std::end(linklist),
                             [name](std::shared_ptr<ChLinkBase> link) { return link->GetName() == name; });
    return (link != std::end(linklist)) ? *link : nullptr;
}

std::shared_ptr<fea::ChMesh> ChAssembly::SearchMesh(const std::string& name) const {
    auto mesh = std::find_if(std::begin(meshlist), std::end(meshlist),
                             [name](std::shared_ptr<fea::ChMesh> mesh) { return mesh->GetName() == name; });
    return (mesh != std::end(meshlist)) ? *mesh : nullptr;
}

std::shared_ptr<ChPhysicsItem> ChAssembly::SearchOtherPhysicsItem(const std::string& name) const {
    auto item = std::find_if(std::begin(otherphysicslist), std::end(otherphysicslist),
                             [name](std::shared_ptr<ChPhysicsItem> item) { return item->GetName() == name; });
    return (item != std::end(otherphysicslist)) ? *item : nullptr;
}

std::shared_ptr<ChPhysicsItem> ChAssembly::Search(const std::string& name) const {
    if (auto mbo = SearchBody(name))
        return mbo;

    if (auto msh = SearchShaft(name))
        return msh;

    if (auto mli = SearchLink(name))
        return mli;

    if (auto mesh = SearchMesh(name))
        return mesh;

    if (auto mph = SearchOtherPhysicsItem(name))
        return mph;

    return std::shared_ptr<ChPhysicsItem>();  // not found; return an empty shared_ptr
}

std::shared_ptr<ChMarker> ChAssembly::SearchMarker(const std::string& name) const {
    // Iterate over all bodies and search in the body's marker list
    for (auto& body : bodylist) {
        if (auto marker = body->SearchMarker(name))
            return marker;
    }

    return nullptr;  // not found
}

std::shared_ptr<ChMarker> ChAssembly::SearchMarker(int id) const {
    // Iterate over all bodies and search in the body's marker list
    for (auto& body : bodylist) {
        if (auto marker = body->SearchMarker(id))
            return marker;
    }

    return nullptr;  // not found
}

// -----------------------------------------------------------------------------

void ChAssembly::SetSystem(ChSystem* m_system) {
    system = m_system;

    for (auto& body : bodylist) {
        body->SetSystem(m_system);
    }
    for (auto& shaft : shaftlist) {
        shaft->SetSystem(m_system);
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

void ChAssembly::AddCollisionModelsToSystem(ChCollisionSystem* coll_sys) const {
    for (const auto& body : bodylist)
        body->AddCollisionModelsToSystem(coll_sys);

    for (const auto& mesh : meshlist)
        mesh->AddCollisionModelsToSystem(coll_sys);

    for (const auto& item : otherphysicslist) {
        if (auto a = std::dynamic_pointer_cast<ChAssembly>(item)) {
            a->AddCollisionModelsToSystem(coll_sys);
            continue;
        }

        item->AddCollisionModelsToSystem(coll_sys);
    }
}

void ChAssembly::RemoveCollisionModelsFromSystem(ChCollisionSystem* coll_sys) const {
    for (const auto& body : bodylist)
        body->RemoveCollisionModelsFromSystem(coll_sys);

    for (const auto& mesh : meshlist)
        mesh->RemoveCollisionModelsFromSystem(coll_sys);

    for (const auto& item : otherphysicslist) {
        if (auto a = std::dynamic_pointer_cast<ChAssembly>(item)) {
            a->RemoveCollisionModelsFromSystem(coll_sys);
            continue;
        }

        item->RemoveCollisionModelsFromSystem(coll_sys);
    }
}

void ChAssembly::SyncCollisionModels() {
    for (auto& body : bodylist) {
        body->SyncCollisionModels();
    }
    for (auto& shaft : shaftlist) {
        shaft->SyncCollisionModels();
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

void ChAssembly::SetupInitial() {
    for (auto& body : bodylist) {
        body->SetupInitial();
    }
    for (auto& shaft : shaftlist) {
        shaft->SetupInitial();
    }
    for (auto& link : linklist) {
        link->SetupInitial();
    }
    for (auto& mesh : meshlist) {
        mesh->SetupInitial();
    }
    for (auto& otherphysics : otherphysicslist) {
        otherphysics->SetupInitial();
    }
}

// Count all bodies, links, meshes, and other physics items.
// Set counters (DOF, num constraints, etc) and offsets.
void ChAssembly::Setup() {
    m_num_bodies_active = 0;
    m_num_bodies_sleep = 0;
    m_num_bodies_fixed = 0;
    m_num_shafts = 0;
    m_num_shafts_sleep = 0;
    m_num_shafts_fixed = 0;
    m_num_coords_pos = 0;
    m_num_coords_vel = 0;
    m_num_constr = 0;
    m_num_constr_bil = 0;
    m_num_constr_uni = 0;
    m_num_links_active = 0;
    m_num_meshes = 0;
    m_num_otherphysicsitems_active = 0;

    // Add any items queued for insertion in the assembly's lists.
    this->FlushBatch();

    for (auto& body : bodylist) {
        if (body->IsFixed())
            m_num_bodies_fixed++;
        else if (body->IsSleeping())
            m_num_bodies_sleep++;
        else {
            m_num_bodies_active++;

            body->SetOffset_x(this->offset_x + m_num_coords_pos);
            body->SetOffset_w(this->offset_w + m_num_coords_vel);
            body->SetOffset_L(this->offset_L + m_num_constr);

            body->Setup();  // currently, no-op

            m_num_coords_pos += body->GetNumCoordsPosLevel();
            m_num_coords_vel += body->GetNumCoordsVelLevel();
            m_num_constr += body->GetNumConstraints();  // not really needed since ChBody introduces no constraints
            m_num_constr_bil +=
                body->GetNumConstraintsBilateral();  // not really needed since ChBody introduces no constraints
            m_num_constr_uni +=
                body->GetNumConstraintsUnilateral();  // not really needed since ChBody introduces no constraints
        }
    }

    for (auto& shaft : shaftlist) {
        if (shaft->IsFixed())
            m_num_shafts_fixed++;
        else if (shaft->IsSleeping())
            m_num_shafts_sleep++;
        else {
            m_num_shafts++;

            shaft->SetOffset_x(this->offset_x + m_num_coords_pos);
            shaft->SetOffset_w(this->offset_w + m_num_coords_vel);
            shaft->SetOffset_L(this->offset_L + m_num_constr);

            shaft->Setup();

            m_num_coords_pos += shaft->GetNumCoordsPosLevel();
            m_num_coords_vel += shaft->GetNumCoordsVelLevel();
            m_num_constr += shaft->GetNumConstraints();
            m_num_constr_bil += shaft->GetNumConstraintsBilateral();
            m_num_constr_uni += shaft->GetNumConstraintsUnilateral();
        }
    }

    for (auto& link : linklist) {
        if (link->IsActive()) {
            m_num_links_active++;

            link->SetOffset_x(this->offset_x + m_num_coords_pos);
            link->SetOffset_w(this->offset_w + m_num_coords_vel);
            link->SetOffset_L(this->offset_L + m_num_constr);

            link->Setup();  // compute DOFs etc. and sets the offsets also in child items, if any

            m_num_coords_pos += link->GetNumCoordsPosLevel();
            m_num_coords_vel += link->GetNumCoordsVelLevel();
            m_num_constr += link->GetNumConstraints();
            m_num_constr_bil += link->GetNumConstraintsBilateral();
            m_num_constr_uni += link->GetNumConstraintsUnilateral();
        }
    }

    for (auto& mesh : meshlist) {
        m_num_meshes++;

        mesh->SetOffset_x(this->offset_x + m_num_coords_pos);
        mesh->SetOffset_w(this->offset_w + m_num_coords_vel);
        mesh->SetOffset_L(this->offset_L + m_num_constr);

        mesh->Setup();  // compute DOFs and iteratively call Setup for child items

        m_num_coords_pos += mesh->GetNumCoordsPosLevel();
        m_num_coords_vel += mesh->GetNumCoordsVelLevel();
        m_num_constr += mesh->GetNumConstraints();
        m_num_constr_bil += mesh->GetNumConstraintsBilateral();
        m_num_constr_uni += mesh->GetNumConstraintsUnilateral();
    }

    for (auto& item : otherphysicslist) {
        if (item->IsActive()) {
            m_num_otherphysicsitems_active++;

            item->SetOffset_x(this->offset_x + m_num_coords_pos);
            item->SetOffset_w(this->offset_w + m_num_coords_vel);
            item->SetOffset_L(this->offset_L + m_num_constr);

            item->Setup();

            m_num_coords_pos += item->GetNumCoordsPosLevel();
            m_num_coords_vel += item->GetNumCoordsVelLevel();
            m_num_constr += item->GetNumConstraints();
            m_num_constr_bil += item->GetNumConstraintsBilateral();
            m_num_constr_uni += item->GetNumConstraintsUnilateral();
        }
    }
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
    for (auto& body : bodylist) {
        body->Update(ChTime, update_assets);
    }
    for (auto& shaft : shaftlist) {
        shaft->Update(ChTime, update_assets);
    }
    for (auto& mesh : meshlist) {
        mesh->Update(ChTime, update_assets);
    }
    for (auto& otherphysics : otherphysicslist) {
        otherphysics->Update(ChTime, update_assets);
    }
    // The state of links depends on the bodylist,shaftlist,meshlist,otherphysicslist,
    // thus the update of linklist must be at the end.
    for (auto& link : linklist) {
        link->Update(ChTime, update_assets);
    }
}

void ChAssembly::ForceToRest() {
    for (auto& body : bodylist) {
        body->ForceToRest();
    }
    for (auto& shaft : shaftlist) {
        shaft->ForceToRest();
    }
    for (auto& link : linklist) {
        link->ForceToRest();
    }
    for (auto& mesh : meshlist) {
        mesh->ForceToRest();
    }
    for (auto& item : otherphysicslist) {
        item->ForceToRest();
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
    for (auto& shaft : shaftlist) {
        if (shaft->IsActive())
            shaft->IntStateGather(displ_x + shaft->GetOffset_x(), x, displ_v + shaft->GetOffset_w(), v, T);
    }
    for (auto& link : linklist) {
        if (link->IsActive())
            link->IntStateGather(displ_x + link->GetOffset_x(), x, displ_v + link->GetOffset_w(), v, T);
    }
    for (auto& mesh : meshlist) {
        mesh->IntStateGather(displ_x + mesh->GetOffset_x(), x, displ_v + mesh->GetOffset_w(), v, T);
    }
    for (auto& item : otherphysicslist) {
        if (item->IsActive())
            item->IntStateGather(displ_x + item->GetOffset_x(), x, displ_v + item->GetOffset_w(), v, T);
    }
    T = GetChTime();
}

void ChAssembly::IntStateScatter(const unsigned int off_x,
                                 const ChState& x,
                                 const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const double T,
                                 bool full_update) {
    // Notes:
    // 1. All IntStateScatter() calls below will automatically call Update() for each object, therefore:
    //    - do not call Update() on this (assembly).
    //    - do not call ChPhysicsItem::IntStateScatter() as this will also result in a redundant Update()
    // 2. Order below is *important*
    //    - in particular, bodies and meshes must be processed *before* links, so that links can use
    //      up-to-date body and node information

    unsigned int displ_x = off_x - this->offset_x;
    unsigned int displ_v = off_v - this->offset_w;

    for (auto& body : bodylist) {
        if (body->IsActive())
            body->IntStateScatter(displ_x + body->GetOffset_x(), x, displ_v + body->GetOffset_w(), v, T, full_update);
        else
            body->Update(T, full_update);
    }
    for (auto& shaft : shaftlist) {
        if (shaft->IsActive())
            shaft->IntStateScatter(displ_x + shaft->GetOffset_x(), x, displ_v + shaft->GetOffset_w(), v, T,
                                   full_update);
        else
            shaft->Update(T, full_update);
    }
    for (auto& mesh : meshlist) {
        mesh->IntStateScatter(displ_x + mesh->GetOffset_x(), x, displ_v + mesh->GetOffset_w(), v, T, full_update);
    }
    for (auto& item : otherphysicslist) {
        if (item->IsActive())
            item->IntStateScatter(displ_x + item->GetOffset_x(), x, displ_v + item->GetOffset_w(), v, T, full_update);
        else
            item->Update(T, full_update);
    }
    // Because the Update() of ChLink() depends on the frames of Body1 and Body2, the state scatter of linklist
    // must be behind of bodylist,shaftlist,meshlist,otherphysicslist; otherwise, the Update() of ChLink() would
    // use the old (un-updated) status of bodylist,shaftlist,meshlist, resulting in a delay of Update() of ChLink()
    // for one time step, then the simulation might diverge!
    for (auto& link : linklist) {
        if (link->IsActive())
            link->IntStateScatter(displ_x + link->GetOffset_x(), x, displ_v + link->GetOffset_w(), v, T, full_update);
        else
            link->Update(T, full_update);
    }

    SetChTime(T);
}

void ChAssembly::IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    unsigned int displ_a = off_a - this->offset_w;

    for (auto& body : bodylist) {
        if (body->IsActive())
            body->IntStateGatherAcceleration(displ_a + body->GetOffset_w(), a);
    }
    for (auto& shaft : shaftlist) {
        if (shaft->IsActive())
            shaft->IntStateGatherAcceleration(displ_a + shaft->GetOffset_w(), a);
    }
    for (auto& link : linklist) {
        if (link->IsActive())
            link->IntStateGatherAcceleration(displ_a + link->GetOffset_w(), a);
    }
    for (auto& mesh : meshlist) {
        mesh->IntStateGatherAcceleration(displ_a + mesh->GetOffset_w(), a);
    }
    for (auto& item : otherphysicslist) {
        if (item->IsActive())
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
    for (auto& shaft : shaftlist) {
        if (shaft->IsActive())
            shaft->IntStateScatterAcceleration(displ_a + shaft->GetOffset_w(), a);
    }
    for (auto& mesh : meshlist) {
        mesh->IntStateScatterAcceleration(displ_a + mesh->GetOffset_w(), a);
    }
    for (auto& item : otherphysicslist) {
        if (item->IsActive())
            item->IntStateScatterAcceleration(displ_a + item->GetOffset_w(), a);
    }
    for (auto& link : linklist) {
        if (link->IsActive())
            link->IntStateScatterAcceleration(displ_a + link->GetOffset_w(), a);
    }
}

// From system to reaction forces (last computed) - some timestepper might need this
void ChAssembly::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    unsigned int displ_L = off_L - this->offset_L;

    for (auto& body : bodylist) {
        if (body->IsActive())
            body->IntStateGatherReactions(displ_L + body->GetOffset_L(), L);
    }
    for (auto& shaft : shaftlist) {
        if (shaft->IsActive())
            shaft->IntStateGatherReactions(displ_L + shaft->GetOffset_L(), L);
    }
    for (auto& link : linklist) {
        if (link->IsActive())
            link->IntStateGatherReactions(displ_L + link->GetOffset_L(), L);
    }
    for (auto& mesh : meshlist) {
        mesh->IntStateGatherReactions(displ_L + mesh->GetOffset_L(), L);
    }
    for (auto& item : otherphysicslist) {
        if (item->IsActive())
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
    for (auto& shaft : shaftlist) {
        if (shaft->IsActive())
            shaft->IntStateScatterReactions(displ_L + shaft->GetOffset_L(), L);
    }
    for (auto& mesh : meshlist) {
        mesh->IntStateScatterReactions(displ_L + mesh->GetOffset_L(), L);
    }
    for (auto& item : otherphysicslist) {
        if (item->IsActive())
            item->IntStateScatterReactions(displ_L + item->GetOffset_L(), L);
    }
    // The state scatter of reactions of link depends on Body1 and Body2, thus it must be at the end.
    for (auto& link : linklist) {
        if (link->IsActive())
            link->IntStateScatterReactions(displ_L + link->GetOffset_L(), L);
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

    for (auto& shaft : shaftlist) {
        if (shaft->IsActive())
            shaft->IntStateIncrement(displ_x + shaft->GetOffset_x(), x_new, x, displ_v + shaft->GetOffset_w(), Dv);
    }

    for (auto& link : linklist) {
        if (link->IsActive())
            link->IntStateIncrement(displ_x + link->GetOffset_x(), x_new, x, displ_v + link->GetOffset_w(), Dv);
    }

    for (auto& mesh : meshlist) {
        mesh->IntStateIncrement(displ_x + mesh->GetOffset_x(), x_new, x, displ_v + mesh->GetOffset_w(), Dv);
    }

    for (auto& item : otherphysicslist) {
        if (item->IsActive())
            item->IntStateIncrement(displ_x + item->GetOffset_x(), x_new, x, displ_v + item->GetOffset_w(), Dv);
    }
}

void ChAssembly::IntStateGetIncrement(const unsigned int off_x,
                                      const ChState& x_new,
                                      const ChState& x,
                                      const unsigned int off_v,
                                      ChStateDelta& Dv) {
    unsigned int displ_x = off_x - this->offset_x;
    unsigned int displ_v = off_v - this->offset_w;

    for (auto& body : bodylist) {
        if (body->IsActive())
            body->IntStateGetIncrement(displ_x + body->GetOffset_x(), x_new, x, displ_v + body->GetOffset_w(), Dv);
    }

    for (auto& shaft : shaftlist) {
        if (shaft->IsActive())
            shaft->IntStateGetIncrement(displ_x + shaft->GetOffset_x(), x_new, x, displ_v + shaft->GetOffset_w(), Dv);
    }

    for (auto& link : linklist) {
        if (link->IsActive())
            link->IntStateGetIncrement(displ_x + link->GetOffset_x(), x_new, x, displ_v + link->GetOffset_w(), Dv);
    }

    for (auto& mesh : meshlist) {
        mesh->IntStateGetIncrement(displ_x + mesh->GetOffset_x(), x_new, x, displ_v + mesh->GetOffset_w(), Dv);
    }

    for (auto& item : otherphysicslist) {
        if (item->IsActive())
            item->IntStateGetIncrement(displ_x + item->GetOffset_x(), x_new, x, displ_v + item->GetOffset_w(), Dv);
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
    for (auto& shaft : shaftlist) {
        if (shaft->IsActive())
            shaft->IntLoadResidual_F(displ_v + shaft->GetOffset_w(), R, c);
    }
    for (auto& link : linklist) {
        if (link->IsActive())
            link->IntLoadResidual_F(displ_v + link->GetOffset_w(), R, c);
    }
    for (auto& mesh : meshlist) {
        mesh->IntLoadResidual_F(displ_v + mesh->GetOffset_w(), R, c);
    }
    for (auto& item : otherphysicslist) {
        if (item->IsActive())
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
    for (auto& shaft : shaftlist) {
        if (shaft->IsActive())
            shaft->IntLoadResidual_Mv(displ_v + shaft->GetOffset_w(), R, w, c);
    }
    for (auto& link : linklist) {
        if (link->IsActive())
            link->IntLoadResidual_Mv(displ_v + link->GetOffset_w(), R, w, c);
    }
    for (auto& mesh : meshlist) {
        mesh->IntLoadResidual_Mv(displ_v + mesh->GetOffset_w(), R, w, c);
    }
    for (auto& item : otherphysicslist) {
        if (item->IsActive())
            item->IntLoadResidual_Mv(displ_v + item->GetOffset_w(), R, w, c);
    }
}

void ChAssembly::IntLoadLumpedMass_Md(const unsigned int off, ChVectorDynamic<>& Md, double& err, const double c) {
    unsigned int displ_v = off - this->offset_w;

    for (auto& body : bodylist) {
        if (body->IsActive())
            body->IntLoadLumpedMass_Md(displ_v + body->GetOffset_w(), Md, err, c);
    }
    for (auto& shaft : shaftlist) {
        if (shaft->IsActive())
            shaft->IntLoadLumpedMass_Md(displ_v + shaft->GetOffset_w(), Md, err, c);
    }
    for (auto& link : linklist) {
        if (link->IsActive())
            link->IntLoadLumpedMass_Md(displ_v + link->GetOffset_w(), Md, err, c);
    }
    for (auto& mesh : meshlist) {
        mesh->IntLoadLumpedMass_Md(displ_v + mesh->GetOffset_w(), Md, err, c);
    }
    for (auto& item : otherphysicslist) {
        if (item->IsActive())
            item->IntLoadLumpedMass_Md(displ_v + item->GetOffset_w(), Md, err, c);
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
    for (auto& shaft : shaftlist) {
        if (shaft->IsActive())
            shaft->IntLoadResidual_CqL(displ_L + shaft->GetOffset_L(), R, L, c);
    }
    for (auto& link : linklist) {
        if (link->IsActive())
            link->IntLoadResidual_CqL(displ_L + link->GetOffset_L(), R, L, c);
    }
    for (auto& mesh : meshlist) {
        mesh->IntLoadResidual_CqL(displ_L + mesh->GetOffset_L(), R, L, c);
    }
    for (auto& item : otherphysicslist) {
        if (item->IsActive())
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
    for (auto& shaft : shaftlist) {
        if (shaft->IsActive())
            shaft->IntLoadConstraint_C(displ_L + shaft->GetOffset_L(), Qc, c, do_clamp, recovery_clamp);
    }
    for (auto& link : linklist) {
        if (link->IsActive())
            link->IntLoadConstraint_C(displ_L + link->GetOffset_L(), Qc, c, do_clamp, recovery_clamp);
    }
    for (auto& mesh : meshlist) {
        mesh->IntLoadConstraint_C(displ_L + mesh->GetOffset_L(), Qc, c, do_clamp, recovery_clamp);
    }
    for (auto& item : otherphysicslist) {
        if (item->IsActive())
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
    for (auto& shaft : shaftlist) {
        if (shaft->IsActive())
            shaft->IntLoadConstraint_Ct(displ_L + shaft->GetOffset_L(), Qc, c);
    }
    for (auto& link : linklist) {
        if (link->IsActive())
            link->IntLoadConstraint_Ct(displ_L + link->GetOffset_L(), Qc, c);
    }
    for (auto& mesh : meshlist) {
        mesh->IntLoadConstraint_Ct(displ_L + mesh->GetOffset_L(), Qc, c);
    }
    for (auto& item : otherphysicslist) {
        if (item->IsActive())
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

    for (auto& shaft : shaftlist) {
        if (shaft->IsActive())
            shaft->IntToDescriptor(displ_v + shaft->GetOffset_w(), v, R, displ_L + shaft->GetOffset_L(), L, Qc);
    }

    for (auto& link : linklist) {
        if (link->IsActive())
            link->IntToDescriptor(displ_v + link->GetOffset_w(), v, R, displ_L + link->GetOffset_L(), L, Qc);
    }

    for (auto& mesh : meshlist) {
        mesh->IntToDescriptor(displ_v + mesh->GetOffset_w(), v, R, displ_L + mesh->GetOffset_L(), L, Qc);
    }

    for (auto& item : otherphysicslist) {
        if (item->IsActive())
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

    for (auto& shaft : shaftlist) {
        if (shaft->IsActive())
            shaft->IntFromDescriptor(displ_v + shaft->GetOffset_w(), v, displ_L + shaft->GetOffset_L(), L);
    }

    for (auto& link : linklist) {
        if (link->IsActive())
            link->IntFromDescriptor(displ_v + link->GetOffset_w(), v, displ_L + link->GetOffset_L(), L);
    }

    for (auto& mesh : meshlist) {
        mesh->IntFromDescriptor(displ_v + mesh->GetOffset_w(), v, displ_L + mesh->GetOffset_L(), L);
    }

    for (auto& item : otherphysicslist) {
        if (item->IsActive())
            item->IntFromDescriptor(displ_v + item->GetOffset_w(), v, displ_L + item->GetOffset_L(), L);
    }
}

// -----------------------------------------------------------------------------

void ChAssembly::InjectVariables(ChSystemDescriptor& descriptor) {
    for (auto& body : bodylist) {
        body->InjectVariables(descriptor);
    }
    for (auto& shaft : shaftlist) {
        shaft->InjectVariables(descriptor);
    }
    for (auto& link : linklist) {
        link->InjectVariables(descriptor);
    }
    for (auto& mesh : meshlist) {
        mesh->InjectVariables(descriptor);
    }
    for (auto& item : otherphysicslist) {
        item->InjectVariables(descriptor);
    }
}

void ChAssembly::VariablesFbReset() {
    for (auto& body : bodylist) {
        body->VariablesFbReset();
    }
    for (auto& shaft : shaftlist) {
        shaft->VariablesFbReset();
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
    for (auto& shaft : shaftlist) {
        shaft->VariablesFbLoadForces(factor);
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
    for (auto& shaft : shaftlist) {
        shaft->VariablesFbIncrementMq();
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
    for (auto& shaft : shaftlist) {
        shaft->VariablesQbLoadSpeed();
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
    for (auto& shaft : shaftlist) {
        shaft->VariablesQbSetSpeed(step);
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
    for (auto& shaft : shaftlist) {
        shaft->VariablesQbIncrementPosition(dt_step);
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

void ChAssembly::InjectConstraints(ChSystemDescriptor& descriptor) {
    for (auto& body : bodylist) {
        body->InjectConstraints(descriptor);
    }
    for (auto& shaft : shaftlist) {
        shaft->InjectConstraints(descriptor);
    }
    for (auto& link : linklist) {
        link->InjectConstraints(descriptor);
    }
    for (auto& mesh : meshlist) {
        mesh->InjectConstraints(descriptor);
    }
    for (auto& item : otherphysicslist) {
        item->InjectConstraints(descriptor);
    }
}

void ChAssembly::ConstraintsBiReset() {
    for (auto& body : bodylist) {
        body->ConstraintsBiReset();
    }
    for (auto& shaft : shaftlist) {
        shaft->ConstraintsBiReset();
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
    for (auto& shaft : shaftlist) {
        shaft->ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);
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
    for (auto& shaft : shaftlist) {
        shaft->ConstraintsBiLoad_Ct(factor);
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
    for (auto& shaft : shaftlist) {
        shaft->ConstraintsBiLoad_Qc(factor);
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
    for (auto& shaft : shaftlist) {
        shaft->ConstraintsFbLoadForces(factor);
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

void ChAssembly::LoadConstraintJacobians() {
    for (auto& body : bodylist) {
        body->LoadConstraintJacobians();
    }
    for (auto& shaft : shaftlist) {
        shaft->LoadConstraintJacobians();
    }
    for (auto& link : linklist) {
        link->LoadConstraintJacobians();
    }
    for (auto& mesh : meshlist) {
        mesh->LoadConstraintJacobians();
    }
    for (auto& item : otherphysicslist) {
        item->LoadConstraintJacobians();
    }
}

void ChAssembly::ConstraintsFetch_react(double factor) {
    for (auto& body : bodylist) {
        body->ConstraintsFetch_react(factor);
    }
    for (auto& shaft : shaftlist) {
        shaft->ConstraintsFetch_react(factor);
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

void ChAssembly::InjectKRMMatrices(ChSystemDescriptor& descriptor) {
    for (auto& body : bodylist) {
        body->InjectKRMMatrices(descriptor);
    }
    for (auto& shaft : shaftlist) {
        shaft->InjectKRMMatrices(descriptor);
    }
    for (auto& link : linklist) {
        link->InjectKRMMatrices(descriptor);
    }
    for (auto& mesh : meshlist) {
        mesh->InjectKRMMatrices(descriptor);
    }
    for (auto& item : otherphysicslist) {
        item->InjectKRMMatrices(descriptor);
    }
}

void ChAssembly::LoadKRMMatrices(double Kfactor, double Rfactor, double Mfactor) {
    for (auto& body : bodylist) {
        body->LoadKRMMatrices(Kfactor, Rfactor, Mfactor);
    }
    for (auto& shaft : shaftlist) {
        shaft->LoadKRMMatrices(Kfactor, Rfactor, Mfactor);
    }
    for (auto& link : linklist) {
        link->LoadKRMMatrices(Kfactor, Rfactor, Mfactor);
    }
    for (auto& mesh : meshlist) {
        mesh->LoadKRMMatrices(Kfactor, Rfactor, Mfactor);
    }
    for (auto& item : otherphysicslist) {
        item->LoadKRMMatrices(Kfactor, Rfactor, Mfactor);
    }
}

// -----------------------------------------------------------------------------
//  STREAMING - FILE HANDLING

void ChAssembly::ShowHierarchy(std::ostream& outstream, int level) const {
    std::string mtabs;
    for (int i = 0; i < level; ++i)
        mtabs += "  ";

    outstream << "\n" << mtabs << "List of the " << (int)bodylist.size() << " added rigid bodies:" << std::endl;
    for (auto& body : bodylist) {
        outstream << mtabs << "  BODY:       " << body->GetIdentifier() << " " << body->GetName() << std::endl;

        for (auto& marker : body->GetMarkers()) {
            outstream << mtabs << "    MARKER:   " << marker->GetIdentifier() << " " << marker->GetName() << std::endl;
        }

        for (auto& force : body->GetForces()) {
            outstream << mtabs << "    FORCE:    " << force->GetIdentifier() << " " << force->GetName() << std::endl;
        }
    }

    outstream << "\n" << mtabs << "List of the " << (int)shaftlist.size() << " added shafts:" << std::endl;
    for (auto& shaft : shaftlist) {
        outstream << mtabs << "  SHAFT:      " << shaft->GetIdentifier() << " " << shaft->GetName() << std::endl;
    }

    outstream << "\n" << mtabs << "List of the " << (int)linklist.size() << " added links:" << std::endl;
    for (auto& link : linklist) {
        auto& rlink = *link.get();
        outstream << mtabs << "  LINK:       " << link->GetIdentifier() << " " << link->GetName() << " ["
                  << typeid(rlink).name() << "]" << std::endl;
        if (auto malink = std::dynamic_pointer_cast<ChLinkMarkers>(link)) {
            if (malink->GetMarker1())
                outstream << mtabs << "    marker1:  " << malink->GetMarker1()->GetIdentifier() << " "
                          << malink->GetMarker1()->GetName() << std::endl;
            if (malink->GetMarker2())
                outstream << mtabs << "    marker2:  " << malink->GetMarker2()->GetIdentifier() << " "
                          << malink->GetMarker2()->GetName() << std::endl;
        }
    }

    outstream << "\n" << mtabs << "List of the " << (int)meshlist.size() << " added meshes:" << std::endl;
    for (auto& mesh : meshlist) {
        outstream << mtabs << "  MESH :      " << mesh->GetIdentifier() << " " << mesh->GetName() << std::endl;
    }

    outstream << "\n"
              << mtabs << "List of other " << (int)otherphysicslist.size() << " added physic items:" << std::endl;
    for (auto& item : otherphysicslist) {
        outstream << mtabs << "  PHYSICS ITEM: " << item->GetIdentifier() << " " << item->GetName() << " ["
                  << typeid(item.get()).name() << "]" << std::endl;

        // recursion:
        if (auto assem = std::dynamic_pointer_cast<ChAssembly>(item))
            assem->ShowHierarchy(outstream, level + 1);
    }

    outstream << std::endl;
}

void ChAssembly::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChAssembly>();

    // serialize parent class
    ChPhysicsItem::ArchiveOut(archive_out);

    // serialize all member data:

    archive_out << CHNVP(bodylist, "bodies");
    archive_out << CHNVP(shaftlist, "shafts");
    archive_out << CHNVP(linklist, "links");
    archive_out << CHNVP(meshlist, "meshes");
    archive_out << CHNVP(otherphysicslist, "other_physics_items");
}

void ChAssembly::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChAssembly>();

    // deserialize parent class
    ChPhysicsItem::ArchiveIn(archive_in);

    // stream in all member data:
    std::vector<std::shared_ptr<ChBody>> tempbodies;
    std::vector<std::shared_ptr<ChShaft>> tempshafts;
    std::vector<std::shared_ptr<ChLinkBase>> templinks;
    std::vector<std::shared_ptr<ChMesh>> tempmeshes;
    std::vector<std::shared_ptr<ChPhysicsItem>> tempitems;
    archive_in >> CHNVP(tempbodies, "bodies");
    archive_in >> CHNVP(tempshafts, "shafts");
    archive_in >> CHNVP(templinks, "links");
    archive_in >> CHNVP(tempmeshes, "meshes");
    archive_in >> CHNVP(tempitems, "other_physics_items");
    // trick needed because the "Add...()" functions are required
    RemoveAllBodies();
    for (auto& body : tempbodies) {
        AddBody(body);
    }
    RemoveAllShafts();
    for (auto& shaft : tempshafts) {
        AddShaft(shaft);
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
