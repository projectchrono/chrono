//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011-2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//


#include <stdlib.h>
#include <algorithm>

#include "core/ChTransform.h"
#include "physics/ChAssembly.h"
#include "physics/ChSystem.h"
#include "physics/ChGlobal.h"
#include "physics/ChBodyAuxRef.h"
#include "core/ChLinearAlgebra.h"

namespace chrono {

using namespace collision;
using namespace geometry;

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChAssembly> a_registration_ChAssembly;


ChAssembly::ChAssembly() {
    linklist.clear();
    bodylist.clear();
    otherphysicslist.clear();
    
    nbodies = 0;
    nlinks = 0;
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

    ChTime = 0;
}

ChAssembly::~ChAssembly() {
    RemoveAllBodies();
    RemoveAllLinks();
    RemoveAllOtherPhysicsItems();
}

void ChAssembly::Copy(ChAssembly* source) {
    // first copy the parent class data...
    ChPhysicsItem::Copy(source);

    this->Clear();
    /*
    //***TO DO*** deeper copy 
    for (unsigned int ip = 0; ip < source->Get_bodylist()->size(); ++ip)  // ITERATE on bodies
        this->Add(source->Get_bodylist()->at(ip)->Clone());

    for (unsigned int ip = 0; ip < source->Get_linklist()->size(); ++ip)  // ITERATE on bodies
        this->Add(source->Get_linklist()->at(ip)->Clone());

    for (unsigned int ip = 0; ip < source->Get_otherphysicslist()->size(); ++ip)  // ITERATE on bodies
        this->Add(source->Get_otherphysicslist()->at(ip)->Clone());
    */
    nbodies = source->GetNbodies();
    nlinks = source->GetNlinks();
    nphysicsitems = source->GetNphysicsItems();
    ncoords = source->GetNcoords();
    ncoords_w = source->GetNcoords_w();
    ndoc = source->GetNdoc();
    ndoc_w = source->GetNdoc_w();
    ndoc_w_C = source->GetNdoc_w_C();
    ndoc_w_D = source->GetNdoc_w_D();
    ndof = source->GetNdof();
    nsysvars = source->GetNsysvars();
    nsysvars_w = source->GetNsysvars_w();
    nbodies_sleep = source->GetNbodiesSleeping();
    nbodies_fixed = source->GetNbodiesFixed();
}

void ChAssembly::Clear() {

    RemoveAllLinks();
    RemoveAllBodies();
    RemoveAllOtherPhysicsItems();

    nbodies = 0;
    nlinks = 0;
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

void ChAssembly::AddBody(ChSharedPtr<ChBody> newbody) {
    assert(std::find<std::vector<ChSharedPtr<ChBody> >::iterator>(bodylist.begin(), bodylist.end(), newbody) ==
           bodylist.end());
    assert(newbody->GetSystem() == 0);  // should remove from other system before adding here

    // set system and also add collision models to system
    newbody->SetSystem(this->GetSystem());
    bodylist.push_back(newbody);
}

void ChAssembly::RemoveBody(ChSharedPtr<ChBody> mbody) {
    assert(std::find<std::vector< ChSharedPtr<ChBody> >::iterator>(bodylist.begin(), bodylist.end(), mbody) != bodylist.end());

    // warning! linear time search, to erase pointer from container.
    bodylist.erase(std::find<std::vector< ChSharedPtr<ChBody> >::iterator>(bodylist.begin(), bodylist.end(), mbody));

    // nullify backward link to system and also remove from collision system
    mbody->SetSystem(0);
}

void ChAssembly::AddLink(ChSharedPtr<ChLink> newlink) {
    assert(std::find<std::vector< ChSharedPtr<ChLink> >::iterator>(linklist.begin(), linklist.end(), newlink) == linklist.end());

    newlink->SetSystem(this->GetSystem());
    linklist.push_back(newlink);
}


void ChAssembly::RemoveLink(ChSharedPtr<ChLink> mlink) {
    assert(std::find<std::vector<ChSharedPtr<ChLink> >::iterator>(linklist.begin(), linklist.end(), mlink) !=
           linklist.end());

    // warning! linear time search, to erase pointer from container!
    linklist.erase(std::find<std::vector<ChSharedPtr<ChLink> >::iterator>(linklist.begin(), linklist.end(), mlink));

    // nullify backward link to system
    mlink->SetSystem(0);
}

void ChAssembly::AddOtherPhysicsItem(ChSharedPtr<ChPhysicsItem> newitem) {
    assert(std::find<std::vector<ChSharedPtr<ChPhysicsItem> >::iterator>(otherphysicslist.begin(), otherphysicslist.end(),
                                                            newitem) == otherphysicslist.end());
    // assert(newitem->GetSystem()==0); // should remove from other system before adding here

    // set system and also add collision models to system
    newitem->SetSystem(this->GetSystem());
    otherphysicslist.push_back(newitem);
}

void ChAssembly::RemoveOtherPhysicsItem(ChSharedPtr<ChPhysicsItem> mitem) {
    assert(std::find<std::vector<ChSharedPtr<ChPhysicsItem> >::iterator>(otherphysicslist.begin(), otherphysicslist.end(),
                                                            mitem) != otherphysicslist.end());

    // warning! linear time search, to erase pointer from container.
    otherphysicslist.erase(std::find<std::vector<ChSharedPtr<ChPhysicsItem> >::iterator>(otherphysicslist.begin(),                                                                         otherphysicslist.end(), mitem));

    // nullify backward link to system and also remove from collision system
    mitem->SetSystem(0);
}

void ChAssembly::Add(ChSharedPtr<ChPhysicsItem> newitem) {
    if (newitem.IsType<ChBody>()) // (typeid(*newitem.get_ptr())==typeid(ChBody)) // if (newitem.IsType<ChBody>()) sends ChBody descendants in ChBody list: this is bad for ChConveyor
    {
        AddBody(newitem.DynamicCastTo<ChBody>());
    } else if (newitem.IsType<ChLink>()) {
        AddLink(newitem.DynamicCastTo<ChLink>());
    } else
        AddOtherPhysicsItem(newitem);
}

void ChAssembly::AddBatch(ChSharedPtr<ChPhysicsItem> newitem) {
    this->batch_to_insert.push_back(newitem);
    // newitem->SetSystem(this->GetSystem());
}

void ChAssembly::FlushBatch() {
    for (int i=0; i<  this->batch_to_insert.size(); ++i) {
        //batch_to_insert[i]->SetSystem(0);
        this->Add(batch_to_insert[i]);
    }
    batch_to_insert.clear();
}

void ChAssembly::Remove(ChSharedPtr<ChPhysicsItem> newitem) {
    if (newitem.IsType<ChBody>()) // (typeid(*newitem.get_ptr())==typeid(ChBody)) // if (newitem.IsType<ChBody>()) sends ChBody descendants in ChBody list: this is bad for ChConveyor
    {
        RemoveBody(newitem.DynamicCastTo<ChBody>());
    } else if (newitem.IsType<ChLink>()) {
        RemoveLink(newitem.DynamicCastTo<ChLink>());
    } else
        RemoveOtherPhysicsItem(newitem);
}

void ChAssembly::RemoveAllBodies() {
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
    {
        ChSharedPtr<ChBody> Bpointer = bodylist[ip];

        // nullify backward link to system and also remove from collision system
        Bpointer->SetSystem(0);
    }
    bodylist.clear();
}

void ChAssembly::RemoveAllLinks() {
    for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
    {
        // nullify backward link to system
        linklist[ip]->SetSystem(0);
    }
    linklist.clear();
}

void ChAssembly::RemoveAllOtherPhysicsItems() {
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChSharedPtr<ChPhysicsItem> PHpointer = otherphysicslist[ip];

        // nullify backward link to system and also remove from collision system
        PHpointer->SetSystem(0);
    }
    otherphysicslist.clear();
}

ChSharedPtr<ChBody> ChAssembly::SearchBody(const char* m_name) {
    return ChContainerSearchFromName<ChSharedPtr<ChBody>, std::vector<ChSharedPtr<ChBody> >::iterator>(m_name, bodylist.begin(), bodylist.end());
}

ChSharedPtr<ChLink> ChAssembly::SearchLink(const char* m_name) {
    return ChContainerSearchFromName<ChSharedPtr<ChLink>, std::vector< ChSharedPtr<ChLink> >::iterator>(m_name, linklist.begin(), linklist.end());
}

ChSharedPtr<ChPhysicsItem> ChAssembly::SearchOtherPhysicsItem(const char* m_name) {
    return ChContainerSearchFromName<ChSharedPtr<ChPhysicsItem>, std::vector< ChSharedPtr<ChPhysicsItem> >::iterator>(
        m_name, otherphysicslist.begin(), otherphysicslist.end());
}

ChSharedPtr<ChPhysicsItem> ChAssembly::Search(const char* m_name) {
    ChSharedPtr<ChBody> mbo = SearchBody(m_name);
    if (!mbo.IsNull())
        return mbo;
    ChSharedPtr<ChLink> mli = SearchLink(m_name);
    if (!mli.IsNull())
        return mli;
    ChSharedPtr<ChPhysicsItem> mph = SearchOtherPhysicsItem(m_name);
    if (!mph.IsNull())
        return mph;
    return (ChSharedPtr<ChPhysicsItem>());  // not found? return a void shared ptr.
}

ChSharedPtr<ChMarker> ChAssembly::SearchMarker(const char* m_name) {
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
    {
        ChSharedPtr<ChBody> Bpointer = bodylist[ip];

        ChSharedPtr<ChMarker> mmark = Bpointer->SearchMarker(m_name);
        if (!mmark.IsNull())
            return mmark;
    }

    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChSharedPtr<ChPhysicsItem> PHpointer = otherphysicslist[ip];

        if (ChSharedPtr<ChBodyAuxRef> mbodyauxref = PHpointer.DynamicCastTo<ChBodyAuxRef>()) {
            ChSharedPtr<ChMarker> mmark = mbodyauxref->SearchMarker(m_name);
            if (!mmark.IsNull())
                return mmark;
        }
    }

    return (ChSharedPtr<ChMarker>());  // not found? return a void shared ptr.
}

ChSharedPtr<ChMarker> ChAssembly::SearchMarker(int markID) {

    ChSharedPtr<ChMarker> res;

    for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
    {
        ChSharedPtr<ChBody> Bpointer = bodylist[ip];

        res = ChContainerSearchFromID<ChSharedPtr<ChMarker>, std::vector<ChSharedPtr<ChMarker>>::const_iterator>(
            markID, Bpointer->GetMarkerList().begin(), Bpointer->GetMarkerList().end());
        if (res) {
            return res;
        }
    }

    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChSharedPtr<ChPhysicsItem> PHpointer = otherphysicslist[ip];

        if (ChSharedPtr<ChBodyAuxRef> mbodyauxref = PHpointer.DynamicCastTo<ChBodyAuxRef>()) {
            res = ChContainerSearchFromID<ChSharedPtr<ChMarker>, std::vector<ChSharedPtr<ChMarker>>::const_iterator>(
                markID, mbodyauxref->GetMarkerList().begin(), mbodyauxref->GetMarkerList().end());
            if (res) {
                return res;
            }
        }
    }

    return (ChSharedPtr<ChMarker>());  // not found? return a void shared ptr.
}


//////////////////////////////////////////////////////////////////

ChAssembly::IteratorBodies& ChAssembly::IteratorBodies::operator=(const IteratorBodies& other) {
    node_ = other.node_;
    return (*this);
}
bool ChAssembly::IteratorBodies::operator==(const IteratorBodies& other) {
    return (node_ == other.node_);
}
bool ChAssembly::IteratorBodies::operator!=(const IteratorBodies& other) {
    return (node_ != other.node_);
}
ChAssembly::IteratorBodies& ChAssembly::IteratorBodies::operator++() {
    node_++;
    return (*this);
}
ChSharedPtr<ChBody> ChAssembly::IteratorBodies::operator*() {
    return (*node_);  // .. here I am not getting a new() data, but a reference to something created elsewhere
}
ChAssembly::IteratorBodies ChAssembly::IterBeginBodies() {
    return (IteratorBodies(this->bodylist.begin()));
}
ChAssembly::IteratorBodies ChAssembly::IterEndBodies() {
    return (IteratorBodies(this->bodylist.end()));
}

//////////////////////////////////////////////////////////////////

ChAssembly::IteratorLinks& ChAssembly::IteratorLinks::operator=(const IteratorLinks& other) {
    node_ = other.node_;
    return (*this);
}
bool ChAssembly::IteratorLinks::operator==(const IteratorLinks& other) {
    return (node_ == other.node_);
}
bool ChAssembly::IteratorLinks::operator!=(const IteratorLinks& other) {
    return (node_ != other.node_);
}
ChAssembly::IteratorLinks& ChAssembly::IteratorLinks::operator++() {
    node_++;
    return (*this);
}

ChSharedPtr<ChLink> ChAssembly::IteratorLinks::operator*() {
    return (*node_);
}
ChAssembly::IteratorLinks ChAssembly::IterBeginLinks() {
    return (IteratorLinks(this->linklist.begin()));
}
ChAssembly::IteratorLinks ChAssembly::IterEndLinks() {
    return (IteratorLinks(this->linklist.end()));
}

//////////////////////////////////////////////////////////////////

ChAssembly::IteratorOtherPhysicsItems& ChAssembly::IteratorOtherPhysicsItems::operator=(
    const IteratorOtherPhysicsItems& other) {
    node_ = other.node_;
    return (*this);
}
bool ChAssembly::IteratorOtherPhysicsItems::operator==(const IteratorOtherPhysicsItems& other) {
    return (node_ == other.node_);
}
bool ChAssembly::IteratorOtherPhysicsItems::operator!=(const IteratorOtherPhysicsItems& other) {
    return (node_ != other.node_);
}
ChAssembly::IteratorOtherPhysicsItems& ChAssembly::IteratorOtherPhysicsItems::operator++() {
    node_++;
    return (*this);
}
ChSharedPtr<ChPhysicsItem> ChAssembly::IteratorOtherPhysicsItems::operator*() {
    return (*node_);  // .. here I am not getting a new() data, but a reference to something created elsewhere
}
ChAssembly::IteratorOtherPhysicsItems ChAssembly::IterBeginOtherPhysicsItems() {
    return (IteratorOtherPhysicsItems(this->otherphysicslist.begin()));
}
ChAssembly::IteratorOtherPhysicsItems ChAssembly::IterEndOtherPhysicsItems() {
    return (IteratorOtherPhysicsItems(this->otherphysicslist.end()));
}

//////////////////////////////////////////////////////////////////

ChAssembly::IteratorPhysicsItems::IteratorPhysicsItems(ChAssembly* msys) {
    this->msystem = msys;
    // RewindToBegin();
    node_body = msystem->Get_bodylist()->begin();
    node_link = msystem->Get_linklist()->begin();
    node_otherphysics = msystem->Get_otherphysicslist()->begin();
    stage = 0;
    mptr = ChSharedPtr<ChPhysicsItem>(0);
    this->operator++();  // initialize with 1st available item
}
ChAssembly::IteratorPhysicsItems::IteratorPhysicsItems() {
    this->msystem = 0;
    this->mptr = ChSharedPtr<ChPhysicsItem>(0);
    this->stage = 9999;
}

ChAssembly::IteratorPhysicsItems::~IteratorPhysicsItems() {
}
/*
void ChAssembly::IteratorPhysicsItems::RewindToBegin()
{
  node_body   = msystem->Get_bodylist()->begin();
  node_link	  = msystem->Get_linklist()->begin();
  node_otherphysics	 = msystem->Get_otherphysicslist()->begin();
  stage = 0;
  mptr = ChSharedPtr<ChPhysicsItem>(0);
  this->operator++(); // initialize with 1st available item
}

bool ChAssembly::IteratorPhysicsItems::ReachedEnd()
{
  if (stage == 9999)
      return true;
  return false;
}
*/
bool ChAssembly::IteratorPhysicsItems::HasItem() {
    if (mptr)
        return true;
    return false;
}

ChAssembly::IteratorPhysicsItems& ChAssembly::IteratorPhysicsItems::operator=(const ChSystem::IteratorPhysicsItems& other) {
    msystem = other.msystem;
    node_body = other.node_body;
    node_link = other.node_link;
    node_otherphysics = other.node_otherphysics;
    stage = other.stage;
    mptr = other.mptr;
    return (*this);
}

bool ChAssembly::IteratorPhysicsItems::operator==(const ChAssembly::IteratorPhysicsItems& other) {
    return ((mptr.get_ptr() == other.mptr.get_ptr()) && (stage == other.stage) && (msystem == other.msystem));  //...***TO CHECK***
}

bool ChAssembly::IteratorPhysicsItems::operator!=(const ChAssembly::IteratorPhysicsItems& other) {
    return !(this->operator==(other));  //...***TO CHECK***
}

ChAssembly::IteratorPhysicsItems& ChAssembly::IteratorPhysicsItems::operator++() {
    switch (stage) {
        case 1: {
            node_body++;  // try next body
            if (node_body != msystem->Get_bodylist()->end()) {
                mptr = (*node_body);
                return (*this);
            }
            break;
        }
        case 2: {
            node_link++;  // try next link
            if (node_link != msystem->Get_linklist()->end()) {
                mptr = (*node_link);
                return (*this);
            }
            break;
        }
        case 3: {
            node_otherphysics++;  // try next otherphysics
            if (node_otherphysics != msystem->Get_otherphysicslist()->end()) {
                mptr = (*node_otherphysics);
                return (*this);
            }
            break;
        }
        default:
            break;
    }
    // Something went wrong, some list was at the end, so jump to beginning of next list
    do {
        switch (stage) {
            case 0: {
                stage = 1;
                if (node_body != msystem->Get_bodylist()->end()) {
                    mptr = (*node_body);
                    return (*this);
                }
                break;
            }
            case 1: {
                stage = 2;
                if (node_link != msystem->Get_linklist()->end()) {
                    mptr = (*node_link);
                    return (*this);
                }
                break;
            }
            case 2: {
                stage = 3;
                if (node_otherphysics != msystem->Get_otherphysicslist()->end()) {
                    mptr = (*node_otherphysics);
                    return (*this);
                }
                break;
            }
            case 3: {
                stage = 9999;
                mptr = ChSharedPtr<ChPhysicsItem>(0);
                return (*this);
            }
        }  // end cases
    } while (true);

    return (*this);
}

ChSharedPtr<ChPhysicsItem> ChAssembly::IteratorPhysicsItems::operator*() {
    return mptr;  
}
ChAssembly::IteratorPhysicsItems ChAssembly::IterBeginPhysicsItems() {
    return (IteratorPhysicsItems(this));
}
ChAssembly::IteratorPhysicsItems ChAssembly::IterEndPhysicsItems() {
    return (IteratorPhysicsItems());
}

void ChAssembly::SetSystem(ChSystem* m_system)  {  
    this->system = m_system;

    for (int ip = 0; ip < bodylist.size(); ++ip)  
    {
        bodylist[ip]->SetSystem(m_system);
    }
    for (int ip = 0; ip < linklist.size(); ++ip)  
    {
        linklist[ip]->SetSystem(m_system);
    }
    for (int ip = 0; ip < otherphysicslist.size(); ++ip)  
    {
        otherphysicslist[ip]->SetSystem(m_system);
    }
}


void ChAssembly::SyncCollisionModels() {
    for (int ip = 0; ip < bodylist.size(); ++ip) {
        bodylist[ip]->SyncCollisionModels();
    }
    for (int ip = 0; ip < linklist.size(); ++ip) {
        linklist[ip]->SyncCollisionModels();
    }
    for (int ip = 0; ip < otherphysicslist.size(); ++ip) {
        otherphysicslist[ip]->SyncCollisionModels();
    }
}

////////////////////////////////
//////
////// UPDATING ROUTINES
//////
//////

// COUNT ALL BODIES AND LINKS, ETC, COMPUTE &SET DOF FOR STATISTICS,
// ALLOCATES OR REALLOCATE BOOKKEEPING DATA/VECTORS, IF ANY

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
    nphysicsitems = 0;

    // Any item being queued for insertion in system's lists? add it.
    this->FlushBatch();

    for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
    {
        ChSharedPtr<ChBody> Bpointer = bodylist[ip];

        if (Bpointer->GetBodyFixed())
            nbodies_fixed++;
        else if (Bpointer->GetSleeping())
            nbodies_sleep++;
        else {
            nbodies++;

            Bpointer->SetOffset_x(this->offset_x + ncoords);
            Bpointer->SetOffset_w(this->offset_w + ncoords_w);
            Bpointer->SetOffset_L(this->offset_L + ndoc_w);

            // Bpointer->Setup(); // unneded since in bodies does nothing

            ncoords   += Bpointer->GetDOF();
            ncoords_w += Bpointer->GetDOF_w();
            ndoc_w    += Bpointer->GetDOC();   // unneeded since ChBody introduces no constraints
            ndoc_w_C  += Bpointer->GetDOC_c(); // unneeded since ChBody introduces no constraints
            ndoc_w_D  += Bpointer->GetDOC_d(); // unneeded since ChBody introduces no constraints
        }
    }

    ndoc += nbodies;  // add one quaternion constr. for each active body.

    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChSharedPtr<ChPhysicsItem> PHpointer = otherphysicslist[ip];

        nphysicsitems++;

        PHpointer->SetOffset_x(this->offset_x + ncoords);
        PHpointer->SetOffset_w(this->offset_w + ncoords_w);
        PHpointer->SetOffset_L(this->offset_L + ndoc_w);

        PHpointer->Setup();  // compute DOFs etc. and sets the offsets also in child items, if assembly-type or
                             // mesh-type stuff

        ncoords   += PHpointer->GetDOF();
        ncoords_w += PHpointer->GetDOF_w();
        ndoc_w    += PHpointer->GetDOC();
        ndoc_w_C  += PHpointer->GetDOC_c();
        ndoc_w_D  += PHpointer->GetDOC_d();
    }

    for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
    {
        ChSharedPtr<ChLink> Lpointer = linklist[ip];

        if (Lpointer->IsActive()) {
            nlinks++;

            Lpointer->SetOffset_x(this->offset_x + ncoords);
            Lpointer->SetOffset_w(this->offset_w + ncoords_w);
            Lpointer->SetOffset_L(this->offset_L + ndoc_w);

            Lpointer->Setup();  // compute DOFs etc. and sets the offsets also in child items, if any

            ncoords   += Lpointer->GetDOF();
            ncoords_w += Lpointer->GetDOF_w();
            ndoc_w    += Lpointer->GetDOC();
            ndoc_w_C  += Lpointer->GetDOC_c();
            ndoc_w_D  += Lpointer->GetDOC_d();
        }
    }

    ndoc = ndoc_w + nbodies;          // number of constraints including quaternion constraints.
    nsysvars = ncoords + ndoc;        // total number of variables (coordinates + lagrangian multipliers)
    nsysvars_w = ncoords_w + ndoc_w;  // total number of variables (with 6 dof per body)

    ndof = ncoords_w-ndoc_w;  // number of degrees of freedom (approximate - does not consider constr. redundancy, etc)
}

// - ALL PHYSICAL ITEMS (BODIES, LINKS,ETC.) ARE UPDATED,
//   ALSO UPDATING THEIR AUXILIARY VARIABLES (ROT.MATRICES, ETC.).
// - UPDATES ALL FORCES  (AUTOMATIC, AS CHILDREN OF BODIES)
// - UPDATES ALL MARKERS (AUTOMATIC, AS CHILDREN OF BODIES).

void ChAssembly::Update(bool update_assets) {

    // --------------------------------------
    // Updates bodies
    // --------------------------------------
    for (int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
    {
        ChSharedPtr<ChBody> Bpointer = bodylist[ip];

        Bpointer->Update(ChTime, update_assets);
    }
    // -----------------------------
    // Updates other physical items
    // -----------------------------
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChSharedPtr<ChPhysicsItem> PHpointer = otherphysicslist[ip];

        PHpointer->Update(ChTime, update_assets);
    }
    // -----------------------------
    // Updates all links
    // -----------------------------
    for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
    {
        ChSharedPtr<ChLink> Lpointer = linklist[ip];

        Lpointer->Update(ChTime, update_assets);
    }
}

void ChAssembly::SetNoSpeedNoAcceleration() {
    for (int ip = 0; ip < bodylist.size(); ++ip) {
        bodylist[ip]->SetNoSpeedNoAcceleration();
    }
    for (int ip = 0; ip < linklist.size(); ++ip) {
        linklist[ip]->SetNoSpeedNoAcceleration();
    }
    for (int ip = 0; ip < otherphysicslist.size(); ++ip) {
        otherphysicslist[ip]->SetNoSpeedNoAcceleration();
    }
}

void ChAssembly::IntStateGather(const unsigned int off_x,  ///< offset in x state vector
                                ChState& x,                ///< state vector, position part
                                const unsigned int off_v,  ///< offset in v state vector
                                ChStateDelta& v,           ///< state vector, speed part
                                double& T)                 ///< time
{
    unsigned int displ_x = off_x - this->offset_x;
    unsigned int displ_v = off_v - this->offset_w;
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  // ITERATE on bodies
    {
        ChSharedPtr<ChBody> Bpointer = bodylist[ip];
        if (Bpointer->IsActive()) 
            Bpointer->IntStateGather(displ_x + Bpointer->GetOffset_x(), x, displ_v + Bpointer->GetOffset_w(), v, T);
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
    {
        ChSharedPtr<ChLink> Lpointer = linklist[ip];
        if (Lpointer->IsActive()) 
            Lpointer->IntStateGather(displ_x + Lpointer->GetOffset_x(), x, displ_v + Lpointer->GetOffset_w(), v, T);
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics items
    {
        ChSharedPtr<ChPhysicsItem> Ppointer = otherphysicslist[ip];
        //if (Ppointer->IsActive()) 
            Ppointer->IntStateGather(displ_x + Ppointer->GetOffset_x(), x, displ_v + Ppointer->GetOffset_w(), v, T);
    }
    T = this->GetChTime();
}


void ChAssembly::IntStateScatter(const unsigned int off_x,  ///< offset in x state vector
                                 const ChState& x,          ///< state vector, position part
                                 const unsigned int off_v,  ///< offset in v state vector
                                 const ChStateDelta& v,     ///< state vector, speed part
                                 const double T)            ///< time
{
    unsigned int displ_x = off_x - this->offset_x;
    unsigned int displ_v = off_v - this->offset_w;
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  
    {
        ChSharedPtr<ChBody> Bpointer = bodylist[ip];
        if (Bpointer->IsActive()) 
            Bpointer->IntStateScatter(displ_x + Bpointer->GetOffset_x(), x, displ_v + Bpointer->GetOffset_w(), v, T);
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip)  
    {
        ChSharedPtr<ChLink> Lpointer = linklist[ip];
        if (Lpointer->IsActive()) 
            Lpointer->IntStateScatter(displ_x + Lpointer->GetOffset_x(), x, displ_v + Lpointer->GetOffset_w(), v, T);
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip)  
    {
        ChSharedPtr<ChPhysicsItem> Ppointer = otherphysicslist[ip];
        //if (Ppointer->IsActive()) 
            Ppointer->IntStateScatter(displ_x + Ppointer->GetOffset_x(), x, displ_v + Ppointer->GetOffset_w(), v, T);
    }
    this->SetChTime(T);
}


void ChAssembly::IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    unsigned int displ_a = off_a - this->offset_w;
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  
    {
        ChSharedPtr<ChBody> Bpointer = bodylist[ip];
        if (Bpointer->IsActive()) 
            Bpointer->IntStateGatherAcceleration(displ_a + Bpointer->GetOffset_w(), a);
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip)  
    {
        ChSharedPtr<ChLink> Lpointer = linklist[ip];
        if (Lpointer->IsActive()) 
            Lpointer->IntStateGatherAcceleration(displ_a + Lpointer->GetOffset_w(), a);
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip)  
    {
        ChSharedPtr<ChPhysicsItem> Ppointer = otherphysicslist[ip];
        //if (Ppointer->IsActive()) 
            Ppointer->IntStateGatherAcceleration(displ_a + Ppointer->GetOffset_w(), a);
    }
}

/// From state derivative (acceleration) to system, sometimes might be needed
void ChAssembly::IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    unsigned int displ_a = off_a - this->offset_w;
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  
    {
        ChSharedPtr<ChBody> Bpointer = bodylist[ip];
        if (Bpointer->IsActive()) 
            Bpointer->IntStateScatterAcceleration(displ_a + Bpointer->GetOffset_w(), a);
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip)  
    {
        ChSharedPtr<ChLink> Lpointer = linklist[ip];
        if (Lpointer->IsActive()) 
            Lpointer->IntStateScatterAcceleration(displ_a + Lpointer->GetOffset_w(), a);
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip)  
    {
        ChSharedPtr<ChPhysicsItem> Ppointer = otherphysicslist[ip];
        //if (Ppointer->IsActive()) 
            Ppointer->IntStateScatterAcceleration(displ_a + Ppointer->GetOffset_w(), a);
    }
}

/// From system to reaction forces (last computed) - some timestepper might need this
void ChAssembly::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    unsigned int displ_L = off_L - this->offset_L;
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  
    {
        ChSharedPtr<ChBody> Bpointer = bodylist[ip];
        if (Bpointer->IsActive()) 
            Bpointer->IntStateGatherReactions(displ_L + Bpointer->GetOffset_L(), L);
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip)  
    {
        ChSharedPtr<ChLink> Lpointer = linklist[ip];
        if (Lpointer->IsActive()) 
            Lpointer->IntStateGatherReactions(displ_L + Lpointer->GetOffset_L(), L);
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip)  
    {
        ChSharedPtr<ChPhysicsItem> Ppointer = otherphysicslist[ip];
        //if (Ppointer->IsActive()) 
            Ppointer->IntStateGatherReactions(displ_L + Ppointer->GetOffset_L(), L);
    }
}

/// From reaction forces to system, ex. store last computed reactions in ChLink objects for plotting etc.
void ChAssembly::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    unsigned int displ_L = off_L - this->offset_L;
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  
    {
        ChSharedPtr<ChBody> Bpointer = bodylist[ip];
        if (Bpointer->IsActive()) 
            Bpointer->IntStateScatterReactions(displ_L + Bpointer->GetOffset_L(), L);
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip)  
    {
        ChSharedPtr<ChLink> Lpointer = linklist[ip];
        if (Lpointer->IsActive()) 
            Lpointer->IntStateScatterReactions(displ_L + Lpointer->GetOffset_L(), L);
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip)  
    {
        ChSharedPtr<ChPhysicsItem> Ppointer = otherphysicslist[ip];
        //if (Ppointer->IsActive()) 
            Ppointer->IntStateScatterReactions(displ_L + Ppointer->GetOffset_L(), L);
    }
}

void ChAssembly::IntStateIncrement(const unsigned int off_x,  ///< offset in x state vector
                                   ChState& x_new,            ///< state vector, position part, incremented result
                                   const ChState& x,          ///< state vector, initial position part
                                   const unsigned int off_v,  ///< offset in v state vector
                                   const ChStateDelta& Dv)    ///< state vector, increment
{
    unsigned int displ_x = off_x - this->offset_x;
    unsigned int displ_v = off_v - this->offset_w;

    for (int ip = 0; ip < bodylist.size(); ++ip)  
    {
        ChSharedPtr<ChBody> Bpointer = bodylist[ip];
        if (Bpointer->IsActive()) 
            Bpointer->IntStateIncrement(displ_x + Bpointer->GetOffset_x(), x_new, x, displ_v + Bpointer->GetOffset_w(), Dv);
    }

    for (int ip = 0; ip < linklist.size(); ++ip)  
    {
        ChSharedPtr<ChLink> Lpointer = linklist[ip];
        if (Lpointer->IsActive()) 
            Lpointer->IntStateIncrement(displ_x + Lpointer->GetOffset_x(), x_new, x, displ_v + Lpointer->GetOffset_w(), Dv);
    }

    for (int ip = 0; ip < otherphysicslist.size(); ++ip)  
    {
        ChSharedPtr<ChPhysicsItem> Ppointer = otherphysicslist[ip];
        //if (Ppointer->IsActive()) 
            Ppointer->IntStateIncrement(displ_x + Ppointer->GetOffset_x(), x_new, x, displ_v + Ppointer->GetOffset_w(), Dv);
    }
}

void ChAssembly::IntLoadResidual_F(const unsigned int off,  ///< offset in R residual
                                   ChVectorDynamic<>& R,    ///< result: the R residual, R += c*F
                                   const double c           ///< a scaling factor
                                   ) {
    unsigned int displ_v = off  - this->offset_w;
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  
    {
        ChSharedPtr<ChBody> Bpointer = bodylist[ip];
        if (Bpointer->IsActive()) 
            Bpointer->IntLoadResidual_F(displ_v + Bpointer->GetOffset_w(), R, c);
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip)  
    {
        ChSharedPtr<ChLink> Lpointer = linklist[ip];
        if (Lpointer->IsActive()) 
            Lpointer->IntLoadResidual_F(displ_v + Lpointer->GetOffset_w(), R, c);
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip)  
    {
        ChSharedPtr<ChPhysicsItem> Ppointer = otherphysicslist[ip];
        //if (Ppointer->IsActive()) 
            Ppointer->IntLoadResidual_F(displ_v + Ppointer->GetOffset_w(), R, c);
    }
}

void ChAssembly::IntLoadResidual_Mv(const unsigned int off,   ///< offset in R residual
                                    ChVectorDynamic<>& R,        ///< result: the R residual, R += c*M*v
                                    const ChVectorDynamic<>& w,  ///< the w vector
                                    const double c               ///< a scaling factor
                                    ) {
    unsigned int displ_v = off  - this->offset_w;
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  
    {
        ChSharedPtr<ChBody> Bpointer = bodylist[ip];
        if (Bpointer->IsActive()) 
            Bpointer->IntLoadResidual_Mv(displ_v + Bpointer->GetOffset_w(), R, w, c);
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip)  
    {
        ChSharedPtr<ChLink> Lpointer = linklist[ip];
        if (Lpointer->IsActive()) 
            Lpointer->IntLoadResidual_Mv(displ_v + Lpointer->GetOffset_w(), R, w, c);
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip)  
    {
        ChSharedPtr<ChPhysicsItem> Ppointer = otherphysicslist[ip];
        //if (Ppointer->IsActive()) 
            Ppointer->IntLoadResidual_Mv(displ_v + Ppointer->GetOffset_w(), R, w, c);
    }
}

void ChAssembly::IntLoadResidual_CqL(const unsigned int off_L,    ///< offset in L multipliers
                                     ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
                                     const ChVectorDynamic<>& L,  ///< the L vector
                                     const double c               ///< a scaling factor
                                     ) {
    unsigned int displ_L = off_L  - this->offset_L;
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  
    {
        ChSharedPtr<ChBody> Bpointer = bodylist[ip];
        if (Bpointer->IsActive()) 
            Bpointer->IntLoadResidual_CqL(displ_L + Bpointer->GetOffset_L(), R, L, c);
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip)  
    {
        ChSharedPtr<ChLink> Lpointer = linklist[ip];
        if (Lpointer->IsActive()) 
            Lpointer->IntLoadResidual_CqL(displ_L + Lpointer->GetOffset_L(), R, L, c);
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip)  
    {
        ChSharedPtr<ChPhysicsItem> Ppointer = otherphysicslist[ip];
        //if (Ppointer->IsActive()) 
            Ppointer->IntLoadResidual_CqL(displ_L + Ppointer->GetOffset_L(), R, L, c);
    }
}

void ChAssembly::IntLoadConstraint_C(const unsigned int off_L,  ///< offset in Qc residual
                                     ChVectorDynamic<>& Qc,     ///< result: the Qc residual, Qc += c*C
                                     const double c,            ///< a scaling factor
                                     bool do_clamp,             ///< apply clamping to c*C?
                                     double recovery_clamp      ///< value for min/max clamping of c*C
                                     ) {
    unsigned int displ_L = off_L  - this->offset_L;
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  
    {
        ChSharedPtr<ChBody> Bpointer = bodylist[ip];
        if (Bpointer->IsActive()) 
            Bpointer->IntLoadConstraint_C(displ_L + Bpointer->GetOffset_L(), Qc, c, do_clamp, recovery_clamp);
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip)  
    {
        ChSharedPtr<ChLink> Lpointer = linklist[ip];
        if (Lpointer->IsActive()) 
            Lpointer->IntLoadConstraint_C(displ_L + Lpointer->GetOffset_L(), Qc, c, do_clamp, recovery_clamp);
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip)  
    {
        ChSharedPtr<ChPhysicsItem> Ppointer = otherphysicslist[ip];
        //if (Ppointer->IsActive()) 
            Ppointer->IntLoadConstraint_C(displ_L + Ppointer->GetOffset_L(), Qc, c, do_clamp, recovery_clamp);
    }
}

void ChAssembly::IntLoadConstraint_Ct(const unsigned int off_L,  ///< offset in Qc residual
                                      ChVectorDynamic<>& Qc,     ///< result: the Qc residual, Qc += c*Ct
                                      const double c             ///< a scaling factor
                                      ) {
    unsigned int displ_L = off_L  - this->offset_L;
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip)  
    {
        ChSharedPtr<ChBody> Bpointer = bodylist[ip];
        if (Bpointer->IsActive()) 
            Bpointer->IntLoadConstraint_Ct(displ_L + Bpointer->GetOffset_L(), Qc, c);
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip)  
    {
        ChSharedPtr<ChLink> Lpointer = linklist[ip];
        if (Lpointer->IsActive()) 
            Lpointer->IntLoadConstraint_Ct(displ_L + Lpointer->GetOffset_L(), Qc, c);
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip)  
    {
        ChSharedPtr<ChPhysicsItem> Ppointer = otherphysicslist[ip];
        //if (Ppointer->IsActive()) 
            Ppointer->IntLoadConstraint_Ct(displ_L + Ppointer->GetOffset_L(), Qc, c);
    }
}

void ChAssembly::IntToLCP(const unsigned int off_v,  ///< offset in v, R
                          const ChStateDelta& v,
                          const ChVectorDynamic<>& R,
                          const unsigned int off_L,  ///< offset in L, Qc
                          const ChVectorDynamic<>& L,
                          const ChVectorDynamic<>& Qc) {
    unsigned int displ_L = off_L  - this->offset_L;
    unsigned int displ_v = off_v  - this->offset_w;
    
    for (int ip = 0; ip < bodylist.size(); ++ip)  
    {
        ChSharedPtr<ChBody> Bpointer = bodylist[ip];
        if (Bpointer->IsActive()) 
            Bpointer->IntToLCP(displ_v + Bpointer->GetOffset_w(),v,R, displ_L + Bpointer->GetOffset_L(),L,Qc);
    }

    for (int ip = 0; ip < linklist.size(); ++ip)  
    {
        ChSharedPtr<ChLink> Lpointer = linklist[ip];
        if (Lpointer->IsActive()) 
            Lpointer->IntToLCP(displ_v + Lpointer->GetOffset_w(),v,R, displ_L + Lpointer->GetOffset_L(),L,Qc);
    }

    for (int ip = 0; ip < otherphysicslist.size(); ++ip)  
    {
        ChSharedPtr<ChPhysicsItem> Ppointer = otherphysicslist[ip];
        //if (Ppointer->IsActive()) 
            Ppointer->IntToLCP(displ_v + Ppointer->GetOffset_w(),v,R, displ_L + Ppointer->GetOffset_L(),L,Qc);
    }
}

void ChAssembly::IntFromLCP(const unsigned int off_v,  ///< offset in v
                            ChStateDelta& v,
                            const unsigned int off_L,  ///< offset in L
                            ChVectorDynamic<>& L) {
    unsigned int displ_L = off_L  - this->offset_L;
    unsigned int displ_v = off_v  - this->offset_w;

    for (int ip = 0; ip < bodylist.size(); ++ip)  
    {
        ChSharedPtr<ChBody> Bpointer = bodylist[ip];
        if (Bpointer->IsActive()) 
            Bpointer->IntFromLCP(displ_v + Bpointer->GetOffset_w(),v, displ_L + Bpointer->GetOffset_L(),L);
    }

    for (int ip = 0; ip < linklist.size(); ++ip)  
    {
        ChSharedPtr<ChLink> Lpointer = linklist[ip];
        if (Lpointer->IsActive()) 
            Lpointer->IntFromLCP(displ_v + Lpointer->GetOffset_w(),v, displ_L + Lpointer->GetOffset_L(),L);
    }

    for (int ip = 0; ip < otherphysicslist.size(); ++ip)  
    {
        ChSharedPtr<ChPhysicsItem> Ppointer = otherphysicslist[ip];
        //if (Ppointer->IsActive()) 
            Ppointer->IntFromLCP(displ_v + Ppointer->GetOffset_w(),v, displ_L + Ppointer->GetOffset_L(),L);
    }
}

////
void ChAssembly::InjectVariables(ChLcpSystemDescriptor& mdescriptor) {
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip) {
        bodylist[ip]->InjectVariables(mdescriptor);
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip) {
        linklist[ip]->InjectVariables(mdescriptor);
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip) {
        otherphysicslist[ip]->InjectVariables(mdescriptor);
    }
}

void ChAssembly::VariablesFbReset() {
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip) {
        bodylist[ip]->VariablesFbReset();
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip) {
        linklist[ip]->VariablesFbReset();
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip) {
        otherphysicslist[ip]->VariablesFbReset();
    }
}

void ChAssembly::VariablesFbLoadForces(double factor) {
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip) {
        bodylist[ip]->VariablesFbLoadForces(factor);
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip) {
        linklist[ip]->VariablesFbLoadForces(factor);
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip) {
        otherphysicslist[ip]->VariablesFbLoadForces(factor);
    }
}

void ChAssembly::VariablesFbIncrementMq() {
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip) {
        bodylist[ip]->VariablesFbIncrementMq();
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip) {
        linklist[ip]->VariablesFbIncrementMq();
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip) {
        otherphysicslist[ip]->VariablesFbIncrementMq();
    }
}

void ChAssembly::VariablesQbLoadSpeed() {
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip) {
        bodylist[ip]->VariablesQbLoadSpeed();
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip) {
        linklist[ip]->VariablesQbLoadSpeed();
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip) {
        otherphysicslist[ip]->VariablesQbLoadSpeed();
    }
}

void ChAssembly::VariablesQbSetSpeed(double step) {
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip) {
        bodylist[ip]->VariablesQbSetSpeed(step);
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip) {
        linklist[ip]->VariablesQbSetSpeed(step);
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip) {
        otherphysicslist[ip]->VariablesQbSetSpeed(step);
    }
}

void ChAssembly::VariablesQbIncrementPosition(double dt_step) {
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip) {
        bodylist[ip]->VariablesQbIncrementPosition(dt_step);
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip) {
        linklist[ip]->VariablesQbIncrementPosition(dt_step);
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip) {
        otherphysicslist[ip]->VariablesQbIncrementPosition(dt_step);
    }
}

void ChAssembly::InjectConstraints(ChLcpSystemDescriptor& mdescriptor) {
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip) {
        bodylist[ip]->InjectConstraints(mdescriptor);
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip) {
        linklist[ip]->InjectConstraints(mdescriptor);
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip) {
        otherphysicslist[ip]->InjectConstraints(mdescriptor);
    }
}

void ChAssembly::ConstraintsBiReset() {
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip) {
        bodylist[ip]->ConstraintsBiReset();
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip) {
        linklist[ip]->ConstraintsBiReset();
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip) {
        otherphysicslist[ip]->ConstraintsBiReset();
    }
}

void ChAssembly::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip) {
        bodylist[ip]->ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip) {
        linklist[ip]->ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip) {
        otherphysicslist[ip]->ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);
    }
}

void ChAssembly::ConstraintsBiLoad_Ct(double factor) {
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip) {
        bodylist[ip]->ConstraintsBiLoad_Ct(factor);
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip) {
        linklist[ip]->ConstraintsBiLoad_Ct(factor);
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip) {
        otherphysicslist[ip]->ConstraintsBiLoad_Ct(factor);
    }
}

void ChAssembly::ConstraintsBiLoad_Qc(double factor) {
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip) {
        bodylist[ip]->ConstraintsBiLoad_Qc(factor);
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip) {
        linklist[ip]->ConstraintsBiLoad_Qc(factor);
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip) {
        otherphysicslist[ip]->ConstraintsBiLoad_Qc(factor);
    }
}

void ChAssembly::ConstraintsFbLoadForces(double factor) {
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip) {
        bodylist[ip]->ConstraintsFbLoadForces(factor);
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip) {
        linklist[ip]->ConstraintsFbLoadForces(factor);
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip) {
        otherphysicslist[ip]->ConstraintsFbLoadForces(factor);
    }
}

void ChAssembly::ConstraintsLoadJacobians() {
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip) {
        bodylist[ip]->ConstraintsLoadJacobians();
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip) {
        linklist[ip]->ConstraintsLoadJacobians();
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip) {
        otherphysicslist[ip]->ConstraintsLoadJacobians();
    }
}

void ChAssembly::ConstraintsLiLoadSuggestedSpeedSolution() {
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip) {
        bodylist[ip]->ConstraintsLiLoadSuggestedSpeedSolution();
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip) {
        linklist[ip]->ConstraintsLiLoadSuggestedSpeedSolution();
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip) {
        otherphysicslist[ip]->ConstraintsLiLoadSuggestedSpeedSolution();
    }
}

void ChAssembly::ConstraintsLiLoadSuggestedPositionSolution() {
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip) {
        bodylist[ip]->ConstraintsLiLoadSuggestedPositionSolution();
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip) {
        linklist[ip]->ConstraintsLiLoadSuggestedPositionSolution();
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip) {
        otherphysicslist[ip]->ConstraintsLiLoadSuggestedPositionSolution();
    }
}

void ChAssembly::ConstraintsLiFetchSuggestedSpeedSolution() {
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip) {
        bodylist[ip]->ConstraintsLiFetchSuggestedSpeedSolution();
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip) {
        linklist[ip]->ConstraintsLiFetchSuggestedSpeedSolution();
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip) {
        otherphysicslist[ip]->ConstraintsLiFetchSuggestedSpeedSolution();
    }
}

void ChAssembly::ConstraintsLiFetchSuggestedPositionSolution() {
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip) {
        bodylist[ip]->ConstraintsLiFetchSuggestedPositionSolution();
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip) {
        linklist[ip]->ConstraintsLiFetchSuggestedPositionSolution();
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip) {
        otherphysicslist[ip]->ConstraintsLiFetchSuggestedPositionSolution();
    }
}

void ChAssembly::ConstraintsFetch_react(double factor) {
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip) {
        bodylist[ip]->ConstraintsFetch_react(factor);
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip) {
        linklist[ip]->ConstraintsFetch_react(factor);
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip) {
        otherphysicslist[ip]->ConstraintsFetch_react(factor);
    }
}

void ChAssembly::InjectKRMmatrices(ChLcpSystemDescriptor& mdescriptor) {
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip) {
        bodylist[ip]->InjectKRMmatrices(mdescriptor);
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip) {
        linklist[ip]->InjectKRMmatrices(mdescriptor);
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip) {
        otherphysicslist[ip]->InjectKRMmatrices(mdescriptor);
    }
}

void ChAssembly::KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor) {
    for (unsigned int ip = 0; ip < bodylist.size(); ++ip) {
        bodylist[ip]->KRMmatricesLoad(Kfactor, Rfactor, Mfactor);
    }
    for (unsigned int ip = 0; ip < linklist.size(); ++ip) {
        linklist[ip]->KRMmatricesLoad(Kfactor, Rfactor, Mfactor);
    }
    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip) {
        otherphysicslist[ip]->KRMmatricesLoad(Kfactor, Rfactor, Mfactor);
    }
}




////////
////////  STREAMING - FILE HANDLING
////////



void ChAssembly::ShowHierarchy(ChStreamOutAscii& m_file, int level) {
    std::string mtabs;
    for (int i=0; i< level; ++i)
        mtabs += "  ";

    m_file << "\n" << mtabs << "List of the " << (int)Get_bodylist()->size() << " added rigid bodies: \n";

    std::vector<ChSharedPtr<ChBody> >::iterator ibody = Get_bodylist()->begin();
    while (ibody != Get_bodylist()->end()) {
        m_file << mtabs << "  BODY:       " << (*ibody)->GetName() << "\n";

        std::vector<ChSharedPtr<ChMarker> >::const_iterator imarker = (*ibody)->GetMarkerList().begin();
        while (imarker != (*ibody)->GetMarkerList().end()) {
            m_file << mtabs << "    MARKER:  " << (*imarker)->GetName() << "\n";
            imarker++;
        }

        std::vector<ChSharedPtr<ChForce> >::const_iterator iforce = (*ibody)->GetForceList().begin();
        while (iforce != (*ibody)->GetForceList().end()) {
            m_file << mtabs << "    FORCE:  " << (*iforce)->GetName() << "\n";
            iforce++;
        }

        ibody++;
    }

    m_file << "\n" << mtabs << "List of the " << (int)Get_linklist()->size() << " added links: \n";

    for (unsigned int ip = 0; ip < linklist.size(); ++ip)  // ITERATE on links
    {
        ChSharedPtr<ChLink> Lpointer = linklist[ip];

        m_file << mtabs << "  LINK:  " << Lpointer->GetName() << " [" << typeid(Lpointer.get_ptr()).name() << "]\n";
        if (ChSharedPtr<ChLinkMarkers> malink =  Lpointer.DynamicCastTo<ChLinkMarkers>() ) {
            if (malink->GetMarker1())
                m_file << mtabs << "    marker1:  " << malink->GetMarker1()->GetName() << "\n";
            if (malink->GetMarker2())
                m_file << mtabs << "    marker2:  " << malink->GetMarker2()->GetName() << "\n";
        }
    }

    m_file << "\n" << mtabs << "List of other " << (int)otherphysicslist.size() << " added physic items: \n";

    for (unsigned int ip = 0; ip < otherphysicslist.size(); ++ip)  // ITERATE on other physics
    {
        ChSharedPtr<ChPhysicsItem> PHpointer = otherphysicslist[ip];

        m_file << mtabs << "  PHYSIC ITEM :   " << PHpointer->GetName() << " [" << typeid(PHpointer.get_ptr()).name() << "]\n";
        
        // recursion:
        if (ChSharedPtr<ChAssembly>assem= PHpointer.DynamicCastTo<ChAssembly>())
            assem->ShowHierarchy(m_file,level+1);
    }

    /*
    m_file << "\n\nFlat ChPhysicalItem list (class name - object name):----- \n\n";

    IteratorAllPhysics mphiter(this);
    while (mphiter.HasItem()) {
        m_file << "  " << mphiter->GetRTTI()->GetName() << "  -  " << mphiter->GetName() << "\n";
        ++mphiter;
    }
    */
    m_file << "\n\n";
}



void ChAssembly::ArchiveOUT(ChArchiveOut& marchive)
{
    // version number
    marchive.VersionWrite(1);

    // serialize parent class
    ChPhysicsItem::ArchiveOUT(marchive);

    // serialize all member data:

    //marchive << CHNVP(bodylist);
    // do rather a custom array save:
    marchive.out_array_pre("bodies", bodylist.size(), "ChBody");
    for (int i = 0; i < bodylist.size(); i++) {
        marchive << CHNVP(bodylist[i],"");
        marchive.out_array_between(bodylist.size(), "bodies");
    }
    marchive.out_array_end(bodylist.size(), "bodies");

    //marchive << CHNVP(linklist);
    // do rather a custom array save:
    marchive.out_array_pre("links", linklist.size(), "ChLink");
    for (int i = 0; i < linklist.size(); i++) {
        marchive << CHNVP(linklist[i],"");
        marchive.out_array_between(linklist.size(), "links");
    }
    marchive.out_array_end(linklist.size(), "links");

    //marchive << CHNVP(otherphysicsitems);
    // do rather a custom array save:
    marchive.out_array_pre("other_physics_list", otherphysicslist.size(), "ChPhysicsItem");
    for (int i = 0; i < otherphysicslist.size(); i++) {
        marchive << CHNVP(otherphysicslist[i],"");
        marchive.out_array_between(otherphysicslist.size(), "other_physics_list");
    }
    marchive.out_array_end(otherphysicslist.size(), "other_physics_list");
}

/// Method to allow de serialization of transient data from archives.
void ChAssembly::ArchiveIN(ChArchiveIn& marchive) 
{
    // version number
    int version = marchive.VersionRead();

    // deserialize parent class
    ChPhysicsItem::ArchiveIN(marchive);

    // stream in all member data:

    //marchive >> CHNVP(bodylist);
    // do rather a custom array load:
    this->RemoveAllBodies();
    size_t num_bodies;
    marchive.in_array_pre("bodies", num_bodies);
    for (int i = 0; i < num_bodies; i++) {
        ChSharedPtr<ChBody> a_body;
        marchive >> CHNVP(a_body,"");
        this->AddBody(a_body);
        marchive.in_array_between("bodies");
    }
    marchive.in_array_end("bodies");

    //marchive >> CHNVP(linklist);
    // do rather a custom array load:
    this->RemoveAllLinks();
    size_t num_links;
    marchive.in_array_pre("links", num_links);
    for (int i = 0; i < num_links; i++) {
        ChSharedPtr<ChLink> a_link;
        marchive >> CHNVP(a_link,"");
        this->AddLink(a_link);
        marchive.in_array_between("links");
    }
    marchive.in_array_end("links");

    //marchive >> CHNVP(otherphysiscslist);
    // do rather a custom array load:
    this->RemoveAllOtherPhysicsItems();
    size_t num_otherphysics;
    marchive.in_array_pre("other_physics_list", num_otherphysics);
    for (int i = 0; i < num_otherphysics; i++) {
        ChSharedPtr<ChPhysicsItem> a_item;
        marchive >> CHNVP(a_item,"");
        this->AddOtherPhysicsItem(a_item);
        marchive.in_array_between("other_physics_list");
    }
    marchive.in_array_end("other_physics_list");

    // Recompute statistics, offsets, etc.
    this->Setup();
}





}  // END_OF_NAMESPACE____

/////////////////////
