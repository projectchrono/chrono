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


#include "chrono/physics/ChModalAssembly.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

using namespace fea;
using namespace collision;
using namespace geometry;

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChModalAssembly)

ChModalAssembly::ChModalAssembly()
    : modal_variables(nullptr),
    n_modes_coords_w(0),
    is_modal(false)
{}

ChModalAssembly::ChModalAssembly(const ChModalAssembly& other) : ChAssembly(other) {
    
    is_modal = other.is_modal;
    modal_q = other.modal_q;
    modal_q_dt = other.modal_q_dt;
    modal_q_dtdt = other.modal_q_dtdt;
    modal_F = other.modal_F;

    //// TODO:  deep copy of the object lists (internal_bodylist, internal_linklist, internal_meshlist,  internal_otherphysicslist)
}

ChModalAssembly::~ChModalAssembly() {
    RemoveAllInternalBodies();
    RemoveAllInternalLinks();
    RemoveAllInternalMeshes();
    RemoveAllInternalOtherPhysicsItems();
    if (modal_variables) 
        delete modal_variables;
}

ChModalAssembly& ChModalAssembly::operator=(ChModalAssembly other) {
    ChModalAssembly tmp(other);
    swap(*this, other);
    return *this;
}

// Note: implement this as a friend function (instead of a member function swap(ChModalAssembly& other)) so that other
// classes that have a ChModalAssembly member (currently only ChSystem) could use it, the same way we use std::swap here.
void swap(ChModalAssembly& first, ChModalAssembly& second) {
    using std::swap;
    // swap(first.nbodies, second.nbodies);
    // ***TODO***
}

void ChModalAssembly::Clear() {
    ChAssembly::Clear(); // parent 

    RemoveAllInternalBodies();
    RemoveAllInternalLinks();
    RemoveAllInternalMeshes();
    RemoveAllInternalOtherPhysicsItems();

    if (modal_variables) 
        delete modal_variables;
}

//---------------------------------------------------------------------------------------
   

void ChModalAssembly::SetupModalData() {

    if (!modal_variables || (modal_variables->Get_ndof() != this->n_modes_coords_w)) {
            
        int bou_int_coords_w = this->n_boundary_coords_w + this->n_internal_coords_w;
        int bou_mod_coords_w = this->n_boundary_coords_w + this->n_modes_coords_w;

        // Initialize ChVariable object used for modal variables
        if (modal_variables)
            delete modal_variables;
        modal_variables = new ChVariablesGenericDiagonalMass(this->n_modes_coords_w);

        // Initialize the modal_Hblock, which is a ChKblockGeneric referencing all ChVariable items:
        std::vector<ChVariables*> mvars;
        // - for BOUNDARY variables: trick to collect all ChVariable references..
        ChSystemDescriptor temporary_descriptor;
        for (auto& body : bodylist)
            body->InjectVariables(temporary_descriptor);
        for (auto& link : linklist)
            link->InjectVariables(temporary_descriptor);
        for (auto& mesh : meshlist)
            mesh->InjectVariables(temporary_descriptor);
        for (auto& item : otherphysicslist)
            item->InjectVariables(temporary_descriptor);
        mvars = temporary_descriptor.GetVariablesList();
        // - for the MODAL variables:
        mvars.push_back(this->modal_variables);
        this->modal_Hblock.SetVariables(mvars);

        // Initialize vectors to be used with modal coordinates:
        this->modal_q.setZero(this->n_modes_coords_w);
        this->modal_q_dt.setZero(this->n_modes_coords_w);
        this->modal_q_dtdt.setZero(this->n_modes_coords_w);
        this->modal_F.setZero(this->n_modes_coords_w);

        this->modal_M.setZero(bou_mod_coords_w, bou_mod_coords_w);
        this->modal_K.setZero(bou_mod_coords_w, bou_mod_coords_w);
        this->modal_R.setZero(bou_mod_coords_w, bou_mod_coords_w);

        this->modes_V.setZero(bou_mod_coords_w, this->n_modes_coords_w);
    }
}

void ChModalAssembly::ComputeModes(int nmodes) {
    
    // cannot use less modes than n. of internal coords, if so, clamp to internal coords
    this->n_modes_coords_w = ChMin(nmodes, this->n_internal_coords_w);

    this->Setup();
    this->SetupModalData();

    ChSparseMatrix full_M;
    ChSparseMatrix full_R;
    ChSparseMatrix full_K;
    ChSparseMatrix full_Cq;

    this->GetSubassemblyMassMatrix(&full_M);
    this->GetSubassemblyDampingMatrix(&full_R);
    this->GetSubassemblyStiffnessMatrix(&full_K);
    this->GetSubassemblyConstraintJacobianMatrix(&full_Cq);

    // TEST use the nullspace method just for golden reference
    ChQuadraticEigenvalueSolverNullspaceDirect eigsolver;
    eigsolver.Solve(full_M, full_R, full_K, full_Cq, this->modes_V, this->modes_eig, 5);

    // TEST
    auto V_real = this->modes_V.real();
    this->modal_M = V_real.transpose() * full_M * V_real;
    this->modal_R = V_real.transpose() * full_R * V_real;
    this->modal_K = V_real.transpose() * full_K * V_real;

    this->Setup();
}
    
void ChModalAssembly::ComputeModes(ChMatrixRef my_modal_M, ChMatrixRef my_modal_R, ChMatrixRef my_modal_K, ChMatrixRef my_modes_V) {

    this->n_modes_coords_w = my_modes_V.cols();

    this->Setup();
    this->SetupModalData();

    int bou_int_coords_w = this->n_boundary_coords_w + this->n_internal_coords_w;
    int bou_mod_coords_w = this->n_boundary_coords_w + this->n_modes_coords_w;
    assert((my_modal_M.rows() == bou_mod_coords_w) && (my_modal_M.cols() == bou_mod_coords_w));
    assert((my_modal_R.rows() == bou_mod_coords_w) && (my_modal_R.cols() == bou_mod_coords_w));
    assert((my_modal_K.rows() == bou_mod_coords_w) && (my_modal_K.cols() == bou_mod_coords_w));
    assert((my_modes_V.rows() == bou_int_coords_w) );



    this->modes_V = my_modes_V;

    this->modal_M = my_modal_M;
    this->modal_R = my_modal_R;
    this->modal_K = my_modal_K;

    this->Setup();
}

void ChModalAssembly::ComputeModes(ChMatrixRef my_modes_V) {

    this->n_modes_coords_w = my_modes_V.cols();

    this->Setup();
    this->SetupModalData();
  
    int bou_int_coords_w = this->n_boundary_coords_w + this->n_internal_coords_w;
    int bou_mod_coords_w = this->n_boundary_coords_w + this->n_modes_coords_w;
    assert((my_modes_V.rows() == bou_int_coords_w) );


    this->modes_V = my_modes_V; 


    ChSparseMatrix full_M;
    ChSparseMatrix full_R;
    ChSparseMatrix full_K;

    this->GetSubassemblyMassMatrix(&full_M);
    this->GetSubassemblyDampingMatrix(&full_R);
    this->GetSubassemblyStiffnessMatrix(&full_K);

    this->modal_M = my_modes_V.transpose() * full_M * my_modes_V;
    this->modal_R = my_modes_V.transpose() * full_R * my_modes_V;
    this->modal_K = my_modes_V.transpose() * full_K * my_modes_V;
}


void ChModalAssembly::ModeIncrementState(int n_mode, double phase, double amplitude) {
    
    bool needs_temporary_bou_int = this->is_modal;
    if (needs_temporary_bou_int) 
        this->is_modal = false; // to have IntStateGather returning all boundary and internal

    if (n_mode >= this->modes_V.cols())
        throw ChException("Error: mode " + std::to_string(n_mode) + " is beyond the " + std::to_string(this->modes_V.cols()) + " computed eigenvectors.");
    
    int bou_int_coords   = this->n_boundary_coords   + this->n_internal_coords;
    int bou_int_coords_w = this->n_boundary_coords_w + this->n_internal_coords_w;
    assert(this->modes_V.rows() == bou_int_coords_w);

    double fooT;
    ChState assembly_x;
    ChState assembly_x_new;
    ChStateDelta assembly_v;
    ChStateDelta assembly_Dx;
    
    assembly_x.setZero(bou_int_coords, nullptr);
    assembly_x_new.setZero(bou_int_coords, nullptr);
    assembly_v.setZero(bou_int_coords_w, nullptr);
    assembly_Dx.setZero(bou_int_coords_w, nullptr);

    this->IntStateGather(0, assembly_x, 0, assembly_v, fooT);
    
    // pick the nth eigenvector
    assembly_Dx = sin(phase) * amplitude * this->modes_V.col(n_mode).real();   //***TODO***: .. + cos(phase) * amplitude * this->modes_V.col(n_mode).imag()
    
    this->IntStateIncrement(0, assembly_x_new, assembly_x, 0, assembly_Dx); // x += amplitude * eigenvector

    this->IntStateScatter(0, assembly_x_new, 0, assembly_v, fooT, true);

    this->Update();

    if (needs_temporary_bou_int) 
        this->is_modal = true;
}




//---------------------------------------------------------------------------------------

// Note: removing items from the assembly incurs linear time cost

void ChModalAssembly::AddInternalBody(std::shared_ptr<ChBody> body) {
    assert(std::find(std::begin(internal_bodylist), std::end(internal_bodylist), body) == internal_bodylist.end());
    assert(body->GetSystem() == nullptr);  // should remove from other system before adding here

    // set system and also add collision models to system
    body->SetSystem(system);
    internal_bodylist.push_back(body);

	////system->is_initialized = false;  // Not needed, unless/until ChBody::SetupInitial does something
	system->is_updated = false;
}

void ChModalAssembly::RemoveInternalBody(std::shared_ptr<ChBody> body) {
    auto itr = std::find(std::begin(internal_bodylist), std::end(internal_bodylist), body);
    assert(itr != internal_bodylist.end());

    internal_bodylist.erase(itr);
    body->SetSystem(nullptr);

    system->is_updated = false;
}

void ChModalAssembly::AddInternalLink(std::shared_ptr<ChLinkBase> link) {
    assert(std::find(std::begin(internal_linklist), std::end(internal_linklist), link) == internal_linklist.end());

    link->SetSystem(system);
    internal_linklist.push_back(link);

	////system->is_initialized = false;  // Not needed, unless/until ChLink::SetupInitial does something
    system->is_updated = false;
}

void ChModalAssembly::RemoveInternalLink(std::shared_ptr<ChLinkBase> link) {
    auto itr = std::find(std::begin(internal_linklist), std::end(internal_linklist), link);
    assert(itr != internal_linklist.end());

    internal_linklist.erase(itr);
    link->SetSystem(nullptr);

    system->is_updated = false;
}

void ChModalAssembly::AddInternalMesh(std::shared_ptr<fea::ChMesh> mesh) {
    assert(std::find(std::begin(internal_meshlist), std::end(internal_meshlist), mesh) == internal_meshlist.end());

    mesh->SetSystem(system);
    internal_meshlist.push_back(mesh);

	system->is_initialized = false;
    system->is_updated = false;
}

void ChModalAssembly::RemoveInternalMesh(std::shared_ptr<fea::ChMesh> mesh) {
    auto itr = std::find(std::begin(internal_meshlist), std::end(internal_meshlist), mesh);
    assert(itr != internal_meshlist.end());

    internal_meshlist.erase(itr);
    mesh->SetSystem(nullptr);

    system->is_updated = false;
}

void ChModalAssembly::AddInternalOtherPhysicsItem(std::shared_ptr<ChPhysicsItem> item) {
    assert(!std::dynamic_pointer_cast<ChBody>(item));
    assert(!std::dynamic_pointer_cast<ChLinkBase>(item));
    assert(!std::dynamic_pointer_cast<ChMesh>(item));
    assert(std::find(std::begin(internal_otherphysicslist), std::end(internal_otherphysicslist), item) == internal_otherphysicslist.end());
    // assert(item->GetSystem()==nullptr); // should remove from other system before adding here

    // set system and also add collision models to system
    item->SetSystem(system);
    internal_otherphysicslist.push_back(item);

	////system->is_initialized = false;  // Not needed, unless/until ChPhysicsItem::SetupInitial does something
    system->is_updated = false;
}

void ChModalAssembly::RemoveInternalOtherPhysicsItem(std::shared_ptr<ChPhysicsItem> item) {
    auto itr = std::find(std::begin(internal_otherphysicslist), std::end(internal_otherphysicslist), item);
    assert(itr != internal_otherphysicslist.end());

    internal_otherphysicslist.erase(itr);
    item->SetSystem(nullptr);

    system->is_updated = false;
}

void ChModalAssembly::AddInternal(std::shared_ptr<ChPhysicsItem> item) {
    if (auto body = std::dynamic_pointer_cast<ChBody>(item)) {
        AddInternalBody(body);
        return;
    }

    if (auto link = std::dynamic_pointer_cast<ChLinkBase>(item)) {
        AddInternalLink(link);
        return;
    }

    if (auto mesh = std::dynamic_pointer_cast<fea::ChMesh>(item)) {
        AddInternalMesh(mesh);
        return;
    }

    AddInternalOtherPhysicsItem(item);
}


void ChModalAssembly::RemoveInternal(std::shared_ptr<ChPhysicsItem> item) {
    if (auto body = std::dynamic_pointer_cast<ChBody>(item)) {
        RemoveInternalBody(body);
        return;
    }

    if (auto link = std::dynamic_pointer_cast<ChLinkBase>(item)) {
        RemoveInternalLink(link);
        return;
    }

    if (auto mesh = std::dynamic_pointer_cast<fea::ChMesh>(item)) {
        RemoveInternalMesh(mesh);
        return;
    }

    RemoveInternalOtherPhysicsItem(item);
}

void ChModalAssembly::RemoveAllInternalBodies() {
    for (auto& body : internal_bodylist) {
        body->SetSystem(nullptr);
    }
    internal_bodylist.clear();

    if (system)
        system->is_updated = false;
}

void ChModalAssembly::RemoveAllInternalLinks() {
    for (auto& link : internal_linklist) {
        link->SetSystem(nullptr);
    }
    internal_linklist.clear();

    if (system)
        system->is_updated = false;
}

void ChModalAssembly::RemoveAllInternalMeshes() {
    for (auto& mesh : internal_meshlist) {
        mesh->SetSystem(nullptr);
    }
    internal_meshlist.clear();

    if (system)
        system->is_updated = false;
}

void ChModalAssembly::RemoveAllInternalOtherPhysicsItems() {
    for (auto& item : internal_otherphysicslist) {
        item->SetSystem(nullptr);
    }
    internal_otherphysicslist.clear();

    if (system)
        system->is_updated = false;
}


// -----------------------------------------------------------------------------


void ChModalAssembly::GetSubassemblyMassMatrix(ChSparseMatrix* M) {

    this->SetupInitial();
    this->Setup();
    this->Update();

    ChSystemDescriptor temp_descriptor;

    this->InjectVariables(temp_descriptor);
    this->InjectKRMmatrices(temp_descriptor);
    this->InjectConstraints(temp_descriptor);

    // Load all KRM matrices with the M part only
    KRMmatricesLoad(0, 0, 1.0);
    // For ChVariable objects without a ChKblock, but still with a mass:
    temp_descriptor.SetMassFactor(1.0);

    // Fill system-level M matrix
    temp_descriptor.ConvertToMatrixForm(nullptr, M, nullptr, nullptr, nullptr, nullptr, false, false);
}

void ChModalAssembly::GetSubassemblyStiffnessMatrix(ChSparseMatrix* K) {
    
    this->SetupInitial();
    this->Setup();
    this->Update();

    ChSystemDescriptor temp_descriptor;

    this->InjectVariables(temp_descriptor);
    this->InjectKRMmatrices(temp_descriptor);
    this->InjectConstraints(temp_descriptor);

    // Load all KRM matrices with the K part only
    this->KRMmatricesLoad(1.0, 0, 0);
    // For ChVariable objects without a ChKblock, but still with a mass:
    temp_descriptor.SetMassFactor(0.0);

    // Fill system-level K matrix
    temp_descriptor.ConvertToMatrixForm(nullptr, K, nullptr, nullptr, nullptr, nullptr, false, false);
}

void ChModalAssembly::GetSubassemblyDampingMatrix(ChSparseMatrix* R) {
    
    this->SetupInitial();
    this->Setup();
    this->Update();

    ChSystemDescriptor temp_descriptor;

    this->InjectVariables(temp_descriptor);
    this->InjectKRMmatrices(temp_descriptor);
    this->InjectConstraints(temp_descriptor);

    // Load all KRM matrices with the R part only
    this->KRMmatricesLoad(0, 1.0, 0);
    // For ChVariable objects without a ChKblock, but still with a mass:
    temp_descriptor.SetMassFactor(0.0);

    // Fill system-level R matrix
    temp_descriptor.ConvertToMatrixForm(nullptr, R, nullptr, nullptr, nullptr, nullptr, false, false);

}

void ChModalAssembly::GetSubassemblyConstraintJacobianMatrix(ChSparseMatrix* Cq) {
    
    this->SetupInitial();
    this->Setup();
    this->Update();

    ChSystemDescriptor temp_descriptor;

    this->InjectVariables(temp_descriptor);
    this->InjectKRMmatrices(temp_descriptor);
    this->InjectConstraints(temp_descriptor);

    // Load all jacobian matrices
    this->ConstraintsLoadJacobians();

    // Fill system-level R matrix
    temp_descriptor.ConvertToMatrixForm(Cq, nullptr, nullptr, nullptr, nullptr, nullptr, false, false);
}

void ChModalAssembly::DumpSubassemblyMatrices(bool save_M, bool save_K, bool save_R, bool save_Cq, const char* path) {
    char filename[300];
    const char* numformat = "%.12g";

    if (save_M) {
        ChSparseMatrix mM;
        this->GetSubassemblyMassMatrix(&mM);
        sprintf(filename, "%s%s", path, "_M.dat");
        ChStreamOutAsciiFile file_M(filename);
        file_M.SetNumFormat(numformat);
        StreamOUTsparseMatlabFormat(mM, file_M);
    }
    if (save_K) {
        ChSparseMatrix mK;
        this->GetSubassemblyStiffnessMatrix(&mK);
        sprintf(filename, "%s%s", path, "_K.dat");
        ChStreamOutAsciiFile file_K(filename);
        file_K.SetNumFormat(numformat);
        StreamOUTsparseMatlabFormat(mK, file_K);
    }
    if (save_R) {
        ChSparseMatrix mR;
        this->GetSubassemblyDampingMatrix(&mR);
        sprintf(filename, "%s%s", path, "_R.dat");
        ChStreamOutAsciiFile file_R(filename);
        file_R.SetNumFormat(numformat);
        StreamOUTsparseMatlabFormat(mR, file_R);
    }
    if (save_Cq) {
        ChSparseMatrix mCq;
        this->GetSubassemblyConstraintJacobianMatrix(&mCq);
        sprintf(filename, "%s%s", path, "_Cq.dat");
        ChStreamOutAsciiFile file_Cq(filename);
        file_Cq.SetNumFormat(numformat);
        StreamOUTsparseMatlabFormat(mCq, file_Cq);
    }
}

// -----------------------------------------------------------------------------

void ChModalAssembly::SetSystem(ChSystem* m_system) {
    
    ChAssembly::SetSystem(m_system); // parent
    

    for (auto& body : internal_bodylist) {
        body->SetSystem(m_system);
    }
    for (auto& link : internal_linklist) {
        link->SetSystem(m_system);
    }
    for (auto& mesh : internal_meshlist) {
        mesh->SetSystem(m_system);
    }
    for (auto& item : internal_otherphysicslist) {
        item->SetSystem(m_system);
    }
}

void ChModalAssembly::SyncCollisionModels() {

    ChAssembly::SyncCollisionModels(); // parent

    for (auto& body : internal_bodylist) {
        body->SyncCollisionModels();
    }
    for (auto& link : internal_linklist) {
        link->SyncCollisionModels();
    }
    for (auto& mesh : internal_meshlist) {
        mesh->SyncCollisionModels();
    }
    for (auto& item : internal_otherphysicslist) {
        item->SyncCollisionModels();
    }
}

// -----------------------------------------------------------------------------
// UPDATING ROUTINES

void ChModalAssembly::SetupInitial() {

    ChAssembly::SetupInitial(); // parent

    for (int ip = 0; ip < internal_bodylist.size(); ++ip) {
        internal_bodylist[ip]->SetupInitial();
    }
    for (int ip = 0; ip < internal_linklist.size(); ++ip) {
        internal_linklist[ip]->SetupInitial();
    }
    for (int ip = 0; ip < internal_meshlist.size(); ++ip) {
        internal_meshlist[ip]->SetupInitial();
    }
    for (int ip = 0; ip < internal_otherphysicslist.size(); ++ip) {
        internal_otherphysicslist[ip]->SetupInitial();
    }
}

// Count all bodies, links, meshes, and other physics items.
// Set counters (DOF, num constraints, etc) and offsets.
void ChModalAssembly::Setup() {

    ChAssembly::Setup(); // parent

    n_boundary_bodies = nbodies;
    n_boundary_links = nlinks;
    n_boundary_meshes = nmeshes;
    n_boundary_physicsitems = nphysicsitems;
    n_boundary_coords = ncoords;
    n_boundary_coords_w = ncoords_w;
    n_boundary_doc = ndoc;
    n_boundary_doc_w = ndoc_w;
    n_boundary_doc_w_C = ndoc_w_C;
    n_boundary_doc_w_D = ndoc_w_D;
    n_boundary_sysvars   = nsysvars;
    n_boundary_sysvars_w = nsysvars_w;
    n_boundary_dof       = ndof;

    n_internal_bodies = 0;
    n_internal_links = 0;
    n_internal_meshes = 0;
    n_internal_physicsitems = 0;
    n_internal_coords = 0;
    n_internal_coords_w = 0;
    n_internal_doc = 0;
    n_internal_doc_w = 0;
    n_internal_doc_w_C = 0;
    n_internal_doc_w_D = 0;

    // For the "internal" items:
    //

    for (auto& body : internal_bodylist) {
        if (body->GetBodyFixed())
        {
            //throw ChException("Cannot use a fixed body as internal");
        }
        else if (body->GetSleeping())
        {
            //throw ChException("Cannot use a sleeping body as internal");
        }
        else {
            n_internal_bodies++;

            body->SetOffset_x(this->offset_x + n_boundary_coords   + n_internal_coords);
            body->SetOffset_w(this->offset_w + n_boundary_coords_w + n_internal_coords_w);
            body->SetOffset_L(this->offset_L + n_boundary_doc_w    + n_internal_doc_w);

            body->Setup();  // currently, no-op

            n_internal_coords += body->GetDOF();
            n_internal_coords_w += body->GetDOF_w();
            n_internal_doc_w += body->GetDOC();      // not really needed since ChBody introduces no constraints
        }
    }

    for (auto& link : internal_linklist) {
        if (link->IsActive()) {
            n_internal_links++;

            link->SetOffset_x(this->offset_x + n_boundary_coords   + n_internal_coords);
            link->SetOffset_w(this->offset_w + n_boundary_coords_w + n_internal_coords_w);
            link->SetOffset_L(this->offset_L + n_boundary_doc_w    + n_internal_doc_w);

            link->Setup();  // compute DOFs etc. and sets the offsets also in child items, if any

            n_internal_coords += link->GetDOF();
            n_internal_coords_w += link->GetDOF_w();
            n_internal_doc_w += link->GetDOC();
            n_internal_doc_w_C += link->GetDOC_c();
            n_internal_doc_w_D += link->GetDOC_d();
        }
    }

    for (auto& mesh : internal_meshlist) {
        n_internal_meshes++;

        mesh->SetOffset_x(this->offset_x + n_boundary_coords   + n_internal_coords);
        mesh->SetOffset_w(this->offset_w + n_boundary_coords_w + n_internal_coords_w);
        mesh->SetOffset_L(this->offset_L + n_boundary_doc_w    + n_internal_doc_w);

        mesh->Setup();  // compute DOFs and iteratively call Setup for child items

        n_internal_coords += mesh->GetDOF();
        n_internal_coords_w += mesh->GetDOF_w();
        n_internal_doc_w += mesh->GetDOC();
        n_internal_doc_w_C += mesh->GetDOC_c();
        n_internal_doc_w_D += mesh->GetDOC_d();
    }

    for (auto& item : internal_otherphysicslist) {
        n_internal_physicsitems++;

        item->SetOffset_x(this->offset_x + n_boundary_coords   + n_internal_coords);
        item->SetOffset_w(this->offset_w + n_boundary_coords_w + n_internal_coords_w);
        item->SetOffset_L(this->offset_L + n_boundary_doc_w    + n_internal_doc_w);

        item->Setup();

        n_internal_coords += item->GetDOF();
        n_internal_coords_w += item->GetDOF_w();
        n_internal_doc_w += item->GetDOC();
        n_internal_doc_w_C += item->GetDOC_c();
        n_internal_doc_w_D += item->GetDOC_d();
    }

    n_internal_doc = n_internal_doc_w + n_internal_bodies;          // number of constraints including quaternion constraints.
    n_internal_sysvars = n_internal_coords + n_internal_doc;        // total number of variables (coordinates + lagrangian multipliers)
    n_internal_sysvars_w = n_internal_coords_w + n_internal_doc_w;  // total number of variables (with 6 dof per body)
    n_internal_dof = n_internal_coords_w - n_internal_doc_w;


    // For the modal part:
    //

    // (nothing to count)


    // For the entire assembly:
    //

    if (this->is_modal == false) {
        ncoords    = n_boundary_coords    + n_internal_coords;
        ncoords_w  = n_boundary_coords_w  + n_internal_coords_w;
        ndoc       = n_boundary_doc       + n_internal_doc;
        ndoc_w     = n_boundary_doc_w     + n_internal_doc_w;
        ndoc_w_C   = n_boundary_doc_w_C   + n_internal_doc_w_C;
        ndoc_w_D   = n_boundary_doc_w_D   + n_internal_doc_w_D;
        nsysvars   = n_boundary_sysvars   + n_internal_sysvars;
        nsysvars_w = n_boundary_sysvars_w + n_internal_sysvars_w;
        ndof       = n_boundary_dof       + n_internal_dof;
        nbodies += n_internal_bodies;
        nlinks  += n_internal_links;
        nmeshes += n_internal_meshes;
        nphysicsitems += n_internal_physicsitems;
    }
    else {
        ncoords    = n_boundary_coords    + n_modes_coords_w; // no need for a n_modes_coords, same as n_modes_coords_w
        ncoords_w  = n_boundary_coords_w  + n_modes_coords_w;
        ndoc       = n_boundary_doc;
        ndoc_w     = n_boundary_doc_w;
        ndoc_w_C   = n_boundary_doc_w_C;
        ndoc_w_D   = n_boundary_doc_w_D;
        nsysvars   = n_boundary_sysvars   + n_modes_coords_w; // no need for a n_modes_coords, same as n_modes_coords_w
        nsysvars_w = n_boundary_sysvars_w + n_modes_coords_w;
        ndof       = n_boundary_dof       + n_modes_coords_w;
    }
}

// Update assembly's own properties first (ChTime and assets, if any).
// Then update all contents of this assembly.
void ChModalAssembly::Update(double mytime, bool update_assets) {

    ChAssembly::Update(mytime, update_assets);  // parent

    Update(update_assets);
}

// Update all physical items (bodies, links, meshes, etc), including their auxiliary variables.
// Updates all forces (automatic, as children of bodies)
// Updates all markers (automatic, as children of bodies).
void ChModalAssembly::Update(bool update_assets) {

    ChAssembly::Update(update_assets);  // parent

    if (is_modal == false) {
        //// NOTE: do not switch these to range for loops (may want to use OMP for)
        for (int ip = 0; ip < (int)internal_bodylist.size(); ++ip) {
            internal_bodylist[ip]->Update(ChTime, update_assets);
        }
        for (int ip = 0; ip < (int)internal_otherphysicslist.size(); ++ip) {
            internal_otherphysicslist[ip]->Update(ChTime, update_assets);
        }
        for (int ip = 0; ip < (int)internal_linklist.size(); ++ip) {
            internal_linklist[ip]->Update(ChTime, update_assets);
        }
        for (int ip = 0; ip < (int)internal_meshlist.size(); ++ip) {
            internal_meshlist[ip]->Update(ChTime, update_assets);
        }
    }
}

void ChModalAssembly::SetNoSpeedNoAcceleration() {

    ChAssembly::SetNoSpeedNoAcceleration();  // parent

    if (is_modal == false) {
        for (auto& body : internal_bodylist) {
            body->SetNoSpeedNoAcceleration();
        }
        for (auto& link : internal_linklist) {
            link->SetNoSpeedNoAcceleration();
        }
        for (auto& mesh : internal_meshlist) {
            mesh->SetNoSpeedNoAcceleration();
        }
        for (auto& item : internal_otherphysicslist) {
            item->SetNoSpeedNoAcceleration();
        }
    }
    else {
        this->modal_q_dt.setZero(this->n_modes_coords_w);
        this->modal_q_dtdt.setZero(this->n_modes_coords_w);
    }
}

void ChModalAssembly::IntStateGather(const unsigned int off_x,
                                ChState& x,
                                const unsigned int off_v,
                                ChStateDelta& v,
                                double& T) {
    ChAssembly::IntStateGather(off_x, x, off_v, v, T);  // parent

    unsigned int displ_x = off_x - this->offset_x;
    unsigned int displ_v = off_v - this->offset_w;

    if (is_modal == false) {
        for (auto& body : internal_bodylist) {
            if (body->IsActive())
                body->IntStateGather(displ_x + body->GetOffset_x(), x, displ_v + body->GetOffset_w(), v, T);
        }
        for (auto& link : internal_linklist) {
            if (link->IsActive())
                link->IntStateGather(displ_x + link->GetOffset_x(), x, displ_v + link->GetOffset_w(), v, T);
        }
        for (auto& mesh : internal_meshlist) {
            mesh->IntStateGather(displ_x + mesh->GetOffset_x(), x, displ_v + mesh->GetOffset_w(), v, T);
        }
        for (auto& item : internal_otherphysicslist) {
            item->IntStateGather(displ_x + item->GetOffset_x(), x, displ_v + item->GetOffset_w(), v, T);
        }
    }
    else {
        x.segment(off_x + this->n_boundary_coords,   this->n_modes_coords_w) = this->modal_q;
        v.segment(off_v + this->n_boundary_coords_w, this->n_modes_coords_w) = this->modal_q_dt;
    }
}

void ChModalAssembly::IntStateScatter(const unsigned int off_x,
                                 const ChState& x,
                                 const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const double T,
                                 bool full_update) {
    ChAssembly::IntStateScatter(off_x, x, off_v, v, T, full_update);  // parent

    unsigned int displ_x = off_x - this->offset_x;
    unsigned int displ_v = off_v - this->offset_w;

    if (is_modal == false) {
        for (auto& body : internal_bodylist) {
            if (body->IsActive())
                body->IntStateScatter(displ_x + body->GetOffset_x(), x, displ_v + body->GetOffset_w(), v, T, full_update);
            else
                body->Update(T, full_update);
        }
        for (auto& mesh : internal_meshlist) {
            mesh->IntStateScatter(displ_x + mesh->GetOffset_x(), x, displ_v + mesh->GetOffset_w(), v, T, full_update);
        }
        for (auto& link : internal_linklist) {
            if (link->IsActive())
                link->IntStateScatter(displ_x + link->GetOffset_x(), x, displ_v + link->GetOffset_w(), v, T, full_update);
            else
                link->Update(T, full_update);
        }
        for (auto& item : internal_otherphysicslist) {
            item->IntStateScatter(displ_x + item->GetOffset_x(), x, displ_v + item->GetOffset_w(), v, T, full_update);
        }
    }
    else {
        this->modal_q    = x.segment(off_x + this->n_boundary_coords,   this->n_modes_coords_w);
        this->modal_q_dt = v.segment(off_v + this->n_boundary_coords_w, this->n_modes_coords_w);
    }
}

void ChModalAssembly::IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {

    ChAssembly::IntStateGatherAcceleration(off_a, a);  // parent

    unsigned int displ_a = off_a - this->offset_w;

    if (is_modal == false) {
        for (auto& body : internal_bodylist) {
            if (body->IsActive())
                body->IntStateGatherAcceleration(displ_a + body->GetOffset_w(), a);
        }
        for (auto& link : internal_linklist) {
            if (link->IsActive())
                link->IntStateGatherAcceleration(displ_a + link->GetOffset_w(), a);
        }
        for (auto& mesh : internal_meshlist) {
            mesh->IntStateGatherAcceleration(displ_a + mesh->GetOffset_w(), a);
        }
        for (auto& item : internal_otherphysicslist) {
            item->IntStateGatherAcceleration(displ_a + item->GetOffset_w(), a);
        }
    }
    else {
        a.segment(off_a + this->n_boundary_coords_w, this->n_modes_coords_w) = this->modal_q_dtdt;
    }
}

// From state derivative (acceleration) to system, sometimes might be needed
void ChModalAssembly::IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {

    ChAssembly::IntStateScatterAcceleration(off_a, a);  // parent

    unsigned int displ_a = off_a - this->offset_w;

    if (is_modal == false) {
        for (auto& body : internal_bodylist) {
            if (body->IsActive())
                body->IntStateScatterAcceleration(displ_a + body->GetOffset_w(), a);
        }
        for (auto& link : internal_linklist) {
            if (link->IsActive())
                link->IntStateScatterAcceleration(displ_a + link->GetOffset_w(), a);
        }
        for (auto& mesh : internal_meshlist) {
            mesh->IntStateScatterAcceleration(displ_a + mesh->GetOffset_w(), a);
        }
        for (auto& item : internal_otherphysicslist) {
            item->IntStateScatterAcceleration(displ_a + item->GetOffset_w(), a);
        }
    }
    else {
        this->modal_q_dtdt = a.segment(off_a + this->n_boundary_coords_w, this->n_modes_coords_w);
    }
}

// From system to reaction forces (last computed) - some timestepper might need this
void ChModalAssembly::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    
    ChAssembly::IntStateGatherReactions(off_L, L);  // parent

    unsigned int displ_L = off_L - this->offset_L;

    if (is_modal == false) {
        for (auto& body : internal_bodylist) {
            if (body->IsActive())
                body->IntStateGatherReactions(displ_L + body->GetOffset_L(), L);
        }
        for (auto& link : internal_linklist) {
            if (link->IsActive())
                link->IntStateGatherReactions(displ_L + link->GetOffset_L(), L);
        }
        for (auto& mesh : internal_meshlist) {
            mesh->IntStateGatherReactions(displ_L + mesh->GetOffset_L(), L);
        }
        for (auto& item : internal_otherphysicslist) {
            item->IntStateGatherReactions(displ_L + item->GetOffset_L(), L);
        }
    }
}

// From reaction forces to system, ex. store last computed reactions in ChLinkBase objects for plotting etc.
void ChModalAssembly::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {

    ChAssembly::IntStateScatterReactions(off_L, L);  // parent

    unsigned int displ_L = off_L - this->offset_L;

    if (is_modal == false) {
        for (auto& body : internal_bodylist) {
            if (body->IsActive())
                body->IntStateScatterReactions(displ_L + body->GetOffset_L(), L);
        }
        for (auto& link : internal_linklist) {
            if (link->IsActive())
                link->IntStateScatterReactions(displ_L + link->GetOffset_L(), L);
        }
        for (auto& mesh : internal_meshlist) {
            mesh->IntStateScatterReactions(displ_L + mesh->GetOffset_L(), L);
        }
        for (auto& item : internal_otherphysicslist) {
            item->IntStateScatterReactions(displ_L + item->GetOffset_L(), L);
        }
    }
}

void ChModalAssembly::IntStateIncrement(const unsigned int off_x,
                                   ChState& x_new,
                                   const ChState& x,
                                   const unsigned int off_v,
                                   const ChStateDelta& Dv) {

    ChAssembly::IntStateIncrement(off_x, x_new, x, off_v, Dv);  // parent

    unsigned int displ_x = off_x - this->offset_x;
    unsigned int displ_v = off_v - this->offset_w;

    if (is_modal == false) {
        for (auto& body : internal_bodylist) {
            if (body->IsActive())
                body->IntStateIncrement(displ_x + body->GetOffset_x(), x_new, x, displ_v + body->GetOffset_w(), Dv);
        }

        for (auto& link : internal_linklist) {
            if (link->IsActive())
                link->IntStateIncrement(displ_x + link->GetOffset_x(), x_new, x, displ_v + link->GetOffset_w(), Dv);
        }

        for (auto& mesh : internal_meshlist) {
            mesh->IntStateIncrement(displ_x + mesh->GetOffset_x(), x_new, x, displ_v + mesh->GetOffset_w(), Dv);
        }

        for (auto& item : internal_otherphysicslist) {
            item->IntStateIncrement(displ_x + item->GetOffset_x(), x_new, x, displ_v + item->GetOffset_w(), Dv);
        }
    }
    else {
        x_new.segment(off_x + this->n_boundary_coords, this->n_modes_coords_w) = 
            x.segment(off_x + this->n_boundary_coords,   this->n_modes_coords_w) 
         + Dv.segment(off_v + this->n_boundary_coords_w, this->n_modes_coords_w);
    }
}


void ChModalAssembly::IntLoadResidual_F(const unsigned int off,  ///< offset in R residual
                                   ChVectorDynamic<>& R,    ///< result: the R residual, R += c*F
                                   const double c)          ///< a scaling factor
{
    ChAssembly::IntLoadResidual_F(off, R, c);  // parent

    unsigned int displ_v = off - this->offset_w;

    if (is_modal == false) {
        for (auto& body : internal_bodylist) {
            if (body->IsActive())
                body->IntLoadResidual_F(displ_v + body->GetOffset_w(), R, c);
        }
        for (auto& link : internal_linklist) {
            if (link->IsActive())
                link->IntLoadResidual_F(displ_v + link->GetOffset_w(), R, c);
        }
        for (auto& mesh : internal_meshlist) {
            mesh->IntLoadResidual_F(displ_v + mesh->GetOffset_w(), R, c);
        }
        for (auto& item : internal_otherphysicslist) {
            item->IntLoadResidual_F(displ_v + item->GetOffset_w(), R, c);
        }
    }
    else {
        R.segment(displ_v + this->n_boundary_coords_w, this->n_modes_coords_w) += c * this->modal_F;
    }
}

void ChModalAssembly::IntLoadResidual_Mv(const unsigned int off,      ///< offset in R residual
                                    ChVectorDynamic<>& R,        ///< result: the R residual, R += c*M*v
                                    const ChVectorDynamic<>& w,  ///< the w vector
                                    const double c               ///< a scaling factor
) {
    ChAssembly::IntLoadResidual_Mv(off, R, w, c);  // parent

    unsigned int displ_v = off - this->offset_w;

    if (is_modal == false) {
        for (auto& body : internal_bodylist) {
            if (body->IsActive())
                body->IntLoadResidual_Mv(displ_v + body->GetOffset_w(), R, w, c);
        }
        for (auto& link : internal_linklist) {
            if (link->IsActive())
                link->IntLoadResidual_Mv(displ_v + link->GetOffset_w(), R, w, c);
        }
        for (auto& mesh : internal_meshlist) {
            mesh->IntLoadResidual_Mv(displ_v + mesh->GetOffset_w(), R, w, c);
        }
        for (auto& item : internal_otherphysicslist) {
            item->IntLoadResidual_Mv(displ_v + item->GetOffset_w(), R, w, c);
        }
    } 
    else {
        R.segment(displ_v + this->n_boundary_coords_w, this->n_modes_coords_w) += c * this->modal_M * w.segment(displ_v + this->n_boundary_coords_w, this->n_modes_coords_w);
    }
}

void ChModalAssembly::IntLoadResidual_CqL(const unsigned int off_L,    ///< offset in L multipliers
                                     ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
                                     const ChVectorDynamic<>& L,  ///< the L vector
                                     const double c               ///< a scaling factor
) {
    ChAssembly::IntLoadResidual_CqL(off_L, R, L, c);  // parent

    unsigned int displ_L = off_L - this->offset_L;

    if (is_modal == false) {
        for (auto& body : internal_bodylist) {
            if (body->IsActive())
                body->IntLoadResidual_CqL(displ_L + body->GetOffset_L(), R, L, c);
        }
        for (auto& link : internal_linklist) {
            if (link->IsActive())
                link->IntLoadResidual_CqL(displ_L + link->GetOffset_L(), R, L, c);
        }
        for (auto& mesh : internal_meshlist) {
            mesh->IntLoadResidual_CqL(displ_L + mesh->GetOffset_L(), R, L, c);
        }
        for (auto& item : internal_otherphysicslist) {
            item->IntLoadResidual_CqL(displ_L + item->GetOffset_L(), R, L, c);
        }
    }
}

void ChModalAssembly::IntLoadConstraint_C(const unsigned int off_L,  ///< offset in Qc residual
                                     ChVectorDynamic<>& Qc,     ///< result: the Qc residual, Qc += c*C
                                     const double c,            ///< a scaling factor
                                     bool do_clamp,             ///< apply clamping to c*C?
                                     double recovery_clamp      ///< value for min/max clamping of c*C
) {
    ChAssembly::IntLoadConstraint_C(off_L, Qc, c, do_clamp, recovery_clamp);  // parent

    unsigned int displ_L = off_L - this->offset_L;

    if (is_modal == false) {
        for (auto& body : internal_bodylist) {
            if (body->IsActive())
                body->IntLoadConstraint_C(displ_L + body->GetOffset_L(), Qc, c, do_clamp, recovery_clamp);
        }
        for (auto& link : internal_linklist) {
            if (link->IsActive())
                link->IntLoadConstraint_C(displ_L + link->GetOffset_L(), Qc, c, do_clamp, recovery_clamp);
        }
        for (auto& mesh : internal_meshlist) {
            mesh->IntLoadConstraint_C(displ_L + mesh->GetOffset_L(), Qc, c, do_clamp, recovery_clamp);
        }
        for (auto& item : internal_otherphysicslist) {
            item->IntLoadConstraint_C(displ_L + item->GetOffset_L(), Qc, c, do_clamp, recovery_clamp);
        }
    }
}

void ChModalAssembly::IntLoadConstraint_Ct(const unsigned int off_L,  ///< offset in Qc residual
                                      ChVectorDynamic<>& Qc,     ///< result: the Qc residual, Qc += c*Ct
                                      const double c             ///< a scaling factor
) {
    ChAssembly::IntLoadConstraint_Ct(off_L, Qc, c);  // parent

    unsigned int displ_L = off_L - this->offset_L;

    if (is_modal == false) {
        for (auto& body : internal_bodylist) {
            if (body->IsActive())
                body->IntLoadConstraint_Ct(displ_L + body->GetOffset_L(), Qc, c);
        }
        for (auto& link : internal_linklist) {
            if (link->IsActive())
                link->IntLoadConstraint_Ct(displ_L + link->GetOffset_L(), Qc, c);
        }
        for (auto& mesh : internal_meshlist) {
            mesh->IntLoadConstraint_Ct(displ_L + mesh->GetOffset_L(), Qc, c);
        }
        for (auto& item : internal_otherphysicslist) {
            item->IntLoadConstraint_Ct(displ_L + item->GetOffset_L(), Qc, c);
        }
    }
}

void ChModalAssembly::IntToDescriptor(const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const ChVectorDynamic<>& R,
                                 const unsigned int off_L,
                                 const ChVectorDynamic<>& L,
                                 const ChVectorDynamic<>& Qc) {

    ChAssembly::IntToDescriptor(off_v, v, R, off_L, L, Qc);  // parent

    unsigned int displ_L = off_L - this->offset_L;
    unsigned int displ_v = off_v - this->offset_w;

    if (is_modal == false) {
        for (auto& body : internal_bodylist) {
            if (body->IsActive())
                body->IntToDescriptor(displ_v + body->GetOffset_w(), v, R, displ_L + body->GetOffset_L(), L, Qc);
        }

        for (auto& link : internal_linklist) {
            if (link->IsActive())
                link->IntToDescriptor(displ_v + link->GetOffset_w(), v, R, displ_L + link->GetOffset_L(), L, Qc);
        }

        for (auto& mesh : internal_meshlist) {
            mesh->IntToDescriptor(displ_v + mesh->GetOffset_w(), v, R, displ_L + mesh->GetOffset_L(), L, Qc);
        }

        for (auto& item : internal_otherphysicslist) {
            item->IntToDescriptor(displ_v + item->GetOffset_w(), v, R, displ_L + item->GetOffset_L(), L, Qc);
        }
    } 
    else {
        this->modal_variables->Get_qb() = v.segment(displ_v + this->n_boundary_coords_w,  this->n_modes_coords_w);
        this->modal_variables->Get_fb() = R.segment(displ_v + this->n_boundary_coords_w,  this->n_modes_coords_w);
    }
}

void ChModalAssembly::IntFromDescriptor(const unsigned int off_v,
                                   ChStateDelta& v,
                                   const unsigned int off_L,
                                   ChVectorDynamic<>& L) {

    ChAssembly::IntFromDescriptor(off_v, v, off_L, L);  // parent

    unsigned int displ_L = off_L - this->offset_L;
    unsigned int displ_v = off_v - this->offset_w;

    if (is_modal == false) {
        for (auto& body : internal_bodylist) {
            if (body->IsActive())
                body->IntFromDescriptor(displ_v + body->GetOffset_w(), v, displ_L + body->GetOffset_L(), L);
        }

        for (auto& link : internal_linklist) {
            if (link->IsActive())
                link->IntFromDescriptor(displ_v + link->GetOffset_w(), v, displ_L + link->GetOffset_L(), L);
        }

        for (auto& mesh : internal_meshlist) {
            mesh->IntFromDescriptor(displ_v + mesh->GetOffset_w(), v, displ_L + mesh->GetOffset_L(), L);
        }

        for (auto& item : internal_otherphysicslist) {
            item->IntFromDescriptor(displ_v + item->GetOffset_w(), v, displ_L + item->GetOffset_L(), L);
        }
    }
    else {
        v.segment(displ_v + this->n_boundary_coords_w,  this->n_modes_coords_w) = this->modal_variables->Get_qb();
    }
}

// -----------------------------------------------------------------------------

void ChModalAssembly::InjectVariables(ChSystemDescriptor& mdescriptor) {

    ChAssembly::InjectVariables(mdescriptor);  // parent

    if (is_modal == false) {
        for (auto& body : internal_bodylist) {
            body->InjectVariables(mdescriptor);
        }
        for (auto& link : internal_linklist) {
            link->InjectVariables(mdescriptor);
        }
        for (auto& mesh : internal_meshlist) {
            mesh->InjectVariables(mdescriptor);
        }
        for (auto& item : internal_otherphysicslist) {
            item->InjectVariables(mdescriptor);
        }
    }
    else {
        mdescriptor.InsertVariables(this->modal_variables);
    }
}



void ChModalAssembly::InjectConstraints(ChSystemDescriptor& mdescriptor) {

    ChAssembly::InjectConstraints(mdescriptor);  // parent

    if (is_modal == false) {
        for (auto& body : internal_bodylist) {
            body->InjectConstraints(mdescriptor);
        }
        for (auto& link : internal_linklist) {
            link->InjectConstraints(mdescriptor);
        }
        for (auto& mesh : internal_meshlist) {
            mesh->InjectConstraints(mdescriptor);
        }
        for (auto& item : internal_otherphysicslist) {
            item->InjectConstraints(mdescriptor);
        }
    }
}



void ChModalAssembly::ConstraintsLoadJacobians() {

    ChAssembly::ConstraintsLoadJacobians();  // parent

    if (is_modal == false) {
        for (auto& body : internal_bodylist) {
            body->ConstraintsLoadJacobians();
        }
        for (auto& link : internal_linklist) {
            link->ConstraintsLoadJacobians();
        }
        for (auto& mesh : internal_meshlist) {
            mesh->ConstraintsLoadJacobians();
        }
        for (auto& item : internal_otherphysicslist) {
            item->ConstraintsLoadJacobians();
        }
    }
}

void ChModalAssembly::InjectKRMmatrices(ChSystemDescriptor& mdescriptor) {

    ChAssembly::InjectKRMmatrices(mdescriptor);  // parent

    if (is_modal == false) {
        for (auto& body : internal_bodylist) {
            body->InjectKRMmatrices(mdescriptor);
        }
        for (auto& link : internal_linklist) {
            link->InjectKRMmatrices(mdescriptor);
        }
        for (auto& mesh : internal_meshlist) {
            mesh->InjectKRMmatrices(mdescriptor);
        }
        for (auto& item : internal_otherphysicslist) {
            item->InjectKRMmatrices(mdescriptor);
        }
    }
    else {
        mdescriptor.InsertKblock(&this->modal_Hblock);
    }
}

void ChModalAssembly::KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor) {

    ChAssembly::KRMmatricesLoad(Kfactor, Rfactor, Mfactor);  // parent

    if (is_modal == false) {
        {
            for (auto& body : internal_bodylist) {
                body->KRMmatricesLoad(Kfactor, Rfactor, Mfactor);
            }
            for (auto& link : internal_linklist) {
                link->KRMmatricesLoad(Kfactor, Rfactor, Mfactor);
            }
            for (auto& mesh : internal_meshlist) {
                mesh->KRMmatricesLoad(Kfactor, Rfactor, Mfactor);
            }
            for (auto& item : internal_otherphysicslist) {
                item->KRMmatricesLoad(Kfactor, Rfactor, Mfactor);
            }
        }
    } 
    else {
        this->modal_Hblock.Get_K() = 
            this->modal_K * Kfactor + 
            this->modal_R * Rfactor + 
            this->modal_M * Mfactor;
    }
}

// -----------------------------------------------------------------------------
//  STREAMING - FILE HANDLING


void ChModalAssembly::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChModalAssembly>();

    // serialize parent class
    ChAssembly::ArchiveOUT(marchive);

    // serialize all member data:

    marchive << CHNVP(internal_bodylist, "internal_bodies");
    marchive << CHNVP(internal_linklist, "internal_links");
    marchive << CHNVP(internal_meshlist, "internal_meshes");
    marchive << CHNVP(internal_otherphysicslist, "internal_other_physics_items");
    marchive << CHNVP(is_modal, "is_modal");
    marchive << CHNVP(modal_q, "modal_q");
    marchive << CHNVP(modal_q_dt, "modal_q_dt");
    marchive << CHNVP(modal_q_dtdt, "modal_q_dtdt");
    marchive << CHNVP(modal_F, "modal_F");
}

void ChModalAssembly::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChModalAssembly>();

    // deserialize parent class
    ChAssembly::ArchiveIN(marchive);

    // stream in all member data:
    
    // trick needed because the "AddIntenal...()" functions are required
    std::vector<std::shared_ptr<ChBody>> tempbodies;
    marchive >> CHNVP(tempbodies, "internal_bodies");
    RemoveAllBodies();
    for (auto& body : tempbodies) 
        AddInternalBody(body);
    std::vector<std::shared_ptr<ChLink>> templinks;
    marchive >> CHNVP(templinks, "internal_links");
    RemoveAllLinks();
    for (auto& link : templinks) 
        AddInternalLink(link);
    std::vector<std::shared_ptr<ChMesh>> tempmeshes;
    marchive >> CHNVP(tempmeshes, "internal_mesh");
    RemoveAllMeshes();
    for (auto& mesh : tempmeshes) 
        AddInternalMesh(mesh);
    std::vector<std::shared_ptr<ChPhysicsItem>> tempotherphysics;
    marchive >> CHNVP(tempotherphysics, "internal_other_physics_items");
    RemoveAllOtherPhysicsItems();
    for (auto& mphys : tempotherphysics) 
        AddInternalOtherPhysicsItem(mphys);

    marchive >> CHNVP(is_modal, "is_modal");
    marchive >> CHNVP(modal_q, "modal_q");
    marchive >> CHNVP(modal_q_dt, "modal_q_dt");
    marchive >> CHNVP(modal_q_dtdt, "modal_q_dtdt");
    marchive >> CHNVP(modal_F, "modal_F");

    // Recompute statistics, offsets, etc.
    Setup();
}


bool ChQuadraticEigenvalueSolverNullspaceDirect::Solve(const ChSparseMatrix& M, const ChSparseMatrix& R, const ChSparseMatrix& K, const ChSparseMatrix& Cq, 
                                                        ChMatrixDynamic<std::complex<double>>& V,    ///< output matrix with eigenvectors as columns, will be resized
                                                        ChVectorDynamic<std::complex<double>>& eig,  ///< output vector with eigenvalues (real part not zero if some damping), will be resized 
                                                        int n_modes)
{
    // The folowing is adapted from the former implementation of Peng Chao

	// Compute the null space of the Cq matrix
    ChMatrixDynamic<> Cq_full = Cq; // because the fullPivLu is not available for sparse matrix in Eigen
	Eigen::MatrixXd Cq_null_space = Cq_full.fullPivLu().kernel();
	Eigen::MatrixXd M_hat = Cq_null_space.transpose() * M * Cq_null_space;
	Eigen::MatrixXd K_hat = Cq_null_space.transpose() * K * Cq_null_space;
	Eigen::MatrixXd R_hat = Cq_null_space.transpose() * R * Cq_null_space;
	
	// frequency-shift，Solve the matrix singular problem - it is not needed now, you can set it to 0
    double freq_shift = 0; // shift value, can take any value 
	Eigen::MatrixXd M_bar = M_hat;
	Eigen::MatrixXd K_bar = pow(freq_shift, 2) * M_hat + freq_shift * R_hat + K_hat;
	Eigen::MatrixXd R_bar = 2 * freq_shift * M_hat + R_hat;
	Eigen::MatrixXd M_bar_inv = M_bar.inverse();  // performance warning! dense inverse matrix! Only small sizes should be used.
	
	// Generate the A matrix of the state equation, whose eigenvalues ​​are the modal frequencies
	int dim = M_bar.rows();
	Eigen::MatrixXd A_tilde(2 * dim, 2 * dim);  
	A_tilde << Eigen::MatrixXd::Zero(dim, dim), Eigen::MatrixXd::Identity(dim, dim),
		       -M_bar_inv * K_bar             , -M_bar_inv * R_bar;
	
	// Call EIGEN3, dense matrix to directly solve the eigenvalues ​​and eigenvectors
	// NOTE：EIGEN3 has a very fast calculation speed in release mode, and a very slow calculation speed in debug mode
	Eigen::EigenSolver<Eigen::MatrixXd> eigen_solver(A_tilde);
	Eigen::VectorXcd eigen_values = eigen_solver.eigenvalues() + freq_shift;
	Eigen::MatrixXcd eigen_vectors = eigen_solver.eigenvectors();
 
    class eig_vect_and_val
    {
    public:
        Eigen::VectorXcd eigen_vect;
        std::complex<double> eigen_val;

        bool operator < (const eig_vect_and_val& str) const {
            return (eigen_val.imag() < str.eigen_val.imag());
        }
    };

	std::vector<eig_vect_and_val> all_eigen_values_and_vectors;
	for (int i = 0; i < eigen_values.size(); i++) {
        eig_vect_and_val vec_and_val{ eigen_vectors.col(i), eigen_values(i) };
        all_eigen_values_and_vectors.push_back(vec_and_val);
	}

	// sort
	std::sort(all_eigen_values_and_vectors.begin(), all_eigen_values_and_vectors.end());   // sort by imaginary part

	// organize and return results
	int middle_number = (int)(all_eigen_values_and_vectors.size() / 2);  // The eigenvalues ​​filtered out are conjugate complex roots, just take half
	int DOF_counts = (int)((all_eigen_values_and_vectors.at(0)).eigen_vect.rows()/2);  // The number of degrees of freedom in the model, used when extracting the mode shape. 

    /*
	OutputEigenResults output_temp;
	Eigen::VectorXcd vector_temp;
	REAL damping_ratio;
	int jj = 0; int i = 0; int index_mode = 0;
	while (i < mode_count) {  
		index_mode = jj + middle_number;
		damping_ratio = std::get<2>(all_eigen_values_and_vectors.at(index_mode));
		if (damping_ratio < 0.95) { // filter by damping ratio 
			std::get<0>(output_temp) = std::get<0>(all_eigen_values_and_vectors.at(index_mode));  // undamped modal freq.
			std::get<1>(output_temp) = std::get<1>(all_eigen_values_and_vectors.at(index_mode));  // damped modal freq.
			std::get<2>(output_temp) = std::get<2>(all_eigen_values_and_vectors.at(index_mode));  // damping ratio
			std::get<3>(output_temp) = std::get<3>(all_eigen_values_and_vectors.at(index_mode));  // plural eigenvalues
			vector_temp = std::get<4>(all_eigen_values_and_vectors.at(index_mode));
			std::get<4>(output_temp) = Cq_null_space * vector_temp.head(DOF_counts);// Mode shape, displacement term
			std::get<5>(output_temp) = Cq_null_space * vector_temp.tail(DOF_counts);// Mode shape, velocity term
			res_eigen_values_and_vectors.push_back(output_temp);
			
			i++;
		} 
		jj++;
	}
    */
    
    // Return values
    V.setZero(M.rows(), n_modes);
    eig.setZero(n_modes);

    for (int i = 0; i < n_modes; i++) {
        int i_half = middle_number + i; // because the n.of eigenvalues is double (conjugate pairs), so just use the 2nd half after sorting
        auto mv = all_eigen_values_and_vectors.at(i_half).eigen_vect.head(DOF_counts);
        auto mvhns = Cq_null_space * mv;

        V.col(i) = mvhns; 
        V.col(i).normalize(); // check if necessary or already normalized
        eig(i) = all_eigen_values_and_vectors.at(i_half).eigen_val;
    }

    return true;
}

}  // end namespace chrono
