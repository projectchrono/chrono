//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChMatterSPH.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include <stdlib.h>
#include <algorithm>

#include "physics/ChMatterSPH.h"
#include "physics/ChSystem.h"

#include "physics/ChProximityContainerSPH.h"
#include "collision/ChCModelBullet.h"
#include "core/ChLinearAlgebra.h"

namespace chrono {

using namespace collision;
using namespace geometry;



//////////////////////////////////////
//////////////////////////////////////

/// CLASS FOR A SPH NODE


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChNodeSPH> a_registration_ChNodeSPH;


ChNodeSPH::ChNodeSPH() {
    this->collision_model = new ChModelBullet;
    this->collision_model->SetContactable(this);
    this->container = 0;

    this->UserForce = VNULL;
    this->h_rad = 0.1;
    this->coll_rad = 0.001;
    this->SetMass(0.01);
    this->volume = 0.01;
    this->density = this->GetMass() / this->volume;
    this->pressure = 0;
}

ChNodeSPH::~ChNodeSPH() {
    delete collision_model;
}

ChNodeSPH::ChNodeSPH(const ChNodeSPH& other) : ChNodeXYZ(other) {
    this->collision_model = new ChModelBullet;
    this->collision_model->SetContactable(this);
    this->collision_model->AddPoint(other.coll_rad);
    this->container = other.container;
    this->UserForce = other.UserForce;
    this->SetKernelRadius(other.h_rad);
    this->SetCollisionRadius(other.coll_rad);
    this->SetMass(other.GetMass());
    this->volume = other.volume;
    this->density = other.density;
    this->pressure = other.pressure;

    this->variables = other.variables;
}

ChNodeSPH& ChNodeSPH::operator=(const ChNodeSPH& other) {
    if (&other == this)
        return *this;

    ChNodeXYZ::operator=(other);

    this->collision_model->ClearModel();
    this->collision_model->AddPoint(other.coll_rad);
    this->collision_model->SetContactable(this);
    this->container = other.container;
    this->UserForce = other.UserForce;
    this->SetKernelRadius(other.h_rad);
    this->SetCollisionRadius(other.coll_rad);
    this->SetMass(other.GetMass());
    this->volume = other.volume;
    this->density = other.density;

    this->variables = other.variables;

    return *this;
}

void ChNodeSPH::SetKernelRadius(double mr) {
    h_rad = mr;
    double aabb_rad = h_rad / 2;  // to avoid too many pairs: bounding boxes hemisizes will sum..  __.__--*--
    ((ChModelBullet*)this->collision_model)->SetSphereRadius(coll_rad, ChMax(0.0, aabb_rad - coll_rad));
}

void ChNodeSPH::SetCollisionRadius(double mr) {
    coll_rad = mr;
    double aabb_rad = h_rad / 2;  // to avoid too many pairs: bounding boxes hemisizes will sum..  __.__--*--
    ((ChModelBullet*)this->collision_model)->SetSphereRadius(coll_rad, ChMax(0.0, aabb_rad - coll_rad));
}

void ChNodeSPH::ContactForceLoadResidual_F(const ChVector<>& F, const ChVector<>& abs_point, ChVectorDynamic<>& R) {
    R.PasteSumVector(F, this->NodeGetOffset_w() + 0, 0);
}

void ChNodeSPH::ComputeJacobianForContactPart(const ChVector<>& abs_point,
                                              ChMatrix33<>& contact_plane,
                                              type_constraint_tuple& jacobian_tuple_N,
                                              type_constraint_tuple& jacobian_tuple_U,
                                              type_constraint_tuple& jacobian_tuple_V,
                                              bool second) {
    ChMatrix33<> Jx1;

    Jx1.CopyFromMatrixT(contact_plane);
    if (!second)
        Jx1.MatrNeg();

    jacobian_tuple_N.Get_Cq()->PasteClippedMatrix(&Jx1, 0, 0, 1, 3, 0, 0);
    jacobian_tuple_U.Get_Cq()->PasteClippedMatrix(&Jx1, 1, 0, 1, 3, 0, 0);
    jacobian_tuple_V.Get_Cq()->PasteClippedMatrix(&Jx1, 2, 0, 1, 3, 0, 0);
}

std::shared_ptr<ChMaterialSurfaceBase>& ChNodeSPH::GetMaterialSurfaceBase() {
    return container->GetMaterialSurfaceBase();
}

ChPhysicsItem* ChNodeSPH::GetPhysicsItem() {
    return container;
}

void ChNodeSPH::ArchiveOUT(ChArchiveOut& marchive)
{
    // version number
    marchive.VersionWrite(1);

    // serialize parent class
    ChNodeXYZ::ArchiveOUT(marchive);

    // serialize all member data:
    //marchive << CHNVP(container);
    marchive << CHNVP(collision_model);
    marchive << CHNVP(UserForce);
    marchive << CHNVP(volume);
    marchive << CHNVP(density);
    marchive << CHNVP(h_rad);
    marchive << CHNVP(coll_rad);
    marchive << CHNVP(pressure);
}

/// Method to allow de serialization of transient data from archives.
void ChNodeSPH::ArchiveIN(ChArchiveIn& marchive) 
{
    // version number
    int version = marchive.VersionRead();

    // deserialize parent class
    ChNodeXYZ::ArchiveIN(marchive);

    // deserialize all member data:
    //marchive >> CHNVP(container);
    marchive >> CHNVP(collision_model);
    marchive >> CHNVP(UserForce);
    marchive >> CHNVP(volume);
    marchive >> CHNVP(density);
    marchive >> CHNVP(h_rad);
    marchive >> CHNVP(coll_rad);
    marchive >> CHNVP(pressure);
}




//////////////////////////////////////
//////////////////////////////////////

/// CLASS FOR SPH MATERIAL


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChContinuumSPH> a_registration_ChContinuumSPH;

void ChContinuumSPH::ArchiveOUT(ChArchiveOut& marchive)
{
    // version number
    marchive.VersionWrite(1);

    // serialize parent class
    ChContinuumMaterial::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(viscosity);
    marchive << CHNVP(surface_tension);
    marchive << CHNVP(pressure_stiffness);
}

/// Method to allow de serialization of transient data from archives.
void ChContinuumSPH::ArchiveIN(ChArchiveIn& marchive) 
{
    // version number
    int version = marchive.VersionRead();

    // deserialize parent class
    ChContinuumMaterial::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(viscosity);
    marchive >> CHNVP(surface_tension);
    marchive >> CHNVP(pressure_stiffness);
}



//////////////////////////////////////
//////////////////////////////////////

/// CLASS FOR SPH NODE CLUSTER


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChMatterSPH> a_registration_ChMatterSPH;


ChMatterSPH::ChMatterSPH() {
    this->do_collide = false;

    this->nodes.clear();

    SetIdentifier(GetUniqueIntID());  // mark with unique ID

    // default DVI material
    matsurface = std::make_shared<ChMaterialSurface>();
}

ChMatterSPH::~ChMatterSPH() {
    // delete nodes
    this->ResizeNnodes(0);
}

void ChMatterSPH::Copy(ChMatterSPH* source) {
    // copy the parent class data...
    ChIndexedNodes::Copy(source);

    do_collide = source->do_collide;

    this->material = source->material;

    this->matsurface = source->matsurface;

    ResizeNnodes(source->GetNnodes());
}

void ChMatterSPH::ResizeNnodes(int newsize) {
    bool oldcoll = this->GetCollide();
    this->SetCollide(false);  // this will remove old particle coll.models from coll.engine, if previously added

    /*
    for (unsigned int j = 0; j < nodes.size(); j++)
    {
        //delete (this->nodes[j]);  ***not needed since using shared ptrs
        this->nodes[j] = 0;
    }
    */
    this->nodes.resize(newsize);

    for (unsigned int j = 0; j < nodes.size(); j++) {
        this->nodes[j] = std::make_shared<ChNodeSPH>();

        this->nodes[j]->SetContainer(this);

        this->nodes[j]->variables.SetUserData((void*)this);  // UserData unuseful in future cuda solver?
        //((ChModelBullet*)this->nodes[j]->collision_model)->SetContactable(this->nodes[j]);
        this->nodes[j]->collision_model->AddPoint(0.001);  //***TEST***
        this->nodes[j]->collision_model->BuildModel();
    }

    this->SetCollide(oldcoll);  // this will also add particle coll.models to coll.engine, if already in a ChSystem
}

void ChMatterSPH::AddNode(ChVector<double> initial_state) {
    auto newp = std::make_shared<ChNodeSPH>();

    newp->SetContainer(this);

    newp->SetPos(initial_state);

    this->nodes.push_back(newp);

    newp->variables.SetUserData((void*)this);  // UserData unuseful in future cuda solver?

    newp->collision_model->AddPoint(0.1);  //***TEST***
    newp->collision_model->BuildModel();   // will also add to system, if collision is on.
}

void ChMatterSPH::FillBox(const ChVector<> size,
                          const double spacing,
                          const double initial_density,
                          const ChCoordsys<> boxcoords,
                          const bool do_centeredcube,
                          const double kernel_sfactor,
                          const double randomness) {
    int samples_x = (int)(size.x / spacing);
    int samples_y = (int)(size.y / spacing);
    int samples_z = (int)(size.z / spacing);
    int totsamples = 0;

    double mrandomness = randomness;
    if (do_centeredcube)
        mrandomness = randomness * 0.5;

    for (int ix = 0; ix < samples_x; ix++)
        for (int iy = 0; iy < samples_y; iy++)
            for (int iz = 0; iz < samples_z; iz++) {
                ChVector<> pos(ix * spacing - 0.5 * size.x, iy * spacing - 0.5 * size.y, iz * spacing - 0.5 * size.z);
                pos += ChVector<>(mrandomness * ChRandom() * spacing, mrandomness * ChRandom() * spacing,
                                  mrandomness * ChRandom() * spacing);
                this->AddNode(boxcoords.TransformLocalToParent(pos));
                totsamples++;

                if (do_centeredcube) {
                    ChVector<> pos2 = pos + 0.5 * ChVector<>(spacing, spacing, spacing);
                    pos2 += ChVector<>(mrandomness * ChRandom() * spacing, mrandomness * ChRandom() * spacing,
                                       mrandomness * ChRandom() * spacing);
                    this->AddNode(boxcoords.TransformLocalToParent(pos2));
                    totsamples++;
                }
            }

    double mtotvol = size.x * size.y * size.z;
    double mtotmass = mtotvol * initial_density;
    double nodemass = mtotmass / (double)totsamples;
    double kernelrad = kernel_sfactor * spacing;

    for (unsigned int ip = 0; ip < this->GetNnodes(); ip++) {
        // downcasting
        std::shared_ptr<ChNodeSPH> mnode(this->nodes[ip]);
        assert(mnode);

        mnode->SetKernelRadius(kernelrad);
        mnode->SetCollisionRadius(spacing * 0.05);
        mnode->SetMass(nodemass);
    }

    this->GetMaterial().Set_density(initial_density);
}

//// STATE BOOKKEEPING FUNCTIONS

void ChMatterSPH::IntStateGather(const unsigned int off_x,  ///< offset in x state vector
                                 ChState& x,                ///< state vector, position part
                                 const unsigned int off_v,  ///< offset in v state vector
                                 ChStateDelta& v,           ///< state vector, speed part
                                 double& T)                 ///< time
{
    for (unsigned int j = 0; j < nodes.size(); j++) {
        x.PasteVector(this->nodes[j]->pos, off_x + 3 * j, 0);
        v.PasteVector(this->nodes[j]->pos_dt, off_v + 3 * j, 0);
    }
    T = this->GetChTime();
}

void ChMatterSPH::IntStateScatter(const unsigned int off_x,  ///< offset in x state vector
                                  const ChState& x,          ///< state vector, position part
                                  const unsigned int off_v,  ///< offset in v state vector
                                  const ChStateDelta& v,     ///< state vector, speed part
                                  const double T)            ///< time
{
    for (unsigned int j = 0; j < nodes.size(); j++) {
        this->nodes[j]->pos = x.ClipVector(off_x + 3 * j, 0);
        this->nodes[j]->pos_dt = v.ClipVector(off_v + 3 * j, 0);
    }
    this->SetChTime(T);
    this->Update();
}

void ChMatterSPH::IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    for (unsigned int j = 0; j < nodes.size(); j++) {
        a.PasteVector(this->nodes[j]->pos_dtdt, off_a + 3 * j, 0);
    }
}

void ChMatterSPH::IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    for (unsigned int j = 0; j < nodes.size(); j++) {
        this->nodes[j]->pos_dtdt = a.ClipVector(off_a + 3 * j, 0);
    }
}

void ChMatterSPH::IntLoadResidual_F(
    const unsigned int off,  ///< offset in R residual (not used here! use particle's offsets)
    ChVectorDynamic<>& R,    ///< result: the R residual, R += c*F
    const double c           ///< a scaling factor
    ) {
    // COMPUTE THE SPH FORCES HERE

    // First, find if any ChProximityContainerSPH object is present
    // in the system,

    std::shared_ptr<ChProximityContainerSPH> edges;
    std::vector<std::shared_ptr<ChPhysicsItem> >::iterator iterotherphysics = this->GetSystem()->Get_otherphysicslist()->begin();
    while (iterotherphysics != this->GetSystem()->Get_otherphysicslist()->end()) {
        if (edges = std::dynamic_pointer_cast<ChProximityContainerSPH>(*iterotherphysics))
            break;
        iterotherphysics++;
    }
    assert(edges);  // If using a ChMatterSPH, you must add also a ChProximityContainerSPH.
    if (!edges)
        return;

    // 1- Per-node initialization

    for (unsigned int j = 0; j < nodes.size(); j++) {
        this->nodes[j]->UserForce = VNULL;
        this->nodes[j]->density = 0;
    }

    // 2- Per-edge initialization and accumulation of particles's density

    edges->AccumulateStep1();

    // 3- Per-node volume and pressure computation

    for (unsigned int j = 0; j < nodes.size(); j++) {
        std::shared_ptr<ChNodeSPH> mnode(this->nodes[j]);
        assert(mnode);

        // node volume is v=mass/density
        if (mnode->density)
            mnode->volume = mnode->GetMass() / mnode->density;
        else
            mnode->volume = 0;

        // node pressure = k(dens - dens_0);
        mnode->pressure = this->material.Get_pressure_stiffness() * (mnode->density - this->material.Get_density());
    }

    // 4- Per-edge forces computation and accumulation

    edges->AccumulateStep2();

    // 5- Per-node load forces in LCP

    for (unsigned int j = 0; j < nodes.size(); j++) {
        // particle gyroscopic force:
        // none.

        // add gravity
        ChVector<> Gforce = GetSystem()->Get_G_acc() * this->nodes[j]->GetMass();
        ChVector<> TotForce = this->nodes[j]->UserForce + Gforce;

        // downcast
        std::shared_ptr<ChNodeSPH> mnode(this->nodes[j]);
        assert(mnode);

        R.PasteSumVector(TotForce * c, off + 3 * j, 0);
    }
}

void ChMatterSPH::IntLoadResidual_Mv(const unsigned int off,      ///< offset in R residual
                                     ChVectorDynamic<>& R,        ///< result: the R residual, R += c*M*v
                                     const ChVectorDynamic<>& w,  ///< the w vector
                                     const double c               ///< a scaling factor
                                     ) {
    for (unsigned int j = 0; j < nodes.size(); j++) {
        R(off + 3 * j) += c * this->nodes[j]->GetMass() * w(off + 3 * j);
        R(off + 3 * j + 1) += c * this->nodes[j]->GetMass() * w(off + 3 * j + 1);
        R(off + 3 * j + 2) += c * this->nodes[j]->GetMass() * w(off + 3 * j + 2);
    }
}

void ChMatterSPH::IntToLCP(const unsigned int off_v,  ///< offset in v, R
                           const ChStateDelta& v,
                           const ChVectorDynamic<>& R,
                           const unsigned int off_L,  ///< offset in L, Qc
                           const ChVectorDynamic<>& L,
                           const ChVectorDynamic<>& Qc) {
    for (unsigned int j = 0; j < nodes.size(); j++) {
        this->nodes[j]->variables.Get_qb().PasteClippedMatrix(&v, off_v + 3 * j, 0, 3, 1, 0, 0);
        this->nodes[j]->variables.Get_fb().PasteClippedMatrix(&R, off_v + 3 * j, 0, 3, 1, 0, 0);
    }
}

void ChMatterSPH::IntFromLCP(const unsigned int off_v,  ///< offset in v
                             ChStateDelta& v,
                             const unsigned int off_L,  ///< offset in L
                             ChVectorDynamic<>& L) {
    for (unsigned int j = 0; j < nodes.size(); j++) {
        v.PasteMatrix(&this->nodes[j]->variables.Get_qb(), off_v + 3 * j, 0);
    }
}

////
void ChMatterSPH::InjectVariables(ChLcpSystemDescriptor& mdescriptor) {
    // this->variables.SetDisabled(!this->IsActive());
    for (unsigned int j = 0; j < nodes.size(); j++) {
        mdescriptor.InsertVariables(&(this->nodes[j]->variables));
    }
}

void ChMatterSPH::VariablesFbReset() {
    for (unsigned int j = 0; j < nodes.size(); j++) {
        this->nodes[j]->variables.Get_fb().FillElem(0.0);
    }
}

void ChMatterSPH::VariablesFbLoadForces(double factor) {
    // COMPUTE THE SPH FORCES HERE

    // First, find if any ChProximityContainerSPH object is present
    // in the system,

    std::shared_ptr<ChProximityContainerSPH> edges;
    std::vector<std::shared_ptr<ChPhysicsItem> >::iterator iterotherphysics = this->GetSystem()->Get_otherphysicslist()->begin();
    while (iterotherphysics != this->GetSystem()->Get_otherphysicslist()->end()) {
        if (edges = std::dynamic_pointer_cast<ChProximityContainerSPH>(*iterotherphysics))
            break;
        iterotherphysics++;
    }
    assert(edges);  // If using a ChMatterSPH, you must add also a ChProximityContainerSPH.
    if (!edges)
        return;

    // 1- Per-node initialization

    for (unsigned int j = 0; j < nodes.size(); j++) {
        this->nodes[j]->UserForce = VNULL;
        this->nodes[j]->density = 0;
    }

    // 2- Per-edge initialization and accumulation of particles's density

    edges->AccumulateStep1();

    // 3- Per-node volume and pressure computation

    for (unsigned int j = 0; j < nodes.size(); j++) {
        std::shared_ptr<ChNodeSPH> mnode(this->nodes[j]);
        assert(mnode);

        // node volume is v=mass/density
        if (mnode->density)
            mnode->volume = mnode->GetMass() / mnode->density;
        else
            mnode->volume = 0;

        // node pressure = k(dens - dens_0);
        mnode->pressure = this->material.Get_pressure_stiffness() * (mnode->density - this->material.Get_density());
    }

    // 4- Per-edge forces computation and accumulation

    edges->AccumulateStep2();

    // 5- Per-node load forces in LCP

    for (unsigned int j = 0; j < nodes.size(); j++) {
        // particle gyroscopic force:
        // none.

        // add gravity
        ChVector<> Gforce = GetSystem()->Get_G_acc() * this->nodes[j]->GetMass();
        ChVector<> TotForce = this->nodes[j]->UserForce + Gforce;

        // downcast
        std::shared_ptr<ChNodeSPH> mnode(this->nodes[j]);
        assert(mnode);

        mnode->variables.Get_fb().PasteSumVector(TotForce * factor, 0, 0);
    }
}

void ChMatterSPH::VariablesQbLoadSpeed() {
    for (unsigned int j = 0; j < nodes.size(); j++) {
        // set current speed in 'qb', it can be used by the LCP solver when working in incremental mode
        this->nodes[j]->variables.Get_qb().PasteVector(this->nodes[j]->GetPos_dt(), 0, 0);
    }
}

void ChMatterSPH::VariablesFbIncrementMq() {
    for (unsigned int j = 0; j < nodes.size(); j++) {
        this->nodes[j]->variables.Compute_inc_Mb_v(this->nodes[j]->variables.Get_fb(),
                                                   this->nodes[j]->variables.Get_qb());
    }
}

void ChMatterSPH::VariablesQbSetSpeed(double step) {
    for (unsigned int j = 0; j < nodes.size(); j++) {
        ChVector<> old_pos_dt = this->nodes[j]->GetPos_dt();

        // from 'qb' vector, sets body speed, and updates auxiliary data
        this->nodes[j]->SetPos_dt(this->nodes[j]->variables.Get_qb().ClipVector(0, 0));

        // Compute accel. by BDF (approximate by differentiation);
        if (step) {
            this->nodes[j]->SetPos_dtdt((this->nodes[j]->GetPos_dt() - old_pos_dt) / step);
        }
    }
}

void ChMatterSPH::VariablesQbIncrementPosition(double dt_step) {
    // if (!this->IsActive())
    //	return;

    for (unsigned int j = 0; j < nodes.size(); j++) {
        // Updates position with incremental action of speed contained in the
        // 'qb' vector:  pos' = pos + dt * speed   , like in an Eulero step.

        ChVector<> newspeed = this->nodes[j]->variables.Get_qb().ClipVector(0, 0);

        // ADVANCE POSITION: pos' = pos + dt * vel
        this->nodes[j]->SetPos(this->nodes[j]->GetPos() + newspeed * dt_step);
    }
}

//////////////

void ChMatterSPH::SetNoSpeedNoAcceleration() {
    for (unsigned int j = 0; j < nodes.size(); j++) {
        this->nodes[j]->SetPos_dt(VNULL);
        this->nodes[j]->SetPos_dtdt(VNULL);
    }
}

//////

void ChMatterSPH::Update(bool update_assets) {
    ChMatterSPH::Update(this->GetChTime(), update_assets);
}

void ChMatterSPH::Update(double mytime, bool update_assets) {
    // Inherit time changes of parent class
    ChPhysicsItem::Update(mytime, update_assets);

    // TrySleeping();    // See if the body can fall asleep; if so, put it to sleeping
    // ClampSpeed();     // Apply limits (if in speed clamping mode) to speeds.
}

// collision stuff
void ChMatterSPH::SetCollide(bool mcoll) {
    if (mcoll == this->do_collide)
        return;

    if (mcoll) {
        this->do_collide = true;
        if (GetSystem()) {
            for (unsigned int j = 0; j < nodes.size(); j++) {
                GetSystem()->GetCollisionSystem()->Add(this->nodes[j]->collision_model);
            }
        }
    } else {
        this->do_collide = false;
        if (GetSystem()) {
            for (unsigned int j = 0; j < nodes.size(); j++) {
                GetSystem()->GetCollisionSystem()->Remove(this->nodes[j]->collision_model);
            }
        }
    }
}

void ChMatterSPH::SyncCollisionModels() {
    for (unsigned int j = 0; j < nodes.size(); j++) {
        this->nodes[j]->collision_model->SyncPosition();
    }
}

void ChMatterSPH::AddCollisionModelsToSystem() {
    assert(this->GetSystem());
    SyncCollisionModels();
    for (unsigned int j = 0; j < nodes.size(); j++) {
        this->GetSystem()->GetCollisionSystem()->Add(this->nodes[j]->collision_model);
    }
}

void ChMatterSPH::RemoveCollisionModelsFromSystem() {
    assert(this->GetSystem());
    for (unsigned int j = 0; j < nodes.size(); j++) {
        this->GetSystem()->GetCollisionSystem()->Remove(this->nodes[j]->collision_model);
    }
}

////

void ChMatterSPH::UpdateParticleCollisionModels() {
    for (unsigned int j = 0; j < nodes.size(); j++) {
        this->nodes[j]->collision_model->ClearModel();
        //***TO DO*** UPDATE RADIUS OF SPHERE?
        // this->nodes[j]->collision_model->AddCopyOfAnotherModel(this->particle_collision_model);
        this->nodes[j]->collision_model->BuildModel();
    }
}

//////// FILE I/O

void ChMatterSPH::ArchiveOUT(ChArchiveOut& marchive)
{
    // version number
    marchive.VersionWrite(1);

    // serialize parent class
    ChIndexedNodes::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(material);
    marchive << CHNVP(matsurface);
    marchive << CHNVP(do_collide);
    marchive << CHNVP(nodes);
}

/// Method to allow de serialization of transient data from archives.
void ChMatterSPH::ArchiveIN(ChArchiveIn& marchive) 
{
    // version number
    int version = marchive.VersionRead();

    // deserialize parent class
    ChIndexedNodes::ArchiveIN(marchive);

    // deserialize all member data:
    RemoveCollisionModelsFromSystem();

    marchive >> CHNVP(material);
    marchive >> CHNVP(matsurface);
    marchive >> CHNVP(do_collide);
    marchive >> CHNVP(nodes);

    for (unsigned int j = 0; j < nodes.size(); j++) {
        this->nodes[j]->SetContainer(this);
    }
    AddCollisionModelsToSystem();
}


}  // END_OF_NAMESPACE____

/////////////////////
