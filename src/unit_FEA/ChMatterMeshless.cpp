//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//
// File authors: Alessandro Tasora

#include <stdlib.h>
#include <algorithm>

#include "ChMatterMeshless.h"
#include "ChProximityContainerMeshless.h"
#include "physics/ChSystem.h"
#include "collision/ChCModelBullet.h"
#include "core/ChLinearAlgebra.h"

namespace chrono {
namespace fea {

using namespace collision;
using namespace geometry;

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChMatterMeshless> a_registration_ChMatterMeshless;

//////////////////////////////////////
//////////////////////////////////////

/// CLASS FOR A MESHLESS NODE

ChNodeMeshless::ChNodeMeshless() {
    this->collision_model = new ChModelBullet;
    this->collision_model->SetContactable(this);

    this->pos_ref = VNULL;
    this->UserForce = VNULL;
    this->h_rad = 0.1;
    this->coll_rad = 0.001;
    this->SetMass(0.01);
    this->volume = 0.01;
    this->density = this->GetMass() / this->volume;
    this->hardening = 0;
    this->container = 0;
}

ChNodeMeshless::~ChNodeMeshless() {
    delete collision_model;
}

ChNodeMeshless::ChNodeMeshless(const ChNodeMeshless& other) : ChNodeXYZ(other) {
    this->collision_model = new ChModelBullet;
    this->collision_model->SetContactable(this);
    this->collision_model->AddPoint(other.coll_rad);

    this->pos_ref = other.pos_ref;
    this->UserForce = other.UserForce;
    this->SetKernelRadius(other.h_rad);
    this->SetCollisionRadius(other.coll_rad);
    this->SetMass(other.GetMass());
    this->volume = other.volume;
    this->density = other.density;
    this->hardening = other.hardening;

    this->t_strain = other.t_strain;
    this->p_strain = other.p_strain;
    this->e_strain = other.e_strain;
    this->e_stress = other.e_stress;

    this->container = other.container;

    this->variables = other.variables;
}

ChNodeMeshless& ChNodeMeshless::operator=(const ChNodeMeshless& other) {
    if (&other == this)
        return *this;

    ChNodeXYZ::operator=(other);

    this->collision_model->ClearModel();
    this->collision_model->AddPoint(other.coll_rad);

    this->collision_model->SetContactable(this);

    this->pos_ref = other.pos_ref;
    this->UserForce = other.UserForce;
    this->SetKernelRadius(other.h_rad);
    this->SetCollisionRadius(other.coll_rad);
    this->SetMass(other.GetMass());
    this->volume = other.volume;
    this->density = other.density;
    this->hardening = other.hardening;

    this->t_strain = other.t_strain;
    this->p_strain = other.p_strain;
    this->e_strain = other.e_strain;
    this->e_stress = other.e_stress;

    this->container = other.container;

    this->variables = other.variables;

    return *this;
}

void ChNodeMeshless::SetKernelRadius(double mr) {
    h_rad = mr;
    double aabb_rad = h_rad / 2;  // to avoid too many pairs: bounding boxes hemisizes will sum..  __.__--*--
    ((ChModelBullet*)this->collision_model)->SetSphereRadius(coll_rad, ChMax(0.0, aabb_rad - coll_rad));
}

void ChNodeMeshless::SetCollisionRadius(double mr) {
    coll_rad = mr;
    double aabb_rad = h_rad / 2;  // to avoid too many pairs: bounding boxes hemisizes will sum..  __.__--*--
    ((ChModelBullet*)this->collision_model)->SetSphereRadius(coll_rad, ChMax(0.0, aabb_rad - coll_rad));
}

void ChNodeMeshless::ContactForceLoadResidual_F(const ChVector<>& F, const ChVector<>& abs_point, 
                                    ChVectorDynamic<>& R) {
    R.PasteSumVector(F, this->NodeGetOffset_w() + 0, 0);
}

void ChNodeMeshless::ComputeJacobianForContactPart(const ChVector<>& abs_point, ChMatrix33<>& contact_plane, 
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

ChSharedPtr<ChMaterialSurfaceBase>& ChNodeMeshless::GetMaterialSurfaceBase()
{
    return container->GetMaterialSurfaceBase();
}

ChPhysicsItem* ChNodeMeshless::GetPhysicsItem()
{
    return container;
}

//////////////////////////////////////
//////////////////////////////////////

/// CLASS FOR Meshless NODE CLUSTER

ChMatterMeshless::ChMatterMeshless() {
    this->do_collide = false;

    // By default, make a VonMises material
    ChSharedPtr<ChContinuumPlasticVonMises> defaultmaterial(new ChContinuumPlasticVonMises);
    this->material = defaultmaterial;

    this->viscosity = 0.0;

    this->nodes.clear();

    // default DVI material
    matsurface = ChSharedPtr<ChMaterialSurface>(new ChMaterialSurface);

    SetIdentifier(GetUniqueIntID());  // mark with unique ID
}

ChMatterMeshless::~ChMatterMeshless() {
    // delete nodes
    this->ResizeNnodes(0);
}

void ChMatterMeshless::Copy(ChMatterMeshless* source) {
    // copy the parent class data...
    ChIndexedNodes::Copy(source);

    do_collide = source->do_collide;

    this->matsurface = source->matsurface;

    ResizeNnodes(source->GetNnodes());
}

void ChMatterMeshless::ReplaceMaterial(ChSharedPtr<ChContinuumElastoplastic> newmaterial) {
    this->material = newmaterial;
}

void ChMatterMeshless::ResizeNnodes(int newsize) {
    bool oldcoll = this->GetCollide();
    this->SetCollide(false);  // this will remove old particle coll.models from coll.engine, if previously added

    /*
    for (unsigned int j = 0; j < nodes.size(); j++)
    {
        // delete (this->nodes[j]); // *** not needed since shared ptrs
        this->nodes[j] = 0;
    }
    */

    this->nodes.resize(newsize);

    for (unsigned int j = 0; j < nodes.size(); j++) {
        this->nodes[j] = ChSharedPtr<ChNodeMeshless>(new ChNodeMeshless);

        this->nodes[j]->variables.SetUserData((void*)this);  // UserData unuseful in future cuda solver?

        this->nodes[j]->collision_model->AddPoint(0.001);  //***TEST***
        this->nodes[j]->collision_model->BuildModel();
    }

    this->SetCollide(oldcoll);  // this will also add particle coll.models to coll.engine, if already in a ChSystem
}

void ChMatterMeshless::AddNode(ChVector<double> initial_state) {
    ChSharedPtr<ChNodeMeshless> newp(new ChNodeMeshless);

    newp->SetPos(initial_state);
    newp->SetPosReference(initial_state);

    this->nodes.push_back(newp);

    newp->SetMatterContainer(this);

    newp->variables.SetUserData((void*)this);  // UserData unuseful in future cuda solver?

    newp->collision_model->AddPoint(0.1);  //***TEST***
    newp->collision_model->BuildModel();    // will also add to system, if collision is on.
}

void ChMatterMeshless::FillBox(const ChVector<> size,
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
        ChSharedPtr<ChNodeMeshless> mnode(this->nodes[ip]);
        assert(!mnode.IsNull());

        mnode->SetKernelRadius(kernelrad);
        mnode->SetCollisionRadius(spacing * 0.1);
        mnode->SetMass(nodemass);
    }

    this->GetMaterial()->Set_density(initial_density);
}

//// STATE BOOKKEEPING FUNCTIONS

void ChMatterMeshless::IntStateGather(const unsigned int off_x,  ///< offset in x state vector
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

void ChMatterMeshless::IntStateScatter(const unsigned int off_x,  ///< offset in x state vector
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

void ChMatterMeshless::IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    for (unsigned int j = 0; j < nodes.size(); j++) {
        a.PasteVector(this->nodes[j]->pos_dtdt, off_a + 3 * j, 0);
    }
}

void ChMatterMeshless::IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    for (unsigned int j = 0; j < nodes.size(); j++) {
        this->nodes[j]->SetPos_dtdt(a.ClipVector(off_a + 3 * j, 0));
    }
}

void ChMatterMeshless::IntLoadResidual_F(
    const unsigned int off,  ///< offset in R residual (not used here! use particle's offsets)
    ChVectorDynamic<>& R,    ///< result: the R residual, R += c*F
    const double c           ///< a scaling factor
    ) {
    // COMPUTE THE MESHLESS FORCES HERE

    // First, find if any ChProximityContainerMeshless object is present
    // in the system,

    ChProximityContainerMeshless* edges = 0;
    std::vector<ChPhysicsItem*>::iterator iterotherphysics = this->GetSystem()->Get_otherphysicslist()->begin();
    while (iterotherphysics != this->GetSystem()->Get_otherphysicslist()->end()) {
        if (edges = dynamic_cast<ChProximityContainerMeshless*>(*iterotherphysics))
            break;
        iterotherphysics++;
    }
    assert(edges);  // If using a ChMatterMeshless, you must add also a ChProximityContainerMeshless.

    // 1- Per-node initialization

    for (unsigned int j = 0; j < nodes.size(); j++) {
        this->nodes[j]->J.FillElem(0.0);
        this->nodes[j]->Amoment.FillElem(0.0);
        this->nodes[j]->t_strain.FillElem(0.0);
        this->nodes[j]->e_stress.FillElem(0.0);
        this->nodes[j]->UserForce = VNULL;
        this->nodes[j]->density = 0;
    }

    // 2- Per-edge initialization and accumulation of values in particles's J, Amoment, m_v, density

    edges->AccumulateStep1();

    // 3- Per-node inversion of A and computation of strain stress

    for (unsigned int j = 0; j < nodes.size(); j++) {
        ChSharedPtr<ChNodeMeshless> mnode(this->nodes[j]);
        assert(!mnode.IsNull());

        // node volume is v=mass/density
        if (mnode->density > 0)
            mnode->volume = mnode->GetMass() / mnode->density;
        else
            mnode->volume = 0;

        // Compute A inverse
        ChMatrix33<> M_tmp = mnode->Amoment;
        double det = M_tmp.FastInvert(&mnode->Amoment);
        if (fabs(det) < 0.00003) {
            mnode->Amoment.FillElem(0);     // deactivate if not possible to invert
            mnode->e_strain.FillElem(0.0);  // detach
        } else {
            // Compute J = ( A^-1 * [dwg | dwg | dwg] )' + I
            M_tmp.MatrMultiply(mnode->Amoment, mnode->J);
            M_tmp.Element(0, 0) += 1;
            M_tmp.Element(1, 1) += 1;
            M_tmp.Element(2, 2) += 1;
            mnode->J.CopyFromMatrixT(M_tmp);

            // Compute step strain tensor  de = J'*J - I
            ChMatrix33<> mtensor;
            mtensor.MatrMultiply(M_tmp, mnode->J);
            mtensor.Element(0, 0) -= 1;
            mtensor.Element(1, 1) -= 1;
            mtensor.Element(2, 2) -= 1;

            mnode->t_strain.ConvertFromMatrix(mtensor);  // store 'step strain' de, change in total strain

            ChStrainTensor<> strainplasticflow;
            this->material->ComputeReturnMapping(strainplasticflow,   // dEp, flow of elastic strain (correction)
                                                 nodes[j]->t_strain,  // increment of total strain
                                                 nodes[j]->e_strain,  // last elastic strain
                                                 nodes[j]->p_strain   // last plastic strain
                                                 );
            ChStrainTensor<> proj_e_strain;
            proj_e_strain.MatrSub(nodes[j]->e_strain, strainplasticflow);
            proj_e_strain.MatrInc(nodes[j]->t_strain);
            this->GetMaterial()->ComputeElasticStress(mnode->e_stress, proj_e_strain);
            mnode->e_stress.ConvertToMatrix(mtensor);

            /*
            // Compute elastic stress tensor  sigma= C*epsilon
            //   NOTE: it should be better to perform stress computation on corrected e_strain, _after_ the
            //   return mapping (see later), but for small timestep it could be the same.
            ChStrainTensor<> guesstot_e_strain; // anticipate the computation of total strain for anticipating strains
            guesstot_e_strain.MatrAdd(mnode->e_strain, mnode->t_strain);
            this->GetMaterial()->ComputeElasticStress(mnode->e_stress, guesstot_e_strain);
            mnode->e_stress.ConvertToMatrix(mtensor);
            */

            // Precompute 2*v*J*sigma*A^-1
            mnode->FA = mnode->J * (mtensor * (mnode->Amoment));
            mnode->FA.MatrScale(2 * mnode->volume);
        }
    }

    // 4- Per-edge force transfer from stress, and add also viscous forces

    edges->AccumulateStep2();

    // 5- Per-node load force

    for (unsigned int j = 0; j < nodes.size(); j++) {
        // particle gyroscopic force:
        // none.

        // add gravity
        ChVector<> Gforce = GetSystem()->Get_G_acc() * this->nodes[j]->GetMass();
        ChVector<> TotForce = this->nodes[j]->UserForce + Gforce;

        ChSharedPtr<ChNodeMeshless> mnode(this->nodes[j]);
        assert(!mnode.IsNull());

        R.PasteSumVector(TotForce * c, off + 3 * j, 0);
    }
}

void ChMatterMeshless::IntLoadResidual_Mv(const unsigned int off,      ///< offset in R residual
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

void ChMatterMeshless::IntToLCP(const unsigned int off_v,  ///< offset in v, R
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

void ChMatterMeshless::IntFromLCP(const unsigned int off_v,  ///< offset in v
                                  ChStateDelta& v,
                                  const unsigned int off_L,  ///< offset in L
                                  ChVectorDynamic<>& L) {
    for (unsigned int j = 0; j < nodes.size(); j++) {
        v.PasteMatrix(&this->nodes[j]->variables.Get_qb(), off_v + 3 * j, 0);
    }
}

////
void ChMatterMeshless::InjectVariables(ChLcpSystemDescriptor& mdescriptor) {
    // this->variables.SetDisabled(!this->IsActive());
    for (unsigned int j = 0; j < nodes.size(); j++) {
        mdescriptor.InsertVariables(&(this->nodes[j]->variables));
    }
}

void ChMatterMeshless::VariablesFbReset() {
    for (unsigned int j = 0; j < nodes.size(); j++) {
        this->nodes[j]->variables.Get_fb().FillElem(0.0);
    }
}

void ChMatterMeshless::VariablesFbLoadForces(double factor) {
    // COMPUTE THE MESHLESS FORCES HERE

    // First, find if any ChProximityContainerMeshless object is present
    // in the system,

    ChProximityContainerMeshless* edges = 0;
    std::vector<ChPhysicsItem*>::iterator iterotherphysics = this->GetSystem()->Get_otherphysicslist()->begin();
    while (iterotherphysics != this->GetSystem()->Get_otherphysicslist()->end()) {
        if (edges = dynamic_cast<ChProximityContainerMeshless*>(*iterotherphysics))
            break;
        iterotherphysics++;
    }
    assert(edges);  // If using a ChMatterMeshless, you must add also a ChProximityContainerMeshless.

    // 1- Per-node initialization

    for (unsigned int j = 0; j < nodes.size(); j++) {
        this->nodes[j]->J.FillElem(0.0);
        this->nodes[j]->Amoment.FillElem(0.0);
        this->nodes[j]->t_strain.FillElem(0.0);
        this->nodes[j]->e_stress.FillElem(0.0);
        this->nodes[j]->UserForce = VNULL;
        this->nodes[j]->density = 0;
    }

    // 2- Per-edge initialization and accumulation of values in particles's J, Amoment, m_v, density

    edges->AccumulateStep1();

    // 3- Per-node inversion of A and computation of strain stress

    for (unsigned int j = 0; j < nodes.size(); j++) {
        ChSharedPtr<ChNodeMeshless> mnode(this->nodes[j]);
        assert(!mnode.IsNull());

        // node volume is v=mass/density
        if (mnode->density > 0)
            mnode->volume = mnode->GetMass() / mnode->density;
        else
            mnode->volume = 0;

        // Compute A inverse
        ChMatrix33<> M_tmp = mnode->Amoment;
        double det = M_tmp.FastInvert(&mnode->Amoment);
        if (fabs(det) < 0.00003) {
            mnode->Amoment.FillElem(0);     // deactivate if not possible to invert
            mnode->e_strain.FillElem(0.0);  // detach
        } else {
            // Compute J = ( A^-1 * [dwg | dwg | dwg] )' + I
            M_tmp.MatrMultiply(mnode->Amoment, mnode->J);
            M_tmp.Element(0, 0) += 1;
            M_tmp.Element(1, 1) += 1;
            M_tmp.Element(2, 2) += 1;
            mnode->J.CopyFromMatrixT(M_tmp);

            // Compute step strain tensor  de = J'*J - I
            ChMatrix33<> mtensor;
            mtensor.MatrMultiply(M_tmp, mnode->J);
            mtensor.Element(0, 0) -= 1;
            mtensor.Element(1, 1) -= 1;
            mtensor.Element(2, 2) -= 1;

            mnode->t_strain.ConvertFromMatrix(mtensor);  // store 'step strain' de, change in total strain

            ChStrainTensor<> strainplasticflow;
            this->material->ComputeReturnMapping(strainplasticflow,   // dEp, flow of elastic strain (correction)
                                                 nodes[j]->t_strain,  // increment of total strain
                                                 nodes[j]->e_strain,  // last elastic strain
                                                 nodes[j]->p_strain   // last plastic strain
                                                 );
            ChStrainTensor<> proj_e_strain;
            proj_e_strain.MatrSub(nodes[j]->e_strain, strainplasticflow);
            proj_e_strain.MatrInc(nodes[j]->t_strain);
            this->GetMaterial()->ComputeElasticStress(mnode->e_stress, proj_e_strain);
            mnode->e_stress.ConvertToMatrix(mtensor);

            /*
            // Compute elastic stress tensor  sigma= C*epsilon
            //   NOTE: it should be better to perform stress computation on corrected e_strain, _after_ the
            //   return mapping (see later), but for small timestep it could be the same.
            ChStrainTensor<> guesstot_e_strain; // anticipate the computation of total strain for anticipating strains
            guesstot_e_strain.MatrAdd(mnode->e_strain, mnode->t_strain);
            this->GetMaterial()->ComputeElasticStress(mnode->e_stress, guesstot_e_strain);
            mnode->e_stress.ConvertToMatrix(mtensor);
            */

            // Precompute 2*v*J*sigma*A^-1
            mnode->FA = mnode->J * (mtensor * (mnode->Amoment));
            mnode->FA.MatrScale(2 * mnode->volume);
        }
    }

    // 4- Per-edge force transfer from stress, and add also viscous forces

    edges->AccumulateStep2();

    // 5- Per-node load force

    for (unsigned int j = 0; j < nodes.size(); j++) {
        // particle gyroscopic force:
        // none.

        // add gravity
        ChVector<> Gforce = GetSystem()->Get_G_acc() * this->nodes[j]->GetMass();
        ChVector<> TotForce = this->nodes[j]->UserForce + Gforce;

        ChSharedPtr<ChNodeMeshless> mnode(this->nodes[j]);
        assert(!mnode.IsNull());

        mnode->variables.Get_fb().PasteSumVector(TotForce * factor, 0, 0);
    }
}

void ChMatterMeshless::VariablesFbIncrementMq() {
    for (unsigned int j = 0; j < nodes.size(); j++) {
        this->nodes[j]->variables.Compute_inc_Mb_v(this->nodes[j]->variables.Get_fb(),
                                                   this->nodes[j]->variables.Get_qb());
    }
}

void ChMatterMeshless::VariablesQbLoadSpeed() {
    for (unsigned int j = 0; j < nodes.size(); j++) {
        // set current speed in 'qb', it can be used by the LCP solver when working in incremental mode
        this->nodes[j]->variables.Get_qb().PasteVector(this->nodes[j]->GetPos_dt(), 0, 0);
    }
}

void ChMatterMeshless::VariablesQbSetSpeed(double step) {
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

void ChMatterMeshless::VariablesQbIncrementPosition(double dt_step) {
    // if (!this->IsActive())
    //	return;

    for (unsigned int j = 0; j < nodes.size(); j++) {
        ChSharedPtr<ChNodeMeshless> mnode(this->nodes[j]);
        assert(!mnode.IsNull());

        // Integrate plastic flow
        ChStrainTensor<> strainplasticflow;
        this->material->ComputeReturnMapping(strainplasticflow,   // dEp, flow of elastic strain (correction)
                                             nodes[j]->t_strain,  // increment of total strain
                                             nodes[j]->e_strain,  // last elastic strain
                                             nodes[j]->p_strain   // last plastic strain
                                             );
        double dtpfact = dt_step * this->material->Get_flow_rate();
        if (dtpfact > 1.0)
            dtpfact = 1.0;  // clamp if dt is larger than plastic flow duration

        this->nodes[j]->p_strain.MatrInc(strainplasticflow * dtpfact);

        // Increment total elastic tensor and proceed for next step
        this->nodes[j]->pos_ref = this->nodes[j]->pos;
        this->nodes[j]->e_strain.MatrInc(this->nodes[j]->t_strain);
        //	this->nodes[j]->e_strain.MatrDec(strainplasticflow*dtpfact);
        this->nodes[j]->t_strain.FillElem(0.0);  // unuseful? will be overwritten anyway
    }

    for (unsigned int j = 0; j < nodes.size(); j++) {
        // Updates position with incremental action of speed contained in the
        // 'qb' vector:  pos' = pos + dt * speed   , like in an Eulero step.

        ChVector<> newspeed = this->nodes[j]->variables.Get_qb().ClipVector(0, 0);

        // ADVANCE POSITION: pos' = pos + dt * vel
        this->nodes[j]->SetPos(this->nodes[j]->GetPos() + newspeed * dt_step);
    }
}

//////////////

void ChMatterMeshless::SetNoSpeedNoAcceleration() {
    for (unsigned int j = 0; j < nodes.size(); j++) {
        this->nodes[j]->SetPos_dt(VNULL);
        this->nodes[j]->SetPos_dtdt(VNULL);
    }
}

//////

void ChMatterMeshless::Update(bool update_assets) {
    ChMatterMeshless::Update(this->GetChTime(), update_assets);
}

void ChMatterMeshless::Update(double mytime, bool update_assets) {
    // Inherit time changes of parent class
    ChPhysicsItem::Update(mytime, update_assets);

    // TrySleeping();    // See if the body can fall asleep; if so, put it to sleeping
    // ClampSpeed();     // Apply limits (if in speed clamping mode) to speeds.
}

// collision stuff
void ChMatterMeshless::SetCollide(bool mcoll) {
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

void ChMatterMeshless::SyncCollisionModels() {
    for (unsigned int j = 0; j < nodes.size(); j++) {
        this->nodes[j]->collision_model->SyncPosition();
    }
}

void ChMatterMeshless::AddCollisionModelsToSystem() {
    assert(this->GetSystem());
    SyncCollisionModels();
    for (unsigned int j = 0; j < nodes.size(); j++) {
        this->GetSystem()->GetCollisionSystem()->Add(this->nodes[j]->collision_model);
    }
}

void ChMatterMeshless::RemoveCollisionModelsFromSystem() {
    assert(this->GetSystem());
    for (unsigned int j = 0; j < nodes.size(); j++) {
        this->GetSystem()->GetCollisionSystem()->Remove(this->nodes[j]->collision_model);
    }
}

////

void ChMatterMeshless::UpdateParticleCollisionModels() {
    for (unsigned int j = 0; j < nodes.size(); j++) {
        this->nodes[j]->collision_model->ClearModel();
        //***TO DO*** UPDATE RADIUS OF MeshlessERE?
        // this->nodes[j]->collision_model->AddCopyOfAnotherModel(this->particle_collision_model);
        this->nodes[j]->collision_model->BuildModel();
    }
}

//////// FILE I/O

void ChMatterMeshless::StreamOUT(ChStreamOutBinary& mstream) {
    // class version number
    mstream.VersionWrite(1);

    // serialize parent class too
    ChIndexedNodes::StreamOUT(mstream);

    // stream out all member data
    mstream.AbstractWrite(this->material.get_ptr());

    //***TO DO*** stream nodes
}

void ChMatterMeshless::StreamIN(ChStreamInBinary& mstream) {
    // class version number
    int version = mstream.VersionRead();

    // deserialize parent class too
    ChIndexedNodes::StreamIN(mstream);

    // stream in all member data
    ChContinuumElastoplastic* mmat;
    mstream.AbstractReadCreate(&mmat);
    this->material = ChSharedPtr<ChContinuumElastoplastic>(mmat);

    //***TO DO*** unstream nodes
}

}  // END_OF_NAMESPACE____
}  // END_OF_NAMESPACE____

/////////////////////
