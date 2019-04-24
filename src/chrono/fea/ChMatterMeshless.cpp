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

#include "chrono/collision/ChCModelBullet.h"
#include "chrono/core/ChLinearAlgebra.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/fea/ChMatterMeshless.h"
#include "chrono/fea/ChProximityContainerMeshless.h"

namespace chrono {
namespace fea {

using namespace collision;
using namespace geometry;

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChMatterMeshless)

ChNodeMeshless::ChNodeMeshless()
    : pos_ref(VNULL), UserForce(VNULL), h_rad(0.1), coll_rad(0.001), volume(0.01), hardening(0), container(NULL) {
    collision_model = new ChModelBullet;
    collision_model->SetContactable(this);

    SetMass(0.01);
    density = GetMass() / volume;
}

ChNodeMeshless::ChNodeMeshless(const ChNodeMeshless& other) : ChNodeXYZ(other) {
    collision_model = new ChModelBullet;
    collision_model->SetContactable(this);
    collision_model->AddPoint(other.coll_rad);

    pos_ref = other.pos_ref;
    UserForce = other.UserForce;
    SetKernelRadius(other.h_rad);
    SetCollisionRadius(other.coll_rad);
    SetMass(other.GetMass());
    volume = other.volume;
    density = other.density;
    hardening = other.hardening;

    t_strain = other.t_strain;
    p_strain = other.p_strain;
    e_strain = other.e_strain;
    e_stress = other.e_stress;

    container = other.container;

    variables = other.variables;
}

ChNodeMeshless::~ChNodeMeshless() {
    delete collision_model;
}

ChNodeMeshless& ChNodeMeshless::operator=(const ChNodeMeshless& other) {
    if (&other == this)
        return *this;

    ChNodeXYZ::operator=(other);

    collision_model->ClearModel();
    collision_model->AddPoint(other.coll_rad);

    collision_model->SetContactable(this);

    pos_ref = other.pos_ref;
    UserForce = other.UserForce;
    SetKernelRadius(other.h_rad);
    SetCollisionRadius(other.coll_rad);
    SetMass(other.GetMass());
    volume = other.volume;
    density = other.density;
    hardening = other.hardening;

    t_strain = other.t_strain;
    p_strain = other.p_strain;
    e_strain = other.e_strain;
    e_stress = other.e_stress;

    container = other.container;

    variables = other.variables;

    return *this;
}

void ChNodeMeshless::SetKernelRadius(double mr) {
    h_rad = mr;
    double aabb_rad = h_rad / 2;  // to avoid too many pairs: bounding boxes hemisizes will sum..  __.__--*--
    ((ChModelBullet*)collision_model)->SetSphereRadius(coll_rad, ChMax(0.0, aabb_rad - coll_rad));
}

void ChNodeMeshless::SetCollisionRadius(double mr) {
    coll_rad = mr;
    double aabb_rad = h_rad / 2;  // to avoid too many pairs: bounding boxes hemisizes will sum..  __.__--*--
    ((ChModelBullet*)collision_model)->SetSphereRadius(coll_rad, ChMax(0.0, aabb_rad - coll_rad));
}

void ChNodeMeshless::ContactForceLoadResidual_F(const ChVector<>& F,
                                                const ChVector<>& abs_point,
                                                ChVectorDynamic<>& R) {
    R.PasteSumVector(F, NodeGetOffset_w() + 0, 0);
}

void ChNodeMeshless::ComputeJacobianForContactPart(const ChVector<>& abs_point,
                                                   ChMatrix33<>& contact_plane,
                                                   type_constraint_tuple& jacobian_tuple_N,
                                                   type_constraint_tuple& jacobian_tuple_U,
                                                   type_constraint_tuple& jacobian_tuple_V,
                                                   bool second) {
    ChMatrix33<> Jx1;

    Jx1.CopyFromMatrixT(contact_plane);
    if (!second)
        Jx1.MatrNeg();

    jacobian_tuple_N.Get_Cq()->PasteClippedMatrix(Jx1, 0, 0, 1, 3, 0, 0);
    jacobian_tuple_U.Get_Cq()->PasteClippedMatrix(Jx1, 1, 0, 1, 3, 0, 0);
    jacobian_tuple_V.Get_Cq()->PasteClippedMatrix(Jx1, 2, 0, 1, 3, 0, 0);
}

std::shared_ptr<ChMaterialSurface>& ChNodeMeshless::GetMaterialSurface() {
    return container->GetMaterialSurface();
}

ChPhysicsItem* ChNodeMeshless::GetPhysicsItem() {
    return container;
}

// -----------------------------------------------------------------------------

/// CLASS FOR Meshless NODE CLUSTER

ChMatterMeshless::ChMatterMeshless() : do_collide(false), viscosity(0) {
    // Default: VonMises material
    material = std::make_shared<ChContinuumPlasticVonMises>();

    // Default: NSC material
    matsurface = std::make_shared<ChMaterialSurfaceNSC>();
}

ChMatterMeshless::ChMatterMeshless(const ChMatterMeshless& other) : ChIndexedNodes(other) {
    do_collide = other.do_collide;

    matsurface = other.matsurface;

    ResizeNnodes(other.GetNnodes());
}

ChMatterMeshless::~ChMatterMeshless() {
    // delete nodes
    ResizeNnodes(0);
}

void ChMatterMeshless::ReplaceMaterial(std::shared_ptr<ChContinuumElastoplastic> newmaterial) {
    material = newmaterial;
}

void ChMatterMeshless::ResizeNnodes(int newsize) {
    bool oldcoll = GetCollide();
    SetCollide(false);  // this will remove old particle coll.models from coll.engine, if previously added

    /*
    for (unsigned int j = 0; j < nodes.size(); j++)
    {
        // delete (nodes[j]); // *** not needed since shared ptrs
        nodes[j] = 0;
    }
    */

    nodes.resize(newsize);

    for (unsigned int j = 0; j < nodes.size(); j++) {
        nodes[j] = std::make_shared<ChNodeMeshless>();

        nodes[j]->variables.SetUserData((void*)this);  // UserData unuseful in future cuda solver?

        nodes[j]->collision_model->AddPoint(0.001);  //***TEST***
        nodes[j]->collision_model->BuildModel();
    }

    SetCollide(oldcoll);  // this will also add particle coll.models to coll.engine, if already in a ChSystem
}

void ChMatterMeshless::AddNode(ChVector<double> initial_state) {
    auto newp = std::make_shared<ChNodeMeshless>();

    newp->SetPos(initial_state);
    newp->SetPosReference(initial_state);

    nodes.push_back(newp);

    newp->SetMatterContainer(this);

    newp->variables.SetUserData((void*)this);  // UserData unuseful in future cuda solver?

    newp->collision_model->AddPoint(0.1);  //***TEST***
    newp->collision_model->BuildModel();   // will also add to system, if collision is on.
}

void ChMatterMeshless::FillBox(const ChVector<> size,
                               const double spacing,
                               const double initial_density,
                               const ChCoordsys<> boxcoords,
                               const bool do_centeredcube,
                               const double kernel_sfactor,
                               const double randomness) {
    int samples_x = (int)(size.x() / spacing);
    int samples_y = (int)(size.y() / spacing);
    int samples_z = (int)(size.z() / spacing);
    int totsamples = 0;

    double mrandomness = randomness;
    if (do_centeredcube)
        mrandomness = randomness * 0.5;

    for (int ix = 0; ix < samples_x; ix++)
        for (int iy = 0; iy < samples_y; iy++)
            for (int iz = 0; iz < samples_z; iz++) {
                ChVector<> pos(ix * spacing - 0.5 * size.x(), iy * spacing - 0.5 * size.y(), iz * spacing - 0.5 * size.z());
                pos += ChVector<>(mrandomness * ChRandom() * spacing, mrandomness * ChRandom() * spacing,
                                  mrandomness * ChRandom() * spacing);
                AddNode(boxcoords.TransformLocalToParent(pos));
                totsamples++;

                if (do_centeredcube) {
                    ChVector<> pos2 = pos + 0.5 * ChVector<>(spacing, spacing, spacing);
                    pos2 += ChVector<>(mrandomness * ChRandom() * spacing, mrandomness * ChRandom() * spacing,
                                       mrandomness * ChRandom() * spacing);
                    AddNode(boxcoords.TransformLocalToParent(pos2));
                    totsamples++;
                }
            }

    double mtotvol = size.x() * size.y() * size.z();
    double mtotmass = mtotvol * initial_density;
    double nodemass = mtotmass / (double)totsamples;
    double kernelrad = kernel_sfactor * spacing;

    for (unsigned int ip = 0; ip < GetNnodes(); ip++) {
        // downcasting
        std::shared_ptr<ChNodeMeshless> mnode(nodes[ip]);
        assert(mnode);

        mnode->SetKernelRadius(kernelrad);
        mnode->SetCollisionRadius(spacing * 0.1);
        mnode->SetMass(nodemass);
    }

    GetMaterial()->Set_density(initial_density);
}

// STATE BOOKKEEPING FUNCTIONS

void ChMatterMeshless::IntStateGather(const unsigned int off_x,  // offset in x state vector
                                      ChState& x,                // state vector, position part
                                      const unsigned int off_v,  // offset in v state vector
                                      ChStateDelta& v,           // state vector, speed part
                                      double& T                  // time
                                      ) {
    for (unsigned int j = 0; j < nodes.size(); j++) {
        x.PasteVector(nodes[j]->pos, off_x + 3 * j, 0);
        v.PasteVector(nodes[j]->pos_dt, off_v + 3 * j, 0);
    }
    T = GetChTime();
}

void ChMatterMeshless::IntStateScatter(const unsigned int off_x,  // offset in x state vector
                                       const ChState& x,          // state vector, position part
                                       const unsigned int off_v,  // offset in v state vector
                                       const ChStateDelta& v,     // state vector, speed part
                                       const double T             // time
                                       ) {
    for (unsigned int j = 0; j < nodes.size(); j++) {
        nodes[j]->pos = x.ClipVector(off_x + 3 * j, 0);
        nodes[j]->pos_dt = v.ClipVector(off_v + 3 * j, 0);
    }
    SetChTime(T);
    Update();
}

void ChMatterMeshless::IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    for (unsigned int j = 0; j < nodes.size(); j++) {
        a.PasteVector(nodes[j]->pos_dtdt, off_a + 3 * j, 0);
    }
}

void ChMatterMeshless::IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    for (unsigned int j = 0; j < nodes.size(); j++) {
        nodes[j]->SetPos_dtdt(a.ClipVector(off_a + 3 * j, 0));
    }
}

void ChMatterMeshless::IntLoadResidual_F(
    const unsigned int off,  // offset in R residual (not used here! use particle's offsets)
    ChVectorDynamic<>& R,    // result: the R residual, R += c*F
    const double c           // a scaling factor
    ) {
    // COMPUTE THE MESHLESS FORCES HERE

    // First, find if any ChProximityContainerMeshless object is present
    // in the system,

    std::shared_ptr<ChProximityContainerMeshless> edges;
    for (auto otherphysics : GetSystem()->Get_otherphysicslist()) {
        if ((edges = std::dynamic_pointer_cast<ChProximityContainerMeshless>(otherphysics)))
            break;
    }
    assert(edges);  // If using a ChMatterMeshless, you must add also a ChProximityContainerMeshless.

    // 1- Per-node initialization

    for (unsigned int j = 0; j < nodes.size(); j++) {
        nodes[j]->J.FillElem(0.0);
        nodes[j]->Amoment.FillElem(0.0);
        nodes[j]->t_strain.FillElem(0.0);
        nodes[j]->e_stress.FillElem(0.0);
        nodes[j]->UserForce = VNULL;
        nodes[j]->density = 0;
    }

    // 2- Per-edge initialization and accumulation of values in particles's J, Amoment, m_v, density

    edges->AccumulateStep1();

    // 3- Per-node inversion of A and computation of strain stress

    for (unsigned int j = 0; j < nodes.size(); j++) {
        std::shared_ptr<ChNodeMeshless> mnode(nodes[j]);
        assert(mnode);

        // node volume is v=mass/density
        if (mnode->density > 0)
            mnode->volume = mnode->GetMass() / mnode->density;
        else
            mnode->volume = 0;

        // Compute A inverse
        ChMatrix33<> M_tmp = mnode->Amoment;
        double det = M_tmp.FastInvert(mnode->Amoment);
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
            material->ComputeReturnMapping(strainplasticflow,   // dEp, flow of elastic strain (correction)
                                           nodes[j]->t_strain,  // increment of total strain
                                           nodes[j]->e_strain,  // last elastic strain
                                           nodes[j]->p_strain   // last plastic strain
                                           );
            ChStrainTensor<> proj_e_strain;
            proj_e_strain.MatrSub(nodes[j]->e_strain, strainplasticflow);
            proj_e_strain.MatrInc(nodes[j]->t_strain);
            GetMaterial()->ComputeElasticStress(mnode->e_stress, proj_e_strain);
            mnode->e_stress.ConvertToMatrix(mtensor);

            /*
            // Compute elastic stress tensor  sigma= C*epsilon
            //   NOTE: it should be better to perform stress computation on corrected e_strain, _after_ the
            //   return mapping (see later), but for small timestep it could be the same.
            ChStrainTensor<> guesstot_e_strain; // anticipate the computation of total strain for anticipating strains
            guesstot_e_strain.MatrAdd(mnode->e_strain, mnode->t_strain);
            GetMaterial()->ComputeElasticStress(mnode->e_stress, guesstot_e_strain);
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
        ChVector<> Gforce = GetSystem()->Get_G_acc() * nodes[j]->GetMass();
        ChVector<> TotForce = nodes[j]->UserForce + Gforce;

        std::shared_ptr<ChNodeMeshless> mnode(nodes[j]);
        assert(mnode);

        R.PasteSumVector(TotForce * c, off + 3 * j, 0);
    }
}

void ChMatterMeshless::IntLoadResidual_Mv(const unsigned int off,      ///< offset in R residual
                                          ChVectorDynamic<>& R,        ///< result: the R residual, R += c*M*v
                                          const ChVectorDynamic<>& w,  ///< the w vector
                                          const double c               ///< a scaling factor
                                          ) {
    for (unsigned int j = 0; j < nodes.size(); j++) {
        R(off + 3 * j) += c * nodes[j]->GetMass() * w(off + 3 * j);
        R(off + 3 * j + 1) += c * nodes[j]->GetMass() * w(off + 3 * j + 1);
        R(off + 3 * j + 2) += c * nodes[j]->GetMass() * w(off + 3 * j + 2);
    }
}

void ChMatterMeshless::IntToDescriptor(const unsigned int off_v,
                                       const ChStateDelta& v,
                                       const ChVectorDynamic<>& R,
                                       const unsigned int off_L,
                                       const ChVectorDynamic<>& L,
                                       const ChVectorDynamic<>& Qc) {
    for (unsigned int j = 0; j < nodes.size(); j++) {
        nodes[j]->variables.Get_qb().PasteClippedMatrix(v, off_v + 3 * j, 0, 3, 1, 0, 0);
        nodes[j]->variables.Get_fb().PasteClippedMatrix(R, off_v + 3 * j, 0, 3, 1, 0, 0);
    }
}

void ChMatterMeshless::IntFromDescriptor(const unsigned int off_v,
                                         ChStateDelta& v,
                                         const unsigned int off_L,
                                         ChVectorDynamic<>& L) {
    for (unsigned int j = 0; j < nodes.size(); j++) {
        v.PasteMatrix(nodes[j]->variables.Get_qb(), off_v + 3 * j, 0);
    }
}

void ChMatterMeshless::InjectVariables(ChSystemDescriptor& mdescriptor) {
    // variables.SetDisabled(!IsActive());
    for (unsigned int j = 0; j < nodes.size(); j++) {
        mdescriptor.InsertVariables(&(nodes[j]->variables));
    }
}

void ChMatterMeshless::VariablesFbReset() {
    for (unsigned int j = 0; j < nodes.size(); j++) {
        nodes[j]->variables.Get_fb().FillElem(0.0);
    }
}

void ChMatterMeshless::VariablesFbLoadForces(double factor) {
    // COMPUTE THE MESHLESS FORCES HERE

    // First, find if any ChProximityContainerMeshless object is present
    // in the system,

    std::shared_ptr<ChProximityContainerMeshless> edges;
    for (auto otherphysics : GetSystem()->Get_otherphysicslist()) {
        if ((edges = std::dynamic_pointer_cast<ChProximityContainerMeshless>(otherphysics)))
            break;
    }
    assert(edges);  // If using a ChMatterMeshless, you must add also a ChProximityContainerMeshless.

    // 1- Per-node initialization

    for (unsigned int j = 0; j < nodes.size(); j++) {
        nodes[j]->J.FillElem(0.0);
        nodes[j]->Amoment.FillElem(0.0);
        nodes[j]->t_strain.FillElem(0.0);
        nodes[j]->e_stress.FillElem(0.0);
        nodes[j]->UserForce = VNULL;
        nodes[j]->density = 0;
    }

    // 2- Per-edge initialization and accumulation of values in particles's J, Amoment, m_v, density

    edges->AccumulateStep1();

    // 3- Per-node inversion of A and computation of strain stress

    for (unsigned int j = 0; j < nodes.size(); j++) {
        std::shared_ptr<ChNodeMeshless> mnode(nodes[j]);
        assert(mnode);

        // node volume is v=mass/density
        if (mnode->density > 0)
            mnode->volume = mnode->GetMass() / mnode->density;
        else
            mnode->volume = 0;

        // Compute A inverse
        ChMatrix33<> M_tmp = mnode->Amoment;
        double det = M_tmp.FastInvert(mnode->Amoment);
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
            material->ComputeReturnMapping(strainplasticflow,   // dEp, flow of elastic strain (correction)
                                           nodes[j]->t_strain,  // increment of total strain
                                           nodes[j]->e_strain,  // last elastic strain
                                           nodes[j]->p_strain   // last plastic strain
                                           );
            ChStrainTensor<> proj_e_strain;
            proj_e_strain.MatrSub(nodes[j]->e_strain, strainplasticflow);
            proj_e_strain.MatrInc(nodes[j]->t_strain);
            GetMaterial()->ComputeElasticStress(mnode->e_stress, proj_e_strain);
            mnode->e_stress.ConvertToMatrix(mtensor);

            /*
            // Compute elastic stress tensor  sigma= C*epsilon
            //   NOTE: it should be better to perform stress computation on corrected e_strain, _after_ the
            //   return mapping (see later), but for small timestep it could be the same.
            ChStrainTensor<> guesstot_e_strain; // anticipate the computation of total strain for anticipating strains
            guesstot_e_strain.MatrAdd(mnode->e_strain, mnode->t_strain);
            GetMaterial()->ComputeElasticStress(mnode->e_stress, guesstot_e_strain);
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
        ChVector<> Gforce = GetSystem()->Get_G_acc() * nodes[j]->GetMass();
        ChVector<> TotForce = nodes[j]->UserForce + Gforce;

        std::shared_ptr<ChNodeMeshless> mnode(nodes[j]);
        assert(mnode);

        mnode->variables.Get_fb().PasteSumVector(TotForce * factor, 0, 0);
    }
}

void ChMatterMeshless::VariablesFbIncrementMq() {
    for (unsigned int j = 0; j < nodes.size(); j++) {
        nodes[j]->variables.Compute_inc_Mb_v(nodes[j]->variables.Get_fb(), nodes[j]->variables.Get_qb());
    }
}

void ChMatterMeshless::VariablesQbLoadSpeed() {
    for (unsigned int j = 0; j < nodes.size(); j++) {
        // set current speed in 'qb', it can be used by the solver when working in incremental mode
        nodes[j]->variables.Get_qb().PasteVector(nodes[j]->GetPos_dt(), 0, 0);
    }
}

void ChMatterMeshless::VariablesQbSetSpeed(double step) {
    for (unsigned int j = 0; j < nodes.size(); j++) {
        ChVector<> old_pos_dt = nodes[j]->GetPos_dt();

        // from 'qb' vector, sets body speed, and updates auxiliary data
        nodes[j]->SetPos_dt(nodes[j]->variables.Get_qb().ClipVector(0, 0));

        // Compute accel. by BDF (approximate by differentiation);
        if (step) {
            nodes[j]->SetPos_dtdt((nodes[j]->GetPos_dt() - old_pos_dt) / step);
        }
    }
}

void ChMatterMeshless::VariablesQbIncrementPosition(double dt_step) {
    // if (!IsActive())
    //	return;

    for (unsigned int j = 0; j < nodes.size(); j++) {
        std::shared_ptr<ChNodeMeshless> mnode(nodes[j]);
        assert(mnode);

        // Integrate plastic flow
        ChStrainTensor<> strainplasticflow;
        material->ComputeReturnMapping(strainplasticflow,   // dEp, flow of elastic strain (correction)
                                       nodes[j]->t_strain,  // increment of total strain
                                       nodes[j]->e_strain,  // last elastic strain
                                       nodes[j]->p_strain   // last plastic strain
                                       );
        double dtpfact = dt_step * material->Get_flow_rate();
        if (dtpfact > 1.0)
            dtpfact = 1.0;  // clamp if dt is larger than plastic flow duration

        nodes[j]->p_strain.MatrInc(strainplasticflow * dtpfact);

        // Increment total elastic tensor and proceed for next step
        nodes[j]->pos_ref = nodes[j]->pos;
        nodes[j]->e_strain.MatrInc(nodes[j]->t_strain);
        //	nodes[j]->e_strain.MatrDec(strainplasticflow*dtpfact);
        nodes[j]->t_strain.FillElem(0.0);  // unuseful? will be overwritten anyway
    }

    for (unsigned int j = 0; j < nodes.size(); j++) {
        // Updates position with incremental action of speed contained in the
        // 'qb' vector:  pos' = pos + dt * speed   , like in an Eulero step.

        ChVector<> newspeed = nodes[j]->variables.Get_qb().ClipVector(0, 0);

        // ADVANCE POSITION: pos' = pos + dt * vel
        nodes[j]->SetPos(nodes[j]->GetPos() + newspeed * dt_step);
    }
}

void ChMatterMeshless::SetNoSpeedNoAcceleration() {
    for (unsigned int j = 0; j < nodes.size(); j++) {
        nodes[j]->SetPos_dt(VNULL);
        nodes[j]->SetPos_dtdt(VNULL);
    }
}

void ChMatterMeshless::Update(bool update_assets) {
    ChMatterMeshless::Update(GetChTime(), update_assets);
}

void ChMatterMeshless::Update(double mytime, bool update_assets) {
    // Inherit time changes of parent class
    ChPhysicsItem::Update(mytime, update_assets);

    // TrySleeping();    // See if the body can fall asleep; if so, put it to sleeping
    // ClampSpeed();     // Apply limits (if in speed clamping mode) to speeds.
}

// collision stuff
void ChMatterMeshless::SetCollide(bool mcoll) {
    if (mcoll == do_collide)
        return;

    if (mcoll) {
        do_collide = true;
        if (GetSystem()) {
            for (unsigned int j = 0; j < nodes.size(); j++) {
                GetSystem()->GetCollisionSystem()->Add(nodes[j]->collision_model);
            }
        }
    } else {
        do_collide = false;
        if (GetSystem()) {
            for (unsigned int j = 0; j < nodes.size(); j++) {
                GetSystem()->GetCollisionSystem()->Remove(nodes[j]->collision_model);
            }
        }
    }
}

void ChMatterMeshless::SyncCollisionModels() {
    for (unsigned int j = 0; j < nodes.size(); j++) {
        nodes[j]->collision_model->SyncPosition();
    }
}

void ChMatterMeshless::AddCollisionModelsToSystem() {
    assert(GetSystem());
    SyncCollisionModels();
    for (unsigned int j = 0; j < nodes.size(); j++) {
        GetSystem()->GetCollisionSystem()->Add(nodes[j]->collision_model);
    }
}

void ChMatterMeshless::RemoveCollisionModelsFromSystem() {
    assert(GetSystem());
    for (unsigned int j = 0; j < nodes.size(); j++) {
        GetSystem()->GetCollisionSystem()->Remove(nodes[j]->collision_model);
    }
}

////

void ChMatterMeshless::UpdateParticleCollisionModels() {
    for (unsigned int j = 0; j < nodes.size(); j++) {
        nodes[j]->collision_model->ClearModel();
        //***TO DO*** UPDATE RADIUS OF MeshlessERE?
        // nodes[j]->collision_model->AddCopyOfAnotherModel(particle_collision_model);
        nodes[j]->collision_model->BuildModel();
    }
}

//////// FILE I/O

void ChMatterMeshless::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChMatterMeshless>();

    // serialize the parent class data too
    ChIndexedNodes::ArchiveOUT(marchive);

    // serialize all member data:
    //***TODO
}

void ChMatterMeshless::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChMatterMeshless>();

    // deserialize the parent class data too
    ChIndexedNodes::ArchiveIN(marchive);

    // deserialize all member data:
    //***TODO
}



}  // end namespace fea
}  // end namespace chrono
