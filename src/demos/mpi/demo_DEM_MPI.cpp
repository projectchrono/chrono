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
//   Demo code about
//     - DEM3 and MPI
// =============================================================================

#include "unit_MPI/ChContactContainerDEMMPI.h"

#include "lcp/ChVariablesGeneric.h"
#include "lcp/ChVariablesBody.h"
#include "core/ChLinearAlgebra.h"
#include "unit_MPI/ChMpi.h"
#include "unit_MPI/ChSystemMPI.h"
#include "unit_MPI/ChBodyDEMMPI.h"
#include "unit_MPI/ChSystemDescriptorMPI.h"
#include "unit_MPI/ChDomainLatticePartitioning.h"
#include "unit_MPI/ChDomainGridPartitioning.h"
#include "unit_MPI/ChSolverDEMMPI.h"

// Use the namespace of Chrono

using namespace chrono;

void create_falling_items(ChSystemMPI& mySys, double prad, int n_bodies, double box_dim, double pmass) {
    double particle_mass = 4.0;
    if (pmass > 0.0) {
        particle_mass = pmass;
    }

    ChSharedPtr<ChBodyDEMMPI> mybody;

    double boxxx = box_dim - 2 * prad;
    for (int ii = 0; ii < n_bodies; ii++) {
        ChVector<> particle_pos(ChRandom() * boxxx - boxxx / 2, ChRandom() * boxxx / 2 + boxxx / 2 + prad,
                                ChRandom() * boxxx - boxxx / 2);
        if (mySys.nodeMPI->IsInto(particle_pos)) {
            ChSharedPtr<ChBodyDEMMPI> mybody(new ChBodyDEMMPI);
            mybody->SetIdentifier(10 + ii);
            mySys.Add(mybody);
            mybody->GetCollisionModel()->ClearModel();
            mybody->GetCollisionModel()->AddSphere(prad);
            mybody->GetCollisionModel()->BuildModel();
            mybody->SetCollide(true);
            mybody->SetPos(particle_pos);
            mybody->SetInertiaXX((2.0 / 5.0) * particle_mass * pow(prad, 2) * ChVector<>(1, 1, 1));
            mybody->SetMass(particle_mass);
            mybody->GetCollisionModel()->SyncPosition();  // really necessary?
            mybody->Update();                             // really necessary?
        }
    }

    /*
    ChVector<> pp1(0.5*box_dim/2,box_dim/2,0.5*box_dim/2);
    if (mySys.nodeMPI->IsInto(pp1))
    {
        mybody = ChSharedPtr<ChBodyDEMMPI>(new ChBodyDEMMPI);
        mybody->SetIdentifier(11);
        mySys.Add(mybody);
        mybody->GetCollisionModel()->ClearModel();
        mybody->GetCollisionModel()->AddSphere(prad);
        mybody->GetCollisionModel()->BuildModel();
        mybody->SetCollide(true);
        mybody->SetPos(pp1);
        mybody->SetMass(particle_mass);
        mybody->SetInertiaXX((2.0/5.0)*particle_mass*prad*prad*ChVector<>(1.0,1.0,1.0));
    }

    ChVector<> pp2(-0.5*box_dim/2,box_dim/2,0.5*box_dim/2);
    if (mySys.nodeMPI->IsInto(pp2))
    {
        mybody = ChSharedPtr<ChBodyDEMMPI>(new ChBodyDEMMPI);
        mybody->SetIdentifier(22);
        mySys.Add(mybody);
        mybody->GetCollisionModel()->ClearModel();
        mybody->GetCollisionModel()->AddSphere(prad);
        mybody->GetCollisionModel()->BuildModel();
        mybody->SetCollide(true);
        mybody->SetPos(pp2);
        mybody->SetMass(particle_mass);
        mybody->SetInertiaXX((2.0/5.0)*particle_mass*prad*prad*ChVector<>(1.0,1.0,1.0));
    }

    ChVector<> pp3(0.5*box_dim/2,box_dim/2,-0.5*box_dim/2);
    if (mySys.nodeMPI->IsInto(pp3))
    {
        mybody = ChSharedPtr<ChBodyDEMMPI>(new ChBodyDEMMPI);
        mybody->SetIdentifier(33);
        mySys.Add(mybody);
        mybody->GetCollisionModel()->ClearModel();
        mybody->GetCollisionModel()->AddSphere(prad);
        mybody->GetCollisionModel()->BuildModel();
        mybody->SetCollide(true);
        mybody->SetPos(pp3);
        mybody->SetMass(particle_mass);
        mybody->SetInertiaXX((2.0/5.0)*particle_mass*prad*prad*ChVector<>(1.0,1.0,1.0));
    }

    ChVector<> pp4(-0.5*box_dim/2,box_dim/2,-0.5*box_dim/2);
    if (mySys.nodeMPI->IsInto(pp4))
    {
        mybody = ChSharedPtr<ChBodyDEMMPI>(new ChBodyDEMMPI);
        mybody->SetIdentifier(44);
        mySys.Add(mybody);
        mybody->GetCollisionModel()->ClearModel();
        mybody->GetCollisionModel()->AddSphere(prad);
        mybody->GetCollisionModel()->BuildModel();
        mybody->SetCollide(true);
        mybody->SetPos(pp4);
        mybody->SetMass(particle_mass);
        mybody->SetInertiaXX((2.0/5.0)*particle_mass*prad*prad*ChVector<>(1.0,1.0,1.0));
    }
    */
}

void add_other_falling_item(ChSystemMPI& mySys, double prad, double box_dim, double rho, int n_curr_bodies) {
    int id_offset = 10;
    double m_s = (2 / 3) * CH_C_PI * pow(prad, 3) * rho;
    double m_c = CH_C_PI * pow(prad, 2) * (2 * prad) * rho;
    double m_cube = prad * prad * prad * rho;

    ChVector<> pos1(0, prad, 0);
    ChVector<> pos2(0, -prad, 0);

    ChSharedPtr<ChBodyDEMMPI> mybody;

    double boxxx = box_dim - 2 * prad;
    ChVector<> particle_pos(0.8 * boxxx / 2, box_dim / 2 + 2 * prad, 0.6 * boxxx / 2);
    if (mySys.nodeMPI->IsInto(particle_pos)) {
        ChSharedPtr<ChBodyDEMMPI> mybody(new ChBodyDEMMPI);
        mybody->SetIdentifier(id_offset + n_curr_bodies + 1);
        mySys.Add(mybody);
        mybody->GetCollisionModel()->ClearModel();
        mybody->GetCollisionModel()->AddBox(prad, prad, prad);
        // mybody->GetCollisionModel()->AddSphere(prad, &pos1);
        // mybody->GetCollisionModel()->AddCylinder(prad,prad,prad);
        // mybody->GetCollisionModel()->AddSphere(prad, &pos2);
        mybody->GetCollisionModel()->BuildModel();
        mybody->SetCollide(true);
        mybody->SetPos(particle_pos);
        // mybody->SetInertiaXX(ChVector<>(((7.0/12)*m_c+4.3*m_s)*pow(prad,2),(0.5*m_c+0.8*m_s)*pow(prad,2),((7.0/12)*m_c+4.3*m_s)*pow(prad,2)));
        // mybody->SetMass(m_c+2*m_s);
        mybody->SetInertiaXX((1.0 / 6.0) * m_cube * pow(prad, 2) * ChVector<>(1.0, 1.0, 1.0));
        mybody->SetMass(m_cube);
        mybody->GetCollisionModel()->SyncPosition();  // really necessary?
        mybody->Update();                             // really necessary?
    }
    ChVector<> particle_pos2(0.8 * boxxx / 2, box_dim / 2 + 2 * prad, 0.2 * boxxx / 2);
    if (mySys.nodeMPI->IsInto(particle_pos2)) {
        ChSharedPtr<ChBodyDEMMPI> mybody(new ChBodyDEMMPI);
        mybody->SetIdentifier(id_offset + n_curr_bodies + 2);
        mySys.Add(mybody);
        mybody->GetCollisionModel()->ClearModel();
        mybody->GetCollisionModel()->AddBox(prad, prad, prad);
        // mybody->GetCollisionModel()->AddSphere(prad, &pos1);
        // mybody->GetCollisionModel()->AddCylinder(prad,prad,prad);
        // mybody->GetCollisionModel()->AddSphere(prad, &pos2);
        mybody->GetCollisionModel()->BuildModel();
        mybody->SetCollide(true);
        mybody->SetPos(particle_pos2);
        // mybody->SetInertiaXX(ChVector<>(((7.0/12)*m_c+4.3*m_s)*pow(prad,2),(0.5*m_c+0.8*m_s)*pow(prad,2),((7.0/12)*m_c+4.3*m_s)*pow(prad,2)));
        // mybody->SetMass(m_c+2*m_s);
        mybody->SetInertiaXX((1.0 / 6.0) * m_cube * pow(prad, 2) * ChVector<>(1.0, 1.0, 1.0));
        mybody->SetMass(m_cube);
        mybody->GetCollisionModel()->SyncPosition();  // really necessary?
        mybody->Update();                             // really necessary?
    }
    ChVector<> particle_pos3(0.8 * boxxx / 2, box_dim / 2 + 2 * prad, -0.2 * boxxx / 2);
    if (mySys.nodeMPI->IsInto(particle_pos3)) {
        ChSharedPtr<ChBodyDEMMPI> mybody(new ChBodyDEMMPI);
        mybody->SetIdentifier(id_offset + n_curr_bodies + 3);
        mySys.Add(mybody);
        mybody->GetCollisionModel()->ClearModel();
        mybody->GetCollisionModel()->AddBox(prad, prad, prad);
        // mybody->GetCollisionModel()->AddSphere(prad, &pos1);
        // mybody->GetCollisionModel()->AddCylinder(prad,prad,prad);
        // mybody->GetCollisionModel()->AddSphere(prad, &pos2);
        mybody->GetCollisionModel()->BuildModel();
        mybody->SetCollide(true);
        mybody->SetPos(particle_pos3);
        // mybody->SetInertiaXX(ChVector<>(((7.0/12)*m_c+4.3*m_s)*pow(prad,2),(0.5*m_c+0.8*m_s)*pow(prad,2),((7.0/12)*m_c+4.3*m_s)*pow(prad,2)));
        // mybody->SetMass(m_c+2*m_s);
        mybody->SetInertiaXX((1.0 / 6.0) * m_cube * pow(prad, 2) * ChVector<>(1.0, 1.0, 1.0));
        mybody->SetMass(m_cube);
        mybody->GetCollisionModel()->SyncPosition();  // really necessary?
        mybody->Update();                             // really necessary?
    }
    ChVector<> particle_pos4(0.8 * boxxx / 2, box_dim / 2 + 2 * prad, -0.6 * boxxx / 2);
    if (mySys.nodeMPI->IsInto(particle_pos4)) {
        ChSharedPtr<ChBodyDEMMPI> mybody(new ChBodyDEMMPI);
        mybody->SetIdentifier(id_offset + n_curr_bodies + 4);
        mySys.Add(mybody);
        mybody->GetCollisionModel()->ClearModel();
        mybody->GetCollisionModel()->AddBox(prad, prad, prad);
        // mybody->GetCollisionModel()->AddSphere(prad, &pos1);
        // mybody->GetCollisionModel()->AddCylinder(prad,prad,prad);
        // mybody->GetCollisionModel()->AddSphere(prad, &pos2);
        mybody->GetCollisionModel()->BuildModel();
        mybody->SetCollide(true);
        mybody->SetPos(particle_pos4);
        // mybody->SetInertiaXX(ChVector<>(((7.0/12)*m_c+4.3*m_s)*pow(prad,2),(0.5*m_c+0.8*m_s)*pow(prad,2),((7.0/12)*m_c+4.3*m_s)*pow(prad,2)));
        // mybody->SetMass(m_c+2*m_s);
        mybody->SetInertiaXX((1.0 / 6.0) * m_cube * pow(prad, 2) * ChVector<>(1.0, 1.0, 1.0));
        mybody->SetMass(m_cube);
        mybody->GetCollisionModel()->SyncPosition();  // really necessary?
        mybody->Update();                             // really necessary?
    }
}

void add_falling_item(ChSystemMPI& mySys, double prad, double box_dim, double pmass, int n_curr_spheres) {
    int id_offset = 10;
    double particle_mass = 4.0;
    if (pmass > 0.0) {
        particle_mass = pmass;
    }

    ChSharedPtr<ChBodyDEMMPI> mybody;

    double boxxx = box_dim - 2 * prad;
    ChVector<> particle_pos(0.8 * boxxx / 2, box_dim / 2 + 2 * prad, 0.6 * boxxx / 2);
    if (mySys.nodeMPI->IsInto(particle_pos)) {
        ChSharedPtr<ChBodyDEMMPI> mybody(new ChBodyDEMMPI);
        mybody->SetIdentifier(id_offset + n_curr_spheres + 1);
        mySys.Add(mybody);
        mybody->GetCollisionModel()->ClearModel();
        mybody->GetCollisionModel()->AddSphere(prad);
        mybody->GetCollisionModel()->BuildModel();
        mybody->SetCollide(true);
        mybody->SetPos(particle_pos);
        mybody->SetInertiaXX((2.0 / 5.0) * particle_mass * pow(prad, 2) * ChVector<>(1, 1, 1));
        mybody->SetMass(particle_mass);
        mybody->GetCollisionModel()->SyncPosition();  // really necessary?
        mybody->Update();                             // really necessary?
    }
    ChVector<> particle_pos2(0.8 * boxxx / 2, box_dim / 2 + 2 * prad, 0.2 * boxxx / 2);
    if (mySys.nodeMPI->IsInto(particle_pos2)) {
        ChSharedPtr<ChBodyDEMMPI> mybody(new ChBodyDEMMPI);
        mybody->SetIdentifier(id_offset + n_curr_spheres + 2);
        mySys.Add(mybody);
        mybody->GetCollisionModel()->ClearModel();
        mybody->GetCollisionModel()->AddSphere(prad);
        mybody->GetCollisionModel()->BuildModel();
        mybody->SetCollide(true);
        mybody->SetPos(particle_pos2);
        mybody->SetInertiaXX((2.0 / 5.0) * particle_mass * pow(prad, 2) * ChVector<>(1, 1, 1));
        mybody->SetMass(particle_mass);
        mybody->GetCollisionModel()->SyncPosition();  // really necessary?
        mybody->Update();                             // really necessary?
    }
    ChVector<> particle_pos3(0.8 * boxxx / 2, box_dim / 2 + 2 * prad, -0.2 * boxxx / 2);
    if (mySys.nodeMPI->IsInto(particle_pos3)) {
        ChSharedPtr<ChBodyDEMMPI> mybody(new ChBodyDEMMPI);
        mybody->SetIdentifier(id_offset + n_curr_spheres + 3);
        mySys.Add(mybody);
        mybody->GetCollisionModel()->ClearModel();
        mybody->GetCollisionModel()->AddSphere(prad);
        mybody->GetCollisionModel()->BuildModel();
        mybody->SetCollide(true);
        mybody->SetPos(particle_pos3);
        mybody->SetInertiaXX((2.0 / 5.0) * particle_mass * pow(prad, 2) * ChVector<>(1, 1, 1));
        mybody->SetMass(particle_mass);
        mybody->GetCollisionModel()->SyncPosition();  // really necessary?
        mybody->Update();                             // really necessary?
    }
    ChVector<> particle_pos4(0.8 * boxxx / 2, box_dim / 2 + 2 * prad, -0.6 * boxxx / 2);
    if (mySys.nodeMPI->IsInto(particle_pos4)) {
        ChSharedPtr<ChBodyDEMMPI> mybody(new ChBodyDEMMPI);
        mybody->SetIdentifier(id_offset + n_curr_spheres + 4);
        mySys.Add(mybody);
        mybody->GetCollisionModel()->ClearModel();
        mybody->GetCollisionModel()->AddSphere(prad);
        mybody->GetCollisionModel()->BuildModel();
        mybody->SetCollide(true);
        mybody->SetPos(particle_pos4);
        mybody->SetInertiaXX((2.0 / 5.0) * particle_mass * pow(prad, 2) * ChVector<>(1, 1, 1));
        mybody->SetMass(particle_mass);
        mybody->GetCollisionModel()->SyncPosition();  // really necessary?
        mybody->Update();                             // really necessary?
    }
}

void add_batch(ChSystemMPI& mySys, double prad, double box_dim, double pmass, int n_batch, int n_curr_spheres) {
    int id_offset = 10;
    double particle_mass = 4.0;
    if (pmass > 0.0) {
        particle_mass = pmass;
    }

    ChSharedPtr<ChBodyDEMMPI> mybody;

    double boxxx = box_dim - 4 * prad;
    double spacing = (boxxx - 8 * prad) / ((n_batch / 4) - 1);
    double first = -boxxx / 2;
    int cid = 0;

    for (int ii = 0; ii < n_batch / 4; ii++) {
        ChVector<> particle_pos(boxxx / 2 - 2 * prad, box_dim / 2 + 2 * prad, first + ii * spacing + 0.0001);
        if (mySys.nodeMPI->IsInto(particle_pos)) {
            ChSharedPtr<ChBodyDEMMPI> mybody(new ChBodyDEMMPI);
            mybody->SetIdentifier(id_offset + n_curr_spheres + cid);
            mybody->SetCollide(true);
            mybody->GetCollisionModel()->ClearModel();
            mybody->GetCollisionModel()->AddSphere(prad);
            mybody->GetCollisionModel()->BuildModel();
            mySys.Add(mybody);
            mybody->SetPos(particle_pos);
            mybody->SetInertiaXX((2.0 / 5.0) * particle_mass * pow(prad, 2) * ChVector<>(1, 1, 1));
            mybody->SetMass(particle_mass);
            mybody->GetCollisionModel()->SyncPosition();  // really necessary?
            mybody->Update();                             // really necessary?
        }
        cid++;
    }

    for (int ii = 0; ii < n_batch / 4; ii++) {
        ChVector<> particle_pos(boxxx / 2 - 6 * prad, box_dim / 2 + 2 * prad, first + ii * spacing + 0.0001);
        if (mySys.nodeMPI->IsInto(particle_pos)) {
            ChSharedPtr<ChBodyDEMMPI> mybody(new ChBodyDEMMPI);
            mybody->SetIdentifier(id_offset + n_curr_spheres + cid);
            mybody->SetCollide(true);
            mybody->GetCollisionModel()->ClearModel();
            mybody->GetCollisionModel()->AddSphere(prad);
            mybody->GetCollisionModel()->BuildModel();
            mySys.Add(mybody);
            mybody->SetPos(particle_pos);
            mybody->SetInertiaXX((2.0 / 5.0) * particle_mass * pow(prad, 2) * ChVector<>(1, 1, 1));
            mybody->SetMass(particle_mass);
            mybody->GetCollisionModel()->SyncPosition();  // really necessary?
            mybody->Update();                             // really necessary?
        }
        cid++;
    }

    // other edge
    for (int ii = 0; ii < n_batch / 4; ii++) {
        ChVector<> particle_pos(first + ii * spacing + 0.0001, box_dim / 2 + 2 * prad, boxxx / 2 - 2 * prad);
        if (mySys.nodeMPI->IsInto(particle_pos)) {
            ChSharedPtr<ChBodyDEMMPI> mybody(new ChBodyDEMMPI);
            mybody->SetIdentifier(id_offset + n_curr_spheres + cid);
            mybody->SetCollide(true);
            mybody->GetCollisionModel()->ClearModel();
            mybody->GetCollisionModel()->AddSphere(prad);
            mybody->GetCollisionModel()->BuildModel();
            mySys.Add(mybody);
            mybody->SetPos(particle_pos);
            mybody->SetInertiaXX((2.0 / 5.0) * particle_mass * pow(prad, 2) * ChVector<>(1, 1, 1));
            mybody->SetMass(particle_mass);
            mybody->GetCollisionModel()->SyncPosition();  // really necessary?
            mybody->Update();                             // really necessary?
        }
        cid++;
    }

    for (int ii = 0; ii < n_batch / 4; ii++) {
        ChVector<> particle_pos(first + ii * spacing + 0.0001, box_dim / 2 + 2 * prad, boxxx / 2 - 6 * prad);
        if (mySys.nodeMPI->IsInto(particle_pos)) {
            ChSharedPtr<ChBodyDEMMPI> mybody(new ChBodyDEMMPI);
            mybody->SetIdentifier(id_offset + n_curr_spheres + cid);
            mybody->SetCollide(true);
            mybody->GetCollisionModel()->ClearModel();
            mybody->GetCollisionModel()->AddSphere(prad);
            mybody->GetCollisionModel()->BuildModel();
            mySys.Add(mybody);
            mybody->SetPos(particle_pos);
            mybody->SetInertiaXX((2.0 / 5.0) * particle_mass * pow(prad, 2) * ChVector<>(1, 1, 1));
            mybody->SetMass(particle_mass);
            mybody->GetCollisionModel()->SyncPosition();  // really necessary?
            mybody->Update();                             // really necessary?
        }
        cid++;
    }
}

void create_boundary_boxes(ChSystemMPI& mySys, double box_dim, int n_bodies) {
    double th = 0.1;
    ChVector<> box_base(0.0, -th / 2, 0.0);
    if (mySys.nodeMPI->IsInto(box_base)) {
        ChSharedPtr<ChBodyDEMMPI> mybody(new ChBodyDEMMPI);
        mybody->SetIdentifier(n_bodies + 101);
        mybody->SetBodyFixed(true);
        mybody->SetCollide(true);
        mybody->GetCollisionModel()->ClearModel();
        mybody->GetCollisionModel()->AddBox(box_dim / 2, th / 2, box_dim / 2);
        mybody->GetCollisionModel()->BuildModel();
        mySys.Add(mybody);
        mybody->SetPos(box_base);
        mybody->GetCollisionModel()->SyncPosition();
        mybody->Update();
    }
    ChVector<> box_left(-box_dim / 2 - th / 2, box_dim / 2, 0.0);
    if (mySys.nodeMPI->IsInto(box_left)) {
        ChSharedPtr<ChBodyDEMMPI> mybody(new ChBodyDEMMPI);
        mybody->SetIdentifier(n_bodies + 102);
        mybody->SetBodyFixed(true);
        mybody->SetCollide(true);
        mybody->GetCollisionModel()->ClearModel();
        mybody->GetCollisionModel()->AddBox(th / 2, box_dim / 2, box_dim / 2);
        mybody->GetCollisionModel()->BuildModel();
        mySys.Add(mybody);
        mybody->SetPos(box_left);
        mybody->GetCollisionModel()->SyncPosition();
        mybody->Update();
    }
    ChVector<> box_right(box_dim / 2 + th / 2, box_dim / 2, 0.0);
    if (mySys.nodeMPI->IsInto(box_right)) {
        ChSharedPtr<ChBodyDEMMPI> mybody(new ChBodyDEMMPI);
        mybody->SetIdentifier(n_bodies + 103);
        mybody->SetBodyFixed(true);
        mybody->SetCollide(true);
        mybody->GetCollisionModel()->ClearModel();
        mybody->GetCollisionModel()->AddBox(th / 2, box_dim / 2, box_dim / 2);
        mybody->GetCollisionModel()->BuildModel();
        mySys.Add(mybody);
        mybody->SetPos(box_right);
        mybody->GetCollisionModel()->SyncPosition();
        mybody->Update();
    }
    ChVector<> box_back(0.0, box_dim / 2, -box_dim / 2 - th / 2);
    if (mySys.nodeMPI->IsInto(box_back)) {
        ChSharedPtr<ChBodyDEMMPI> mybody(new ChBodyDEMMPI);
        mybody->SetIdentifier(n_bodies + 104);
        mybody->SetBodyFixed(true);
        mybody->SetCollide(true);
        mybody->GetCollisionModel()->ClearModel();
        mybody->GetCollisionModel()->AddBox(box_dim / 2, box_dim / 2, th / 2);
        mybody->GetCollisionModel()->BuildModel();
        mySys.Add(mybody);
        mybody->SetPos(box_back);
        mybody->GetCollisionModel()->SyncPosition();
        mybody->Update();
    }
    ChVector<> box_front(0.0, box_dim / 2, box_dim / 2 + th / 2);
    if (mySys.nodeMPI->IsInto(box_front)) {
        ChSharedPtr<ChBodyDEMMPI> mybody(new ChBodyDEMMPI);
        mybody->SetIdentifier(n_bodies + 105);
        mybody->SetBodyFixed(true);
        mybody->SetCollide(true);
        mybody->GetCollisionModel()->ClearModel();
        mybody->GetCollisionModel()->AddBox(box_dim / 2, box_dim / 2, th / 2);
        mybody->GetCollisionModel()->BuildModel();
        mySys.Add(mybody);
        mybody->SetPos(box_front);
        mybody->GetCollisionModel()->SyncPosition();
        mybody->Update();
    }
}

void create_boxes_slope(ChSystemMPI& mySys, double box_dim) {
    double th = 0.1;
    double lb = sqrt(pow(box_dim / 2, 2) + pow(box_dim, 2));
    ChVector<> box_base(0.0 + 0.0001, (lb / 2) * sin(atan(0.5)), 0.0 + 0.0001);
    GetLog() << mySys.nodeMPI->id_MPI << " check body 50000\n";
    if (mySys.nodeMPI->IsInto(box_base)) {
        GetLog() << mySys.nodeMPI->id_MPI << " adding body 50000\n";
        ChSharedPtr<ChBodyDEMMPI> mybody(new ChBodyDEMMPI);
        mybody->SetIdentifier(50000);
        mybody->SetBodyFixed(true);
        mybody->SetCollide(true);
        mybody->GetCollisionModel()->ClearModel();
        mybody->GetCollisionModel()->AddBox(lb / 2, th / 2, box_dim / 2);
        mybody->GetCollisionModel()->BuildModel();
        mySys.Add(mybody);
        mybody->SetPos(box_base);
        ChQuaternion<> rotation_base(0.0, 0.0, 0.0, 0.0);
        rotation_base.Q_from_AngZ(atan(0.5));
        mybody->SetRot(rotation_base);
        mybody->GetCollisionModel()->SyncPosition();
        mybody->Update();
    }
    GetLog() << mySys.nodeMPI->id_MPI << " check body 50005\n";
    if (mySys.nodeMPI->IsInto(box_base)) {
        GetLog() << mySys.nodeMPI->id_MPI << " adding body 50005\n";
        ChSharedPtr<ChBodyDEMMPI> mybody(new ChBodyDEMMPI);
        mybody->SetIdentifier(50005);
        mybody->SetBodyFixed(true);
        mybody->SetCollide(true);
        mybody->GetCollisionModel()->ClearModel();
        mybody->GetCollisionModel()->AddBox(box_dim / 2, th / 2, lb / 2);
        mybody->GetCollisionModel()->BuildModel();
        mySys.Add(mybody);
        mybody->SetPos(box_base);
        ChQuaternion<> rotation_base(0.0, 0.0, 0.0, 0.0);
        rotation_base.Q_from_AngX(-atan(0.5));
        mybody->SetRot(rotation_base);
        mybody->GetCollisionModel()->SyncPosition();
        mybody->Update();
    }
    GetLog() << mySys.nodeMPI->id_MPI << " check body 50001\n";
    ChVector<> box_left(-box_dim / 2, box_dim / 4, 0.0 + 0.0001);
    if (mySys.nodeMPI->IsInto(box_left)) {
        GetLog() << mySys.nodeMPI->id_MPI << " adding body 50001\n";
        ChSharedPtr<ChBodyDEMMPI> mybody(new ChBodyDEMMPI);
        mybody->SetIdentifier(50001);
        mybody->SetBodyFixed(true);
        mybody->SetCollide(true);
        mybody->GetCollisionModel()->ClearModel();
        mybody->GetCollisionModel()->AddBox(th / 2, box_dim / 4, box_dim / 2);
        mybody->GetCollisionModel()->BuildModel();
        mySys.Add(mybody);
        mybody->SetPos(box_left);
        mybody->GetCollisionModel()->SyncPosition();
        mybody->Update();
    }
    GetLog() << mySys.nodeMPI->id_MPI << " check body 50002\n";
    ChVector<> box_right(box_dim / 2, box_dim / 4, 0.0 + 0.0001);
    if (mySys.nodeMPI->IsInto(box_right)) {
        GetLog() << mySys.nodeMPI->id_MPI << " adding body 50002\n";
        ChSharedPtr<ChBodyDEMMPI> mybody(new ChBodyDEMMPI);
        mybody->SetIdentifier(50002);
        mybody->SetBodyFixed(true);
        mybody->SetCollide(true);
        mybody->GetCollisionModel()->ClearModel();
        mybody->GetCollisionModel()->AddBox(th / 2, box_dim / 4, box_dim / 2);
        mybody->GetCollisionModel()->BuildModel();
        mySys.Add(mybody);
        mybody->SetPos(box_right);
        mybody->GetCollisionModel()->SyncPosition();
        mybody->Update();
    }
    GetLog() << mySys.nodeMPI->id_MPI << " check body 50003\n";
    ChVector<> box_back(0.0 + 0.0001, box_dim / 4, -box_dim / 2);
    if (mySys.nodeMPI->IsInto(box_back)) {
        GetLog() << mySys.nodeMPI->id_MPI << " adding body 50003\n";
        ChSharedPtr<ChBodyDEMMPI> mybody(new ChBodyDEMMPI);
        mybody->SetIdentifier(50003);
        mybody->SetBodyFixed(true);
        mybody->SetCollide(true);
        mybody->GetCollisionModel()->ClearModel();
        mybody->GetCollisionModel()->AddBox(box_dim / 2, box_dim / 4, th / 2);
        mybody->GetCollisionModel()->BuildModel();
        mySys.Add(mybody);
        mybody->SetPos(box_back);
        mybody->GetCollisionModel()->SyncPosition();
        mybody->Update();
    }
    GetLog() << mySys.nodeMPI->id_MPI << " check body 50004\n";
    ChVector<> box_front(0.0 + 0.0001, box_dim / 4, box_dim / 2);
    if (mySys.nodeMPI->IsInto(box_front)) {
        GetLog() << mySys.nodeMPI->id_MPI << " adding body 50004\n";
        ChSharedPtr<ChBodyDEMMPI> mybody(new ChBodyDEMMPI);
        mybody->SetIdentifier(50004);
        mybody->SetBodyFixed(true);
        mybody->SetCollide(true);
        mybody->GetCollisionModel()->ClearModel();
        mybody->GetCollisionModel()->AddBox(box_dim / 2, box_dim / 4, th / 2);
        mybody->GetCollisionModel()->BuildModel();
        mySys.Add(mybody);
        mybody->SetPos(box_front);
        mybody->GetCollisionModel()->SyncPosition();
        mybody->Update();
    }
}

void
create_fancy_boundary(ChSystemMPI& mySys, double a, double b, double c, double d, double e, double f, double width) {
    double th = 0.1;

    ChVector<> box1(-a, d / 2, 0.0);
    if (mySys.nodeMPI->IsInto(box1)) {
        ChSharedPtr<ChBodyDEMMPI> mybody(new ChBodyDEMMPI);
        mybody->SetIdentifier(50001);
        mybody->SetBodyFixed(true);
        mybody->SetCollide(true);
        mybody->GetCollisionModel()->ClearModel();
        mybody->GetCollisionModel()->AddBox(th / 2, d / 2, width / 2);
        mybody->GetCollisionModel()->BuildModel();
        mySys.Add(mybody);
        mybody->SetPos(box1);
        mybody->GetCollisionModel()->SyncPosition();
        mybody->Update();
    }
    ChVector<> box2(c, d / 2, 0.0);
    if (mySys.nodeMPI->IsInto(box2)) {
        ChSharedPtr<ChBodyDEMMPI> mybody(new ChBodyDEMMPI);
        mybody->SetIdentifier(50002);
        mybody->SetBodyFixed(true);
        mybody->SetCollide(true);
        mybody->GetCollisionModel()->ClearModel();
        mybody->GetCollisionModel()->AddBox(th / 2, d / 2, width / 2);
        mybody->GetCollisionModel()->BuildModel();
        mySys.Add(mybody);
        mybody->SetPos(box2);
        mybody->GetCollisionModel()->SyncPosition();
        mybody->Update();
    }
    ChVector<> box3((c - a) / 2, 0.0, 0.0);
    if (mySys.nodeMPI->IsInto(box3)) {
        ChSharedPtr<ChBodyDEMMPI> mybody(new ChBodyDEMMPI);
        mybody->SetIdentifier(50003);
        mybody->SetBodyFixed(true);
        mybody->SetCollide(true);
        mybody->GetCollisionModel()->ClearModel();
        mybody->GetCollisionModel()->AddBox((a + c) / 2, th / 2, width / 2);
        mybody->GetCollisionModel()->BuildModel();
        mySys.Add(mybody);
        mybody->SetPos(box3);
        mybody->GetCollisionModel()->SyncPosition();
        mybody->Update();
    }
    ChVector<> box4((c - b) / 2, f, 0.0);
    if (mySys.nodeMPI->IsInto(box4)) {
        ChSharedPtr<ChBodyDEMMPI> mybody(new ChBodyDEMMPI);
        mybody->SetIdentifier(50004);
        mybody->SetBodyFixed(true);
        mybody->SetCollide(true);
        mybody->GetCollisionModel()->ClearModel();
        mybody->GetCollisionModel()->AddBox((b + c) / 2, th / 2, width / 2);
        mybody->GetCollisionModel()->BuildModel();
        mySys.Add(mybody);
        mybody->SetPos(box4);
        mybody->GetCollisionModel()->SyncPosition();
        mybody->Update();
    }
    double tmp5 = sqrt(pow(d - e, 2) + pow(a - b, 2));
    ChVector<> box5(-b - (tmp5 / 2) * cos(atan((d - e) / (a - b))), e + (tmp5 / 2) * sin(atan((d - e) / (a - b))), 0.0);
    if (mySys.nodeMPI->IsInto(box5)) {
        ChSharedPtr<ChBodyDEMMPI> mybody(new ChBodyDEMMPI);
        mybody->SetIdentifier(50005);
        mybody->SetBodyFixed(true);
        mybody->SetCollide(true);
        mybody->GetCollisionModel()->ClearModel();
        mybody->GetCollisionModel()->AddBox(tmp5 / 2, th / 2, width / 2);
        mybody->GetCollisionModel()->BuildModel();
        mySys.Add(mybody);
        mybody->SetPos(box5);
        ChQuaternion<> rotation_base(0.0, 0.0, 0.0, 0.0);
        rotation_base.Q_from_AngZ(-atan((d - e) / (a - b)));
        mybody->SetRot(rotation_base);
        mybody->GetCollisionModel()->SyncPosition();
        mybody->Update();
    }
    ChVector<> box6(-b, (e + f) / 2, 0.0);
    if (mySys.nodeMPI->IsInto(box6)) {
        ChSharedPtr<ChBodyDEMMPI> mybody(new ChBodyDEMMPI);
        mybody->SetIdentifier(50006);
        mybody->SetBodyFixed(true);
        mybody->SetCollide(true);
        mybody->GetCollisionModel()->ClearModel();
        mybody->GetCollisionModel()->AddBox(th / 2, (e - f) / 2, width / 2);
        mybody->GetCollisionModel()->BuildModel();
        mySys.Add(mybody);
        mybody->SetPos(box6);
        mybody->GetCollisionModel()->SyncPosition();
        mybody->Update();
    }
    ChVector<> box7((c - a) / 2, d / 2, -width / 2);
    if (mySys.nodeMPI->IsInto(box7)) {
        ChSharedPtr<ChBodyDEMMPI> mybody(new ChBodyDEMMPI);
        mybody->SetIdentifier(50007);
        mybody->SetBodyFixed(true);
        mybody->SetCollide(true);
        mybody->GetCollisionModel()->ClearModel();
        mybody->GetCollisionModel()->AddBox((a + c) / 2, d / 2, th / 2);
        mybody->GetCollisionModel()->BuildModel();
        mySys.Add(mybody);
        mybody->SetPos(box7);
        mybody->GetCollisionModel()->SyncPosition();
        mybody->Update();
    }
    ChVector<> box8((c - a) / 2, d / 2, width / 2);
    if (mySys.nodeMPI->IsInto(box8)) {
        ChSharedPtr<ChBodyDEMMPI> mybody(new ChBodyDEMMPI);
        mybody->SetIdentifier(50008);
        mybody->SetBodyFixed(true);
        mybody->SetCollide(true);
        mybody->GetCollisionModel()->ClearModel();
        mybody->GetCollisionModel()->AddBox((a + c) / 2, d / 2, th / 2);
        mybody->GetCollisionModel()->BuildModel();
        mySys.Add(mybody);
        mybody->SetPos(box8);
        mybody->GetCollisionModel()->SyncPosition();
        mybody->Update();
    }
}

void create_funnel_boundary(ChSystemMPI& mySys,
                            double a,
                            double b,
                            double c,
                            double d,
                            double e,
                            double f,
                            double g,
                            double h,
                            double width) {
    double th = 0.1;

    ChVector<> box1(-(b / 2) - a, h + c + (d / 2), 0.0);
    if (mySys.nodeMPI->IsInto(box1)) {
        ChSharedPtr<ChBodyDEMMPI> mybody(new ChBodyDEMMPI);
        mybody->SetIdentifier(50001);
        mybody->SetBodyFixed(true);
        mybody->SetCollide(true);
        mybody->GetCollisionModel()->ClearModel();
        mybody->GetCollisionModel()->AddBox(th / 2, d / 2, width / 2);
        mybody->GetCollisionModel()->BuildModel();
        mySys.Add(mybody);
        mybody->SetPos(box1);
        mybody->GetCollisionModel()->SyncPosition();
        mybody->Update();
    }
    ChVector<> box2((b / 2) + a, h + c + (d / 2), 0.0);
    if (mySys.nodeMPI->IsInto(box2)) {
        ChSharedPtr<ChBodyDEMMPI> mybody(new ChBodyDEMMPI);
        mybody->SetIdentifier(50002);
        mybody->SetBodyFixed(true);
        mybody->SetCollide(true);
        mybody->GetCollisionModel()->ClearModel();
        mybody->GetCollisionModel()->AddBox(th / 2, d / 2, width / 2);
        mybody->GetCollisionModel()->BuildModel();
        mySys.Add(mybody);
        mybody->SetPos(box2);
        mybody->GetCollisionModel()->SyncPosition();
        mybody->Update();
    }
    double tmp3 = sqrt(pow(a, 2) + pow(c, 2));
    ChVector<> box3(-(b / 2) - tmp3 * cos(atan(c / a)), h + tmp3 * sin(atan(c / a)), 0.0);
    if (mySys.nodeMPI->IsInto(box3)) {
        ChSharedPtr<ChBodyDEMMPI> mybody(new ChBodyDEMMPI);
        mybody->SetIdentifier(50003);
        mybody->SetBodyFixed(true);
        mybody->SetCollide(true);
        mybody->GetCollisionModel()->ClearModel();
        mybody->GetCollisionModel()->AddBox(tmp3 / 2, th / 2, width / 2);
        mybody->GetCollisionModel()->BuildModel();
        mySys.Add(mybody);
        mybody->SetPos(box3);
        ChQuaternion<> rotation_base(0.0, 0.0, 0.0, 0.0);
        rotation_base.Q_from_AngZ(-atan(c / a));
        mybody->SetRot(rotation_base);
        mybody->GetCollisionModel()->SyncPosition();
        mybody->Update();
    }
    ChVector<> box4((b / 2) + tmp3 * cos(atan(c / a)), h + tmp3 * sin(atan(c / a)), 0.0);
    if (mySys.nodeMPI->IsInto(box4)) {
        ChSharedPtr<ChBodyDEMMPI> mybody(new ChBodyDEMMPI);
        mybody->SetIdentifier(50004);
        mybody->SetBodyFixed(true);
        mybody->SetCollide(true);
        mybody->GetCollisionModel()->ClearModel();
        mybody->GetCollisionModel()->AddBox(tmp3 / 2, th / 2, width / 2);
        mybody->GetCollisionModel()->BuildModel();
        mySys.Add(mybody);
        mybody->SetPos(box4);
        ChQuaternion<> rotation_base(0.0, 0.0, 0.0, 0.0);
        rotation_base.Q_from_AngZ(atan(c / a));
        mybody->SetRot(rotation_base);
        mybody->GetCollisionModel()->SyncPosition();
        mybody->Update();
    }
    ChVector<> box5(0.0, h, 0.0);
    if (mySys.nodeMPI->IsInto(box5)) {
        ChSharedPtr<ChBodyDEMMPI> mybody(new ChBodyDEMMPI);
        mybody->SetIdentifier(50005);
        mybody->SetBodyFixed(true);
        mybody->SetCollide(true);
        mybody->GetCollisionModel()->ClearModel();
        mybody->GetCollisionModel()->AddBox(e / 2, th / 2, width / 2);
        mybody->GetCollisionModel()->BuildModel();
        mySys.Add(mybody);
        mybody->SetPos(box5);
        mybody->GetCollisionModel()->SyncPosition();
        mybody->Update();
    }
    ChVector<> box6(-f / 2, g / 2, 0.0);
    if (mySys.nodeMPI->IsInto(box6)) {
        ChSharedPtr<ChBodyDEMMPI> mybody(new ChBodyDEMMPI);
        mybody->SetIdentifier(50006);
        mybody->SetBodyFixed(true);
        mybody->SetCollide(true);
        mybody->GetCollisionModel()->ClearModel();
        mybody->GetCollisionModel()->AddBox(th / 2, g / 2, width / 2);
        mybody->GetCollisionModel()->BuildModel();
        mySys.Add(mybody);
        mybody->SetPos(box6);
        mybody->GetCollisionModel()->SyncPosition();
        mybody->Update();
    }
    ChVector<> box7(f / 2, g / 2, 0.0);
    if (mySys.nodeMPI->IsInto(box7)) {
        ChSharedPtr<ChBodyDEMMPI> mybody(new ChBodyDEMMPI);
        mybody->SetIdentifier(50007);
        mybody->SetBodyFixed(true);
        mybody->SetCollide(true);
        mybody->GetCollisionModel()->ClearModel();
        mybody->GetCollisionModel()->AddBox(th / 2, g / 2, width / 2);
        mybody->GetCollisionModel()->BuildModel();
        mySys.Add(mybody);
        mybody->SetPos(box7);
        mybody->GetCollisionModel()->SyncPosition();
        mybody->Update();
    }
    ChVector<> box8(0.0, 0.0, 0.0);
    if (mySys.nodeMPI->IsInto(box8)) {
        ChSharedPtr<ChBodyDEMMPI> mybody(new ChBodyDEMMPI);
        mybody->SetIdentifier(50008);
        mybody->SetBodyFixed(true);
        mybody->SetCollide(true);
        mybody->GetCollisionModel()->ClearModel();
        mybody->GetCollisionModel()->AddBox(f / 2, th / 2, width / 2);
        mybody->GetCollisionModel()->BuildModel();
        mySys.Add(mybody);
        mybody->SetPos(box8);
        mybody->GetCollisionModel()->SyncPosition();
        mybody->Update();
    }
    ChVector<> box9(0.0, (h + c + d) / 2, -width / 2);
    if (mySys.nodeMPI->IsInto(box9)) {
        ChSharedPtr<ChBodyDEMMPI> mybody(new ChBodyDEMMPI);
        mybody->SetIdentifier(50009);
        mybody->SetBodyFixed(true);
        mybody->SetCollide(true);
        mybody->GetCollisionModel()->ClearModel();
        mybody->GetCollisionModel()->AddBox(f / 2, (h + c + d) / 2, th / 2);
        mybody->GetCollisionModel()->BuildModel();
        mySys.Add(mybody);
        mybody->SetPos(box9);
        mybody->GetCollisionModel()->SyncPosition();
        mybody->Update();
    }
    ChVector<> box10(0.0, (h + c + d) / 2, width / 2);
    if (mySys.nodeMPI->IsInto(box10)) {
        ChSharedPtr<ChBodyDEMMPI> mybody(new ChBodyDEMMPI);
        mybody->SetIdentifier(50010);
        mybody->SetBodyFixed(true);
        mybody->SetCollide(true);
        mybody->GetCollisionModel()->ClearModel();
        mybody->GetCollisionModel()->AddBox(f / 2, (h + c + d) / 2, th / 2);
        mybody->GetCollisionModel()->BuildModel();
        mySys.Add(mybody);
        mybody->SetPos(box10);
        mybody->GetCollisionModel()->SyncPosition();
        mybody->Update();
    }
}

void add_fancy_items(ChSystemMPI& mySys,
                     double prad,
                     double pmass,
                     double width,
                     double g,
                     double h,
                     int n_batch,
                     int n_curr_spheres) {
    int id_offset = 10;
    double particle_mass = 4.0;
    if (pmass > 0.0) {
        particle_mass = pmass;
    }

    ChSharedPtr<ChBodyDEMMPI> mybody;

    double spacing = (width - 8 * prad) / (n_batch - 1);
    double first = -(width - 8 * prad) / 2;

    for (int jj = 0; jj < n_batch; jj++) {
        ChVector<> particle_pos(-g + (ChRandom() * 2 * prad - prad), h, first + jj * spacing);
        if (mySys.nodeMPI->IsInto(particle_pos)) {
            ChSharedPtr<ChBodyDEMMPI> mybody(new ChBodyDEMMPI);
            mybody->SetIdentifier(id_offset + n_curr_spheres + jj);
            mySys.Add(mybody);
            mybody->GetCollisionModel()->ClearModel();
            mybody->GetCollisionModel()->AddSphere(prad);
            mybody->GetCollisionModel()->BuildModel();
            mybody->SetCollide(true);
            mybody->SetPos(particle_pos);
            mybody->SetInertiaXX((2.0 / 5.0) * particle_mass * pow(prad, 2) * ChVector<>(1, 1, 1));
            mybody->SetMass(particle_mass);
            mybody->GetCollisionModel()->SyncPosition();  // really necessary?
            mybody->Update();                             // really necessary?
        }
    }
}

int main(int argc, char* argv[]) {
    double particle_radius = 0.1;
    int num_particles = 160;
    int curr_particles = 0;
    int num_batch = 16;
    double box = 4.0;

    double aa = 4.0;
    double bb = 1.0;
    double cc = 1.0;
    double dd = 8.0;
    double ee = 6.0;
    double ff = 2.0;
    double gg = 3.0;
    double hh = 8.0;
    double width = 10.0;

    double time_step = 0.00001;
    double end_time = 4.0;
    int outMult = 3000;
    int addMult = 30000;
    int frame_number = 0;
    int fOuts = outMult;
    int fAdds = addMult;

    GetLog() << "\n\n\n ----------------------------------------------\n";

    // Initialize the MPI functionality. Use the CHMPI static functions.
    CHMPI::Init(argc, argv);

    GetLog() << "\n\n\n ----------------------------------------------\n"
             << "Example: use MPI for a multi-domain simulation \n\n";

    // Get infos about how many processes are launched,
    // and about the ID of this specific process.
    int numprocs = CHMPI::CommSize();
    int myid = CHMPI::CommRank();

    // Instead of using the usual ChSystem, rather use ChSystemMPI. It is
    // a specialized class for physical systems that can be used
    // for domain decomposition with MPI communication.

    ChSystemMPI mysystem;

    // Since we run multiple processes of this program, we must
    // set a 'topology' between them. Currently, cubic lattice
    // subdivision is supported.
    // A 'partitioner' tool will help to setup this.

    /*
    // This is the old way
    ChDomainLatticePartitioning mypartitioner(2,1,1,			// nx ny nz domains
                                        ChVector<>(-100,-10,-100),	// min world
                                        ChVector<>( 100, 50, 100) );	// max world
    */

    // This is the new way
    std::vector<double> x_s(4, 1.0);
    std::vector<double> y_s(1, 60.0);
    std::vector<double> z_s(1, 200.0);
    ChDomainGridPartitioning mypartitioner(x_s, y_s, z_s, ChVector<>(-2, -10, -100));
    mypartitioner.SetupNode(mysystem.nodeMPI, myid);  // btw: please take care that must be numprocs=nx*ny*nz

    mysystem.SetSolverType(ChSystem::SOLVER_DEM);
    // Prepare the system with a special 'system descriptor'
    // that is necessary when doing simulations with MPI.
    ChSystemDescriptorMPIgrid3D mydescriptor(mysystem.nodeMPI);  // NEW
    mysystem.ChangeSystemDescriptor(&mydescriptor);

    // Use the DEMz solver
    ChSolverDEMMPI mysolver;
    mysystem.ChangeSolverSpeed(&mysolver);

    GetLog() << "1\n";
    // Save on file the aabb of the boundaries of each domain, for debugging/visualization
    bool save_domain_boxes_on_file = false;
    CHMPIfile* domainfile = 0;
    if (save_domain_boxes_on_file) {
        // CHMPIfile::FileDelete("output\\domains.dat"); // delete prev.file, if any. Otherwise might partially
        // overwrite
        // domainfile = new
        // CHMPIfile("E:\\cygwin\\home\\heyn\\SVN_italy\\code\\code\\ChronoEngine\\bin\\data\\mpi\\output_DEM3_pills\\domains.dat",
        // CHMPIfile::CHMPI_MODE_WRONLY | CHMPIfile::CHMPI_MODE_CREATE);
        domainfile =
            new CHMPIfile("output_DEM_MPI/domains.dat", CHMPIfile::CHMPI_MODE_WRONLY | CHMPIfile::CHMPI_MODE_CREATE);
        GetLog() << "2\n";
        char buffer[100];
        sprintf(buffer, "%d, %g, %g, %g, %g, %g, %g ,\n", mysystem.nodeMPI->id_MPI,
                ((ChDomainNodeMPIgrid3D*)mysystem.nodeMPI)->min_box.x,
                ((ChDomainNodeMPIgrid3D*)mysystem.nodeMPI)->min_box.y,
                ((ChDomainNodeMPIgrid3D*)mysystem.nodeMPI)->min_box.z,
                ((ChDomainNodeMPIgrid3D*)mysystem.nodeMPI)->max_box.x,
                ((ChDomainNodeMPIgrid3D*)mysystem.nodeMPI)->max_box.y,
                ((ChDomainNodeMPIgrid3D*)mysystem.nodeMPI)->max_box.z);
        domainfile->WriteOrdered((char*)buffer, strlen(buffer));
        delete domainfile;
    }

    // add some boundary boxes
    // create_boundary_boxes(mysystem, box, num_particles);
    create_boxes_slope(mysystem, box);  // best
    // create_fancy_boundary(mysystem, aa, bb, cc, dd, ee, ff, width); //for testing
    mysystem
        .CustomEndOfStep();  //?????????????????????????????????????????????????????????????????????????????????????????
    GetLog() << "Added boxes\n";

    // CREATE THE DEM PARTICLES
    // create_falling_items(mysystem, particle_radius, num_particles, box, 0.0);//add some spheres in a box
    // GetLog() << "Added "<< num_particles <<" spheres \n";

    // Save on file the position and rotation of each box body, for debugging/visualization
    bool save_boxes_on_file = true;
    CHMPIfile* boxfile = 0;
    if (save_boxes_on_file) {
        // boxfile = new
        // CHMPIfile("E:\\cygwin\\home\\heyn\\SVN_italy\\code\\code\\ChronoEngine\\bin\\data\\mpi\\output_DEM3_pills\\boxes.dat",
        // CHMPIfile::CHMPI_MODE_WRONLY | CHMPIfile::CHMPI_MODE_CREATE);
        boxfile =
            new CHMPIfile("output_DEM_MPI/boxes.dat", CHMPIfile::CHMPI_MODE_WRONLY | CHMPIfile::CHMPI_MODE_CREATE);
        mysystem.WriteOrderedDumpState(*boxfile);
        delete boxfile;
    }

    /*
    domainfile = new
    CHMPIfile("E:\\cygwin\\home\\heyn\\SVN_italy\\code\\code\\ChronoEngine\\bin\\data\\mpi\\output_DEM3\\test2.dat",
    CHMPIfile::CHMPI_MODE_WRONLY | CHMPIfile::CHMPI_MODE_CREATE);
        sprintf(buffer1, "%d,\n", mysystem.Get_otherphysicslist()->size());
        domainfile->WriteOrdered((char*)buffer1, strlen(buffer1));
        delete domainfile;
    */

    // IMPORTANT!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // This takes care of the interaction between the bodies

    ChSharedPtr<ChContactContainerDEMMPI> my_dem_container1(new ChContactContainerDEMMPI);
    mysystem.ChangeContactContainer(my_dem_container1.get_ptr());

    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    //
    // PERFORM SOME TIME STEPS OF SIMULATION
    //

    bool save_positions_on_file = true;
    bool save_debug_files = false;

    // Initial setup
    mysystem.CustomEndOfStep();  // some prob here

    while (mysystem.GetChTime() < end_time) {
        if (curr_particles < num_particles && fAdds == addMult) {
            // add_falling_item(mysystem, particle_radius, box, 0.0, curr_particles);
            add_batch(mysystem, particle_radius, box, 0.0, num_batch, curr_particles);  // best
            // add_fancy_items(mysystem, particle_radius, 0.0, width, gg, hh, 30, curr_particles);
            mysystem
                .CustomEndOfStep();  //?????????????????????????????????????????????????????????????????????????????????????????
            // add_other_falling_item(mysystem, particle_radius, box, 950.0, curr_particles);
            curr_particles += num_batch;
            fAdds = 0;
        }
        if (save_positions_on_file && fOuts == outMult) {
            char padnumber[100];
            sprintf(padnumber, "%d", (frame_number + 10000));
            char filename[100];

            GetLog() << "\nID=" << myid << " frame=" << padnumber << "\n";
            // sprintf(filename,
            // "E:\\cygwin\\home\\heyn\\SVN_italy\\code\\code\\ChronoEngine\\bin\\data\\mpi\\output_DEM3_pills\\pos%s.dat",
            // padnumber+1);
            sprintf(filename, "output_DEM_MPI/pos%s.dat", padnumber + 1);

            // CHMPIfile::FileDelete(filename); // Delete prev.,if any. Otherwise might partially overwrite
            CHMPIfile* posfile = new CHMPIfile(filename, CHMPIfile::CHMPI_MODE_WRONLY | CHMPIfile::CHMPI_MODE_CREATE);
            // mysystem.WriteOrderedDumpAABB(*posfile); //old, dump state instead
            mysystem.WriteOrderedDumpState(*posfile);
            delete posfile;

            if (save_debug_files) {
                // sprintf(filename,
                // "E:\\cygwin\\home\\heyn\\SVN_italy\\code\\code\\ChronoEngine\\bin\\data\\mpi\\output_DEM3_pills\\debug%s.dat",
                // padnumber+1);
                sprintf(filename, "output_DEM_MPI/debug%s.dat", padnumber + 1);
                // CHMPIfile::FileDelete(filename); // Delete prev.,if any. Otherwise might partially overwrite
                CHMPIfile* debugfile =
                    new CHMPIfile(filename, CHMPIfile::CHMPI_MODE_WRONLY | CHMPIfile::CHMPI_MODE_CREATE);
                mysystem.WriteOrderedDumpDebugging(*debugfile);
                delete debugfile;
            }

            frame_number++;
            fOuts = 0;
        }

        // Advance the simulation time step
        mysystem.DoStepDynamics(time_step);
        fOuts++;
        fAdds++;
    }

    // Terminate the MPI functionality.
    CHMPI::Finalize();
    return 0;
}
