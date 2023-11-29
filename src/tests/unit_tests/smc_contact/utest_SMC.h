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
// Authors: Radu Serban, Cecily Sunday
// =============================================================================
//
// Common utility functions for SMC validation tests
//
// =============================================================================

#include "chrono/ChConfig.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#ifdef SMC_MULTICORE
    #include "chrono_multicore/physics/ChSystemMulticore.h"
#endif

using namespace chrono;

std::string ForceModel_name(ChSystemSMC::ContactForceModel f) {
    switch (f) {
        case ChSystemSMC::ContactForceModel::Hooke:
            return "Hooke";
        case ChSystemSMC::ContactForceModel::Hertz:
            return "Hertz";
        case ChSystemSMC::ContactForceModel::PlainCoulomb:
            return "PlainCoulomb";
        case ChSystemSMC::ContactForceModel::Flores:
            return "Flores";
        default:
            return "";
    }
}

std::shared_ptr<ChBody> AddSphere(int id,
                                  ChSystem* sys,
                                  std::shared_ptr<ChMaterialSurfaceSMC> mat,
                                  double radius,
                                  double mass,
                                  ChVector<> pos,
                                  ChVector<> init_v) {
    // Shared parameters for the falling ball
    ChVector<> inertia(0.4 * mass * radius * radius * ChVector<>(1, 1, 1));
    ChQuaternion<> rot(1, 0, 0, 0);
    ChVector<> init_w(0, 0, 0);

    // Create a spherical body. Set body parameters and sphere collision model
    auto body = chrono_types::make_shared<ChBody>();
    body->SetIdentifier(id);
    body->SetMass(mass);
    body->SetPos(pos);
    body->SetRot(rot);
    body->SetPos_dt(init_v);
    body->SetWvel_par(init_w);
    body->SetInertiaXX(inertia);
    body->SetBodyFixed(false);
    body->SetCollide(true);

    utils::AddSphereGeometry(body.get(), mat, radius);

    // Return a pointer to the sphere object
    sys->AddBody(body);
    return body;
}

std::shared_ptr<ChBody> AddWall(int id,
                                ChSystem* sys,
                                std::shared_ptr<ChMaterialSurfaceSMC> mat,
                                ChVector<> size,
                                double mass,
                                ChVector<> pos,
                                ChVector<> init_v,
                                bool wall) {
    // Set parameters for the containing bin
    ChVector<> inertia((1.0 / 12.0) * mass * (pow(size.y(), 2) + pow(size.z(), 2)),
                       (1.0 / 12.0) * mass * (pow(size.x(), 2) + pow(size.z(), 2)),
                       (1.0 / 12.0) * mass * (pow(size.x(), 2) + pow(size.y(), 2)));
    ChQuaternion<> rot(1, 0, 0, 0);

    // Create container. Set body parameters and container collision model
    auto body = chrono_types::make_shared<ChBody>();
    body->SetIdentifier(id);
    body->SetMass(mass);
    body->SetPos(pos);
    body->SetRot(rot);
    body->SetPos_dt(init_v);
    body->SetInertiaXX(inertia);
    body->SetBodyFixed(wall);
    body->SetCollide(true);

    utils::AddBoxGeometry(body.get(), mat, size);

    // Return a pointer to the wall object
    sys->AddBody(body);
    return body;
}

#ifdef SMC_SEQUENTIAL
void SetSimParameters(
    ChSystemSMC* sys,
    const ChVector<>& gravity,
    ChSystemSMC::ContactForceModel fmodel,
    ChSystemSMC::TangentialDisplacementModel tmodel = ChSystemSMC::TangentialDisplacementModel::MultiStep) {
    // Set solver settings and collision detection parameters
    sys->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys->Set_G_acc(gravity);

    sys->SetMaxiter(100);
    sys->SetSolverTolerance(1e-3);

    sys->SetContactForceModel(fmodel);
    sys->SetTangentialDisplacementModel(tmodel);
    sys->SetAdhesionForceModel(ChSystemSMC::AdhesionForceModel::Constant);
}
#endif

#ifdef SMC_MULTICORE
void SetSimParameters(
    ChSystemMulticoreSMC* sys,
    const ChVector<>& gravity,
    ChSystemSMC::ContactForceModel fmodel,
    ChSystemSMC::TangentialDisplacementModel tmodel = ChSystemSMC::TangentialDisplacementModel::MultiStep) {
    // Set solver settings and collision detection parameters
    sys->SetCollisionSystemType(ChCollisionSystem::Type::MULTICORE);
    sys->Set_G_acc(gravity);

    sys->GetSettings()->solver.max_iteration_bilateral = 100;
    sys->GetSettings()->solver.tolerance = 1e-3;

    sys->GetSettings()->solver.contact_force_model = fmodel;  // Types: Hooke, Hertz, PlainCoulomb, Flores
    sys->GetSettings()->solver.adhesion_force_model = ChSystemSMC::AdhesionForceModel::Constant;
    sys->GetSettings()->solver.tangential_displ_mode = tmodel;

    sys->GetSettings()->collision.bins_per_axis = vec3(1, 1, 1);
    sys->GetSettings()->collision.narrowphase_algorithm = ChNarrowphase::Algorithm::HYBRID;
}
#endif

bool CalcKE(ChSystem* sys, const double& threshold) {
    const std::shared_ptr<ChBody> body = sys->Get_bodylist().at(1);

    ChVector<> eng_trn = 0.5 * body->GetMass() * body->GetPos_dt() * body->GetPos_dt();
    ChVector<> eng_rot = 0.5 * body->GetInertiaXX() * body->GetWvel_par() * body->GetWvel_par();

    double KE_trn = eng_trn.x() + eng_trn.y() + eng_trn.z();
    double KE_rot = eng_rot.x() + eng_rot.y() + eng_rot.z();
    double KE_tot = KE_trn + KE_rot;

    if (KE_tot < threshold)
        return true;
    return false;
}

bool CalcAverageKE(ChSystem* sys, const double& threshold) {
    // Calculate average KE
    double KE_trn = 0;
    double KE_rot = 0;

    for (auto body : sys->Get_bodylist()) {
        ChVector<> eng_trn = 0.5 * body->GetMass() * body->GetPos_dt() * body->GetPos_dt();
        ChVector<> eng_rot = 0.5 * body->GetInertiaXX() * body->GetWvel_par() * body->GetWvel_par();

        KE_trn += eng_trn.x() + eng_trn.y() + eng_trn.z();
        KE_rot += eng_rot.x() + eng_rot.y() + eng_rot.z();
    }

    double KE_trn_avg = KE_trn / sys->Get_bodylist().size();
    double KE_rot_avg = KE_rot / sys->Get_bodylist().size();
    double KE_tot_avg = KE_trn_avg + KE_rot_avg;

    // Return true if the calc falls below the given threshold
    if (KE_tot_avg < threshold)
        return true;
    return false;
}
