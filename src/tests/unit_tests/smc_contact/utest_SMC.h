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

std::shared_ptr<ChBody> AddSphere(ChSystem* sys,
                                  std::shared_ptr<ChContactMaterialSMC> mat,
                                  double radius,
                                  double mass,
                                  ChVector3d pos,
                                  ChVector3d init_v) {
    // Shared parameters for the falling ball
    ChVector3d inertia(0.4 * mass * radius * radius * ChVector3d(1, 1, 1));
    ChQuaternion<> rot(1, 0, 0, 0);
    ChVector3d init_w(0, 0, 0);

    // Create a spherical body. Set body parameters and sphere collision model
    auto body = chrono_types::make_shared<ChBody>();
    body->SetMass(mass);
    body->SetPos(pos);
    body->SetRot(rot);
    body->SetPosDt(init_v);
    body->SetAngVelParent(init_w);
    body->SetInertiaXX(inertia);
    body->SetFixed(false);
    body->EnableCollision(true);

    utils::AddSphereGeometry(body.get(), mat, radius);

    // Return a pointer to the sphere object
    sys->AddBody(body);
    return body;
}

std::shared_ptr<ChBody> AddWall(ChSystem* sys,
                                std::shared_ptr<ChContactMaterialSMC> mat,
                                ChVector3d size,
                                double mass,
                                ChVector3d pos,
                                ChVector3d init_v,
                                bool wall) {
    // Set parameters for the containing bin
    ChVector3d inertia((1.0 / 12.0) * mass * (std::pow(size.y(), 2) + std::pow(size.z(), 2)),
                       (1.0 / 12.0) * mass * (std::pow(size.x(), 2) + std::pow(size.z(), 2)),
                       (1.0 / 12.0) * mass * (std::pow(size.x(), 2) + std::pow(size.y(), 2)));
    ChQuaternion<> rot(1, 0, 0, 0);

    // Create container. Set body parameters and container collision model
    auto body = chrono_types::make_shared<ChBody>();
    body->SetMass(mass);
    body->SetPos(pos);
    body->SetRot(rot);
    body->SetPosDt(init_v);
    body->SetInertiaXX(inertia);
    body->SetFixed(wall);
    body->EnableCollision(true);

    utils::AddBoxGeometry(body.get(), mat, size);

    // Return a pointer to the wall object
    sys->AddBody(body);
    return body;
}

#ifdef SMC_SEQUENTIAL
void SetSimParameters(
    ChSystemSMC* sys,
    const ChVector3d& gravity,
    ChSystemSMC::ContactForceModel fmodel,
    ChSystemSMC::TangentialDisplacementModel tmodel = ChSystemSMC::TangentialDisplacementModel::MultiStep) {
    // Set solver settings and collision detection parameters
    sys->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys->SetGravitationalAcceleration(gravity);

    if (sys->GetSolver()->IsIterative())
        sys->GetSolver()->AsIterative()->SetTolerance(1e-3);

    sys->SetContactForceModel(fmodel);
    sys->SetTangentialDisplacementModel(tmodel);
    sys->SetAdhesionForceModel(ChSystemSMC::AdhesionForceModel::Constant);
}
#endif

#ifdef SMC_MULTICORE
void SetSimParameters(
    ChSystemMulticoreSMC* sys,
    const ChVector3d& gravity,
    ChSystemSMC::ContactForceModel fmodel,
    ChSystemSMC::TangentialDisplacementModel tmodel = ChSystemSMC::TangentialDisplacementModel::MultiStep) {
    // Set solver settings and collision detection parameters
    sys->SetCollisionSystemType(ChCollisionSystem::Type::MULTICORE);
    sys->SetGravitationalAcceleration(gravity);

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
    const std::shared_ptr<ChBody> body = sys->GetBodies().at(1);

    ChVector3d eng_trn = 0.5 * body->GetMass() * body->GetPosDt() * body->GetPosDt();
    ChVector3d eng_rot = 0.5 * body->GetInertiaXX() * body->GetAngVelParent() * body->GetAngVelParent();

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

    for (auto body : sys->GetBodies()) {
        ChVector3d eng_trn = 0.5 * body->GetMass() * body->GetPosDt() * body->GetPosDt();
        ChVector3d eng_rot = 0.5 * body->GetInertiaXX() * body->GetAngVelParent() * body->GetAngVelParent();

        KE_trn += eng_trn.x() + eng_trn.y() + eng_trn.z();
        KE_rot += eng_rot.x() + eng_rot.y() + eng_rot.z();
    }

    double KE_trn_avg = KE_trn / sys->GetBodies().size();
    double KE_rot_avg = KE_rot / sys->GetBodies().size();
    double KE_tot_avg = KE_trn_avg + KE_rot_avg;

    // Return true if the calc falls below the given threshold
    if (KE_tot_avg < threshold)
        return true;
    return false;
}
