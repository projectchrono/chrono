// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Collision test
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono_irrlicht/ChIrrApp.h"

using namespace chrono;
using namespace chrono::irrlicht;

// ====================================================================================

class ContactManager : public ChContactContainer::ReportContactCallback {
  public:
    ContactManager() {}
    virtual bool OnReportContact(const ChVector<>& pA,
                                 const ChVector<>& pB,
                                 const ChMatrix33<>& plane_coord,
                                 const double& distance,
                                 const double& eff_radius,
                                 const ChVector<>& cforce,
                                 const ChVector<>& ctorque,
                                 ChContactable* modA,
                                 ChContactable* modB) override;
};

// ====================================================================================

int main(int argc, char* argv[]) {
    // Collision settings
    enum class CollisionType { CYLINDER, CAPSULE, CYLSHELL, MESH };
    CollisionType object_model = CollisionType::CYLSHELL;
    std::string tire_mesh_file = GetChronoDataFile("vehicle/hmmwv/hmmwv_tire_fine.obj");
    ////std::string tire_mesh_file = GetChronoDataFile("vehicle/hmmwv/hmmwv_tire_coarse.obj");

    // Contact material properties
    ChContactMethod contact_method = ChContactMethod::SMC;
    bool use_mat_properties = true;

    float object_friction = 0.8f;
    float object_restitution = 0.0f;
    float object_young_modulus = 1e7f;
    float object_poisson_ratio = 0.3f;
    ////float object_adhesion = 0.0f;
    float object_kn = 2e3;
    float object_gn = 40;
    float object_kt = 2e5;
    float object_gt = 20;

    float ground_friction = 0.9f;
    float ground_restitution = 0.0f;
    float ground_young_modulus = 1e7f;
    float ground_poisson_ratio = 0.3f;
    ////float ground_adhesion = 0.0f;
    float ground_kn = 2e3;
    float ground_gn = 40;
    float ground_kt = 2e5;
    float ground_gt = 20;

    // Parameters for the falling object
    double init_height = 0.65;
    double init_x = 0.0;
    double init_z = 0.0;
    double init_roll = 0 * CH_C_DEG_TO_RAD;

    ChVector<> init_vel(0, 0, 0);
    ChVector<> init_omg(0, 0, 5);

    double radius = 0.5;  // cylinder radius
    double hlen = 0.4;    // cylinder half-length

    // Step size
    double time_step = contact_method == ChContactMethod::NSC ? 1e-3 : 1e-4;

    // Print settings
    std::cout << "-----------------------" << std::endl;
    std::cout << "Contact method:        " << (contact_method == ChContactMethod::SMC ? "SMC" : "NSC") << std::endl;
    std::cout << "Step size:             " << time_step << std::endl;
    std::cout << "Object collision type: ";
    switch (object_model) {
        case CollisionType::CYLINDER:
            std::cout << "CYLINDER" << std::endl;
            break;
        case CollisionType::CAPSULE:
            std::cout << "CAPSULE" << std::endl;
            break;
        case CollisionType::CYLSHELL:
            std::cout << "CYLSHELL" << std::endl;
            break;
        case CollisionType::MESH:
            std::cout << "MESH" << std::endl;
            break;
    }
    std::cout << "-----------------------" << std::endl;

    // Create the system
    ChSystem* system = nullptr;

    switch (contact_method) {
        case ChContactMethod::NSC:
            system = new ChSystemNSC();
            break;
        case ChContactMethod::SMC:
            system = new ChSystemSMC(use_mat_properties);
            break;
    }

    system->Set_G_acc(ChVector<>(0, -9.81, 0));

    // Create the Irrlicht visualization
    ChIrrApp application(system, L"Collision test", irr::core::dimension2d<irr::u32>(800, 600));

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(irr::core::vector3df(3, 1, (float)init_z), irr::core::vector3df(0, 0, (float)init_z));

    // Render contact forces or normals
    application.SetSymbolscale(5e-4);
    application.SetContactsDrawMode(IrrContactsDrawMode::CONTACT_FORCES);
    ////application.SetSymbolscale(1);
    ////application.SetContactsDrawMode(IrrContactsDrawMode::CONTACT_NORMALS);

    // Rotation Z->Y (because meshes used here assume Z up)
    ChQuaternion<> z2y = Q_from_AngX(-CH_C_PI_2);

    // Create the falling object
    auto object = std::shared_ptr<ChBody>(system->NewBody());
    system->AddBody(object);

    object->SetName("object");
    object->SetMass(1500);
    object->SetInertiaXX(40.0 * ChVector<>(1, 1, 0.2));
    object->SetPos(ChVector<>(init_x, init_height, init_z));
    object->SetRot(z2y * Q_from_AngX(init_roll));
    object->SetPos_dt(init_vel);
    object->SetWvel_par(init_omg);
    object->SetCollide(true);
    object->SetBodyFixed(false);

    auto object_mat = ChMaterialSurface::DefaultMaterial(contact_method);
    object_mat->SetFriction(object_friction);
    object_mat->SetRestitution(object_restitution);
    if (contact_method == ChContactMethod::SMC) {
        auto matSMC = std::static_pointer_cast<ChMaterialSurfaceSMC>(object_mat);
        matSMC->SetYoungModulus(object_young_modulus);
        matSMC->SetPoissonRatio(object_poisson_ratio);
        matSMC->SetKn(object_kn);
        matSMC->SetGn(object_gn);
        matSMC->SetKt(object_kt);
        matSMC->SetGt(object_gt);
    }

    switch (object_model) {
        case CollisionType::CYLINDER: {
            object->GetCollisionModel()->ClearModel();
            object->GetCollisionModel()->AddCylinder(object_mat, radius, radius, hlen, ChVector<>(0), ChMatrix33<>(1));
            object->GetCollisionModel()->BuildModel();

            auto cyl = chrono_types::make_shared<ChCylinderShape>();
            cyl->GetCylinderGeometry().p1 = ChVector<>(0, +hlen, 0);
            cyl->GetCylinderGeometry().p2 = ChVector<>(0, -hlen, 0);
            cyl->GetCylinderGeometry().rad = radius;
            object->AddAsset(cyl);

            break;
        }
        case CollisionType::CAPSULE: {
            object->GetCollisionModel()->ClearModel();
            object->GetCollisionModel()->AddCapsule(object_mat, radius, hlen, ChVector<>(0), ChMatrix33<>(1));
            object->GetCollisionModel()->BuildModel();

            auto cap = chrono_types::make_shared<ChCapsuleShape>();
            cap->GetCapsuleGeometry().rad = radius;
            cap->GetCapsuleGeometry().hlen = hlen;
            object->AddAsset(cap);

            break;
        }
        case CollisionType::CYLSHELL: {
            object->GetCollisionModel()->ClearModel();
            object->GetCollisionModel()->AddCylindricalShell(object_mat, radius, hlen, ChVector<>(0), ChMatrix33<>(1));
            object->GetCollisionModel()->BuildModel();

            auto cyl = chrono_types::make_shared<ChCylinderShape>();
            cyl->GetCylinderGeometry().p1 = ChVector<>(0, +hlen, 0);
            cyl->GetCylinderGeometry().p2 = ChVector<>(0, -hlen, 0);
            cyl->GetCylinderGeometry().rad = radius;
            object->AddAsset(cyl);

            break;
        }
        case CollisionType::MESH: {
            double sphere_r = 0.005;
            auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
            if (!trimesh->LoadWavefrontMesh(tire_mesh_file, true, false))
                return 1;

            object->GetCollisionModel()->ClearModel();
            object->GetCollisionModel()->AddTriangleMesh(object_mat, trimesh, false, false, ChVector<>(0),
                                                         ChMatrix33<>(1), sphere_r);
            object->GetCollisionModel()->BuildModel();

            auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
            trimesh_shape->SetMesh(trimesh);
            ////trimesh_shape->SetWireframe(true);
            object->AddAsset(trimesh_shape);

            break;
        }
    }

    auto tex = chrono_types::make_shared<ChTexture>(GetChronoDataFile("concrete.jpg"));
    object->AddAsset(tex);

    // Create ground body
    auto ground = std::shared_ptr<ChBody>(system->NewBody());
    system->AddBody(ground);

    ground->SetName("ground");
    ground->SetMass(1);
    ground->SetPos(ChVector<>(0, 0, 0));
    ground->SetCollide(true);
    ground->SetBodyFixed(true);

    auto ground_mat = ChMaterialSurface::DefaultMaterial(contact_method);
    ground_mat->SetFriction(ground_friction);
    ground_mat->SetRestitution(ground_restitution);
    if (contact_method == ChContactMethod::SMC) {
        auto matSMC = std::static_pointer_cast<ChMaterialSurfaceSMC>(ground_mat);
        matSMC->SetYoungModulus(ground_young_modulus);
        matSMC->SetPoissonRatio(ground_poisson_ratio);
        matSMC->SetKn(ground_kn);
        matSMC->SetGn(ground_gn);
        matSMC->SetKt(ground_kt);
        matSMC->SetGt(ground_gt);
    }

    double hx = 1;
    double hy = 0.5;
    double hz = 1;

    ground->GetCollisionModel()->ClearModel();
    ground->GetCollisionModel()->AddBox(ground_mat, hx, hy, hz, ChVector<>(0, -hy, 0));
    ground->GetCollisionModel()->BuildModel();

    auto box = chrono_types::make_shared<ChBoxShape>();
    box->GetBoxGeometry().Size = ChVector<>(hx, hy, hz);
    box->GetBoxGeometry().Pos = ChVector<>(0, -hy, 0);
    ground->AddAsset(box);

    auto texture = chrono_types::make_shared<ChTexture>();
    texture->SetTextureFilename(GetChronoDataFile("textures/checker1.png"));
    texture->SetTextureScale(4, 2);
    ground->AddAsset(texture);

    // Complete asset construction
    application.AssetBindAll();
    application.AssetUpdateAll();

    auto cmanager = chrono_types::make_shared<ContactManager>();

    // Simulation loop
    while (application.GetDevice()->run()) {
        application.BeginScene();
        application.DrawAll();
        application.EndScene();

        system->DoStepDynamics(time_step);

        ////if (system->GetNcontacts()) {
        ////    // Report all contacts
        ////    std::cout << system->GetChTime() << "  " << system->GetNcontacts() << std::endl;
        ////    system->GetContactContainer()->ReportAllContacts(cmanager);

        ////    // Cumulative contact force on object
        ////    ChVector<> frc1 = object->GetContactForce();
        ////    ChVector<> trq1 = object->GetContactTorque();
        ////    printf("  Contact force at COM:  %7.3f  %7.3f  %7.3f", frc1.x(), frc1.y(), frc1.z());
        ////    printf("  Contact torque at COM: %7.3f  %7.3f  %7.3f\n", trq1.x(), trq1.y(), trq1.z());
        ////}
    }

    return 0;
}

// ====================================================================================

bool ContactManager::OnReportContact(const ChVector<>& pA,
                                     const ChVector<>& pB,
                                     const ChMatrix33<>& plane_coord,
                                     const double& distance,
                                     const double& eff_radius,
                                     const ChVector<>& cforce,
                                     const ChVector<>& ctorque,
                                     ChContactable* modA,
                                     ChContactable* modB) {
    auto bodyA = static_cast<ChBody*>(modA);
    auto bodyB = static_cast<ChBody*>(modB);

    std::cout << "  " << bodyA->GetName() << "  " << bodyB->GetName() << std::endl;
    std::cout << "  " << distance << std::endl;
    std::cout << "  " << pA << "    " << pB << std::endl;
    std::cout << "  " << plane_coord.Get_A_Xaxis() << std::endl;
    std::cout << std::endl;

    return true;
}
