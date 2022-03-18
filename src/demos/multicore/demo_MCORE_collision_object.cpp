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

#include "chrono_multicore/physics/ChSystemMulticore.h"
#include "chrono_multicore/collision/ChCollisionSystemChronoMulticore.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono/core/ChRealtimeStep.h"

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
    // Collision detection system
    collision::ChCollisionSystemType collision_type = collision::ChCollisionSystemType::CHRONO;

    // Narrowphase algorithm
    collision::ChNarrowphase::Algorithm narrowphase_algorithm = collision::ChNarrowphase::Algorithm::HYBRID;

    // Collision envelope (NSC only)
    double collision_envelope = 0.05;

    // Collision shape
    enum class CollisionShape { SPHERE, CYLINDER, CAPSULE, CYLSHELL, MESH };
    CollisionShape object_model = CollisionShape::CYLSHELL;

    std::string tire_mesh_file = GetChronoDataFile("vehicle/hmmwv/hmmwv_tire_fine.obj");
    ////std::string tire_mesh_file = GetChronoDataFile("vehicle/hmmwv/hmmwv_tire_coarse.obj");

    // Contact material properties
    ChContactMethod contact_method = ChContactMethod::SMC;
    bool use_mat_properties = true;

    float object_friction = 0.8f;
    float object_restitution = 0.0f;
    float object_young_modulus = 2e7f;
    float object_poisson_ratio = 0.3f;
    ////float object_adhesion = 0.0f;
    float object_kn = 2e5;
    float object_gn = 40;
    float object_kt = 2e5;
    float object_gt = 20;

    float ground_friction = 0.9f;
    float ground_restitution = 0.0f;
    float ground_young_modulus = 2e7f;
    float ground_poisson_ratio = 0.3f;
    ////float ground_adhesion = 0.0f;
    float ground_kn = 2e5;
    float ground_gn = 40;
    float ground_kt = 2e5;
    float ground_gt = 20;

    // Parameters for the falling object
    double init_height = 0.65;
    double init_x = 0.0;
    double init_z = 0.0;
    double init_roll = 30 * CH_C_DEG_TO_RAD;

    ChVector<> init_vel(0, 0, 0);
    ChVector<> init_omg(0, 0, 0);

    double radius = 0.5;  // cylinder radius
    double hlen = 0.1;    // cylinder half-length

    // Fixed box (ground) dimensions
    double hx = 4;
    double hy = 0.5;
    double hz = 2;

    // Step size
    double time_step = contact_method == ChContactMethod::NSC ? 1e-3 : 1e-4;

    // Print settings
    std::cout << "-----------------------" << std::endl;
    std::cout << "Contact method:        " << (contact_method == ChContactMethod::SMC ? "SMC" : "NSC") << std::endl;
    std::cout << "Step size:             " << time_step << std::endl;
    std::cout << "Object collision type: ";
    switch (object_model) {
        case CollisionShape::SPHERE:
            std::cout << "SPHERE" << std::endl;
            break;
        case CollisionShape::CYLINDER:
            std::cout << "CYLINDER" << std::endl;
            break;
        case CollisionShape::CAPSULE:
            std::cout << "CAPSULE" << std::endl;
            break;
        case CollisionShape::CYLSHELL:
            std::cout << "CYLSHELL" << std::endl;
            break;
        case CollisionShape::MESH:
            std::cout << "MESH" << std::endl;
            break;
    }
    std::cout << "-----------------------" << std::endl;

    // -----------------
    // Create the system
    // -----------------

    ChSystemMulticore* system = nullptr;

    switch (contact_method) {
        case ChContactMethod::NSC: {
            auto sysNSC = new ChSystemMulticoreNSC();
            sysNSC->ChangeSolverType(SolverType::APGD);
            sysNSC->GetSettings()->solver.solver_mode = SolverMode::SPINNING;
            sysNSC->GetSettings()->solver.max_iteration_normal = 0;
            sysNSC->GetSettings()->solver.max_iteration_sliding = 0;
            sysNSC->GetSettings()->solver.max_iteration_spinning = 100;
            sysNSC->GetSettings()->solver.max_iteration_bilateral = 0;
            sysNSC->GetSettings()->solver.alpha = 0;
            sysNSC->GetSettings()->solver.contact_recovery_speed = 10;
            sysNSC->GetSettings()->collision.collision_envelope = collision_envelope;
            system = sysNSC;
            break;
        }
        case ChContactMethod::SMC: {
            auto sysSMC = new ChSystemMulticoreSMC();
            sysSMC->GetSettings()->solver.use_material_properties = use_mat_properties;
            sysSMC->GetSettings()->solver.contact_force_model = ChSystemSMC::Hertz;
            sysSMC->GetSettings()->solver.tangential_displ_mode = ChSystemSMC::OneStep;
            sysSMC->GetSettings()->collision.collision_envelope = 0;
            system = sysSMC;
            break;
        }
    }

    system->SetCollisionSystemType(collision_type);
    system->Set_G_acc(ChVector<>(0, -9.81, 0));
    //system->Set_G_acc(ChVector<>(0, 0, 0));

    // Set number of threads
    system->SetNumThreads(2);

    system->GetSettings()->solver.use_full_inertia_tensor = false;
    system->GetSettings()->solver.tolerance = 0;

    system->GetSettings()->collision.narrowphase_algorithm = narrowphase_algorithm;
    system->GetSettings()->collision.bins_per_axis = vec3(1, 1, 1);

    // Create the Irrlicht visualization
    ChIrrApp application(system, L"Collision test (Chrono::Multicore)", irr::core::dimension2d<irr::u32>(800, 600));
    application.AddLogo();
    application.AddSkyBox();
    application.AddTypicalLights();
    application.AddCamera(irr::core::vector3df(3, 1, (float)init_z), irr::core::vector3df(0, 0, (float)init_z));

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
    object->SetWvel_loc(init_omg);
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
        case CollisionShape::SPHERE: {
            object->GetCollisionModel()->ClearModel();
            object->GetCollisionModel()->AddSphere(object_mat, radius, ChVector<>(0));
            object->GetCollisionModel()->BuildModel();

            auto sphere = chrono_types::make_shared<ChSphereShape>();
            sphere->GetSphereGeometry().rad = radius;
            object->AddAsset(sphere);

            break;
        }
        case CollisionShape::CYLINDER: {
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
        case CollisionShape::CAPSULE: {
            object->GetCollisionModel()->ClearModel();
            object->GetCollisionModel()->AddCapsule(object_mat, radius, hlen, ChVector<>(0), ChMatrix33<>(1));
            object->GetCollisionModel()->BuildModel();

            auto cap = chrono_types::make_shared<ChCapsuleShape>();
            cap->GetCapsuleGeometry().rad = radius;
            cap->GetCapsuleGeometry().hlen = hlen;
            object->AddAsset(cap);

            break;
        }
        case CollisionShape::CYLSHELL: {
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
        case CollisionShape::MESH: {
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

    auto tex = chrono_types::make_shared<ChTexture>(GetChronoDataFile("textures/concrete.jpg"));
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
    ChRealtimeStepTimer rt;
    while (application.GetDevice()->run()) {
        application.BeginScene();
        application.DrawAll();
        application.EndScene();

        system->DoStepDynamics(time_step);

        /*
        std::cout << "----\nTime: " << system->GetChTime() << "  " << system->GetNcontacts() << std::endl;
        std::cout << "Object position: " << object->GetPos() << std::endl;
        if (system->GetNcontacts()) {
            // Report all contacts
            system->GetContactContainer()->ReportAllContacts(cmanager);

            // Cumulative contact force on object
            system->GetContactContainer()->ComputeContactForces();
            ChVector<> frc1 = object->GetContactForce();
            ChVector<> trq1 = object->GetContactTorque();
            std::cout << "Contact force at COM:  " << frc1 << std::endl;
            std::cout << "Contact torque at COM: " << trq1 << std::endl;
        }
        */

        rt.Spin(time_step);
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
    std::cout << "  " << bodyA->GetPos() << std::endl;
    std::cout << "  " << distance << std::endl;
    std::cout << "  " << pA << "    " << pB << std::endl;
    std::cout << "  " << plane_coord.Get_A_Xaxis() << std::endl;
    std::cout << std::endl;

    return true;
}
