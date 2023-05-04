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
#include "chrono/core/ChRealtimeStep.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

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
    // Collision detection sys
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
    double size_x = 8;
    double size_y = 1;
    double size_z = 4;

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
    // Create the sys
    // -----------------

    ChSystemMulticore* sys = nullptr;

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
            sys = sysNSC;
            break;
        }
        case ChContactMethod::SMC: {
            auto sysSMC = new ChSystemMulticoreSMC();
            sysSMC->GetSettings()->solver.use_material_properties = use_mat_properties;
            sysSMC->GetSettings()->solver.contact_force_model = ChSystemSMC::Hertz;
            sysSMC->GetSettings()->solver.tangential_displ_mode = ChSystemSMC::OneStep;
            sysSMC->GetSettings()->collision.collision_envelope = 0;
            sys = sysSMC;
            break;
        }
    }

    sys->SetCollisionSystemType(collision_type);
    sys->Set_G_acc(ChVector<>(0, -9.81, 0));
    // sys->Set_G_acc(ChVector<>(0, 0, 0));

    // Set number of threads
    sys->SetNumThreads(2);

    sys->GetSettings()->solver.use_full_inertia_tensor = false;
    sys->GetSettings()->solver.tolerance = 0;

    sys->GetSettings()->collision.narrowphase_algorithm = narrowphase_algorithm;
    sys->GetSettings()->collision.bins_per_axis = vec3(1, 1, 1);

    // Rotation Z->Y (because meshes used here assume Z up)
    ChQuaternion<> z2y = Q_from_AngX(-CH_C_PI_2);

    // Create the falling object
    auto object = std::shared_ptr<ChBody>(sys->NewBody());
    sys->AddBody(object);

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

            auto sphere = chrono_types::make_shared<ChSphereShape>(radius);
            sphere->SetTexture(GetChronoDataFile("textures/concrete.jpg"));
            object->AddVisualShape(sphere);

            break;
        }
        case CollisionShape::CYLINDER: {
            object->GetCollisionModel()->ClearModel();
            object->GetCollisionModel()->AddCylinder(object_mat, radius, 2 * hlen, ChVector<>(0),
                                                     Q_from_AngX(CH_C_PI_2));
            object->GetCollisionModel()->BuildModel();

            auto cyl = chrono_types::make_shared<ChCylinderShape>(radius, 2 * hlen);
            cyl->SetTexture(GetChronoDataFile("textures/concrete.jpg"));
            object->AddVisualShape(cyl, ChFrame<>(VNULL, Q_from_AngX(CH_C_PI_2)));

            break;
        }
        case CollisionShape::CAPSULE: {
            object->GetCollisionModel()->ClearModel();
            object->GetCollisionModel()->AddCapsule(object_mat, radius, 2 * hlen, ChVector<>(0),
                                                    Q_from_AngX(CH_C_PI_2));
            object->GetCollisionModel()->BuildModel();

            auto cap = chrono_types::make_shared<ChCapsuleShape>(radius, 2 * hlen);
            cap->SetTexture(GetChronoDataFile("textures/concrete.jpg"));
            object->AddVisualShape(cap, ChFrame<>(VNULL, Q_from_AngX(CH_C_PI_2)));

            break;
        }
        case CollisionShape::CYLSHELL: {
            object->GetCollisionModel()->ClearModel();
            object->GetCollisionModel()->AddCylindricalShell(object_mat, radius, 2 * hlen, ChVector<>(0),
                                                             Q_from_AngX(CH_C_PI_2));
            object->GetCollisionModel()->BuildModel();

            auto cyl = chrono_types::make_shared<ChCylinderShape>(radius, 2 * hlen);
            cyl->SetTexture(GetChronoDataFile("textures/concrete.jpg"));
            object->AddVisualShape(cyl, ChFrame<>(VNULL, Q_from_AngX(CH_C_PI_2)));

            break;
        }
        case CollisionShape::MESH: {
            double sphere_r = 0.005;
            auto trimesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(tire_mesh_file);
            if (!trimesh)
                return 1;

            object->GetCollisionModel()->ClearModel();
            object->GetCollisionModel()->AddTriangleMesh(object_mat, trimesh, false, false, ChVector<>(0),
                                                         ChMatrix33<>(1), sphere_r);
            object->GetCollisionModel()->BuildModel();

            auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
            trimesh_shape->SetMesh(trimesh);
            ////trimesh_shape->SetWireframe(true);
            trimesh_shape->SetTexture(GetChronoDataFile("textures/concrete.jpg"));
            object->AddVisualShape(trimesh_shape);

            break;
        }
    }

    // Create ground body
    auto ground = std::shared_ptr<ChBody>(sys->NewBody());
    sys->AddBody(ground);

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
    ground->GetCollisionModel()->AddBox(ground_mat, size_x, size_y, size_z, ChVector<>(0, -size_y / 2, 0));
    ground->GetCollisionModel()->BuildModel();

    auto box = chrono_types::make_shared<ChBoxShape>(size_x, size_y, size_z);
    box->SetTexture(GetChronoDataFile("textures/checker1.png"), 4, 2);
    ground->AddVisualShape(box, ChFrame<>(ChVector<>(0, -size_y / 2, 0), QUNIT));

    // Create the Irrlicht visualization sys
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Collision test (Chrono::Multicore)");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(3, 1, init_z), ChVector<>(0, 0, init_z));
    vis->AddTypicalLights();

    // Render contact forces or normals
    vis->SetSymbolScale(5e-4);
    vis->EnableContactDrawing(ContactsDrawMode::CONTACT_FORCES);
    ////vis->SetSymbolScale(1);
    ////vis->EnableContactDrawing(ContactsDrawMode::CONTACT_NORMALS);

    auto cmanager = chrono_types::make_shared<ContactManager>();

    // Simulation loop
    ChRealtimeStepTimer rt;
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        sys->DoStepDynamics(time_step);

        /*
        std::cout << "----\nTime: " << sys->GetChTime() << "  " << sys->GetNcontacts() << std::endl;
        std::cout << "Object position: " << object->GetPos() << std::endl;
        if (sys->GetNcontacts()) {
            // Report all contacts
            sys->GetContactContainer()->ReportAllContacts(cmanager);

            // Cumulative contact force on object
            sys->GetContactContainer()->ComputeContactForces();
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
