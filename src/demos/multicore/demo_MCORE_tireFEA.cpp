#include "chrono/ChConfig.h"

#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChMeshFileLoader.h"

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsGenerators.h"

#include "chrono_multicore/physics/ChSystemMulticore.h"
#include "chrono_multicore/physics/Ch3DOFContainer.h"

#include "chrono_opengl/ChOpenGLWindow.h"

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::collision;

real time_step = 0.0005;

void AddContainer(ChSystemMulticoreNSC* sys) {
    auto mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    mat->SetFriction(0.4f);

    ChVector<> hdim(2, 2, 2);

    auto box = utils::CreateBoxContainer(sys, 0, mat, hdim, 0.1, Vector(0, 0, -1), QUNIT, true, false, true, true);
}

int main(int argc, char* argv[]) {
    // Global parameter for tire:
    double tire_rad = 0.8;
    double tire_vel_z0 = -3;
    ChVector<> tire_center(0, 0, 0);
    ChMatrix33<> tire_alignment(Q_from_AngAxis(CH_C_PI, VECT_Y));  // create rotated 180 deg on y

    double tire_w0 = tire_vel_z0 / tire_rad;

    // Create a Chrono::Multicore physical system
    ChSystemMulticoreNSC my_system;

    auto fea_container = chrono_types::make_shared<ChFEAContainer>();
    my_system.Add3DOFContainer(fea_container);

    fea_container->kernel_radius = .01;
    fea_container->material_density = 1000;
    fea_container->contact_mu = 0;
    fea_container->contact_cohesion = 0;
    fea_container->youngs_modulus = 1e7;  // 2e8;
    fea_container->poisson_ratio = .2;
    fea_container->contact_recovery_speed = 12;
    fea_container->beta = .1 / time_step;
    fea_container->rigid_constraint_recovery_speed = .1;

    my_system.GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    my_system.GetSettings()->solver.max_iteration_normal = 0;
    my_system.GetSettings()->solver.max_iteration_sliding = 80;
    my_system.GetSettings()->solver.max_iteration_spinning = 0;
    my_system.GetSettings()->solver.max_iteration_bilateral = 0;
    my_system.GetSettings()->solver.max_iteration_fem = 0;
    my_system.GetSettings()->solver.tolerance = 0;
    my_system.GetSettings()->solver.alpha = 0;
    my_system.GetSettings()->solver.use_full_inertia_tensor = false;
    my_system.GetSettings()->solver.contact_recovery_speed = 1;
    my_system.GetSettings()->solver.cache_step_length = true;
    my_system.ChangeSolverType(SolverType::BB);
    my_system.GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_HYBRID_MPR;
    my_system.GetSettings()->collision.collision_envelope = (fea_container->kernel_radius * .05);
    my_system.GetSettings()->collision.bins_per_axis = vec3(20, 20, 20);
    // my_system.SetLoggingLevel(LOG_TRACE, true);
    // my_system.SetLoggingLevel(LOG_INFO, true);

    double gravity = 9.81;
    my_system.Set_G_acc(ChVector<>(0, 0, 0));

    ////AddContainer(&my_system);

    auto cmaterial = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    cmaterial->SetFriction(0.4f);
    auto box = utils::CreateBoxContainer(&my_system, 0, cmaterial, ChVector<>(2, 2, 2), 0.1, ChVector<>(0, 0, -1),
                                         QUNIT, true, false, true, true);

    auto PLATE = chrono_types::make_shared<ChBody>(chrono_types::make_shared<ChCollisionModelMulticore>());
    utils::InitializeObject(PLATE, 100000, cmaterial, ChVector<>(0, 0, 0), QUNIT, false, true, 2, 6);
    utils::AddBoxGeometry(PLATE.get(), ChVector<>(.1, .1, .1));
    utils::FinalizeObject(PLATE, (ChSystemMulticore*)&my_system);

    auto my_mesh = chrono_types::make_shared<ChMesh>();

    // Create a material, that must be assigned to each solid element in the mesh, and set its parameters
    auto mmaterial = chrono_types::make_shared<ChContinuumElastic>();
    mmaterial->Set_E(0.016e9);  // rubber 0.01e9, steel 200e9
    mmaterial->Set_v(0.4);
    mmaterial->Set_RayleighDampingK(0.004);
    mmaterial->Set_density(1000);

    // Load an ABAQUS .INP tetahedron mesh file from disk, defining a tetahedron mesh.
    // Note that not all features of INP files are supported. Also, quadratic tetahedrons are promoted to linear.
    uint current_nodes = my_system.data_manager->num_fea_nodes;
    std::map<std::string, std::vector<std::shared_ptr<ChNodeFEAbase> > > node_sets;
    ChMeshFileLoader::FromAbaqusFile(my_mesh,
                                     GetChronoDataFile("models/tractor_wheel/tractor_wheel_coarse.INP").c_str(),
                                     mmaterial, node_sets, tire_center, tire_alignment);

    //
    uint num_nodes = my_mesh->GetNnodes();
    uint num_elements = my_mesh->GetNelements();

    //    for (unsigned int i = 0; i < my_mesh->GetNnodes(); ++i) {
    //        auto node_pos = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(i))->GetPos();
    //        ChVector<> tang_vel = Vcross(ChVector<>(tire_w0, 0, 0), node_pos - tire_center);
    //        std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(i))->SetPos_dt(ChVector<>(0, 0, 0) + tang_vel);
    //    }

    my_system.Add(my_mesh);

    for (auto nodeset_it = node_sets.begin(); nodeset_it != node_sets.end(); ++nodeset_it) {
        std::cout << "number of node sets: " << nodeset_it->second.size() << std::endl;
        for (auto j = 0; j < nodeset_it->second.size(); j++) {
            fea_container->AddConstraint(nodeset_it->second[j]->GetIndex() + current_nodes, PLATE);
        }
    }

    my_system.Initialize();

    real3 initpos = my_system.data_manager->host_data.pos_node_fea[42];
    real g = 0;

    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1280, 720, "tireFEA", &my_system);
    gl_window.SetCamera(ChVector<>(0, -2, 0), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), .2f);
    gl_window.Pause();
    while (true) {
        if (gl_window.Active()) {
            if (gl_window.DoStepDynamics(time_step)) {
                real3 pos = my_system.data_manager->host_data.pos_node_fea[42] - initpos;
                std::cout << "pos: " << pos << " " << g << std::endl;
                if (g < gravity) {
                    g += 0.1;
                }
                my_system.Set_G_acc(ChVector<>(0, 0, -g));
            }
            gl_window.Render();
        } else {
            break;
        }
    }

    return 0;
}
