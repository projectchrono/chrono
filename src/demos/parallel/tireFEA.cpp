#include "chrono/ChConfig.h"
#include "chrono/physics/ChSystem.h"
#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLoaderUV.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/lcp/ChLcpIterativeMINRES.h"

#include "chrono_fea/ChElementTetra_4.h"
#include "chrono_fea/ChMesh.h"
#include "chrono_fea/ChMeshFileLoader.h"
#include "chrono_fea/ChContactSurfaceMesh.h"
#include "chrono_fea/ChContactSurfaceNodeCloud.h"
#include "chrono_fea/ChVisualizationFEAmesh.h"
#include "chrono_fea/ChLinkPointFrame.h"

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsGenerators.h"

#include "chrono_parallel/physics/Ch3DOFContainer.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

using namespace chrono;
using namespace chrono::fea;

ChFEAContainer* fea_container;

real time_step = 0.0005;
real time_end = 10;
void AddContainer(ChSystemParallelNSC* sys) {
    // IDs for the two bodies
    int binId = -200;
    int mixerId = -201;

    // Create a common material
    auto mat = std::make_shared<ChMaterialSurfaceNSC>();
    mat->SetFriction(0.4f);

    ChVector<> hdim(2, 2, 2);

    utils::CreateBoxContainer(sys, 0, mat, hdim, 0.1, Vector(0, 0, -1), QUNIT, true, false, true, true);
}

int main(int argc, char* argv[]) {
    // Global parameter for tire:
    double tire_rad = 0.8;
    double tire_vel_z0 = -3;
    ChVector<> tire_center(0, 0, 0);
    ChMatrix33<> tire_alignment(Q_from_AngAxis(CH_C_PI, VECT_Y));  // create rotated 180� on y

    double tire_w0 = tire_vel_z0 / tire_rad;

    // Create a Chrono::Engine physical system
    ChSystemParallelNSC my_system;
    fea_container = new ChFEAContainer(&my_system);
    fea_container->kernel_radius = .01;
    fea_container->material_density = 1000;
    fea_container->contact_mu = 0;
    fea_container->contact_cohesion = 0;
    fea_container->youngs_modulus = 1e7;  // 2e8;
    fea_container->poisson_ratio = .2;
    fea_container->contact_recovery_speed = 12;
    fea_container->beta = .1/time_step;
    fea_container->rigid_constraint_recovery_speed = .1;
    my_system.GetSettings()->solver.solver_mode = SLIDING;
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
    my_system.ChangeSolverType(BB);
    my_system.GetSettings()->collision.narrowphase_algorithm = NARROWPHASE_HYBRID_MPR;
    my_system.GetSettings()->collision.collision_envelope = (fea_container->kernel_radius * .05);
    my_system.GetSettings()->collision.bins_per_axis = vec3(20, 20, 20);
    //my_system.SetLoggingLevel(LOG_TRACE, true);
   // my_system.SetLoggingLevel(LOG_INFO, true);
    double gravity = 9.81;
    my_system.Set_G_acc(ChVector<>(0, 0, 0));

    AddContainer(&my_system);

    auto material_plate = std::make_shared<ChMaterialSurfaceNSC>();
    std::shared_ptr<ChBody> PLATE = std::make_shared<ChBody>(new chrono::collision::ChCollisionModelParallel);
    utils::InitializeObject(PLATE, 100000, material_plate, ChVector<>(0, 0, 0), QUNIT, false, true, 2, 6);
    utils::AddBoxGeometry(PLATE.get(), ChVector<>(.1, .1, .1));
    utils::FinalizeObject(PLATE, (ChSystemParallel*)&my_system);

    auto my_mesh = std::make_shared<ChMesh>();

    // Create a material, that must be assigned to each solid element in the mesh,
    // and set its parameters

    auto mmaterial = std::make_shared<ChContinuumElastic>();
    mmaterial->Set_E(0.016e9);  // rubber 0.01e9, steel 200e9
    mmaterial->Set_v(0.4);
    mmaterial->Set_RayleighDampingK(0.004);
    mmaterial->Set_density(1000);

    // Load an ABAQUS .INP tetahedron mesh file from disk, defining a tetahedron mesh.
    // Note that not all features of INP files are supported. Also, quadratic tetahedrons are promoted to linear.
    // This is much easier than creating all nodes and elements via C++ programming.
    // Ex. you can generate these .INP files using Abaqus or exporting from the SolidWorks simulation tool.
    uint current_nodes = my_system.data_manager->num_fea_nodes;

    std::vector<std::vector<std::shared_ptr<ChNodeFEAbase> > > node_sets;
    ChMeshFileLoader::FromAbaqusFile(my_mesh, GetChronoDataFile("fea/beam.INP").c_str(), mmaterial, node_sets,
                                     tire_center, tire_alignment);

    //
    uint num_nodes = my_mesh->GetNnodes();
    uint num_elements = my_mesh->GetNelements();

    //    for (unsigned int i = 0; i < my_mesh->GetNnodes(); ++i) {
    //        auto node_pos = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(i))->GetPos();
    //        ChVector<> tang_vel = Vcross(ChVector<>(tire_w0, 0, 0), node_pos - tire_center);
    //        std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(i))->SetPos_dt(ChVector<>(0, 0, 0) + tang_vel);
    //    }

    my_system.Add(my_mesh);

    for (int i = 0; i < node_sets.size(); i++) {
        printf("Node sets: %d\n", node_sets[i].size());
        for (int j = 0; j < node_sets[i].size(); j++) {
            fea_container->AddConstraint(node_sets[i][j]->GetIndex() + current_nodes, PLATE);
        }
    }

    my_system.Initialize();

    //
    // THE SOFT-REAL-TIME CYCLE
    //
    real3 initpos = my_system.data_manager->host_data.pos_node_fea[42];
    real g = 0;
#if 1
    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1280, 720, "fluidDVI", &my_system);
    gl_window.SetCamera(ChVector<>(0, -2, 0), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), .2);
    gl_window.Pause();
    // Uncomment the following two lines for the OpenGL manager to automatically
    // run the simulation in an infinite loop.
    // gl_window.StartDrawLoop(time_step);
    // return 0;
    while (true) {
        if (gl_window.Active()) {
            if (gl_window.DoStepDynamics(time_step)) {
            	real3 pos = my_system.data_manager->host_data.pos_node_fea[42] - initpos;
            	        printf("pos: %f %f %f [%f]\n", pos.x, pos.y, pos.z, g);
            	        if (g < gravity) {
            	            g += 0.001;
            	        }
            	        my_system.Set_G_acc(ChVector<>(0, 0, -g));
            }
            gl_window.Render();
        } else {
            break;
        }
    }
#else
    // Run simulation for specified time
    int num_steps = std::ceil(time_end / time_step);

    double time = 0;
    for (int i = 0; i < num_steps; i++) {
        my_system.DoStepDynamics(time_step);
        real3 pos = my_system.data_manager->host_data.pos_node_fea[42] - initpos;
        printf("pos: %f %f %f [%f]\n", pos.x, pos.y, pos.z, g);
        if (g < gravity) {
            g += 0.001;
        }
        my_system.Set_G_acc(ChVector<>(0, 0, -g));
        time += time_step;
    }
#endif

    return 0;
}
