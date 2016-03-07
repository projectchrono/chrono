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

real time_step = 0.001;
real time_end = 10;
void AddContainer(ChSystemParallelDVI* sys) {
    // IDs for the two bodies
    int binId = -200;
    int mixerId = -201;

    // Create a common material
    auto mat = std::make_shared<ChMaterialSurface>();
    mat->SetFriction(0.4f);

    ChVector<> hdim(2.5, 2.5, 2.5);

    utils::CreateBoxContainer(sys, 0, mat, hdim, 0.2, Vector(0, 0, -.7), QUNIT, true, false, true, true);
}

int main(int argc, char* argv[]) {
    // Global parameter for tire:
    double tire_rad = 0.8;
    double tire_vel_z0 = -3;
    ChVector<> tire_center(.23, 0, 0);
    ChMatrix33<> tire_alignment(Q_from_AngAxis(CH_C_PI, VECT_Y));  // create rotated 180� on y

    double tire_w0 = tire_vel_z0 / tire_rad;

    // Create a Chrono::Engine physical system
    ChSystemParallelDVI my_system;
    fea_container = new ChFEAContainer(&my_system);
    fea_container->kernel_radius = .05;
    fea_container->material_density = 1200;
    fea_container->contact_mu = .1;
    fea_container->contact_cohesion = 0;
    fea_container->youngs_modulus = 1e5;  // 2e8;
    fea_container->poisson_ratio = .2;
    fea_container->contact_recovery_speed = 10000000;
    fea_container->rigid_constraint_recovery_speed = .1;
    my_system.GetSettings()->solver.solver_mode = SLIDING;
    my_system.GetSettings()->solver.max_iteration_normal = 0;
    my_system.GetSettings()->solver.max_iteration_sliding = 40;
    my_system.GetSettings()->solver.max_iteration_spinning = 0;
    my_system.GetSettings()->solver.max_iteration_bilateral = 0;
    my_system.GetSettings()->solver.tolerance = 0;
    my_system.GetSettings()->solver.alpha = 0;
    my_system.GetSettings()->solver.use_full_inertia_tensor = false;
    my_system.GetSettings()->solver.contact_recovery_speed = 1;
    my_system.GetSettings()->solver.cache_step_length = true;
    my_system.ChangeSolverType(BB);
    my_system.GetSettings()->collision.narrowphase_algorithm = NARROWPHASE_HYBRID_MPR;
    my_system.GetSettings()->collision.collision_envelope = (fea_container->kernel_radius * .05);
    my_system.GetSettings()->collision.bins_per_axis = int3(2, 2, 2);
    my_system.SetLoggingLevel(LOG_TRACE, true);
    my_system.SetLoggingLevel(LOG_INFO, true);
    double gravity = 9.81;
    my_system.Set_G_acc(ChVector<>(0, 0, -gravity));

    AddContainer(&my_system);

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

    std::vector<std::vector<std::shared_ptr<ChNodeFEAbase> > > node_sets;
    ChMeshFileLoader::FromAbaqusFile(my_mesh, GetChronoDataFile("fea/tire_3.INP").c_str(), mmaterial, node_sets,
                                     tire_center, tire_alignment);

    //
    uint num_nodes = my_mesh->GetNnodes();
    uint num_elements = my_mesh->GetNelements();

    for (unsigned int i = 0; i < my_mesh->GetNnodes(); ++i) {
        auto node_pos = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(i))->GetPos();
        ChVector<> tang_vel = Vcross(ChVector<>(tire_w0, 0, 0), node_pos - tire_center);
        std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(i))->SetPos_dt(ChVector<>(0, 0, 0) + tang_vel);
    }

    my_system.Add(my_mesh);
    my_system.Initialize();

//
// THE SOFT-REAL-TIME CYCLE
//

#ifdef CHRONO_OPENGL
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
            gl_window.DoStepDynamics(time_step);
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
        time += time_step;
    }
#endif

    return 0;
}
