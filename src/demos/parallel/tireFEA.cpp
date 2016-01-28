#include "chrono/ChConfig.h"
#include "chrono/physics/ChSystem.h"
#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLoaderUV.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/lcp/ChLcpIterativeMINRES.h"

#include "chrono_fea/ChElementTetra_4.h"
#include "chrono_fea/ChMesh.h"
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
    ChSharedPtr<ChMaterialSurface> mat(new ChMaterialSurface);
    mat->SetFriction(0.4f);

    ChVector<> hdim(2.5, 2.5, 2.5);

    utils::CreateBoxContainer(sys, 0, mat, hdim, 0.2, Vector(0, 0, -1), QUNIT, true, false, true, true);
}

int main(int argc, char* argv[]) {
    // Global parameter for tire:
    double tire_rad = 0.8;
    double tire_vel_z0 = -3;
    ChVector<> tire_center(0, 0, .3);
    ChMatrix33<> tire_alignment(Q_from_AngAxis(CH_C_PI, VECT_Y));  // create rotated 180� on y

    double tire_w0 = tire_vel_z0 / tire_rad;

    // Create a Chrono::Engine physical system
    ChSystemParallelDVI my_system;
    fea_container = new ChFEAContainer(&my_system);
    fea_container->kernel_radius = .05;
    fea_container->material_density = 1200;
    fea_container->contact_mu = 1;
    fea_container->contact_cohesion = 0;
    fea_container->youngs_modulus = 5e5;//2e8;
    fea_container->poisson_ratio = .2;
    fea_container->contact_recovery_speed = 10000000;
    my_system.GetSettings()->solver.solver_mode = SLIDING;
    my_system.GetSettings()->solver.max_iteration_normal = 0;
    my_system.GetSettings()->solver.max_iteration_sliding = 40;
    my_system.GetSettings()->solver.max_iteration_spinning = 0;
    my_system.GetSettings()->solver.max_iteration_bilateral = 0;
    my_system.GetSettings()->solver.tolerance = 0;
    my_system.GetSettings()->solver.alpha = 0;
    my_system.GetSettings()->solver.use_full_inertia_tensor = false;
    my_system.GetSettings()->collision.use_two_level = false;
    my_system.GetSettings()->solver.contact_recovery_speed = 1;
    my_system.GetSettings()->solver.cache_step_length = true;
    my_system.ChangeSolverType(BB);
    my_system.GetSettings()->collision.narrowphase_algorithm = NARROWPHASE_HYBRID_MPR;
    my_system.GetSettings()->collision.collision_envelope = (fea_container->kernel_radius * .05);
    my_system.GetSettings()->collision.bins_per_axis = int3(2, 2, 2);
    my_system.SetLoggingLevel(LOG_TRACE, true);
    my_system.SetLoggingLevel(LOG_INFO, true);
    double gravity =  9.81;
    my_system.Set_G_acc(ChVector<>(0, 0, -gravity));

     AddContainer(&my_system);

    //
    // CREATE THE PHYSICAL SYSTEM
    //

    // Create the surface material, containing information
    // about friction etc.

    ChSharedPtr<ChMaterialSurfaceDEM> mysurfmaterial(new ChMaterialSurfaceDEM);
    mysurfmaterial->SetYoungModulus(10e4);
    mysurfmaterial->SetFriction(0.3f);
    mysurfmaterial->SetRestitution(0.2f);
    mysurfmaterial->SetAdhesion(0);

    ChSharedPtr<ChMaterialSurfaceDEM> mysurfmaterial2(new ChMaterialSurfaceDEM);
    mysurfmaterial->SetYoungModulus(30e4);
    mysurfmaterial->SetFriction(0.3f);
    mysurfmaterial->SetRestitution(0.2f);
    mysurfmaterial->SetAdhesion(0);

    // RIGID BODIES
    // Create some rigid bodies, for instance a floor:

    // FINITE ELEMENT MESH
    // Create a mesh, that is a container for groups
    // of FEA elements and their referenced nodes.

    ChSharedPtr<ChMesh> my_mesh(new ChMesh);

    // Create a material, that must be assigned to each solid element in the mesh,
    // and set its parameters

    ChSharedPtr<ChContinuumElastic> mmaterial(new ChContinuumElastic);
    mmaterial->Set_E(0.016e9);  // rubber 0.01e9, steel 200e9
    mmaterial->Set_v(0.4);
    mmaterial->Set_RayleighDampingK(0.004);
    mmaterial->Set_density(1000);

    // Load an ABAQUS .INP tetahedron mesh file from disk, defining a tetahedron mesh.
    // Note that not all features of INP files are supported. Also, quadratic tetahedrons are promoted to linear.
    // This is much easier than creating all nodes and elements via C++ programming.
    // Ex. you can generate these .INP files using Abaqus or exporting from the SolidWorks simulation tool.

    std::vector<std::vector<ChSharedPtr<ChNodeFEAbase> > > node_sets;

    //    my_mesh->LoadFromAbaqusFile(GetChronoDataFile("fea/tractor_wheel_coarse.INP").c_str(), mmaterial, node_sets,
    //                                tire_center, tire_alignment);
//Ruota_V3_Piena.INP
    my_mesh->LoadFromAbaqusFile(GetChronoDataFile("fea/tire.INP").c_str(), mmaterial, node_sets, tire_center,
                                tire_alignment);

    // Apply initial speed and angular speed
//    double speed_x0 = 0.5;
    for (unsigned int i = 0; i < my_mesh->GetNnodes(); ++i) {
        ChVector<> node_pos = my_mesh->GetNode(i).DynamicCastTo<ChNodeFEAxyz>()->GetPos();
        ChVector<> tang_vel = Vcross(ChVector<>(tire_w0, 0, 0), node_pos - tire_center);
        my_mesh->GetNode(i).DynamicCastTo<ChNodeFEAxyz>()->SetPos_dt(ChVector<>(0, 0, 0) + tang_vel);
    }

    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);
    // std::vector<real3> pn(4);
    // std::vector<real3> vn(4);

    //
    //    pn[0] = real3(1.000000, -0.180000, 1.500000);
    //    pn[1] = real3(1.000000, -0.180000, -0.500000);
    //    pn[2] = real3(1.000000, 1.820000, -0.500000);
    //    pn[3] = real3(1.000000, 1.820000, 1.500000);
    //    pn[4] = real3(-1.000000, -0.180000, -0.500000);
    //    pn[5] = real3(-1.000000, 1.820000, -0.500000);
    //    pn[6] = real3(-1.000000, -0.180000, 1.500000);
    //    pn[7] = real3(-1.000000, 1.820000, 1.500000);
    //    pn[8] = real3(-0.053533, 0.891667, 0.474167);

    // pn[0] = real3(1.000000, 1.820000, -0.500000);
    // pn[1] = real3(-0.053533, 0.891667, 0.474167);
    // pn[2] = real3(-1.000000, 1.820000, 1.500000);
    // pn[3] = real3(1.000000, 1.820000, 1.500000);

    // std::vector<uint4> elements(1);
    // elements[0] = _make_uint4(2, 8, 7, 3);
    // elements[0] = _make_uint4(0, 1, 2, 3);
    /// printf("%d %d %d %d \n", elements[0].x, elements[0].y, elements[0].z, elements[0].w);
    // elements[0] = Sort(elements[0]);

    // printf("%d %d %d %d \n",elements[0].x,elements[0].y,elements[0].z,elements[0].w);

    //
    //    pn[0] = real3(-1, -1, -1);
    //    pn[1] = real3(1., -1, -1);
    //    pn[2] = real3(1., 1., -1);
    //    pn[3] = real3(-1, 1., -1);
    //
    //    pn[4] = real3(-1, -1, 1.);
    //    pn[5] = real3(1., -1, 1.);
    //    pn[6] = real3(1., 1., 1.);
    //    pn[7] = real3(-1, 1., 1.);
    //
    // std::fill(vn.begin(), vn.end(), real3(0));
    //    std::vector<uint4> elements(4);
    //    elements[0] = _make_uint4(0, 1, 2, 5);
    //    elements[1] = _make_uint4(0, 2, 3, 7);
    //    elements[2] = _make_uint4(0, 4, 5, 7);
    //    elements[3] = _make_uint4(2, 5, 6, 7);
    //    vn[0] = real3(0, 0, -10);
    //    vn[1] = real3(0, 0, -10);
    //    vn[2] = real3(0, 0, -10);
    //    vn[3] = real3(0, 0, -10);
    //    vn[4] = real3(0, 0, 10);
    //    vn[5] = real3(0, 0, 10);
    //    vn[6] = real3(0, 0, 10);
    //    vn[7] = real3(0, 0, 10);
    // fea_container->AddNodes(pn, vn);
    // fea_container->AddElements(elements);

    //    real3 c1, c2, c3;
    //    c1 = pn[elements[0].y] - pn[elements[0].x];
    //    c2 = pn[elements[0].z] - pn[elements[0].x];
    //    c3 = pn[elements[0].w] - pn[elements[0].x];
    //    Mat33 Ds = Mat33(c1, c2, c3);
    //
    //    real det = Determinant(Ds);
    //    if (det < 0) {
    //        Swap(pn[0], pn[1]);
    //        c1 = pn[elements[0].y] - pn[elements[0].x];
    //        c2 = pn[elements[0].z] - pn[elements[0].x];
    //        c3 = pn[elements[0].w] - pn[elements[0].x];
    //        Ds = Mat33(c1, c2, c3);
    //    }

    // Add a rim
    //    ChSharedPtr<ChBody> mwheel_rim(new ChBody);
    //    mwheel_rim->SetMass(80);
    //    mwheel_rim->SetInertiaXX(ChVector<>(60, 60, 60));
    //    mwheel_rim->SetPos(tire_center);
    //    mwheel_rim->SetRot(tire_alignment);
    //    mwheel_rim->SetPos_dt(ChVector<>(0, 0, tire_vel_z0));
    //    mwheel_rim->SetWvel_par(ChVector<>(tire_w0, 0, 0));
    //    my_system.Add(mwheel_rim);

    // ChSharedPtr<ChObjShapeFile> mobjmesh(new ChObjShapeFile);
    // mobjmesh->SetFilename(GetChronoDataFile("fea/tractor_wheel_rim.obj"));
    // mwheel_rim->AddAsset(mobjmesh);

    // Conect rim and tire using constraints.
    // Do these constraints where the 2nd node set has been marked in the .INP file.
    //    int nodeset_index = 1;
    //    for (int i = 0; i < node_sets[nodeset_index].size(); ++i) {
    //        ChSharedPtr<ChLinkPointFrame> mlink(new ChLinkPointFrame);
    //        mlink->Initialize(node_sets[nodeset_index][i].DynamicCastTo<ChNodeFEAxyz>(), mwheel_rim);
    //        my_system.Add(mlink);
    //    }

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
