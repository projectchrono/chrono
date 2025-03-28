// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Radu Serban
// =============================================================================

#include <cassert>
#include <cstdlib>
#include <ctime>
#include <iomanip>

#include "chrono/core/ChRotation.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/assets/ChVisualSystem.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"

#include "chrono/fea/ChLinkNodeSlopeFrame.h"
#include "chrono/fea/ChLinkNodeFrame.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChMeshExporter.h"
#include "chrono/fea/ChBuilderBeam.h"

#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

#include "chrono_fsi/sph/ChFsiProblemSPH.h"

#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationVSG.h"
#endif

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

using std::cout;
using std::cerr;
using std::endl;

// -----------------------------------------------------------------------------

bool use_FEA_node_directions = true;

// -----------------------------------------------------------------------------

void CreateContainer(ChFsiProblemSPH& fsi, std::shared_ptr<ChBody> ground, const ChVector3d& csize);
void CreateCables(ChFsiProblemSPH& fsi, std::shared_ptr<ChBody> ground);

// -----------------------------------------------------------------------------

#ifdef CHRONO_VSG
class MarkerPositionVisibilityCallback : public ChFsiVisualizationVSG::MarkerVisibilityCallback {
  public:
    MarkerPositionVisibilityCallback() {}
    virtual bool get(unsigned int n) const override { return pos[n].y > 0; }
};
#endif

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    // Parse command line arguments
    double t_end = 10.0;
    bool verbose = true;
    bool render = true;
    double render_fps = 400;

    // Create the Chrono system and associated collision system
    ChSystemSMC sysMBS;
    sysMBS.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create the FSI problem
    double initial_spacing = 0.01;

    ChFsiProblemCartesian fsi(sysMBS, initial_spacing);
    fsi.SetVerbose(verbose);
    ChFsiSystemSPH& sysFSI = fsi.GetSystemFSI();

    // Set gravitational acceleration
    const ChVector3d gravity(0, 0, -9.81);
    fsi.SetGravitationalAcceleration(gravity);

    // Set integration step size
    double step_size = 2.5e-4;
    fsi.SetStepSizeCFD(step_size);
    fsi.SetStepsizeMBD(step_size);

    // Set fluid phase properties
    ChFsiFluidSystemSPH::ElasticMaterialProperties mat_props;
    mat_props.density = 1700;
    mat_props.Young_modulus = 1e6;
    mat_props.Poisson_ratio = 0.3;
    mat_props.mu_I0 = 0.03;
    mat_props.mu_fric_s = 0.5;
    mat_props.mu_fric_2 = 0.5;
    mat_props.average_diam = 0.005;
    mat_props.cohesion_coeff = 0;

    fsi.SetElasticSPH(mat_props);

    // Set SPH solution parameters
    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.sph_method = SPHMethod::WCSPH;
    sph_params.initial_spacing = initial_spacing;
    sph_params.d0_multiplier = 1.0;
    sph_params.shifting_method = ShiftingMethod::PPST_XSPH;
    sph_params.shifting_xsph_eps = 0.25;
    sph_params.shifting_ppst_pull = 1.0;
    sph_params.shifting_ppst_push = 3.0;
    sph_params.kernel_threshold = 0.8;
    sph_params.artificial_viscosity = 0.5;
    sph_params.num_proximity_search_steps = 1;
    sph_params.boundary_type = BoundaryType::ADAMI;
    sph_params.viscosity_type = ViscosityType::ARTIFICIAL_BILATERAL;

    fsi.SetSPHParameters(sph_params);

    // Enable/disable use of node directions for FSI flexible meshes
    fsi.EnableNodeDirections(use_FEA_node_directions);

    // Dimension of computational domain and intial fluid domain
    ChVector3d csize(3.0, 0.2, 2.0);
    ChVector3d fsize(1.0, 0.2, 1.0);

    // Create FSI solid phase
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    ground->EnableCollision(true);
    sysMBS.AddBody(ground);
    CreateContainer(fsi, ground, csize);
    CreateCables(fsi, ground);

    // Enable depth-based initial pressure for SPH particles
    fsi.RegisterParticlePropertiesCallback(chrono_types::make_shared<DepthPressurePropertiesCallback>(fsize.z()));

    // Create SPH material (do not create any boundary BCEs)
    fsi.Construct({fsize.x(), fsize.y(), fsize.z()},       // box dimensions
                  {-csize.x() / 2 + fsize.x() / 2, 0, 0},  // reference location
                  BoxSide::NONE                            // no boundary BCEs
    );

    // Create container (with bottom and left/right boundaries)
    fsi.AddBoxContainer({csize.x(), csize.y(), csize.z()},                // length x width x height
                        ChVector3d(0, 0, 0),                              // reference location
                        BoxSide::Z_NEG | BoxSide::X_NEG | BoxSide::X_POS  // bottom and left/right walls
    );

    // Explicitly set computational domain (necessary if no side walls)
    ChVector3d cMin = ChVector3d(-5 * csize.x(), -csize.y() / 2 - initial_spacing / 2, -5 * csize.z());
    ChVector3d cMax = ChVector3d(+5 * csize.x(), +csize.y() / 2 + initial_spacing / 2, +5 * csize.z());
    fsi.SetComputationalDomain(ChAABB(cMin, cMax), PeriodicSide::Y);

    // Initialize FSI problem
    fsi.Initialize();

    // Create oputput directories
    std::string out_dir = GetChronoOutputPath() + "FSI_FEAdirections/";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }

    // Create a run-tme visualizer
    std::shared_ptr<ChVisualSystem> vis;

#ifdef CHRONO_VSG
    if (render) {
        // FSI plugin
        auto col_callback = chrono_types::make_shared<ParticleVelocityColorCallback>(0, 2.5);

        auto visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);
        visFSI->EnableFluidMarkers(true);
        visFSI->EnableBoundaryMarkers(false);
        visFSI->EnableRigidBodyMarkers(false);
        visFSI->SetSPHColorCallback(col_callback);
        visFSI->SetSPHVisibilityCallback(chrono_types::make_shared<MarkerPositionVisibilityCallback>());

        // VSG visual system (attach visFSI as plugin)
        auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        visVSG->AttachPlugin(visFSI);
        visVSG->AttachSystem(&sysMBS);
        visVSG->SetWindowTitle("Flexible Cable");
        visVSG->SetWindowSize(1280, 800);
        visVSG->SetWindowPosition(100, 100);
        visVSG->AddCamera(ChVector3d(1.5, -1.5, 0.5), ChVector3d(0, 0, 0));
        visVSG->SetLightIntensity(0.9f);
        visVSG->SetLightDirection(-CH_PI_2, CH_PI / 6);

        visVSG->Initialize();
        vis = visVSG;
    }
#else
    render = false;
#endif

// Set MBS solver
#ifdef CHRONO_PARDISO_MKL
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    mkl_solver->LockSparsityPattern(true);
    sysMBS.SetSolver(mkl_solver);
#else
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    sysMBS.SetSolver(solver);
    solver->SetMaxIterations(2000);
    solver->SetTolerance(1e-12);
    solver->EnableDiagonalPreconditioner(true);
    solver->SetVerbose(false);
#endif

    // Simulation loop
    double time = 0.0;
    int sim_frame = 0;
    int render_frame = 0;

    std::string out_file = out_dir + "/results.txt";
    std::ofstream ofile(out_file, std::ios::trunc);

    ChTimer timer;
    timer.start();
    while (time < t_end) {
        // Render FSI system
        if (render && time >= render_frame / render_fps) {
            if (!vis->Run())
                break;
            vis->Render();

            render_frame++;
        }

        fsi.DoStepDynamics(step_size);

        time += step_size;
        sim_frame++;
    }
    timer.stop();
    cout << "\nSimulation time: " << timer() << " seconds\n" << endl;

    ofile.close();

    return 0;
}

// -----------------------------------------------------------------------------
// Create the solid objects in the MBD system and their counterparts in the FSI system

void CreateContainer(ChFsiProblemSPH& fsi, std::shared_ptr<ChBody> ground, const ChVector3d& csize) {
    ChSystem& sysMBS = fsi.GetMultibodySystem();

    // Contact material (default properties)
    auto contact_material_info = ChContactMaterialData();
    contact_material_info.mu = 0.1f;
    auto contact_material = contact_material_info.CreateMaterial(sysMBS.GetContactMethod());

    // Add collision geometry
    utils::AddBoxGeometry(ground.get(), contact_material, ChVector3d(csize.x(), csize.y(), 0.1),
                          ChVector3d(0, 0, -0.05));
    utils::AddBoxGeometry(ground.get(), contact_material, ChVector3d(0.1, csize.y(), csize.z() + 0.2),
                          ChVector3d(+csize.x() / 2 + 0.05, 0, csize.z() / 2));
    utils::AddBoxGeometry(ground.get(), contact_material, ChVector3d(0.1, csize.y(), csize.z() + 0.2),
                          ChVector3d(-csize.x() / 2 - 0.05, 0, csize.z() / 2));
    utils::AddBoxGeometry(ground.get(), contact_material, ChVector3d(csize.x() + 0.2, 0.1, csize.z() + 0.2),
                          ChVector3d(0, +csize.y() / 2 + 0.05, csize.z() / 2), QUNIT, false);
    utils::AddBoxGeometry(ground.get(), contact_material, ChVector3d(csize.x() + 0.2, 0.1, csize.z() + 0.2),
                          ChVector3d(0, -csize.y() / 2 - 0.05, csize.z() / 2), QUNIT, false);
}

std::shared_ptr<ChMesh> CreateFlexibleCable1(ChSystem& sysMBS, double loc_x, std::shared_ptr<ChBody> ground) {
    // Material Properties
    double E = 5e9;
    double density = 8000;
    double BeamRayleighDamping = 0.02;

    auto section = chrono_types::make_shared<ChBeamSectionCable>();
    section->SetDiameter(0.02);
    section->SetYoungModulus(E);
    section->SetDensity(density);
    section->SetRayleighDamping(BeamRayleighDamping);

    auto mesh = chrono_types::make_shared<fea::ChMesh>();

    int num_elements = 5;
    auto length = 0.15;
    auto dq = QuatFromAngleY(-CH_PI_4 / (num_elements - 1));
    auto p = ChVector3d(loc_x, 0.0, 0.02);
    auto dir = ChVector3d(0, 0, 1);

    mesh->AddNode(chrono_types::make_shared<ChNodeFEAxyzD>(p, dir));

    for (int i = 0; i < num_elements; i++) {
        p += length * dir;

        ////cout << "i = " << i << endl;
        ////cout << "  dir: " << dir << endl;
        ////cout << "  p: " << p << endl;

        mesh->AddNode(chrono_types::make_shared<ChNodeFEAxyzD>(p, dir));
        auto element = chrono_types::make_shared<ChElementCableANCF>();
        element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(i)),
                          std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(i + 1)));
        element->SetSection(section);
        element->SetRestLength(length);
        mesh->AddElement(element);
        dir = dq.Rotate(dir).GetNormalized();
    }

    auto node0 = std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(0));
    auto pos_const = chrono_types::make_shared<ChLinkNodeFrame>();
    auto dir_const = chrono_types::make_shared<ChLinkNodeSlopeFrame>();
    pos_const->Initialize(node0, ground);
    dir_const->Initialize(node0, ground);
    dir_const->SetDirectionInAbsoluteCoords(node0->GetSlope1());
    sysMBS.Add(pos_const);
    sysMBS.Add(dir_const);

    // Add the mesh to the MBS system
    sysMBS.Add(mesh);

    auto vis_mesh = chrono_types::make_shared<ChVisualShapeFEA>();
    vis_mesh->SetFEMdataType(ChVisualShapeFEA::DataType::ELEM_BEAM_MZ);
    vis_mesh->SetColorscaleMinMax(-0.4, 0.4);
    vis_mesh->SetSmoothFaces(true);
    vis_mesh->SetWireframe(false);
    mesh->AddVisualShapeFEA(vis_mesh);

    return mesh;
}

std::shared_ptr<ChMesh> CreateFlexibleCable2(ChSystem& sysMBS, double loc_x, std::shared_ptr<ChBody> ground) {
    // Material Properties
    double E = 5e8;
    double density = 8000;
    double BeamRayleighDamping = 0.02;

    auto section = chrono_types::make_shared<ChBeamSectionCable>();
    section->SetDiameter(0.02);
    section->SetYoungModulus(E);
    section->SetDensity(density);
    section->SetRayleighDamping(BeamRayleighDamping);

    auto mesh = chrono_types::make_shared<fea::ChMesh>();

    auto length = 0.4;

    auto p0 = ChVector3d(loc_x, 0.0, 0.02);
    auto p1 = p0 + ChVector3d(0, 0, length);
    auto p2 = p1 + ChVector3d(length, 0, 0);

    auto d0 = ChVector3d(0, 0, 1);
    auto d1 = ChVector3d(CH_SQRT_2 / 2, 0, CH_SQRT_2 / 2);
    auto d2 = ChVector3d(1, 0, 0);

    auto node0 = chrono_types::make_shared<ChNodeFEAxyzD>(p0, d0);
    auto node1 = chrono_types::make_shared<ChNodeFEAxyzD>(p1, d1);
    auto node2 = chrono_types::make_shared<ChNodeFEAxyzD>(p2, d2);
    mesh->AddNode(node0);
    mesh->AddNode(node1);
    mesh->AddNode(node2);

    {
        auto element = chrono_types::make_shared<ChElementCableANCF>();
        element->SetNodes(node0, node1);
        element->SetSection(section);
        element->SetRestLength(length);
        mesh->AddElement(element);
    }

    {
        auto element = chrono_types::make_shared<ChElementCableANCF>();
        element->SetNodes(node1, node2);
        element->SetSection(section);
        element->SetRestLength(length);
        mesh->AddElement(element);
    }

    auto pos_const = chrono_types::make_shared<ChLinkNodeFrame>();
    auto dir_const = chrono_types::make_shared<ChLinkNodeSlopeFrame>();
    pos_const->Initialize(node0, ground);
    dir_const->Initialize(node0, ground);
    dir_const->SetDirectionInAbsoluteCoords(node0->GetSlope1());
    sysMBS.Add(pos_const);
    sysMBS.Add(dir_const);

    sysMBS.Add(mesh);

    auto vis_mesh = chrono_types::make_shared<ChVisualShapeFEA>();
    vis_mesh->SetFEMdataType(ChVisualShapeFEA::DataType::ELEM_BEAM_MZ);
    vis_mesh->SetColorscaleMinMax(-0.4, 0.4);
    vis_mesh->SetSmoothFaces(true);
    vis_mesh->SetWireframe(false);
    mesh->AddVisualShapeFEA(vis_mesh);

    return mesh;
}

void CreateCables(ChFsiProblemSPH& fsi, std::shared_ptr<ChBody> ground) {
    ChSystem& sysMBS = fsi.GetMultibodySystem();

    // Downstream locations
    double cable1_x = -0.3;
    double cable2_x = +0.8;

    fsi.SetBcePattern1D(BcePatternMesh1D::STAR, false);

    // Create the first flexible cable and add to FSI system
    auto mesh1 = CreateFlexibleCable1(sysMBS, cable1_x, ground);
    fsi.AddFeaMesh(mesh1, false);

    // Create the second flexible cable
    auto mesh2 = CreateFlexibleCable2(sysMBS, cable2_x, ground);
    fsi.AddFeaMesh(mesh2, false);
}
