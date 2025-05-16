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
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGeometry.h"

#include "chrono/fea/ChLinkNodeSlopeFrame.h"
#include "chrono/fea/ChLinkNodeFrame.h"
#include "chrono/fea/ChMaterialBeamANCF.h"
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

enum class ElementType { ANCF_CABLE, ANCF_3243, ANCF_3333, EULER };

ElementType element_type = ElementType::ANCF_CABLE;
int num_elements = 4;
bool use_FEA_node_directions = false;

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
    double t_end = 1.5;
    bool verbose = false;
    bool render = true;
    double render_fps = 400;
    bool snapshots = false;

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
    sph_params.integration_scheme = IntegrationScheme::RK2;
    sph_params.initial_spacing = initial_spacing;
    sph_params.d0_multiplier = 1.0;
    sph_params.shifting_method = ShiftingMethod::PPST_XSPH;
    sph_params.shifting_xsph_eps = 0.25;
    sph_params.shifting_ppst_pull = 1.0;
    sph_params.shifting_ppst_push = 3.0;
    sph_params.kernel_threshold = 0.8;
    sph_params.artificial_viscosity = 0.5;
    sph_params.num_proximity_search_steps = 1;
    sph_params.boundary_method = BoundaryMethod::ADAMI;
    sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_BILATERAL;

    fsi.SetSPHParameters(sph_params);

    // Enable/disable use of node directions for FSI flexible meshes
    fsi.EnableNodeDirections(use_FEA_node_directions);

    // Dimension of computational domain and intial fluid domain
    ChVector3d csize(2.0, 0.15, 1.0);
    ChVector3d fsize(0.6, 0.15, 0.8);

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
    fsi.SetComputationalDomain(ChAABB(cMin, cMax), BC_Y_PERIODIC);

    // Initialize FSI problem
    fsi.Initialize();

    // Create oputput directories
    std::string out_dir = GetChronoOutputPath() + "FSI_FEAdirections";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }
    if (snapshots) {
        if (!filesystem::create_directory(filesystem::path(out_dir + "/snapshots"))) {
            std::cerr << "Error creating directory " << out_dir + "/snapshots" << std::endl;
            return 1;
        }
    }

    // Create a run-time visualizer
    std::shared_ptr<ChVisualSystem> vis;

#ifdef CHRONO_VSG
    if (render) {
        // FSI plugin
        double vel_min = 0.0;
        double vel_max = 2.5;
        auto col_callback = chrono_types::make_shared<ParticleVelocityColorCallback>(vel_min, vel_max);

        auto visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);
        visFSI->EnableFluidMarkers(true);
        visFSI->EnableBoundaryMarkers(false);
        visFSI->EnableRigidBodyMarkers(false);
        visFSI->SetSPHColorCallback(col_callback, ChColormap::Type::KINDLMANN);
        visFSI->SetSPHVisibilityCallback(chrono_types::make_shared<MarkerPositionVisibilityCallback>());

        // VSG visual system (attach visFSI as plugin)
        auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        visVSG->AttachPlugin(visFSI);
        visVSG->AttachSystem(&sysMBS);
        visVSG->SetWindowTitle("Flexible Cable");
        visVSG->SetWindowSize(1280, 800);
        visVSG->SetWindowPosition(100, 100);
        visVSG->AddCamera(ChVector3d(0, -2.0, 0.3), ChVector3d(0, 0, 0.3));
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

    ChTimer timer;
    timer.start();
    while (time < t_end) {
        // Render FSI system
        if (render && time >= render_frame / render_fps) {
            if (!vis->Run())
                break;
            vis->Render();

            if (snapshots) {
                if (verbose)
                    std::cout << " -- Snapshot frame " << render_frame << " at t = " << time << std::endl;
                std::ostringstream filename;
                filename << out_dir << "/snapshots/img_" << std::setw(5) << std::setfill('0') << render_frame + 1
                         << ".png";
                vis->WriteImageToFile(filename.str());
            }

            render_frame++;
        }

        fsi.DoStepDynamics(step_size);

        time += step_size;
        sim_frame++;
    }
    timer.stop();
    cout << "\nSimulation time: " << timer() << " seconds\n" << endl;

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

const double beam_length = 0.6;
const double section_dim = 0.02;

std::shared_ptr<ChMesh> CreateANCFCable(ChSystem& sysMBS, double x, int n) {
    // Material Properties
    double E = 8e7;
    double density = 8000;
    double BeamRayleighDamping = 0.1;

    auto section = chrono_types::make_shared<ChBeamSectionCable>();
    section->SetDiameter(section_dim);
    section->SetYoungModulus(E);
    section->SetDensity(density);
    section->SetRayleighDamping(BeamRayleighDamping);

    auto mesh = chrono_types::make_shared<fea::ChMesh>();

    auto length = beam_length / n;
    auto p = ChVector3d(x, 0.0, 0.005);
    auto dir = ChVector3d(0, 0, 1);

    auto node0 = chrono_types::make_shared<ChNodeFEAxyzD>(p, dir);
    node0->SetFixed(true);
    mesh->AddNode(node0);

    for (int i = 0; i < n; i++) {
        p += length * dir;

        mesh->AddNode(chrono_types::make_shared<ChNodeFEAxyzD>(p, dir));
        auto element = chrono_types::make_shared<ChElementCableANCF>();
        element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(i)),
                          std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(i + 1)));
        element->SetSection(section);
        element->SetRestLength(length);
        mesh->AddElement(element);
    }

    // Add the mesh to the MBS system
    sysMBS.Add(mesh);

    auto vis_mesh = chrono_types::make_shared<ChVisualShapeFEA>();
    vis_mesh->SetFEMdataType(ChVisualShapeFEA::DataType::ELEM_BEAM_MZ);
    vis_mesh->SetColormapRange(-0.4, 0.4);
    vis_mesh->SetSmoothFaces(true);
    vis_mesh->SetWireframe(false);
    mesh->AddVisualShapeFEA(vis_mesh);

    return mesh;
}

std::shared_ptr<ChMesh> CreateANCF3243Beam(ChSystem& sysMBS, double x, int n) {
    auto mesh = chrono_types::make_shared<fea::ChMesh>();

    double E = 8e6;
    double nu = 0.3;
    double density = 1000;
    const double k1 = 10 * (1 + nu) / (12 + 11 * nu);
    const double k2 = k1;  // k1, k2: Timoshenko shear correection coefficients
    auto material = chrono_types::make_shared<ChMaterialBeamANCF>(density, E, nu, E * nu, k1, k2);

    double length = beam_length / n;

    // Create the first node and fix it completely to ground (Cantilever constraint)
    auto nodeA = chrono_types::make_shared<ChNodeFEAxyzDDD>(ChVector3d(x, 0, 0.005), VECT_Z, VECT_X, VECT_Y);
    nodeA->SetFixed(true);
    mesh->AddNode(nodeA);

    for (int i = 1; i <= n; i++) {
        auto nodeB = chrono_types::make_shared<ChNodeFEAxyzDDD>(ChVector3d(x, 0, length * i), VECT_Z, VECT_X, VECT_Y);
        mesh->AddNode(nodeB);

        auto element = chrono_types::make_shared<ChElementBeamANCF_3243>();
        element->SetNodes(nodeA, nodeB);
        element->SetDimensions(length, section_dim, section_dim);
        element->SetMaterial(material);
        element->SetAlphaDamp(0.0);
        mesh->AddElement(element);

        nodeA = nodeB;
    }

    // Add the mesh to the MBS system
    sysMBS.Add(mesh);

    auto vis_mesh1 = chrono_types::make_shared<ChVisualShapeFEA>();
    vis_mesh1->SetFEMdataType(ChVisualShapeFEA::DataType::ELEM_BEAM_MZ);
    vis_mesh1->SetColormapRange(-0.4, 0.4);
    vis_mesh1->SetSmoothFaces(true);
    vis_mesh1->SetWireframe(false);
    mesh->AddVisualShapeFEA(vis_mesh1);

    auto vis_mesh2 = chrono_types::make_shared<ChVisualShapeFEA>();
    vis_mesh2->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_CSYS);
    vis_mesh2->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    vis_mesh2->SetSymbolsThickness(0.006);
    vis_mesh2->SetSymbolsScale(0.01);
    vis_mesh2->SetZbufferHide(false);
    mesh->AddVisualShapeFEA(vis_mesh2);

    return mesh;
}

std::shared_ptr<ChMesh> CreateANCF3333Beam(ChSystem& sysMBS, double x, int n) {
    auto mesh = chrono_types::make_shared<fea::ChMesh>();

    double E = 1e7;
    double nu = 0.3;
    double density = 1000;
    const double k1 = 10 * (1 + nu) / (12 + 11 * nu);
    const double k2 = k1;  // k1, k2: Timoshenko shear correection coefficients
    auto material = chrono_types::make_shared<ChMaterialBeamANCF>(density, E, nu, E * nu, k1, k2);

    double length = beam_length / n;

    // Setup beam cross section gradients to initially aligned with the global x and y directions
    ChVector3d dir1(1, 0, 0);
    ChVector3d dir2(0, 1, 0);

    // Create the first node and fix it completely to ground (Cantilever constraint)
    auto nodeA = chrono_types::make_shared<ChNodeFEAxyzDD>(ChVector3d(x, 0, 0.005), dir1, dir2);
    nodeA->SetFixed(true);
    mesh->AddNode(nodeA);

    for (int i = 1; i <= n; i++) {
        auto nodeB =
            chrono_types::make_shared<ChNodeFEAxyzDD>(ChVector3d(x, 0, 0.5 * length * (2 * i - 0)), dir1, dir2);
        auto nodeC =
            chrono_types::make_shared<ChNodeFEAxyzDD>(ChVector3d(x, 0, 0.5 * length * (2 * i - 1)), dir1, dir2);
        mesh->AddNode(nodeB);
        mesh->AddNode(nodeC);

        auto element = chrono_types::make_shared<ChElementBeamANCF_3333>();
        element->SetNodes(nodeA, nodeB, nodeC);
        element->SetDimensions(length, section_dim, section_dim);
        element->SetMaterial(material);
        element->SetAlphaDamp(0.0);
        mesh->AddElement(element);

        nodeA = nodeB;
    }

    // Add the mesh to the MBS system
    sysMBS.Add(mesh);

    auto vis_mesh1 = chrono_types::make_shared<ChVisualShapeFEA>();
    vis_mesh1->SetFEMdataType(ChVisualShapeFEA::DataType::ELEM_BEAM_MZ);
    vis_mesh1->SetColormapRange(-0.4, 0.4);
    vis_mesh1->SetSmoothFaces(true);
    vis_mesh1->SetWireframe(false);
    mesh->AddVisualShapeFEA(vis_mesh1);

    auto vis_mesh2 = chrono_types::make_shared<ChVisualShapeFEA>();
    vis_mesh2->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_CSYS);
    vis_mesh2->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    vis_mesh2->SetSymbolsThickness(0.006);
    vis_mesh2->SetSymbolsScale(0.01);
    vis_mesh2->SetZbufferHide(false);
    mesh->AddVisualShapeFEA(vis_mesh2);

    return mesh;
}

std::shared_ptr<ChMesh> CreateEulerBeam(ChSystem& sysMBS, double x, int n) {
    auto mesh = chrono_types::make_shared<fea::ChMesh>();
    auto section = chrono_types::make_shared<ChBeamSectionEulerAdvanced>();
    section->SetAsRectangularSection(section_dim, section_dim);
    section->SetYoungModulus(1e7);
    section->SetShearModulus(1e7 * 0.3);
    section->SetRayleighDamping(0.0);

    ChBuilderBeamEuler builder;

    builder.BuildBeam(mesh,                           // containing mesh
                      section,                        // Euler beam section specification
                      n,                              // number of elements
                      ChVector3d(x, 0, 0.005),        // beginning of beam
                      ChVector3d(x, 0, beam_length),  // end of beam
                      ChVector3d(0, 0, 1));           // the up direction of the section for the beam

    builder.GetLastBeamNodes().front()->SetFixed(true);

    // Add the mesh to the MBS system
    sysMBS.Add(mesh);

    auto vis_mesh1 = chrono_types::make_shared<ChVisualShapeFEA>();
    vis_mesh1->SetFEMdataType(ChVisualShapeFEA::DataType::ELEM_BEAM_MZ);
    vis_mesh1->SetColormapRange(-0.4, 0.4);
    vis_mesh1->SetSmoothFaces(true);
    vis_mesh1->SetWireframe(false);
    mesh->AddVisualShapeFEA(vis_mesh1);

    auto vis_mesh2 = chrono_types::make_shared<ChVisualShapeFEA>();
    vis_mesh2->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_CSYS);
    vis_mesh2->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    vis_mesh2->SetSymbolsThickness(0.006);
    vis_mesh2->SetSymbolsScale(0.01);
    vis_mesh2->SetZbufferHide(false);
    mesh->AddVisualShapeFEA(vis_mesh2);

    return mesh;
}

void CreateCables(ChFsiProblemSPH& fsi, std::shared_ptr<ChBody> ground) {
    ChSystem& sysMBS = fsi.GetMultibodySystem();

    std::shared_ptr<fea::ChMesh> mesh;
    switch (element_type) {
        case ElementType::ANCF_CABLE:
            mesh = CreateANCFCable(sysMBS, 0, num_elements);
            break;
        case ElementType::ANCF_3243:
            mesh = CreateANCF3243Beam(sysMBS, 0, num_elements);
            break;
        case ElementType::ANCF_3333:
            mesh = CreateANCF3333Beam(sysMBS, 0, num_elements);
            break;
        case ElementType::EULER:
            mesh = CreateEulerBeam(sysMBS, 0, num_elements);
            break;
    }

    fsi.SetBcePattern1D(BcePatternMesh1D::STAR, false);
    fsi.AddFeaMesh(mesh, false);
}
