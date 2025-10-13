// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Test of a stand-alone wheel with FEA tire
//
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChDeformableTire.h"

#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"
#ifdef CHRONO_FSI
    #include "chrono_vehicle/terrain/CRMTerrain.h"
using namespace chrono::fsi;
using namespace chrono::fsi::sph;
#endif

#include "chrono/assets/ChVisualSystem.h"
#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif

#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
    #ifdef CHRONO_FSI
        #include "chrono_fsi/sph/visualization/ChSphVisualizationVSG.h"
    #endif
using namespace chrono::vsg3d;
#endif

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
    #include "chrono_postprocess/ChBlender.h"
using namespace chrono::postprocess;
#endif

#include "chrono_thirdparty/filesystem/path.h"

#include "demos/SetChronoSolver.h"

using namespace chrono;
using namespace chrono::vehicle;

using std::cout;
using std::cerr;
using std::endl;

// -----------------------------------------------------------------------------

// Run-time visualization system (Irrlicht or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Tire JSON specification file
std::string tire_json = "Polaris/Polaris_ANCF4Tire_Lumped.json";
////std::string tire_json = "hmmwv/tire/HMMWV_ANCF4Tire.json";

// Mass load on wheel/spindle (kg)
double load_mass = 200;

// Internal tire pressure (N/m2)
// Manufacturer recommended values:
//   HMMWV:   37 psi (255 kPa)
//   Polaris: 16 psi (110 kPa)
double tire_pressure = 110e3;

bool fix_wheel = false;
bool enable_tire_pressure = true;

// Terrain type
enum TerrainType { RIGID, SCM, CRM };
TerrainType terrain_type = TerrainType::RIGID;

// Output plots
bool timing_plots = false;
bool deflection_plots = true;

// Simulation time
double t_end = 0.5;

// -----------------------------------------------------------------------------

class DummyWheel : public ChWheel {
  public:
    DummyWheel() : ChWheel("tire_wheel"), m_inertia(ChVector3d(0)) {}
    virtual double GetWheelMass() const override { return 0; }
    virtual const ChVector3d& GetWheelInertia() const override { return m_inertia; }
    virtual double GetRadius() const override { return 1; }
    virtual double GetWidth() const override { return 1; }

  private:
    ChVector3d m_inertia;
};

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    cout << "Copyright (c) 2024 projectchrono.org\nChrono version: " << CHRONO_VERSION << endl;

    // Create Chrono system
    ChSystemSMC sys;
    ChVector3d gacc(0, 0, -9.81);
    sys.SetGravitationalAcceleration(gacc);
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Solver and integrator settings
    double step_size = 1e-4;
    auto solver_type = ChSolver::Type::PARDISO_MKL;
    auto integrator_type = ChTimestepper::Type::HHT;
    int num_threads_chrono = std::min(8, ChOMP::GetNumProcs());
    int num_threads_collision = 1;
    int num_threads_eigen = 1;
    int num_threads_pardiso = std::min(8, ChOMP::GetNumProcs());
    SetChronoSolver(sys, solver_type, integrator_type, num_threads_pardiso);
    sys.SetNumThreads(num_threads_chrono, num_threads_collision, num_threads_eigen);

    auto ls_direct = std::dynamic_pointer_cast<ChDirectSolverLS>(sys.GetSolver());
    auto ls_iterative = std::dynamic_pointer_cast<ChIterativeSolverLS>(sys.GetSolver());
    auto hht = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());

    if (hht) {
        hht->SetAlpha(-0.2);
        hht->SetMaxIters(5);
        hht->SetAbsTolerances(1e-2);
        hht->SetJacobianUpdateMethod(ChTimestepperImplicit::JacobianUpdate::EVERY_STEP);
        hht->SetStepControl(false);
        hht->SetMinStepSize(1e-4);
        ////hht->SetVerbose(true);
    }

    // Create the spindle body (at origin)
    auto spindle = chrono_types::make_shared<ChSpindle>();
    spindle->SetPos(VNULL);
    spindle->SetMass(load_mass);
    spindle->SetFixed(fix_wheel);
    sys.AddBody(spindle);

    // Create the wheel subsystem, arbitrarily assuming LEFT side
    auto wheel = chrono_types::make_shared<DummyWheel>();
    wheel->Initialize(nullptr, spindle, LEFT);
    wheel->SetVisualizationType(VisualizationType::NONE);

    // Create the tire
    auto tire = ReadTireJSON(GetVehicleDataFile(tire_json));
    auto tire_def = std::dynamic_pointer_cast<ChDeformableTire>(tire);
    if (!tire_def) {
        cerr << "ERROR: Incorrect tire specification JSON file" << endl;
        return 1;
    }
    tire_def->SetPressure(tire_pressure);
    tire_def->EnablePressure(enable_tire_pressure);
    tire_def->EnableContact(true);
    tire_def->EnableRimConnection(true);
    tire_def->SetContactSurfaceType(ChTire::ContactSurfaceType::TRIANGLE_MESH);

    // Initialize tire and associate with wheel
    wheel->SetTire(tire);
    tire->Initialize(wheel);
    tire->SetVisualizationType(VisualizationType::MESH);

    // Extract tire information
    auto mesh = tire_def->GetMesh();
    auto radius = tire_def->GetRadius();
    cout << "Tire input file: " << tire_json << endl;
    cout << "Tire radius:     " << radius << endl;
    cout << "Tire mass:       " << tire_def->GetTireMass() << endl;
    cout << "Tire pressure:   " << tire_pressure << endl;
    cout << "Mass load:       " << load_mass << endl;

    // If wheel not fixed and if tire contact enabled,
    // - connect spindle to ground with a vertical prismatic joint
    // - create terrain (below tire)

    auto floor = chrono_types::make_shared<ChBody>();
    floor->SetPos(VNULL);
    floor->SetFixed(true);
    floor->EnableCollision(true);
    sys.AddBody(floor);

    auto joint = chrono_types::make_shared<ChLinkLockPrismatic>();
    joint->Initialize(floor, spindle, ChFramed());
    sys.AddLink(joint);

#ifndef CHRONO_FSI
    if (terrain_type == TerrainType::CRM) {
        cout << "CRM terrain not available (Chrono::FSI not enabled)." << endl;
        cout << "Revert to rigid terrain." << endl;
        terrain_type = TerrainType::RIGID;
    }
#endif

    std::shared_ptr<ChTerrain> terrain;
    switch (terrain_type) {
        case TerrainType::RIGID: {
            ChContactMaterialData mat_data;
            mat_data.mu = 0.9f;
            mat_data.cr = 0.01f;
            mat_data.Y = 2e7f;
            auto mat = mat_data.CreateMaterial(ChContactMethod::SMC);

            auto terrain_rigid = chrono_types::make_shared<RigidTerrain>(&sys);
            auto patch = terrain_rigid->AddPatch(mat, ChCoordsys<>(ChVector3d(0, 0, -radius - 0.01), QUNIT), 4 * radius,
                                                 4 * radius);
            patch->SetTexture(GetVehicleDataFile("terrain/textures/concrete.jpg"), 10, 5);
            terrain_rigid->Initialize();

            terrain = terrain_rigid;
            break;
        }
        case TerrainType::SCM: {
            auto terrain_scm = chrono_types::make_shared<SCMTerrain>(&sys);
            terrain_scm->SetReferenceFrame(ChCoordsys<>(ChVector3d(0, 0, -radius - 0.01), QUNIT));
            terrain_scm->Initialize(4 * radius, 4 * radius, 0.05);
            terrain_scm->SetSoilParameters(0.2e6,  // Bekker Kphi
                                           0,      // Bekker Kc
                                           1.1,    // Bekker n exponent
                                           0,      // Mohr cohesive limit (Pa)
                                           30,     // Mohr friction limit (degrees)
                                           0.01,   // Janosi shear coefficient (m)
                                           4e7,    // Elastic stiffness (Pa/m), before plastic yield, must be > Kphi
                                           3e4  // Damping (Pa s/m), proportional to negative vertical speed (optional)
            );
            terrain_scm->SetPlotType(vehicle::SCMTerrain::PLOT_PRESSURE, 0, 30000.2);
            terrain_scm->SetMeshWireframe(true);

            terrain = terrain_scm;
            break;
        }
        case TerrainType::CRM: {
#ifdef CHRONO_FSI
            double spacing = 0.04;
            auto terrain_crm = chrono_types::make_shared<CRMTerrain>(sys, spacing);
            terrain_crm->SetGravitationalAcceleration(gacc);
            terrain_crm->SetStepSizeCFD(step_size);

            // Soft soil
            double density = 1600;
            double cohesion = 500;
            double friction = 0.7;

            // Hard soil
            ////double density = 1800;
            ////double cohesion = 1000;
            ////double friction = 0.9;

            ChFsiFluidSystemSPH::ElasticMaterialProperties mat_props;
            mat_props.density = density;
            mat_props.Young_modulus = 1e6;
            mat_props.Poisson_ratio = 0.3;
            mat_props.mu_I0 = 0.04;
            mat_props.mu_fric_s = friction;
            mat_props.mu_fric_2 = friction;
            mat_props.average_diam = 0.005;
            mat_props.cohesion_coeff = cohesion;
            terrain_crm->SetElasticSPH(mat_props);

            // Set SPH solver parameters
            ChFsiFluidSystemSPH::SPHParameters sph_params;
            sph_params.integration_scheme = IntegrationScheme::RK2;
            sph_params.initial_spacing = spacing;
            sph_params.shifting_method = ShiftingMethod::PPST_XSPH;
            sph_params.d0_multiplier = 1;
            sph_params.free_surface_threshold = 0.8;
            sph_params.artificial_viscosity = 0.5;
            sph_params.use_consistent_gradient_discretization = false;
            sph_params.use_consistent_laplacian_discretization = false;
            sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_BILATERAL;
            sph_params.boundary_method = BoundaryMethod::ADAMI;
            terrain_crm->SetSPHParameters(sph_params);

            terrain_crm->SetBcePattern2D(BcePatternMesh2D::INWARD, false);
            terrain_crm->AddFeaMesh(tire_def->GetMesh(), false);

            terrain_crm->Construct({4 * radius, 4 * radius, 0.25},            // length X width X height
                                   ChVector3d(0, 0, -radius - 0.25 - 0.075),  // patch center
                                   BoxSide::ALL & ~BoxSide::Z_POS             // all boundaries, except top
            );

            terrain_crm->Initialize();

            terrain = terrain_crm;
#endif
            break;
        }
    }

    // Create the visualization window (only for the first tire)
    std::shared_ptr<ChVisualSystem> vis;
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
            vis_irr->AttachSystem(&sys);
            vis_irr->SetWindowSize(800, 600);
            vis_irr->SetWindowTitle("FEA tire");
            vis_irr->SetCameraVertical(CameraVerticalDir::Z);
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddTypicalLights();
            vis_irr->AddCamera(ChVector3d(0, -1.5, 0), VNULL);
            vis_irr->EnableContactDrawing(ContactsDrawMode::CONTACT_NORMALS);

            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
            vis_vsg->AttachSystem(&sys);

    #ifdef CHRONO_FSI
            if (terrain_type == TerrainType::CRM) {
                auto sysFSI = std::static_pointer_cast<CRMTerrain>(terrain)->GetFsiSystemSPH();
                auto visFSI = chrono_types::make_shared<ChSphVisualizationVSG>(sysFSI.get());
                visFSI->EnableFluidMarkers(true);
                visFSI->EnableBoundaryMarkers(false);
                visFSI->EnableRigidBodyMarkers(false);

                vis_vsg->AttachPlugin(visFSI);
            }
    #endif

            vis_vsg->SetWindowTitle("FEA tire");
            vis_vsg->AddCamera(ChVector3d(0, -1.5, 0), VNULL);
            vis_vsg->SetWindowSize(1280, 800);
            vis_vsg->SetBackgroundColor(ChColor(0.8f, 0.85f, 0.9f));
            vis_vsg->EnableSkyBox();
            vis_vsg->SetCameraVertical(CameraVerticalDir::Z);
            vis_vsg->SetCameraAngleDeg(40.0);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            vis_vsg->EnableShadows();
            vis_vsg->SetContactNormalsVisibility(true, -1);
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    // Set up output
    std::string out_dir = GetChronoOutputPath() + "FEAtire_TEST";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }
    utils::ChWriterCSV csv;
    csv.SetDelimiter(" ");

    // Simulation loop

    double time = 0;
    while (time < t_end && vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        sys.ResetTimers();
        mesh->ResetTimers();
        mesh->ResetCounters();
        if (ls_direct)
            ls_direct->ResetTimers();

        terrain->Advance(step_size);
        if (terrain_type != TerrainType::CRM)
            sys.DoStepDynamics(step_size);

        ////std::cout << time << " ----------------------" << std::endl;
        ////for (const auto& node : mesh->GetNodes()) {
        ////    auto n = std::static_pointer_cast<fea::ChNodeFEAxyz>(node);
        ////    auto f = n->GetForce();
        ////    if (f.Length() > 1e-6)
        ////        std::cout << "..." << f << std::endl;
        ////}

        time += step_size;

        csv << time << sys.GetRTF();
        csv << sys.GetTimerStep() << sys.GetTimerAdvance() << sys.GetTimerUpdate();
        csv << sys.GetTimerJacobian() << sys.GetTimerLSsetup() << sys.GetTimerLSsolve();
        csv << mesh->GetTimeInternalForces() << mesh->GetTimeJacobianLoad();
        if (ls_direct) {
            csv << ls_direct->GetTimeSetup_Assembly() << ls_direct->GetTimeSetup_SolverCall()
                << ls_direct->GetTimeSolve_Assembly() << ls_direct->GetTimeSolve_SolverCall();
        }
        double spindle_pos = spindle->GetPos().z();
        double spindle_diff = radius - spindle_pos;
        double tire_deflection = spindle_pos > 0 ? spindle_diff : 0;
        csv << spindle_pos << 1000 * tire_deflection;
        csv << endl;
    }

    std::string out_file = out_dir + "/timing.out";
    csv.WriteToFile(out_file);

#ifdef CHRONO_POSTPROCESS
    if (deflection_plots) {
        {
            postprocess::ChGnuPlot gplot(out_dir + "/spindle_height.gpl");
            gplot.SetGrid();
            gplot.SetLabelX("time (s)");
            gplot.SetLabelY("height (m)");
            gplot.SetTitle("Spindle Height");
            gplot.Plot(out_file, 1, 15, "height", " with lines lt 1 lw 2");
        }
        {
            postprocess::ChGnuPlot gplot(out_dir + "/tire_deflection.gpl");
            gplot.SetGrid();
            gplot.SetLabelX("time (s)");
            gplot.SetLabelY("deflection (mm)");
            gplot.SetTitle("Tire deflection");
            gplot.Plot(out_file, 1, 16, "deflection", " with lines lt 1 lw 2");
        }
    }

    if (timing_plots) {
        {
            postprocess::ChGnuPlot gplot(out_dir + "/rtf.gpl");
            gplot.SetGrid();
            gplot.SetLabelX("time (s)");
            gplot.SetLabelY("RTF");
            gplot.SetTitle("Real Time Factor");
            gplot.Plot(out_file, 1, 2, "rtf", " with lines lt 1 lw 2");
        }
        {
            postprocess::ChGnuPlot gplot(out_dir + "/step.gpl");
            gplot.SetGrid();
            gplot.SetLabelX("time (s)");
            gplot.SetLabelY("Timer");
            gplot.SetTitle("Simulation step timers");
            gplot.SetCommand("set logscale y 10");
            gplot.Plot(out_file, 1, 3, "step", " with lines lt 1 lw 2");
            gplot.Plot(out_file, 1, 4, "advance", " with lines lt 2 lw 2");
            gplot.Plot(out_file, 1, 5, "update", " with lines lt 3 lw 2");
            gplot.Plot(out_file, 1, 6, "jacobian", " with lines lt 4 lw 2");
            gplot.Plot(out_file, 1, 7, "LS setup", " with lines lt 5 lw 2");
            gplot.Plot(out_file, 1, 8, "LS solve", " with lines lt 6 lw 2");
        }
        if (ls_direct) {
            postprocess::ChGnuPlot gplot(out_dir + "/ls_direct.gpl");
            gplot.SetGrid();
            gplot.SetLabelX("time (s)");
            gplot.SetLabelY("Timer");
            gplot.SetTitle("Direct linear solver timers");
            gplot.SetCommand("set logscale y 10");
            gplot.Plot(out_file, 1, 11, "setup assembly", " with lines lt 1 lw 2");
            gplot.Plot(out_file, 1, 12, "setup call", " with lines lt 2 lw 2");
            gplot.Plot(out_file, 1, 13, "solve assembly", " with lines lt 3 lw 2");
            gplot.Plot(out_file, 1, 14, "solve call", " with lines lt 4 lw 2");
        }
        {
            postprocess::ChGnuPlot gplot(out_dir + "/fea.gpl");
            gplot.SetGrid();
            gplot.SetLabelX("time (s)");
            gplot.SetLabelY("Timer");
            gplot.SetTitle("FEA timers");
            gplot.SetCommand("set logscale y 10");
            gplot.Plot(out_file, 1, 9, "internal forces", " with lines lt 1 lw 2");
            gplot.Plot(out_file, 1, 10, "jacobian load", " with lines lt 2 lw 2");
        }
    }
#endif

    return 0;
}
