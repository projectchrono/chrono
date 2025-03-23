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

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChDeformableTire.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono/assets/ChVisualSystem.h"
#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
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

// Mass of the wheel/spindle
double wheel_mass = 18;

bool fix_wheel = false;
bool tire_pressure = true;
bool tire_contact = true;

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
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Solver and integrator settings
    double step_size = 1e-4;
    auto solver_type = ChSolver::Type::PARDISO_MKL;
    auto integrator_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;
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
        hht->SetStepControl(false);
        hht->SetMinStepSize(1e-4);
        ////hht->SetVerbose(true);
    }

    // Create the spindle body (at origin)
    auto spindle = chrono_types::make_shared<ChBody>();
    spindle->SetPos(VNULL);
    spindle->SetMass(wheel_mass);
    spindle->SetFixed(fix_wheel);
    sys.AddBody(spindle);

    // Create the wheel subsystem, arbitrarily assuming LEFT side
    auto wheel = chrono_types::make_shared<DummyWheel>();
    wheel->Initialize(nullptr, spindle, LEFT);
    wheel->SetVisualizationType(VisualizationType::NONE);

    // Create the tire
    auto tire = ReadTireJSON(vehicle::GetDataFile(tire_json));
    auto tire_def = std::dynamic_pointer_cast<ChDeformableTire>(tire);
    if (!tire_def) {
        cerr << "ERROR: Incorrect tire specification JSON file" << endl;
        return 1;
    }
    tire_def->EnablePressure(tire_pressure);
    tire_def->EnableContact(tire_contact);
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
    cout << "Wheel mass:      " << wheel_mass << endl;
 
    // If wheel not fixed and if tire contact enabled,
    // - create rigid floor (below tire)
    // - connect spindle to ground with a vertical prismatic joint
    if (!fix_wheel && tire_contact) {
        ChContactMaterialData mat_data;
        mat_data.mu = 0.9f;
        auto floor = chrono_types::make_shared<ChBody>();
        floor->SetPos(VNULL);
        floor->SetFixed(true);
        floor->EnableCollision(true);
        utils::AddBoxGeometry(floor.get(), mat_data.CreateMaterial(ChContactMethod::SMC),
                              ChVector3d(4 * radius, 4 * radius, 0.1), ChVector3d(0, 0, -radius - 0.1));
        sys.AddBody(floor);

        auto joint = chrono_types::make_shared<ChLinkLockPrismatic>();
        joint->Initialize(floor, spindle, ChFramed());
        sys.AddLink(joint);
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

            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
            vis_vsg->AttachSystem(&sys);
            vis_vsg->SetWindowTitle("FEA tire");
            vis_vsg->AddCamera(ChVector3d(0, -1.5, 0), VNULL);
            vis_vsg->SetWindowSize(ChVector2i(800, 600));
            vis_vsg->SetClearColor(ChColor(0.8f, 0.85f, 0.9f));
            vis_vsg->EnableSkyBox();
            vis_vsg->SetCameraVertical(CameraVerticalDir::Z);
            vis_vsg->SetCameraAngleDeg(40.0);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            vis_vsg->EnableShadows();
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

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        sys.ResetTimers();
        mesh->ResetTimers();
        mesh->ResetCounters();
        if (ls_direct)
            ls_direct->ResetTimers();

        sys.DoStepDynamics(step_size);

        csv << sys.GetChTime() << sys.GetRTF();
        csv << sys.GetTimerStep() << sys.GetTimerAdvance() << sys.GetTimerUpdate();
        csv << sys.GetTimerJacobian() << sys.GetTimerLSsetup() << sys.GetTimerLSsolve();
        csv << mesh->GetTimeInternalForces() << mesh->GetTimeJacobianLoad();
        if (ls_direct) {
            csv << ls_direct->GetTimeSetup_Assembly() << ls_direct->GetTimeSetup_SolverCall()
                << ls_direct->GetTimeSolve_Assembly() << ls_direct->GetTimeSolve_SolverCall();
        }
        csv << endl;
    }

    std::string out_file = out_dir + "/timing.out";
    csv.WriteToFile(out_file);

#ifdef CHRONO_POSTPROCESS
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
#endif

    return 0;
}
