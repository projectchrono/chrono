// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Jason Zhou
// =============================================================================
//
// Demo to show Viper Rover operated on SCM Terrain
//
// =============================================================================

#include "chrono_models/robot/viper/Viper.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/input_output/ChWriterCSV.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"

#include "chrono/core/ChTimer.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include <cstdlib>
#include <string>

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

#include "chrono/assets/ChVisualSystem.h"
#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif
#ifdef CHRONO_VSG
    #include "chrono_vehicle/visualization/ChScmVisualizationVSG.h"
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;
using namespace chrono::viper;

// -----------------------------------------------------------------------------

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Headless baseline: set render=false to run without a display window.
bool render = false;
double t_end = 2.0;  // simulation end time when running headless (seconds)

bool output = false;

// SCM grid spacing (BENCH: override with env SCM_DELTA, in metres)
double mesh_resolution = 0.02;

// BENCH: number of Curiosity-style obstacles to drop on the terrain (env SCM_ROCKS, 0 or 6)
int num_rocks = 0;

// Enable/disable bulldozing effects
bool enable_bulldozing = true;

// Enable/disable active domains feature
bool enable_active_domains = true;

// If true, use provided callback to change soil properties based on location
bool var_params = false;

// Define Viper rover wheel type
ViperWheelType wheel_type = ViperWheelType::RealWheel;

// -----------------------------------------------------------------------------

// Custom callback for setting location-dependent soil properties.
// Note that the location is given in the SCM reference frame.
class MySoilParams : public vehicle::SCMTerrain::SoilParametersCallback {
  public:
    virtual void Set(const ChVector3d& loc,
                     double& Bekker_Kphi,
                     double& Bekker_Kc,
                     double& Bekker_n,
                     double& Mohr_cohesion,
                     double& Mohr_friction,
                     double& Janosi_shear,
                     double& elastic_K,
                     double& damping_R) override {
        Bekker_Kphi = 0.82e6;
        Bekker_Kc = 0.14e4;
        Bekker_n = 1.0;
        Mohr_cohesion = 0.017e4;
        Mohr_friction = 35.0;
        Janosi_shear = 1.78e-2;
        elastic_K = 2e8;
        damping_R = 3e4;
    }
};

// Use custom material for the Viper Wheel
bool use_custom_mat = false;

// -----------------------------------------------------------------------------

// Return customized wheel material parameters
std::shared_ptr<ChContactMaterial> CustomWheelMaterial(ChContactMethod contact_method) {
    float mu = 0.4f;   // coefficient of friction
    float cr = 0.1f;   // coefficient of restitution
    float Y = 2e7f;    // Young's modulus
    float nu = 0.3f;   // Poisson ratio
    float kn = 2e5f;   // normal stiffness
    float gn = 40.0f;  // normal viscous damping
    float kt = 2e5f;   // tangential stiffness
    float gt = 20.0f;  // tangential viscous damping

    switch (contact_method) {
        case ChContactMethod::NSC: {
            auto matNSC = chrono_types::make_shared<ChContactMaterialNSC>();
            matNSC->SetFriction(mu);
            matNSC->SetRestitution(cr);
            return matNSC;
        }
        case ChContactMethod::SMC: {
            auto matSMC = chrono_types::make_shared<ChContactMaterialSMC>();
            matSMC->SetFriction(mu);
            matSMC->SetRestitution(cr);
            matSMC->SetYoungModulus(Y);
            matSMC->SetPoissonRatio(nu);
            matSMC->SetKn(kn);
            matSMC->SetGn(gn);
            matSMC->SetKt(kt);
            matSMC->SetGt(gt);
            return matSMC;
        }
        default:
            return std::shared_ptr<ChContactMaterial>();
    }
}

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Opt-in run-time visualization (VSG). Headless by default; set SCM_VIS=1 to open a window.
    if (const char* e = std::getenv("SCM_VIS")) {
        if (std::string(e) != "0")
            render = true;
    }

    // Wheel dimensions (for SCM active domains)
    double wheel_diameter = 0.6;
    double wheel_width = 0.4;
    ChVector3d wheel_size(wheel_diameter, wheel_width, wheel_diameter);

    // Create a Chrono physical system and associated collision system
    ChSystemSMC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    // Initialize output
    std::string out_dir = GetChronoOutputPath() + "ROBOT_Viper_SCM";
    if (output) {
        if (!CreateOutputDirectory(std::filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }
    ChWriterCSV csv(" ");

    // Create the rover
    auto driver = chrono_types::make_shared<ViperDCMotorControl>();

    Viper viper(&sys, wheel_type);

    viper.SetDriver(driver);
    if (use_custom_mat)
        viper.SetWheelContactMaterial(CustomWheelMaterial(ChContactMethod::NSC));

    viper.Initialize(ChFrame<>(ChVector3d(-5, 0, -0.2), QUNIT));

    // BENCH knobs
    if (const char* e = std::getenv("SCM_DELTA"))
        mesh_resolution = std::atof(e);
    if (const char* e = std::getenv("SCM_ROCKS"))
        num_rocks = std::atoi(e);
    if (const char* e = std::getenv("SCM_TEND"))
        t_end = std::atof(e);
    int warmup_steps = 400;  // BENCH: steps excluded from the steady-state average (env SCM_WARMUP)
    if (const char* e = std::getenv("SCM_WARMUP"))
        warmup_steps = std::atoi(e);
    double rock_drop = 0.02;  // BENCH: obstacle clearance above the soil surface (env SCM_ROCK_DROP)
    if (const char* e = std::getenv("SCM_ROCK_DROP"))
        rock_drop = std::atof(e);
    ChTimer steady_timer;
    bool steady_started = false;

    // BENCH: the six Curiosity-demo obstacles, mapped into this demo's Z-up world frame.
    std::vector<std::shared_ptr<ChBodyAuxRef>> rocks;
    if (num_rocks > 0) {
        std::vector<std::string> rock_meshfile = {
            "robot/curiosity/rocks/rock1.obj", "robot/curiosity/rocks/rock1.obj",  //
            "robot/curiosity/rocks/rock1.obj", "robot/curiosity/rocks/rock1.obj",  //
            "robot/curiosity/rocks/rock3.obj", "robot/curiosity/rocks/rock3.obj"   //
        };
        std::vector<ChVector3d> rock_pos = {
            ChVector3d(-2.5, -1.0, 0), ChVector3d(-2.5, +1.0, 0),  //
            ChVector3d(-1.0, -1.0, 0), ChVector3d(-1.0, +1.0, 0),  //
            ChVector3d(+0.5, -1.0, 0), ChVector3d(+0.5, +1.0, 0)   //
        };
        const double soil_z = -0.5;  // terrain reference frame, see SetReferenceFrame below
        std::vector<double> rock_scale = {0.8, 0.8, 0.45, 0.45, 0.45, 0.45};
        double rock_density = 8000;
        auto rock_mat = ChContactMaterial::DefaultMaterial(sys.GetContactMethod());

        for (int i = 0; i < num_rocks && i < 6; i++) {
            auto mesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(rock_meshfile[i]), false, true);
            mesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(rock_scale[i]));

            double mass;
            ChVector3d cog;
            ChMatrix33<> inertia;
            mesh->ComputeMassProperties(true, mass, cog, inertia);
            ChMatrix33<> principal_inertia_rot;
            ChVector3d principal_I;
            ChInertiaUtils::PrincipalInertia(inertia, principal_I, principal_inertia_rot);

            // Sit each obstacle rock_drop above the soil, measured from its own scaled geometry, so
            // the drop is short and identical for every rock regardless of mesh size. Curiosity's demo
            // uses a fixed 0.2 m origin height, which is a different fall per mesh and takes longer to
            // settle than a short benchmark run allows.
            ChVector3d pos = rock_pos[i];
            pos.z() = soil_z + rock_drop - mesh->GetBoundingBox().min.z();

            auto body = chrono_types::make_shared<ChBodyAuxRef>();
            sys.Add(body);
            body->SetFixed(false);
            body->SetFrameRefToAbs(ChFrame<>(pos, QUNIT));
            body->SetFrameCOMToRef(ChFrame<>(cog, principal_inertia_rot));
            body->SetMass(mass * rock_density);
            body->SetInertiaXX(rock_density * principal_I);

            auto ct_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(rock_mat, mesh, false, false, 0.005);
            body->AddCollisionShape(ct_shape);
            body->EnableCollision(true);

            auto vis_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
            vis_shape->SetMesh(mesh);
            vis_shape->SetBackfaceCull(true);
            body->AddVisualShape(vis_shape);
            rocks.push_back(body);
        }
    }

    // Get wheels and bodies to set up SCM patches
    auto Wheel_1 = viper.GetWheel(ViperWheelID::V_LF)->GetBody();
    auto Wheel_2 = viper.GetWheel(ViperWheelID::V_RF)->GetBody();
    auto Wheel_3 = viper.GetWheel(ViperWheelID::V_LB)->GetBody();
    auto Wheel_4 = viper.GetWheel(ViperWheelID::V_RB)->GetBody();
    auto Body_1 = viper.GetChassis()->GetBody();

    //
    // THE DEFORMABLE TERRAIN
    //

    // Create the 'deformable terrain' object
    vehicle::SCMTerrain terrain(&sys);

    // Displace/rotate the terrain reference frame.
    // Note that SCMTerrain uses a default ISO reference frame (Z up). Since the mechanism is modeled here in
    // a Y-up global frame, we rotate the terrain frame by -90 degrees about the X axis.
    terrain.SetReferenceFrame(ChCoordsys<>(ChVector3d(0, 0, -0.5)));

    // Add an active domain for every wheel. NOTE: this must happen *before* Initialize() -- Initialize()
    // calls SetupInitial() internally, which (if m_user_domains is still false at that point) pushes a
    // null-body default domain that AddActiveDomain() never clears, crashing the first headless step.
    // The HMMWV demo (demo_VEH_SCMTerrain_WheeledVehicle.cpp) already calls AddActiveDomain before
    // Initialize(); this demo didn't, which is what actually caused that segfault, not a library bug.
    if (enable_active_domains) {
        terrain.AddActiveDomain(Wheel_1, ChVector3d(0, 0, 0), wheel_size);
        terrain.AddActiveDomain(Wheel_2, ChVector3d(0, 0, 0), wheel_size);
        terrain.AddActiveDomain(Wheel_3, ChVector3d(0, 0, 0), wheel_size);
        terrain.AddActiveDomain(Wheel_4, ChVector3d(0, 0, 0), wheel_size);

        // BENCH: obstacles need their own domains or SCM never ray-casts under them -- they get no
        // support force and fall through the terrain. Same dimensions demo_ROBOT_Curiosity_SCM uses.
        for (auto& r : rocks)
            terrain.AddActiveDomain(r, VNULL, ChVector3d(2.0, 2.0, 2.0));
    }

    // Use a regular grid:
    double length = 14;
    double width = 4;
    terrain.Initialize(length, width, mesh_resolution);

    // Set the soil terramechanical parameters
    if (var_params) {
        // Here we use the soil callback defined at the beginning of the code
        auto my_params = chrono_types::make_shared<MySoilParams>();
        terrain.RegisterSoilParametersCallback(my_params);
    } else {
        // If var_params is set to be false, these parameters will be used
        terrain.SetSoilParameters(2e6,   // Bekker Kphi
                                  0,     // Bekker Kc
                                  1.1,   // Bekker n exponent
                                  0,     // Mohr cohesive limit (Pa)
                                  30,    // Mohr friction limit (degrees)
                                  0.01,  // Janosi shear coefficient (m)
                                  2e8,   // Elastic stiffness (Pa/m), before plastic yield, must be > Kphi
                                  3e4    // Damping (Pa s/m), proportional to negative vertical speed (optional)
        );
    }

    // Set up bulldozing factors
    if (enable_bulldozing) {
        terrain.EnableBulldozing(true);  // inflate soil at the border of the rut
        terrain.SetBulldozingParameters(
            55,  // angle of friction for erosion of displaced material at the border of the rut
            1,   // displaced material vs downward pressed material.
            5,   // number of erosion refinements per timestep
            6);  // number of concentric vertex selections subject to erosion
    }

    // Set some visualization parameters: either with a texture, or with falsecolor plot, etc.
    terrain.SetPlotType(vehicle::SCMTerrain::PLOT_PRESSURE, 0, 20000);

    terrain.SetMeshWireframe(false);

    // Create the run-time visualization interface
#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    std::shared_ptr<ChVisualSystem> vis;
    if (render)
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
            vis_irr->AttachSystem(&sys);
            vis_irr->SetCameraVertical(CameraVerticalDir::Z);
            vis_irr->SetWindowSize(800, 600);
            vis_irr->SetWindowTitle("Viper Rover on SCM");
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddCamera(ChVector3d(1.0, 2.0, 1.4), ChVector3d(0, 0, wheel_diameter));
            vis_irr->AddTypicalLights();
            vis_irr->AddLightWithShadow(ChVector3d(-5.0, -0.5, 8.0), ChVector3d(-1, 0, 0), 100, 1, 35, 85, 512,
                                        ChColor(0.8f, 0.8f, 0.8f));
            vis_irr->EnableShadows();

            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            // SCM plugin
            auto visSCM = chrono_types::make_shared<vehicle::ChScmVisualizationVSG>(&terrain);

            auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
            vis_vsg->AttachSystem(&sys);
            vis_vsg->AttachPlugin(visSCM);
            vis_vsg->SetWindowSize(1280, 800);
            vis_vsg->SetWindowTitle("Viper Rover on SCM");
            vis_vsg->AddCamera(ChVector3d(1.0, 2.0, 1.4), ChVector3d(0, 0, wheel_diameter));
            vis_vsg->EnableShadows();
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    // Select SCM contact-force backend at run time: env SCM_GPU=0 -> CPU, else GPU (default on when built
    // with SCM GPU support).
#ifdef CHRONO_HAS_SCM_GPU
    {
        auto scm_cfg = terrain.GetScmGpuConfig();
        const char* e = std::getenv("SCM_GPU");
        scm_cfg.enabled = !(e && std::string(e) == "0");
        scm_cfg.min_hits = 0;     // force GPU when enabled (don't fall back to CPU)
        scm_cfg.profile = false;  // clean timing (no per-step stderr)
        terrain.SetScmGpuConfig(scm_cfg);
        std::cout << "SCM contact-force backend: " << (scm_cfg.enabled ? "GPU (HIP)" : "CPU") << std::endl;
    }
#else
    std::cout << "SCM contact-force backend: CPU (built without SCM GPU)" << std::endl;
#endif

    // Select SCM ray-cast backend at run time: env SCM_RAYCAST_GPU = "hip" | "ref" | "cpu".
    // Unset means the SCMTerrain default, which is the GPU backend in a build configured with it.
    {
        const char* e = std::getenv("SCM_RAYCAST_GPU");
        std::string mode = e ? e : "default";
#ifdef CHRONO_HAS_SCM_GPU
        if (mode == "ref") {
            // The HIP backend is on by default and wins the dispatch, so it has to be turned off
            // explicitly for the reference path to be reached.
            terrain.EnableRaycastGpuHip(false);
            terrain.EnableRaycastGpuReference(true);
            std::cout << "SCM ray-cast backend: CPU reference (slow, correctness stand-in only)" << std::endl;
        } else if (mode == "cpu") {
            terrain.EnableRaycastGpuHip(false);
            std::cout << "SCM ray-cast backend: CPU (Bullet)" << std::endl;
        } else {
            terrain.EnableRaycastGpuHip(true);
            std::cout << "SCM ray-cast backend: HIP" << (e ? "" : " (default)") << std::endl;
            const char* prec = std::getenv("SCM_RAYCAST_GPU_PRECISION");
            std::cout << "  precision: " << (prec ? prec : "default (fp64 on AMD, fp32 on NVIDIA HIP backend)")
                      << std::endl;
        }
#else
        if (mode == "ref") {
            terrain.EnableRaycastGpuReference(true);
            std::cout << "SCM ray-cast backend: CPU reference (slow, correctness stand-in only)" << std::endl;
        } else {
            std::cout << "SCM ray-cast backend: CPU (Bullet, built without SCM GPU)" << std::endl;
        }
#endif
    }

    ChTimer sim_timer;
    int nsteps = 0;
    sim_timer.start();
    while (render ? vis->Run() : (sys.GetChTime() < t_end)) {
        if (render) {
            vis->BeginScene();
            vis->SetCameraTarget(Body_1->GetPos());
            vis->Render();
            vis->EndScene();
        }

        if (output) {
            // write drive torques of all four wheels into file
            csv << sys.GetChTime() << viper.GetWheelTracTorque(ViperWheelID::V_LF)
                << viper.GetWheelTracTorque(ViperWheelID::V_RF) << viper.GetWheelTracTorque(ViperWheelID::V_LB)
                << viper.GetWheelTracTorque(ViperWheelID::V_RB) << std::endl;
        }

        sys.DoStepDynamics(5e-4);
        viper.Update();
        nsteps++;
        if (!steady_started && nsteps >= warmup_steps) {
            steady_timer.start();
            steady_started = true;
        }
        if (nsteps % 200 == 0)
            std::cout << "t=" << sys.GetChTime() << "  RTF=" << sys.GetRTF() << std::endl;
        if (nsteps % 1000 == 0)
            terrain.PrintStepStatistics(std::cout);
    }
    sim_timer.stop();
    if (steady_started)
        steady_timer.stop();
    int steady_steps = nsteps > warmup_steps ? nsteps - warmup_steps : 0;
    double steady_ms = steady_steps > 0 ? 1e3 * steady_timer() / steady_steps : 0.0;
    double ms_per_step = 1e3 * sim_timer() / (nsteps > 0 ? nsteps : 1);
    double avg_rtf = (sim_timer() / (nsteps > 0 ? nsteps : 1)) / 5e-4;  // wall/sim per step
    std::cout << "[BENCH] delta=" << mesh_resolution << " rocks=" << num_rocks << "  steps=" << nsteps << "  sim_time=" << sys.GetChTime() << "s"
              << "  wall=" << sim_timer() << "s"
              << "  (" << ms_per_step << " ms/step, avg RTF=" << avg_rtf << ")"
              << "  steady=" << steady_ms << " ms/step over " << steady_steps << " steps"
              << "  (RTF=" << (steady_ms * 1e-3) / 5e-4 << ")"
              << "  x_end=" << Body_1->GetPos().x();
    if (!rocks.empty()) {
        double zmin = 1e9, zmax = -1e9;
        for (auto& r : rocks) {
            zmin = std::min(zmin, r->GetPos().z());
            zmax = std::max(zmax, r->GetPos().z());
        }
        std::cout << "  rock_z=[" << zmin << ", " << zmax << "]";
    }
    // BENCH: wheel seating, on the same line so one grep captures the whole record.
    // sinkage = how far the LF wheel's lowest point sits below the deformed soil surface;
    // rut = how far that surface has been pushed below its undisturbed level.
    {
        ChVector3d wp = Wheel_1->GetPos();
        double soil = terrain.GetHeight(ChVector3d(wp.x(), wp.y(), 0));
        std::cout << "  sinkage=" << (soil - (wp.z() - wheel_diameter / 2)) << "  rut=" << (-0.5 - soil);
    }
    std::cout << std::endl;

    if (output) {
        csv.WriteToFile(out_dir + "/output.dat");
    }

    return 0;
}
