// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Khailanii Slaton, Dan Negrut
// =============================================================================
//
// CRM plate-sinkage (bevameter-style) demo.
//
// A rigid circular plate is pushed vertically at a constant velocity into a bed
// of frictional (optionally cohesive) granular soil modeled with the Chrono CRM
// solver (continuum representation of the terrain, elastic-plastic SPH), and the
// bearing pressure under the plate is logged against sinkage. This
// "pressure-sinkage" curve is the classical terramechanics measurement used to
// characterize soil bearing response.
//
// One program, many scenarios: every soil and test parameter is a command-line
// option with a sensible default, so a user can reproduce a specific test or
// sweep a parameter without editing code.
//
// Example command lines:
//   demo_FSI-SPH_PlateSinkage
//       nominal: 300 mm plate, phi = 40 deg, c = 1 kPa, mu(I) rheology, Earth g
//   demo_FSI-SPH_PlateSinkage --gravity moon
//       lunar gravity (1.62 m/s^2): lower confinement, softer bearing response
//   demo_FSI-SPH_PlateSinkage --cohesion 0
//       cohesionless (dry granular) soil: isolates the frictional contribution
//   demo_FSI-SPH_PlateSinkage --rheology mcc
//       Modified Cam-Clay rheology instead of mu(I)
//   demo_FSI-SPH_PlateSinkage --spacing 0.02 --no_vis --output
//       coarser/faster preview (~5 min), headless, write data files
//
// Stages: (1) gravity settle with the plate held above the surface, (2)
// constant-velocity push to the target sinkage. Output: a pressure-sinkage CSV
// and, optionally, the settled particle state and a gnuplot of the curve.
//
// UNITS: SI throughout (m, s, kg, N, Pa). Angles are in DEGREES on the command
// line. Gravity acts along -Z. Stress sign convention is compression-negative
// (so the vertical stress in a bed under gravity is negative). The logged
// bearing pressure is a magnitude in Pa. Build against a Chrono install with the
// FSI-SPH module enabled (requires CUDA or HIP).
// =============================================================================

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/functions/ChFunctionConst.h"
#include "chrono/functions/ChFunctionRamp.h"
#include "chrono/assets/ChVisualSystem.h"
#include "chrono/utils/ChUtilsSamplers.h"

#include "chrono_fsi/sph/ChFsiSystemSPH.h"

#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChSphVisualizationVSG.h"
#endif

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

#include "chrono_thirdparty/cxxopts/ChCLI.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

using std::cerr;
using std::cout;
using std::endl;

// -----------------------------------------------------------------------------
// Problem specification (defaults are the demo's "nominal" model). All values SI;
// angles in degrees. These are generic, representative soil parameters, not tied
// to any particular measured material.
// -----------------------------------------------------------------------------
struct PlateSinkageParams {
    // Constitutive model
    std::string rheology = "mu_i";  // "mu_i" (default) or "mcc"
    std::string gravity = "earth";  // "earth" (9.81) or "moon" (1.62) [m/s^2]

    // Soil (our-owned, representative values)
    double rho = 1600;        // bulk density [kg/m^3]
    double Emod = 28e6;       // Young's modulus [Pa] (generic demo value, 28 MPa)
    double nu = 0.25;         // Poisson ratio [-]
    double phi_deg = 40;      // internal friction angle [deg]
    double cohesion = 1000;   // cohesion [Pa] (1 kPa; consumed by mu(I) only)
    double grain_diam = 0.005;  // mean grain diameter [m] (mu(I) inertial number)

    // Test (plate and push)
    double diameter = 0.300;       // plate diameter [m] (300 mm circular plate)
    double velocity = 0.005;       // push velocity [m/s] (5 mm/s; quasi-static)
    double target_sinkage = 0.020;  // target sinkage = plate push travel [m] (20 mm)

    // Domain and resolution
    double spacing = 0.0125;  // SPH particle spacing d0 [m] (converged default,
                              //   ~15 min on one GPU; use --spacing 0.02 for a
                              //   coarser ~5 min preview, which runs stiffer)
    double container_x = 0.9;  // soil bin size, X [m] (3 plate diameters wide)
    double container_y = 0.9;  // soil bin size, Y [m]
    double container_z = 0.5;  // soil bin depth, Z [m]

    // Numerics (stabilization)
    double settle_time = 0.5;            // minimum gravity-settle time [s]
    double artificial_viscosity = 0.2;   // artificial viscosity coefficient [-]
    double free_surface_threshold = 0.8;  // particle-deficiency free-surface cutoff [-]

    // Output and visualization
    int output_points = 100;   // number of equidistant-in-time output intervals (curve has ~this many rows)
    bool output = false;       // write settled particle state + geostatic profile
    bool render = true;        // run-time visualization (VSG); --no_vis disables
    bool verbose = false;      // verbose solver output
};

// -----------------------------------------------------------------------------

bool GetProblemSpecs(int argc, char* argv[], PlateSinkageParams& p) {
    ChCLI cli(argv[0], "Chrono CRM plate-sinkage demo (mu(I) / MCC rheology)");

    cli.AddOption<std::string>("Soil", "rheology", "Rheology model (mu_i/mcc)", p.rheology);
    cli.AddOption<double>("Soil", "rho", "Bulk density [kg/m^3]", std::to_string(p.rho));
    cli.AddOption<double>("Soil", "Emod", "Young's modulus [Pa]", std::to_string(p.Emod));
    cli.AddOption<double>("Soil", "nu", "Poisson ratio [-]", std::to_string(p.nu));
    cli.AddOption<double>("Soil", "phi_deg", "Internal friction angle [deg]", std::to_string(p.phi_deg));
    cli.AddOption<double>("Soil", "cohesion", "Cohesion [Pa] (mu(I) only)", std::to_string(p.cohesion));
    cli.AddOption<double>("Soil", "grain_diam", "Mean grain diameter [m] (mu(I))", std::to_string(p.grain_diam));

    cli.AddOption<double>("Test", "diameter", "Plate diameter [m]", std::to_string(p.diameter));
    cli.AddOption<double>("Test", "velocity", "Push velocity [m/s]", std::to_string(p.velocity));
    cli.AddOption<double>("Test", "target_sinkage", "Target sinkage = plate push travel [m]; reported sinkage is measured from the settled surface", std::to_string(p.target_sinkage));
    cli.AddOption<std::string>("Test", "gravity", "Gravity (earth/moon)", p.gravity);

    cli.AddOption<double>("Domain", "spacing", "SPH particle spacing d0 [m]", std::to_string(p.spacing));
    cli.AddOption<double>("Domain", "container_x", "Soil bin size X [m]", std::to_string(p.container_x));
    cli.AddOption<double>("Domain", "container_y", "Soil bin size Y [m]", std::to_string(p.container_y));
    cli.AddOption<double>("Domain", "container_z", "Soil bin depth Z [m]", std::to_string(p.container_z));

    cli.AddOption<double>("Numerics", "settle_time", "Minimum gravity-settle time [s]", std::to_string(p.settle_time));
    cli.AddOption<double>("Numerics", "artificial_viscosity", "Artificial viscosity coefficient [-]", std::to_string(p.artificial_viscosity));
    cli.AddOption<double>("Numerics", "free_surface_threshold", "Free-surface threshold [-]", std::to_string(p.free_surface_threshold));

    cli.AddOption<int>("Output", "output_points", "Number of equidistant-in-time output intervals (curve has ~this many rows)", std::to_string(p.output_points));
    cli.AddOption<bool>("Output", "output", "Write settled particle state and geostatic profile");
    cli.AddOption<bool>("Output", "quiet", "Disable verbose solver output");
    cli.AddOption<bool>("Visualization", "no_vis", "Disable run-time visualization");

    if (!cli.Parse(argc, argv)) {
        cli.Help();
        return false;
    }

    p.rheology = cli.GetAsType<std::string>("rheology");
    p.gravity = cli.GetAsType<std::string>("gravity");
    p.rho = cli.GetAsType<double>("rho");
    p.Emod = cli.GetAsType<double>("Emod");
    p.nu = cli.GetAsType<double>("nu");
    p.phi_deg = cli.GetAsType<double>("phi_deg");
    p.cohesion = cli.GetAsType<double>("cohesion");
    p.grain_diam = cli.GetAsType<double>("grain_diam");

    p.diameter = cli.GetAsType<double>("diameter");
    p.velocity = cli.GetAsType<double>("velocity");
    p.target_sinkage = cli.GetAsType<double>("target_sinkage");

    p.spacing = cli.GetAsType<double>("spacing");
    p.container_x = cli.GetAsType<double>("container_x");
    p.container_y = cli.GetAsType<double>("container_y");
    p.container_z = cli.GetAsType<double>("container_z");

    p.settle_time = cli.GetAsType<double>("settle_time");
    p.artificial_viscosity = cli.GetAsType<double>("artificial_viscosity");
    p.free_surface_threshold = cli.GetAsType<double>("free_surface_threshold");

    p.output_points = cli.GetAsType<int>("output_points");
    p.output = cli.GetAsType<bool>("output");
    p.verbose = !cli.GetAsType<bool>("quiet");
    p.render = !cli.GetAsType<bool>("no_vis");
    return true;
}

// Case-insensitive lowercase helper for parsing rheology/gravity strings.
static std::string to_lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return (char)std::tolower(c); });
    return s;
}

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    PlateSinkageParams p;
    if (!GetProblemSpecs(argc, argv, p))
        return 1;

    // ---- Interpret string options ------------------------------------------
    const std::string rheology = to_lower(p.rheology);
    const bool use_mcc = (rheology == "mcc");
    if (!use_mcc && rheology != "mu_i" && rheology != "mui" && rheology != "mu_of_i") {
        cerr << "ERROR: --rheology must be mu_i or mcc (got " << p.rheology << ")" << endl;
        return 1;
    }
    const std::string grav = to_lower(p.gravity);
    if (grav != "earth" && grav != "moon") {
        cerr << "ERROR: --gravity must be earth or moon (got " << p.gravity << ")" << endl;
        return 1;
    }
    const double gravity_mag = (grav == "moon") ? 1.62 : 9.81;  // [m/s^2]
    const double phi = p.phi_deg * CH_PI / 180.0;  // friction angle [rad]
    if (use_mcc && p.cohesion != 0)
        cout << "[warn] cohesion is not consumed by the MCC rheology; ignored" << endl;

    // Reject nonphysical inputs rather than producing garbage or dividing by zero downstream.
    auto require_pos = [](double v, const char* name) {
        if (v <= 0) {
            cerr << "ERROR: --" << name << " must be positive (got " << v << ")" << endl;
            return false;
        }
        return true;
    };
    if (!(require_pos(p.rho, "rho") && require_pos(p.Emod, "Emod") && require_pos(p.diameter, "diameter") &&
          require_pos(p.velocity, "velocity") && require_pos(p.target_sinkage, "target_sinkage") &&
          require_pos(p.spacing, "spacing") && require_pos(p.container_x, "container_x") &&
          require_pos(p.container_y, "container_y") && require_pos(p.container_z, "container_z")))
        return 1;
    if (p.nu < 0 || p.nu >= 0.5) {
        cerr << "ERROR: --nu must be in [0, 0.5) (got " << p.nu << ")" << endl;
        return 1;
    }
    if (p.output_points < 1) {
        cerr << "ERROR: --output_points must be >= 1 (got " << p.output_points << ")" << endl;
        return 1;
    }
    if (p.diameter >= p.container_x || p.diameter >= p.container_y) {
        cerr << "ERROR: plate diameter (" << p.diameter << " m) must be smaller than the container width" << endl;
        return 1;
    }
    if (p.phi_deg < 0 || p.phi_deg >= 90) {
        cerr << "ERROR: --phi_deg must be in [0, 90) deg (got " << p.phi_deg << ")" << endl;
        return 1;
    }
    if (p.cohesion < 0) {
        cerr << "ERROR: --cohesion must be >= 0 (got " << p.cohesion << ")" << endl;
        return 1;
    }
    if (p.grain_diam <= 0 || p.free_surface_threshold <= 0 || p.settle_time < 0 || p.artificial_viscosity < 0) {
        cerr << "ERROR: --grain_diam and --free_surface_threshold must be > 0; --settle_time and "
                "--artificial_viscosity must be >= 0" << endl;
        return 1;
    }
    if (p.container_x <= 2 * p.spacing || p.container_y <= 2 * p.spacing || p.container_z <= 2 * p.spacing) {
        cerr << "ERROR: each container dimension must exceed 2*spacing (spacing = " << p.spacing << " m)" << endl;
        return 1;
    }

    // ---- Derived plate quantities ------------------------------------------
    const double plate_radius = p.diameter / 2;                        // [m]
    const double plate_thickness = 0.02;                               // rigid puck height [m] (sized for BCE support)
    const double plate_density = 7.8e3;                                // [kg/m^3] (steel; mass only sets inertia,
                                                                       //   the motion is motor-prescribed)
    const double plate_area = CH_PI * plate_radius * plate_radius;     // [m^2] (divides force -> pressure)
    const double plate_mass = plate_density * plate_area * plate_thickness;  // [kg]

    // ---- Physics systems ----------------------------------------------------
    ChSystemSMC sysMBS;
    ChFsiFluidSystemSPH sysSPH;
    ChFsiSystemSPH sysFSI(&sysMBS, &sysSPH);
    sysFSI.SetVerbose(p.verbose);

    // Explicit integration step. The CRM solver is stable at a CFL-limited step;
    // this seed is small enough for the stiffness here and the variable time step
    // adapts from it. meta_step is the FSI communication interval.
    const double step_size = 2e-5;                // [s]
    const double meta_step = 5 * step_size;       // [s]
    sysFSI.SetStepSizeCFD(step_size);
    sysFSI.SetStepsizeMBD(step_size);

    const ChVector3d gravity(0, 0, -gravity_mag);
    sysSPH.SetGravitationalAcceleration(gravity);
    sysMBS.SetGravitationalAcceleration(gravity);

    // ---- Soil constitutive model -------------------------------------------
    // The CRM soil is an elastic-plastic continuum. Two rheologies are offered:
    //   mu(I): inertial-number granular friction; cohesion enters here.
    //   MCC  : Modified Cam-Clay critical-state plasticity.
    ChFsiFluidSystemSPH::ElasticMaterialProperties mat;
    mat.density = p.rho;
    mat.Young_modulus = p.Emod;
    mat.Poisson_ratio = p.nu;
    if (use_mcc) {
        mat.rheology_model = RheologyCRM::MCC;
        mat.mcc_M = (6.0 * std::sin(phi)) / (3.0 - std::sin(phi));  // CSL slope from phi
        mat.mcc_kappa = 0.00625;   // swelling (unload/reload) slope
        mat.mcc_lambda = 0.025;    // normal-compression-line slope
        mat.mcc_v_lambda = 2.0;    // specific volume at reference pressure
    } else {
        mat.rheology_model = RheologyCRM::MU_OF_I;
        mat.mu_fric_s = std::tan(phi);   // static friction coefficient = tan(phi)
        mat.mu_fric_2 = std::tan(phi);   // high-inertia friction (same here)
        mat.mu_I0 = 0.03;                // reference inertial number
        mat.average_diam = p.grain_diam;  // mean grain diameter [m]
        mat.cohesion_coeff = p.cohesion;  // cohesion [Pa]
    }
    sysSPH.SetElasticSPH(mat);

    // ---- SPH numerical parameters (held at validated values) ----------------
    ChFsiFluidSystemSPH::SPHParameters sph;
    sph.integration_scheme = IntegrationScheme::RK2;
    sph.initial_spacing = p.spacing;
    sph.d0_multiplier = 1.3;                       // smoothing-length / spacing ratio
    sph.artificial_viscosity = p.artificial_viscosity;
    sph.shifting_method = ShiftingMethod::PPST_XSPH;
    sph.shifting_xsph_eps = 0.5;
    sph.shifting_ppst_pull = 1.0;
    sph.shifting_ppst_push = 3.0;
    sph.free_surface_threshold = p.free_surface_threshold;
    sph.num_proximity_search_steps = 1;
    sph.kernel_type = KernelType::WENDLAND;
    sph.boundary_method = BoundaryMethod::ADAMI;
    sph.viscosity_method = ViscosityMethod::ARTIFICIAL_BILATERAL;
    sph.use_variable_time_step = true;
    sysSPH.SetSPHParameters(sph);

    // ---- Terrain particles (fresh lattice) ----------------------------------
    // A regular grid of SPH particles fills the soil bin. The top row defines the
    // initial free surface; particles are initialized with a hydrostatic pressure
    // so the bed starts near geostatic equilibrium and settles quickly.
    chrono::utils::ChGridSampler<> sampler(p.spacing);
    ChVector3d box_center(0, 0, p.container_z / 2);
    ChVector3d box_half(p.container_x / 2 - p.spacing, p.container_y / 2 - p.spacing, p.container_z / 2 - p.spacing);
    std::vector<ChVector3d> points = sampler.SampleBox(box_center, box_half);
    if (points.empty()) {
        cerr << "ERROR: no soil particles sampled; container too small for the spacing" << endl;
        return 1;
    }

    double soil_top_init = -std::numeric_limits<double>::infinity();  // [m]
    for (const auto& pt : points)
        soil_top_init = std::max(soil_top_init, pt.z());

    const double rho0 = sysSPH.GetDensity();
    const double p_floor = p.rho * gravity_mag * p.spacing / 4;  // keep near-surface pressure finite [Pa]
    for (const auto& pt : points) {
        double p0 = std::max(rho0 * gravity_mag * (soil_top_init - pt.z()), p_floor);  // hydrostatic pressure [Pa]
        // Seed the normal-stress diagonal (tauXx,Yy,Zz) to -p0: an isotropic compressive
        // stress equal to the hydrostatic pressure (this is the full stress, not a
        // deviator). pc0 = p0 seeds the MCC pre-consolidation pressure (ignored by mu(I)).
        sysSPH.AddSPHParticle(pt, rho0, p0, sysSPH.GetViscosity(), ChVector3d(0), ChVector3d(-p0, -p0, -p0),
                              ChVector3d(0), p0);
    }

    // ---- Computational domain (a bit larger than the bin) -------------------
    ChVector3d dom_min(-p.container_x / 2 * 1.2, -p.container_y / 2 * 1.2, -0.1);
    ChVector3d dom_max(p.container_x / 2 * 1.2, p.container_y / 2 * 1.2, p.container_z * 1.3);
    sysSPH.SetComputationalDomain(ChAABB(dom_min, dom_max), BC_NONE);

    // ---- Container (fixed body with wall BCE markers) -----------------------
    auto container = chrono_types::make_shared<ChBody>();
    container->SetFixed(true);
    sysMBS.AddBody(container);
    const double bce_z = p.container_z * 1.05;  // walls a little taller than the soil
    auto box_bce = sysSPH.CreatePointsBoxContainer(ChVector3d(p.container_x, p.container_y, bce_z), ChVector3i(2, 2, -1));
    sysFSI.AddFsiBoundary(box_bce, ChFrame<>(ChVector3d(0, 0, bce_z / 2), QUNIT));

    // ---- Plate (rigid FSI body) --------------------------------------------
    // The plate MUST be the first FSI body added: the force readout below uses
    // GetFsiBodyForce(0). It starts half a spacing above the soil, held by a
    // linear motor, and is driven straight down during the push.
    auto plate = chrono_types::make_shared<ChBody>();
    const double gap = 0.5 * p.spacing;  // initial plate clearance above the soil [m]
    const double plate_z0 = soil_top_init + gap + plate_thickness / 2;
    plate->SetPos(ChVector3d(0, 0, plate_z0));
    plate->SetMass(plate_mass);
    const double Izz = plate_mass * plate_radius * plate_radius / 2;
    plate->SetInertiaXX(ChVector3d(Izz, Izz, Izz));
    sysMBS.AddBody(plate);
    auto plate_bce = sysSPH.CreatePointsCylinderInterior(plate_radius, plate_thickness, true);
    sysFSI.AddFsiBody(plate, plate_bce, ChFrame<>(ChVector3d(0), QUNIT), false);

    // ---- Linear motor: holds the plate during settle, then pushes -----------
    // ChLinkMotorLinearPosition prescribes translation along the link Z axis; an
    // identity frame makes that world-vertical. A constant 0 holds it in place;
    // the push replaces this with a downward ramp.
    auto motor = chrono_types::make_shared<ChLinkMotorLinearPosition>();
    motor->SetMotorFunction(chrono_types::make_shared<ChFunctionConst>(0.0));
    motor->Initialize(plate, container, ChFrame<>(ChVector3d(0), QUNIT));
    sysMBS.AddLink(motor);

    sysFSI.Initialize();

    cout << "[demo] rheology=" << (use_mcc ? "MCC" : "mu(I)") << "  particles=" << points.size()
         << "  spacing=" << p.spacing << "  plate D=" << p.diameter << " m"
         << "  gravity=" << gravity_mag << " m/s^2" << endl;
    cout << "[demo] soil_top_init=" << soil_top_init << " m  plate_bottom_init=" << (plate_z0 - plate_thickness / 2)
         << " m" << endl;

    // ---- Output directory ---------------------------------------------------
    std::string out_dir = GetChronoOutputPath() + "FSI_Plate_Sinkage/";
    if (!CreateOutputDirectory(std::filesystem::path(out_dir))) {
        cerr << "Error creating output directory " << out_dir << endl;
        return 1;
    }

    // ---- Run-time visualization (optional) ----------------------------------
    std::shared_ptr<ChVisualSystem> vis;
#ifdef CHRONO_VSG
    if (p.render) {
        auto visFSI = chrono_types::make_shared<ChSphVisualizationVSG>(&sysFSI);
        visFSI->EnableFluidMarkers(true);         // soil particles
        visFSI->EnableBoundaryMarkers(false);     // container walls (hidden for clarity)
        visFSI->EnableRigidBodyMarkers(true);     // the plate
        visFSI->SetSPHColorCallback(
            chrono_types::make_shared<ParticleVelocityColorCallback>(0, 2 * p.velocity), ChColormap::Type::FAST);

        auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        visVSG->AttachPlugin(visFSI);
        visVSG->AttachSystem(&sysMBS);
        visVSG->SetWindowTitle("CRM Plate Sinkage");
        visVSG->SetWindowSize(1280, 800);
        visVSG->AddCamera(ChVector3d(1.5 * p.container_x, -1.5 * p.container_y, soil_top_init + 0.4 * p.container_z),
                          ChVector3d(0, 0, soil_top_init));
        visVSG->SetLightIntensity(0.9f);
        visVSG->Initialize();
        vis = visVSG;
    }
#else
    p.render = false;
#endif

    // ---- Stage 1: gravity settle (plate held) -------------------------------
    // Run at least settle_time, then continue in 0.25 s chunks until the fastest
    // particle is slower than half the push velocity (a handful of surface grains
    // rattle indefinitely, so there is an absolute floor and a chunk cap).
    double t = 0;
    while (t < p.settle_time) {
        sysFSI.DoStepDynamics(meta_step);
        t += meta_step;
        if (p.render && !vis->Run())
            return 0;
        if (p.render)
            vis->Render();
    }
    const double settle_vtol = std::max(0.5 * p.velocity, 0.02);  // [m/s]
    auto max_particle_speed = [&]() {
        double vmax = 0;
        for (const auto& v : sysSPH.GetParticleVelocities())
            vmax = std::max(vmax, v.Length());
        return vmax;
    };
    for (int chunk = 0; chunk < 6; chunk++) {
        double vmax = max_particle_speed();
        cout << "[demo] settle: max particle speed " << vmax << " m/s at t=" << t << " s" << endl;
        if (vmax < settle_vtol)
            break;
        double t_chunk_end = t + 0.25;
        while (t < t_chunk_end) {
            sysFSI.DoStepDynamics(meta_step);
            t += meta_step;
            if (p.render && !vis->Run())
                return 0;
            if (p.render)
                vis->Render();
        }
    }

    // ---- Settled state: datum and (optional) diagnostics --------------------
    // The sinkage datum is the settled soil top, taken as the 95th percentile of
    // core-region particle heights (robust against a few fluffed surface grains).
    double soil_top = soil_top_init;  // [m]
    {
        auto pos = sysSPH.GetParticlePositions();
        std::vector<double> core_z;
        const double r_core2 = 0.64 * plate_radius * plate_radius;  // inner 80% radius of the plate
        for (const auto& q : pos) {
            if (q.x() * q.x() + q.y() * q.y() < r_core2)
                core_z.push_back(q.z());
        }
        if (core_z.size() > 20) {
            std::sort(core_z.begin(), core_z.end());
            soil_top = core_z[(size_t)(0.95 * (core_z.size() - 1))];
        }
        cout << "[demo] settle done at t=" << t << " s  soil_top=" << soil_top << " m" << endl;
    }
    if (p.output) {
        // Full per-particle state (positions, velocities, density/pressure, the 6
        // Cauchy stress components). For a static bed, sigma_zz(z) should equal
        // rho*g*(soil_top - z); this is the standard geostatic sanity check.
        std::string pdir = out_dir + "particles";
        std::filesystem::create_directories(pdir);
        sysSPH.SetOutputLevel(OutputLevel::CRM_FULL);
        sysSPH.SaveParticleData(pdir);
        // Subsampled settled profile for a quick plot of pressure vs depth.
        auto pos = sysSPH.GetParticlePositions();
        auto props = sysSPH.GetParticleFluidProperties();  // x=rho, y=pressure, z=viscosity
        std::ofstream geo(out_dir + "settled_profile.csv");
        geo << "z_m,pressure_Pa,rho_kgm3\n";
        for (size_t i = 0; i < pos.size(); i += 23)
            geo << pos[i].z() << "," << props[i].y() << "," << props[i].x() << "\n";
    }

    // ---- Stage 2: constant-velocity push ------------------------------------
    // Prescribe plate position z(t) = -velocity * (t - t_push), a downward ramp from the
    // settle end. Total travel = the requested target sinkage. The plate starts a
    // half-spacing above the surface; during settle the free surface rises ~that amount to
    // meet it, so the reported sinkage (soil_top - plate_bottom) runs from ~0 at push start
    // to ~target_sinkage at the end. Termination is on PRESCRIBED travel (exact under
    // displacement control), not on the noisier particle-derived datum.
    const double t_push = t;
    motor->SetMotorFunction(chrono_types::make_shared<ChFunctionRamp>(p.velocity * t_push, -p.velocity));
    const double travel_target = p.target_sinkage;            // plate downward travel (stroke) [m]
    const double push_duration = travel_target / p.velocity;  // [s]
    const double t_end_max = t_push + 1.5 * push_duration;    // safety cap [s]
    const double out_interval = push_duration / std::max(1, p.output_points);  // [s]
    double next_out = t_push;
    int next_progress_pct = 10;

    // Pressure-sinkage curve. Columns:
    //   time_s        : simulation time [s]
    //   plate_travel_m: prescribed downward plate displacement from push start [m]
    //   sinkage_m     : soil_top - plate_bottom [m] (penetration below the surface)
    //   force_N       : vertical soil-on-plate reaction [N] (negative = pushes up)
    //   pressure_Pa   : |force_N| / plate_area [Pa] (the bearing pressure plotted)
    std::ofstream csv(out_dir + "pressure_sinkage.csv");
    csv << "time_s,plate_travel_m,sinkage_m,force_N,pressure_Pa\n";
    csv << std::setprecision(9);

    double plate_travel = 0;
    while (plate_travel < travel_target && t < t_end_max) {
        sysFSI.DoStepDynamics(meta_step);
        t += meta_step;
        plate_travel = p.velocity * (t - t_push);

        if (p.render) {
            if (!vis->Run())
                break;
            vis->Render();
        }

        if (t >= next_out) {
            double plate_bottom = plate->GetPos().z() - plate_thickness / 2;  // [m]
            double sinkage = soil_top - plate_bottom;                         // [m]
            double force_z = sysFSI.GetFsiBodyForce(0).z();                   // [N] (plate is FSI body 0)
            double pressure = std::abs(force_z) / plate_area;                 // [Pa]
            csv << t << "," << plate_travel << "," << sinkage << "," << force_z << "," << pressure << "\n";
            next_out += out_interval;

            int pct = (int)(100 * plate_travel / travel_target);
            if (pct >= next_progress_pct) {
                cout << "[demo] push " << pct << "%  sinkage=" << sinkage << " m  pressure=" << pressure / 1e3
                     << " kPa  t=" << t << " s" << endl;
                next_progress_pct += 10;
            }
        }
    }
    csv.close();

    double force_end = sysFSI.GetFsiBodyForce(0).z();
    cout << "[demo] done: final sinkage=" << (soil_top - (plate->GetPos().z() - plate_thickness / 2))
         << " m  final pressure=" << std::abs(force_end) / plate_area / 1e3 << " kPa" << endl;
    cout << "[demo] pressure-sinkage curve written to " << out_dir << "pressure_sinkage.csv" << endl;

#ifdef CHRONO_POSTPROCESS
    // Plot bearing pressure vs sinkage (CSV columns 5 and 3: pressure in Pa, sinkage in m).
    postprocess::ChGnuPlot gplot(out_dir + "pressure_sinkage.gpl");
    gplot.SetGrid();
    gplot.SetTitle("CRM plate sinkage");
    gplot.SetLabelX("sinkage [m]");
    gplot.SetLabelY("bearing pressure [Pa]");
    gplot.Plot(out_dir + "pressure_sinkage.csv", 3, 5, "", " with lines lt -1 lw 2 lc rgb '#3333BB' ");
#endif

    return 0;
}
