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
// Author: Wei Hu, Huzaifa Mustafa Unjhawala
// =============================================================================
// Cone Penetration Validation Problem involving a cone falling from a height
// onto a soil surface.
// Reference solution:
// https://sbel.wisc.edu/documents/TR-2016-04.pdf
//
// =============================================================================

#include <cassert>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <iomanip>
#include <fstream>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"

#include "chrono_fsi/sph/ChFsiSystemSPH.h"
#include "chrono_fsi/sph/ChFsiFluidSystemSPH.h"

#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChSphVisualizationVSG.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

#ifdef CHRONO_VSG
// Run-time visualization system (OpenGL or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;
#endif
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
#ifdef CHRONO_VSG
class MarkerPositionVisibilityCallback : public ChSphVisualizationVSG::MarkerVisibilityCallback {
  public:
    MarkerPositionVisibilityCallback() {}
    virtual bool get(unsigned int n) const override { return pos[n].y > 0; }
};
#endif

struct sand_material {
    double density_max = 1780;
    double density_min = 1520;
    double Young_modulus = 2e6;
    double Poisson_ratio = 0.3;
    double mu_I0 = 0.04;
    double mu_fric_s = 0.7;
    double mu_fric_2 = 0.7;
    double average_diam = 0.0007;
    double cohesion_coeff = 0;
};

struct bead_material {
    double density_max = 1630;
    double density_min = 1500;
    double Young_modulus = 2e6;
    double Poisson_ratio = 0.3;
    double mu_I0 = 0.04;
    double mu_fric_s = 0.7;
    double mu_fric_2 = 0.7;
    double average_diam = 0.003;
    double cohesion_coeff = 0;
};

struct cone_60 {
    double length = 0.02210;
    double diameter = 0.01986;
    double mass = 0.1357;
};

struct cone_30 {
    double length = 0.03436;
    double diameter = 0.00921;
    double mass = 0.1411;
};

struct solid_material {
    double youngs_modulus = 193e9;
    double friction_coefficient = 0.7;
    double restitution = 0.05;
    double adhesion = 0;
};

// Function to handle CLI arguments
bool GetProblemSpecs(int argc,
                     char** argv,
                     double& t_end,
                     int& ps_freq,
                     double& initial_spacing,
                     double& d0_multiplier,
                     double& time_step,
                     std::string& boundary_type,
                     std::string& viscosity_type,
                     std::string& kernel_type,
                     std::string& gran_material,
                     int& rel_density,
                     int& cone_type,
                     double& container_depth,
                     double& Hdrop,
                     double& artificial_viscosity,
                     double& mu_s,
                     double& mu_2,
                     double& mu_i0,
                     std::string& rheology_model_crm,
                     double& pre_pressure_scale,
                     double& kappa,
                     double& lambda) {
    ChCLI cli(argv[0], "FSI Cone Penetration Demo");

    cli.AddOption<double>("Simulation", "t_end", "End time", std::to_string(t_end));
    cli.AddOption<int>("Simulation", "ps_freq", "Proximity search frequency", std::to_string(ps_freq));
    cli.AddOption<double>("Simulation", "initial_spacing", "Initial spacing", std::to_string(initial_spacing));
    cli.AddOption<double>("Simulation", "d0_multiplier", "D0 multiplier", std::to_string(d0_multiplier));
    cli.AddOption<std::string>("Simulation", "boundary_type", "Boundary condition type (holmes/adami)", "adami");
    cli.AddOption<std::string>("Simulation", "viscosity_type",
                               "Viscosity type (artificial_unilateral/artificial_bilateral)", viscosity_type);
    cli.AddOption<std::string>("Simulation", "kernel_type", "Kernel type (cubic/wendland)", kernel_type);
    cli.AddOption<double>("Simulation", "time_step", "Time step", std::to_string(time_step));

    cli.AddOption<std::string>("Physics", "gran_material", "Granular material type (sand/bead)", gran_material);
    cli.AddOption<int>("Geometry", "rel_density", "Relative density(0/1)",
                       std::to_string(rel_density));  // See linked document at top to see what this means
    cli.AddOption<int>("Geometry", "cone_type", "Cone type (1 - 30 degrees/2 - 60 degrees)", std::to_string(cone_type));
    cli.AddOption<double>("Geometry", "container_depth", "Container depth (m)", std::to_string(container_depth));
    cli.AddOption<double>("Geometry", "Hdrop", "Drop height (times cone length - 0/0.5/1)", std::to_string(Hdrop));
    cli.AddOption<double>("Physics", "artificial_viscosity", "Artificial viscosity",
                          std::to_string(artificial_viscosity));

    cli.AddOption<double>("Physics", "mu_s", "Static friction coefficient", std::to_string(mu_s));
    cli.AddOption<double>("Physics", "mu_2", "Dynamic friction coefficient", std::to_string(mu_2));
    cli.AddOption<double>("Physics", "mu_i0", "Initial friction coefficient", std::to_string(mu_i0));

    cli.AddOption<std::string>("Physics", "rheology_model_crm", "Rheology model (MU_OF_I/MCC)", rheology_model_crm);
    cli.AddOption<double>("Physics", "pre_pressure_scale", "Pre-pressure scale", std::to_string(pre_pressure_scale));
    cli.AddOption<double>("Physics", "kappa", "kappa", std::to_string(kappa));
    cli.AddOption<double>("Physics", "lambda", "lambda", std::to_string(lambda));
    if (!cli.Parse(argc, argv))
        return false;

    t_end = cli.GetAsType<double>("t_end");
    ps_freq = cli.GetAsType<int>("ps_freq");
    initial_spacing = cli.GetAsType<double>("initial_spacing");
    d0_multiplier = cli.GetAsType<double>("d0_multiplier");
    time_step = cli.GetAsType<double>("time_step");
    boundary_type = cli.GetAsType<std::string>("boundary_type");
    viscosity_type = cli.GetAsType<std::string>("viscosity_type");
    kernel_type = cli.GetAsType<std::string>("kernel_type");
    gran_material = cli.GetAsType<std::string>("gran_material");
    rel_density = cli.GetAsType<int>("rel_density");
    cone_type = cli.GetAsType<int>("cone_type");
    container_depth = cli.GetAsType<double>("container_depth");
    Hdrop = cli.GetAsType<double>("Hdrop");
    artificial_viscosity = cli.GetAsType<double>("artificial_viscosity");
    mu_s = cli.GetAsType<double>("mu_s");
    mu_2 = cli.GetAsType<double>("mu_2");
    mu_i0 = cli.GetAsType<double>("mu_i0");
    rheology_model_crm = cli.GetAsType<std::string>("rheology_model_crm");
    pre_pressure_scale = cli.GetAsType<double>("pre_pressure_scale");
    kappa = cli.GetAsType<double>("kappa");
    lambda = cli.GetAsType<double>("lambda");
    return true;
}

void CalculateConeProperties(const double length,
                             const double diameter,
                             const double mass,
                             double Hdrop,
                             double g,
                             double fzDim,
                             double initial_spacing,
                             double& volume,
                             double& cone_mass,
                             ChMatrix33<>& inertia,
                             double& cone_z_pos,
                             double& cone_z_vel,
                             double& cone_length,
                             double& cone_diameter) {
    volume = ChCone::CalcVolume(length, diameter);
    cone_mass = mass;
    inertia = mass * ChCone::CalcGyration(length, diameter);
    double impact_vel = std::sqrt(2 * Hdrop * length * g);
    cone_z_pos = fzDim + length + 0.5 * initial_spacing;
    cone_z_vel = impact_vel;
    cone_length = length;
    cone_diameter = diameter;
}

//------------------------------------------------------------------
// Function to generate a cone mesh and save to VTK file
//------------------------------------------------------------------
void WriteConeVTK(const std::string& filename,
                  std::shared_ptr<ChBody> body,
                  double radius,
                  double length,
                  int resolution = 16) {
    // Generate a cone mesh
    ChTriangleMeshConnected mesh;
    std::vector<ChVector3d>& vertices = mesh.GetCoordsVertices();
    std::vector<ChVector3i>& indices = mesh.GetIndicesVertexes();

    // Create local coordinate system that matches how the cone is created in the simulation
    // In the simulation, the cone is rotated with Q_FLIP_AROUND_X, so the apex points in -Z direction
    // Here we create the mesh with the apex pointing in +Z direction, and let the body's rotation handle the
    // orientation

    // Add the apex of the cone
    vertices.push_back(ChVector3d(0, 0, length));

    // Create vertices for the base of the cone
    for (int i = 0; i < resolution; i++) {
        double theta = 2 * CH_PI * i / resolution;
        double x = radius * cos(theta);
        double y = radius * sin(theta);
        double z = 0;
        vertices.push_back(ChVector3d(x, y, z));
    }

    // Create triangular faces
    for (int i = 0; i < resolution; i++) {
        int next = (i + 1) % resolution;
        indices.push_back(ChVector3i(0, i + 1, next + 1));  // Apex to base triangle
    }

    // Add base triangles (to close the cone if needed)
    int center_index = vertices.size();
    vertices.push_back(ChVector3d(0, 0, 0));  // Center of base
    for (int i = 0; i < resolution; i++) {
        int next = (i + 1) % resolution;
        indices.push_back(ChVector3i(center_index, next + 1, i + 1));  // Base triangles
    }

    // Now write the mesh to VTK file, transforming it to the body's position
    std::ofstream outf(filename);
    outf << "# vtk DataFile Version 2.0" << std::endl;
    outf << "Cone VTK from simulation" << std::endl;
    outf << "ASCII" << std::endl;
    outf << "DATASET UNSTRUCTURED_GRID" << std::endl;

    // Write vertices transformed by body frame
    ChFrame<> frame = body->GetFrameRefToAbs();
    outf << "POINTS " << vertices.size() << " float" << std::endl;
    for (const auto& v : vertices) {
        auto w = frame.TransformPointLocalToParent(v);
        outf << w.x() << " " << w.y() << " " << w.z() << std::endl;
    }

    // Write triangular cells
    int nf = static_cast<int>(indices.size());
    outf << "CELLS " << nf << " " << 4 * nf << std::endl;
    for (const auto& f : indices) {
        outf << "3 " << f.x() << " " << f.y() << " " << f.z() << std::endl;
    }

    // Write cell types (5 = VTK_TRIANGLE)
    outf << "CELL_TYPES " << nf << std::endl;
    for (int i = 0; i < nf; i++) {
        outf << "5" << std::endl;
    }
    outf.close();
}

int main(int argc, char* argv[]) {
    double t_end = 0.5;
    int ps_freq = 1;
    double initial_spacing = 0.001;
    double d0_multiplier = 1.3;
    double time_step = 2e-5;
    std::string boundary_type = "adami";
    std::string viscosity_type = "artificial_bilateral";
    std::string kernel_type = "cubic";

    bool verbose = true;
    bool output = true;
    double output_fps = 400;
    bool snapshots = false;
    bool render = true;
    double render_fps = 400;

    std::string gran_material = "sand";  // This can also be "bead"
    int rel_density = 0;                 // This means that the density is rho_min of measured - if 0, then its rho_max
    int cone_type = 1;                   // This means 30 deg cone
    double container_depth = 0.1;        // This is  in meters
    double Hdrop = 1.0;                  // This is 0.5 times length of cone
    double artificial_viscosity = 0.2;
    double mu_s = 0.80;
    double mu_2 = 1.00;
    double mu_i0 = 0.08;

    // If MCC rheology model is used, then these values are not used
    std::string rheology_model_crm = "MU_OF_I";
    double pre_pressure_scale = 2;
    double kappa = 0.01;
    double lambda = 0.04;
    if (!GetProblemSpecs(argc, argv, t_end, ps_freq, initial_spacing, d0_multiplier, time_step, boundary_type,
                         viscosity_type, kernel_type, gran_material, rel_density, cone_type, container_depth, Hdrop,
                         artificial_viscosity, mu_s, mu_2, mu_i0, rheology_model_crm, pre_pressure_scale, kappa,
                         lambda)) {
        return 1;
    }

    bool write_marker_files = false;
    // Print all problem specs
    std::cout << "Problem Specs:" << std::endl;
    std::cout << "t_end: " << t_end << std::endl;
    std::cout << "ps_freq: " << ps_freq << std::endl;
    std::cout << "initial_spacing: " << initial_spacing << std::endl;
    std::cout << "d0_multiplier: " << d0_multiplier << std::endl;
    std::cout << "time_step: " << time_step << std::endl;
    std::cout << "boundary_type: " << boundary_type << std::endl;
    std::cout << "viscosity_type: " << viscosity_type << std::endl;
    std::cout << "kernel_type: " << kernel_type << std::endl;
    std::cout << "gran_material: " << gran_material << std::endl;
    std::cout << "rel_density: " << rel_density << std::endl;
    std::cout << "cone_type: " << cone_type << std::endl;
    std::cout << "container_depth: " << container_depth << std::endl;
    std::cout << "Hdrop: " << Hdrop << std::endl;
    std::cout << "artificial_viscosity: " << artificial_viscosity << std::endl;
    std::cout << "mu_s: " << mu_s << std::endl;
    std::cout << "mu_2: " << mu_2 << std::endl;
    std::cout << "mu_i0: " << mu_i0 << std::endl;
    std::cout << "kappa: " << kappa << std::endl;
    std::cout << "lambda: " << lambda << std::endl;
    // Create a physics system
    ChSystemSMC sysMBS;

    // Create a fluid system
    ChFsiFluidSystemSPH sysSPH;
    sysSPH.SetOutputLevel(OutputLevel::CRM_FULL);
    // Create an FSI system
    ChFsiSystemSPH sysFSI(&sysMBS, &sysSPH);

    sysFSI.SetStepSizeCFD(time_step);
    sysFSI.SetStepsizeMBD(time_step);

    ChFsiFluidSystemSPH::ElasticMaterialProperties mat_props;
    ChFsiFluidSystemSPH::SPHParameters sph_params;

    if (gran_material == "sand") {
        sand_material sand_mat;
        mat_props.density = -(sand_mat.density_max - sand_mat.density_min) * rel_density + sand_mat.density_max;
        mat_props.Young_modulus = sand_mat.Young_modulus;
        mat_props.Poisson_ratio = sand_mat.Poisson_ratio;
        if (rheology_model_crm == "MU_OF_I") {
            mat_props.rheology_model = RheologyCRM::MU_OF_I;
            mat_props.mu_I0 = mu_i0;
            mat_props.mu_fric_s = mu_s;
            mat_props.mu_fric_2 = mu_2;
            mat_props.average_diam = sand_mat.average_diam;
            mat_props.cohesion_coeff = sand_mat.cohesion_coeff;  // default
        } else {
            mat_props.rheology_model = RheologyCRM::MCC;
            double angle_mus = std::atan(mu_s);
            mat_props.mcc_M = (6 * std::sin(angle_mus)) / (3 - std::sin(angle_mus));
            std::cout << "MCC M: " << mat_props.mcc_M << std::endl;
            // mat_props.mcc_M = 1.02;
            mat_props.mcc_kappa = kappa;
            mat_props.mcc_lambda = lambda;
        }
    } else if (gran_material == "bead") {
        bead_material bead_mat;
        mat_props.density = -(bead_mat.density_max - bead_mat.density_min) * rel_density + bead_mat.density_max;
        mat_props.Young_modulus = bead_mat.Young_modulus;
        mat_props.Poisson_ratio = bead_mat.Poisson_ratio;
        if (rheology_model_crm == "MU_OF_I") {
            mat_props.rheology_model = RheologyCRM::MU_OF_I;
            mat_props.mu_I0 = mu_i0;
            mat_props.mu_fric_s = mu_s;
            mat_props.mu_fric_2 = mu_2;
            mat_props.average_diam = bead_mat.average_diam;
            mat_props.cohesion_coeff = bead_mat.cohesion_coeff;  // default
        } else {
            mat_props.rheology_model = RheologyCRM::MCC;
            double angle_mus = std::atan(mu_s);
            mat_props.mcc_M = (6 * std::sin(angle_mus)) / (3 - std::sin(angle_mus));
            std::cout << "MCC M: " << mat_props.mcc_M << std::endl;
            // mat_props.mcc_M = 1.02;
            mat_props.mcc_kappa = kappa;
            mat_props.mcc_lambda = lambda;
        }
    } else {
        std::cerr << "Invalid gran_material: " << gran_material << std::endl;
        return 1;
    }
    sysSPH.SetElasticSPH(mat_props);
    sph_params.integration_scheme = IntegrationScheme::RK2;
    sph_params.initial_spacing = initial_spacing;
    sph_params.d0_multiplier = d0_multiplier;
    sph_params.artificial_viscosity = artificial_viscosity;
    sph_params.shifting_method = ShiftingMethod::PPST_XSPH;
    sph_params.shifting_xsph_eps = 0.5;
    sph_params.shifting_ppst_pull = 1.0;
    sph_params.shifting_ppst_push = 3.0;
    sph_params.free_surface_threshold = 2.0;
    sph_params.num_proximity_search_steps = ps_freq;
    if (kernel_type == "cubic") {
        sph_params.kernel_type = KernelType::CUBIC_SPLINE;
    } else if (kernel_type == "wendland") {
        sph_params.kernel_type = KernelType::WENDLAND;
    } else {
        std::cerr << "Invalid kernel type: " << kernel_type << std::endl;
        return 1;
    }

    // Set boundary type
    if (boundary_type == "holmes") {
        sph_params.boundary_method = BoundaryMethod::HOLMES;
    } else {
        sph_params.boundary_method = BoundaryMethod::ADAMI;
    }

    // Set viscosity type
    if (viscosity_type == "artificial_bilateral") {
        sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_BILATERAL;
    } else {
        sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_UNILATERAL;
    }
    sph_params.use_variable_time_step = true;
    sysSPH.SetSPHParameters(sph_params);

    double g = 9.81;
    sysSPH.SetGravitationalAcceleration(ChVector3d(0, 0, -g));
    sysMBS.SetGravitationalAcceleration(sysSPH.GetGravitationalAcceleration());

    sysFSI.SetVerbose(verbose);

    // ==============================
    // Create container and granular material
    // ==============================
    // Create a container
    // sand clearance
    double clearance = 0.2 * container_depth;
    double bxDim = 0.1;
    double byDim = 0.1;
    double fzDim = container_depth;
    double bzDim = fzDim + clearance;

    // Set the periodic boundary condition
    ChVector3d cMin(-bxDim / 2 * 1.2, -byDim / 2 * 1.2, -bzDim * 1.2);
    ChVector3d cMax(bxDim / 2 * 1.2, byDim / 2 * 1.2, (bzDim + 0.05 + initial_spacing) * 1.2);
    sysSPH.SetComputationalDomain(ChAABB(cMin, cMax), BC_NONE);

    auto box = chrono_types::make_shared<ChBody>();
    box->SetPos(ChVector3d(0.0, 0.0, 0.0));
    box->SetRot(ChQuaternion<>(1, 0, 0, 0));
    box->SetFixed(true);
    sysMBS.AddBody(box);

    // Create granular material
    // Create SPH particle locations using a regular grid sampler
    chrono::utils::ChGridSampler<> sampler(initial_spacing);
    ChVector3d boxCenter(0, 0, fzDim / 2);
    ChVector3d boxHalfDim(bxDim / 2 - initial_spacing, byDim / 2 - initial_spacing, fzDim / 2 - initial_spacing);
    std::vector<ChVector3d> points = sampler.SampleBox(boxCenter, boxHalfDim);

    // Add SPH particles to the fluid system
    double gz = std::abs(sysSPH.GetGravitationalAcceleration().z());
    for (const auto& p : points) {
        auto rho_ini = sysSPH.GetDensity();
        double pre_ini = rho_ini * gz * (-p.z() + fzDim);
        double preconsidation_pressure = pre_ini * pre_pressure_scale;
        // double pre_ini = sysSPH.GetDensity() * gz * (-p.z() + fzDim);
        // double rho_ini = sysSPH.GetDensity() + pre_ini / (sysSPH.GetSoundSpeed() * sysSPH.GetSoundSpeed());
        sysSPH.AddSPHParticle(p, rho_ini, pre_ini, sysSPH.GetViscosity(), ChVector3d(0),
                              ChVector3d(-pre_ini, -pre_ini, -pre_ini), ChVector3d(0, 0, 0), preconsidation_pressure);
    }

    // Set Material
    solid_material solid_mat;
    auto cmaterial = chrono_types::make_shared<ChContactMaterialSMC>();
    cmaterial->SetYoungModulus(solid_mat.youngs_modulus);
    cmaterial->SetFriction(solid_mat.friction_coefficient);
    cmaterial->SetRestitution(solid_mat.restitution);
    cmaterial->SetAdhesion(solid_mat.adhesion);

    // Add collision geometry for the container walls
    chrono::utils::AddBoxContainer(box, cmaterial,                                 //
                                   ChFrame<>(ChVector3d(0, 0, bzDim / 2), QUNIT),  //
                                   ChVector3d(bxDim, byDim, bzDim), 0.1,           //
                                   ChVector3i(2, 2, -1),                           //
                                   false);
    box->EnableCollision(false);

    // Add BCE particles attached on the walls into FSI system
    auto box_bce = sysSPH.CreatePointsBoxContainer(ChVector3d(bxDim, byDim, bzDim), ChVector3i(2, 2, -1));
    sysFSI.AddFsiBoundary(box_bce, ChFrame<>(ChVector3d(0, 0, bzDim / 2), QUNIT));

    // ==============================
    // Create cone
    // ==============================
    double volume;
    double mass;
    ChMatrix33<> inertia;
    double cone_z_pos, cone_z_vel, cone_length, cone_diameter;
    if (cone_type == 1) {
        cone_30 cone_30;
        CalculateConeProperties(cone_30.length, cone_30.diameter, cone_30.mass, Hdrop, g, fzDim, initial_spacing,
                                volume, mass, inertia, cone_z_pos, cone_z_vel, cone_length, cone_diameter);

    } else {
        cone_60 cone_60;
        CalculateConeProperties(cone_60.length, cone_60.diameter, cone_60.mass, Hdrop, g, fzDim, initial_spacing,
                                volume, mass, inertia, cone_z_pos, cone_z_vel, cone_length, cone_diameter);
    }
    auto vis_material = chrono_types::make_shared<ChVisualMaterial>();
    auto cone = chrono_types::make_shared<ChBody>();

    cone->SetPos(ChVector3d(0, 0, cone_z_pos));
    cone->SetPosDt(ChVector3d(0, 0, -cone_z_vel));
    cone->SetMass(mass);
    ChQuaternion<> cone_rot = Q_FLIP_AROUND_X;
    cone->SetRot(cone_rot);
    cone->SetInertia(inertia);
    sysMBS.AddBody(cone);
    chrono::utils::AddConeGeometry(cone.get(), cmaterial, cone_diameter / 2., cone_length,
                                   ChVector3d(0, 0, cone_length / 2), QUNIT, true, vis_material);
    cone->GetCollisionModel()->SetSafeMargin(initial_spacing);

    // Register cone as FSI body with explicit BCE points
    // We use the Truncated Cone to improve stability as the single point at the cone tip causes instability
    // This would make the cone a bit bigger than it actually is and thus to keep the cone profile the same, we reduce
    // the length
    // auto cone_bce = sysSPH.CreatePointsConeInterior(cone_diameter / 2, cone_length, true);
    int np_h = (int)std::round(cone_length / initial_spacing);
    double delta_h = cone_length / np_h;
    double cone_tip_radius = cone_diameter / 2 * delta_h / cone_length;
    auto cone_bce = sysSPH.CreatePointsTruncatedConeInterior(cone_diameter / 2, cone_tip_radius,
                                                             cone_length - initial_spacing, true);
    sysFSI.AddFsiBody(cone, cone_bce, ChFrame<>(VNULL, QUNIT), false);

    sysFSI.Initialize();

    // Output directories
    std::string out_dir;
    if (output || snapshots) {
        // Base output directory
        std::stringstream mu_params;
        mu_params << std::fixed << std::setprecision(2);
        mu_params << "FSI_ConePenetration_mu_s_" << mu_s << "_mu_2_" << mu_2 << "_mu_i0_" << mu_i0;

        std::string base_dir = GetChronoOutputPath() + mu_params.str() + "/";

        if (!filesystem::create_directory(filesystem::path(base_dir))) {
            std::cerr << "Error creating directory " << base_dir << std::endl;
            return 1;
        }

        // Create nested directories
        std::stringstream hdrop_stream;
        hdrop_stream << std::fixed << std::setprecision(1) << Hdrop;

        std::vector<std::string> subdirs = {"Hdrop_" + hdrop_stream.str(),
                                            "granMaterial_" + gran_material,
                                            "relDensity_" + std::to_string(rel_density),
                                            "coneType_" + std::to_string(cone_type),
                                            "boundaryType_" + boundary_type,
                                            "viscosityType_" + viscosity_type,
                                            "kernelType_" + kernel_type};

        for (const auto& subdir : subdirs) {
            base_dir += subdir + "/";
            if (!filesystem::create_directory(filesystem::path(base_dir))) {
                std::cerr << "Error creating directory " << base_dir << std::endl;
                return 1;
            }
        }

        // Add flat structure
        std::stringstream ss;
        ss << "ps_" << ps_freq;
        ss << "_s_" << initial_spacing;
        ss << "_d0_" << d0_multiplier;
        ss << "_t_" << time_step;
        ss << "_av_" << artificial_viscosity;
        if (rheology_model_crm == "MCC") {
            ss << "_pre_pressure_scale_" << pre_pressure_scale;
            ss << "_kappa_" << kappa;
            ss << "_lambda_" << lambda;
        }
        out_dir = base_dir + ss.str();

        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cerr << "Error creating directory " << out_dir << std::endl;
            return 1;
        }

        if (output) {
            if (!filesystem::create_directory(filesystem::path(out_dir + "/particles"))) {
                std::cerr << "Error creating directory " << out_dir + "/particles" << std::endl;
                return 1;
            }
            if (!filesystem::create_directory(filesystem::path(out_dir + "/fsi"))) {
                std::cerr << "Error creating directory " << out_dir + "/fsi" << std::endl;
                return 1;
            }
            if (!filesystem::create_directory(filesystem::path(out_dir + "/vtk"))) {
                std::cerr << "Error creating directory " << out_dir + "/vtk" << std::endl;
                return 1;
            }
        }

        if (snapshots) {
            if (!filesystem::create_directory(filesystem::path(out_dir + "/snapshots"))) {
                std::cerr << "Error creating directory " << out_dir + "/snapshots" << std::endl;
                return 1;
            }
        }
    }

    std::shared_ptr<ChVisualSystem> vis;
    // Create a run-time visualizer
#ifdef CHRONO_VSG
    auto col_callback = chrono_types::make_shared<ParticleVelocityColorCallback>(0, 2);
    if (render) {
        auto visFSI = chrono_types::make_shared<ChSphVisualizationVSG>(&sysFSI);
        visFSI->EnableFluidMarkers(true);
        visFSI->EnableBoundaryMarkers(true);
        visFSI->EnableRigidBodyMarkers(true);
        visFSI->SetSPHColorCallback(col_callback);
        visFSI->SetSPHVisibilityCallback(chrono_types::make_shared<MarkerPositionVisibilityCallback>());
        visFSI->SetBCEVisibilityCallback(chrono_types::make_shared<MarkerPositionVisibilityCallback>());

        auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        visVSG->AttachPlugin(visFSI);
        visVSG->AttachSystem(&sysMBS);
        visVSG->SetWindowTitle("FSI Cone Penetration");
        visVSG->SetWindowSize(1280, 720);
        visVSG->SetWindowPosition(100, 100);
        visVSG->AddCamera(ChVector3d(0, -3 * byDim, 0.75 * bzDim), ChVector3d(0, 0, 0.75 * bzDim));
        visVSG->SetLightIntensity(0.9);
        visVSG->SetLightDirection(-CH_PI_2, CH_PI / 6);

        visVSG->Initialize();
        vis = visVSG;
    }
#else
    render = false;
#endif
    // Start the simulation
    double time = 0.0;
    int sim_frame = 0;
    int out_frame = 0;
    int render_frame = 0;
    double dT = sysFSI.GetStepSizeCFD();

    double rtf_average = 0.0;
    unsigned int rtf_count = 0;

    std::string out_file = out_dir + "/cone_penetration_depth.txt";
    std::ofstream ofile(out_file, std::ios::trunc);

    // Add comma-separated header to the output file
    ofile << "Time,PenetrationDepth,ConePosX,ConePosY,ConePosZ,ConeVelX,ConeVelY,ConeVelZ" << std::endl;

    ChTimer timer;
    timer.start();
    while (time < t_end) {
        if (output && time >= out_frame / output_fps) {
            if (write_marker_files) {
                sysSPH.SaveParticleData(out_dir + "/particles");
                sysSPH.SaveSolidData(out_dir + "/fsi", time);
                // Output cone VTK file
                std::stringstream vtk_filename;
                vtk_filename << out_dir << "/vtk/cone_" << std::setw(5) << std::setfill('0') << out_frame << ".vtk";
                WriteConeVTK(vtk_filename.str(), cone, cone_diameter / 2, cone_length);
            }
            std::cout << " -- Output frame " << out_frame << " at t = " << time << std::endl;
            out_frame++;
        }
#ifdef CHRONO_VSG
        if (render && time >= render_frame / render_fps) {
            if (!vis->Run())
                break;
            vis->Render();

            if (snapshots) {
                std::cout << " -- Snapshot frame " << render_frame << " at t = " << time << std::endl;
                std::ostringstream filename;
                filename << out_dir << "/snapshots/" << std::setw(5) << std::setfill('0') << render_frame << ".jpg";
                vis->WriteImageToFile(filename.str());
            }

            render_frame++;
        }
#endif
        // Write penetration depth to file
        // This is done so as to match the penetration depth in the experiment - see
        // https://sbel.wisc.edu/documents/TR-2016-04.pdf
        double d_pen = fzDim + 0.5 * initial_spacing + cone_length - cone->GetPos().z();
        ofile << time << "," << d_pen << "," << cone->GetPos().x() << "," << cone->GetPos().y() << ","
              << cone->GetPos().z() << "," << cone->GetPosDt().x() << "," << cone->GetPosDt().y() << ","
              << cone->GetPosDt().z() << std::endl;
        // Advance simulation for one timestep for all systems
        sysFSI.DoStepDynamics(dT);
        double rtf = sysFSI.GetRtf();
        rtf_average = (rtf_average * rtf_count + rtf) / (rtf_count + 1);
        rtf_count++;
        time += dT;

        sim_frame++;
    }
    timer.stop();
    std::cout << "End Time: " << t_end << std::endl;
    std::cout << "\nSimulation time: " << timer() << " seconds\n" << std::endl;
    std::cout << "Average RTF: " << rtf_average << std::endl;
    std::cout << "Simulation finished" << std::endl;

    // Write runtime information to a file
    if (output) {
        std::ofstream runtime_file(out_dir + "/runtime.txt");
        runtime_file << "Runtime: " << timer() << " seconds\n" << std::endl;
        runtime_file << "Simulation time: " << time << std::endl;
        runtime_file << "Average RTF: " << rtf_average << std::endl;
        runtime_file << "ps_freq: " << ps_freq << std::endl;
        runtime_file << "Simulation finished";
        runtime_file.close();
    }

    return 0;
}
