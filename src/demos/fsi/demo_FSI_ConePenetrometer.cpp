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
// Author: Huzaifa Mustafa Unjhawala
// =============================================================================
// Cone Penetrometer Validation Problem involving immersing a cone at a specified velocity
// and measureing the force on the cone tip
// Thesis reference for similar test in GRC-1
// https://uwmadison.box.com/s/606t8ppn48kpp5y53c6wp5tftjm27sfb - Page 58
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
#include "chrono/physics/ChLinkMotorLinearSpeed.h"
#include "chrono_fsi/sph/ChFsiSystemSPH.h"
#include "chrono_fsi/sph/ChFsiFluidSystemSPH.h"
// #include "chrono_fsi/sph/ChFsiProblemSPH.h"

#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChSphVisualizationVSG.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;
// -----------------------------------------------------------------------------

#ifdef CHRONO_VSG
class MarkerPositionVisibilityCallback : public ChSphVisualizationVSG::MarkerVisibilityCallback {
  public:
    MarkerPositionVisibilityCallback() {}
    virtual bool get(unsigned int n) const override { return pos[n].y > 0; }
};
#endif
// -----------------------------------------------------------------------------
// Material properties of the 15 materials
// std::vector<double> y_modulus = {1e6, 1e6};  // in Pa
// std::vector<double> y_modulus = {1e6};  // in Pa
double nu_poisson = 0.3;
// std::vector<double> cohesions = {0};  // In Pa
// // std::vector<double> densities = {1600, 1800};  // In kg/m^3
// std::vector<double> densities = {1600};  // In kg/m^3
// // std::vector<double> mu_s = {0.5727, 0.9131};
// // std::vector<double> mu_2 = {0.5727, 0.9131};
// std::vector<double> mu_s = {0.5727};
// std::vector<double> mu_2 = {0.5727};

// Cone material
struct solid_material {
    double youngs_modulus = 193e9;
    double friction_coefficient = 0.7;
    double density = 7.8e3;
    double restitution = 0.05;
    double adhesion = 0;
};

// Add cone properties struct near material properties
struct ConeProperties {
    double surface_area = 323e-6;  // 323 mm^2
    double diameter = sqrt(surface_area * 4 / CH_PI);
    double length = 0.01756;  // 60 deg tip
    double density = 7.8e3;
};

// -----------------------------------------------------------------------------

struct SimParams {
    // Simulation parameters
    int ps_freq;
    double initial_spacing;
    double d0_multiplier;
    double time_step;
    std::string boundary_type;
    std::string viscosity_type;
    std::string kernel_type;

    // Physics parameters
    double artificial_viscosity;
    double penetration_depth;
    double container_height;

    // Output/rendering parameters
    bool verbose;
    bool output;
    double output_fps;
    bool snapshots;
    bool render;
    double render_fps;
    bool write_marker_files;
    // Need to play around with these too
    double mu_s;
    double mu_2;
    double cohesion;
    double density;
    double y_modulus;
    std::string integration_scheme;
    std::string rheology_model_crm;
    double pre_pressure_scale;
};
void SimulateMaterial(int i, const SimParams& params, const ConeProperties& coneProp);

//------------------------------------------------------------------
// Function to generate a cone mesh and save to VTK file
//------------------------------------------------------------------
void WriteConeVTK(const std::string& filename,
                  std::shared_ptr<ChBody> body,
                  double radius,
                  double height,
                  int resolution = 16) {
    // Generate a cone mesh
    ChTriangleMeshConnected mesh;
    std::vector<ChVector3d>& vertices = mesh.GetCoordsVertices();
    std::vector<ChVector3i>& indices = mesh.GetIndicesVertexes();

    // Add tip vertex
    vertices.push_back(ChVector3d(0, 0, -height));

    // Add base vertices in circular pattern
    for (int i = 0; i < resolution; i++) {
        double theta = 2 * CH_PI * i / resolution;
        double x = radius * cos(theta);
        double y = radius * sin(theta);
        vertices.push_back(ChVector3d(x, y, 0));
    }

    // Create triangular faces for the cone
    for (int i = 0; i < resolution; i++) {
        int next = (i + 1) % resolution + 1;  // +1 because index 0 is the tip
        indices.push_back(ChVector3i(0, i + 1, next));
    }

    // Add base of cone (as triangular fan)
    // First add center of base
    vertices.push_back(ChVector3d(0, 0, 0));
    int center_idx = vertices.size() - 1;

    // Create triangles for the base
    for (int i = 0; i < resolution; i++) {
        int current = i + 1;  // +1 because index 0 is the tip
        int next = (i + 1) % resolution + 1;
        indices.push_back(ChVector3i(center_idx, next, current));
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

//------------------------------------------------------------------
// Function to generate a cylinder mesh and save to VTK file
//------------------------------------------------------------------
void WriteCylinderVTK(const std::string& filename,
                      std::shared_ptr<ChBody> body,
                      double radius,
                      double height,
                      int resolution = 16) {
    // Generate a cylinder mesh
    ChTriangleMeshConnected mesh;
    std::vector<ChVector3d>& vertices = mesh.GetCoordsVertices();
    std::vector<ChVector3i>& indices = mesh.GetIndicesVertexes();

    // Create top and bottom circular caps
    for (int cap = 0; cap < 2; cap++) {
        double z = (cap == 0) ? 0 : height;

        // Add center vertex for the cap
        int center_idx = vertices.size();
        vertices.push_back(ChVector3d(0, 0, z));

        // Add vertices around the cap
        int first_idx = vertices.size();
        for (int i = 0; i < resolution; i++) {
            double theta = 2 * CH_PI * i / resolution;
            double x = radius * cos(theta);
            double y = radius * sin(theta);
            vertices.push_back(ChVector3d(x, y, z));
        }

        // Create triangles for the cap
        for (int i = 0; i < resolution; i++) {
            int current = first_idx + i;
            int next = first_idx + (i + 1) % resolution;
            if (cap == 0) {  // Bottom cap (counter-clockwise)
                indices.push_back(ChVector3i(center_idx, next, current));
            } else {  // Top cap (clockwise)
                indices.push_back(ChVector3i(center_idx, current, next));
            }
        }
    }

    // Create the side of the cylinder
    int bottom_start = 1;                           // Index of first vertex on bottom cap
    int top_start = bottom_start + resolution + 1;  // Index of first vertex on top cap

    for (int i = 0; i < resolution; i++) {
        int i1 = bottom_start + i;
        int i2 = bottom_start + (i + 1) % resolution;
        int i3 = top_start + i;
        int i4 = top_start + (i + 1) % resolution;

        // Add two triangles for each quad on the side
        indices.push_back(ChVector3i(i1, i2, i3));
        indices.push_back(ChVector3i(i3, i2, i4));
    }

    // Now write the mesh to VTK file, transforming it to the body's position
    std::ofstream outf(filename);
    outf << "# vtk DataFile Version 2.0" << std::endl;
    outf << "Cylinder VTK from simulation" << std::endl;
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

//------------------------------------------------------------------
// Function to write a combined VTK with both cone and cylinder
//------------------------------------------------------------------
void WriteConePenetrometerVTK(const std::string& filename,
                              std::shared_ptr<ChBody> cone,
                              double cone_radius,
                              double cone_height,
                              std::shared_ptr<ChBody> cylinder,
                              double cyl_radius,
                              double cyl_height,
                              int resolution = 16) {
    // Generate meshes for both cone and cylinder
    ChTriangleMeshConnected cone_mesh;
    std::vector<ChVector3d>& cone_vertices = cone_mesh.GetCoordsVertices();
    std::vector<ChVector3i>& cone_indices = cone_mesh.GetIndicesVertexes();

    ChTriangleMeshConnected cyl_mesh;
    std::vector<ChVector3d>& cyl_vertices = cyl_mesh.GetCoordsVertices();
    std::vector<ChVector3i>& cyl_indices = cyl_mesh.GetIndicesVertexes();

    // Create cone mesh
    // Add tip vertex
    cone_vertices.push_back(ChVector3d(0, 0, -cone_height));

    // Add base vertices in circular pattern
    for (int i = 0; i < resolution; i++) {
        double theta = 2 * CH_PI * i / resolution;
        double x = cone_radius * cos(theta);
        double y = cone_radius * sin(theta);
        cone_vertices.push_back(ChVector3d(x, y, 0));
    }

    // Create triangular faces for the cone
    for (int i = 0; i < resolution; i++) {
        int next = (i + 1) % resolution + 1;  // +1 because index 0 is the tip
        cone_indices.push_back(ChVector3i(0, i + 1, next));
    }

    // Add base of cone (as triangular fan)
    cone_vertices.push_back(ChVector3d(0, 0, 0));
    int center_idx = cone_vertices.size() - 1;

    // Create triangles for the base
    for (int i = 0; i < resolution; i++) {
        int current = i + 1;  // +1 because index 0 is the tip
        int next = (i + 1) % resolution + 1;
        cone_indices.push_back(ChVector3i(center_idx, next, current));
    }

    // Create cylinder mesh
    // Create top and bottom circular caps
    for (int cap = 0; cap < 2; cap++) {
        double z = (cap == 0) ? 0 : cyl_height;

        // Add center vertex for the cap
        int center_idx = cyl_vertices.size();
        cyl_vertices.push_back(ChVector3d(0, 0, z));

        // Add vertices around the cap
        int first_idx = cyl_vertices.size();
        for (int i = 0; i < resolution; i++) {
            double theta = 2 * CH_PI * i / resolution;
            double x = cyl_radius * cos(theta);
            double y = cyl_radius * sin(theta);
            cyl_vertices.push_back(ChVector3d(x, y, z));
        }

        // Create triangles for the cap
        for (int i = 0; i < resolution; i++) {
            int current = first_idx + i;
            int next = first_idx + (i + 1) % resolution;
            if (cap == 0) {  // Bottom cap (counter-clockwise)
                cyl_indices.push_back(ChVector3i(center_idx, next, current));
            } else {  // Top cap (clockwise)
                cyl_indices.push_back(ChVector3i(center_idx, current, next));
            }
        }
    }

    // Create the side of the cylinder
    int bottom_start = 1;                           // Index of first vertex on bottom cap
    int top_start = bottom_start + resolution + 1;  // Index of first vertex on top cap

    for (int i = 0; i < resolution; i++) {
        int i1 = bottom_start + i;
        int i2 = bottom_start + (i + 1) % resolution;
        int i3 = top_start + i;
        int i4 = top_start + (i + 1) % resolution;

        // Add two triangles for each quad on the side
        cyl_indices.push_back(ChVector3i(i1, i2, i3));
        cyl_indices.push_back(ChVector3i(i3, i2, i4));
    }

    // Now write the combined mesh to VTK file, transforming to respective body frames
    std::ofstream outf(filename);
    outf << "# vtk DataFile Version 2.0" << std::endl;
    outf << "Cone Penetrometer VTK from simulation" << std::endl;
    outf << "ASCII" << std::endl;
    outf << "DATASET UNSTRUCTURED_GRID" << std::endl;

    // Calculate the total number of vertices and triangles
    int total_vertices = cone_vertices.size() + cyl_vertices.size();
    int total_triangles = cone_indices.size() + cyl_indices.size();

    // Write all vertices
    outf << "POINTS " << total_vertices << " float" << std::endl;

    // Write cone vertices transformed by cone frame
    ChFrame<> cone_frame = cone->GetFrameRefToAbs();
    for (const auto& v : cone_vertices) {
        auto w = cone_frame.TransformPointLocalToParent(v);
        outf << w.x() << " " << w.y() << " " << w.z() << std::endl;
    }

    // Write cylinder vertices transformed by cylinder frame
    ChFrame<> cyl_frame = cylinder->GetFrameRefToAbs();
    for (const auto& v : cyl_vertices) {
        auto w = cyl_frame.TransformPointLocalToParent(v);
        outf << w.x() << " " << w.y() << " " << w.z() << std::endl;
    }

    // Write all triangular cells
    outf << "CELLS " << total_triangles << " " << 4 * total_triangles << std::endl;

    // Write cone triangles
    for (const auto& f : cone_indices) {
        outf << "3 " << f.x() << " " << f.y() << " " << f.z() << std::endl;
    }

    // Write cylinder triangles (adjusting indices to account for cone vertices)
    int offset = cone_vertices.size();
    for (const auto& f : cyl_indices) {
        outf << "3 " << (f.x() + offset) << " " << (f.y() + offset) << " " << (f.z() + offset) << std::endl;
    }

    // Write cell types (5 = VTK_TRIANGLE)
    outf << "CELL_TYPES " << total_triangles << std::endl;
    for (int i = 0; i < total_triangles; i++) {
        outf << "5" << std::endl;
    }

    // Add scalar data to identify parts
    outf << "CELL_DATA " << total_triangles << std::endl;
    outf << "SCALARS part int 1" << std::endl;
    outf << "LOOKUP_TABLE default" << std::endl;

    // Write 0 for cone triangles and 1 for cylinder triangles
    for (size_t i = 0; i < cone_indices.size(); i++) {
        outf << "0" << std::endl;
    }
    for (size_t i = 0; i < cyl_indices.size(); i++) {
        outf << "1" << std::endl;
    }

    outf.close();
}

// Function to handle CLI arguments
bool GetProblemSpecs(int argc, char** argv, SimParams& params) {
    ChCLI cli(argv[0], "FSI Cone Penetrometer Demo");

    cli.AddOption<int>("Simulation", "ps_freq", "Proximity search frequency", std::to_string(params.ps_freq));
    cli.AddOption<double>("Simulation", "initial_spacing", "Initial spacing", std::to_string(params.initial_spacing));
    cli.AddOption<double>("Simulation", "d0_multiplier", "D0 multiplier", std::to_string(params.d0_multiplier));
    cli.AddOption<std::string>("Simulation", "boundary_type", "Boundary condition type (holmes/adami)",
                               params.boundary_type);
    cli.AddOption<std::string>("Simulation", "viscosity_type",
                               "Viscosity type (artificial_unilateral/artificial_bilateral)", params.viscosity_type);
    cli.AddOption<std::string>("Simulation", "kernel_type", "Kernel type (cubic/wendland)", params.kernel_type);
    cli.AddOption<double>("Simulation", "time_step", "Time step", std::to_string(params.time_step));

    cli.AddOption<double>("Physics", "artificial_viscosity", "Artificial viscosity",
                          std::to_string(params.artificial_viscosity));
    cli.AddOption<double>("Physics", "penetration_depth",
                          "Penetration depth at which we would like to take the readings",
                          std::to_string(params.penetration_depth));
    cli.AddOption<double>("Physics", "container_height", "Container height", std::to_string(params.container_height));
    cli.AddOption<double>("Physics", "mu_s", "Friction coefficient", std::to_string(params.mu_s));
    cli.AddOption<double>("Physics", "mu_2", "Friction coefficient", std::to_string(params.mu_2));
    cli.AddOption<double>("Physics", "cohesion", "Cohesion", std::to_string(params.cohesion));
    cli.AddOption<double>("Physics", "density", "Density", std::to_string(params.density));
    cli.AddOption<double>("Physics", "y_modulus", "Young's modulus", std::to_string(params.y_modulus));
    cli.AddOption<std::string>("Physics", "integration_scheme", "Integration scheme (euler/rk2)",
                               params.integration_scheme);
    cli.AddOption<std::string>("Physics", "rheology_model_crm", "Rheology model (MU_OF_I/MCC)",
                               params.rheology_model_crm);
    cli.AddOption<double>("Physics", "pre_pressure_scale", "Pre-pressure scale",
                          std::to_string(params.pre_pressure_scale));
    if (!cli.Parse(argc, argv))
        return false;

    params.ps_freq = cli.GetAsType<int>("ps_freq");
    params.initial_spacing = cli.GetAsType<double>("initial_spacing");
    params.d0_multiplier = cli.GetAsType<double>("d0_multiplier");
    params.time_step = cli.GetAsType<double>("time_step");
    params.boundary_type = cli.GetAsType<std::string>("boundary_type");
    params.viscosity_type = cli.GetAsType<std::string>("viscosity_type");
    params.kernel_type = cli.GetAsType<std::string>("kernel_type");
    params.artificial_viscosity = cli.GetAsType<double>("artificial_viscosity");
    params.penetration_depth = cli.GetAsType<double>("penetration_depth");
    params.container_height = cli.GetAsType<double>("container_height");
    params.mu_s = cli.GetAsType<double>("mu_s");
    params.mu_2 = cli.GetAsType<double>("mu_2");
    params.cohesion = cli.GetAsType<double>("cohesion");
    params.density = cli.GetAsType<double>("density");
    params.y_modulus = cli.GetAsType<double>("y_modulus");
    params.integration_scheme = cli.GetAsType<std::string>("integration_scheme");
    params.rheology_model_crm = cli.GetAsType<std::string>("rheology_model_crm");
    params.pre_pressure_scale = cli.GetAsType<double>("pre_pressure_scale");
    return true;
}

int main(int argc, char* argv[]) {
    ConeProperties coneProp;  // Create cone instance

    SimParams params = {/*ps_freq*/ 1,
                        /*initial_spacing*/ 0.002,
                        /*d0_multiplier*/ 1.3,
                        /*time_step*/ 2e-5,
                        /*boundary_type*/ "adami",
                        /*viscosity_type*/ "artificial_bilateral",
                        /*kernel_type*/ "wendland",
                        /*artificial_viscosity*/ 0.2,
                        /*penetration_depth*/ 0.18,  // 18 cm is the max depth
                        /*container_height*/ 0.24,
                        /*verbose*/ true,
                        /*output*/ true,
                        /*output_fps*/ 20,
                        /*snapshots*/ true,
                        /*render*/ true,
                        /*render_fps*/ 200,
                        /*write_marker_files*/ false,
                        /*mu_s*/ 0.67,
                        /*mu_2*/ 0.67,
                        /*cohesions*/ 0,
                        /*densities*/ 1600,
                        /*y_modulus*/ 1e6,
                        /*integration_scheme*/ "rk2",
                        /*rheology_model_crm*/ "MU_OF_I",
                        /*pre_pressure_scale*/ 5000};

    if (!GetProblemSpecs(argc, argv, params)) {
        return 1;
    }

    std::cout << "Problem Specs:" << std::endl;

    std::cout << "ps_freq: " << params.ps_freq << std::endl;
    std::cout << "initial_spacing: " << params.initial_spacing << std::endl;
    std::cout << "d0_multiplier: " << params.d0_multiplier << std::endl;
    std::cout << "time_step: " << params.time_step << std::endl;
    std::cout << "boundary_type: " << params.boundary_type << std::endl;
    std::cout << "viscosity_type: " << params.viscosity_type << std::endl;
    std::cout << "kernel_type: " << params.kernel_type << std::endl;
    std::cout << "artificial_viscosity: " << params.artificial_viscosity << std::endl;
    std::cout << "penetration_depth: " << params.penetration_depth << std::endl;
    std::cout << "container_height: " << params.container_height << std::endl;
    std::cout << "mu_s: " << params.mu_s << std::endl;
    std::cout << "mu_2: " << params.mu_2 << std::endl;
    std::cout << "cohesion: " << params.cohesion << std::endl;
    std::cout << "density: " << params.density << std::endl;
    std::cout << "y_modulus: " << params.y_modulus << std::endl;
    std::cout << "integration_scheme: " << params.integration_scheme << std::endl;
    std::cout << "rheology_model_crm: " << params.rheology_model_crm << std::endl;
    std::cout << "pre_pressure_scale: " << params.pre_pressure_scale << std::endl;

    int num_materials = 1;
    for (int i = 0; i < num_materials; i++) {
        SimulateMaterial(i, params, coneProp);
    }
}

void SimulateMaterial(int i, const SimParams& params, const ConeProperties& coneProp) {
    double penetration_velocity = 0.03;  // 0.3 cm/s
    double t_end = params.penetration_depth / penetration_velocity;
    std::cout << "t_end: " << t_end << std::endl;

    double container_diameter = 0.3;                    // container diameter (m)
    double container_height = params.container_height;  // configurable via CLI
    double cyl_length = 0.2;

    // Create a physics system
    ChSystemSMC sysMBS;

    // Create a fluid system
    ChFsiFluidSystemSPH sysSPH;
    // Create an FSI system
    ChFsiSystemSPH sysFSI(&sysMBS, &sysSPH);
    // Set gravitational acceleration
    const ChVector3d gravity(0, 0, -9.81);
    sysSPH.SetGravitationalAcceleration(gravity);
    sysMBS.SetGravitationalAcceleration(gravity);

    sysFSI.SetStepSizeCFD(params.time_step);
    sysFSI.SetStepsizeMBD(params.time_step);

    sysSPH.SetVerbose(true);

    // -------------------------------------------------------------------------
    // Material and SPH parameters
    ChFsiFluidSystemSPH::ElasticMaterialProperties mat_props;
    ChFsiFluidSystemSPH::SPHParameters sph_params;

    mat_props.density = params.density;
    mat_props.Young_modulus = params.y_modulus;
    mat_props.Poisson_ratio = nu_poisson;
    if (params.rheology_model_crm == "MU_OF_I") {
        mat_props.rheology_model = RheologyCRM::MU_OF_I;
        mat_props.mu_I0 = 0.04;
        mat_props.mu_fric_s = params.mu_s;
        mat_props.mu_fric_2 = params.mu_2;
        mat_props.average_diam = 0.0002;
        mat_props.cohesion_coeff = params.cohesion;
    } else {
        mat_props.rheology_model = RheologyCRM::MCC;
        double angle_mus = std::atan(params.mu_s);
        mat_props.mcc_M = (6 * std::sin(angle_mus)) / (3 - std::sin(angle_mus));
        // mat_props.mcc_M = 1.34;
        mat_props.mcc_kappa = 0.0125;
        mat_props.mcc_lambda = 0.075;
    }

    sysSPH.SetElasticSPH(mat_props);
    if (params.integration_scheme == "euler") {
        sph_params.integration_scheme = IntegrationScheme::EULER;
    } else if (params.integration_scheme == "rk2") {
        sph_params.integration_scheme = IntegrationScheme::RK2;
    } else if (params.integration_scheme == "verlet") {
        sph_params.integration_scheme = IntegrationScheme::VERLET;
    } else if (params.integration_scheme == "symplectic") {
        sph_params.integration_scheme = IntegrationScheme::SYMPLECTIC;
    }
    sph_params.initial_spacing = params.initial_spacing;
    sph_params.num_bce_layers = 3;
    sph_params.d0_multiplier = params.d0_multiplier;
    sph_params.artificial_viscosity = params.artificial_viscosity;
    sph_params.shifting_method = ShiftingMethod::NONE;
    sph_params.shifting_xsph_eps = 0.5;
    sph_params.shifting_ppst_pull = 1.0;
    sph_params.shifting_ppst_push = 3.0;
    sph_params.free_surface_threshold = 2.0;
    sph_params.num_proximity_search_steps = params.ps_freq;

    if (params.kernel_type == "cubic") {
        sph_params.kernel_type = KernelType::CUBIC_SPLINE;
    } else if (params.kernel_type == "wendland") {
        sph_params.kernel_type = KernelType::WENDLAND;
    }
    if (params.boundary_type == "holmes") {
        sph_params.boundary_method = BoundaryMethod::HOLMES;
    } else {
        sph_params.boundary_method = BoundaryMethod::ADAMI;
    }

    // Set viscosity type
    if (params.viscosity_type == "artificial_bilateral") {
        sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_BILATERAL;
    } else {
        sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_UNILATERAL;
    }

    sysSPH.SetSPHParameters(sph_params);
    // -------------------------------------------------------------------------

    // ==============================
    // Create container and granular material
    // ==============================
    // Create a container
    // sand clearance
    double clearance = 0.2 * container_height;
    double bxDim = container_diameter;
    double byDim = container_diameter;
    double fzDim = container_height;
    double bzDim = fzDim + clearance;

    // Set the periodic boundary condition
    ChVector3d cMin(-bxDim / 2 * 1.2, -byDim / 2 * 1.2, -bzDim * 1.2);
    ChVector3d cMax(bxDim / 2 * 1.2, byDim / 2 * 1.2, (bzDim + std::max(bzDim * 1.2, cyl_length)));
    sysSPH.SetComputationalDomain(ChAABB(cMin, cMax), BC_NONE);

    auto box = chrono_types::make_shared<ChBody>();
    box->SetPos(ChVector3d(0.0, 0.0, 0.0));
    box->SetRot(ChQuaternion<>(1, 0, 0, 0));
    box->SetFixed(true);
    sysMBS.AddBody(box);

    // Create granular material
    // Create SPH particle locations using a regular grid sampler
    chrono::utils::ChGridSampler<> sampler(params.initial_spacing);
    ChVector3d boxCenter(0, 0, fzDim / 2);
    ChVector3d boxHalfDim(bxDim / 2 - params.initial_spacing, byDim / 2 - params.initial_spacing,
                          fzDim / 2 - params.initial_spacing);
    std::vector<ChVector3d> points = sampler.SampleBox(boxCenter, boxHalfDim);

    double gz = std::abs(sysSPH.GetGravitationalAcceleration().z());
    for (const auto& p : points) {
        // double depth_cm = (-p.z() + fzDim) * 100;
        // double b = 12.2;
        // double c = 18;
        // double fzDim_cm = fzDim * 100;
        // double g = (depth_cm + b) / (depth_cm + c);
        // double gbar = 1.0 - ((c - b) / fzDim_cm) * std::log((c + fzDim_cm) / c);
        // double density_mean_target = params.density;
        // double rho_ini = density_mean_target * g / gbar;
        auto rho_ini = params.density;
        double pre_ini = rho_ini * gz * (-p.z() + fzDim);
        double preconsidation_pressure = pre_ini * params.pre_pressure_scale;
        sysSPH.AddSPHParticle(p, rho_ini, pre_ini, sysSPH.GetViscosity(), ChVector3d(0),
                              ChVector3d(-pre_ini, -pre_ini, -pre_ini), ChVector3d(0, 0, 0), preconsidation_pressure);
    }

    solid_material solid_mat;
    auto cmaterial = chrono_types::make_shared<ChContactMaterialSMC>();
    auto vis_material = chrono_types::make_shared<ChVisualMaterial>();
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
    // Add BCE particles attached on the walls into FSI system (new API)
    auto box_bce = sysSPH.CreatePointsBoxContainer(ChVector3d(bxDim, byDim, bzDim), ChVector3i(2, 2, -1));
    sysFSI.AddFsiBoundary(box_bce, ChFrame<>(ChVector3d(0, 0, bzDim / 2), QUNIT));

    // ==============================
    // Create cone
    // ==============================

    auto cone = chrono_types::make_shared<ChBody>();
    double cone_z_pos = bzDim - clearance + coneProp.length;
    cone->SetPos(ChVector3d(0, 0, cone_z_pos));

    ChQuaternion<> cone_rot = Q_FLIP_AROUND_X;

    cone->SetRot(cone_rot);

    double volume = ChCone::CalcVolume(coneProp.diameter / 2, coneProp.length);
    double cone_mass = coneProp.density * volume;
    std::cout << "cone_mass: " << cone_mass << std::endl;
    cone->SetMass(cone_mass / 2);
    // Mass/2 because we also have a cylinder that takes up half the mass
    ChMatrix33<> inertia = cone_mass / 2 * ChCone::CalcGyration(coneProp.diameter / 2, coneProp.length);
    cone->SetInertia(inertia);
    cone->SetFixed(false);
    sysMBS.AddBody(cone);

    chrono::utils::AddConeGeometry(cone.get(), cmaterial, coneProp.diameter / 2., coneProp.length,
                                   ChVector3d(0, 0, coneProp.length / 2), QUNIT, false, vis_material);
    cone->GetCollisionModel()->SetSafeMargin(params.initial_spacing);

    // Register cone as FSI body with explicit BCE points
    // We use the Truncated Cone to improve stability as the single point at the cone tip causes instability
    // This would make the cone a bit bigger than it actually is and thus to keep the cone profile the same, we reduce
    // the length
    // auto cone_bce = sysSPH.CreatePointsConeInterior(coneProp.diameter / 2, coneProp.length, true);
    int np_h = (int)std::round(coneProp.length / params.initial_spacing);
    double delta_h = coneProp.length / np_h;
    double cone_tip_radius = coneProp.diameter / 2 * delta_h / coneProp.length;
    auto cone_bce = sysSPH.CreatePointsTruncatedConeInterior(coneProp.diameter / 2, cone_tip_radius,
                                                             coneProp.length - params.initial_spacing, true);
    sysFSI.AddFsiBody(cone, cone_bce, ChFrame<>(VNULL, QUNIT), false);
    // Create the linear motor to move the cone at the prescribed velocity
    auto motor = chrono_types::make_shared<ChLinkMotorLinearSpeed>();
    motor->SetSpeedFunction(chrono_types::make_shared<ChFunctionConst>(-penetration_velocity));
    motor->Initialize(cone, box, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
    sysMBS.AddLink(motor);

    // Add cylinder on top of cone like in the experiment
    // This is to prevent soild from falling on top of the cone, pushing it down
    auto cyl = chrono_types::make_shared<ChBody>();

    double cyl_radius = coneProp.diameter / 2;
    cyl->SetPos(ChVector3d(0, 0, cone_z_pos + cyl_length / 2));
    cyl->SetRot(ChQuaternion<>(1, 0, 0, 0));
    double cyl_volume = CH_PI * cyl_radius * cyl_radius * cyl_length;
    double cyl_mass = coneProp.density * cyl_volume;
    cyl->SetMass(cone_mass / 2);  // *10 because we fake the length compared to DEM
    ChMatrix33<> cyl_inertia = cyl_mass * ChCylinder::CalcGyration(cyl_radius, cyl_length);
    cyl->SetInertia(cyl_inertia);
    sysMBS.AddBody(cyl);
    chrono::utils::AddCylinderGeometry(cyl.get(), cmaterial, cyl_radius, cyl_length, ChVector3d(0, 0, 0), QUNIT, false,
                                       vis_material);
    cyl->GetCollisionModel()->SetSafeMargin(params.initial_spacing);
    // Register cylinder with explicit BCE points. Shorten to avoid overlap with cone
    auto cyl_bce = sysSPH.CreatePointsCylinderInterior(cyl_radius, cyl_length - 2 * params.initial_spacing, true);
    sysFSI.AddFsiBody(cyl, cyl_bce, ChFrame<>(VNULL, QUNIT), false);
    // Constraint cylinder to cone
    auto constraint = chrono_types::make_shared<ChLinkLockLock>();
    constraint->Initialize(cone, cyl, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
    sysMBS.AddLink(constraint);
    sysSPH.SetOutputLevel(OutputLevel::CRM_FULL);
    sysFSI.Initialize();

    // Output directories
    std::string out_dir;
    if (params.output || params.snapshots) {
        // Base output directory depends on testing_mode and includes container height in cm
        std::string base_dir;
        const std::string heightCmStr = [&]() {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(1) << (params.container_height * 100.0) << "cm";
            return oss.str();
        }();
        // Helper lambda to convert a value to a string using an ostringstream.
        auto toString = [](auto value) -> std::string {
            std::ostringstream oss;
            oss << value;
            return oss.str();
        };

        // Convert new parameters to strings
        const std::string rheologyModelStr = params.rheology_model_crm;
        const std::string prePressureScaleStr = toString(params.pre_pressure_scale);
        base_dir = GetChronoOutputPath() + "FSI_ConePenetrometer_GRC1_" + heightCmStr + "_" + rheologyModelStr + "_" +
                   prePressureScaleStr + "/";
        if (!filesystem::create_directory(filesystem::path(base_dir))) {
            std::cerr << "Error creating directory " << base_dir << std::endl;
            return;
        }

        // Format the penetration depth with fixed precision.
        const std::string penetrationDepthStr = [&]() {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(2) << params.penetration_depth;
            return oss.str();
        }();

        // Convert array values to strings.
        const std::string youngsModulusStr = toString(params.y_modulus);
        const std::string densityStr = toString(params.density);
        const std::string muSStr = toString(params.mu_s);
        const std::string mu2Str = toString(params.mu_2);
        const std::string cohesionStr = toString(params.cohesion);

        // Build the vector of subdirectory names.
        std::vector<std::string> subdirs = {"penetrationDepth_" + penetrationDepthStr,
                                            "youngsModulus_" + youngsModulusStr,
                                            "density_" + densityStr,
                                            "mu_s_" + muSStr,
                                            "mu_2_" + mu2Str,
                                            "cohesion_" + cohesionStr,
                                            "boundaryType_" + params.boundary_type,
                                            "viscosityType_" + params.viscosity_type,
                                            "kernelType_" + params.kernel_type};

        for (const auto& subdir : subdirs) {
            base_dir += subdir + "/";
            if (!filesystem::create_directory(filesystem::path(base_dir))) {
                std::cerr << "Error creating directory " << base_dir << std::endl;
                return;
            }
        }

        // Add flat structure
        std::stringstream ss;
        ss << "ps_" << params.ps_freq;
        ss << "_s_" << params.initial_spacing;
        ss << "_d0_" << params.d0_multiplier;
        ss << "_t_" << params.time_step;
        ss << "_av_" << params.artificial_viscosity;
        out_dir = base_dir + ss.str();

        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cerr << "Error creating directory " << out_dir << std::endl;
            return;
        }

        if (params.output) {
            if (!filesystem::create_directory(filesystem::path(out_dir + "/particles"))) {
                std::cerr << "Error creating directory " << out_dir + "/particles" << std::endl;
                return;
            }
            if (!filesystem::create_directory(filesystem::path(out_dir + "/fsi"))) {
                std::cerr << "Error creating directory " << out_dir + "/fsi" << std::endl;
                return;
            }
            if (!filesystem::create_directory(filesystem::path(out_dir + "/vtk"))) {
                std::cerr << "Error creating directory " << out_dir + "/vtk" << std::endl;
                return;
            }
        }

        if (params.snapshots) {
            if (!filesystem::create_directory(filesystem::path(out_dir + "/snapshots"))) {
                std::cerr << "Error creating directory " << out_dir + "/snapshots" << std::endl;
                return;
            }
        }

        // Write initial VTK files for the cone and cylinder setup
        std::cout << "Writing initial VTK files for visualization..." << std::endl;
        WriteConeVTK(out_dir + "/vtk/cone_initial.vtk", cone, coneProp.diameter / 2, coneProp.length);
        WriteCylinderVTK(out_dir + "/vtk/cylinder_initial.vtk", cyl, cyl_radius, cyl_length);
    }

    // Create a run-time visualizer
    std::shared_ptr<ChVisualSystem> vis;
    // Create a run-time visualizer
#ifdef CHRONO_VSG
    auto col_callback = chrono_types::make_shared<ParticlePressureColorCallback>(0, 30000, false);
    if (params.render) {
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
        visVSG->SetWindowTitle("FSI Cone Penetrometer");
        visVSG->SetWindowSize(1280, 720);
        visVSG->AddCamera(ChVector3d(0, -2 * container_height, 0.75 * container_height),
                          ChVector3d(0, 0, 0.55 * container_height));
        visVSG->SetLightIntensity(0.9);
        visVSG->SetLightDirection(-CH_PI_2, CH_PI / 6);

        visVSG->Initialize();
        vis = visVSG;
    }
#endif

    // Start the simulation
    double time = 0.0;
    int sim_frame = 0;
    int out_frame = 0;
    int pres_out_frame = 0;
    int render_frame = 0;
    double dT = sysFSI.GetStepSizeCFD();
    double pres_out_fps = 100;

    std::string out_file = out_dir + "/force_vs_time.txt";
    std::ofstream ofile(out_file, std::ios::trunc);

    // Add comma-separated header to the output file
    ofile << "Time,Force-x,Force-y,Force-z,position-x,position-y,penetration-depth,cone-pressure" << std::endl;

    ChTimer timer;
    timer.start();
    while (time < t_end) {
        // Calculate current penetration depth
        double current_depth = -(cone->GetPos().z() - (bzDim - clearance));

        // Check if penetration depth is reached within tolerance
        const double depth_tolerance = 1e-6;  // Small tolerance for floating-point comparison
        if (std::abs(current_depth - params.penetration_depth) < depth_tolerance) {
            std::cout << "Penetration depth reached" << std::endl;
            std::cout << "Current depth: " << current_depth << std::endl;
            std::cout << "Target depth: " << params.penetration_depth << std::endl;
            std::cout << "Time: " << time << std::endl;
            std::cout << "Cone now fixed" << std::endl;
            motor->SetSpeedFunction(chrono_types::make_shared<ChFunctionConst>(0));
        }
        if (params.output && time >= out_frame / params.output_fps) {
            if (params.write_marker_files) {
                sysSPH.SaveParticleData(out_dir + "/particles");
                sysSPH.SaveSolidData(out_dir + "/fsi", time);
                // Write VTK files for cone and cylinder
                std::stringstream cone_vtk_filename;
                cone_vtk_filename << out_dir << "/vtk/cone_" << std::setw(5) << std::setfill('0') << out_frame
                                  << ".vtk";
                WriteConeVTK(cone_vtk_filename.str(), cone, coneProp.diameter / 2, coneProp.length);

                std::stringstream cyl_vtk_filename;
                cyl_vtk_filename << out_dir << "/vtk/cylinder_" << std::setw(5) << std::setfill('0') << out_frame
                                 << ".vtk";
                WriteCylinderVTK(cyl_vtk_filename.str(), cyl, cyl_radius, cyl_length);
            }
            // std::cout << "Time: " << time << std::endl;
            std::cout << "Current Depth: " << current_depth << std::endl;
            double cone_pressure = abs(cone->GetAppliedForce().z() / (CH_PI * pow(coneProp.diameter / 2, 2)));
            std::cout << "Cone Pressure: " << cone_pressure << std::endl;
            out_frame++;
        }
#ifdef CHRONO_VSG
        if (params.render && time >= render_frame / params.render_fps) {
            if (!vis->Run())
                break;
            vis->Render();

            if (params.snapshots) {
                std::cout << " -- Snapshot frame " << render_frame << " at t = " << time << std::endl;
                std::ostringstream filename;
                filename << out_dir << "/snapshots/" << std::setw(5) << std::setfill('0') << render_frame << ".jpg";
                vis->WriteImageToFile(filename.str());
            }

            render_frame++;
        }
#endif
        if (time >= pres_out_frame / pres_out_fps) {
            double cone_pressure = abs(cone->GetAppliedForce().z() / (CH_PI * pow(coneProp.diameter / 2, 2)));
            ofile << time << "," << cone->GetAppliedForce().x() << "," << cone->GetAppliedForce().y() << ","
                  << cone->GetAppliedForce().z() << "," << cone->GetPos().x() << "," << cone->GetPos().y() << ","
                  << current_depth << "," << cone_pressure << std::endl;
            pres_out_frame++;
        }

        // Advance simulation for one timestep for all systems
        sysFSI.DoStepDynamics(dT);
        time += dT;

        sim_frame++;
    }
    timer.stop();
    std::cout << "\nSimulation time: " << timer() << " seconds\n" << std::endl;

    ofile.close();
}