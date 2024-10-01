// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Implementation of the GPU granular TERRAIN NODE (using Chrono::Gpu).
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <algorithm>
#include <iomanip>
#include <limits>

#include "chrono/utils/ChUtilsSamplers.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/assets/ChVisualShapeSphere.h"

#include "chrono_vehicle/cosim/terrain/ChVehicleCosimTerrainNodeGranularGPU.h"

#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
#endif
#ifdef CHRONO_OPENGL
    #include "chrono_opengl/ChVisualSystemOpenGL.h"
#endif

using std::cout;
using std::cerr;
using std::endl;

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// Extension to computational domain in vertical direction.
//// TODO: eliminate when Chrono::Gpu can use a domain that is *not* centered at origin!
const double EXTRA_HEIGHT = 1.0;

// All obstacle bodies have this tag
static constexpr int tag_obstacles = 100;

// -----------------------------------------------------------------------------

ChVehicleCosimTerrainNodeGranularGPU::ChVehicleCosimTerrainNodeGranularGPU(double length, double width)
    : ChVehicleCosimTerrainNodeChrono(Type::GRANULAR_GPU, length, width, ChContactMethod::SMC),
      m_sampling_type(utils::SamplingType::POISSON_DISK),
      m_init_depth(0.2),
      m_separation_factor(1.001),
      m_in_layers(false),
      m_constructed(false),
      m_use_checkpoint(false),
      m_settling_output(false),
      m_settling_fps(100),
      m_num_particles(0) {
    // Default granular material properties
    m_radius_g = 0.01;
    m_rho_g = 2000;

    m_fixed_settling_duration = true;
    m_time_settling = 0.4;
    m_KE_settling = 1e-3;

    // Default granular system settings
    m_integrator_type = gpu::CHGPU_TIME_INTEGRATOR::CENTERED_DIFFERENCE;
    m_tangential_model = gpu::CHGPU_FRICTION_MODE::MULTI_STEP;

    // Create systems
    m_system = new ChSystemSMC();
    m_system->SetGravitationalAcceleration(ChVector3d(0, 0, m_gacc));

    // Defer construction of the granular system to Construct
    m_systemGPU = nullptr;
}

ChVehicleCosimTerrainNodeGranularGPU::ChVehicleCosimTerrainNodeGranularGPU(const std::string& specfile)
    : ChVehicleCosimTerrainNodeChrono(Type::GRANULAR_GPU, 0, 0, ChContactMethod::SMC),
      m_constructed(false),
      m_use_checkpoint(false),
      m_settling_output(false),
      m_settling_fps(100),
      m_num_particles(0) {
    // Default granular system settings
    m_integrator_type = gpu::CHGPU_TIME_INTEGRATOR::CENTERED_DIFFERENCE;
    m_tangential_model = gpu::CHGPU_FRICTION_MODE::MULTI_STEP;

    // Create systems
    m_system = new ChSystemSMC();
    m_system->SetGravitationalAcceleration(ChVector3d(0, 0, m_gacc));

    // Defer construction of the granular system to Construct
    m_systemGPU = nullptr;

    // Read GPU granular terrain parameters from provided specfile
    SetFromSpecfile(specfile);
}

ChVehicleCosimTerrainNodeGranularGPU ::~ChVehicleCosimTerrainNodeGranularGPU() {}

// -----------------------------------------------------------------------------

//// TODO: error checking
void ChVehicleCosimTerrainNodeGranularGPU::SetFromSpecfile(const std::string& specfile) {
    Document d;
    ReadSpecfile(specfile, d);

    m_dimX = d["Patch dimensions"]["Length"].GetDouble();
    m_dimY = d["Patch dimensions"]["Width"].GetDouble();

    m_radius_g = d["Granular material"]["Radius"].GetDouble();
    m_rho_g = d["Granular material"]["Density"].GetDouble();

    auto material = chrono_types::make_shared<ChContactMaterialSMC>();
    double coh_pressure = d["Material properties"]["Cohesion pressure"].GetDouble();
    double coh_force = CH_PI * m_radius_g * m_radius_g * coh_pressure;

    material->SetFriction(d["Material properties"]["Coefficient of friction"].GetDouble());
    material->SetRestitution(d["Material properties"]["Coefficient of restitution"].GetDouble());
    material->SetYoungModulus(d["Material properties"]["Young modulus"].GetDouble());
    material->SetPoissonRatio(d["Material properties"]["Poisson ratio"].GetDouble());
    material->SetAdhesion(static_cast<float>(coh_force));
    material->SetKn(d["Material properties"]["Kn"].GetDouble());
    material->SetGn(d["Material properties"]["Gn"].GetDouble());
    material->SetKt(d["Material properties"]["Kt"].GetDouble());
    material->SetGt(d["Material properties"]["Gt"].GetDouble());
    m_material_terrain = material;

    std::string sampling = d["Particle generation"]["Sampling method"].GetString();
    if (sampling.compare("POISSON_DISK") == 0)
        m_sampling_type = utils::SamplingType::POISSON_DISK;
    else if (sampling.compare("HCP_PACK") == 0)
        m_sampling_type = utils::SamplingType::HCP_PACK;
    else if (sampling.compare("REGULAR_GRID") == 0)
        m_sampling_type = utils::SamplingType::REGULAR_GRID;

    m_init_depth = d["Particle generation"]["Initial height"].GetDouble();
    m_separation_factor = d["Particle generation"]["Separation factor"].GetDouble();
    m_in_layers = d["Particle generation"]["Initialize in layers"].GetBool();

    std::string model = d["Simulation settings"]["Tangential displacement model"].GetString();
    if (model.compare("MULTI_STEP") == 0)
        m_tangential_model = gpu::CHGPU_FRICTION_MODE::MULTI_STEP;
    else if (model.compare("SINGLE_STEP") == 0)
        m_tangential_model = gpu::CHGPU_FRICTION_MODE::SINGLE_STEP;
    else if (model.compare("FRCTIONLESS") == 0)
        m_tangential_model = gpu::CHGPU_FRICTION_MODE::FRICTIONLESS;

    m_fixed_proxies = d["Simulation settings"]["Fix proxies"].GetBool();
}

void ChVehicleCosimTerrainNodeGranularGPU::SetGranularMaterial(double radius, double density) {
    m_radius_g = radius;
    m_rho_g = density;
}

////void ChVehicleCosimTerrainNodeGranularGPU::SetContactForceModel(ChSystemSMC::ContactForceModel model) {
////}

void ChVehicleCosimTerrainNodeGranularGPU::SetIntegratorType(gpu::CHGPU_TIME_INTEGRATOR type) {
    m_integrator_type = type;
}

void ChVehicleCosimTerrainNodeGranularGPU::SetTangentialDisplacementModel(gpu::CHGPU_FRICTION_MODE model) {
    m_tangential_model = model;
}

void ChVehicleCosimTerrainNodeGranularGPU::SetSamplingMethod(utils::SamplingType type,
                                                             double init_height,
                                                             double sep_factor,
                                                             bool in_layers) {
    m_sampling_type = type;
    m_init_depth = init_height;
    m_separation_factor = sep_factor;
    m_in_layers = in_layers;
}

void ChVehicleCosimTerrainNodeGranularGPU::SetMaterialSurface(const std::shared_ptr<ChContactMaterialSMC>& mat) {
    m_material_terrain = mat;
}

void ChVehicleCosimTerrainNodeGranularGPU::SetInputFromCheckpoint(const std::string& filename) {
    m_use_checkpoint = true;
    m_checkpoint_filename = filename;
}

void ChVehicleCosimTerrainNodeGranularGPU::SetSettlingTime(double time) {
    m_time_settling = time;
    m_fixed_settling_duration = true;
}

void ChVehicleCosimTerrainNodeGranularGPU::SetSettlingKineticEneryThreshold(double threshold) {
    m_KE_settling = threshold;
    m_fixed_settling_duration = false;
}

void ChVehicleCosimTerrainNodeGranularGPU::EnableSettlingOutput(bool output, double output_fps) {
    m_settling_output = output;
    m_settling_fps = output_fps;
}

// -----------------------------------------------------------------------------
// Complete construction of the mechanical system.
// This function is invoked automatically from Settle and Initialize.
// - adjust system settings
// - create the container body (implicit boundaries?)
// - create the granular material
// -----------------------------------------------------------------------------
void ChVehicleCosimTerrainNodeGranularGPU::Construct() {
    if (m_constructed)
        return;

    if (m_verbose)
        cout << "[Terrain node] GRANULAR_GPU " << endl;

#ifndef CHRONO_OPENGL
    // Disable rendering if Chrono::OpenGL not available (performance considerations).
    m_renderRT = false;
#endif

    // Calculate domain size
    //// TODO: For now, we need to hack in a larger dimZ to accomodate any wheels that will only show up later.
    ////       This limitations needs to be removed in Chrono::Gpu!
    ////       Second, we are limited to creating this domain centered at the origin!?!
    float r = m_separation_factor * (float)m_radius_g;
    float delta = 2.0f * r;
    float dimZ = m_init_depth + EXTRA_HEIGHT;
    auto box = ChVector3f(m_dimX, m_dimY, dimZ);

    // Create granular system here
    m_systemGPU = new gpu::ChSystemGpuMesh((float)m_radius_g, (float)m_rho_g, box);
    m_systemGPU->SetGravitationalAcceleration(ChVector3f(0, 0, (float)m_gacc));
    m_systemGPU->SetTimeIntegrator(m_integrator_type);
    m_systemGPU->SetFrictionMode(m_tangential_model);
    m_systemGPU->SetParticleOutputMode(gpu::CHGPU_OUTPUT_MODE::CSV);
    m_systemGPU->SetVerbosity(gpu::CHGPU_VERBOSITY::QUIET);
    m_systemGPU->SetMeshVerbosity(gpu::CHGPU_MESH_VERBOSITY::QUIET);

    // Set composite material properties for internal contacts.
    // Defer setting composite material properties for external contacts until creation of proxies (when we have
    // received object material)
    SetMatPropertiesInternal();

    // Set integration step-size
    m_systemGPU->SetFixedStepSize((float)m_step_size);

    // Create granular material
    std::vector<ChVector3f> pos(m_num_particles);

    if (m_use_checkpoint) {
        // Read particle state from checkpoint file
        std::string checkpoint_filename = m_node_out_dir + "/" + m_checkpoint_filename;
        std::ifstream ifile(checkpoint_filename);
        if (!ifile.is_open()) {
            cout << "ERROR: could not open checkpoint file " << checkpoint_filename << endl;
            MPI_Abort(MPI_COMM_WORLD, 1);
        }

        // Read number of particles in checkpoint
        std::string line;
        std::getline(ifile, line);
        std::istringstream iss(line);
        iss >> m_num_particles;

        pos.resize(m_num_particles);
        std::vector<ChVector3f> vel(m_num_particles);
        std::vector<ChVector3f> omg(m_num_particles);
        for (unsigned int i = 0; i < m_num_particles; i++) {
            std::getline(ifile, line);
            std::istringstream iss1(line);
            unsigned int identifier;
            iss1 >> identifier >> pos[i].x() >> pos[i].y() >> pos[i].z() >> vel[i].x() >> vel[i].y() >> vel[i].z() >>
                omg[i].x() >> omg[i].y() >> omg[i].z();
            assert(identifier == i);
        }
        m_systemGPU->SetParticles(pos, vel, omg);

        if (m_verbose)
            cout << "[Terrain node] read " << checkpoint_filename << "   num. particles = " << m_num_particles << endl;
    } else {
        // Generate particles using the specified volume sampling type
        utils::ChSampler<float>* sampler;
        switch (m_sampling_type) {
            default:
            case utils::SamplingType::POISSON_DISK:
                sampler = new utils::ChPDSampler<float>(delta);
                break;
            case utils::SamplingType::HCP_PACK:
                sampler = new utils::ChHCPSampler<float>(delta);
                break;
            case utils::SamplingType::REGULAR_GRID:
                sampler = new utils::ChGridSampler<float>(delta);
                break;
        }

        if (m_in_layers) {
            ChVector3f hdims(m_dimX / 2 - r, m_dimY / 2 - r, 0);
            double z = delta;
            while (z < m_init_depth) {
                auto p = sampler->SampleBox(ChVector3d(0, 0, z - dimZ / 2), hdims);
                pos.insert(pos.end(), p.begin(), p.end());
                if (m_verbose)
                    cout << "   z =  " << z << "\tnum particles = " << pos.size() << endl;
                z += delta;
            }
        } else {
            ChVector3d hdims(m_dimX / 2 - r, m_dimY / 2 - r, m_init_depth / 2 - r);
            auto p = sampler->SampleBox(ChVector3d(0, 0, m_init_depth / 2 - dimZ / 2), hdims);
            pos.insert(pos.end(), p.begin(), p.end());
        }

        m_systemGPU->SetParticles(pos);
        m_num_particles = (unsigned int)pos.size();
        if (m_verbose)
            cout << "[Terrain node] Generated num particles = " << m_num_particles << endl;

        delete sampler;
    }

    m_systemGPU->SetBDFixed(true);

    // Find "height" of granular material
    //// TODO: cannot call get_max_z() here!!!
    ////CalcInitHeight();
    auto init_height = -std::numeric_limits<float>::max();
    for (const auto& p : pos) {
        if (p.z() > init_height)
            init_height = p.z();
    }
    m_init_height = (double)init_height + m_radius_g;
    if (m_verbose)
        cout << "[Terrain node] initial height = " << m_init_height << endl;

    // Complete construction of the granular system
    // Note that no meshes are defined yet, so this only initializes the granular material.
    m_systemGPU->Initialize();

    // Create bodies in Chrono system (visualization only)
    if (m_renderRT) {
        for (const auto& p : pos) {
            auto body = chrono_types::make_shared<ChBody>();
            body->SetPos(p);
            body->SetFixed(true);
            auto sph = chrono_types::make_shared<ChVisualShapeSphere>(m_radius_g);
            body->AddVisualShape(sph);
            m_system->AddBody(body);
        }
    }

    // Add all rigid obstacles
    for (auto& b : m_obstacles) {
        auto mat = b.m_contact_mat.CreateMaterial(m_system->GetContactMethod());
        auto trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(GetChronoDataFile(b.m_mesh_filename), true, true);
        double mass;
        ChVector3d baricenter;
        ChMatrix33<> inertia;
        trimesh->ComputeMassProperties(true, mass, baricenter, inertia);

        auto body = chrono_types::make_shared<ChBody>();
        body->SetName("obstacle");
        body->SetTag(tag_obstacles);
        body->SetPos(b.m_init_pos);
        body->SetRot(b.m_init_rot);
        body->SetMass(mass * b.m_density);
        body->SetInertia(inertia * b.m_density);
        body->SetFixed(false);
        body->EnableCollision(true);

        auto ct_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(mat, trimesh, false, false, m_radius_g);
        body->AddCollisionShape(ct_shape);
        body->GetCollisionModel()->SetFamily(2);

        auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(filesystem::path(b.m_mesh_filename).stem());
        body->AddVisualShape(trimesh_shape, ChFrame<>());

        m_system->AddBody(body);

        // Set mesh for granular system
        /*auto imesh =*/m_systemGPU->AddMesh(trimesh, mass);
    }

    // Write file with terrain node settings
    std::ofstream outf;
    outf.open(m_node_out_dir + "/settings.info", std::ios::out);
    outf << "System settings" << endl;
    outf << "   Integration step size = " << m_step_size << endl;
    outf << "Terrain patch dimensions" << endl;
    outf << "   X = " << m_dimX << "  Y = " << m_dimY << endl;
    outf << "Terrain material properties" << endl;
    auto mat = std::static_pointer_cast<ChContactMaterialSMC>(m_material_terrain);
    outf << "   Coefficient of friction    = " << mat->GetSlidingFriction() << endl;
    outf << "   Coefficient of restitution = " << mat->GetRestitution() << endl;
    outf << "   Young modulus              = " << mat->GetYoungModulus() << endl;
    outf << "   Poisson ratio              = " << mat->GetPoissonRatio() << endl;
    outf << "   Adhesion force             = " << mat->GetAdhesion() << endl;
    outf << "   Kn = " << mat->GetKn() << endl;
    outf << "   Gn = " << mat->GetGn() << endl;
    outf << "   Kt = " << mat->GetKt() << endl;
    outf << "   Gt = " << mat->GetGt() << endl;
    outf << "Granular material properties" << endl;
    outf << "   particle radius  = " << m_radius_g << endl;
    outf << "   particle density = " << m_rho_g << endl;
    outf << "   number particles = " << m_num_particles << endl;

    // Mark system as constructed.
    m_constructed = true;
}

// -----------------------------------------------------------------------------
// Settling phase for the terrain node
// - settle terrain through simulation
// - update initial height of terrain
// -----------------------------------------------------------------------------
void ChVehicleCosimTerrainNodeGranularGPU::Settle() {
    Construct();

    // Create subdirectory for output from settling simulation (if enabled)
    if (m_settling_output) {
        if (!filesystem::create_directory(filesystem::path(m_node_out_dir + "/settling"))) {
            cout << "Error creating directory " << m_node_out_dir + "/settling" << endl;
            return;
        }
    }

    // Packing density at initial configuration
    double depth0;
    double eta0 = CalculatePackingDensity(depth0);

    // Simulate settling of granular terrain
    int output_steps = (int)std::ceil(1 / (m_settling_fps * m_step_size));
    int total_steps = (int)std::ceil(m_time_settling / m_step_size);
    int output_frame = 0;
    int n_contacts;
    int max_contacts = 0;
    unsigned long long int cum_contacts = 0;

    std::cout << "[Terrain node] START settling" << endl;

    int steps = 0;
    double time = 0;
    float KE = 0;
    while (true) {
        // Advance step
        m_timer.reset();
        m_timer.start();
        if (m_renderRT) {
            m_system->DoStepDynamics(m_step_size);
        }
        m_systemGPU->AdvanceSimulation((float)m_step_size);
        m_timer.stop();
        m_cum_sim_time += m_timer();

        n_contacts = GetNumContacts();
        cum_contacts += n_contacts;
        max_contacts = std::max(max_contacts, n_contacts);

        if (m_verbose)
            cout << '\r' << std::fixed << std::setprecision(6) << (steps + 1) * m_step_size << "  ["
                 << m_timer.GetTimeSeconds() << "]   " << n_contacts << std::flush;

        // Output (if enabled)
        if (m_settling_output && steps % output_steps == 0) {
            std::string filename = OutputFilename(m_node_out_dir + "/settling", "settling", "csv", output_frame + 1, 5);
            m_systemGPU->SetParticleOutputFlags(gpu::CHGPU_OUTPUT_FLAGS::VEL_COMPONENTS |
                                                gpu::CHGPU_OUTPUT_FLAGS::ANG_VEL_COMPONENTS |
                                                gpu::CHGPU_OUTPUT_FLAGS::FORCE_COMPONENTS);
            m_systemGPU->WriteParticleFile(filename);
            output_frame++;
        }

        // Render (if enabled)
        Render(m_step_size);

        steps++;
        time += m_step_size;

        // Stopping criteria
        if (m_fixed_settling_duration) {
            ProgressBar(steps, total_steps);
            if (time >= m_time_settling) {
                KE = m_systemGPU->GetParticlesKineticEnergy();
                break;
            }
        } else if (time > 0.1) {
            KE = m_systemGPU->GetParticlesKineticEnergy();
            if (KE <= m_KE_settling)
                break;
        }
    }

    cout << endl;
    cout << "[Terrain node] settling time = " << m_cum_sim_time << endl;

    // Find "height" of granular material after settling
    m_init_height = m_systemGPU->GetMaxParticleZ() + m_radius_g;
    if (m_verbose)
        cout << "[Terrain node] initial height = " << m_init_height << endl;

    // Packing density after settling
    double depth1;
    double eta1 = CalculatePackingDensity(depth1);
    if (m_verbose) {
        cout << "[Terrain node] settling phase ended at time:              " << time << endl;
        cout << "[Terrain node] total kinetic energy after settling:       " << KE << endl;
        cout << "[Terrain node] material depth before and after settling:  " << depth0 << " -> " << depth1 << endl;
        cout << "[Terrain node] packing density before and after settling: " << eta0 << " -> " << eta1 << endl;
    }

    // Write file with stats for the settling phase
    std::ofstream outf;
    outf.open(m_node_out_dir + "/settling_stats.info", std::ios::out);
    outf << "Number particles:           " << m_num_particles << endl;
    outf << "Initial material depth:     " << depth0 << endl;
    outf << "Initial packing density:    " << eta0 << endl;
    outf << "Final material depth:       " << depth1 << endl;
    outf << "Final packing density:      " << eta1 << endl;
    outf << "Final kinetic energy:       " << KE << endl;
    outf << "Average number of contacts: " << cum_contacts / (double)steps << endl;
    outf << "Maximum number contacts:    " << max_contacts << endl;
    outf << "Output?                     " << (m_settling_output ? "YES" : "NO") << endl;
    outf << "Output frequency (FPS):     " << m_settling_fps << endl;
    outf << "Settling duration:          " << time << endl;
    outf << "Settling simulation time:   " << m_cum_sim_time << endl;

    // Reset cumulative simulation time
    m_cum_sim_time = 0;
}

// -----------------------------------------------------------------------------

//// TODO: Consider looking only at particles below a certain fraction of the
//// height of the highest particle.  This would eliminate errors in estimating
//// the total volume stemming from a "non-smooth" top surface or stray particles.
double ChVehicleCosimTerrainNodeGranularGPU::CalculatePackingDensity(double& depth) {
    // Find height of granular material
    double z_max = m_systemGPU->GetMaxParticleZ();
    double z_min = -(m_init_depth + EXTRA_HEIGHT) / 2;
    depth = z_max - z_min;

    // Find total volume of granular material
    double Vt = m_dimX * m_dimY * (z_max - z_min);

    // Find volume of granular particles
    double Vs = m_num_particles * (4.0 / 3) * CH_PI * std::pow(m_radius_g, 3);

    // Packing density = Vs/Vt
    return Vs / Vt;
}

// -----------------------------------------------------------------------------

// Set composite material properties for internal contacts (assume same material for spheres and walls)
void ChVehicleCosimTerrainNodeGranularGPU::SetMatPropertiesInternal() {
    auto material_terrain = std::static_pointer_cast<ChContactMaterialSMC>(m_material_terrain);

    m_systemGPU->SetKn_SPH2SPH(material_terrain->GetKn());
    m_systemGPU->SetKn_SPH2WALL(material_terrain->GetKn());

    m_systemGPU->SetGn_SPH2SPH(material_terrain->GetGn());
    m_systemGPU->SetGn_SPH2WALL(material_terrain->GetGn());

    m_systemGPU->SetKt_SPH2SPH(material_terrain->GetKt());
    m_systemGPU->SetKt_SPH2WALL(material_terrain->GetKt());

    m_systemGPU->SetGt_SPH2SPH(material_terrain->GetGt());
    m_systemGPU->SetGt_SPH2WALL(material_terrain->GetGt());

    m_systemGPU->SetStaticFrictionCoeff_SPH2SPH(material_terrain->GetStaticFriction());
    m_systemGPU->SetStaticFrictionCoeff_SPH2WALL(material_terrain->GetStaticFriction());

    //// TODO: adhesion/cohesion
    //// TODO: why are cohesion and adhesion defined as ratios to sphere weight?!?
    m_systemGPU->SetCohesionRatio(0);
    m_systemGPU->SetAdhesionRatio_SPH2WALL(0);
}

// Set composite material properties for external contacts (granular-object)
void ChVehicleCosimTerrainNodeGranularGPU::SetMatPropertiesExternal(unsigned int i_shape) {
    //// RADU TODO
    //// Chrono::GPU is currently limited to a single material for an interacting object?!?
    //// For now, use the first one only
    auto mat_props = m_geometry[i_shape].materials[0];

    auto material_terrain = std::static_pointer_cast<ChContactMaterialSMC>(m_material_terrain);
    auto material = std::static_pointer_cast<ChContactMaterialSMC>(mat_props.CreateMaterial(m_method));

    const auto& strategy = m_system->GetMaterialCompositionStrategy();
    auto Kn = strategy.CombineStiffnessCoefficient(material_terrain->GetKn(), material->GetKn());
    auto Kt = strategy.CombineStiffnessCoefficient(material_terrain->GetKt(), material->GetKt());
    auto Gn = strategy.CombineDampingCoefficient(material_terrain->GetGn(), material->GetGn());
    auto Gt = strategy.CombineDampingCoefficient(material_terrain->GetGt(), material->GetGt());
    auto mu = strategy.CombineFriction(m_material_terrain->GetStaticFriction(), material->GetStaticFriction());

    m_systemGPU->SetKn_SPH2MESH(Kn);
    m_systemGPU->SetGn_SPH2MESH(Gn);
    m_systemGPU->SetKt_SPH2MESH(Kt);
    m_systemGPU->SetGt_SPH2MESH(Gt);
    m_systemGPU->SetStaticFrictionCoeff_SPH2MESH(mu);

    //// TODO: adhesion/cohesion
    //// TODO: why are cohesion and adhesion defined as ratios to sphere weight?!?
    m_systemGPU->SetAdhesionRatio_SPH2MESH(0);
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeGranularGPU::CreateRigidProxy(unsigned int i) {
    // Get shape associated with the given object
    int i_shape = m_obj_map[i];

    // Create the proxy associated with the given object
    auto proxy = chrono_types::make_shared<ProxyBodySet>();

    auto body = chrono_types::make_shared<ChBody>();
    body->SetTag(0);
    body->SetMass(m_load_mass[i]);
    ////body->SetInertiaXX();   //// TODO
    body->SetFixed(m_fixed_proxies);
    body->EnableCollision(true);

    // Create visualization asset (use collision shapes)
    m_geometry[i_shape].CreateVisualizationAssets(body, VisualizationType::COLLISION);

    // Create collision shapes (only if obstacles are present)
    auto num_obstacles = m_obstacles.size();
    if (num_obstacles > 0) {
        for (auto& mesh : m_geometry[i_shape].coll_meshes)
            mesh.radius = m_radius_g;
        m_geometry[i_shape].CreateCollisionShapes(body, 1, m_method);
        body->GetCollisionModel()->SetFamily(1);
        body->GetCollisionModel()->DisallowCollisionsWith(1);
    }

    m_system->AddBody(body);
    proxy->AddBody(body, 0);

    m_proxies[i] = proxy;

    // Set mesh for granular system
    //// RADU TODO: what about other collision primitives?!?
    for (auto& mesh : m_geometry[i_shape].coll_meshes) {
        auto imesh = m_systemGPU->AddMesh(mesh.trimesh, (float)m_load_mass[i]);
        if (imesh != i + num_obstacles) {
            throw std::runtime_error("Error adding GPU mesh for object " + std::to_string(i));
        }
    }

    // Set composite material properties for external contacts
    SetMatPropertiesExternal(i_shape);

    // Complete construction of the granular system
    m_systemGPU->InitializeMeshes();
}

// Once all proxy bodies are created, complete construction of the underlying system.
void ChVehicleCosimTerrainNodeGranularGPU::OnInitialize(unsigned int num_objects) {
    ChVehicleCosimTerrainNodeChrono::OnInitialize(num_objects);

    // Create the visualization window
    if (m_renderRT) {
#if defined(CHRONO_VSG)
        auto vsys_vsg = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        vsys_vsg->AttachSystem(m_system);
        vsys_vsg->SetWindowTitle("Terrain Node (GranularGPU)");
        vsys_vsg->SetWindowSize(ChVector2i(1280, 720));
        vsys_vsg->SetWindowPosition(ChVector2i(100, 100));
        vsys_vsg->SetUseSkyBox(false);
        vsys_vsg->SetClearColor(ChColor(0.455f, 0.525f, 0.640f));
        vsys_vsg->AddCamera(m_cam_pos, ChVector3d(0, 0, 0));
        vsys_vsg->SetCameraAngleDeg(40);
        vsys_vsg->SetLightIntensity(1.0f);
        vsys_vsg->SetImageOutputDirectory(m_node_out_dir + "/images");
        vsys_vsg->SetImageOutput(m_writeRT);
        vsys_vsg->Initialize();

        m_vsys = vsys_vsg;
#elif defined(CHRONO_OPENGL)
        auto vsys_gl = chrono_types::make_shared<opengl::ChVisualSystemOpenGL>();
        vsys_gl->AttachSystem(m_system);
        vsys_gl->SetWindowTitle("Terrain Node (GranularGPU)");
        vsys_gl->SetWindowSize(1280, 720);
        vsys_gl->SetRenderMode(opengl::WIREFRAME);
        vsys_gl->Initialize();
        vsys_gl->AddCamera(m_cam_pos, ChVector3d(0, 0, 0));
        vsys_gl->SetCameraProperties(0.05f);
        vsys_gl->SetCameraVertical(CameraVerticalDir::Z);

        m_vsys = vsys_gl;
#endif
    }
}

// Set state of proxy rigid body.
void ChVehicleCosimTerrainNodeGranularGPU::UpdateRigidProxy(unsigned int i, BodyState& rigid_state) {
    auto proxy = std::static_pointer_cast<ProxyBodySet>(m_proxies[i]);
    proxy->bodies[0]->SetPos(rigid_state.pos);
    proxy->bodies[0]->SetPosDt(rigid_state.lin_vel);
    proxy->bodies[0]->SetRot(rigid_state.rot);
    proxy->bodies[0]->SetAngVelParent(rigid_state.ang_vel);

    m_systemGPU->ApplyMeshMotion(i, rigid_state.pos, rigid_state.rot, rigid_state.lin_vel, rigid_state.ang_vel);
}

// Collect resultant contact force and torque on rigid proxy body.
void ChVehicleCosimTerrainNodeGranularGPU::GetForceRigidProxy(unsigned int i, TerrainForce& rigid_contact) {
    ChVector3d force;
    ChVector3d torque;
    m_systemGPU->CollectMeshContactForces(i, force, torque);

    rigid_contact.point = ChVector3d(0, 0, 0);
    rigid_contact.force = force;
    rigid_contact.moment = torque;
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeGranularGPU::OnAdvance(double step_size) {
    double t = 0;
    while (t < step_size) {
        double h = std::min<>(m_step_size, step_size - t);
        if (m_renderRT) {
            m_system->DoStepDynamics(h);
        }
        m_systemGPU->AdvanceSimulation((float)h);
        t += h;
    }
}

void ChVehicleCosimTerrainNodeGranularGPU::OnRender() {
#ifdef CHRONO_OPENGL
    if (!m_vsys)
        return;
    if (!m_vsys->Run())
        MPI_Abort(MPI_COMM_WORLD, 1);

    UpdateVisualizationParticles();

    if (m_track && !m_proxies.empty()) {
        auto proxy = std::static_pointer_cast<ProxyBodySet>(m_proxies[0]);  // proxy for first object
        ChVector3d cam_point = proxy->bodies[0]->GetPos();                  // position of first body in proxy set
        m_vsys->UpdateCamera(m_cam_pos, cam_point);
    }

    m_vsys->Render();
#endif
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeGranularGPU::UpdateVisualizationParticles() {
    // Note: it is assumed that the visualization bodies were created before the proxy body(ies).
    const auto& blist = m_system->GetBodies();
    for (unsigned int i = 0; i < m_num_particles; i++) {
        auto pos = m_systemGPU->GetParticlePosition(i);
        blist[i]->SetPos(pos);
    }
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeGranularGPU::WriteCheckpoint(const std::string& filename) const {
    assert(m_num_particles == m_systemGPU->GetNumParticles());
    utils::ChWriterCSV csv(" ");

    // Write number of granular material bodies.
    csv << m_num_particles << endl;

    for (unsigned int i = 0; i < m_num_particles; i++) {
        auto pos = m_systemGPU->GetParticlePosition(i);
        auto vel = m_systemGPU->GetParticleVelocity(i);
        auto omg = m_systemGPU->GetParticleAngVelocity(i);
        csv << i << pos << vel << omg << endl;
    }

    std::string checkpoint_filename = m_node_out_dir + "/" + filename;
    csv.WriteToFile(checkpoint_filename);
    if (m_verbose)
        cout << "[Terrain node] write checkpoint ===> " << checkpoint_filename << endl;
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeGranularGPU::OnOutputData(int frame) {
    // Create and write frame output file.
    std::string filename = OutputFilename(m_node_out_dir + "/simulation", "simulation", "csv", frame + 1, 5);

    m_systemGPU->SetParticleOutputMode(gpu::CHGPU_OUTPUT_MODE::CSV);
    m_systemGPU->SetParticleOutputFlags(gpu::CHGPU_OUTPUT_FLAGS::VEL_COMPONENTS |
                                        gpu::CHGPU_OUTPUT_FLAGS::ANG_VEL_COMPONENTS |
                                        gpu::CHGPU_OUTPUT_FLAGS::FORCE_COMPONENTS);
    m_systemGPU->WriteParticleFile(filename);
}

void ChVehicleCosimTerrainNodeGranularGPU::OutputVisualizationData(int frame) {
    auto filename = OutputFilename(m_node_out_dir + "/visualization", "vis", "chpf", frame, 5);
    m_systemGPU->SetParticleOutputMode(gpu::CHGPU_OUTPUT_MODE::CHPF);
    m_systemGPU->WriteParticleFile(filename);
    if (m_obstacles.size() > 0) {
        filename = OutputFilename(m_node_out_dir + "/visualization", "vis", "dat", frame, 5);
        // Include only obstacle bodies
        utils::WriteVisualizationAssets(
            m_system, filename, [](const ChBody& b) -> bool { return b.GetTag() == tag_obstacles; }, true);
    }
}

}  // end namespace vehicle
}  // end namespace chrono
