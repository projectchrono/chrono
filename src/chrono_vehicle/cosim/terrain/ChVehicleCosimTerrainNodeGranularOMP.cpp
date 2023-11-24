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
// Implementation of the OpenMP granular TERRAIN NODE (using Chrono::Multicore).
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

//// RADU TODO:
////    better approximation of mass / inertia? (CreateMeshProxy)
////    angular velocity (UpdateMeshProxy)

#include <algorithm>
#include <cmath>
#include <set>
#include <limits>
#include <unordered_map>

#include <omp.h>

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono/assets/ChVisualShapeTriangleMesh.h"

#include "chrono_vehicle/cosim/terrain/ChVehicleCosimTerrainNodeGranularOMP.h"

#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
#endif
#ifdef CHRONO_OPENGL
    #include "chrono_opengl/ChVisualSystemOpenGL.h"
#endif

using std::cout;
using std::endl;

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// Ensure that all bodies other than obstacles or granular particles are created with a smaller identifier.
// This allows filtering particle bodies or particle+obstacle bodies.
static const int body_id_obstacles = 100000;  // start identifier for obstacle bodies
static const int body_id_particles = 110000;  // start identifier for particle bodies

// -----------------------------------------------------------------------------
// Construction of the terrain node:
// - create the (multicore) Chrono system and set solver parameters
// - create the OpenGL visualization window
// -----------------------------------------------------------------------------
ChVehicleCosimTerrainNodeGranularOMP::ChVehicleCosimTerrainNodeGranularOMP(double length,
                                                                           double width,
                                                                           ChContactMethod method)
    : ChVehicleCosimTerrainNodeChrono(Type::GRANULAR_OMP, length, width, method),
      m_radius_p(5e-3),
      m_sampling_type(utils::SamplingType::POISSON_DISK),
      m_init_depth(0.2),
      m_separation_factor(1.001),
      m_in_layers(false),
      m_constructed(false),
      m_use_checkpoint(false),
      m_settling_output(false),
      m_settling_fps(100),
      m_thick(0.2),
      m_num_particles(0) {
    // Default granular material properties
    m_radius_g = 0.01;
    m_rho_g = 2000;

    m_fixed_settling_duration = true;
    m_time_settling = 0.4;
    m_KE_settling = 1e-3;

    // Create system and set method-specific solver settings
    switch (m_method) {
        case ChContactMethod::SMC: {
            ChSystemMulticoreSMC* sys = new ChSystemMulticoreSMC;
            sys->GetSettings()->solver.contact_force_model = ChSystemSMC::Hertz;
            sys->GetSettings()->solver.tangential_displ_mode = ChSystemSMC::TangentialDisplacementModel::OneStep;
            sys->GetSettings()->solver.use_material_properties = true;
            m_system = sys;

            break;
        }
        case ChContactMethod::NSC: {
            ChSystemMulticoreNSC* sys = new ChSystemMulticoreNSC;
            sys->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
            sys->GetSettings()->solver.max_iteration_normal = 0;
            sys->GetSettings()->solver.max_iteration_sliding = 200;
            sys->GetSettings()->solver.max_iteration_spinning = 0;
            sys->GetSettings()->solver.alpha = 0;
            sys->GetSettings()->solver.contact_recovery_speed = -1;
            sys->GetSettings()->collision.collision_envelope = 0.001;
            sys->ChangeSolverType(SolverType::APGD);
            m_system = sys;

            break;
        }
    }

    // Create an associated collision system
    m_system->SetCollisionSystemType(ChCollisionSystem::Type::MULTICORE);

    // Solver settings independent of method type
    m_system->Set_G_acc(ChVector<>(0, 0, m_gacc));
    m_system->GetSettings()->solver.use_full_inertia_tensor = false;
    m_system->GetSettings()->solver.tolerance = 0.1;
    m_system->GetSettings()->solver.max_iteration_bilateral = 100;
    m_system->GetSettings()->collision.narrowphase_algorithm = ChNarrowphase::Algorithm::HYBRID;

    // Set default number of threads
    m_system->SetNumThreads(1);
}

ChVehicleCosimTerrainNodeGranularOMP::ChVehicleCosimTerrainNodeGranularOMP(ChContactMethod method,
                                                                           const std::string& specfile)
    : ChVehicleCosimTerrainNodeChrono(Type::GRANULAR_OMP, 0, 0, method),
      m_constructed(false),
      m_use_checkpoint(false),
      m_settling_output(false),
      m_settling_fps(100),
      m_thick(0.2),
      m_num_particles(0) {
    // Create system and set method-specific solver settings
    switch (m_method) {
        case ChContactMethod::SMC: {
            ChSystemMulticoreSMC* sys = new ChSystemMulticoreSMC;
            sys->GetSettings()->solver.contact_force_model = ChSystemSMC::Hertz;
            sys->GetSettings()->solver.tangential_displ_mode = ChSystemSMC::TangentialDisplacementModel::OneStep;
            sys->GetSettings()->solver.use_material_properties = true;
            m_system = sys;

            break;
        }
        case ChContactMethod::NSC: {
            ChSystemMulticoreNSC* sys = new ChSystemMulticoreNSC;
            sys->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
            sys->GetSettings()->solver.max_iteration_normal = 0;
            sys->GetSettings()->solver.max_iteration_sliding = 200;
            sys->GetSettings()->solver.max_iteration_spinning = 0;
            sys->GetSettings()->solver.alpha = 0;
            sys->GetSettings()->solver.contact_recovery_speed = -1;
            sys->GetSettings()->collision.collision_envelope = 0.001;
            sys->ChangeSolverType(SolverType::APGD);
            m_system = sys;

            break;
        }
    }

    // Solver settings independent of method type
    m_system->Set_G_acc(ChVector<>(0, 0, m_gacc));
    m_system->GetSettings()->solver.use_full_inertia_tensor = false;
    m_system->GetSettings()->solver.tolerance = 0.1;
    m_system->GetSettings()->solver.max_iteration_bilateral = 100;
    m_system->GetSettings()->collision.narrowphase_algorithm = ChNarrowphase::Algorithm::HYBRID;

    // Set default number of threads
    m_system->SetNumThreads(1);

    // Read granular OMP terrain parameters from provided specfile
    SetFromSpecfile(specfile);
}

ChVehicleCosimTerrainNodeGranularOMP::~ChVehicleCosimTerrainNodeGranularOMP() {}

// -----------------------------------------------------------------------------

//// TODO: error checking
void ChVehicleCosimTerrainNodeGranularOMP::SetFromSpecfile(const std::string& specfile) {
    Document d;
    ReadSpecfile(specfile, d);

    m_dimX = d["Patch dimensions"]["Length"].GetDouble();
    m_dimY = d["Patch dimensions"]["Width"].GetDouble();

    m_radius_g = d["Granular material"]["Radius"].GetDouble();
    m_rho_g = d["Granular material"]["Density"].GetDouble();
    m_system->GetSettings()->collision.collision_envelope = 0.1 * m_radius_g;

    double coh_pressure = d["Material properties"]["Cohesion pressure"].GetDouble();
    double coh_force = CH_C_PI * m_radius_g * m_radius_g * coh_pressure;

    switch (GetSystem()->GetContactMethod()) {
        case ChContactMethod::SMC: {
            auto material = chrono_types::make_shared<ChMaterialSurfaceSMC>();
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
            break;
        }
        case ChContactMethod::NSC: {
            auto material = chrono_types::make_shared<ChMaterialSurfaceNSC>();
            material->SetFriction(d["Material properties"]["Coefficient of friction"].GetDouble());
            material->SetRestitution(d["Material properties"]["Coefficient of restitution"].GetDouble());
            material->SetCohesion(static_cast<float>(coh_force));
            m_material_terrain = material;
            break;
        }
    }

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

    std::string n_model = d["Simulation settings"]["Normal contact model"].GetString();
    if (n_model.compare("Hertz") == 0)
        m_system->GetSettings()->solver.contact_force_model = ChSystemSMC::ContactForceModel::Hertz;
    else if (n_model.compare("Hooke") == 0)
        m_system->GetSettings()->solver.contact_force_model = ChSystemSMC::ContactForceModel::Hooke;
    else if (n_model.compare("Flores") == 0)
        m_system->GetSettings()->solver.contact_force_model = ChSystemSMC::ContactForceModel::Flores;
    else if (n_model.compare("Hertz") == 0)
        m_system->GetSettings()->solver.contact_force_model = ChSystemSMC::ContactForceModel::PlainCoulomb;

    std::string t_model = d["Simulation settings"]["Tangential displacement model"].GetString();
    if (t_model.compare("MultiStep") == 0)
        m_system->GetSettings()->solver.tangential_displ_mode = ChSystemSMC::TangentialDisplacementModel::MultiStep;
    else if (t_model.compare("OneStep") == 0)
        m_system->GetSettings()->solver.tangential_displ_mode = ChSystemSMC::TangentialDisplacementModel::OneStep;
    else if (t_model.compare("None") == 0)
        m_system->GetSettings()->solver.tangential_displ_mode = ChSystemSMC::TangentialDisplacementModel::None;

    m_system->GetSettings()->solver.use_material_properties =
        d["Simulation settings"]["Use material properties"].GetBool();

    m_radius_p = d["Simulation settings"]["Proxy contact radius"].GetDouble();
    m_fixed_proxies = d["Simulation settings"]["Fix proxies"].GetBool();
}

void ChVehicleCosimTerrainNodeGranularOMP::SetNumThreads(int num_threads) {
    m_system->SetNumThreads(num_threads);
}

void ChVehicleCosimTerrainNodeGranularOMP::SetWallThickness(double thickness) {
    m_thick = thickness;
}

void ChVehicleCosimTerrainNodeGranularOMP::SetGranularMaterial(double radius, double density) {
    m_radius_g = radius;
    m_rho_g = density;
    m_system->GetSettings()->collision.collision_envelope = 0.1 * radius;
}

void ChVehicleCosimTerrainNodeGranularOMP::UseMaterialProperties(bool flag) {
    assert(m_system->GetContactMethod() == ChContactMethod::SMC);
    m_system->GetSettings()->solver.use_material_properties = flag;
}

void ChVehicleCosimTerrainNodeGranularOMP::SetContactForceModel(ChSystemSMC::ContactForceModel model) {
    assert(m_system->GetContactMethod() == ChContactMethod::SMC);
    m_system->GetSettings()->solver.contact_force_model = model;
}

void ChVehicleCosimTerrainNodeGranularOMP::SetTangentialDisplacementModel(
    ChSystemSMC::TangentialDisplacementModel model) {
    assert(m_system->GetContactMethod() == ChContactMethod::SMC);
    m_system->GetSettings()->solver.tangential_displ_mode = model;
}

void ChVehicleCosimTerrainNodeGranularOMP::SetSamplingMethod(utils::SamplingType type,
                                                             double init_height,
                                                             double sep_factor,
                                                             bool in_layers) {
    m_sampling_type = type;
    m_init_depth = init_height;
    m_separation_factor = sep_factor;
    m_in_layers = in_layers;
}

void ChVehicleCosimTerrainNodeGranularOMP::SetMaterialSurface(const std::shared_ptr<ChMaterialSurface>& mat) {
    assert(mat->GetContactMethod() == m_system->GetContactMethod());
    m_material_terrain = mat;
}

void ChVehicleCosimTerrainNodeGranularOMP::SetInputFromCheckpoint(const std::string& filename) {
    m_use_checkpoint = true;
    m_checkpoint_filename = filename;
}

void ChVehicleCosimTerrainNodeGranularOMP::SetSettlingTime(double time) {
    m_time_settling = time;
    m_fixed_settling_duration = true;
}

void ChVehicleCosimTerrainNodeGranularOMP::SetSettlingKineticEneryThreshold(double threshold) {
    m_KE_settling = threshold;
    m_fixed_settling_duration = false;
}

void ChVehicleCosimTerrainNodeGranularOMP::EnableSettlingOutput(bool output, double output_fps) {
    m_settling_output = output;
    m_settling_fps = output_fps;
}

// -----------------------------------------------------------------------------
// Complete construction of the mechanical system.
// This function is invoked automatically from Settle and Initialize.
// - adjust system settings
// - create the container body
// - create the granular material
// -----------------------------------------------------------------------------
void ChVehicleCosimTerrainNodeGranularOMP::Construct() {
    if (m_constructed)
        return;

    if (m_verbose)
        cout << "[Terrain node] GRANULAR_OMP "
             << " method = " << static_cast<std::underlying_type<ChContactMethod>::type>(m_method) << endl;

#pragma omp parallel
#pragma omp master
    {
        // Sanity check: print number of threads in a parallel region
        if (m_verbose)
            cout << "[Terrain node] actual number of OpenMP threads: " << omp_get_num_threads() << endl;
    }

    // Calculate container (half) height
    double r = m_separation_factor * m_radius_g;
    double delta = 2.0f * r;

    // Estimates for number of bins for broad-phase.
    int factor = 2;
    int binsX = (int)std::ceil(m_dimX / (2 * m_radius_g)) / factor;
    int binsY = (int)std::ceil(m_dimY / (2 * m_radius_g)) / factor;
    int binsZ = 1;
    m_system->GetSettings()->collision.bins_per_axis = vec3(binsX, binsY, binsZ);
    if (m_verbose)
        cout << "[Terrain node] broad-phase bins: " << binsX << " x " << binsY << " x " << binsZ << endl;

    // ---------------------
    // Create container body
    // ---------------------

    auto container = chrono_types::make_shared<ChBody>();
    m_system->AddBody(container);
    container->SetIdentifier(-1);
    container->SetMass(1);
    container->SetBodyFixed(true);
    container->SetCollide(true);

    double hdimX = m_dimX / 2;
    double hdimY = m_dimY / 2;
    double hdimZ = 0.5 * m_init_depth;
    double hthick = m_thick / 2;

    // Bottom box
    utils::AddBoxGeometry(container.get(), m_material_terrain, ChVector<>(m_dimX, m_dimY, m_thick),
                          ChVector<>(0, 0, -m_thick / 2), ChQuaternion<>(1, 0, 0, 0), true);
    // Front box
    utils::AddBoxGeometry(container.get(), m_material_terrain, ChVector<>(m_thick, m_dimY, m_init_depth + m_thick),
                          ChVector<>(hdimX + hthick, 0, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), false);
    // Rear box
    utils::AddBoxGeometry(container.get(), m_material_terrain, ChVector<>(m_thick, m_dimY, m_init_depth + m_thick),
                          ChVector<>(-hdimX - hthick, 0, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), false);
    // Left box
    utils::AddBoxGeometry(container.get(), m_material_terrain, ChVector<>(m_dimX, m_thick, m_init_depth + m_thick),
                          ChVector<>(0, hdimY + hthick, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), false);
    // Right box
    utils::AddBoxGeometry(container.get(), m_material_terrain, ChVector<>(m_dimX, m_thick, m_init_depth + m_thick),
                          ChVector<>(0, -hdimY - hthick, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), false);

    // Enable deactivation of bodies that exit a specified bounding box.
    // We set this bounding box to encapsulate the container with a conservative height.
    m_system->GetSettings()->collision.use_aabb_active = true;
    m_system->GetSettings()->collision.aabb_min = real3(-hdimX - hthick, -hdimY - hthick, -hthick);
    m_system->GetSettings()->collision.aabb_max = real3(+hdimX + hthick, +hdimY + hthick, 2 * hdimZ + 2);

    // --------------------------
    // Generate granular material
    // --------------------------

    // Cache the number of bodies that have been added so far to the multicore system.
    // ATTENTION: This will be used to set the state of granular material particles if
    // initializing them from a checkpoint file.
    uint particles_start_index = m_system->data_manager->num_rigid_bodies;

    // Create a particle generator and a mixture entirely made out of spheres
    utils::Generator gen(m_system);
    std::shared_ptr<utils::MixtureIngredient> m1 = gen.AddMixtureIngredient(utils::MixtureType::SPHERE, 1.0);
    m1->setDefaultMaterial(m_material_terrain);
    m1->setDefaultDensity(m_rho_g);
    m1->setDefaultSize(m_radius_g);

    // Set starting value for body identifiers
    gen.setBodyIdentifier(body_id_particles);

    // Create particles using the specified volume sampling type
    utils::Sampler<double>* sampler;
    switch (m_sampling_type) {
        default:
        case utils::SamplingType::POISSON_DISK:
            sampler = new utils::PDSampler<double>(delta);
            break;
        case utils::SamplingType::HCP_PACK:
            sampler = new utils::HCPSampler<double>(delta);
            break;
        case utils::SamplingType::REGULAR_GRID:
            sampler = new utils::GridSampler<double>(delta);
            break;
    }

    if (m_in_layers) {
        ChVector<> hdims(hdimX - r, hdimY - r, 0);
        double z = delta;
        while (z < m_init_depth) {
            gen.CreateObjectsBox(*sampler, ChVector<>(0, 0, z), hdims);
            if (m_verbose)
                cout << "   z =  " << z << "\tnum particles = " << gen.getTotalNumBodies() << endl;
            z += delta;
        }
    } else {
        ChVector<> hdims(hdimX - r, hdimY - r, m_init_depth / 2 - r);
        gen.CreateObjectsBox(*sampler, ChVector<>(0, 0, m_init_depth / 2), hdims);
    }

    m_num_particles = gen.getTotalNumBodies();
    if (m_verbose)
        cout << "[Terrain node] Generated num particles = " << m_num_particles << endl;

    // -------------------------------------------------------
    // If requested, overwrite particle states from checkpoint
    // -------------------------------------------------------
    if (m_use_checkpoint) {
        // Open input file stream
        std::string checkpoint_filename = m_node_out_dir + "/" + m_checkpoint_filename;
        std::ifstream ifile(checkpoint_filename);
        if (!ifile.is_open()) {
            cout << "ERROR: could not open checkpoint file " << checkpoint_filename << endl;
            MPI_Abort(MPI_COMM_WORLD, 1);
        }
        std::string line;

        // Read and discard line with current time
        std::getline(ifile, line);

        // Read number of particles in checkpoint
        unsigned int num_particles;
        {
            std::getline(ifile, line);
            std::istringstream iss(line);
            iss >> num_particles;

            if (num_particles != m_num_particles) {
                cout << "ERROR: inconsistent number of particles in checkpoint file!" << endl;
                MPI_Abort(MPI_COMM_WORLD, 1);
            }
        }

        // Read granular material state from checkpoint
        for (uint ib = particles_start_index; ib < m_system->Get_bodylist().size(); ++ib) {
            std::getline(ifile, line);
            std::istringstream iss(line);
            int identifier;
            ChVector<> pos;
            ChQuaternion<> rot;
            ChVector<> pos_dt;
            ChQuaternion<> rot_dt;
            iss >> identifier >> pos.x() >> pos.y() >> pos.z() >> rot.e0() >> rot.e1() >> rot.e2() >> rot.e3() >>
                pos_dt.x() >> pos_dt.y() >> pos_dt.z() >> rot_dt.e0() >> rot_dt.e1() >> rot_dt.e2() >> rot_dt.e3();

            auto body = m_system->Get_bodylist()[ib];
            assert(body->GetIdentifier() == identifier);
            body->SetPos(ChVector<>(pos.x(), pos.y(), pos.z()));
            body->SetRot(ChQuaternion<>(rot.e0(), rot.e1(), rot.e2(), rot.e3()));
            body->SetPos_dt(ChVector<>(pos_dt.x(), pos_dt.y(), pos_dt.z()));
            body->SetRot_dt(ChQuaternion<>(rot_dt.e0(), rot_dt.e1(), rot_dt.e2(), rot_dt.e3()));
        }

        if (m_verbose)
            cout << "[Terrain node] read " << checkpoint_filename << "   num. particles = " << num_particles << endl;
    }

    // Find "height" of granular material
    m_init_height = CalcCurrentHeight() + m_radius_g;
    if (m_verbose)
        cout << "[Terrain node] initial height = " << m_init_height << endl;

    // ----------------------
    // Create rigid obstacles
    // ----------------------

    int id = body_id_obstacles;
    for (auto& b : m_obstacles) {
        auto mat = b.m_contact_mat.CreateMaterial(m_system->GetContactMethod());
        auto trimesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(b.m_mesh_filename),
                                                                                  true, true);
        double mass;
        ChVector<> baricenter;
        ChMatrix33<> inertia;
        trimesh->ComputeMassProperties(true, mass, baricenter, inertia);

        auto body = chrono_types::make_shared<ChBody>();
        body->SetNameString("obstacle");
        body->SetIdentifier(id++);
        body->SetPos(b.m_init_pos);
        body->SetRot(b.m_init_rot);
        body->SetMass(mass * b.m_density);
        body->SetInertia(inertia * b.m_density);
        body->SetBodyFixed(false);
        body->SetCollide(true);

        auto ct_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(mat, trimesh, false, false, m_radius_g);
        body->AddCollisionShape(ct_shape);
        body->GetCollisionModel()->SetFamily(2);

        auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(filesystem::path(b.m_mesh_filename).stem());
        body->AddVisualShape(trimesh_shape, ChFrame<>());
        m_system->AddBody(body);
    }

    // Write file with terrain node settings
    std::ofstream outf;
    outf.open(m_node_out_dir + "/settings.info", std::ios::out);
    outf << "System settings" << endl;
    outf << "   Integration step size = " << m_step_size << endl;
    outf << "   Contact method = " << (m_method == ChContactMethod::SMC ? "SMC" : "NSC") << endl;
    outf << "   Use material properties? " << (m_system->GetSettings()->solver.use_material_properties ? "YES" : "NO")
         << endl;
    outf << "   Collision envelope = " << m_system->GetSettings()->collision.collision_envelope << endl;
    outf << "Terrain patch dimensions" << endl;
    outf << "   X = " << m_dimX << "  Y = " << m_dimY << endl;
    outf << "Terrain material properties" << endl;
    switch (m_method) {
        case ChContactMethod::SMC: {
            auto mat = std::static_pointer_cast<ChMaterialSurfaceSMC>(m_material_terrain);
            outf << "   Coefficient of friction    = " << mat->GetKfriction() << endl;
            outf << "   Coefficient of restitution = " << mat->GetRestitution() << endl;
            outf << "   Young modulus              = " << mat->GetYoungModulus() << endl;
            outf << "   Poisson ratio              = " << mat->GetPoissonRatio() << endl;
            outf << "   Adhesion force             = " << mat->GetAdhesion() << endl;
            outf << "   Kn = " << mat->GetKn() << endl;
            outf << "   Gn = " << mat->GetGn() << endl;
            outf << "   Kt = " << mat->GetKt() << endl;
            outf << "   Gt = " << mat->GetGt() << endl;
            break;
        }
        case ChContactMethod::NSC: {
            auto mat = std::static_pointer_cast<ChMaterialSurfaceNSC>(m_material_terrain);
            outf << "   Coefficient of friction    = " << mat->GetKfriction() << endl;
            outf << "   Coefficient of restitution = " << mat->GetRestitution() << endl;
            outf << "   Cohesion force             = " << mat->GetCohesion() << endl;
            break;
        }
    }
    outf << "Granular material properties" << endl;
    outf << "   particle radius  = " << m_radius_g << endl;
    outf << "   particle density = " << m_rho_g << endl;
    outf << "   number particles = " << m_num_particles << endl;
    outf << "Proxy body properties" << endl;
    outf << "   proxies fixed? " << (m_fixed_proxies ? "YES" : "NO") << endl;
    outf << "   proxy contact radius = " << m_radius_p << endl;

    // Mark system as constructed.
    m_constructed = true;
}

// -----------------------------------------------------------------------------
// Settling phase for the terrain node
// - settle terrain through simulation
// - update initial height of terrain
// -----------------------------------------------------------------------------
void ChVehicleCosimTerrainNodeGranularOMP::Settle() {
    Construct();

    // Create subdirectory for output from settling simulation (if enabled)
    if (m_settling_output) {
        if (!filesystem::create_directory(filesystem::path(m_node_out_dir + "/settling"))) {
            std::cout << "Error creating directory " << m_node_out_dir + "/settling" << std::endl;
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
    int cum_contacts = 0;

    std::cout << "[Terrain node] START settling" << endl;

    int steps = 0;
    double time = 0;
    float KE = 0;
    while (true) {
        // Advance step
        m_timer.reset();
        m_timer.start();
        m_system->DoStepDynamics(m_step_size);
        m_timer.stop();
        m_cum_sim_time += m_timer();

        n_contacts = GetNumContacts();
        cum_contacts += n_contacts;
        max_contacts = std::max(max_contacts, n_contacts);

        if (m_verbose)
            cout << '\r' << std::fixed << std::setprecision(6) << m_system->GetChTime() << "  ["
                 << m_timer.GetTimeSeconds() << "]  " << n_contacts << std::flush;

        // Output (if enabled)
        if (m_settling_output && steps % output_steps == 0) {
            std::string filename = OutputFilename(m_node_out_dir + "/settling", "settling", "dat", output_frame + 1, 5);
            utils::CSV_writer csv(" ");
            WriteParticleInformation(csv);
            csv.write_to_file(filename);
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
                KE = CalcTotalKineticEnergy();
                break;
            }
        } else if (time > 0.1) {
            KE = CalcTotalKineticEnergy();
            if (KE <= m_KE_settling)
                break;
        }
    }

    cout << endl;
    cout << "[Terrain node] settling time = " << m_cum_sim_time << endl;

    // Find "height" of granular material after settling
    m_init_height = CalcCurrentHeight() + m_radius_g;
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

double ChVehicleCosimTerrainNodeGranularOMP::CalcTotalKineticEnergy() {
    double KE = 0;
    for (const auto& body : m_system->Get_bodylist()) {
        if (body->GetIdentifier() > 0) {
            auto omg = body->GetWvel_par();
            auto J = body->GetInertiaXX();
            KE += body->GetMass() * body->GetPos_dt().Length2() + omg.Dot(J * omg);
        }
    }
    return 0.5 * KE;
}

double ChVehicleCosimTerrainNodeGranularOMP::CalcCurrentHeight() {
    double height = -std::numeric_limits<double>::max();
    for (const auto& body : m_system->Get_bodylist()) {
        if (body->GetIdentifier() > 0 && body->GetPos().z() > height)
            height = body->GetPos().z();
    }
    return height;
}

//// TODO: Consider looking only at particles below a certain fraction of the
//// height of the highest particle.  This would eliminate errors in estimating
//// the total volume stemming from a "non-smooth" top surface or stray particles.
double ChVehicleCosimTerrainNodeGranularOMP::CalculatePackingDensity(double& depth) {
    // Find height of granular material
    double z_max = CalcCurrentHeight();
    double z_min = 0;
    depth = z_max - z_min;

    // Find total volume of granular material
    double Vt = m_dimX * m_dimY * (z_max - z_min);

    // Find volume of granular particles
    double Vs = m_num_particles * (4.0 / 3) * CH_C_PI * std::pow(m_radius_g, 3);

    // Packing density = Vs/Vt
    return Vs / Vt;
}

// -----------------------------------------------------------------------------

// Create bodies with triangular contact geometry as proxies for the mesh faces.
// Used for flexible bodies.
// Assign to each body an identifier equal to the index of its corresponding mesh face.
// Maintain a list of all bodies associated with the object.
// Add all proxy bodies to the same collision family and disable collision between any
// two members of this family.
void ChVehicleCosimTerrainNodeGranularOMP::CreateMeshProxy(unsigned int i) {
    // Get shape associated with the given object
    int i_shape = m_obj_map[i];

    // Create the proxy associated with the given object
    auto proxy = chrono_types::make_shared<ProxyBodySet>();

    // Note: it is assumed that there is one and only one mesh defined!
    auto nt = m_geometry[i_shape].m_coll_meshes[0].m_trimesh->getNumTriangles();
    auto i_mat = m_geometry[i_shape].m_coll_meshes[0].m_matID;
    auto material = m_geometry[i_shape].m_materials[i_mat].CreateMaterial(m_method);

    //// RADU TODO:  better approximation of mass / inertia?
    double mass_p = m_load_mass[i_shape] / nt;
    ChVector<> inertia_p = 1e-3 * mass_p * ChVector<>(0.1, 0.1, 0.1);

    for (int it = 0; it < nt; it++) {
        auto body = chrono_types::make_shared<ChBody>();
        body->SetIdentifier(it);
        body->SetMass(mass_p);
        body->SetInertiaXX(inertia_p);
        body->SetBodyFixed(m_fixed_proxies);
        body->SetCollide(true);

        // Create contact shape.
        // Note that the vertex locations will be updated at every synchronization time.
        std::string name = "tri_" + std::to_string(it);
        double len = 0.1;

        utils::AddTriangleGeometry(body.get(), material, ChVector<>(len, 0, 0), ChVector<>(0, len, 0),
                                   ChVector<>(0, 0, len), name);
        body->GetCollisionModel()->SetFamily(1);
        body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);

        m_system->AddBody(body);
        m_system->GetCollisionSystem()->BindItem(body);

        proxy->AddBody(body, it);
    }

    m_proxies[i] = proxy;
}

void ChVehicleCosimTerrainNodeGranularOMP::CreateRigidProxy(unsigned int i) {
    // Get shape associated with the given object
    int i_shape = m_obj_map[i];

    // Create the proxy associated with the given object
    auto proxy = chrono_types::make_shared<ProxyBodySet>();

    auto body = chrono_types::make_shared<ChBody>();
    body->SetIdentifier(0);
    body->SetMass(m_load_mass[i]);
    ////body->SetInertiaXX();   //// TODO
    body->SetBodyFixed(m_fixed_proxies);
    body->SetCollide(true);

    // Create visualization assets (use collision shapes)
    m_geometry[i_shape].CreateVisualizationAssets(body, VisualizationType::PRIMITIVES, true);

    // Create collision shapes
    for (auto& mesh : m_geometry[i_shape].m_coll_meshes)
        mesh.m_radius = m_radius_p;
    m_geometry[i_shape].CreateCollisionShapes(body, 1, m_method);
    body->GetCollisionModel()->SetFamily(1);
    body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);

    m_system->AddBody(body);
    m_system->GetCollisionSystem()->BindItem(body);

    proxy->AddBody(body, 0);

    m_proxies[i] = proxy;
}

// Once all proxy bodies are created, complete construction of the underlying system.
void ChVehicleCosimTerrainNodeGranularOMP::OnInitialize(unsigned int num_objects) {
    ChVehicleCosimTerrainNodeChrono::OnInitialize(num_objects);

    // Create the visualization window
    if (m_renderRT) {
#if defined(CHRONO_VSG)
        auto vsys_vsg = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        vsys_vsg->AttachSystem(m_system);
        vsys_vsg->SetWindowTitle("Terrain Node (GranularOMP)");
        vsys_vsg->SetWindowSize(ChVector2<int>(1280, 720));
        vsys_vsg->SetWindowPosition(ChVector2<int>(100, 100));
        vsys_vsg->SetUseSkyBox(false);
        vsys_vsg->SetClearColor(ChColor(0.455f, 0.525f, 0.640f));
        vsys_vsg->AddCamera(m_cam_pos, ChVector<>(0, 0, 0));
        vsys_vsg->SetCameraAngleDeg(40);
        vsys_vsg->SetLightIntensity(1.0f);
        vsys_vsg->SetImageOutputDirectory(m_node_out_dir + "/images");
        vsys_vsg->SetImageOutput(m_writeRT);
        vsys_vsg->Initialize();

        m_vsys = vsys_vsg;
#elif defined(CHRONO_OPENGL)
        auto vsys_gl = chrono_types::make_shared<opengl::ChVisualSystemOpenGL>();
        vsys_gl->AttachSystem(m_system);
        vsys_gl->SetWindowTitle("Terrain Node (GranularOMP)");
        vsys_gl->SetWindowSize(1280, 720);
        vsys_gl->SetRenderMode(opengl::WIREFRAME);
        vsys_gl->Initialize();
        vsys_gl->AddCamera(m_cam_pos, ChVector<>(0, 0, 0));
        vsys_gl->SetCameraProperties(0.05f);
        vsys_gl->SetCameraVertical(CameraVerticalDir::Z);

        m_vsys = vsys_gl;
#endif
    }
}

// Set position, orientation, and velocity of proxy bodies based on mesh faces.
// The proxy body is effectively reconstructed at each synchronization time:
//    - position at the center of mass of the three vertices
//    - orientation: identity
//    - linear and angular velocity: consistent with vertex velocities
//    - contact shape: redefined to match vertex locations
void ChVehicleCosimTerrainNodeGranularOMP::UpdateMeshProxy(unsigned int i, MeshState& mesh_state) {
    // Get shape associated with the given object
    int i_shape = m_obj_map[i];

    // Get the proxy (body set) associated with this object
    auto proxy = std::static_pointer_cast<ProxyBodySet>(m_proxies[i]);

    // Note: it is assumed that there is one and only one mesh defined!
    const auto& trimesh = m_geometry[i_shape].m_coll_meshes[0].m_trimesh;
    const auto& idx_verts = trimesh->getIndicesVertexes();
    int nt = trimesh->getNumTriangles();

    // shape_data contains all triangle vertex locations, in groups of three real3, one group for each triangle.
    auto& shape_data = m_system->data_manager->cd_data->shape_data.triangle_rigid;

    for (int it = 0; it < nt; it++) {
        // Vertex locations (expressed in global frame)
        const ChVector<>& pA = mesh_state.vpos[idx_verts[it].x()];
        const ChVector<>& pB = mesh_state.vpos[idx_verts[it].y()];
        const ChVector<>& pC = mesh_state.vpos[idx_verts[it].z()];

        // Position and orientation of proxy body
        ChVector<> pos = (pA + pB + pC) / 3;
        proxy->bodies[it]->SetPos(pos);
        proxy->bodies[it]->SetRot(ChQuaternion<>(1, 0, 0, 0));

        // Velocity (absolute) and angular velocity (local)
        // These are the solution of an over-determined 9x6 linear system. However, for a centroidal
        // body reference frame, the linear velocity is the average of the 3 vertex velocities.
        // This leaves a 9x3 linear system for the angular velocity which should be solved in a
        // least-square sense:   Ax = b   =>  (A'A)x = A'b
        const ChVector<>& vA = mesh_state.vvel[idx_verts[it].x()];
        const ChVector<>& vB = mesh_state.vvel[idx_verts[it].y()];
        const ChVector<>& vC = mesh_state.vvel[idx_verts[it].z()];

        ChVector<> vel = (vA + vB + vC) / 3;
        proxy->bodies[it]->SetPos_dt(vel);

        //// RADU TODO: angular velocity
        proxy->bodies[it]->SetWvel_loc(ChVector<>(0, 0, 0));

        // Update triangle contact shape (expressed in local frame) by writting directly
        // into the Chrono::Multicore data structures.
        // ATTENTION: It is assumed that no other triangle contact shapes have been added
        // to the system BEFORE those corresponding to the object mesh faces!
        shape_data[3 * it + 0] = real3(pA.x() - pos.x(), pA.y() - pos.y(), pA.z() - pos.z());
        shape_data[3 * it + 1] = real3(pB.x() - pos.x(), pB.y() - pos.y(), pB.z() - pos.z());
        shape_data[3 * it + 2] = real3(pC.x() - pos.x(), pC.y() - pos.y(), pC.z() - pos.z());
    }

    PrintMeshProxiesUpdateData(i, mesh_state);
}

// Set state of proxy rigid body.
void ChVehicleCosimTerrainNodeGranularOMP::UpdateRigidProxy(unsigned int i, BodyState& rigid_state) {
    auto proxy = std::static_pointer_cast<ProxyBodySet>(m_proxies[i]);
    proxy->bodies[0]->SetPos(rigid_state.pos);
    proxy->bodies[0]->SetPos_dt(rigid_state.lin_vel);
    proxy->bodies[0]->SetRot(rigid_state.rot);
    proxy->bodies[0]->SetWvel_par(rigid_state.ang_vel);
}

// Calculate barycentric coordinates (a1, a2, a3) for a given point P
// with respect to the triangle with vertices {v1, v2, v3}
ChVector<> ChVehicleCosimTerrainNodeGranularOMP::CalcBarycentricCoords(const ChVector<>& v1,
                                                                       const ChVector<>& v2,
                                                                       const ChVector<>& v3,
                                                                       const ChVector<>& vP) {
    ChVector<> v12 = v2 - v1;
    ChVector<> v13 = v3 - v1;
    ChVector<> v1P = vP - v1;

    double d_12_12 = Vdot(v12, v12);
    double d_12_13 = Vdot(v12, v13);
    double d_13_13 = Vdot(v13, v13);
    double d_1P_12 = Vdot(v1P, v12);
    double d_1P_13 = Vdot(v1P, v13);

    double denom = d_12_12 * d_13_13 - d_12_13 * d_12_13;

    double a2 = (d_13_13 * d_1P_12 - d_12_13 * d_1P_13) / denom;
    double a3 = (d_12_12 * d_1P_13 - d_12_13 * d_1P_12) / denom;
    double a1 = 1 - a2 - a3;

    return ChVector<>(a1, a2, a3);
}

// Collect contact forces on the (face) proxy bodies that are in contact.
// Load mesh vertex forces and corresponding indices.
void ChVehicleCosimTerrainNodeGranularOMP::GetForceMeshProxy(unsigned int i, MeshContact& mesh_contact) {
    // Get shape associated with the given object
    int i_shape = m_obj_map[i];

    // Get the proxy (body set) associated with this object
    auto proxy = std::static_pointer_cast<ProxyBodySet>(m_proxies[i]);

    // Note: it is assumed that there is one and only one mesh defined!
    const auto& trimesh = m_geometry[i_shape].m_coll_meshes[0].m_trimesh;
    const auto& idx_verts = trimesh->getIndicesVertexes();
    int nt = trimesh->getNumTriangles();

    // Maintain an unordered map of vertex indices and associated contact forces.
    std::unordered_map<int, ChVector<>> my_map;

    for (int it = 0; it < nt; it++) {
        // Get cumulative contact force at triangle centroid.
        // Do nothing if zero force.
        real3 rforce = m_system->GetBodyContactForce(proxy->bodies[it]);
        if (IsZero(rforce))
            continue;

        // Centroid has barycentric coordinates {1/3, 1/3, 1/3}, so force is
        // distributed equally to the three vertices.
        ChVector<> force(rforce.x / 3, rforce.y / 3, rforce.z / 3);

        // For each vertex of the triangle, if it appears in the map, increment
        // the total contact force. Otherwise, insert a new entry in the map.
        auto v1 = my_map.find(idx_verts[it].x());
        if (v1 != my_map.end()) {
            v1->second += force;
        } else {
            my_map[idx_verts[it].x()] = force;
        }

        auto v2 = my_map.find(idx_verts[it].y());
        if (v2 != my_map.end()) {
            v2->second += force;
        } else {
            my_map[idx_verts[it].y()] = force;
        }

        auto v3 = my_map.find(idx_verts[it].z());
        if (v3 != my_map.end()) {
            v3->second += force;
        } else {
            my_map[idx_verts[it].z()] = force;
        }
    }

    // Extract map keys (indices of vertices in contact) and map values
    // (corresponding contact forces) and load output vectors.
    // Note: could improve efficiency by reserving space for vectors.
    mesh_contact.nv = 0;
    for (const auto& kv : my_map) {
        mesh_contact.vidx.push_back(kv.first);
        mesh_contact.vforce.push_back(kv.second);
        mesh_contact.nv++;
    }
}

// Collect resultant contact force and torque on rigid proxy body.
void ChVehicleCosimTerrainNodeGranularOMP::GetForceRigidProxy(unsigned int i, TerrainForce& rigid_contact) {
    auto proxy = std::static_pointer_cast<ProxyBodySet>(m_proxies[i]);
    rigid_contact.point = ChVector<>(0, 0, 0);
    rigid_contact.force = proxy->bodies[0]->GetContactForce();
    rigid_contact.moment = proxy->bodies[0]->GetContactTorque();
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeGranularOMP::OnAdvance(double step_size) {
    ChVehicleCosimTerrainNodeChrono::OnAdvance(step_size);

    // Force a calculation of cumulative contact forces for all bodies in the system
    // (needed at the next synchronization)
    m_system->CalculateContactForces();
}

void ChVehicleCosimTerrainNodeGranularOMP::OnRender() {
    if (!m_vsys)
        return;
    if (!m_vsys->Run())
        MPI_Abort(MPI_COMM_WORLD, 1);

    if (m_track && !m_proxies.empty()) {
        auto proxy = std::static_pointer_cast<ProxyBodySet>(m_proxies[0]);  // proxy for first object
        ChVector<> cam_point = proxy->bodies[0]->GetPos();                  // position of first body in proxy set
        m_vsys->UpdateCamera(m_cam_pos, cam_point);
    }

    m_vsys->BeginScene();
    m_vsys->Render();
    m_vsys->EndScene();
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeGranularOMP::OnOutputData(int frame) {
    // Create and write frame output file.
    std::string filename = OutputFilename(m_node_out_dir + "/simulation", "simulation", "dat", frame + 1, 5);

    utils::CSV_writer csv(" ");
    WriteParticleInformation(csv);
    csv.write_to_file(filename);
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeGranularOMP::WriteParticleInformation(utils::CSV_writer& csv) {
    // Write current time, number of granular particles and their radius
    ////csv << m_system->GetChTime() << endl;
    ////csv << m_num_particles << m_radius_g << endl;

    // Write particle positions and linear velocities
    for (auto body : m_system->Get_bodylist()) {
        if (body->GetIdentifier() < body_id_particles)
            continue;
        ////csv << body->GetIdentifier() << body->GetPos() << body->GetPos_dt() << endl;
        csv << body->GetPos() << body->GetPos_dt() << endl;
    }
}

void ChVehicleCosimTerrainNodeGranularOMP::WriteCheckpoint(const std::string& filename) const {
    utils::CSV_writer csv(" ");

    // Write current time and number of granular material bodies.
    csv << m_system->GetChTime() << endl;
    csv << m_num_particles << endl;

    // Loop over all bodies in the system and write state for granular material bodies.
    // Filter granular material using the body identifier.
    for (auto& body : m_system->Get_bodylist()) {
        if (body->GetIdentifier() < body_id_particles)
            continue;
        csv << body->GetIdentifier() << body->GetPos() << body->GetRot() << body->GetPos_dt() << body->GetRot_dt()
            << endl;
    }

    std::string checkpoint_filename = m_node_out_dir + "/" + filename;
    csv.write_to_file(checkpoint_filename);
    if (m_verbose)
        cout << "[Terrain node] write checkpoint ===> " << checkpoint_filename << endl;
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeGranularOMP::OutputVisualizationData(int frame) {
    auto filename = OutputFilename(m_node_out_dir + "/visualization", "vis", "dat", frame, 5);
    // Include only obstacles and particles
    utils::WriteVisualizationAssets(
        m_system, filename, [](const ChBody& b) -> bool { return b.GetIdentifier() >= body_id_obstacles; }, true);
}

void ChVehicleCosimTerrainNodeGranularOMP::PrintMeshProxiesUpdateData(unsigned int i, const MeshState& mesh_state) {
    {
        auto proxy = std::static_pointer_cast<ProxyBodySet>(m_proxies[i]);

        auto lowest = std::min_element(
            proxy->bodies.begin(), proxy->bodies.end(),
            [](std::shared_ptr<ChBody> a, std::shared_ptr<ChBody> b) { return a->GetPos().z() < b->GetPos().z(); });
        double height = (*lowest)->GetPos().z();
        const ChVector<>& vel = (*lowest)->GetPos_dt();
        cout << "[Terrain node] object: " << i << "  lowest proxy:  height = " << height << "  velocity = " << vel
             << endl;
    }

    {
        auto lowest = std::min_element(mesh_state.vpos.begin(), mesh_state.vpos.end(),
                                       [](const ChVector<>& a, const ChVector<>& b) { return a.z() < b.z(); });
        cout << "[Terrain node] object: " << i << "  lowest vertex:  height = " << (*lowest).z() << endl;
    }
}

}  // end namespace vehicle
}  // end namespace chrono
