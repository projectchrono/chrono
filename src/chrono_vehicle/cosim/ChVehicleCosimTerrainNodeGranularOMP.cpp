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
////    better approximation of mass / inertia? (CreateMeshProxies)
////    angular velocity (UpdateMeshProxies)
////    implement (PrintMeshProxiesContactData)

#include <algorithm>
#include <cmath>
#include <set>
#include <limits>
#include <unordered_map>

#include <omp.h>
#include <mpi.h>

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono_vehicle/cosim/ChVehicleCosimTerrainNodeGranularOMP.h"

#ifdef CHRONO_OPENGL
    #include "chrono_opengl/ChOpenGLWindow.h"
#endif

using std::cout;
using std::endl;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Construction of the terrain node:
// - create the (multicore) Chrono system and set solver parameters
// - create the OpenGL visualization window
// -----------------------------------------------------------------------------
ChVehicleCosimTerrainNodeGranularOMP::ChVehicleCosimTerrainNodeGranularOMP(ChContactMethod method, int num_threads)
    : ChVehicleCosimTerrainNode(Type::GRANULAR_OMP, method),
      m_radius_p(5e-3),
      m_sampling_type(utils::SamplingType::POISSON_DISK),
      m_in_layers(false),
      m_Id_g(10000),
      m_constructed(false),
      m_use_checkpoint(false),
      m_settling_output(false),
      m_settling_fps(100),
      m_num_particles(0) {
    cout << "[Terrain node] GRANULAR_OMP "
         << " method = " << static_cast<std::underlying_type<ChContactMethod>::type>(method)
         << " num_threads = " << num_threads << endl;

    // ------------------------
    // Default model parameters
    // ------------------------

    // Default container wall thickness
    m_hthick = 0.1;

    // Default granular material properties
    m_radius_g = 0.01;
    m_rho_g = 2000;
    m_init_depth = 0.2;
    m_time_settling = 0.4;

    // ---------------------------
    // Create the multicore system
    // ---------------------------

    // Create system and set default method-specific solver settings
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
    m_system->GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_HYBRID_MPR;

    // Set number of threads
    m_system->SetNumThreads(num_threads);

#pragma omp parallel
#pragma omp master
    {
        // Sanity check: print number of threads in a parallel region
        cout << "[Terrain node] actual number of OpenMP threads: " << omp_get_num_threads() << endl;
    }
}

ChVehicleCosimTerrainNodeGranularOMP::~ChVehicleCosimTerrainNodeGranularOMP() {
    delete m_system;
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeGranularOMP::SetWallThickness(double thickness) {
    m_hthick = thickness / 2;
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
                                                             bool in_layers) {
    m_sampling_type = type;
    m_init_depth = init_height;
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

    // Calculate container (half) height
    double separation_factor = 1.2;
    double r = separation_factor * m_radius_g;
    double delta = 2.0f * r;
    double hdimZ = 0.5 * m_init_depth;

    // Estimates for number of bins for broad-phase.
    int factor = 2;
    int binsX = (int)std::ceil(m_hdimX / m_radius_g) / factor;
    int binsY = (int)std::ceil(m_hdimY / m_radius_g) / factor;
    int binsZ = 1;
    m_system->GetSettings()->collision.bins_per_axis = vec3(binsX, binsY, binsZ);
    cout << "[Terrain node] broad-phase bins: " << binsX << " x " << binsY << " x " << binsZ << endl;

    // ---------------------
    // Create container body
    // ---------------------

    auto container = std::shared_ptr<ChBody>(m_system->NewBody());
    m_system->AddBody(container);
    container->SetIdentifier(-1);
    container->SetMass(1);
    container->SetBodyFixed(true);
    container->SetCollide(true);

    container->GetCollisionModel()->ClearModel();
    // Bottom box
    utils::AddBoxGeometry(container.get(), m_material_terrain, ChVector<>(m_hdimX, m_hdimY, m_hthick),
                          ChVector<>(0, 0, -m_hthick), ChQuaternion<>(1, 0, 0, 0), true);
    // Front box
    utils::AddBoxGeometry(container.get(), m_material_terrain, ChVector<>(m_hthick, m_hdimY, hdimZ + m_hthick),
                          ChVector<>(m_hdimX + m_hthick, 0, hdimZ - m_hthick), ChQuaternion<>(1, 0, 0, 0), false);
    // Rear box
    utils::AddBoxGeometry(container.get(), m_material_terrain, ChVector<>(m_hthick, m_hdimY, hdimZ + m_hthick),
                          ChVector<>(-m_hdimX - m_hthick, 0, hdimZ - m_hthick), ChQuaternion<>(1, 0, 0, 0), false);
    // Left box
    utils::AddBoxGeometry(container.get(), m_material_terrain, ChVector<>(m_hdimX, m_hthick, hdimZ + m_hthick),
                          ChVector<>(0, m_hdimY + m_hthick, hdimZ - m_hthick), ChQuaternion<>(1, 0, 0, 0), false);
    // Right box
    utils::AddBoxGeometry(container.get(), m_material_terrain, ChVector<>(m_hdimX, m_hthick, hdimZ + m_hthick),
                          ChVector<>(0, -m_hdimY - m_hthick, hdimZ - m_hthick), ChQuaternion<>(1, 0, 0, 0), false);
    container->GetCollisionModel()->BuildModel();

    // Enable deactivation of bodies that exit a specified bounding box.
    // We set this bounding box to encapsulate the container with a conservative height.
    m_system->GetSettings()->collision.use_aabb_active = true;
    m_system->GetSettings()->collision.aabb_min = real3(-m_hdimX - m_hthick, -m_hdimY - m_hthick, -m_hthick);
    m_system->GetSettings()->collision.aabb_max = real3(+m_hdimX + m_hthick, +m_hdimY + m_hthick, 2 * hdimZ + 2);

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
    gen.setBodyIdentifier(m_Id_g);

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
        ChVector<> hdims(m_hdimX - r, m_hdimY - r, 0);
        double z = delta;
        while (z < m_init_depth) {
            gen.CreateObjectsBox(*sampler, ChVector<>(0, 0, z), hdims);
            cout << "   z =  " << z << "\tnum particles = " << gen.getTotalNumBodies() << endl;
            z += delta;
        }
    } else {
        ChVector<> hdims(m_hdimX - r, m_hdimY - r, m_init_depth / 2 - r);
        gen.CreateObjectsBox(*sampler, ChVector<>(0, 0, m_init_depth / 2), hdims);
    }

    m_num_particles = gen.getTotalNumBodies();
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

        cout << "[Terrain node] read " << checkpoint_filename << "   num. particles = " << num_particles << endl;
    }

    // Find "height" of granular material
    m_init_height = CalcCurrentHeight() + m_radius_g;
    cout << "[Terrain node] initial height = " << m_init_height << endl;

    // Mark system as constructed.
    m_constructed = true;

#ifdef CHRONO_OPENGL
    // Create the visualization window
    if (m_render) {
        opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
        gl_window.Initialize(1280, 720, "Terrain Node (GranularOMP)", m_system);
        gl_window.SetCamera(ChVector<>(0, -3, 0), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), 0.05f);
        gl_window.SetRenderMode(opengl::WIREFRAME);
    }
#endif

    // Write file with terrain node settings
    std::ofstream outf;
    outf.open(m_node_out_dir + "/settings.dat", std::ios::out);
    outf << "System settings" << endl;
    outf << "   Integration step size = " << m_step_size << endl;
    outf << "   Contact method = " << (m_method == ChContactMethod::SMC ? "SMC" : "NSC") << endl;
    outf << "   Use material properties? " << (m_system->GetSettings()->solver.use_material_properties ? "YES" : "NO")
         << endl;
    outf << "   Collision envelope = " << m_system->GetSettings()->collision.collision_envelope << endl;
    outf << "Terrain patch dimensions" << endl;
    outf << "   X = " << 2 * m_hdimX << "  Y = " << 2 * m_hdimY << endl;
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
}

// -----------------------------------------------------------------------------
// Settling phase for the terrain node
// - settle terrain through simulation
// - update initial height of terrain
// -----------------------------------------------------------------------------
void ChVehicleCosimTerrainNodeGranularOMP::Settle() {
    Construct();

    // Packing density at initial configuration
    double eta0 = CalculatePackingDensity();

    // Simulate settling of granular terrain
    int sim_steps = (int)std::ceil(m_time_settling / m_step_size);
    int output_steps = (int)std::ceil(1 / (m_settling_fps * m_step_size));
    int output_frame = 0;

    double render_time = 0;

    for (int is = 0; is < sim_steps; is++) {
        // Advance step
        m_timer.reset();
        m_timer.start();
        m_system->DoStepDynamics(m_step_size);
        m_timer.stop();
        m_cum_sim_time += m_timer();
        cout << '\r' << std::fixed << std::setprecision(6) << m_system->GetChTime() << "  [" << m_timer.GetTimeSeconds()
             << "]" << std::flush;

        // Output (if enabled)
        if (m_settling_output && is % output_steps == 0) {
            char filename[100];
            sprintf(filename, "%s/settling_%04d.dat", m_node_out_dir.c_str(), output_frame + 1);
            utils::CSV_writer csv(" ");
            WriteParticleInformation(csv);
            csv.write_to_file(filename);
            output_frame++;
        }

        // Render (if enabled)
        if (m_render && m_system->GetChTime() > render_time) {
            OnRender(m_system->GetChTime());
            render_time += std::max(m_render_step, m_step_size);
        }
    }

    cout << endl;
    cout << "[Terrain node] settling time = " << m_cum_sim_time << endl;
    m_cum_sim_time = 0;

    // Find "height" of granular material after settling
    m_init_height = CalcCurrentHeight() + m_radius_g;
    cout << "[Terrain node] initial height = " << m_init_height << endl;

    // Packing density after settling
    double eta1 = CalculatePackingDensity();
    cout << "[Terrain node] packing density before and after settling: " << eta0 << " -> " << eta1 << endl;
}

// -----------------------------------------------------------------------------

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
double ChVehicleCosimTerrainNodeGranularOMP::CalculatePackingDensity() {
    // Find height of granular material
    double z_max = CalcCurrentHeight();
    double z_min = 0;

    // Find total volume of granular material
    double Vt = (2 * m_hdimX) * (2 * m_hdimY) * (z_max - z_min);

    // Find volume of granular particles
    double Vs = m_num_particles * (4.0 / 3) * CH_C_PI * std::pow(m_radius_g, 3);

    // Packing density = Vs/Vt
    return Vs / Vt;
}

// -----------------------------------------------------------------------------

// Create bodies with triangular contact geometry as proxies for the tire mesh faces.
// Used for flexible tires.
// Assign to each body an identifier equal to the index of its corresponding mesh face.
// Maintain a list of all bodies associated with the tire.
// Add all proxy bodies to the same collision family and disable collision between any
// two members of this family.
void ChVehicleCosimTerrainNodeGranularOMP::CreateMeshProxies() {
    //// RADU TODO:  better approximation of mass / inertia?
    double mass_p = m_rig_mass / m_mesh_data.nt;
    ChVector<> inertia_p = 1e-3 * mass_p * ChVector<>(0.1, 0.1, 0.1);

    for (unsigned int it = 0; it < m_mesh_data.nt; it++) {
        auto body = std::shared_ptr<ChBody>(m_system->NewBody());
        body->SetIdentifier(it);
        body->SetMass(mass_p);
        body->SetInertiaXX(inertia_p);
        body->SetBodyFixed(m_fixed_proxies);
        body->SetCollide(true);

        // Create contact shape.
        // Note that the vertex locations will be updated at every synchronization time.
        std::string name = "tri_" + std::to_string(it);
        double len = 0.1;

        body->GetCollisionModel()->ClearModel();
        utils::AddTriangleGeometry(body.get(), m_material_tire, ChVector<>(len, 0, 0), ChVector<>(0, len, 0),
                                   ChVector<>(0, 0, len), name);
        body->GetCollisionModel()->SetFamily(1);
        body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
        body->GetCollisionModel()->BuildModel();

        m_system->AddBody(body);
        m_proxies.push_back(ProxyBody(body, it));
    }
}

void ChVehicleCosimTerrainNodeGranularOMP::CreateWheelProxy() {
    auto body = std::shared_ptr<ChBody>(m_system->NewBody());
    body->SetIdentifier(0);
    body->SetMass(m_rig_mass);
    ////body->SetInertiaXX();   //// TODO
    body->SetBodyFixed(m_fixed_proxies);
    body->SetCollide(true);

    // Create collision mesh
    auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
    trimesh->getCoordsVertices() = m_mesh_data.verts;
    trimesh->getCoordsNormals() = m_mesh_data.norms;
    trimesh->getIndicesVertexes() = m_mesh_data.idx_verts;
    trimesh->getIndicesNormals() = m_mesh_data.idx_norms;

    // Set collision shape
    body->GetCollisionModel()->ClearModel();
    body->GetCollisionModel()->AddTriangleMesh(m_material_tire, trimesh, false, false, ChVector<>(0), ChMatrix33<>(1),
                                               m_radius_p);
    body->GetCollisionModel()->SetFamily(1);
    body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
    body->GetCollisionModel()->BuildModel();

    // Set visualization asset
    auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->Pos = ChVector<>(0, 0, 0);
    trimesh_shape->Rot = ChQuaternion<>(1, 0, 0, 0);
    body->GetAssets().push_back(trimesh_shape);

    m_system->AddBody(body);
    m_proxies.push_back(ProxyBody(body, 0));
}

// Set position, orientation, and velocity of proxy bodies based on tire mesh faces.
// The proxy body is effectively reconstructed at each synchronization time:
//    - position at the center of mass of the three vertices
//    - orientation: identity
//    - linear and angular velocity: consistent with vertex velocities
//    - contact shape: redefined to match vertex locations
void ChVehicleCosimTerrainNodeGranularOMP::UpdateMeshProxies() {
    // Readability replacement: shape_data contains all triangle vertex locations, in groups
    // of three real3, one group for each triangle.
    auto& shape_data = m_system->data_manager->shape_data.triangle_rigid;

    for (unsigned int it = 0; it < m_mesh_data.nt; it++) {
        // Vertex locations (expressed in global frame)
        const ChVector<>& pA = m_mesh_state.vpos[m_mesh_data.idx_verts[it].x()];
        const ChVector<>& pB = m_mesh_state.vpos[m_mesh_data.idx_verts[it].y()];
        const ChVector<>& pC = m_mesh_state.vpos[m_mesh_data.idx_verts[it].z()];

        // Position and orientation of proxy body
        ChVector<> pos = (pA + pB + pC) / 3;
        m_proxies[it].m_body->SetPos(pos);
        m_proxies[it].m_body->SetRot(ChQuaternion<>(1, 0, 0, 0));

        // Velocity (absolute) and angular velocity (local)
        // These are the solution of an over-determined 9x6 linear system. However, for a centroidal
        // body reference frame, the linear velocity is the average of the 3 vertex velocities.
        // This leaves a 9x3 linear system for the angular velocity which should be solved in a
        // least-square sense:   Ax = b   =>  (A'A)x = A'b
        const ChVector<>& vA = m_mesh_state.vvel[m_mesh_data.idx_verts[it].x()];
        const ChVector<>& vB = m_mesh_state.vvel[m_mesh_data.idx_verts[it].y()];
        const ChVector<>& vC = m_mesh_state.vvel[m_mesh_data.idx_verts[it].z()];

        ChVector<> vel = (vA + vB + vC) / 3;
        m_proxies[it].m_body->SetPos_dt(vel);

        //// RADU TODO: angular velocity
        m_proxies[it].m_body->SetWvel_loc(ChVector<>(0, 0, 0));

        // Update triangle contact shape (expressed in local frame) by writting directly
        // into the Chrono::Multicore data structures.
        // ATTENTION: It is assumed that no other triangle contact shapes have been added
        // to the system BEFORE those corresponding to the tire mesh faces!
        shape_data[3 * it + 0] = real3(pA.x() - pos.x(), pA.y() - pos.y(), pA.z() - pos.z());
        shape_data[3 * it + 1] = real3(pB.x() - pos.x(), pB.y() - pos.y(), pB.z() - pos.z());
        shape_data[3 * it + 2] = real3(pC.x() - pos.x(), pC.y() - pos.y(), pC.z() - pos.z());
    }
}

// Set state of wheel proxy body.
void ChVehicleCosimTerrainNodeGranularOMP::UpdateWheelProxy() {
    m_proxies[0].m_body->SetPos(m_wheel_state.pos);
    m_proxies[0].m_body->SetPos_dt(m_wheel_state.lin_vel);
    m_proxies[0].m_body->SetRot(m_wheel_state.rot);
    m_proxies[0].m_body->SetWvel_par(m_wheel_state.ang_vel);
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
void ChVehicleCosimTerrainNodeGranularOMP::GetForcesMeshProxies() {
    // Maintain an unordered map of vertex indices and associated contact forces.
    std::unordered_map<int, ChVector<>> my_map;

    for (unsigned int it = 0; it < m_mesh_data.nt; it++) {
        // Get cumulative contact force at triangle centroid.
        // Do nothing if zero force.
        real3 rforce = m_system->GetBodyContactForce(m_proxies[it].m_body);
        if (IsZero(rforce))
            continue;

        // Centroid has barycentric coordinates {1/3, 1/3, 1/3}, so force is
        // distributed equally to the three vertices.
        ChVector<> force(rforce.x / 3, rforce.y / 3, rforce.z / 3);

        // For each vertex of the triangle, if it appears in the map, increment
        // the total contact force. Otherwise, insert a new entry in the map.
        auto v1 = my_map.find(m_mesh_data.idx_verts[it].x());
        if (v1 != my_map.end()) {
            v1->second += force;
        } else {
            my_map[m_mesh_data.idx_verts[it].x()] = force;
        }

        auto v2 = my_map.find(m_mesh_data.idx_verts[it].y());
        if (v2 != my_map.end()) {
            v2->second += force;
        } else {
            my_map[m_mesh_data.idx_verts[it].y()] = force;
        }

        auto v3 = my_map.find(m_mesh_data.idx_verts[it].z());
        if (v3 != my_map.end()) {
            v3->second += force;
        } else {
            my_map[m_mesh_data.idx_verts[it].z()] = force;
        }
    }

    // Extract map keys (indices of vertices in contact) and map values
    // (corresponding contact forces) and load output vectors.
    // Note: could improve efficiency by reserving space for vectors.
    m_mesh_contact.nv = 0;
    for (const auto& kv : my_map) {
        m_mesh_contact.vidx.push_back(kv.first);
        m_mesh_contact.vforce.push_back(kv.second);
        m_mesh_contact.nv++;
    }
}

// Collect resultant contact force and torque on wheel proxy body.
void ChVehicleCosimTerrainNodeGranularOMP::GetForceWheelProxy() {
    m_wheel_contact.point = ChVector<>(0, 0, 0);
    m_wheel_contact.force = m_proxies[0].m_body->GetContactForce();
    m_wheel_contact.moment = m_proxies[0].m_body->GetContactTorque();
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeGranularOMP::OnSynchronize(int step_number, double time) {
    // Nothing needed here
}

void ChVehicleCosimTerrainNodeGranularOMP::OnAdvance(double step_size) {
    // Force a calculation of cumulative contact forces for all bodies in the system
    // (needed at the next synchronization)
    m_system->CalculateContactForces();
}

void ChVehicleCosimTerrainNodeGranularOMP::OnRender(double time) {
#ifdef CHRONO_OPENGL
    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    if (gl_window.Active()) {
        gl_window.Render();
    } else {
        MPI_Abort(MPI_COMM_WORLD, 1);
    }
#endif
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeGranularOMP::OnOutputData(int frame) {
    // Create and write frame output file.
    char filename[100];
    sprintf(filename, "%s/data_%04d.dat", m_node_out_dir.c_str(), frame + 1);

    utils::CSV_writer csv(" ");
    WriteParticleInformation(csv);
    csv.write_to_file(filename);
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeGranularOMP::WriteParticleInformation(utils::CSV_writer& csv) {
    // Write current time, number of granular particles and their radius
    csv << m_system->GetChTime() << endl;
    csv << m_num_particles << m_radius_g << endl;

    // Write particle positions and linear velocities
    for (auto body : m_system->Get_bodylist()) {
        if (body->GetIdentifier() < m_Id_g)
            continue;
        csv << body->GetIdentifier() << body->GetPos() << body->GetPos_dt() << endl;
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
        if (body->GetIdentifier() < m_Id_g)
            continue;
        csv << body->GetIdentifier() << body->GetPos() << body->GetRot() << body->GetPos_dt() << body->GetRot_dt()
            << endl;
    }

    std::string checkpoint_filename = m_node_out_dir + "/" + filename;
    csv.write_to_file(checkpoint_filename);
    cout << "[Terrain node] write checkpoint ===> " << checkpoint_filename << endl;
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeGranularOMP::PrintMeshProxiesUpdateData() {
    {
        auto lowest = std::min_element(m_proxies.begin(), m_proxies.end(), [](const ProxyBody& a, const ProxyBody& b) {
            return a.m_body->GetPos().z() < b.m_body->GetPos().z();
        });
        const ChVector<>& vel = (*lowest).m_body->GetPos_dt();
        double height = (*lowest).m_body->GetPos().z();
        cout << "[Terrain node] lowest proxy:  index = " << (*lowest).m_index << "  height = " << height
             << "  velocity = " << vel.x() << "  " << vel.y() << "  " << vel.z() << endl;
    }

    {
        auto lowest = std::min_element(m_mesh_state.vpos.begin(), m_mesh_state.vpos.end(),
                                       [](const ChVector<>& a, const ChVector<>& b) { return a.z() < b.z(); });
        cout << "[Terrain node] lowest vertex:  height = " << (*lowest).z() << endl;
    }
}

void ChVehicleCosimTerrainNodeGranularOMP::PrintWheelProxyUpdateData() {
    //// TODO
}

void ChVehicleCosimTerrainNodeGranularOMP::PrintMeshProxiesContactData() {
    //// RADU TODO: implement this
}

void ChVehicleCosimTerrainNodeGranularOMP::PrintWheelProxyContactData() {
    //// RADU TODO: implement this
}

}  // end namespace vehicle
}  // end namespace chrono
