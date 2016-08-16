// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2015 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Antonio Recuero
// =============================================================================
//
// Definition of the TERRAIN NODE.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <algorithm>
#include <cmath>
#include <set>
#include <unordered_map>

#include <omp.h>
#include "mpi.h"

#include "chrono/ChConfig.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

#include "TerrainNode.h"

using std::cout;
using std::endl;

using namespace chrono;

const std::string TerrainNode::m_checkpoint_filename = "checkpoint.dat";

// -----------------------------------------------------------------------------
// Construction of the terrain node:
// - create the (parallel) Chrono system and set solver parameters
// - create the OpenGL visualization window
// -----------------------------------------------------------------------------
TerrainNode::TerrainNode(Type type,
                         ChMaterialSurfaceBase::ContactMethod method,
                         int num_tires,
                         bool use_checkpoint,
                         bool render,
                         int num_threads)
    : BaseNode("TERRAIN"),
      m_type(type),
      m_method(method),
      m_num_tires(num_tires),
      m_use_checkpoint(use_checkpoint),
      m_render(render),
      m_constructed(false),
      m_settling_output(false),
      m_num_particles(0),
      m_particles_start_index(0),
      m_proxy_start_index(0),
      m_init_height(0) {
    m_prefix = "[Terrain node]";

    cout << m_prefix << " type = " << type << " method = " << method << " use_checkpoint = " << use_checkpoint
         << " num_threads = " << num_threads << endl;

    m_tire_data.resize(m_num_tires);

    // ------------------------
    // Default model parameters
    // ------------------------

    // Default container dimensions
    m_hdimX = 1.0;
    m_hdimY = 0.25;
    m_hdimZ = 0.5;
    m_hthick = 0.1;

    // Default granular material properties
    m_radius_g = 0.01;
    m_rho_g = 2000;
    m_num_layers = 5;
    m_time_settling = 0.4;

    // Default proxy body properties
    m_fixed_proxies = false;
    m_mass_pN = 1;
    m_radius_pN = 0.01;
    m_mass_pF = 1;

    // Default terrain contact material
    switch (m_method) {
        case ChMaterialSurfaceBase::DEM:
            m_material_terrain = std::make_shared<ChMaterialSurfaceDEM>();
            break;
        case ChMaterialSurfaceBase::DVI:
            m_material_terrain = std::make_shared<ChMaterialSurface>();
            break;
    }

    // --------------------------
    // Create the parallel system
    // --------------------------

    // Create system and set default method-specific solver settings
    switch (m_method) {
        case ChMaterialSurfaceBase::DEM: {
            ChSystemParallelDEM* sys = new ChSystemParallelDEM;
            sys->GetSettings()->solver.contact_force_model = ChSystemDEM::Hertz;
            sys->GetSettings()->solver.tangential_displ_mode = ChSystemDEM::TangentialDisplacementModel::OneStep;
            sys->GetSettings()->solver.use_material_properties = true;
            m_system = sys;

            break;
        }
        case ChMaterialSurfaceBase::DVI: {
            ChSystemParallelDVI* sys = new ChSystemParallelDVI;
            sys->GetSettings()->solver.solver_mode = SLIDING;
            sys->GetSettings()->solver.max_iteration_normal = 0;
            sys->GetSettings()->solver.max_iteration_sliding = 200;
            sys->GetSettings()->solver.max_iteration_spinning = 0;
            sys->GetSettings()->solver.alpha = 0;
            sys->GetSettings()->solver.contact_recovery_speed = -1;
            sys->GetSettings()->collision.collision_envelope = 0.001;
            sys->ChangeSolverType(APGD);
            m_system = sys;

            break;
        }
    }

    // Solver settings independent of method type
    m_system->Set_G_acc(ChVector<>(0, 0, m_gacc));
    m_system->GetSettings()->perform_thread_tuning = false;
    m_system->GetSettings()->solver.use_full_inertia_tensor = false;
    m_system->GetSettings()->solver.tolerance = 0.1;
    m_system->GetSettings()->solver.max_iteration_bilateral = 100;
    m_system->GetSettings()->collision.narrowphase_algorithm = NARROWPHASE_HYBRID_MPR;

    // Set number of threads
    m_system->SetParallelThreadNumber(num_threads);
    CHOMPfunctions::SetNumThreads(num_threads);

#pragma omp parallel
#pragma omp master
    {
        // Sanity check: print number of threads in a parallel region
        cout << m_prefix << " actual number of OpenMP threads: " << omp_get_num_threads() << endl;
    }

#ifdef CHRONO_OPENGL
    // -------------------------------
    // Create the visualization window
    // -------------------------------

    if (m_render) {
        opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
        gl_window.Initialize(1280, 720, "Terrain Node", m_system);
        gl_window.SetCamera(ChVector<>(0, -4, 0), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), 0.05f);
        gl_window.SetRenderMode(opengl::WIREFRAME);
    }
#endif
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
TerrainNode::~TerrainNode() {
    delete m_system;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void TerrainNode::SetContainerDimensions(double length, double width, double height, double thickness) {
    m_hdimX = length / 2;
    m_hdimY = width / 2;
    m_hdimZ = height / 2;
    m_hthick = thickness / 2;
}

void TerrainNode::SetGranularMaterial(double radius, double density, int num_layers) {
    assert(m_type == GRANULAR);
    m_radius_g = radius;
    m_rho_g = density;
    m_num_layers = num_layers;
    m_system->GetSettings()->collision.collision_envelope = 0.1 * radius;
}

void TerrainNode::UseMaterialProperties(bool flag) {
    assert(m_system->GetContactMethod() == ChMaterialSurfaceBase::DEM);
    m_system->GetSettings()->solver.use_material_properties = flag;
}

void TerrainNode::SetContactForceModel(ChSystemDEM::ContactForceModel model) {
    assert(m_system->GetContactMethod() == ChMaterialSurfaceBase::DEM);
    m_system->GetSettings()->solver.contact_force_model = model;
}

void TerrainNode::SetMaterialSurface(const std::shared_ptr<ChMaterialSurfaceBase>& mat) {
    assert(mat->GetContactMethod() == m_system->GetContactMethod());
    m_material_terrain = mat;
}

void TerrainNode::SetProxyProperties(double mass, double radius, bool fixed) {
    assert(m_type == RIGID);
    m_mass_pN = mass;
    m_radius_pN = radius;
    m_fixed_proxies = fixed;
}

void TerrainNode::SetProxyProperties(double mass, bool fixed) {
    assert(m_type == GRANULAR);
    m_mass_pF = mass;
    m_fixed_proxies = fixed;
}

// -----------------------------------------------------------------------------
// Complete construction of the mechanical system.
// This function is invoked automatically from Settle and Initialize.
// - adjust system settings
// - create the container body
// - if specified, create the granular material
// -----------------------------------------------------------------------------
void TerrainNode::Construct() {
    if (m_constructed)
        return;

    // Estimates for number of bins for broad-phase.
    int factor = 2;
    int binsX = (int)std::ceil(m_hdimX / m_radius_g) / factor;
    int binsY = (int)std::ceil(m_hdimY / m_radius_g) / factor;
    int binsZ = 1;
    m_system->GetSettings()->collision.bins_per_axis = I3(binsX, binsY, binsZ);
    cout << m_prefix << " broad-phase bins: " << binsX << " x " << binsY << " x " << binsZ << endl;

    // ---------------------
    // Create container body
    // ---------------------

    auto container = std::shared_ptr<ChBody>(m_system->NewBody());
    m_system->AddBody(container);
    container->SetIdentifier(-1);
    container->SetMass(1);
    container->SetBodyFixed(true);
    container->SetCollide(true);
    container->SetMaterialSurface(m_material_terrain);

    container->GetCollisionModel()->ClearModel();
    // Bottom box
    utils::AddBoxGeometry(container.get(), ChVector<>(m_hdimX, m_hdimY, m_hthick), ChVector<>(0, 0, -m_hthick),
                          ChQuaternion<>(1, 0, 0, 0), true);
    // Front box
    utils::AddBoxGeometry(container.get(), ChVector<>(m_hthick, m_hdimY, m_hdimZ + m_hthick),
                          ChVector<>(m_hdimX + m_hthick, 0, m_hdimZ - m_hthick), ChQuaternion<>(1, 0, 0, 0), false);
    // Rear box
    utils::AddBoxGeometry(container.get(), ChVector<>(m_hthick, m_hdimY, m_hdimZ + m_hthick),
                          ChVector<>(-m_hdimX - m_hthick, 0, m_hdimZ - m_hthick), ChQuaternion<>(1, 0, 0, 0), false);
    // Left box
    utils::AddBoxGeometry(container.get(), ChVector<>(m_hdimX, m_hthick, m_hdimZ + m_hthick),
                          ChVector<>(0, m_hdimY + m_hthick, m_hdimZ - m_hthick), ChQuaternion<>(1, 0, 0, 0), false);
    // Right box
    utils::AddBoxGeometry(container.get(), ChVector<>(m_hdimX, m_hthick, m_hdimZ + m_hthick),
                          ChVector<>(0, -m_hdimY - m_hthick, m_hdimZ - m_hthick), ChQuaternion<>(1, 0, 0, 0), false);
    container->GetCollisionModel()->BuildModel();

    // Enable deactivation of bodies that exit a specified bounding box.
    // We set this bounding box to encapsulate the container with a conservative height.
    m_system->GetSettings()->collision.use_aabb_active = true;
    m_system->GetSettings()->collision.aabb_min = real3(-m_hdimX - m_hthick, -m_hdimY - m_hthick, -m_hthick);
    m_system->GetSettings()->collision.aabb_max = real3(m_hdimX + m_hthick, m_hdimY + m_hthick, 2 * m_hdimZ + 2);

    // If using RIGID terrain, the contact will be between the container and proxy bodies.
    // Since collision between two bodies fixed to ground is ignored, if the proxy bodies
    // are fixed, we make the container a free body connected through a weld joint to ground.
    // If using GRANULAR terrain, this is not an issue as the proxy bodies do not interact
    // with the container, but rather with the granular material.
    if (m_type == RIGID && m_fixed_proxies) {
        container->SetBodyFixed(false);

        auto ground = std::shared_ptr<ChBody>(m_system->NewBody());
        ground->SetIdentifier(-2);
        ground->SetBodyFixed(true);
        ground->SetCollide(false);
        m_system->AddBody(ground);

        auto weld = std::make_shared<ChLinkLockLock>();
        weld->Initialize(ground, container, ChCoordsys<>(VNULL, QUNIT));
        m_system->AddLink(weld);
    }

    // --------------------------
    // Generate granular material
    // --------------------------

    // Granular material properties.
    m_Id_g = 10000;

    // Cache the number of bodies that have been added so far to the parallel system.
    // ATTENTION: This will be used to set the state of granular material particles if
    // initializing them from a checkpoint file.

    m_particles_start_index = m_system->data_manager->num_rigid_bodies;

    // Create particles
    if (m_type == GRANULAR) {
        // Create a particle generator and a mixture entirely made out of spheres
        utils::Generator gen(m_system);
        std::shared_ptr<utils::MixtureIngredient> m1 = gen.AddMixtureIngredient(utils::SPHERE, 1.0);
        m1->setDefaultMaterial(m_material_terrain);
        m1->setDefaultDensity(m_rho_g);
        m1->setDefaultSize(m_radius_g);

        // Set starting value for body identifiers
        gen.setBodyIdentifier(m_Id_g);

        // Create particles in layers until reaching the desired number of particles
        double r = 1.01 * m_radius_g;
        ChVector<> hdims(m_hdimX - r, m_hdimY - r, 0);
        ChVector<> center(0, 0, 2 * r);

        for (int il = 0; il < m_num_layers; il++) {
            gen.createObjectsBox(utils::POISSON_DISK, 2 * r, center, hdims);
            center.z += 2 * r;
        }

        m_num_particles = gen.getTotalNumBodies();
        cout << m_prefix << " Generated particles:  " << m_num_particles << endl;
    }

    // Cache the number of contact shapes that have been added so far to the parallel system.
    // ATTENTION: This will be used to index into the various global arrays to access/modify
    // information on contact shapes for the proxy bodies.  The implicit assumption here is
    // that *NO OTHER CONTACT SHAPES* are created before the proxy bodies!

    m_proxy_start_index = m_system->data_manager->num_rigid_shapes;

    // --------------------------------------
    // Write file with terrain node settings
    // --------------------------------------

    std::ofstream outf;
    outf.open(m_node_out_dir + "/settings.dat", std::ios::out);
    outf << "Terrain type = " << (m_type == RIGID ? "RIGID" : "GRANULAR") << endl;
    outf << "System settings" << endl;
    outf << "   Integration step size = " << m_step_size << endl;
    outf << "   Contact method = " << (m_method == ChMaterialSurfaceBase::DEM ? "DEM" : "DVI") << endl;
    outf << "   Use material properties? " << (m_system->GetSettings()->solver.use_material_properties ? "YES" : "NO")
         << endl;
    outf << "   Collision envelope = " << m_system->GetSettings()->collision.collision_envelope << endl;
    outf << "Container dimensions" << endl;
    outf << "   X = " << 2 * m_hdimX << "  Y = " << 2 * m_hdimY << "  Z = " << 2 * m_hdimZ << endl;
    outf << "   wall thickness = " << 2 * m_hthick << endl;
    outf << "Terrain material properties" << endl;
    switch (m_method) {
        case ChMaterialSurfaceBase::DEM: {
            auto mat = std::static_pointer_cast<ChMaterialSurfaceDEM>(m_material_terrain);
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
        case ChMaterialSurfaceBase::DVI: {
            auto mat = std::static_pointer_cast<ChMaterialSurface>(m_material_terrain);
            outf << "   Coefficient of friction    = " << mat->GetKfriction() << endl;
            outf << "   Coefficient of restitution = " << mat->GetRestitution() << endl;
            outf << "   Cohesion force             = " << mat->GetCohesion() << endl;
            break;
        }
    }
    outf << "Granular material properties" << endl;
    outf << "   particle radius  = " << m_radius_g << endl;
    outf << "   particle density = " << m_rho_g << endl;
    outf << "   number layers    = " << m_num_layers << endl;
    outf << "   number particles = " << m_num_particles << endl;
    outf << "Proxy body properties" << endl;
    outf << "   proxies fixed? " << (m_fixed_proxies ? "YES" : "NO") << endl;
    switch (m_type) {
        case RIGID:
            outf << "   proxy radius = " << m_radius_pN << endl;
            outf << "   proxy mass = " << m_mass_pN << endl;
            break;
        case GRANULAR:
            outf << "   proxy mass = " << m_mass_pF << endl;
    }

    // Mark system as constructed.
    m_constructed = true;
}

// -----------------------------------------------------------------------------
// Settling phase for the terrain node
// - if not already done, complete system construction
// - simulate granular material to settle or read from checkpoint
// - record height of terrain
// -----------------------------------------------------------------------------
void TerrainNode::Settle() {
    assert(m_type == GRANULAR);

    Construct();

    if (m_use_checkpoint) {
        // ------------------------------------------------
        // Initialize granular terrain from checkpoint file
        // ------------------------------------------------

        // Open input file stream
        std::string checkpoint_filename = m_out_dir + "/" + m_checkpoint_filename;
        std::ifstream ifile(checkpoint_filename);
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
        for (int ib = m_particles_start_index; ib < m_system->Get_bodylist()->size(); ++ib) {
            std::getline(ifile, line);
            std::istringstream iss(line);
            int identifier;
            ChVector<> pos;
            ChQuaternion<> rot;
            ChVector<> pos_dt;
            ChQuaternion<> rot_dt;
            iss >> identifier >> pos.x >> pos.y >> pos.z >> rot.e0 >> rot.e1 >> rot.e2 >> rot.e3 >> pos_dt.x >>
                pos_dt.y >> pos_dt.z >> rot_dt.e0 >> rot_dt.e1 >> rot_dt.e2 >> rot_dt.e3;

            auto body = (*m_system->Get_bodylist())[ib];
            assert(body->GetIdentifier() == identifier);
            body->SetPos(ChVector<>(pos.x, pos.y, pos.z));
            body->SetRot(ChQuaternion<>(rot.e0, rot.e1, rot.e2, rot.e3));
            body->SetPos_dt(ChVector<>(pos_dt.x, pos_dt.y, pos_dt.z));
            body->SetRot_dt(ChQuaternion<>(rot_dt.e0, rot_dt.e1, rot_dt.e2, rot_dt.e3));
        }

        cout << m_prefix << " read checkpoint <=== " << checkpoint_filename << "   num. particles = " << num_particles
             << endl;

    } else {
        // -------------------------------------
        // Simulate settling of granular terrain
        // -------------------------------------
        double output_fps = 100;
        int sim_steps = (int)std::ceil(m_time_settling / m_step_size);
        int output_steps = (int)std::ceil(1 / (output_fps * m_step_size));
        int output_frame = 0;

        for (int is = 0; is < sim_steps; is++) {
            // Advance step
            m_timer.reset();
            m_timer.start();
            m_system->DoStepDynamics(m_step_size);
            m_timer.stop();
            m_cum_sim_time += m_timer();
            cout << '\r' << std::fixed << std::setprecision(6) << m_system->GetChTime() << "  ["
                 << m_timer.GetTimeSeconds() << "]" << std::flush;

            // Output (if enabled)
            if (m_settling_output && is % output_steps == 0) {
                char filename[100];
                sprintf(filename, "%s/settling_%04d.dat", m_node_out_dir.c_str(), output_frame + 1);
                utils::CSV_writer csv(" ");
                WriteParticleInformation(csv);
                csv.write_to_file(filename);
                output_frame++;
            }

#ifdef CHRONO_OPENGL
            // OpenGL rendering
            if (m_render) {
                opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
                if (gl_window.Active()) {
                    gl_window.Render();
                } else {
                    MPI_Abort(MPI_COMM_WORLD, 1);
                }
            }
#endif
        }

        cout << m_prefix << " settling time = " << m_cum_sim_time << endl;
        m_cum_sim_time = 0;
    }

    // Find "height" of granular material
    m_init_height = 0;
    for (auto body : *m_system->Get_bodylist()) {
        if (body->GetIdentifier() > 0 && body->GetPos().z > m_init_height)
            m_init_height = body->GetPos().z;
    }
    m_init_height += m_radius_g;
}

// -----------------------------------------------------------------------------
// Initialization of the terrain node:
// - if not already done, complete system construction
// - send information on terrain height
// - receive information on tire mesh topology (number vertices and triangles)
// - receive tire contact material properties and create the "tire" material
// - create the appropriate proxy bodies (state not set yet)
// -----------------------------------------------------------------------------
void TerrainNode::Initialize() {
    Construct();

    // Reset system time
    m_system->SetChTime(0);
    
    // Send information for initial vehicle position
    double init_dim[2] = { m_init_height, m_hdimX };
    MPI_Send(init_dim, 2, MPI_DOUBLE, VEHICLE_NODE_RANK, 0, MPI_COMM_WORLD);

    cout << m_prefix << " Sent initial terrain height = " << init_dim[0] << endl;
    cout << m_prefix << " Sent container half-length = " << init_dim[1] << endl;

#ifdef CHRONO_OPENGL
    // Move OpenGL camera
    if (m_render) {
        opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
        gl_window.SetCamera(ChVector<>(2.75 - m_hdimX, -4, 0), ChVector<>(2.75 - m_hdimX, 0, 0), ChVector<>(0, 0, 1), 0.05f);
    }

#endif


    // Loop over all tires, receive information, create proxies.
    unsigned int start_vert_index = 0;
    unsigned int start_tri_index = 0;

    for (int which = 0; which < m_num_tires; which++) {

        // Receive tire contact surface specification.
        unsigned int surf_props[2];
        MPI_Status status_p;
        MPI_Recv(surf_props, 2, MPI_UNSIGNED, TIRE_NODE_RANK(which), 0, MPI_COMM_WORLD, &status_p);
        m_tire_data[which].m_num_vert = surf_props[0];
        m_tire_data[which].m_num_tri = surf_props[1];

        m_tire_data[which].m_vertex_states.resize(surf_props[0]);
        m_tire_data[which].m_triangles.resize(surf_props[1]);

        m_tire_data[which].m_start_vert = start_vert_index;
        m_tire_data[which].m_start_tri = start_tri_index;
        start_vert_index += surf_props[0];
        start_tri_index += surf_props[1];

        cout << m_prefix << " Received vertices = " << surf_props[0] << " triangles = " << surf_props[1] << endl;

        // Receive tire contact material properties.
        // Create the "tire" contact material, but defer using it until the proxy bodies are created.
        float mat_props[8];
        MPI_Status status_m;
        MPI_Recv(mat_props, 8, MPI_FLOAT, TIRE_NODE_RANK(which), 0, MPI_COMM_WORLD, &status_m);

        switch (m_method) {
        case ChMaterialSurfaceBase::DEM: {
            // Properties for tire
            auto mat_tire = std::make_shared<ChMaterialSurfaceDEM>();
            mat_tire->SetFriction(mat_props[0]);
            mat_tire->SetRestitution(mat_props[1]);
            mat_tire->SetYoungModulus(mat_props[2]);
            mat_tire->SetPoissonRatio(mat_props[3]);
            mat_tire->SetKn(mat_props[4]);
            mat_tire->SetGn(mat_props[5]);
            mat_tire->SetKt(mat_props[6]);
            mat_tire->SetGt(mat_props[7]);

            m_tire_data[which].m_material_tire = mat_tire;

            break;
        }
        case ChMaterialSurfaceBase::DVI: {
            auto mat_tire = std::make_shared<ChMaterialSurface>();
            mat_tire->SetFriction(mat_props[0]);
            mat_tire->SetRestitution(mat_props[1]);

            m_tire_data[which].m_material_tire = mat_tire;

            break;
        }
        }

        cout << m_prefix << " received tire material:  friction = " << mat_props[0] << endl;

        // -------------------
        // Create proxy bodies
        // -------------------

        switch (m_type) {
        case RIGID:
            // For contact with rigid ground, represent the tire as spheres associated with mesh vertices.
            CreateNodeProxies(which);
            break;
        case GRANULAR:
            // For contact with granular terrain, represent the tire as triangles associated with mesh faces.
            CreateFaceProxies(which);
            break;
        }

    }
}

// Create bodies with spherical contact geometry as proxies for the tire mesh vertices.
// Assign to each body an identifier equal to the index of its corresponding mesh vertex.
// Maintain a list of all bodies associated with the tire.
// Add all proxy bodies to the same collision family and disable collision between any
// two members of this family.
void TerrainNode::CreateNodeProxies(int which) {
    ChVector<> inertia_pN = 0.4 * m_mass_pN * m_radius_pN * m_radius_pN * ChVector<>(1, 1, 1);
    for (unsigned int iv = 0; iv < m_tire_data[which].m_num_vert; iv++) {
        auto body = std::shared_ptr<ChBody>(m_system->NewBody());
        m_system->AddBody(body);
        body->SetIdentifier(m_tire_data[which].m_start_vert + iv);
        body->SetMass(m_mass_pN);
        body->SetInertiaXX(inertia_pN);
        body->SetBodyFixed(m_fixed_proxies);
        body->SetCollide(true);
        body->SetMaterialSurface(m_tire_data[which].m_material_tire);

        body->GetCollisionModel()->ClearModel();
        utils::AddSphereGeometry(body.get(), m_radius_pN, ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0), true);
        body->GetCollisionModel()->SetFamily(1);
        body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
        body->GetCollisionModel()->BuildModel();

        m_tire_data[which].m_proxies.push_back(ProxyBody(body, iv));
    }
}

// Create bodies with triangular contact geometry as proxies for the tire mesh faces.
// Assign to each body an identifier equal to the index of its corresponding mesh face.
// Maintain a list of all bodies associated with the tire.
// Add all proxy bodies to the same collision family and disable collision between any
// two members of this family.
void TerrainNode::CreateFaceProxies(int which) {
    //// TODO:  better approximation of mass / inertia?
    ChVector<> inertia_pF = 1e-3 * m_mass_pF * ChVector<>(0.1, 0.1, 0.1);

    for (unsigned int it = 0; it < m_tire_data[which].m_num_tri; it++) {
        auto body = std::shared_ptr<ChBody>(m_system->NewBody());
        body->SetIdentifier(m_tire_data[which].m_start_tri + it);
        body->SetMass(m_mass_pF);

        body->SetInertiaXX(inertia_pF);
        body->SetBodyFixed(m_fixed_proxies);
        body->SetCollide(true);
        body->SetMaterialSurface(m_tire_data[which].m_material_tire);

        // Create contact shape.
        // Note that the vertex locations will be updated at every synchronization time.
        std::string name = "tri_" + std::to_string(m_tire_data[which].m_start_tri + it);
        double len = 0.1;

        body->GetCollisionModel()->ClearModel();
        utils::AddTriangle(body.get(), ChVector<>(len, 0, 0), ChVector<>(0, len, 0), ChVector<>(0, 0, len), name);
        body->GetCollisionModel()->SetFamily(1);
        body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
        body->GetCollisionModel()->BuildModel();

        m_tire_data[which].m_proxies.push_back(ProxyBody(body, it));

        m_system->AddBody(body);
    }
}

// -----------------------------------------------------------------------------
// Synchronization of the terrain node:
// - receive tire mesh vertex states and set states of proxy bodies
// - calculate current cumulative contact forces on all system bodies
// - extract and send forces at each vertex
// -----------------------------------------------------------------------------
void TerrainNode::Synchronize(int step_number, double time) {

    // --------------------------------------------------------------
    // Loop over all tires, receive mesh vertex state, update proxies
    // --------------------------------------------------------------

    for (int which = 0; which < m_num_tires; which++) {

        // Receive tire mesh vertex locations and velocities.
        MPI_Status status;
        unsigned int num_vert = m_tire_data[which].m_num_vert;
        unsigned int num_tri = m_tire_data[which].m_num_tri;
        double* vert_data = new double[2 * 3 * num_vert];
        int* tri_data = new int[3 * num_tri];
        MPI_Recv(vert_data, 2 * 3 * num_vert, MPI_DOUBLE, TIRE_NODE_RANK(which), step_number, MPI_COMM_WORLD, &status);
        MPI_Recv(tri_data, 3 * num_tri, MPI_INT, TIRE_NODE_RANK(which), step_number, MPI_COMM_WORLD, &status);

        for (unsigned int iv = 0; iv < num_vert; iv++) {
            unsigned int offset = 3 * iv;
            m_tire_data[which].m_vertex_states[iv].pos = ChVector<>(vert_data[offset + 0], vert_data[offset + 1], vert_data[offset + 2]);
            offset += 3 * num_vert;
            m_tire_data[which].m_vertex_states[iv].vel = ChVector<>(vert_data[offset + 0], vert_data[offset + 1], vert_data[offset + 2]);
        }

        for (unsigned int it = 0; it < num_tri; it++) {
            m_tire_data[which].m_triangles[it].v1 = tri_data[3 * it + 0];
            m_tire_data[which].m_triangles[it].v2 = tri_data[3 * it + 1];
            m_tire_data[which].m_triangles[it].v3 = tri_data[3 * it + 2];
        }

        delete[] vert_data;
        delete[] tri_data;

        // Set position, rotation, and velocity of proxy bodies.
        switch (m_type) {
        case RIGID:
            UpdateNodeProxies(which);
            PrintNodeProxiesUpdateData(which);
            break;
        case GRANULAR:
            UpdateFaceProxies(which);
            PrintFaceProxiesUpdateData(which);
            break;
        }
    }

    // ------------------------------------------------------------
    // Calculate cumulative contact forces for all bodies in system
    // ------------------------------------------------------------

    m_system->CalculateContactForces();

    std::string msg =
        " step number: " + std::to_string(step_number) + "  num contacts: " + std::to_string(m_system->GetNcontacts());

    // -----------------------------------------------------------------
    // Loop over all tires, calculate vertex contact forces, send forces
    // -----------------------------------------------------------------

    msg += "  [  ";

    for (int which = 0; which < m_num_tires; which++) {

        // Collect contact forces on subset of mesh vertices.
        // Note that no forces are collected at the first step.
        std::vector<double> vert_forces;
        std::vector<int> vert_indices;

        if (step_number > 0) {
            switch (m_type) {
            case RIGID:
                ForcesNodeProxies(which, vert_forces, vert_indices);
                break;
            case GRANULAR:
                ForcesFaceProxies(which, vert_forces, vert_indices);
                break;
            }
        }

        // Send vertex indices and forces.
        int num_vert = (int)vert_indices.size();
        MPI_Send(vert_indices.data(), num_vert, MPI_INT, TIRE_NODE_RANK(which), step_number, MPI_COMM_WORLD);
        MPI_Send(vert_forces.data(), 3 * num_vert, MPI_DOUBLE, TIRE_NODE_RANK(which), step_number, MPI_COMM_WORLD);

        msg += std::to_string(num_vert) + "  ";
    }

    msg += "]";
    cout << m_prefix << msg << endl;

}

// Set position and velocity of proxy bodies based on tire mesh vertices.
// Set orientation to identity and angular velocity to zero.
void TerrainNode::UpdateNodeProxies(int which) {
    for (unsigned int iv = 0; iv < m_tire_data[which].m_num_vert; iv++) {
        m_tire_data[which].m_proxies[iv].m_body->SetPos(m_tire_data[which].m_vertex_states[iv].pos);
        m_tire_data[which].m_proxies[iv].m_body->SetPos_dt(m_tire_data[which].m_vertex_states[iv].vel);
        m_tire_data[which].m_proxies[iv].m_body->SetRot(ChQuaternion<>(1, 0, 0, 0));
        m_tire_data[which].m_proxies[iv].m_body->SetRot_dt(ChQuaternion<>(0, 0, 0, 0));
    }
}

// Set position, orientation, and velocity of proxy bodies based on tire mesh faces.
// The proxy body is effectively reconstructed at each synchronization time:
//    - position at the center of mass of the three vertices
//    - orientation: identity
//    - linear and angular velocity: consistent with vertex velocities
//    - contact shape: redefined to match vertex locations
void TerrainNode::UpdateFaceProxies(int which) {
    // Readability replacements
    auto& dataA = m_system->data_manager->host_data.ObA_rigid;  // all first vertices
    auto& dataB = m_system->data_manager->host_data.ObB_rigid;  // all second vertices
    auto& dataC = m_system->data_manager->host_data.ObC_rigid;  // all third vertices

    for (unsigned int it = 0; it < m_tire_data[which].m_num_tri; it++) {
        Triangle tri = m_tire_data[which].m_triangles[it];

        // Vertex locations (expressed in global frame)
        const ChVector<>& pA = m_tire_data[which].m_vertex_states[tri.v1].pos;
        const ChVector<>& pB = m_tire_data[which].m_vertex_states[tri.v2].pos;
        const ChVector<>& pC = m_tire_data[which].m_vertex_states[tri.v3].pos;

        // Position and orientation of proxy body
        ChVector<> pos = (pA + pB + pC) / 3;
        m_tire_data[which].m_proxies[it].m_body->SetPos(pos);
        m_tire_data[which].m_proxies[it].m_body->SetRot(ChQuaternion<>(1, 0, 0, 0));

        // Velocity (absolute) and angular velocity (local)
        // These are the solution of an over-determined 9x6 linear system. However, for a centroidal
        // body reference frame, the linear velocity is the average of the 3 vertex velocities.
        // This leaves a 9x3 linear system for the angular velocity which should be solved in a
        // least-square sense:   Ax = b   =>  (A'A)x = A'b
        const ChVector<>& vA = m_tire_data[which].m_vertex_states[tri.v1].vel;
        const ChVector<>& vB = m_tire_data[which].m_vertex_states[tri.v2].vel;
        const ChVector<>& vC = m_tire_data[which].m_vertex_states[tri.v3].vel;

        ChVector<> vel = (vA + vB + vC) / 3;
        m_tire_data[which].m_proxies[it].m_body->SetPos_dt(vel);

        //// TODO: angular velocity
        m_tire_data[which].m_proxies[it].m_body->SetWvel_loc(ChVector<>(0, 0, 0));

        // Update contact shape (expressed in local frame).
        // Write directly into the Chrono::Parallel data structures, properly offsetting
        // to the entries corresponding to the proxy bodies.
        unsigned int offset = m_proxy_start_index + m_tire_data[which].m_start_tri + it;
        dataA[offset] = R3(pA.x - pos.x, pA.y - pos.y, pA.z - pos.z);
        dataB[offset] = R3(pB.x - pos.x, pB.y - pos.y, pB.z - pos.z);
        dataC[offset] = R3(pC.x - pos.x, pC.y - pos.y, pC.z - pos.z);
    }
}

// Collect contact forces on the (node) proxy bodies that are in contact.
// Load mesh vertex forces and corresponding indices.
void TerrainNode::ForcesNodeProxies(int which, std::vector<double>& vert_forces, std::vector<int>& vert_indices) {
    for (unsigned int iv = 0; iv < m_tire_data[which].m_num_vert; iv++) {
        real3 force = m_system->GetBodyContactForce(m_tire_data[which].m_proxies[iv].m_body);

        if (!IsZero(force)) {
            vert_forces.push_back(force.x);
            vert_forces.push_back(force.y);
            vert_forces.push_back(force.z);
            vert_indices.push_back(m_tire_data[which].m_proxies[iv].m_index);
        }
    }
}

// Calculate barycentric coordinates (a1, a2, a3) for a given point P
// with respect to the triangle with vertices {v1, v2, v3}
ChVector<> TerrainNode::CalcBarycentricCoords(const ChVector<>& v1,
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
void TerrainNode::ForcesFaceProxies(int which, std::vector<double>& vert_forces, std::vector<int>& vert_indices) {
    // Maintain an unordered map of vertex indices and associated contact forces.
    std::unordered_map<int, ChVector<>> my_map;

    for (unsigned int it = 0; it < m_tire_data[which].m_num_tri; it++) {
        Triangle tri = m_tire_data[which].m_triangles[it];

        // Get cumulative contact force at triangle centroid.
        // Do nothing if zero force.
        real3 rforce = m_system->GetBodyContactForce(m_tire_data[which].m_proxies[it].m_body);
        if (IsZero(rforce))
            continue;

        // Centroid has barycentric coordinates {1/3, 1/3, 1/3}, so force is
        // distributed equally to the three vertices.
        ChVector<> force(rforce.x / 3, rforce.y / 3, rforce.z / 3);

        // For each vertex of the triangle, if it appears in the map, increment
        // the total contact force. Otherwise, insert a new entry in the map.
        auto v1 = my_map.find(tri.v1);
        if (v1 != my_map.end()) {
            v1->second += force;
        } else {
            my_map[tri.v1] = force;
        }

        auto v2 = my_map.find(tri.v2);
        if (v2 != my_map.end()) {
            v2->second += force;
        } else {
            my_map[tri.v2] = force;
        }

        auto v3 = my_map.find(tri.v3);
        if (v3 != my_map.end()) {
            v3->second += force;
        } else {
            my_map[tri.v3] = force;
        }
    }

    // Extract map keys (indices of vertices in contact) and map values
    // (corresponding contact forces) and load output vectors.
    // Note: could improve efficiency by reserving space for vectors.
    for (auto kv : my_map) {
        vert_indices.push_back(kv.first);
        vert_forces.push_back(kv.second.x);
        vert_forces.push_back(kv.second.y);
        vert_forces.push_back(kv.second.z);
    }
}

// -----------------------------------------------------------------------------
// Advance simulation of the terrain node by the specified duration
// -----------------------------------------------------------------------------
void TerrainNode::Advance(double step_size) {
    m_timer.reset();
    m_timer.start();
    double t = 0;
    while (t < step_size) {
        double h = std::min<>(m_step_size, step_size - t);
        m_system->DoStepDynamics(h);
        t += h;
    }
    m_timer.stop();
    m_cum_sim_time += m_timer();

#ifdef CHRONO_OPENGL
    if (m_render) {
        opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
        if (gl_window.Active()) {
            gl_window.Render();
        } else {
            MPI_Abort(MPI_COMM_WORLD, 1);
        }
    }
#endif

    for (int which = 0; which < m_num_tires; which++) {
        switch (m_type) {
            case RIGID:
                PrintNodeProxiesContactData(which);
                break;
            case GRANULAR:
                PrintFaceProxiesContactData(which);
                break;
        }
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void TerrainNode::OutputData(int frame) {
    // Append to results output file
    if (m_outf.is_open()) {
        //// TODO
    }

    // Create and write frame output file.
    char filename[100];
    sprintf(filename, "%s/data_%04d.dat", m_node_out_dir.c_str(), frame + 1);

    utils::CSV_writer csv(" ");
    WriteParticleInformation(csv);
    csv.write_to_file(filename);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void TerrainNode::WriteParticleInformation(utils::CSV_writer& csv) {
    // Write current time, number of granular particles and their radius
    csv << m_system->GetChTime() << endl;
    csv << m_num_particles << m_radius_g << endl;

    // Write particle positions and linear velocities
    for (auto body : *m_system->Get_bodylist()) {
        if (body->GetIdentifier() < m_Id_g)
            continue;
        csv << body->GetIdentifier() << body->GetPos() << body->GetPos_dt() << endl;
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void TerrainNode::WriteCheckpoint() {
    utils::CSV_writer csv(" ");

    // Write current time and number of granular material bodies.
    csv << m_system->GetChTime() << endl;
    csv << m_num_particles << endl;

    // Loop over all bodies in the system and write state for granular material bodies.
    // Filter granular material using the body identifier.
    for (auto body : *m_system->Get_bodylist()) {
        if (body->GetIdentifier() < m_Id_g)
            continue;
        csv << body->GetIdentifier() << body->GetPos() << body->GetRot() << body->GetPos_dt() << body->GetRot_dt()
            << endl;
    }

    std::string checkpoint_filename = m_out_dir + "/" + m_checkpoint_filename;
    csv.write_to_file(checkpoint_filename);
    cout << m_prefix << " write checkpoint ===> " << checkpoint_filename << endl;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void TerrainNode::PrintNodeProxiesContactData(int which) {
    //// TODO: implement this
}

void TerrainNode::PrintFaceProxiesContactData(int which) {
    //// TODO: implement this
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void TerrainNode::PrintNodeProxiesUpdateData(int which) {
    //// TODO: implement this
}

void TerrainNode::PrintFaceProxiesUpdateData(int which) {
    //// TODO: implement this
}
