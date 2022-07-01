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
// Implementation of the MPI granular TERRAIN NODE (using Chrono::Distributed).
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// CURRENT LIMITATIONS:
// - rigid obstacles not supported
// - settling phase not implemented
//
//
// =============================================================================

#include <algorithm>
#include <cmath>
#include <set>
#include <limits>
#include <unordered_map>

#include <omp.h>

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono_distributed/collision/ChBoundary.h"
#include "chrono_distributed/collision/ChCollisionModelDistributed.h"

#include "chrono_vehicle/cosim/terrain/ChVehicleCosimTerrainNodeGranularMPI.h"

#ifdef CHRONO_OPENGL
    #include "chrono_opengl/ChOpenGLWindow.h"
#endif

using std::cout;
using std::cerr;
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
// - create the (distributed) Chrono system and set solver parameters
// - create the OpenGL visualization window
//
// ATTENTION: A distributed system requires an MPI subcommunicator.
// This is available only after initialization of the co-simulation framework.
// -----------------------------------------------------------------------------
ChVehicleCosimTerrainNodeGranularMPI::ChVehicleCosimTerrainNodeGranularMPI(double length, double width)
    : ChVehicleCosimTerrainNodeChrono(Type::GRANULAR_MPI, length, width, ChContactMethod::SMC),
      m_radius_p(5e-3),
      m_sampling_type(utils::SamplingType::POISSON_DISK),
      m_init_depth(0.2),
      m_separation_factor(1.001),
      m_in_layers(false),
      m_constructed(false),
      m_proxies_constructed(false),
      m_hthick(0.1),
      m_num_particles(0),
      m_system(nullptr) {
    if (!cosim::IsFrameworkInitialized()) {
        if (m_rank == TERRAIN_NODE_RANK)
            cerr << "Error: Co-simulation framework not initialized." << endl;
        MPI_Abort(MPI_COMM_WORLD, 1);
    }

    // Get the rank within the intracommunicator
    MPI_Comm_rank(cosim::GetTerrainIntracommunicator(), &m_sub_rank);

    // Default granular material properties
    m_radius_g = 0.01;
    m_rho_g = 2000;

    // Create the distributed system
    CreateSystem();

    if (m_rank == TERRAIN_NODE_RANK && !m_system->OnMaster()) {
        cerr << "Error: Inconsistent intracommunicator rank." << endl;
        MPI_Abort(MPI_COMM_WORLD, 1);
    }
}

ChVehicleCosimTerrainNodeGranularMPI::ChVehicleCosimTerrainNodeGranularMPI(const std::string& specfile)
    : ChVehicleCosimTerrainNodeChrono(Type::GRANULAR_MPI, 0, 0, ChContactMethod::SMC),
      m_constructed(false),
      m_proxies_constructed(false),
      m_hthick(0.1),
      m_num_particles(0),
      m_system(nullptr) {
    if (!cosim::IsFrameworkInitialized()) {
        if (m_rank == TERRAIN_NODE_RANK)
            cerr << "Error: rank " << MBS_NODE_RANK << " is not running an MBS node." << endl;
        MPI_Abort(MPI_COMM_WORLD, 1);
    }

    // Get the rank within the intracommunicator
    MPI_Comm_rank(cosim::GetTerrainIntracommunicator(), &m_sub_rank);

    // Create the distributed system
    CreateSystem();

    if (m_rank == TERRAIN_NODE_RANK && !m_system->OnMaster()) {
        cerr << "Error: Inconsistent intracommunicator rank." << endl;
        MPI_Abort(MPI_COMM_WORLD, 1);
    }

    // Read granular OMP terrain parameters from provided specfile
    SetFromSpecfile(specfile);
    m_init_height = m_init_depth;
}

ChVehicleCosimTerrainNodeGranularMPI::~ChVehicleCosimTerrainNodeGranularMPI() {
    delete m_system;
}

void ChVehicleCosimTerrainNodeGranularMPI::CreateSystem() {
    // Create the Chrono::Distributed system.
    m_system = new ChSystemDistributed(cosim::GetTerrainIntracommunicator(), 0, 100000);
    m_system->GetSettings()->solver.contact_force_model = ChSystemSMC::ContactForceModel::Hertz;
    m_system->GetSettings()->solver.tangential_displ_mode = ChSystemSMC::TangentialDisplacementModel::OneStep;
    m_system->GetSettings()->solver.use_material_properties = true;

    m_system->Set_G_acc(ChVector<>(0, 0, m_gacc));
    m_system->GetSettings()->solver.use_full_inertia_tensor = false;
    m_system->GetSettings()->solver.tolerance = 0.1;
    m_system->GetSettings()->solver.max_iteration_bilateral = 100;

    m_system->GetSettings()->collision.narrowphase_algorithm = collision::ChNarrowphase::Algorithm::HYBRID;
    m_system->GetSettings()->collision.collision_envelope = 0;

    m_system->SetNumThreads(1);

#pragma omp parallel
#pragma omp master
    {
        // Sanity check: print number of threads in a parallel region
        cout << "[Terrain node] [" << m_rank << " " << m_sub_rank
             << "]  actual number of OpenMP threads: " << omp_get_num_threads() << endl;
    }
}

// -----------------------------------------------------------------------------

//// TODO: error checking
void ChVehicleCosimTerrainNodeGranularMPI::SetFromSpecfile(const std::string& specfile) {
    Document d;
    ReadSpecfile(specfile, d);

    double length = d["Patch dimensions"]["Length"].GetDouble();
    double width = d["Patch dimensions"]["Width"].GetDouble();
    m_hdimX = length / 2;
    m_hdimY = width / 2;

    m_radius_g = d["Granular material"]["Radius"].GetDouble();
    m_rho_g = d["Granular material"]["Density"].GetDouble();

    double coh_pressure = d["Material properties"]["Cohesion pressure"].GetDouble();
    double coh_force = CH_C_PI * m_radius_g * m_radius_g * coh_pressure;

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

void ChVehicleCosimTerrainNodeGranularMPI::SetNumThreads(int num_threads) {
    m_system->SetNumThreads(num_threads);
}

void ChVehicleCosimTerrainNodeGranularMPI::SetWallThickness(double thickness) {
    m_hthick = thickness / 2;
}

void ChVehicleCosimTerrainNodeGranularMPI::SetGranularMaterial(double radius, double density) {
    m_radius_g = radius;
    m_rho_g = density;
}

void ChVehicleCosimTerrainNodeGranularMPI::UseMaterialProperties(bool flag) {
    m_system->GetSettings()->solver.use_material_properties = flag;
}

void ChVehicleCosimTerrainNodeGranularMPI::SetContactForceModel(ChSystemSMC::ContactForceModel model) {
    m_system->GetSettings()->solver.contact_force_model = model;
}

void ChVehicleCosimTerrainNodeGranularMPI::SetTangentialDisplacementModel(
    ChSystemSMC::TangentialDisplacementModel model) {
    m_system->GetSettings()->solver.tangential_displ_mode = model;
}

void ChVehicleCosimTerrainNodeGranularMPI::SetSamplingMethod(utils::SamplingType type,
                                                             double init_height,
                                                             double sep_factor,
                                                             bool in_layers) {
    m_sampling_type = type;
    m_init_depth = init_height;
    m_separation_factor = sep_factor;
    m_in_layers = in_layers;
}

void ChVehicleCosimTerrainNodeGranularMPI::SetMaterialSurface(const std::shared_ptr<ChMaterialSurfaceSMC>& mat) {
    m_material_terrain = mat;
}

// -----------------------------------------------------------------------------
// Complete construction of the mechanical system.
// Invoked by ChVehicleCosimTerrainNodeChrono::OnInitialize.
// - create distributed system (MPI subcommunicator available here)
// - adjust system settings
// - create the container body
// - create the granular material
// -----------------------------------------------------------------------------
void ChVehicleCosimTerrainNodeGranularMPI::Construct() {
    if (m_constructed)
        return;

    // Resize vector of additional tire data
    m_tire_data.resize(m_num_tire_nodes);

    // Issue messages and render only on the main terrain node
    m_verbose = m_verbose && (m_rank == TERRAIN_NODE_RANK);
    m_render = m_render && (m_rank == TERRAIN_NODE_RANK);

    // Calculate container (half) height
    double r = m_separation_factor * m_radius_g;
    double delta = 2.0f * r;

    // Domain decomposition
    ChVector<> lo(-m_hdimX - delta, -m_hdimY - delta, -2 * m_radius_g);
    ChVector<> hi(+m_hdimX + delta, +m_hdimY + delta, m_init_depth + 3 * m_radius_g);
    m_system->GetDomain()->SetSplitAxis(0);  //// TODO: let user specify this
    m_system->GetDomain()->SetSimDomain(lo, hi);

    m_system->SetGhostLayer(2 * m_radius_g);

    // Estimates for number of bins for broad-phase.
    int factor = 2;
    auto& sub_lo = m_system->GetDomain()->GetSubLo();
    auto& sub_hi = m_system->GetDomain()->GetSubHi();
    auto sub_hdim = (sub_hi - sub_lo) / 2;
    int binsX = (int)std::ceil(sub_hdim.x() / m_radius_g) / factor;
    int binsY = (int)std::ceil(sub_hdim.y() / m_radius_g) / factor;
    int binsZ = 1;
    m_system->GetSettings()->collision.bins_per_axis = vec3(binsX, binsY, binsZ);
    if (m_verbose)
        cout << "[Terrain node] broad-phase bins: " << binsX << " x " << binsY << " x " << binsZ << endl;

    // ---------------------
    // Create container body
    // ----------------------

    auto container = std::shared_ptr<ChBody>(m_system->NewBody());
    container->SetIdentifier(-1);
    container->SetPos(ChVector<>(0, 0, 0));
    container->SetMass(1);
    container->SetBodyFixed(true);
    container->SetCollide(true);
    m_system->AddBodyAllRanks(container);

    double dimX = 2 * m_hdimX;
    double dimY = 2 * m_hdimY;
    double dimZ = m_init_depth;
    double hdimZ = dimZ / 2;
    auto cb = new ChBoundary(container, std::static_pointer_cast<ChMaterialSurfaceSMC>(m_material_terrain));
    cb->AddPlane(ChFrame<>(ChVector<>(0, 0, 0), QUNIT), ChVector2<>(dimX, dimY));
    cb->AddPlane(ChFrame<>(ChVector<>(-m_hdimX, 0, hdimZ), Q_from_AngY(+CH_C_PI_2)), ChVector2<>(dimZ, dimY));
    cb->AddPlane(ChFrame<>(ChVector<>(+m_hdimX, 0, hdimZ), Q_from_AngY(-CH_C_PI_2)), ChVector2<>(dimZ, dimY));
    cb->AddPlane(ChFrame<>(ChVector<>(0, -m_hdimY, hdimZ), Q_from_AngX(-CH_C_PI_2)), ChVector2<>(dimX, dimZ));
    cb->AddPlane(ChFrame<>(ChVector<>(0, +m_hdimY, hdimZ), Q_from_AngX(+CH_C_PI_2)), ChVector2<>(dimX, dimZ));

    // Enable deactivation of bodies that exit a specified bounding box.
    // We set this bounding box to encapsulate the container with a conservative height.
    m_system->GetSettings()->collision.use_aabb_active = true;
    m_system->GetSettings()->collision.aabb_min = real3(-m_hdimX - m_hthick, -m_hdimY - m_hthick, -m_hthick);
    m_system->GetSettings()->collision.aabb_max = real3(+m_hdimX + m_hthick, +m_hdimY + m_hthick, 2 * hdimZ + 2);

    // --------------------------
    // Generate granular material
    // --------------------------

    // Mass and inertia moments for a granular particle
    double mass_g = m_rho_g * 4 / 3 * CH_C_PI * m_radius_g * m_radius_g * m_radius_g;
    ChVector<> inertia_g = (2.0 / 5.0) * mass_g * m_radius_g * m_radius_g * ChVector<>(1, 1, 1);

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

    // Create particle locations
    std::vector<ChVector<>> points;
    if (m_in_layers) {
        ChVector<> hdims(m_hdimX - r, m_hdimY - r, 0);
        double z = delta;
        while (z < m_init_depth) {
            auto points_new = sampler->SampleBox(ChVector<>(0, 0, z), hdims);
            points.insert(points.end(), points_new.begin(), points_new.end());
            if (m_verbose)
                cout << "   z =  " << z << "\tnum particles = " << points.size() << endl;
            z += delta;
        }
    } else {
        ChVector<> hdims(m_hdimX - r, m_hdimY - r, hdimZ - r);
        points = sampler->SampleBox(ChVector<>(0, 0, hdimZ), hdims);
    }

    // Create particle bodies with identifiers starting at m_Id_g
    m_num_particles = (unsigned int)points.size();
    if (m_verbose)
        cout << "[Terrain node] Generated num particles = " << m_num_particles << endl;

    m_init_height = -std::numeric_limits<double>::max();
    int particle_id = body_id_particles;
    for (unsigned int i = 0; i < m_num_particles; i++) {
        auto body =
            chrono_types::make_shared<ChBody>(chrono_types::make_shared<collision::ChCollisionModelDistributed>());
        body->SetIdentifier(particle_id++);
        body->SetMass(mass_g);
        body->SetInertiaXX(inertia_g);
        body->SetPos(points[i]);
        body->SetRot(ChQuaternion<>(1, 0, 0, 0));
        body->SetBodyFixed(false);
        body->SetCollide(true);

        body->GetCollisionModel()->ClearModel();
        utils::AddSphereGeometry(body.get(), m_material_terrain, m_radius_g);
        body->GetCollisionModel()->BuildModel();

        m_system->AddBody(body);

        if (points[i].z() > m_init_height)
            m_init_height = points[i].z();
    }

    if (m_verbose)
        cout << "[Terrain node] initial height = " << m_init_height << endl;

#ifdef CHRONO_OPENGL
    // Render only on the main terrain node
    m_render = m_render && (m_rank == TERRAIN_NODE_RANK);

    // Create the visualization window
    if (m_render) {
        opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
        gl_window.AttachSystem(m_system);
        gl_window.Initialize(1280, 720, "Terrain Node (GranularOMP)");
        gl_window.SetCamera(ChVector<>(0, -3, 0), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), 0.05f);
        gl_window.SetRenderMode(opengl::WIREFRAME);
    }
#endif

    // Write file with terrain node settings
    if (m_rank == TERRAIN_NODE_RANK) {
        std::ofstream outf;
        outf.open(m_node_out_dir + "/settings.info", std::ios::out);
        outf << "System settings" << endl;
        outf << "   Integration step size = " << m_step_size << endl;
        outf << "   Use material properties? "
             << (m_system->GetSettings()->solver.use_material_properties ? "YES" : "NO") << endl;
        outf << "Terrain patch dimensions" << endl;
        outf << "   X = " << 2 * m_hdimX << "  Y = " << 2 * m_hdimY << endl;
        outf << "Terrain material properties" << endl;

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

        outf << "Granular material properties" << endl;
        outf << "   particle radius  = " << m_radius_g << endl;
        outf << "   particle density = " << m_rho_g << endl;
        outf << "   number particles = " << m_num_particles << endl;
        outf << "Proxy body properties" << endl;
        outf << "   proxies fixed? " << (m_fixed_proxies ? "YES" : "NO") << endl;
        outf << "   proxy contact radius = " << m_radius_p << endl;
    }

    // Mark system as constructed.
    m_constructed = true;
}

// -----------------------------------------------------------------------------

double ChVehicleCosimTerrainNodeGranularMPI::CalcTotalKineticEnergy() {
    double KE = 0;

    int i = 0;
    auto bl_itr = m_system->data_manager->body_list->begin();
    for (; bl_itr != m_system->data_manager->body_list->end(); bl_itr++, i++) {
        if (m_system->ddm->comm_status[i] != chrono::distributed::EMPTY && (*bl_itr)->GetIdentifier() > 0) {
            const auto& vel = (*bl_itr)->GetPos_dt();
            const auto& omg = (*bl_itr)->GetWvel_par();
            const auto& m = (*bl_itr)->GetMass();
            const auto& J = (*bl_itr)->GetInertiaXX();
            KE += m * vel.Length2() + omg.Dot(J * omg);
        }
    }

    return 0.5 * KE;
}

double ChVehicleCosimTerrainNodeGranularMPI::CalcCurrentHeight() {
    double height = -std::numeric_limits<double>::max();

    int i = 0;
    auto bl_itr = m_system->data_manager->body_list->begin();
    for (; bl_itr != m_system->data_manager->body_list->end(); bl_itr++, i++) {
        if (m_system->ddm->comm_status[i] != chrono::distributed::EMPTY) {
            const auto& pos = (*bl_itr)->GetPos();
            if ((*bl_itr)->GetIdentifier() > 0 && pos.z() > height)
                height = pos.z();
        }
    }

    return height;
}

//// TODO: Consider looking only at particles below a certain fraction of the
//// height of the highest particle.  This would eliminate errors in estimating
//// the total volume stemming from a "non-smooth" top surface or stray particles.
double ChVehicleCosimTerrainNodeGranularMPI::CalculatePackingDensity(double& depth) {
    // Find height of granular material
    double z_max = CalcCurrentHeight();
    double z_min = 0;
    depth = z_max - z_min;

    // Find total volume of granular material
    double Vt = (2 * m_hdimX) * (2 * m_hdimY) * (z_max - z_min);

    // Find volume of granular particles
    double Vs = m_num_particles * (4.0 / 3) * CH_C_PI * std::pow(m_radius_g, 3);

    // Packing density = Vs/Vt
    return Vs / Vt;
}

// -----------------------------------------------------------------------------

// Tire mesh information is available in m_mesh_data only on the main terrain node.
// Broadcast information to intra-communicator.
void ChVehicleCosimTerrainNodeGranularMPI::ScatterInitData(unsigned int i) {
    auto root = m_system->GetMasterRank();
    auto comm = m_system->GetCommunicator();

    double tire_info[2];
    if (m_rank == TERRAIN_NODE_RANK) {
        tire_info[0] = m_tire_radius[i];
        tire_info[1] = m_tire_width[i];
    }
    MPI_Bcast(tire_info, 2, MPI_DOUBLE, root, comm);
    if (m_rank != TERRAIN_NODE_RANK) {
        m_tire_radius[i] = tire_info[0];
        m_tire_width[i] = tire_info[1];
    }

    unsigned int surf_props[3];
    if (m_rank == TERRAIN_NODE_RANK) {
        surf_props[0] = m_mesh_data[i].nv;
        surf_props[1] = m_mesh_data[i].nn;
        surf_props[2] = m_mesh_data[i].nt;
    }
    MPI_Bcast(surf_props, 3, MPI_UNSIGNED, root, comm);
    if (m_rank != TERRAIN_NODE_RANK) {
        m_mesh_data[i].nv = surf_props[0];
        m_mesh_data[i].nn = surf_props[1];
        m_mesh_data[i].nt = surf_props[2];
        m_mesh_data[i].verts.resize(m_mesh_data[i].nv);
        m_mesh_data[i].norms.resize(m_mesh_data[i].nn);
        m_mesh_data[i].idx_verts.resize(m_mesh_data[i].nt);
        m_mesh_data[i].idx_norms.resize(m_mesh_data[i].nt);
        m_mesh_state[i].vpos.resize(m_mesh_data[i].nv);
        m_mesh_state[i].vvel.resize(m_mesh_data[i].nv);
    }

    double* vert_data = new double[3 * m_mesh_data[i].nv + 3 * m_mesh_data[i].nn];
    int* tri_data = new int[3 * m_mesh_data[i].nt + 3 * m_mesh_data[i].nt];
    if (m_rank == TERRAIN_NODE_RANK) {
        for (unsigned int iv = 0; iv < m_mesh_data[i].nv; iv++) {
            vert_data[3 * iv + 0] = m_mesh_data[i].verts[iv].x();
            vert_data[3 * iv + 1] = m_mesh_data[i].verts[iv].y();
            vert_data[3 * iv + 2] = m_mesh_data[i].verts[iv].z();
        }
        for (unsigned int in = 0; in < m_mesh_data[i].nn; in++) {
            vert_data[3 * m_mesh_data[i].nv + 3 * in + 0] = m_mesh_data[i].norms[in].x();
            vert_data[3 * m_mesh_data[i].nv + 3 * in + 1] = m_mesh_data[i].norms[in].y();
            vert_data[3 * m_mesh_data[i].nv + 3 * in + 2] = m_mesh_data[i].norms[in].z();
        }
        for (unsigned int it = 0; it < m_mesh_data[i].nt; it++) {
            tri_data[6 * it + 0] = m_mesh_data[i].idx_verts[it].x();
            tri_data[6 * it + 1] = m_mesh_data[i].idx_verts[it].y();
            tri_data[6 * it + 2] = m_mesh_data[i].idx_verts[it].z();
            tri_data[6 * it + 3] = m_mesh_data[i].idx_norms[it].x();
            tri_data[6 * it + 4] = m_mesh_data[i].idx_norms[it].y();
            tri_data[6 * it + 5] = m_mesh_data[i].idx_norms[it].z();
        }
    }
    MPI_Bcast(vert_data, 3 * m_mesh_data[i].nv + 3 * m_mesh_data[i].nn, MPI_DOUBLE, root, comm);
    MPI_Bcast(tri_data, 3 * m_mesh_data[i].nt + 3 * m_mesh_data[i].nt, MPI_INT, root, comm);
    if (m_rank != TERRAIN_NODE_RANK) {
        for (unsigned int iv = 0; iv < m_mesh_data[i].nv; iv++) {
            m_mesh_data[i].verts[iv].x() = vert_data[3 * iv + 0];
            m_mesh_data[i].verts[iv].y() = vert_data[3 * iv + 1];
            m_mesh_data[i].verts[iv].z() = vert_data[3 * iv + 2];
        }
        for (unsigned int in = 0; in < m_mesh_data[i].nn; in++) {
            m_mesh_data[i].norms[in].x() = vert_data[3 * m_mesh_data[i].nv + 3 * in + 0];
            m_mesh_data[i].norms[in].y() = vert_data[3 * m_mesh_data[i].nv + 3 * in + 1];
            m_mesh_data[i].norms[in].z() = vert_data[3 * m_mesh_data[i].nv + 3 * in + 2];
        }
        for (unsigned int it = 0; it < m_mesh_data[i].nt; it++) {
            m_mesh_data[i].idx_verts[it].x() = tri_data[6 * it + 0];
            m_mesh_data[i].idx_verts[it].y() = tri_data[6 * it + 1];
            m_mesh_data[i].idx_verts[it].z() = tri_data[6 * it + 2];
            m_mesh_data[i].idx_norms[it].x() = tri_data[6 * it + 3];
            m_mesh_data[i].idx_norms[it].y() = tri_data[6 * it + 4];
            m_mesh_data[i].idx_norms[it].z() = tri_data[6 * it + 5];
        }
    }
    delete[] vert_data;
    delete[] tri_data;

    MPI_Bcast(&m_load_mass[i], 1, MPI_DOUBLE, root, comm);

    float mat_props[8];
    if (m_rank == TERRAIN_NODE_RANK) {
        mat_props[0] = m_mat_props[i].mu;
        mat_props[1] = m_mat_props[i].cr;
        mat_props[2] = m_mat_props[i].Y;
        mat_props[3] = m_mat_props[i].nu;
        mat_props[4] = m_mat_props[i].kn;
        mat_props[5] = m_mat_props[i].gn;
        mat_props[6] = m_mat_props[i].kt;
        mat_props[7] = m_mat_props[i].gt;
    }
    MPI_Bcast(mat_props, 8, MPI_FLOAT, root, comm);
    if (m_rank != TERRAIN_NODE_RANK) {
        m_mat_props[i].mu = mat_props[0];
        m_mat_props[i].cr = mat_props[1];
        m_mat_props[i].Y = mat_props[2];
        m_mat_props[i].nu = mat_props[3];
        m_mat_props[i].kn = mat_props[4];
        m_mat_props[i].gn = mat_props[5];
        m_mat_props[i].kt = mat_props[6];
        m_mat_props[i].gt = mat_props[7];
    }
}

// Body position must be known when adding the body to a Chrono::Distributed system, so that the body is assigned to the
// appropriate rank.  However, in the co-simulation framework, the wheel/spindle position is not known until the first
// synchronization.  We defer proxy body creation until the first synchronization time.

void ChVehicleCosimTerrainNodeGranularMPI::CreateMeshProxies(unsigned int i) {
    ScatterInitData(i);

    m_tire_data[i].m_gids.resize(m_mesh_data[i].nt);
    m_tire_data[i].m_start_tri = (i == 0) ? 0 : m_tire_data[i - 1].m_start_tri + m_mesh_data[i].nt;
}

void ChVehicleCosimTerrainNodeGranularMPI::CreateWheelProxy(unsigned int i) {
    ScatterInitData(i);

    m_tire_data[i].m_gids.resize(m_mesh_data[i].nt);
    m_tire_data[i].m_start_tri = (i == 0) ? 0 : m_tire_data[i - 1].m_start_tri + m_mesh_data[i].nt;
}

// Since Chrono::Distributed cannot treat a trimesh as a single collision object,
// we always create a number of proxy bodies with triangle collision shape equal
// to the number of faces in the tire mesh. These proxies are used for both rigid
// and flexible tires.
// Assign to each body an identifier equal to the index of its corresponding mesh face.
// Add all proxy bodies to the same collision family and disable collision between any
// two members of this family.

void ChVehicleCosimTerrainNodeGranularMPI::CreateMeshProxiesInternal(unsigned int i) {
    //// RADU TODO:  better approximation of mass / inertia?
    double mass_p = m_load_mass[i] / m_mesh_data[i].nt;
    ChVector<> inertia_p = 1e-3 * mass_p * ChVector<>(0.1, 0.1, 0.1);
    auto material_tire = m_mat_props[i].CreateMaterial(m_method);

    for (unsigned int it = 0; it < m_mesh_data[i].nt; it++) {
        unsigned int body_id = m_tire_data[i].m_start_tri + it;

        auto body = std::shared_ptr<ChBody>(m_system->NewBody());
        body->SetIdentifier(body_id);
        body->SetMass(mass_p);
        body->SetInertiaXX(inertia_p);
        body->SetBodyFixed(m_fixed_proxies);

        // Determine initial position.
        // Use current information in m_mesh_state which encodes the current wheel position.
        const auto& tri = m_mesh_data[i].idx_verts[it];
        const auto& pA = m_mesh_state[i].vpos[tri[0]];
        const auto& pB = m_mesh_state[i].vpos[tri[1]];
        const auto& pC = m_mesh_state[i].vpos[tri[2]];
        ChVector<> pos = (pA + pB + pC) / 3;
        body->SetPos(pos);

        // Create contact shape.
        std::string name = "tri_" + std::to_string(body_id);
        body->GetCollisionModel()->ClearModel();
        utils::AddTriangleGeometry(body.get(), material_tire, pA - pos, pB - pos, pC - pos, name);
        body->GetCollisionModel()->SetFamily(1);
        body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
        body->GetCollisionModel()->BuildModel();

        body->SetCollide(true);
        m_system->AddBody(body);

        // Update map global ID -> triangle index
        m_tire_data[i].m_gids[it] = body->GetGid();
        m_tire_data[i].m_map[body->GetGid()] = it;
    }
}

void ChVehicleCosimTerrainNodeGranularMPI::CreateWheelProxyInternal(unsigned int i) {
    //// RADU TODO:  better approximation of mass / inertia?
    double mass_p = m_load_mass[i] / m_mesh_data[i].nt;
    ChVector<> inertia_p = 1e-3 * mass_p * ChVector<>(0.1, 0.1, 0.1);
    auto material_tire = m_mat_props[i].CreateMaterial(m_method);

    for (unsigned int it = 0; it < m_mesh_data[i].nt; it++) {
        unsigned int body_id = m_tire_data[i].m_start_tri + it;

        auto body = std::shared_ptr<ChBody>(m_system->NewBody());
        body->SetIdentifier(body_id);
        body->SetMass(mass_p);
        body->SetInertiaXX(inertia_p);
        body->SetBodyFixed(m_fixed_proxies);

        // Determine initial position.
        // Use current information in m_spindle_state.
        const auto& tri = m_mesh_data[i].idx_verts[it];
        const auto& pA = m_mesh_data[i].verts[tri[0]];
        const auto& pB = m_mesh_data[i].verts[tri[1]];
        const auto& pC = m_mesh_data[i].verts[tri[2]];
        ChVector<> pos = (pA + pB + pC) / 3;

        body->SetPos(m_spindle_state[i].pos + pos);

        // Create contact shape.
        std::string name = "tri_" + std::to_string(body_id);
        body->GetCollisionModel()->ClearModel();
        utils::AddTriangleGeometry(body.get(), material_tire, pA - pos, pB - pos, pC - pos, name);
        body->GetCollisionModel()->SetFamily(1);
        body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
        body->GetCollisionModel()->BuildModel();

        body->SetCollide(true);
        m_system->AddBody(body);

        // Update map global ID -> triangle index
        m_tire_data[i].m_gids[it] = body->GetGid();
        m_tire_data[i].m_map[body->GetGid()] = it;
    }
}

// -----------------------------------------------------------------------------

// Set position, orientation, and velocity of proxy bodies based on tire mesh faces.
// The proxy body is effectively reconstructed at each synchronization time:
//    - position at the center of mass of the three vertices
//    - orientation: identity
//    - linear and angular velocity: consistent with vertex velocities
//    - contact shape: redefined to match vertex locations
void ChVehicleCosimTerrainNodeGranularMPI::UpdateMeshProxies(unsigned int i, MeshState& mesh_state) {
    auto& mesh_data = m_mesh_data[i];  // mesh data for the i-th tire
    auto& tire_data = m_tire_data[i];  // additional data for the i-th tire

    // 1. Scatter current tire mesh state from the main terrain node to intra-communicator.

    double* vert_data = new double[2 * 3 * mesh_data.nv];
    if (m_rank == TERRAIN_NODE_RANK) {
        for (unsigned int iv = 0; iv < mesh_data.nv; iv++) {
            unsigned int offset = 3 * iv;
            vert_data[offset + 0] = mesh_state.vpos[iv].x();
            vert_data[offset + 1] = mesh_state.vpos[iv].y();
            vert_data[offset + 2] = mesh_state.vpos[iv].z();
            offset += 3 * mesh_data.nv;
            vert_data[offset + 0] = mesh_state.vvel[iv].x();
            vert_data[offset + 1] = mesh_state.vvel[iv].y();
            vert_data[offset + 2] = mesh_state.vvel[iv].z();
        }
    }
    MPI_Bcast(vert_data, 2 * 3 * mesh_data.nv, MPI_DOUBLE, m_system->GetMasterRank(), m_system->GetCommunicator());
    if (m_rank != TERRAIN_NODE_RANK) {
        for (unsigned int iv = 0; iv < mesh_data.nv; iv++) {
            unsigned int offset = 3 * iv;
            m_mesh_state[i].vpos[iv] = ChVector<>(vert_data[offset + 0], vert_data[offset + 1], vert_data[offset + 2]);
            offset += 3 * mesh_data.nv;
            m_mesh_state[i].vvel[iv] = ChVector<>(vert_data[offset + 0], vert_data[offset + 1], vert_data[offset + 2]);
        }
    }
    delete[] vert_data;

    // 2. Create proxies

    if (!m_proxies_constructed) {
        CreateMeshProxiesInternal(i);
        m_proxies_constructed = true;
    }

    // 3. Use information in mesh_state to update proxy states.

    std::vector<ChSystemDistributed::BodyState> states(mesh_data.nt);
    std::vector<ChSystemDistributed::TriData> shapes(mesh_data.nt);
    std::vector<int> shape_idx(mesh_data.nt, 0);

    for (unsigned int it = 0; it < mesh_data.nt; it++) {
        // Associated mesh triangle
        const auto& tri = mesh_data.idx_verts[it];

        // Vertex locations and velocities (expressed in global frame)
        const auto& pA = mesh_state.vpos[tri.x()];
        const auto& pB = mesh_state.vpos[tri.y()];
        const auto& pC = mesh_state.vpos[tri.z()];

        const auto& vA = mesh_state.vvel[tri.x()];
        const auto& vB = mesh_state.vvel[tri.y()];
        const auto& vC = mesh_state.vvel[tri.z()];

        // Position and orientation of proxy body (at triangle barycenter)
        ChVector<> pos = (pA + pB + pC) / 3;
        states[it].pos = pos;
        states[it].rot = QUNIT;

        // Linear velocity (absolute) and angular velocity (local)
        // These are the solution of an over-determined 9x6 linear system. However, for a centroidal
        // body reference frame, the linear velocity is the average of the 3 vertex velocities.
        // This leaves a 9x3 linear system for the angular velocity which should be solved in a
        // least-square sense:   Ax = b   =>  (A'A)x = A'b
        states[it].pos_dt = (vA + vB + vC) / 3;
        states[it].rot_dt = QNULL;  //// TODO: angular velocity

        // Triangle contact shape (expressed in local frame).
        shapes[it].v1 = pA - pos;
        shapes[it].v2 = pB - pos;
        shapes[it].v3 = pC - pos;
    }

    // Update body states
    m_system->SetBodyStates(tire_data.m_gids, states);

    // Update collision shapes (one triangle per collision model)
    m_system->SetTriangleShapes(tire_data.m_gids, shape_idx, shapes);
}

// Set state of wheel proxy body.
void ChVehicleCosimTerrainNodeGranularMPI::UpdateWheelProxy(unsigned int i, BodyState& spindle_state) {
    auto& mesh_data = m_mesh_data[i];  // mesh data for the i-th tire
    auto& tire_data = m_tire_data[i];  // additional data for the i-th tire

    // 1. Scatter current spindle body state from the main terrain node to intra-communicator.

    double state_data[13];
    if (m_rank == TERRAIN_NODE_RANK) {
        state_data[0] = spindle_state.pos.x();
        state_data[1] = spindle_state.pos.y();
        state_data[2] = spindle_state.pos.z();
        state_data[3] = spindle_state.rot.e0();
        state_data[4] = spindle_state.rot.e1();
        state_data[5] = spindle_state.rot.e2();
        state_data[6] = spindle_state.rot.e3();
        state_data[7] = spindle_state.lin_vel.x();
        state_data[8] = spindle_state.lin_vel.y();
        state_data[9] = spindle_state.lin_vel.z();
        state_data[10] = spindle_state.ang_vel.x();
        state_data[11] = spindle_state.ang_vel.y();
        state_data[12] = spindle_state.ang_vel.z();
    }
    MPI_Bcast(state_data, 13, MPI_DOUBLE, m_system->GetMasterRank(), m_system->GetCommunicator());
    if (m_rank != TERRAIN_NODE_RANK) {
        spindle_state.pos = ChVector<>(state_data[0], state_data[1], state_data[2]);
        spindle_state.rot = ChQuaternion<>(state_data[3], state_data[4], state_data[5], state_data[6]);
        spindle_state.lin_vel = ChVector<>(state_data[7], state_data[8], state_data[9]);
        spindle_state.ang_vel = ChVector<>(state_data[10], state_data[11], state_data[12]);
    }

    // 2. Create proxies

    if (!m_proxies_constructed) {
        CreateWheelProxyInternal(i);
        m_proxies_constructed = true;
    }

    // 2. Use information in spindle_state to update proxy states (apply rigid body motion).

    std::vector<ChSystemDistributed::BodyState> states(mesh_data.nt);
    ChMatrix33<> spindle_rot(spindle_state.rot);

    for (unsigned int it = 0; it < mesh_data.nt; it++) {
        // Associated mesh triangle
        const auto& tri = mesh_data.idx_verts[it];

        const auto& pA = mesh_data.verts[tri[0]];
        const auto& pB = mesh_data.verts[tri[1]];
        const auto& pC = mesh_data.verts[tri[2]];
        ChVector<> pos = (pA + pB + pC) / 3;

        states[it].pos = spindle_state.pos + spindle_rot * pos;
        states[it].rot = spindle_state.rot;
        states[it].pos_dt = spindle_state.lin_vel + Vcross(spindle_state.ang_vel, pos);
        states[it].rot_dt = QNULL;  //// TODO: angular velocity
    }

    // Update body states
    m_system->SetBodyStates(tire_data.m_gids, states);
}

// Calculate barycentric coordinates (a1, a2, a3) for a given point P
// with respect to the triangle with vertices {v1, v2, v3}
ChVector<> ChVehicleCosimTerrainNodeGranularMPI::CalcBarycentricCoords(const ChVector<>& v1,
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

// -----------------------------------------------------------------------------

// Collect contact forces on the (face) proxy bodies that are in contact.
// Load mesh vertex forces and corresponding indices.
// Note: Only the main terrain node needs to load output.
void ChVehicleCosimTerrainNodeGranularMPI::GetForcesMeshProxies(unsigned int i, MeshContact& mesh_contact) {
    auto& mesh_data = m_mesh_data[i];  // mesh data for the i-th tire
    auto& tire_data = m_tire_data[i];  // additional data for the i-th tire

    // Gather contact forces on proxy bodies on the terrain master rank.
    auto force_pairs = m_system->GetBodyContactForces(tire_data.m_gids);

    if (m_rank != TERRAIN_NODE_RANK)
        return;

    // Maintain an unordered map of vertex indices and associated contact forces.
    std::unordered_map<int, ChVector<>> my_map;

    // Loop over all triangles that experienced contact and accumulate forces on adjacent vertices.
    for (const auto& force_pair : force_pairs) {
        auto gid = force_pair.first;                // global ID of the proxy body
        auto it = tire_data.m_map[gid];             // index of corresponding triangle
        const auto& tri = mesh_data.idx_verts[it];  // triangle vertex indices

        // Centroid has barycentric coordinates {1/3, 1/3, 1/3}, so force is distributed equally to the three vertices.
        ChVector<> force = force_pair.second / 3;

        // For each vertex of the triangle, if it appears in the map, increment the total contact force.
        // Otherwise, insert a new entry in the map.
        auto v1 = my_map.find(tri[0]);
        if (v1 != my_map.end()) {
            v1->second += force;
        } else {
            my_map[tri[0]] = force;
        }

        auto v2 = my_map.find(tri[1]);
        if (v2 != my_map.end()) {
            v2->second += force;
        } else {
            my_map[tri[1]] = force;
        }

        auto v3 = my_map.find(tri[2]);
        if (v3 != my_map.end()) {
            v3->second += force;
        } else {
            my_map[tri[2]] = force;
        }
    }

    // Extract map keys (indices of vertices in contact) and map values (corresponding contact forces) and load output
    // vectors. Note: could improve efficiency by reserving space for vectors.
    mesh_contact.nv = 0;
    for (const auto& kv : my_map) {
        mesh_contact.vidx.push_back(kv.first);
        mesh_contact.vforce.push_back(kv.second);
        mesh_contact.nv++;
    }
}

// Collect resultant contact force and torque on wheel proxy body.
// Note: Only the main terrain node needs to load output.
void ChVehicleCosimTerrainNodeGranularMPI::GetForceWheelProxy(unsigned int i, TerrainForce& wheel_contact) {
    auto& mesh_data = m_mesh_data[i];  // mesh data for the i-th tire
    auto& tire_data = m_tire_data[i];  // additional data for the i-th tire

    // Gather contact forces on proxy bodies on the terrain master rank.
    auto force_pairs = m_system->GetBodyContactForces(tire_data.m_gids);

    if (m_rank != TERRAIN_NODE_RANK)
        return;

    // Current spindle body position
    const auto& spindle_pos = m_spindle_state[i].pos;

    // Loop over all triangles that experienced contact and accumulate forces on spindle.
    wheel_contact.point = ChVector<>(0, 0, 0);
    wheel_contact.force = ChVector<>(0, 0, 0);
    wheel_contact.moment = ChVector<>(0, 0, 0);

    for (const auto& force_pair : force_pairs) {
        auto gid = force_pair.first;                // global ID of the proxy body
        auto it = tire_data.m_map[gid];             // index of corresponding triangle
        const auto& tri = mesh_data.idx_verts[it];  // triangle vertex indices
        const auto& force = force_pair.second;      // force on proxy body

        const auto& pA = mesh_data.verts[tri[0]];
        const auto& pB = mesh_data.verts[tri[1]];
        const auto& pC = mesh_data.verts[tri[2]];
        ChVector<> pos = (pA + pB + pC) / 3;  // local position of triangle on spindle body

        wheel_contact.force += force;
        wheel_contact.moment += Vcross(Vsub(pos, spindle_pos), force);
    }
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeGranularMPI::OnAdvance(double step_size) {
    ChVehicleCosimTerrainNodeChrono::OnAdvance(step_size);

    // Force a calculation of cumulative contact forces for all bodies in the system
    // (needed at the next synchronization)
    m_system->CalculateContactForces();
}

void ChVehicleCosimTerrainNodeGranularMPI::Render(double time) {
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

void ChVehicleCosimTerrainNodeGranularMPI::OnOutputData(int frame) {
    // Create and write frame output file.
    std::string filename = OutputFilename(m_node_out_dir + "/simulation", "simulation", "dat", frame + 1, 5);

    //// TODO
}

void ChVehicleCosimTerrainNodeGranularMPI::WriteCheckpoint(const std::string& filename) const {
    if (m_verbose)
        cout << "[Terrain node] write checkpoint ===> CURRENTLY NOT IMPLEMENTED!" << endl;

    //// TODO
}

}  // end namespace vehicle
}  // end namespace chrono
