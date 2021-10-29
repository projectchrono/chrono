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
// ATTENTION: A distributed system requires an MPI subcommunicator.  This is
//  available only after initialization of the base node. As such, construction
//  of the distributed system must be deferred to Construct() and settling is
//  currently not possible.
// -----------------------------------------------------------------------------
ChVehicleCosimTerrainNodeGranularMPI::ChVehicleCosimTerrainNodeGranularMPI(double length, double width)
    : ChVehicleCosimTerrainNodeChrono(Type::GRANULAR_MPI, length, width, ChContactMethod::SMC),
      m_radius_p(5e-3),
      m_sampling_type(utils::SamplingType::POISSON_DISK),
      m_init_depth(0.2),
      m_separation_factor(1.001),
      m_in_layers(false),
      m_constructed(false),
      m_hthick(0.1),
      m_num_particles(0),
      m_system(nullptr),
      m_num_threads(1),
      m_force_model(ChSystemSMC::ContactForceModel::Hertz),
      m_displ_mode(ChSystemSMC::TangentialDisplacementModel::OneStep),
      m_use_mat_props(true) {
    // Default granular material properties
    m_radius_g = 0.01;
    m_rho_g = 2000;
}

ChVehicleCosimTerrainNodeGranularMPI::ChVehicleCosimTerrainNodeGranularMPI(const std::string& specfile)
    : ChVehicleCosimTerrainNodeChrono(Type::GRANULAR_MPI, 0, 0, ChContactMethod::SMC),
      m_constructed(false),
      m_hthick(0.1),
      m_num_particles(0),
      m_system(nullptr),
      m_num_threads(1),
      m_force_model(ChSystemSMC::ContactForceModel::Hertz),
      m_displ_mode(ChSystemSMC::TangentialDisplacementModel::OneStep),
      m_use_mat_props(true) {
    // Read granular OMP terrain parameters from provided specfile
    SetFromSpecfile(specfile);
}

ChVehicleCosimTerrainNodeGranularMPI::~ChVehicleCosimTerrainNodeGranularMPI() {
    delete m_system;
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
        m_force_model = ChSystemSMC::ContactForceModel::Hertz;
    else if (n_model.compare("Hooke") == 0)
        m_force_model = ChSystemSMC::ContactForceModel::Hooke;
    else if (n_model.compare("Flores") == 0)
        m_force_model = ChSystemSMC::ContactForceModel::Flores;
    else if (n_model.compare("Hertz") == 0)
        m_force_model = ChSystemSMC::ContactForceModel::PlainCoulomb;

    std::string t_model = d["Simulation settings"]["Tangential displacement model"].GetString();
    if (t_model.compare("MultiStep") == 0)
        m_displ_mode = ChSystemSMC::TangentialDisplacementModel::MultiStep;
    else if (t_model.compare("OneStep") == 0)
        m_displ_mode = ChSystemSMC::TangentialDisplacementModel::OneStep;
    else if (t_model.compare("None") == 0)
        m_displ_mode = ChSystemSMC::TangentialDisplacementModel::None;

    m_use_mat_props = d["Simulation settings"]["Use material properties"].GetBool();

    m_radius_p = d["Simulation settings"]["Proxy contact radius"].GetDouble();
    m_fixed_proxies = d["Simulation settings"]["Fix proxies"].GetBool();
}

void ChVehicleCosimTerrainNodeGranularMPI::SetNumThreads(int num_threads) {
    m_num_threads = num_threads;
}

void ChVehicleCosimTerrainNodeGranularMPI::SetWallThickness(double thickness) {
    m_hthick = thickness / 2;
}

void ChVehicleCosimTerrainNodeGranularMPI::SetGranularMaterial(double radius, double density) {
    m_radius_g = radius;
    m_rho_g = density;
}

void ChVehicleCosimTerrainNodeGranularMPI::UseMaterialProperties(bool flag) {
    m_use_mat_props = flag;
}

void ChVehicleCosimTerrainNodeGranularMPI::SetContactForceModel(ChSystemSMC::ContactForceModel model) {
    m_force_model = model;
}

void ChVehicleCosimTerrainNodeGranularMPI::SetTangentialDisplacementModel(
    ChSystemSMC::TangentialDisplacementModel model) {
    m_displ_mode = model;
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

    std::cout << "SUB RANK = " << m_sub_rank << std::endl;


    // Create the Chrono::Distributed system.
    m_system = new ChSystemDistributed(TerrainCommunicator(), m_radius_g * 2, 100000);
    m_system->GetSettings()->solver.contact_force_model = m_force_model;
    m_system->GetSettings()->solver.tangential_displ_mode = m_displ_mode;
    m_system->GetSettings()->solver.use_material_properties = m_use_mat_props;

    m_system->Set_G_acc(ChVector<>(0, 0, m_gacc));
    m_system->GetSettings()->solver.use_full_inertia_tensor = false;
    m_system->GetSettings()->solver.tolerance = 0.1;
    m_system->GetSettings()->solver.max_iteration_bilateral = 100;

    m_system->GetSettings()->collision.narrowphase_algorithm = collision::ChNarrowphase::Algorithm::HYBRID;
    m_system->GetSettings()->collision.collision_envelope = 0;

    m_system->SetNumThreads(m_num_threads);

#pragma omp parallel
#pragma omp master
    {
        // Sanity check: print number of threads in a parallel region
        if (m_verbose)
            cout << "[Terrain node] [" << m_rank << " " << m_sub_rank
                 << "]  actual number of OpenMP threads: " << omp_get_num_threads() << endl;
    }

    // Calculate container (half) height
    double r = m_separation_factor * m_radius_g;
    double delta = 2.0f * r;

    // Domain decomposition
    ChVector<> lo(-m_hdimX - delta, -m_hdimY - delta, -2 * m_radius_g);
    ChVector<> hi(+m_hdimX + delta, +m_hdimY + delta, +3 * m_radius_g);
    m_system->GetDomain()->SetSplitAxis(0);   //// TODO: let user specify this
    m_system->GetDomain()->SetSimDomain(lo, hi);

    // Estimates for number of bins for broad-phase.
    int factor = 2;
    auto& sub_lo = m_system->GetDomain()->GetSubLo();
    auto& sub_hi = m_system->GetDomain()->GetSubHi();
    auto sub_hdim = (sub_hi - sub_lo) / 2;
    int binsX = (int)std::ceil(sub_hdim.x() / m_radius_g) / factor;
    int binsY = (int)std::ceil(sub_hdim.y() / m_radius_g) / factor;
    int binsZ = 1;
    m_system->GetSettings()->collision.bins_per_axis = vec3(binsX, binsY, binsZ);
    if (m_verbose && m_sub_rank == 0)
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
            if (m_verbose && m_sub_rank == 0)
                cout << "   z =  " << z << "\tnum particles = " << points.size() << endl;
            z += delta;
        }
    } else {
        ChVector<> hdims(m_hdimX - r, m_hdimY - r, hdimZ - r);
        points = sampler->SampleBox(ChVector<>(0, 0, hdimZ), hdims);
    }

    // Create particle bodies with identifiers starting at m_Id_g
    m_num_particles = (unsigned int)points.size();
    if (m_verbose && m_sub_rank == 0)
        cout << "[Terrain node] Generated num particles = " << m_num_particles << endl;

    m_init_height = -std::numeric_limits<double>::max();
    int particle_id = body_id_particles;
    for (unsigned int i = 0; i < m_num_particles; i++) {
        auto body = chrono_types::make_shared<ChBody>(chrono_types::make_shared<collision::ChCollisionModelDistributed>());
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

    cout << "[Terrain node] initial height = " << m_init_height << endl;

    // ----------------------
    // Create rigid obstacles
    // ----------------------

    int obstacle_id = body_id_obstacles;
    for (auto& b : m_obstacles) {
        auto mat = b.m_contact_mat.CreateMaterial(m_system->GetContactMethod());
        auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(GetChronoDataFile(b.m_mesh_filename), true, true);
        double mass;
        ChVector<> baricenter;
        ChMatrix33<> inertia;
        trimesh->ComputeMassProperties(true, mass, baricenter, inertia);

        auto body = std::shared_ptr<ChBody>(m_system->NewBody());
        body->SetNameString("obstacle");
        body->SetIdentifier(obstacle_id++);
        body->SetPos(b.m_init_pos);
        body->SetRot(b.m_init_rot);
        body->SetMass(mass * b.m_density);
        body->SetInertia(inertia * b.m_density);
        body->SetBodyFixed(false);
        body->SetCollide(true);

        body->GetCollisionModel()->ClearModel();
        body->GetCollisionModel()->AddTriangleMesh(mat, trimesh, false, false, ChVector<>(0), ChMatrix33<>(1),
                                                   m_radius_g);
        body->GetCollisionModel()->SetFamily(2);
        body->GetCollisionModel()->BuildModel();

        auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(filesystem::path(b.m_mesh_filename).stem());
        trimesh_shape->Pos = ChVector<>(0, 0, 0);
        trimesh_shape->Rot = ChQuaternion<>(1, 0, 0, 0);
        body->GetAssets().push_back(trimesh_shape);

        m_system->AddBody(body);
    }

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
    if (m_sub_rank == 0) {
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
    for (const auto& body : m_system->Get_bodylist()) {
        if (body->GetIdentifier() > 0) {
            auto omg = body->GetWvel_par();
            auto J = body->GetInertiaXX();
            KE += body->GetMass() * body->GetPos_dt().Length2() + omg.Dot(J * omg);
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
            ChVector<> pos = (*bl_itr)->GetPos();
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

// Create bodies with triangular contact geometry as proxies for the tire mesh faces.
// Used for flexible tires.
// Assign to each body an identifier equal to the index of its corresponding mesh face.
// Maintain a list of all bodies associated with the tire.
// Add all proxy bodies to the same collision family and disable collision between any
// two members of this family.
void ChVehicleCosimTerrainNodeGranularMPI::CreateMeshProxies(unsigned int i) {
    //// RADU TODO:  better approximation of mass / inertia?
    double mass_p = m_load_mass[i] / m_mesh_data[i].nt;
    ChVector<> inertia_p = 1e-3 * mass_p * ChVector<>(0.1, 0.1, 0.1);

    for (unsigned int it = 0; it < m_mesh_data[i].nt; it++) {
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
        utils::AddTriangleGeometry(body.get(), m_material_tire[i], ChVector<>(len, 0, 0), ChVector<>(0, len, 0),
                                   ChVector<>(0, 0, len), name);
        body->GetCollisionModel()->SetFamily(1);
        body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
        body->GetCollisionModel()->BuildModel();

        m_system->AddBody(body);
        m_proxies[i].push_back(ProxyBody(body, it));
    }
}

void ChVehicleCosimTerrainNodeGranularMPI::CreateWheelProxy(unsigned int i) {
    auto body = std::shared_ptr<ChBody>(m_system->NewBody());
    body->SetIdentifier(0);
    body->SetMass(m_load_mass[i]);
    ////body->SetInertiaXX();   //// TODO
    body->SetBodyFixed(m_fixed_proxies);
    body->SetCollide(true);

    // Create collision mesh
    auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
    trimesh->getCoordsVertices() = m_mesh_data[i].verts;
    trimesh->getCoordsNormals() = m_mesh_data[i].norms;
    trimesh->getIndicesVertexes() = m_mesh_data[i].idx_verts;
    trimesh->getIndicesNormals() = m_mesh_data[i].idx_norms;

    // Set collision shape
    body->GetCollisionModel()->ClearModel();
    body->GetCollisionModel()->AddTriangleMesh(m_material_tire[i], trimesh, false, false, ChVector<>(0),
                                               ChMatrix33<>(1), m_radius_p);
    body->GetCollisionModel()->SetFamily(1);
    body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
    body->GetCollisionModel()->BuildModel();

    // Set visualization asset
    auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName("wheel_" + std::to_string(i));
    trimesh_shape->Pos = ChVector<>(0, 0, 0);
    trimesh_shape->Rot = ChQuaternion<>(1, 0, 0, 0);
    body->GetAssets().push_back(trimesh_shape);

    m_system->AddBody(body);
    m_proxies[i].push_back(ProxyBody(body, 0));
}

// Set position, orientation, and velocity of proxy bodies based on tire mesh faces.
// The proxy body is effectively reconstructed at each synchronization time:
//    - position at the center of mass of the three vertices
//    - orientation: identity
//    - linear and angular velocity: consistent with vertex velocities
//    - contact shape: redefined to match vertex locations
void ChVehicleCosimTerrainNodeGranularMPI::UpdateMeshProxies(unsigned int i, const MeshState& mesh_state) {
    auto& proxies = m_proxies[i];  // proxies for the i-th tire

    // shape_data contains all triangle vertex locations, in groups of three real3, one group for each triangle.
    auto& shape_data = m_system->data_manager->cd_data->shape_data.triangle_rigid;

    for (unsigned int it = 0; it < m_mesh_data[i].nt; it++) {
        // Vertex locations (expressed in global frame)
        const ChVector<>& pA = mesh_state.vpos[m_mesh_data[i].idx_verts[it].x()];
        const ChVector<>& pB = mesh_state.vpos[m_mesh_data[i].idx_verts[it].y()];
        const ChVector<>& pC = mesh_state.vpos[m_mesh_data[i].idx_verts[it].z()];

        // Position and orientation of proxy body
        ChVector<> pos = (pA + pB + pC) / 3;
        proxies[it].m_body->SetPos(pos);
        proxies[it].m_body->SetRot(ChQuaternion<>(1, 0, 0, 0));

        // Velocity (absolute) and angular velocity (local)
        // These are the solution of an over-determined 9x6 linear system. However, for a centroidal
        // body reference frame, the linear velocity is the average of the 3 vertex velocities.
        // This leaves a 9x3 linear system for the angular velocity which should be solved in a
        // least-square sense:   Ax = b   =>  (A'A)x = A'b
        const ChVector<>& vA = mesh_state.vvel[m_mesh_data[i].idx_verts[it].x()];
        const ChVector<>& vB = mesh_state.vvel[m_mesh_data[i].idx_verts[it].y()];
        const ChVector<>& vC = mesh_state.vvel[m_mesh_data[i].idx_verts[it].z()];

        ChVector<> vel = (vA + vB + vC) / 3;
        proxies[it].m_body->SetPos_dt(vel);

        //// RADU TODO: angular velocity
        proxies[it].m_body->SetWvel_loc(ChVector<>(0, 0, 0));

        // Update triangle contact shape (expressed in local frame) by writting directly
        // into the Chrono::Multicore data structures.
        // ATTENTION: It is assumed that no other triangle contact shapes have been added
        // to the system BEFORE those corresponding to the tire mesh faces!
        shape_data[3 * it + 0] = real3(pA.x() - pos.x(), pA.y() - pos.y(), pA.z() - pos.z());
        shape_data[3 * it + 1] = real3(pB.x() - pos.x(), pB.y() - pos.y(), pB.z() - pos.z());
        shape_data[3 * it + 2] = real3(pC.x() - pos.x(), pC.y() - pos.y(), pC.z() - pos.z());
    }

    if (m_verbose && m_sub_rank == 0)
        PrintMeshProxiesUpdateData(i, mesh_state);
}

// Set state of wheel proxy body.
void ChVehicleCosimTerrainNodeGranularMPI::UpdateWheelProxy(unsigned int i, const BodyState& spindle_state) {
    auto& proxies = m_proxies[i];  // proxies for the i-th tire

    proxies[0].m_body->SetPos(spindle_state.pos);
    proxies[0].m_body->SetPos_dt(spindle_state.lin_vel);
    proxies[0].m_body->SetRot(spindle_state.rot);
    proxies[0].m_body->SetWvel_par(spindle_state.ang_vel);
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

// Collect contact forces on the (face) proxy bodies that are in contact.
// Load mesh vertex forces and corresponding indices.
void ChVehicleCosimTerrainNodeGranularMPI::GetForcesMeshProxies(unsigned int i, MeshContact& mesh_contact) {
    const auto& proxies = m_proxies[i];  // proxies for the i-th tire

    // Maintain an unordered map of vertex indices and associated contact forces.
    std::unordered_map<int, ChVector<>> my_map;

    for (unsigned int it = 0; it < m_mesh_data[i].nt; it++) {
        // Get cumulative contact force at triangle centroid.
        // Do nothing if zero force.
        real3 rforce = m_system->GetBodyContactForce(proxies[it].m_body->GetGid());
        if (IsZero(rforce))
            continue;

        // Centroid has barycentric coordinates {1/3, 1/3, 1/3}, so force is
        // distributed equally to the three vertices.
        ChVector<> force(rforce.x / 3, rforce.y / 3, rforce.z / 3);

        // For each vertex of the triangle, if it appears in the map, increment
        // the total contact force. Otherwise, insert a new entry in the map.
        auto v1 = my_map.find(m_mesh_data[i].idx_verts[it].x());
        if (v1 != my_map.end()) {
            v1->second += force;
        } else {
            my_map[m_mesh_data[i].idx_verts[it].x()] = force;
        }

        auto v2 = my_map.find(m_mesh_data[i].idx_verts[it].y());
        if (v2 != my_map.end()) {
            v2->second += force;
        } else {
            my_map[m_mesh_data[i].idx_verts[it].y()] = force;
        }

        auto v3 = my_map.find(m_mesh_data[i].idx_verts[it].z());
        if (v3 != my_map.end()) {
            v3->second += force;
        } else {
            my_map[m_mesh_data[i].idx_verts[it].z()] = force;
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

// Collect resultant contact force and torque on wheel proxy body.
void ChVehicleCosimTerrainNodeGranularMPI::GetForceWheelProxy(unsigned int i, TerrainForce& wheel_contact) {
    const auto& proxies = m_proxies[i];  // proxies for the i-th tire

    wheel_contact.point = ChVector<>(0, 0, 0);
    wheel_contact.force = proxies[0].m_body->GetContactForce();
    wheel_contact.moment = proxies[0].m_body->GetContactTorque();
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

    utils::CSV_writer csv(" ");
    WriteParticleInformation(csv);
    csv.write_to_file(filename);
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeGranularMPI::WriteParticleInformation(utils::CSV_writer& csv) {
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

void ChVehicleCosimTerrainNodeGranularMPI::WriteCheckpoint(const std::string& filename) const {
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
    if (m_verbose && m_sub_rank == 0)
        cout << "[Terrain node] write checkpoint ===> " << checkpoint_filename << endl;
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeGranularMPI::PrintMeshProxiesUpdateData(unsigned int i, const MeshState& mesh_state) {
    {
        auto lowest = std::min_element(
            m_proxies[i].begin(), m_proxies[i].end(),
            [](const ProxyBody& a, const ProxyBody& b) { return a.m_body->GetPos().z() < b.m_body->GetPos().z(); });
        const ChVector<>& vel = (*lowest).m_body->GetPos_dt();
        double height = (*lowest).m_body->GetPos().z();
        cout << "[Terrain node] tire: " << i << "  lowest proxy:  index = " << (*lowest).m_index
             << "  height = " << height << "  velocity = " << vel.x() << "  " << vel.y() << "  " << vel.z() << endl;
    }

    {
        auto lowest = std::min_element(mesh_state.vpos.begin(), mesh_state.vpos.end(),
                                       [](const ChVector<>& a, const ChVector<>& b) { return a.z() < b.z(); });
        cout << "[Terrain node] tire: " << i << "  lowest vertex:  height = " << (*lowest).z() << endl;
    }
}

}  // end namespace vehicle
}  // end namespace chrono
