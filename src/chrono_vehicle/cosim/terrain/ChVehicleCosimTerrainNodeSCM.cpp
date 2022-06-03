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
// Definition of the SCM deformable TERRAIN NODE.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <algorithm>
#include <cmath>
#include <set>

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono_vehicle/cosim/terrain/ChVehicleCosimTerrainNodeSCM.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#endif

using std::cout;
using std::endl;

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// Maximum sinkage for rendering
static const double max_sinkage = 0.15;

// -----------------------------------------------------------------------------
// Construction of the terrain node:
// - create the Chrono system and set solver parameters
// - create the Irrlicht visualization window
// -----------------------------------------------------------------------------
ChVehicleCosimTerrainNodeSCM::ChVehicleCosimTerrainNodeSCM(double length, double width)
    : ChVehicleCosimTerrainNodeChrono(Type::SCM, length, width, ChContactMethod::SMC),
      m_radius_p(5e-3),
      m_use_checkpoint(false) {

    // Create system and set default method-specific solver settings
    m_system = new ChSystemSMC;

    // Solver settings independent of method type
    m_system->Set_G_acc(ChVector<>(0, 0, m_gacc));

    // Set default number of threads
    m_system->SetNumThreads(1, 1, 1);
}

ChVehicleCosimTerrainNodeSCM::ChVehicleCosimTerrainNodeSCM(const std::string& specfile)
    : ChVehicleCosimTerrainNodeChrono(Type::SCM, 0, 0, ChContactMethod::SMC), m_use_checkpoint(false) {

    // Create system and set default method-specific solver settings
    m_system = new ChSystemSMC;

    // Solver settings independent of method type
    m_system->Set_G_acc(ChVector<>(0, 0, m_gacc));

    // Read SCM parameters from provided specfile
    SetFromSpecfile(specfile);
}

ChVehicleCosimTerrainNodeSCM::~ChVehicleCosimTerrainNodeSCM() {
    delete m_terrain;
    delete m_system;
}

// -----------------------------------------------------------------------------

//// TODO: error checking
void ChVehicleCosimTerrainNodeSCM::SetFromSpecfile(const std::string& specfile) {
    Document d;
    ReadSpecfile(specfile, d);

    double length = d["Patch dimensions"]["Length"].GetDouble();
    double width = d["Patch dimensions"]["Width"].GetDouble();
    m_hdimX = length / 2;
    m_hdimY = width / 2;

    m_spacing = d["Grid spacing"].GetDouble();

    m_Bekker_Kphi = d["Soil parameters"]["Bekker Kphi"].GetDouble();
    m_Bekker_Kc = d["Soil parameters"]["Bekker Kc"].GetDouble();
    m_Bekker_n = d["Soil parameters"]["Bekker n exponent"].GetDouble();
    m_Mohr_cohesion = d["Soil parameters"]["Mohr cohesive limit"].GetDouble();
    m_Mohr_friction = d["Soil parameters"]["Mohr friction limit"].GetDouble();
    m_Janosi_shear = d["Soil parameters"]["Janosi shear coefficient"].GetDouble();

    m_elastic_K = d["Soil parameters"]["Elastic stiffness"].GetDouble();
    m_damping_R = d["Soil parameters"]["Damping"].GetDouble();

    m_radius_p = d["Simulation settings"]["Proxy contact radius"].GetDouble();
    m_fixed_proxies = d["Simulation settings"]["Fix proxies"].GetBool();
}

void ChVehicleCosimTerrainNodeSCM::SetPropertiesSCM(double spacing,
                                                    double Bekker_Kphi,
                                                    double Bekker_Kc,
                                                    double Bekker_n,
                                                    double Mohr_cohesion,
                                                    double Mohr_friction,
                                                    double Janosi_shear,
                                                    double elastic_K,
                                                    double damping_R) {
    m_spacing = spacing;

    m_Bekker_Kphi = Bekker_Kphi;
    m_Bekker_Kc = Bekker_Kc;
    m_Bekker_n = Bekker_n;
    m_Mohr_cohesion = Mohr_cohesion;
    m_Mohr_friction = Mohr_friction;
    m_Janosi_shear = Janosi_shear;

    m_elastic_K = elastic_K;
    m_damping_R = damping_R;
}

void ChVehicleCosimTerrainNodeSCM::SetInputFromCheckpoint(const std::string& filename) {
    m_use_checkpoint = true;
    m_checkpoint_filename = filename;
}

void ChVehicleCosimTerrainNodeSCM::SetNumThreads(int num_threads) {
    m_system->SetNumThreads(num_threads, 1, 1);
}

// -----------------------------------------------------------------------------
// Complete construction of the mechanical system.
// This function is invoked automatically from Initialize.
// - adjust system settings
// - create the container body
// - if specified, create the granular material
// -----------------------------------------------------------------------------
void ChVehicleCosimTerrainNodeSCM::Construct() {
    if (m_verbose)
        cout << "[Terrain node] SCM " << endl;

    // Create the SCM patch (default center at origin)
    m_terrain = new SCMDeformableTerrain(m_system);
    m_terrain->SetSoilParameters(m_Bekker_Kphi, m_Bekker_Kc, m_Bekker_n,            //
                                 m_Mohr_cohesion, m_Mohr_friction, m_Janosi_shear,  //
                                 m_elastic_K, m_damping_R);
    m_terrain->SetPlotType(vehicle::SCMDeformableTerrain::PLOT_SINKAGE, 0, max_sinkage);
    m_terrain->Initialize(2 * m_hdimX, 2 * m_hdimY, m_spacing);

    // If indicated, set node heights from checkpoint file
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

        // Read number of modified nodes
        int num_nodes;
        std::getline(ifile, line);
        std::istringstream iss(line);
        iss >> num_nodes;

        std::vector<SCMDeformableTerrain::NodeLevel> nodes(num_nodes);
        for (int i = 0; i < num_nodes; i++) {
            std::getline(ifile, line);
            std::istringstream iss1(line);
            int x, y;
            double h;
            iss1 >> x >> y >> h;
            nodes[i] = std::make_pair(ChVector2<>(x, y), h);
        }

        m_terrain->SetModifiedNodes(nodes);

        if (m_verbose)
            cout << "[Terrain node] read " << checkpoint_filename << "   num. nodes = " << num_nodes << endl;
    }

    // Add all rigid obstacles
    for (auto& b : m_obstacles) {
        auto mat = b.m_contact_mat.CreateMaterial(m_system->GetContactMethod());
        auto trimesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(b.m_mesh_filename),
                                                                                  true, true);
        double mass;
        ChVector<> baricenter;
        ChMatrix33<> inertia;
        trimesh->ComputeMassProperties(true, mass, baricenter, inertia);

        auto body = std::shared_ptr<ChBody>(m_system->NewBody());
        body->SetPos(b.m_init_pos);
        body->SetRot(b.m_init_rot);
        body->SetMass(mass * b.m_density);
        body->SetInertia(inertia * b.m_density);
        body->SetBodyFixed(false);
        body->SetCollide(true);

        body->GetCollisionModel()->ClearModel();
        body->GetCollisionModel()->AddTriangleMesh(mat, trimesh, false, false, ChVector<>(0), ChMatrix33<>(1),
                                                   m_radius_p);
        body->GetCollisionModel()->SetFamily(2);
        body->GetCollisionModel()->BuildModel();

        auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        body->AddVisualShape(trimesh_shape, ChFrame<>());

        // Add corresponding moving patch to SCM terrain
        m_terrain->AddMovingPatch(body, b.m_oobb_center, b.m_oobb_dims);

        m_system->AddBody(body);
    }

#ifdef CHRONO_IRRLICHT
    // Create the visualization window
    if (m_render) {
        m_vsys = chrono_types::make_shared<irrlicht::ChVisualSystemIrrlicht>();
        m_system->SetVisualSystem(m_vsys);
        m_vsys->SetCameraVertical(CameraVerticalDir::Z);
        m_vsys->SetWindowSize(1280, 720);
        m_vsys->SetWindowTitle("Terrain Node (SCM)");
        m_vsys->Initialize();
        m_vsys->AddLogo();
        m_vsys->AddSkyBox();
        m_vsys->AddTypicalLights();
        m_vsys->AddCamera(ChVector<>(m_hdimX, 1.4, 1.0), ChVector<>(0, 0, 0));
    }
#endif

    // Write file with terrain node settings
    std::ofstream outf;
    outf.open(m_node_out_dir + "/settings.info", std::ios::out);
    outf << "System settings" << endl;
    outf << "  Integration step size = " << m_step_size << endl;
    outf << "Patch dimensions" << endl;
    outf << "  X = " << 2 * m_hdimX << "  Y = " << 2 * m_hdimY << endl;
    outf << "  spacing = " << m_spacing << endl;
    outf << "Terrain material properties" << endl;
    outf << "  Kphi = " << m_Bekker_Kphi << endl;
    outf << "  Kc   = " << m_Bekker_Kc << endl;
    outf << "  n    = " << m_Bekker_n << endl;
    outf << "  c    = " << m_Mohr_cohesion << endl;
    outf << "  mu   = " << m_Mohr_friction << endl;
    outf << "  J    = " << m_Janosi_shear << endl;
    outf << "  Ke   = " << m_elastic_K << endl;
    outf << "  Rd   = " << m_damping_R << endl;
}

// Create bodies with triangular contact geometry as proxies for the tire mesh faces.
// Used for flexible tires.
// Assign to each body an identifier equal to the index of its corresponding mesh face.
// Maintain a list of all bodies associated with the tire.
// Add all proxy bodies to the same collision family and disable collision between any
// two members of this family.
void ChVehicleCosimTerrainNodeSCM::CreateMeshProxies(unsigned int i) {
    //// TODO

#ifdef CHRONO_IRRLICHT
    // Bind Irrlicht assets
    if (m_render) {
        m_vsys->BindAll();
    }
#endif
}

void ChVehicleCosimTerrainNodeSCM::CreateWheelProxy(unsigned int i) {
    auto material_tire = m_mat_props[i].CreateMaterial(m_method);

    // Create wheel proxy body
    auto body = std::shared_ptr<ChBody>(m_system->NewBody());
    body->SetIdentifier(0);
    body->SetMass(m_load_mass[i]);
    ////body->SetInertiaXX();   //// TODO
    body->SetBodyFixed(false);  // Cannot fix the proxies with SCM
    body->SetCollide(true);

    // Create collision mesh
    auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
    trimesh->getCoordsVertices() = m_mesh_data[i].verts;
    trimesh->getCoordsNormals() = m_mesh_data[i].norms;
    trimesh->getIndicesVertexes() = m_mesh_data[i].idx_verts;
    trimesh->getIndicesNormals() = m_mesh_data[i].idx_norms;

    // Set collision shape
    body->GetCollisionModel()->ClearModel();
    body->GetCollisionModel()->AddTriangleMesh(material_tire, trimesh, false, false, ChVector<>(0), ChMatrix33<>(1),
                                               m_radius_p);
    body->GetCollisionModel()->SetFamily(1);
    body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
    body->GetCollisionModel()->BuildModel();

    // Set visualization asset
    auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    trimesh_shape->SetMesh(trimesh);
    body->AddVisualShape(trimesh_shape, ChFrame<>());

    m_system->AddBody(body);
    m_proxies[i].push_back(ProxyBody(body, 0));

    // Add corresponding moving patch to SCM terrain
    m_terrain->AddMovingPatch(body, ChVector<>(0, 0, 0),
                              ChVector<>(2 * m_tire_radius[i], m_tire_width[i], 2 * m_tire_radius[i]));

#ifdef CHRONO_IRRLICHT
    // Bind Irrlicht assets
    if (m_render) {
        m_vsys->BindAll();
    }
#endif
}

// Set position, orientation, and velocity of proxy bodies based on tire mesh faces.
void ChVehicleCosimTerrainNodeSCM::UpdateMeshProxies(unsigned int i, MeshState& mesh_state) {
    //// TODO
}

// Set state of wheel proxy body.
void ChVehicleCosimTerrainNodeSCM::UpdateWheelProxy(unsigned int i, BodyState& spindle_state) {
    auto& proxies = m_proxies[i];  // proxies for the i-th tire

    proxies[0].m_body->SetPos(spindle_state.pos);
    proxies[0].m_body->SetPos_dt(spindle_state.lin_vel);
    proxies[0].m_body->SetRot(spindle_state.rot);
    proxies[0].m_body->SetWvel_par(spindle_state.ang_vel);
}

// Collect contact forces on the (face) proxy bodies that are in contact.
// Load mesh vertex forces and corresponding indices.
void ChVehicleCosimTerrainNodeSCM::GetForcesMeshProxies(unsigned int i, MeshContact& mesh_contact) {
    //// TODO
}

// Collect resultant contact force and torque on wheel proxy body.
void ChVehicleCosimTerrainNodeSCM::GetForceWheelProxy(unsigned int i, TerrainForce& wheel_contact) {
    const auto& proxies = m_proxies[i];  // proxies for the i-th tire

    wheel_contact = m_terrain->GetContactForce(proxies[0].m_body);
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeSCM::Render(double time) {
#ifdef CHRONO_IRRLICHT
    if (!m_vsys->Run()) {
        MPI_Abort(MPI_COMM_WORLD, 1);
    }
    m_vsys->BeginScene();
    m_vsys->DrawAll();
    irrlicht::tools::drawColorbar(m_vsys.get(), 0, max_sinkage, "Sinkage [m]", 1180);
    m_vsys->EndScene();
#endif
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeSCM::OnOutputData(int frame) {
    //// TODO
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeSCM::WriteCheckpoint(const std::string& filename) const {
    utils::CSV_writer csv(" ");

    // Get all SCM grid nodes modified from start of simulation
    const auto& nodes = m_terrain->GetModifiedNodes(true);

    // Write current time and total number of modified grid nodes.
    csv << m_system->GetChTime() << endl;
    csv << nodes.size() << endl;

    // Write node locations and heights
    for (const auto& node : nodes) {
        csv << node.first.x() << node.first.y() << node.second << endl;
    }

    std::string checkpoint_filename = m_node_out_dir + "/" + filename;
    csv.write_to_file(checkpoint_filename);
    if (m_verbose)
        cout << "[Terrain node] write checkpoint ===> " << checkpoint_filename << endl;
}

}  // end namespace vehicle
}  // end namespace chrono
