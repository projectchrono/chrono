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

#include <mpi.h>

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono_vehicle/cosim/ChVehicleCosimTerrainNodeSCM.h"

#ifdef CHRONO_IRRLICHT
#include "chrono_irrlicht/ChIrrApp.h"
#endif

using std::cout;
using std::endl;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Construction of the terrain node:
// - create the Chrono system and set solver parameters
// - create the Irrlicht visualization window
// -----------------------------------------------------------------------------
ChVehicleCosimTerrainNodeSCM::ChVehicleCosimTerrainNodeSCM(int num_threads)
    : ChVehicleCosimTerrainNode(Type::SCM, ChContactMethod::SMC), m_radius_p(5e-3), m_use_checkpoint(false) {
    cout << "[Terrain node] SCM " << endl;

#ifdef CHRONO_IRRLICHT
    m_irrapp = nullptr;
#endif

    // Create system and set default method-specific solver settings
    m_system = new ChSystemSMC;

    // Solver settings independent of method type
    m_system->Set_G_acc(ChVector<>(0, 0, m_gacc));

    // Set number of threads
    m_system->SetNumThreads(num_threads, 1, 1);
}

ChVehicleCosimTerrainNodeSCM::~ChVehicleCosimTerrainNodeSCM() {
#ifdef CHRONO_IRRLICHT
    delete m_irrapp;
#endif
    delete m_terrain;
    delete m_system;
}

// -----------------------------------------------------------------------------

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

// -----------------------------------------------------------------------------
// Complete construction of the mechanical system.
// This function is invoked automatically from Initialize.
// - adjust system settings
// - create the container body
// - if specified, create the granular material
// -----------------------------------------------------------------------------
void ChVehicleCosimTerrainNodeSCM::Construct() {
    // Create the SCM patch (default center at origin)
    m_terrain = new SCMDeformableTerrain(m_system);
    m_terrain->SetSoilParameters(m_Bekker_Kphi, m_Bekker_Kc, m_Bekker_n,            //
                                 m_Mohr_cohesion, m_Mohr_friction, m_Janosi_shear,  //
                                 m_elastic_K, m_damping_R);
    m_terrain->SetPlotType(vehicle::SCMDeformableTerrain::PLOT_SINKAGE, 0, 0.05);
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
            std::istringstream iss(line);
            int x, y;
            double h;
            iss >> x >> y >> h;
            nodes[i] = std::make_pair(ChVector2<>(x, y), h);
        }

        m_terrain->SetModifiedNodes(nodes);

        cout << "[Terrain node] read " << checkpoint_filename << "   num. nodes = " << num_nodes << endl;
    }

#ifdef CHRONO_IRRLICHT
    // Create the visualization window
    if (m_render) {
        m_irrapp = new irrlicht::ChIrrApp(m_system, L"Terrain Node (SCM)", irr::core::dimension2d<irr::u32>(1280, 720),
                                          irrlicht::VerticalDir::Y, false, true);
        m_irrapp->AddTypicalLogo();
        m_irrapp->AddTypicalSky();
        m_irrapp->AddTypicalLights();
        m_irrapp->AddTypicalCamera(irr::core::vector3df(2.0f, 1.4f, 0.0f), irr::core::vector3df(0, 0, 0));
        m_irrapp->AddLightWithShadow(irr::core::vector3df(1.5f, 5.5f, -2.5f), irr::core::vector3df(0, 0, 0), 3, 2.2,
                                     7.2, 40, 512, irr::video::SColorf(0.8f, 0.8f, 1.0f));
    }
#endif

    // Write file with terrain node settings
    std::ofstream outf;
    outf.open(m_node_out_dir + "/settings.dat", std::ios::out);
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
void ChVehicleCosimTerrainNodeSCM::CreateMeshProxies() {
    //// TODO

#ifdef CHRONO_IRRLICHT
    // Bind Irrlicht assets
    if (m_render) {
        m_irrapp->AssetBindAll();
        m_irrapp->AssetUpdateAll();
    }
#endif
}

void ChVehicleCosimTerrainNodeSCM::CreateWheelProxy() {
    // Create wheel proxy body
    auto body = std::shared_ptr<ChBody>(m_system->NewBody());
    body->SetIdentifier(0);
    body->SetMass(m_rig_mass);
    ////body->SetInertiaXX();   //// TODO
    body->SetBodyFixed(false);  // Cannot fix the proxies with SCM
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

#ifdef CHRONO_IRRLICHT
    // Bind Irrlicht assets
    if (m_render) {
        m_irrapp->AssetBindAll();
        m_irrapp->AssetUpdateAll();
    }
#endif
}

// Set position, orientation, and velocity of proxy bodies based on tire mesh faces.
void ChVehicleCosimTerrainNodeSCM::UpdateMeshProxies() {
    //// TODO
}

// Set state of wheel proxy body.
void ChVehicleCosimTerrainNodeSCM::UpdateWheelProxy() {
    m_proxies[0].m_body->SetPos(m_wheel_state.pos);
    m_proxies[0].m_body->SetPos_dt(m_wheel_state.lin_vel);
    m_proxies[0].m_body->SetRot(m_wheel_state.rot);
    m_proxies[0].m_body->SetWvel_par(m_wheel_state.ang_vel);
}

// Collect contact forces on the (face) proxy bodies that are in contact.
// Load mesh vertex forces and corresponding indices.
void ChVehicleCosimTerrainNodeSCM::GetForcesMeshProxies() {
    //// TODO
}

// Collect resultant contact force and torque on wheel proxy body.
void ChVehicleCosimTerrainNodeSCM::GetForceWheelProxy() {
    m_wheel_contact = m_terrain->GetContactForce(m_proxies[0].m_body);
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeSCM::OnRender(double time) {
#ifdef CHRONO_IRRLICHT
    if (!m_irrapp->GetDevice()->run()) {
        MPI_Abort(MPI_COMM_WORLD, 1);
    }
    m_irrapp->BeginScene();
    ////m_irrapp->GetSceneManager()->getActiveCamera()->setTarget(irr::core::vector3dfCH(mrigidbody->GetPos()));
    m_irrapp->DrawAll();
    irrlicht::tools::drawColorbar(0, 30000, "Pressure yield [Pa]", m_irrapp->GetDevice(), 1180);
    m_irrapp->EndScene();
#endif
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeSCM::OnOutputData(int frame) {
    //// TODO
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeSCM::WriteCheckpoint(const std::string& filename) {
    utils::CSV_writer csv(" ");

    // Get all SCM grid nodes modified from start of simulation
    auto& nodes = m_terrain->GetModifiedNodes(true);

    // Write current time and total number of modified grid nodes.
    csv << m_system->GetChTime() << endl;
    csv << nodes.size() << endl;

    // Write node locations and heights
    for (auto& node : nodes) {
        csv << node.first.x() << node.first.y() << node.second << endl;
    }

    std::string checkpoint_filename = m_node_out_dir + "/" + filename;
    csv.write_to_file(checkpoint_filename);
    cout << "[Terrain node] write checkpoint ===> " << checkpoint_filename << endl;
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeSCM::PrintMeshProxiesUpdateData() {
    //// TODO
}

void ChVehicleCosimTerrainNodeSCM::PrintWheelProxyUpdateData() {
    //// TODO
}

void ChVehicleCosimTerrainNodeSCM::PrintMeshProxiesContactData() {
    //// TODO
}

void ChVehicleCosimTerrainNodeSCM::PrintWheelProxyContactData() {
    //// TODO
}

}  // end namespace vehicle
}  // end namespace chrono
