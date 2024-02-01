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

#include "chrono/assets/ChVisualShapeTriangleMesh.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono_vehicle/cosim/terrain/ChVehicleCosimTerrainNodeSCM.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#endif
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
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
      m_terrain(nullptr),
      m_radius_p(5e-3),
      m_use_checkpoint(false) {
    // Create system and set default method-specific solver settings
    m_system = new ChSystemSMC;

    // Create an associated collision system
    m_system->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Solver settings independent of method type
    m_system->Set_G_acc(ChVector<>(0, 0, m_gacc));

    // Set default number of threads
    m_system->SetNumThreads(1, 1, 1);
}

ChVehicleCosimTerrainNodeSCM::ChVehicleCosimTerrainNodeSCM(const std::string& specfile)
    : ChVehicleCosimTerrainNodeChrono(Type::SCM, 0, 0, ChContactMethod::SMC),
      m_terrain(nullptr),
      m_use_checkpoint(false) {
    // Create system and set default method-specific solver settings
    m_system = new ChSystemSMC;

    // Create an associated collision system
    m_system->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Solver settings independent of method type
    m_system->Set_G_acc(ChVector<>(0, 0, m_gacc));

    // Set default number of threads
    m_system->SetNumThreads(1, 1, 1);

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

    m_dimX = d["Patch dimensions"]["Length"].GetDouble();
    m_dimY = d["Patch dimensions"]["Width"].GetDouble();

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
    m_terrain = new SCMTerrain(m_system);
    m_terrain->SetSoilParameters(m_Bekker_Kphi, m_Bekker_Kc, m_Bekker_n,            //
                                 m_Mohr_cohesion, m_Mohr_friction, m_Janosi_shear,  //
                                 m_elastic_K, m_damping_R);
    m_terrain->SetPlotType(vehicle::SCMTerrain::PLOT_SINKAGE, 0, max_sinkage);
    m_terrain->SetMeshWireframe(false);
    m_terrain->SetCosimulationMode(true);
    m_terrain->Initialize(m_dimX, m_dimY, m_spacing);

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

        std::vector<SCMTerrain::NodeLevel> nodes(num_nodes);
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

        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos(b.m_init_pos);
        body->SetRot(b.m_init_rot);
        body->SetMass(mass * b.m_density);
        body->SetInertia(inertia * b.m_density);
        body->SetBodyFixed(false);

        body->SetCollide(true);
        auto ct_shape =
            chrono_types::make_shared<ChCollisionShapeTriangleMesh>(mat, trimesh, false, false, m_radius_p);
        body->AddCollisionShape(ct_shape);
        body->GetCollisionModel()->SetFamily(2);

        auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        trimesh_shape->SetMesh(trimesh);
        body->AddVisualShape(trimesh_shape, ChFrame<>());

        // Add corresponding moving patch to SCM terrain
        m_terrain->AddMovingPatch(body, b.m_oobb_center, b.m_oobb_dims);

        m_system->AddBody(body);
    }

    // Write file with terrain node settings
    std::ofstream outf;
    outf.open(m_node_out_dir + "/settings.info", std::ios::out);
    outf << "System settings" << endl;
    outf << "  Integration step size = " << m_step_size << endl;
    outf << "Patch dimensions" << endl;
    outf << "  X = " << m_dimX << "  Y = " << m_dimY << endl;
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

// Create bodies with triangular contact geometry as proxies for the mesh faces.
// Used for flexible bodies.
// Assign to each body an identifier equal to the index of its corresponding mesh face.
// Maintain a list of all bodies associated with the object.
// Add all proxy bodies to the same collision family and disable collision between any
// two members of this family.
void ChVehicleCosimTerrainNodeSCM::CreateMeshProxy(unsigned int i) {
    // Get shape associated with the given object
    int i_shape = m_obj_map[i];

    // Create the proxy associated with the given object
    auto proxy = chrono_types::make_shared<ProxyMesh>();

    // Note: it is assumed that there is one and only one mesh defined!
    const auto& trimesh_shape = m_geometry[i_shape].m_coll_meshes[0];
    const auto& trimesh = trimesh_shape.m_trimesh;
    auto material = m_geometry[i_shape].m_materials[trimesh_shape.m_matID].CreateMaterial(m_method);

    //// RADU TODO:  better approximation of mass / inertia?
    ////double mass_p = m_load_mass[i_shape] / trimesh->getNumTriangles();

    // Create a contact surface mesh constructed from the provided trimesh
    auto surface = chrono_types::make_shared<fea::ChContactSurfaceMesh>(material);
    surface->ConstructFromTrimesh(trimesh, m_radius_p);

    // Create maps from pointer-based to index-based for the nodes in the mesh contact surface.
    // Note that here, the contact surface includes all faces in the geometry trimesh..
    int vertex_index = 0;
    for (const auto& tri : surface->GetTriangleList()) {
        if (proxy->ptr2ind_map.insert({tri->GetNode(0), vertex_index}).second) {
            proxy->ind2ptr_map.insert({vertex_index, tri->GetNode(0)});
            ++vertex_index;
        }
        if (proxy->ptr2ind_map.insert({tri->GetNode(1), vertex_index}).second) {
            proxy->ind2ptr_map.insert({vertex_index, tri->GetNode(1)});
            ++vertex_index;
        }
        if (proxy->ptr2ind_map.insert({tri->GetNode(2), vertex_index}).second) {
            proxy->ind2ptr_map.insert({vertex_index, tri->GetNode(2)});
            ++vertex_index;
        }
    }

    assert(proxy->ptr2ind_map.size() == surface->GetNumVertices());
    assert(proxy->ind2ptr_map.size() == surface->GetNumVertices());

    // Construct the FEA mesh and add to it the contact surface.
    // Do not add nodes to the mesh, else they will be polluted during integration
    proxy->mesh = chrono_types::make_shared<fea::ChMesh>();
    proxy->mesh->AddContactSurface(surface);

    auto vis_mesh = chrono_types::make_shared<ChVisualShapeFEA>(proxy->mesh);
    vis_mesh->SetFEMdataType(ChVisualShapeFEA::DataType::CONTACTSURFACES);
    vis_mesh->SetWireframe(true);
    proxy->mesh->AddVisualShapeFEA(vis_mesh);

    m_system->AddMesh(proxy->mesh);
    m_system->GetCollisionSystem()->BindItem(proxy->mesh);

    m_proxies[i] = proxy;
}

void ChVehicleCosimTerrainNodeSCM::CreateRigidProxy(unsigned int i) {
    // Get shape associated with the given object
    int i_shape = m_obj_map[i];

    // Create the proxy associated with the given object
    auto proxy = chrono_types::make_shared<ProxyBodySet>();

    // Create wheel proxy body
    auto body = chrono_types::make_shared<ChBody>();
    body->SetIdentifier(0);
    body->SetMass(m_load_mass[i_shape]);
    ////body->SetInertiaXX();   //// TODO
    body->SetBodyFixed(false);  // Cannot fix the proxies with SCM
    body->SetCollide(true);

    // Create visualization asset (use collision shapes)
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

    // Add corresponding moving patch to SCM terrain
    //// RADU TODO: this may be overkill for tracked vehicles!
    m_terrain->AddMovingPatch(body, m_aabb[i_shape].Center(), m_aabb[i_shape].Size());
}

// Once all proxy bodies are created, complete construction of the underlying system.
void ChVehicleCosimTerrainNodeSCM::OnInitialize(unsigned int num_objects) {
    ChVehicleCosimTerrainNodeChrono::OnInitialize(num_objects);

    // Create the visualization window
    if (m_renderRT) {
#if defined(CHRONO_VSG)
        auto vsys_vsg = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        vsys_vsg->AttachSystem(m_system);
        vsys_vsg->SetWindowTitle("Terrain Node (SCM)");
        vsys_vsg->SetWindowSize(ChVector2<int>(1280, 720));
        vsys_vsg->SetWindowPosition(ChVector2<int>(100, 100));
        vsys_vsg->SetUseSkyBox(false);
        vsys_vsg->SetClearColor(ChColor(0.455f, 0.525f, 0.640f));
        vsys_vsg->AddCamera(m_cam_pos, ChVector<>(0, 0, 0));
        vsys_vsg->SetCameraAngleDeg(40);
        vsys_vsg->SetLightIntensity(1.0f);
        ////vsys_vsg->AddGuiColorbar("Sinkage (m)", 0.0, 0.1);
        vsys_vsg->SetImageOutputDirectory(m_node_out_dir + "/images");
        vsys_vsg->SetImageOutput(m_writeRT);
        vsys_vsg->Initialize();

        m_vsys = vsys_vsg;
#elif defined(CHRONO_IRRLICHT)
        auto vsys_irr = chrono_types::make_shared<irrlicht::ChVisualSystemIrrlicht>();
        vsys_irr->AttachSystem(m_system);
        vsys_irr->SetWindowTitle("Terrain Node (SCM)");
        vsys_irr->SetCameraVertical(CameraVerticalDir::Z);
        vsys_irr->SetWindowSize(1280, 720);
        vsys_irr->Initialize();
        vsys_irr->AddLogo();
        vsys_irr->AddSkyBox();
        vsys_irr->AddTypicalLights();
        vsys_irr->AddCamera(m_cam_pos, ChVector<>(0, 0, 0));

        m_vsys = vsys_irr;
#endif
    }
}

// Set position, orientation, and velocity of proxy bodies based on mesh faces.
void ChVehicleCosimTerrainNodeSCM::UpdateMeshProxy(unsigned int i, MeshState& mesh_state) {
    // Get the proxy (contact surface) associated with this object
    auto proxy = std::static_pointer_cast<ProxyMesh>(m_proxies[i]);

    int num_vertices = (int)mesh_state.vpos.size();

    for (int in = 0; in < num_vertices; in++) {
        auto& node = proxy->ind2ptr_map[in];
        node->SetPos(mesh_state.vpos[in]);
        node->SetPos_dt(mesh_state.vvel[in]);
    }
}

// Set state of proxy rigid body.
void ChVehicleCosimTerrainNodeSCM::UpdateRigidProxy(unsigned int i, BodyState& rigid_state) {
    auto proxy = std::static_pointer_cast<ProxyBodySet>(m_proxies[i]);
    proxy->bodies[0]->SetPos(rigid_state.pos);
    proxy->bodies[0]->SetPos_dt(rigid_state.lin_vel);
    proxy->bodies[0]->SetRot(rigid_state.rot);
    proxy->bodies[0]->SetWvel_par(rigid_state.ang_vel);
}

// Collect contact forces on the (face) proxy bodies that are in contact.
// Load mesh vertex forces and corresponding indices.
void ChVehicleCosimTerrainNodeSCM::GetForceMeshProxy(unsigned int i, MeshContact& mesh_contact) {
    auto proxy = std::static_pointer_cast<ProxyMesh>(m_proxies[i]);

    ChVector<> force;
    mesh_contact.nv = 0;
    for (const auto& node : proxy->ptr2ind_map) {
        bool in_contact = m_terrain->GetContactForceNode(node.first, force);
        if (in_contact) {
            mesh_contact.vidx.push_back(node.second);
            mesh_contact.vforce.push_back(force);
            mesh_contact.nv++;
        }
    }
}

// Collect resultant contact force and torque on rigid proxy body.
void ChVehicleCosimTerrainNodeSCM::GetForceRigidProxy(unsigned int i, TerrainForce& rigid_contact) {
    auto proxy = std::static_pointer_cast<ProxyBodySet>(m_proxies[i]);
    
    ChVector<> force;
    ChVector<>torque;
    m_terrain->GetContactForceBody(proxy->bodies[0], force, torque);

    rigid_contact.point = proxy->bodies[0]->GetPos();
    rigid_contact.force = force;
    rigid_contact.moment = torque;
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeSCM::OnRender() {
    if (!m_vsys)
        return;
    if (!m_vsys->Run())
        MPI_Abort(MPI_COMM_WORLD, 1);

    if (m_track) {
        ChVector<> cam_point;
        if (auto proxy_b = std::dynamic_pointer_cast<ProxyBodySet>(m_proxies[0])) {
            cam_point = proxy_b->bodies[0]->GetPos();
        } else if (auto proxy_m = std::dynamic_pointer_cast<ProxyMesh>(m_proxies[0])) {
            cam_point = proxy_m->ind2ptr_map[0]->GetPos();
        }

        m_vsys->UpdateCamera(m_cam_pos, cam_point);
    }

    m_vsys->BeginScene();
    m_vsys->Render();
    m_vsys->EndScene();
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
