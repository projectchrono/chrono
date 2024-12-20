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
// Authors: Wei Hu, Radu Serban
// =============================================================================
//
// Definition of the SPH granular TERRAIN NODE (using Chrono::FSI).
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <algorithm>
#include <cmath>
#include <set>
#include <limits>

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/cosim/terrain/ChVehicleCosimTerrainNodeGranularSPH.h"

#include "chrono_fsi/sph/utils/ChUtilsPrintSph.cuh"

#ifdef CHRONO_OPENGL
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationGL.h"
#endif
#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationVSG.h"
#endif

using std::cout;
using std::endl;

using namespace chrono::fsi;
using namespace rapidjson;

namespace chrono {
namespace vehicle {

// All obstacle bodies have this tag
static constexpr int tag_obstacles = 100;

// -----------------------------------------------------------------------------
// Construction of the terrain node:
// - create the Chrono system and set solver parameters
// - create the Chrono FSI system
// -----------------------------------------------------------------------------
ChVehicleCosimTerrainNodeGranularSPH::ChVehicleCosimTerrainNodeGranularSPH(double length, double width)
    : ChVehicleCosimTerrainNodeChrono(Type::GRANULAR_SPH, length, width, ChContactMethod::SMC),
      m_terrain(nullptr),
      m_depth(0),
      m_active_box_size(0),
      m_show_geometry(true),
      m_show_bce(true) {
    // Create the system and set default method-specific solver settings
    m_system = new ChSystemSMC;

    // Solver settings independent of method type
    m_system->SetGravitationalAcceleration(ChVector3d(0, 0, m_gacc));

    // Set number of threads
    m_system->SetNumThreads(1);

    // Default granular material properties
    m_radius = 0.01;
    m_density = 2000;
    m_cohesion = 0;
}

ChVehicleCosimTerrainNodeGranularSPH::ChVehicleCosimTerrainNodeGranularSPH(const std::string& specfile)
    : ChVehicleCosimTerrainNodeChrono(Type::GRANULAR_SPH, 0, 0, ChContactMethod::SMC),
      m_terrain(nullptr),
      m_active_box_size(0),
      m_show_geometry(true),
      m_show_bce(true) {
    // Create systems
    m_system = new ChSystemSMC;

    // Solver settings independent of method type
    m_system->SetGravitationalAcceleration(ChVector3d(0, 0, m_gacc));

    // Set number of threads
    m_system->SetNumThreads(1);

    // Read SPH granular terrain parameters from provided specfile
    SetFromSpecfile(specfile);
}

ChVehicleCosimTerrainNodeGranularSPH::~ChVehicleCosimTerrainNodeGranularSPH() {
    delete m_terrain;
    delete m_system;
}

// -----------------------------------------------------------------------------

//// TODO: error checking
void ChVehicleCosimTerrainNodeGranularSPH::SetFromSpecfile(const std::string& specfile) {
    Document d;
    ReadSpecfile(specfile, d);

    if (d.HasMember("Patch data files")) {
        auto sph_filename = d["Patch data files"]["SPH particles file"].GetString();
        auto bce_filename = d["Patch data files"]["BCE markers file"].GetString();
        m_sph_filename = vehicle::GetDataFile(sph_filename);
        m_bce_filename = vehicle::GetDataFile(bce_filename);
        m_depth = 0;
        m_terrain_type = ConstructionMethod::FILES;
    } else {
        assert(d.HasMember("Patch dimensions"));
        m_dimX = d["Patch dimensions"]["Length"].GetDouble();
        m_dimY = d["Patch dimensions"]["Width"].GetDouble();
        m_depth = d["Patch dimensions"]["Depth"].GetDouble();
        m_terrain_type = ConstructionMethod::PATCH;
    }

    m_radius = d["Granular material"]["Radius"].GetDouble();
    m_density = d["Granular material"]["Density"].GetDouble();
    m_cohesion = d["Granular material"]["Cohesion"].GetDouble();
    m_init_height = m_depth;

    // Cache the name of the specfile (used when the CRMTerrain is actually constructed)
    m_specfile = specfile;
}

void ChVehicleCosimTerrainNodeGranularSPH::SetGranularMaterial(double radius, double density, double cohesion) {
    m_radius = radius;
    m_density = density;
    m_cohesion = cohesion;
}

void ChVehicleCosimTerrainNodeGranularSPH::SetPropertiesSPH(const std::string& specfile, double depth) {
    m_depth = depth;
    m_init_height = m_depth;

    // Cache the name of the specfile (used when the CRMTerrain is actually constructed)
    m_specfile = specfile;
}

void ChVehicleCosimTerrainNodeGranularSPH::SetSolidVisualization(bool show_geometry, bool show_bce) {
    m_show_geometry = show_geometry;
    m_show_bce = show_bce;
}

// -----------------------------------------------------------------------------
// Complete construction of the mechanical system.
// This function is invoked automatically from OnInitialize.
// - adjust system settings
// - create the container body
// - add fluid particles
// - create obstacle bodies (if any)
// -----------------------------------------------------------------------------
void ChVehicleCosimTerrainNodeGranularSPH::Construct() {
    if (m_verbose)
        cout << "[Terrain node] GRANULAR_SPH " << endl;

    // Create the SPH terrain
    double initSpace0 = 2 * m_radius;
    m_terrain = new CRMTerrain(*m_system, initSpace0);
    //////m_terrain->SetVerbose(true);
    ChFsiSystemSPH& sysFSI = m_terrain->GetSystemFSI();
    ChFluidSystemSPH& sysSPH = m_terrain->GetFluidSystemSPH();

    // Let the FSI system read its parameters
    if (!m_specfile.empty())
        sysSPH.ReadParametersFromFile(m_specfile);

    // Reload simulation parameters to FSI system
    sysFSI.SetStepSizeCFD(m_step_size);
    sysFSI.SetStepsizeMBD(m_step_size);

    sysSPH.SetSPHMethod(SPHMethod::WCSPH);
    sysSPH.SetConsistentDerivativeDiscretization(false, false);
    sysSPH.SetOutputLevel(OutputLevel::STATE);
    sysSPH.SetGravitationalAcceleration(ChVector3d(0, 0, m_gacc));
    sysSPH.SetDensity(m_density);
    sysSPH.SetCohesionForce(m_cohesion);

    // Construct the CRMTerrain (generate SPH boundary BCE points)
    switch (m_terrain_type) {
        case ConstructionMethod::PATCH:
            m_terrain->Construct({m_dimX, m_dimY, m_depth}, VNULL, BoxSide::ALL & ~BoxSide::Z_POS);
            break;
        case ConstructionMethod::FILES:
            m_terrain->Construct(m_sph_filename, m_bce_filename, VNULL);
            break;
    }

    // Add all rigid obstacles
    for (auto& b : m_obstacles) {
        // Estimate obstacle inertial properties
        auto trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(GetChronoDataFile(b.m_mesh_filename), true, true);
        double mass;
        ChVector3d baricenter;
        ChMatrix33<> inertia;
        trimesh->ComputeMassProperties(true, mass, baricenter, inertia);

        // Create obstacle body
        auto body = chrono_types::make_shared<ChBody>();
        body->SetName("obstacle");
        body->SetTag(tag_obstacles);
        body->SetPos(b.m_init_pos);
        body->SetRot(b.m_init_rot);
        body->SetMass(mass * b.m_density);
        body->SetInertia(inertia * b.m_density);
        body->SetFixed(false);
        body->EnableCollision(true);

        // Set obstacle geometry
        double thickness = 0.01;
        utils::ChBodyGeometry geometry;
        geometry.materials.push_back(b.m_contact_mat);
        geometry.coll_meshes.push_back(
            utils::ChBodyGeometry::TrimeshShape(VNULL, GetChronoDataFile(b.m_mesh_filename), VNULL, thickness));

        // Create visualization and collision shapes
        geometry.CreateVisualizationAssets(body, VisualizationType::COLLISION);
        geometry.CreateCollisionShapes(body, 2, m_method);

        // Add the obstacle body to the underlying Chrono and FSI systems (obstacles may be embedded)
        m_system->AddBody(body);
        m_terrain->AddRigidBody(body, geometry, true);
    }

    // Write file with terrain node settings
    std::ofstream outf;
    outf.open(m_node_out_dir + "/settings.info", std::ios::out);
    outf << "System settings" << endl;
    outf << "   Integration step size = " << m_step_size << endl;
    outf << "Patch dimensions" << endl;
    outf << "   X = " << m_dimX << "  Y = " << m_dimY << endl;
    outf << "   depth = " << m_depth << endl;
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeGranularSPH::CreateRigidProxy(unsigned int i) {
    // Get shape associated with the given object
    int i_shape = m_obj_map[i];

    // Create the proxy associated with the given object
    auto proxy = chrono_types::make_shared<ProxyBodySet>();

    // Create proxy body
    auto body = chrono_types::make_shared<ChBody>();
    body->SetTag(0);
    body->SetPos(m_init_loc[i]);
    body->SetMass(m_load_mass[i]);
    body->SetFixed(true);  // proxy body always fixed

    // Create visualization asset (use collision shapes)
    if (m_show_geometry) {
        m_geometry[i_shape].CreateVisualizationAssets(body, VisualizationType::COLLISION);
    }

    // Create collision shapes (only if obstacles are present)
    auto num_obstacles = m_obstacles.size();
    if (num_obstacles > 0) {
        for (auto& mesh : m_geometry[i_shape].coll_meshes)
            mesh.radius = m_radius;
        m_geometry[i_shape].CreateCollisionShapes(body, 1, m_method);
        body->EnableCollision(true);
        body->GetCollisionModel()->SetFamily(1);
        body->GetCollisionModel()->DisallowCollisionsWith(1);
    } else {
        body->EnableCollision(false);
    }

    // Add this body to the underlying Chrono and FSI systems
    m_system->AddBody(body);
    m_terrain->AddRigidBody(body, m_geometry[i_shape], false);

    proxy->AddBody(body, 0);
    m_proxies[i] = proxy;

    // Update dimension of FSI active domain based on shape AABB
    m_active_box_size = std::max(m_active_box_size, m_aabb[i_shape].Size().Length());
}

// Set state of proxy rigid body.
void ChVehicleCosimTerrainNodeGranularSPH::UpdateRigidProxy(unsigned int i, BodyState& rigid_state) {
    auto proxy = std::static_pointer_cast<ProxyBodySet>(m_proxies[i]);
    proxy->bodies[0]->SetPos(rigid_state.pos);
    proxy->bodies[0]->SetPosDt(rigid_state.lin_vel);
    proxy->bodies[0]->SetRot(rigid_state.rot);
    proxy->bodies[0]->SetAngVelParent(rigid_state.ang_vel);
    proxy->bodies[0]->SetAngAccParent(VNULL);
}

// Collect resultant contact force and torque on rigid proxy body.
void ChVehicleCosimTerrainNodeGranularSPH::GetForceRigidProxy(unsigned int i, TerrainForce& rigid_contact) {
    auto proxy = std::static_pointer_cast<ProxyBodySet>(m_proxies[i]);
    rigid_contact.point = ChVector3d(0, 0, 0);
    rigid_contact.force = proxy->bodies[0]->GetAccumulatedForce();
    rigid_contact.moment = proxy->bodies[0]->GetAccumulatedTorque();
}

// -----------------------------------------------------------------------------
// Create bodies with triangular contact geometry as proxies for the mesh faces.
// Used for flexible bodies.
// Assign to each body an identifier equal to the index of its corresponding mesh face.
// Maintain a list of all bodies associated with the object.
// Add all proxy bodies to the same collision family and disable collision between any
// two members of this family.
void ChVehicleCosimTerrainNodeGranularSPH::CreateMeshProxy(unsigned int i) {
    if (m_verbose) {
        cout << "[Terrain node] Create mesh proxy " << i << endl;
    }

    // Get shape associated with the given object
    int i_shape = m_obj_map[i];

    // Create the proxy associated with the given object
    auto proxy = chrono_types::make_shared<ProxyMesh>();

    // Note: it is assumed that there is one and only one mesh defined!
    const auto& trimesh_shape = m_geometry[i_shape].coll_meshes[0];
    const auto& trimesh = trimesh_shape.trimesh;
    auto material = m_geometry[i_shape].materials[trimesh_shape.matID].CreateMaterial(m_method);

    // Create a contact surface mesh constructed from the provided trimesh
    auto surface = chrono_types::make_shared<fea::ChContactSurfaceMesh>(material);
    surface->ConstructFromTrimesh(trimesh, m_radius);

    // Create maps from pointer-based to index-based for the nodes in the mesh contact surface.
    // Note that here, the contact surface includes all faces in the geometry trimesh..
    int vertex_index = 0;
    for (const auto& tri : surface->GetTrianglesXYZ()) {
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

    if (m_show_geometry) {
        auto vis_mesh = chrono_types::make_shared<ChVisualShapeFEA>();
        vis_mesh->SetFEMdataType(ChVisualShapeFEA::DataType::CONTACTSURFACES);
        vis_mesh->SetWireframe(true);
        proxy->mesh->AddVisualShapeFEA(vis_mesh);
    }

    // Add mesh to MBS and FSI systems
    m_system->AddMesh(proxy->mesh);
    m_terrain->AddFeaMesh(proxy->mesh, false);
    
    m_proxies[i] = proxy;
}

// Set position, orientation, and velocity of proxy bodies based on mesh faces.
void ChVehicleCosimTerrainNodeGranularSPH::UpdateMeshProxy(unsigned int i, MeshState& mesh_state) {
    // Get the proxy (contact surface) associated with this object
    auto proxy = std::static_pointer_cast<ProxyMesh>(m_proxies[i]);

    int num_vertices = (int)mesh_state.vpos.size();

    for (int in = 0; in < num_vertices; in++) {
        auto& node = proxy->ind2ptr_map[in];
        node->SetPos(mesh_state.vpos[in]);
        node->SetPosDt(mesh_state.vvel[in]);
    }
}

// Collect forces on the nodes
void ChVehicleCosimTerrainNodeGranularSPH::GetForceMeshProxy(unsigned int i, MeshContact& mesh_contact) {
    static constexpr double zero_force_length2 = 1e-10;
    auto proxy = std::static_pointer_cast<ProxyMesh>(m_proxies[i]);

    ChVector3d force;
    mesh_contact.nv = 0;
    for (const auto& node : proxy->ptr2ind_map) {
        const auto& f = node.first->GetForce();
        if (f.Length2() > zero_force_length2) {
            mesh_contact.vidx.push_back(node.second);
            mesh_contact.vforce.push_back(node.first->GetForce());
            mesh_contact.nv++;
        }
    }
    ////if (i == 0 && mesh_contact.nv > 0) {
    ////    cout << " num contact nodes: " << mesh_contact.nv << endl;
    ////}
}

// -----------------------------------------------------------------------------

// Once all proxy bodies are created, complete construction of the underlying FSI system.
void ChVehicleCosimTerrainNodeGranularSPH::OnInitialize(unsigned int num_objects) {
    ChVehicleCosimTerrainNodeChrono::OnInitialize(num_objects);

    // Initialize the SPH terrain
    m_terrain->Initialize();

    // Initialize run-time visualization
    if (m_renderRT) {
        ChFsiSystemSPH& sysFSI = m_terrain->GetSystemFSI();

#if defined(CHRONO_VSG)
        auto vsys_VSG = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);
        vsys_VSG->SetClearColor(ChColor(0.455f, 0.525f, 0.640f));
        m_vsys = vsys_VSG;
#elif defined(CHRONO_OPENGL)
        m_vsys = chrono_types::make_shared<ChFsiVisualizationGL>(&sysFSI);
#endif

        if (m_vsys) {
            const auto& aabb_particles = m_terrain->GetSPHBoundingBox();

            m_vsys->SetTitle("Terrain Node (GranularSPH)");
            m_vsys->SetVerbose(false);
            m_vsys->SetSize(1280, 720);
            m_vsys->AddCamera(m_cam_pos, ChVector3d(0, 0, 0));
            m_vsys->SetCameraMoveScale(0.2f);
            m_vsys->EnableFluidMarkers(true);
            m_vsys->EnableBoundaryMarkers(false);
            m_vsys->EnableRigidBodyMarkers(m_show_bce);
            m_vsys->EnableFlexBodyMarkers(m_show_bce);
            m_vsys->SetRenderMode(ChFsiVisualization::RenderMode::SOLID);
            m_vsys->SetParticleRenderMode(ChFsiVisualization::RenderMode::SOLID);
            m_vsys->SetSPHColorCallback(chrono_types::make_shared<ParticleHeightColorCallback>(
                ChColor(0.10f, 0.40f, 0.65f), aabb_particles.min.z(), aabb_particles.max.z()));
            m_vsys->SetImageOutputDirectory(m_node_out_dir + "/images");
            m_vsys->SetImageOutput(m_writeRT);
            m_vsys->AttachSystem(m_system);
            m_vsys->Initialize();
        }
    }
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeGranularSPH::OnAdvance(double step_size) {
    double t = 0;
    while (t < step_size) {
        double h = std::min<>(m_step_size, step_size - t);
        m_terrain->GetSystemFSI().DoStepDynamics(h);
        t += h;
    }
}

void ChVehicleCosimTerrainNodeGranularSPH::OnRender() {
    if (!m_vsys)
        return;

    if (m_track)
        m_vsys->UpdateCamera(m_cam_pos, m_chassis_loc);

    auto ok = m_vsys->Render();
    if (!ok)
        MPI_Abort(MPI_COMM_WORLD, 1);
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeGranularSPH::OnOutputData(int frame) {
    // Save SPH and BCE particles' information into CSV files
    m_terrain->GetSystemFSI().GetFluidSystemSPH().SaveParticleData(m_node_out_dir + "/simulation");
}

void ChVehicleCosimTerrainNodeGranularSPH::OutputVisualizationData(int frame) {
    auto filename = OutputFilename(m_node_out_dir + "/visualization", "vis", "chpf", frame, 5);
    m_terrain->GetSystemFSI().GetFluidSystemSPH().WriteParticleFile(filename, OutputMode::CHPF);
    if (m_obstacles.size() > 0) {
        filename = OutputFilename(m_node_out_dir + "/visualization", "vis", "dat", frame, 5);
        // Include only obstacle bodies
        utils::WriteVisualizationAssets(
            m_system, filename, [](const ChBody& b) -> bool { return b.GetTag() == tag_obstacles; }, true);
    }
}

void ChVehicleCosimTerrainNodeGranularSPH::PrintMeshProxiesUpdateData(unsigned int i, const MeshState& mesh_state) {}

}  // end namespace vehicle
}  // end namespace chrono
