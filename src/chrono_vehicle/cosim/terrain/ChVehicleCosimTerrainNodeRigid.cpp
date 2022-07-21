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
// Definition of the rigid TERRAIN NODE (using Chrono::Multicore).
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

#include "chrono_vehicle/cosim/terrain/ChVehicleCosimTerrainNodeRigid.h"

#include "chrono_thirdparty/filesystem/path.h"

using std::cout;
using std::endl;

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// Ensure that all bodies other than the rigid terrain and obstacles are created with a smaller identifier.
// This allows filtering terrain+obstacle bodies.
static const int body_id_terrain = 100000;
static const int body_id_obstacles = 100001;

// -----------------------------------------------------------------------------
// Construction of the terrain node:
// - create the Chrono system and set solver parameters
// - create the OpenGL visualization window
// -----------------------------------------------------------------------------
ChVehicleCosimTerrainNodeRigid::ChVehicleCosimTerrainNodeRigid(double length,
                                                               double width,
                                                               ChContactMethod method)
    : ChVehicleCosimTerrainNodeChrono(Type::RIGID, length, width, method), m_radius_p(0.01) {
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

    // Set number of threads
    m_system->SetNumThreads(1);

    // Create OpenGL visualization system
#ifdef CHRONO_OPENGL
    m_vsys = new opengl::ChVisualSystemOpenGL;
#endif
}

ChVehicleCosimTerrainNodeRigid::ChVehicleCosimTerrainNodeRigid(ChContactMethod method, const std::string& specfile)
    : ChVehicleCosimTerrainNodeChrono(Type::RIGID, 0, 0, method) {
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

    // Set number of threads
    m_system->SetNumThreads(1);

    // Read rigid terrain parameters from provided specfile
    SetFromSpecfile(specfile);

    // Create OpenGL visualization system
#ifdef CHRONO_OPENGL
    m_vsys = new opengl::ChVisualSystemOpenGL;
#endif
}

ChVehicleCosimTerrainNodeRigid::~ChVehicleCosimTerrainNodeRigid() {
    delete m_system;
#ifdef CHRONO_OPENGL
    delete m_vsys;
#endif
}

// -----------------------------------------------------------------------------

//// TODO: error checking
void ChVehicleCosimTerrainNodeRigid::SetFromSpecfile(const std::string& specfile) {
    Document d;
    ReadSpecfile(specfile, d);

    double length = d["Patch dimensions"]["Length"].GetDouble();
    double width = d["Patch dimensions"]["Width"].GetDouble();
    m_hdimX = length / 2;
    m_hdimY = width / 2;

    switch (GetSystem()->GetContactMethod()) {
        case ChContactMethod::SMC: {
            auto material = chrono_types::make_shared<ChMaterialSurfaceSMC>();
            material->SetFriction(d["Material properties"]["Coefficient of friction"].GetDouble());
            material->SetRestitution(d["Material properties"]["Coefficient of restitution"].GetDouble());
            material->SetYoungModulus(d["Material properties"]["Young modulus"].GetDouble());
            material->SetPoissonRatio(d["Material properties"]["Poisson ratio"].GetDouble());
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
            m_material_terrain = material;
            break;
        }
    }

    if (m_system->GetContactMethod() == ChContactMethod::SMC) {
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
    }

    m_radius_p = d["Simulation settings"]["Proxy contact radius"].GetDouble();
    m_fixed_proxies = d["Simulation settings"]["Fix proxies"].GetBool();
}

void ChVehicleCosimTerrainNodeRigid::UseMaterialProperties(bool flag) {
    assert(m_system->GetContactMethod() == ChContactMethod::SMC);
    m_system->GetSettings()->solver.use_material_properties = flag;
}

void ChVehicleCosimTerrainNodeRigid::SetContactForceModel(ChSystemSMC::ContactForceModel model) {
    assert(m_system->GetContactMethod() == ChContactMethod::SMC);
    m_system->GetSettings()->solver.contact_force_model = model;
}

void ChVehicleCosimTerrainNodeRigid::SetMaterialSurface(const std::shared_ptr<ChMaterialSurface>& mat) {
    assert(mat->GetContactMethod() == m_system->GetContactMethod());
    m_material_terrain = mat;
}

// -----------------------------------------------------------------------------
// Complete construction of the mechanical system.
// This function is invoked automatically from Initialize.
// - adjust system settings
// - create the container body so that the top surface is at m_init_height = 0
// - if specified, create the granular material
// -----------------------------------------------------------------------------
void ChVehicleCosimTerrainNodeRigid::Construct() {
    if (m_verbose)
        cout << "[Terrain node] RIGID "
             << " method = " << static_cast<std::underlying_type<ChContactMethod>::type>(m_method) << endl;

    // Create container body
    auto container = std::shared_ptr<ChBody>(m_system->NewBody());
    m_system->AddBody(container);
    container->SetIdentifier(body_id_terrain);
    container->SetNameString("container");
    container->SetMass(1);
    container->SetBodyFixed(true);
    container->SetCollide(true);

    container->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(container.get(), m_material_terrain, ChVector<>(m_hdimX, m_hdimY, 0.1),
                          ChVector<>(0, 0, -0.1), ChQuaternion<>(1, 0, 0, 0), true);
    container->GetCollisionModel()->BuildModel();

    // If using RIGID terrain, the contact will be between the container and proxy bodies.
    // Since collision between two bodies fixed to ground is ignored, if the proxy bodies
    // are fixed, we make the container a free body connected through a weld joint to ground.
    if (m_fixed_proxies) {
        container->SetBodyFixed(false);

        auto ground = std::shared_ptr<ChBody>(m_system->NewBody());
        ground->SetNameString("ground");
        ground->SetIdentifier(-2);
        ground->SetBodyFixed(true);
        ground->SetCollide(false);
        m_system->AddBody(ground);

        auto weld = chrono_types::make_shared<ChLinkLockLock>();
        weld->Initialize(ground, container, ChCoordsys<>(VNULL, QUNIT));
        m_system->AddLink(weld);
    }

    // Add all rigid obstacles
    int id = body_id_obstacles;
    for (auto& b : m_obstacles) {
        auto mat = b.m_contact_mat.CreateMaterial(m_system->GetContactMethod());
        auto trimesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(b.m_mesh_filename),
                                                                                  true, true);
        double mass;
        ChVector<> baricenter;
        ChMatrix33<> inertia;
        trimesh->ComputeMassProperties(true, mass, baricenter, inertia);

        auto body = std::shared_ptr<ChBody>(m_system->NewBody());
        body->SetNameString("obstacle");
        body->SetIdentifier(id++);
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
        trimesh_shape->SetName(filesystem::path(b.m_mesh_filename).stem());
        body->AddVisualShape(trimesh_shape, ChFrame<>());

        m_system->AddBody(body);
    }

#ifdef CHRONO_OPENGL
    // Create the visualization window
    if (m_render) {
        m_vsys->AttachSystem(m_system);
        m_vsys->SetWindowTitle("Terrain Node (Rigid)");
        m_vsys->SetWindowSize(1280, 720);
        m_vsys->SetRenderMode(opengl::SOLID);
        m_vsys->Initialize();
        m_vsys->SetCameraPosition(ChVector<>(0, -2, 1), ChVector<>(0, 0, 0));
        m_vsys->SetCameraProperties(0.05f);
        m_vsys->SetCameraVertical(CameraVerticalDir::Z);
    }
#endif

    // Write file with terrain node settings
    std::ofstream outf;
    outf.open(m_node_out_dir + "/settings.info", std::ios::out);
    outf << "System settings" << endl;
    outf << "   Integration step size = " << m_step_size << endl;
    outf << "   Contact method = " << (m_method == ChContactMethod::SMC ? "SMC" : "NSC") << endl;
    outf << "   Use material properties? " << (m_system->GetSettings()->solver.use_material_properties ? "YES" : "NO")
         << endl;
    outf << "   Collision envelope = " << m_system->GetSettings()->collision.collision_envelope << endl;
    outf << "Patch dimensions" << endl;
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
    outf << "Proxy body properties" << endl;
    outf << "   proxies fixed? " << (m_fixed_proxies ? "YES" : "NO") << endl;
    outf << "   proxy contact radius = " << m_radius_p << endl;
}

// Create bodies with spherical contact geometry as proxies for the tire mesh vertices.
// Used for flexible tires.
// Assign to each body an identifier equal to the index of its corresponding mesh vertex.
// Maintain a list of all bodies associated with the tire.
// Add all proxy bodies to the same collision family and disable collision between any
// two members of this family.
void ChVehicleCosimTerrainNodeRigid::CreateMeshProxies(unsigned int i) {
    double mass_p = m_load_mass[i] / m_mesh_data[i].nv;
    ChVector<> inertia_p = 0.4 * mass_p * m_radius_p * m_radius_p * ChVector<>(1, 1, 1);
    auto material_tire = m_mat_props[i].CreateMaterial(m_method);

    for (unsigned int iv = 0; iv < m_mesh_data[i].nv; iv++) {
        auto body = std::shared_ptr<ChBody>(m_system->NewBody());
        body->SetIdentifier(iv);
        body->SetMass(mass_p);
        body->SetInertiaXX(inertia_p);
        body->SetBodyFixed(m_fixed_proxies);
        body->SetCollide(true);

        body->GetCollisionModel()->ClearModel();
        utils::AddSphereGeometry(body.get(), material_tire, m_radius_p, ChVector<>(0, 0, 0),
                                 ChQuaternion<>(1, 0, 0, 0), true);
        body->GetCollisionModel()->SetFamily(1);
        body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
        body->GetCollisionModel()->BuildModel();

        m_system->AddBody(body);
        m_proxies[i].push_back(ProxyBody(body, iv));
    }
}

void ChVehicleCosimTerrainNodeRigid::CreateWheelProxy(unsigned int i) {
    auto material_tire = m_mat_props[i].CreateMaterial(m_method);

    // Create wheel proxy body
    auto body = std::shared_ptr<ChBody>(m_system->NewBody());
    body->SetNameString("proxy_" + std::to_string(i));
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
    body->GetCollisionModel()->AddTriangleMesh(material_tire, trimesh, false, false, ChVector<>(0), ChMatrix33<>(1),
                                               m_radius_p);
    body->GetCollisionModel()->SetFamily(1);
    body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
    body->GetCollisionModel()->BuildModel();

    // Set visualization asset
    auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName("wheel_" + std::to_string(i));
    body->AddVisualShape(trimesh_shape, ChFrame<>());

    m_system->AddBody(body);
    m_proxies[i].push_back(ProxyBody(body, 0));
}

// Set position and velocity of proxy bodies based on tire mesh vertices.
// Set orientation to identity and angular velocity to zero.
void ChVehicleCosimTerrainNodeRigid::UpdateMeshProxies(unsigned int i, MeshState& mesh_state) {
    auto& proxies = m_proxies[i];  // proxies for the i-th tire

    for (unsigned int iv = 0; iv < m_mesh_data[i].nv; iv++) {
        proxies[iv].m_body->SetPos(mesh_state.vpos[iv]);
        proxies[iv].m_body->SetPos_dt(mesh_state.vvel[iv]);
        proxies[iv].m_body->SetRot(ChQuaternion<>(1, 0, 0, 0));
        proxies[iv].m_body->SetRot_dt(ChQuaternion<>(0, 0, 0, 0));
    }

    PrintMeshProxiesUpdateData(i, mesh_state);
}

// Set state of wheel proxy body.
void ChVehicleCosimTerrainNodeRigid::UpdateWheelProxy(unsigned int i, BodyState& spindle_state) {
    auto& proxies = m_proxies[i];  // proxies for the i-th tire

    proxies[0].m_body->SetPos(spindle_state.pos);
    proxies[0].m_body->SetPos_dt(spindle_state.lin_vel);
    proxies[0].m_body->SetRot(spindle_state.rot);
    proxies[0].m_body->SetWvel_par(spindle_state.ang_vel);
}

// Collect contact forces on the (node) proxy bodies that are in contact.
// Load mesh vertex forces and corresponding indices.
void ChVehicleCosimTerrainNodeRigid::GetForcesMeshProxies(unsigned int i, MeshContact& mesh_contact) {
    const auto& proxies = m_proxies[i];  // proxies for the i-th tire

    mesh_contact.nv = 0;
    for (unsigned int iv = 0; iv < m_mesh_data[i].nv; iv++) {
        ChVector<> force = proxies[iv].m_body->GetContactForce();
        if (force.Length() > 1e-15) {
            mesh_contact.vforce.push_back(force);
            mesh_contact.vidx.push_back(proxies[iv].m_index);
            mesh_contact.nv++;
        }
    }
}

// Collect resultant contact force and torque on wheel proxy body.
void ChVehicleCosimTerrainNodeRigid::GetForceWheelProxy(unsigned int i, TerrainForce& wheel_contact) {
    const auto& proxies = m_proxies[i];  // proxies for the i-th tire

    wheel_contact.point = ChVector<>(0, 0, 0);
    wheel_contact.force = proxies[0].m_body->GetContactForce();
    wheel_contact.moment = proxies[0].m_body->GetContactTorque();
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeRigid::OnAdvance(double step_size) {
    ChVehicleCosimTerrainNodeChrono::OnAdvance(step_size);

    // Force a calculation of cumulative contact forces for all bodies in the system
    // (needed at the next synchronization)
    m_system->CalculateContactForces();
}

void ChVehicleCosimTerrainNodeRigid::Render(double time) {
#ifdef CHRONO_OPENGL
    if (m_vsys->Run()) {
        m_vsys->Render();
    } else {
        MPI_Abort(MPI_COMM_WORLD, 1);
    }
#endif
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeRigid::OutputVisualizationData(int frame) {
    auto filename = OutputFilename(m_node_out_dir + "/visualization", "vis", "dat", frame, 5);
    // Include only main body and obstacles
    utils::WriteVisualizationAssets(
        m_system, filename, [](const ChBody& b) -> bool { return b.GetIdentifier() >= body_id_terrain; }, true);
}

void ChVehicleCosimTerrainNodeRigid::PrintMeshProxiesUpdateData(unsigned int i, const MeshState& mesh_state) {
    auto lowest = std::min_element(
        m_proxies[i].begin(), m_proxies[i].end(),
        [](const ProxyBody& a, const ProxyBody& b) { return a.m_body->GetPos().z() < b.m_body->GetPos().z(); });
    const ChVector<>& vel = (*lowest).m_body->GetPos_dt();
    double height = (*lowest).m_body->GetPos().z();
    cout << "[Terrain node] tire: " << i << "  lowest proxy:  index = " << (*lowest).m_index << "  height = " << height
         << "  velocity = " << vel.x() << "  " << vel.y() << "  " << vel.z() << endl;
}

}  // end namespace vehicle
}  // end namespace chrono
