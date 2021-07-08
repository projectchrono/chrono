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

#include <mpi.h>

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono_vehicle/cosim/ChVehicleCosimTerrainNodeRigid.h"

#ifdef CHRONO_OPENGL
    #include "chrono_opengl/ChOpenGLWindow.h"
#endif

using std::cout;
using std::endl;

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Construction of the terrain node:
// - create the Chrono system and set solver parameters
// - create the OpenGL visualization window
// -----------------------------------------------------------------------------
ChVehicleCosimTerrainNodeRigid::ChVehicleCosimTerrainNodeRigid(ChContactMethod method)
    : ChVehicleCosimTerrainNode(Type::RIGID, method), m_radius_p(0.01) {
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
}

ChVehicleCosimTerrainNodeRigid::ChVehicleCosimTerrainNodeRigid(ChContactMethod method, const std::string& specfile)
    : ChVehicleCosimTerrainNode(Type::RIGID, method) {
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
}

ChVehicleCosimTerrainNodeRigid::~ChVehicleCosimTerrainNodeRigid() {
    delete m_system;
}

// -----------------------------------------------------------------------------

//// TODO: error checking
void ChVehicleCosimTerrainNodeRigid::SetFromSpecfile(const std::string& specfile) {
    Document d;
    ReadSpecfile(specfile, d);

    double length = d["Patch dimensions"]["Length"].GetDouble();
    double width = d["Patch dimensions"]["Width"].GetDouble();
    SetPatchDimensions(length, width);

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
// - create the container body
// - if specified, create the granular material
// -----------------------------------------------------------------------------
void ChVehicleCosimTerrainNodeRigid::Construct() {
    if (m_verbose)
        cout << "[Terrain node] RIGID "
             << " method = " << static_cast<std::underlying_type<ChContactMethod>::type>(m_method) << endl;

    // Create container body
    auto container = std::shared_ptr<ChBody>(m_system->NewBody());
    m_system->AddBody(container);
    container->SetIdentifier(-1);
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
        ground->SetIdentifier(-2);
        ground->SetBodyFixed(true);
        ground->SetCollide(false);
        m_system->AddBody(ground);

        auto weld = chrono_types::make_shared<ChLinkLockLock>();
        weld->Initialize(ground, container, ChCoordsys<>(VNULL, QUNIT));
        m_system->AddLink(weld);
    }

#ifdef CHRONO_OPENGL
    // Create the visualization window
    if (m_render) {
        opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
        gl_window.Initialize(1280, 720, "Terrain Node (Rigid)", m_system);
        gl_window.SetCamera(ChVector<>(0, -2, 1), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), 0.05f);
        gl_window.SetRenderMode(opengl::SOLID);
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
void ChVehicleCosimTerrainNodeRigid::CreateMeshProxies() {
    double mass_p = m_rig_mass / m_mesh_data.nv;
    ChVector<> inertia_p = 0.4 * mass_p * m_radius_p * m_radius_p * ChVector<>(1, 1, 1);

    for (unsigned int iv = 0; iv < m_mesh_data.nv; iv++) {
        auto body = std::shared_ptr<ChBody>(m_system->NewBody());
        body->SetIdentifier(iv);
        body->SetMass(mass_p);
        body->SetInertiaXX(inertia_p);
        body->SetBodyFixed(m_fixed_proxies);
        body->SetCollide(true);

        body->GetCollisionModel()->ClearModel();
        utils::AddSphereGeometry(body.get(), m_material_tire, m_radius_p, ChVector<>(0, 0, 0),
                                 ChQuaternion<>(1, 0, 0, 0), true);
        body->GetCollisionModel()->SetFamily(1);
        body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
        body->GetCollisionModel()->BuildModel();

        m_system->AddBody(body);
        m_proxies.push_back(ProxyBody(body, iv));
    }
}

void ChVehicleCosimTerrainNodeRigid::CreateWheelProxy() {
    // Create wheel proxy body
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

// Set position and velocity of proxy bodies based on tire mesh vertices.
// Set orientation to identity and angular velocity to zero.
void ChVehicleCosimTerrainNodeRigid::UpdateMeshProxies() {
    for (unsigned int iv = 0; iv < m_mesh_data.nv; iv++) {
        m_proxies[iv].m_body->SetPos(m_mesh_state.vpos[iv]);
        m_proxies[iv].m_body->SetPos_dt(m_mesh_state.vvel[iv]);
        m_proxies[iv].m_body->SetRot(ChQuaternion<>(1, 0, 0, 0));
        m_proxies[iv].m_body->SetRot_dt(ChQuaternion<>(0, 0, 0, 0));
    }
}

// Set state of wheel proxy body.
void ChVehicleCosimTerrainNodeRigid::UpdateWheelProxy() {
    m_proxies[0].m_body->SetPos(m_wheel_state.pos);
    m_proxies[0].m_body->SetPos_dt(m_wheel_state.lin_vel);
    m_proxies[0].m_body->SetRot(m_wheel_state.rot);
    m_proxies[0].m_body->SetWvel_par(m_wheel_state.ang_vel);
}

// Collect contact forces on the (node) proxy bodies that are in contact.
// Load mesh vertex forces and corresponding indices.
void ChVehicleCosimTerrainNodeRigid::GetForcesMeshProxies() {
    m_mesh_contact.nv = 0;
    for (unsigned int iv = 0; iv < m_mesh_data.nv; iv++) {
        ChVector<> force = m_proxies[iv].m_body->GetContactForce();
        if (force.Length() > 1e-15) {
            m_mesh_contact.vforce.push_back(force);
            m_mesh_contact.vidx.push_back(m_proxies[iv].m_index);
            m_mesh_contact.nv++;
        }
    }
}

// Collect resultant contact force and torque on wheel proxy body.
void ChVehicleCosimTerrainNodeRigid::GetForceWheelProxy() {
    m_wheel_contact.point = ChVector<>(0, 0, 0);
    m_wheel_contact.force = m_proxies[0].m_body->GetContactForce();
    m_wheel_contact.moment = m_proxies[0].m_body->GetContactTorque();
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeRigid::OnAdvance(double step_size) {
    // Force a calculation of cumulative contact forces for all bodies in the system
    // (needed at the next synchronization)
    m_system->CalculateContactForces();
}

void ChVehicleCosimTerrainNodeRigid::OnRender(double time) {
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

void ChVehicleCosimTerrainNodeRigid::PrintMeshProxiesUpdateData() {
    auto lowest = std::min_element(m_proxies.begin(), m_proxies.end(), [](const ProxyBody& a, const ProxyBody& b) {
        return a.m_body->GetPos().z() < b.m_body->GetPos().z();
    });
    const ChVector<>& vel = (*lowest).m_body->GetPos_dt();
    double height = (*lowest).m_body->GetPos().z();
    cout << "[Terrain node] lowest proxy:  index = " << (*lowest).m_index << "  height = " << height
         << "  velocity = " << vel.x() << "  " << vel.y() << "  " << vel.z() << endl;
}

void ChVehicleCosimTerrainNodeRigid::PrintWheelProxyUpdateData() {
    //// TODO
}

void ChVehicleCosimTerrainNodeRigid::PrintMeshProxiesContactData() {
    // Information on all contacts.
    // Note that proxy body identifiers match the index of the associated mesh vertex.
    auto bodies = m_system->Get_bodylist();
    auto dm = m_system->data_manager;
    auto& bids = dm->cd_data->bids_rigid_rigid;
    ////auto& cpta = dm->host_data.cpta_rigid_rigid;
    ////auto& cptb = dm->host_data.cptb_rigid_rigid;
    auto& dpth = dm->cd_data->dpth_rigid_rigid;
    auto& norm = dm->cd_data->norm_rigid_rigid;
    std::set<int> vertices_in_contact;
    cout << "[Terrain node] contact information (" << dm->cd_data->num_rigid_contacts << ")" << endl;
    for (uint ic = 0; ic < dm->cd_data->num_rigid_contacts; ic++) {
        int idA = bids[ic].x;
        int idB = bids[ic].y;
        int indexA = bodies[idA]->GetIdentifier();
        int indexB = bodies[idB]->GetIdentifier();
        if (indexA > 0)
            vertices_in_contact.insert(indexA);
        if (indexB > 0)
            vertices_in_contact.insert(indexB);

        cout << "  id1 = " << indexA << "  id2 = " << indexB << "   dpth = " << dpth[ic] << "  normal = " << norm[ic].x
             << "  " << norm[ic].y << "  " << norm[ic].z << endl;
    }

    // Cumulative contact forces on proxy bodies.
    m_system->CalculateContactForces();
    cout << "[Terrain node] vertex forces (" << vertices_in_contact.size() << ")" << endl;
    for (unsigned int iv = 0; iv < m_mesh_data.nv; iv++) {
        if (vertices_in_contact.find(iv) != vertices_in_contact.end()) {
            real3 force = m_system->GetBodyContactForce(m_proxies[iv].m_body);
            cout << "  id = " << m_proxies[iv].m_index << "  force = " << force.x << "  " << force.y << "  " << force.z
                 << endl;
        }
    }

    ////auto container = std::static_pointer_cast<ChContactContainerMulticore>(m_system->GetContactContainer());
    ////auto contacts = container->GetContactList();

    ////for (auto it = contacts.begin(); it != contacts.end(); ++it) {
    ////    ChBody* bodyA = static_cast<ChBody*>((*it)->GetObjA());
    ////    ChBody* bodyB = static_cast<ChBody*>((*it)->GetObjA());

    ////    cout << " id1 = " << bodyA->GetIdentifier() << "  id2 = " << bodyB->GetIdentifier() << endl;
    ////}
}

void ChVehicleCosimTerrainNodeRigid::PrintWheelProxyContactData() {
    //// RADU TODO: implement this
}

}  // end namespace vehicle
}  // end namespace chrono
