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

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono/assets/ChVisualShapeTriangleMesh.h"

#include "chrono_vehicle/cosim/terrain/ChVehicleCosimTerrainNodeRigid.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#endif
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
#endif
#ifdef CHRONO_OPENGL
    #include "chrono_opengl/ChVisualSystemOpenGL.h"
#endif

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
ChVehicleCosimTerrainNodeRigid::ChVehicleCosimTerrainNodeRigid(double length, double width, ChContactMethod method)
    : ChVehicleCosimTerrainNodeChrono(Type::RIGID, length, width, method), m_radius_p(0.01) {
    // Create system and set default method-specific solver settings
    switch (m_method) {
        case ChContactMethod::SMC: {
            ChSystemSMC* sys = new ChSystemSMC;
            m_system = sys;
            break;
        }
        case ChContactMethod::NSC: {
            ChSystemNSC* sys = new ChSystemNSC;
            m_system = sys;
            break;
        }
    }

    m_system->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    m_system->Set_G_acc(ChVector<>(0, 0, m_gacc));
    m_system->SetNumThreads(1);
}

ChVehicleCosimTerrainNodeRigid::ChVehicleCosimTerrainNodeRigid(const std::string& specfile, ChContactMethod method)
    : ChVehicleCosimTerrainNodeChrono(Type::RIGID, 0, 0, method) {
    // Create system and set default method-specific solver settings
    switch (m_method) {
        case ChContactMethod::SMC: {
            ChSystemSMC* sys = new ChSystemSMC;
            m_system = sys;
            break;
        }
        case ChContactMethod::NSC: {
            ChSystemNSC* sys = new ChSystemNSC;
            m_system = sys;
            break;
        }
    }

    m_system->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    m_system->Set_G_acc(ChVector<>(0, 0, m_gacc));
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

    m_dimX = d["Patch dimensions"]["Length"].GetDouble();
    m_dimY = d["Patch dimensions"]["Width"].GetDouble();

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
        auto sysSMC = static_cast<ChSystemSMC*>(m_system);
        std::string n_model = d["Simulation settings"]["Normal contact model"].GetString();
        if (n_model.compare("Hertz") == 0)
            sysSMC->SetContactForceModel(ChSystemSMC::ContactForceModel::Hertz);
        else if (n_model.compare("Hooke") == 0)
            sysSMC->SetContactForceModel(ChSystemSMC::ContactForceModel::Hooke);
        else if (n_model.compare("Flores") == 0)
            sysSMC->SetContactForceModel(ChSystemSMC::ContactForceModel::Flores);
        else if (n_model.compare("Hertz") == 0)
            sysSMC->SetContactForceModel(ChSystemSMC::ContactForceModel::PlainCoulomb);

        std::string t_model = d["Simulation settings"]["Tangential displacement model"].GetString();
        if (t_model.compare("MultiStep") == 0)
            sysSMC->SetTangentialDisplacementModel(ChSystemSMC::TangentialDisplacementModel::MultiStep);
        else if (t_model.compare("OneStep") == 0)
            sysSMC->SetTangentialDisplacementModel(ChSystemSMC::TangentialDisplacementModel::OneStep);
        else if (t_model.compare("None") == 0)
            sysSMC->SetTangentialDisplacementModel(ChSystemSMC::TangentialDisplacementModel::None);

        sysSMC->UseMaterialProperties(d["Simulation settings"]["Use material properties"].GetBool());
    }

    m_radius_p = d["Simulation settings"]["Proxy contact radius"].GetDouble();
    m_fixed_proxies = d["Simulation settings"]["Fix proxies"].GetBool();
}

void ChVehicleCosimTerrainNodeRigid::UseMaterialProperties(bool flag) {
    assert(m_system->GetContactMethod() == ChContactMethod::SMC);
    static_cast<ChSystemSMC*>(m_system)->UseMaterialProperties(flag);
}

void ChVehicleCosimTerrainNodeRigid::SetContactForceModel(ChSystemSMC::ContactForceModel model) {
    assert(m_system->GetContactMethod() == ChContactMethod::SMC);
    static_cast<ChSystemSMC*>(m_system)->SetContactForceModel(model);
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
    auto container = chrono_types::make_shared<ChBody>();
    m_system->AddBody(container);
    container->SetIdentifier(body_id_terrain);
    container->SetNameString("container");
    container->SetMass(1);
    container->SetBodyFixed(true);
    container->SetCollide(true);

    auto vis_mat = std::make_shared<ChVisualMaterial>(*ChVisualMaterial::Default());
    vis_mat->SetKdTexture(GetChronoDataFile("textures/checker2.png"));
    vis_mat->SetTextureScale(m_dimX, m_dimY);

    utils::AddBoxGeometry(container.get(), m_material_terrain, ChVector<>(m_dimX, m_dimY, 0.2), ChVector<>(0, 0, -0.1),
                          ChQuaternion<>(1, 0, 0, 0), true, vis_mat);

    // If using RIGID terrain, the contact will be between the container and proxy bodies.
    // Since collision between two bodies fixed to ground is ignored, if the proxy bodies
    // are fixed, we make the container a free body connected through a weld joint to ground.
    if (m_fixed_proxies) {
        container->SetBodyFixed(false);

        auto ground = chrono_types::make_shared<ChBody>();
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

        auto body = chrono_types::make_shared<ChBody>();
        body->SetNameString("obstacle");
        body->SetIdentifier(id++);
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
        trimesh_shape->SetName(filesystem::path(b.m_mesh_filename).stem());
        body->AddVisualShape(trimesh_shape, ChFrame<>());

        m_system->AddBody(body);
    }

    // Write file with terrain node settings
    std::ofstream outf;
    outf.open(m_node_out_dir + "/settings.info", std::ios::out);
    outf << "System settings" << endl;
    outf << "   Integration step size = " << m_step_size << endl;
    outf << "   Contact method = " << (m_method == ChContactMethod::SMC ? "SMC" : "NSC") << endl;
    outf << "Patch dimensions" << endl;
    outf << "   X = " << m_dimX << "  Y = " << m_dimY << endl;
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

// Create bodies with spherical contact geometry as proxies for the mesh vertices.
// Used for flexible bodies.
// Assign to each body an identifier equal to the index of its corresponding mesh vertex.
// Maintain a list of all bodies associated with the object.
// Add all proxy bodies to the same collision family and disable collision between any
// two members of this family.
void ChVehicleCosimTerrainNodeRigid::CreateMeshProxy(unsigned int i) {
    // Get shape associated with the given object
    int i_shape = m_obj_map[i];

    // Create the proxy associated with the given object
    auto proxy = chrono_types::make_shared<ProxyBodySet>();

    // Note: it is assumed that there is one and only one mesh defined!
    auto nv = m_geometry[i_shape].m_coll_meshes[0].m_trimesh->getNumVertices();
    auto i_mat = m_geometry[i_shape].m_coll_meshes[0].m_matID;
    auto material = m_geometry[i_shape].m_materials[i_mat].CreateMaterial(m_method);

    double mass_p = m_load_mass[i_shape] / nv;
    ChVector<> inertia_p = 0.4 * mass_p * m_radius_p * m_radius_p * ChVector<>(1, 1, 1);

    for (int iv = 0; iv < nv; iv++) {
        auto body = chrono_types::make_shared<ChBody>();
        body->SetIdentifier(iv);
        body->SetMass(mass_p);
        body->SetInertiaXX(inertia_p);
        body->SetBodyFixed(m_fixed_proxies);
        body->SetCollide(true);

        utils::AddSphereGeometry(body.get(), material, m_radius_p, ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0),
                                 true);
        body->GetCollisionModel()->SetFamily(1);
        body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);

        m_system->AddBody(body);
        m_system->GetCollisionSystem()->BindItem(body);

        proxy->AddBody(body, iv);
    }

    m_proxies[i] = proxy;
}

void ChVehicleCosimTerrainNodeRigid::CreateRigidProxy(unsigned int i) {
    // Get shape associated with the given object
    int i_shape = m_obj_map[i];

    // Create the proxy associated with the given object
    auto proxy = chrono_types::make_shared<ProxyBodySet>();

    // Create wheel proxy body
    auto body = chrono_types::make_shared<ChBody>();
    body->SetNameString("proxy_" + std::to_string(i));
    body->SetIdentifier(0);
    body->SetMass(m_load_mass[i]);
    ////body->SetInertiaXX();   //// TODO
    body->SetBodyFixed(m_fixed_proxies);
    body->SetCollide(true);

    // Create visualization assets (use collision shapes)
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
}

// Once all proxy bodies are created, complete construction of the underlying system.
void ChVehicleCosimTerrainNodeRigid::OnInitialize(unsigned int num_objects) {
    ChVehicleCosimTerrainNodeChrono::OnInitialize(num_objects);

    // Create the visualization window
    if (m_renderRT) {
#if defined(CHRONO_VSG)
        auto vsys_vsg = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        vsys_vsg->AttachSystem(m_system);
        vsys_vsg->SetWindowTitle("Terrain Node (Rigid)");
        vsys_vsg->SetWindowSize(ChVector2<int>(1280, 720));
        vsys_vsg->SetWindowPosition(ChVector2<int>(100, 100));
        vsys_vsg->SetUseSkyBox(false);
        vsys_vsg->SetClearColor(ChColor(0.455f, 0.525f, 0.640f));
        vsys_vsg->AddCamera(m_cam_pos, ChVector<>(0, 0, 0));
        vsys_vsg->SetCameraAngleDeg(40);
        vsys_vsg->SetLightIntensity(1.0f);
        vsys_vsg->SetImageOutputDirectory(m_node_out_dir + "/images");
        vsys_vsg->SetImageOutput(m_writeRT);
        vsys_vsg->Initialize();

        m_vsys = vsys_vsg;
#elif defined(CHRONO_IRRLICHT)
        auto vsys_irr = chrono_types::make_shared<irrlicht::ChVisualSystemIrrlicht>();
        vsys_irr->AttachSystem(m_system);
        vsys_irr->SetWindowTitle("Terrain Node (Rigid)");
        vsys_irr->SetCameraVertical(CameraVerticalDir::Z);
        vsys_irr->SetWindowSize(1280, 720);
        vsys_irr->Initialize();
        vsys_irr->AddLogo();
        vsys_irr->AddSkyBox();
        vsys_irr->AddTypicalLights();
        vsys_irr->AddCamera(m_cam_pos, ChVector<>(0, 0, 0));

        m_vsys = vsys_irr;
#elif defined(CHRONO_OPENGL)
        auto vsys_gl = chrono_types::make_shared<opengl::ChVisualSystemOpenGL>();
        vsys_gl->AttachSystem(m_system);
        vsys_gl->SetWindowTitle("Terrain Node (Rigid)");
        vsys_gl->SetWindowSize(1280, 720);
        vsys_gl->SetRenderMode(opengl::SOLID);
        vsys_gl->Initialize();
        vsys_gl->AddCamera(m_cam_pos, ChVector<>(0, 0, 0));
        vsys_gl->SetCameraProperties(0.05f);
        vsys_gl->SetCameraVertical(CameraVerticalDir::Z);

        m_vsys = vsys_gl;
#endif
    }
}

// Set position and velocity of proxy bodies based on mesh vertices.
// Set orientation to identity and angular velocity to zero.
void ChVehicleCosimTerrainNodeRigid::UpdateMeshProxy(unsigned int i, MeshState& mesh_state) {
    // Get the proxy (body set) associated with this object
    auto proxy = std::static_pointer_cast<ProxyBodySet>(m_proxies[i]);
    auto num_bodies = proxy->bodies.size();

    for (size_t iv = 0; iv < num_bodies; iv++) {
        proxy->bodies[iv]->SetPos(mesh_state.vpos[iv]);
        proxy->bodies[iv]->SetPos_dt(mesh_state.vvel[iv]);
        proxy->bodies[iv]->SetRot(ChQuaternion<>(1, 0, 0, 0));
        proxy->bodies[iv]->SetRot_dt(ChQuaternion<>(0, 0, 0, 0));
    }

    ////if (m_verbose)
    ////    PrintMeshProxiesUpdateData(i, mesh_state);
}

// Set state of wheel proxy body.
void ChVehicleCosimTerrainNodeRigid::UpdateRigidProxy(unsigned int i, BodyState& rigid_state) {
    auto proxy = std::static_pointer_cast<ProxyBodySet>(m_proxies[i]);
    proxy->bodies[0]->SetPos(rigid_state.pos);
    proxy->bodies[0]->SetPos_dt(rigid_state.lin_vel);
    proxy->bodies[0]->SetRot(rigid_state.rot);
    proxy->bodies[0]->SetWvel_par(rigid_state.ang_vel);
}

// Collect contact forces on the (node) proxy bodies that are in contact.
// Load mesh vertex forces and corresponding indices.
void ChVehicleCosimTerrainNodeRigid::GetForceMeshProxy(unsigned int i, MeshContact& mesh_contact) {
    // Get the proxy (body set) associated with this object
    auto proxy = std::static_pointer_cast<ProxyBodySet>(m_proxies[i]);
    auto num_bodies = proxy->bodies.size();

    mesh_contact.nv = 0;
    for (size_t iv = 0; iv < num_bodies; iv++) {
        ChVector<> force = proxy->bodies[iv]->GetContactForce();
        if (force.Length() > 1e-15) {
            mesh_contact.vforce.push_back(force);
            mesh_contact.vidx.push_back(proxy->indices[iv]);
            mesh_contact.nv++;
        }
    }
}

// Collect resultant contact force and torque on rigid proxy body.
void ChVehicleCosimTerrainNodeRigid::GetForceRigidProxy(unsigned int i, TerrainForce& rigid_contact) {
    auto proxy = std::static_pointer_cast<ProxyBodySet>(m_proxies[i]);
    rigid_contact.point = ChVector<>(0, 0, 0);
    rigid_contact.force = proxy->bodies[0]->GetContactForce();
    rigid_contact.moment = proxy->bodies[0]->GetContactTorque();
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeRigid::OnRender() {
    if (!m_vsys)
        return;
    if (!m_vsys->Run())
        MPI_Abort(MPI_COMM_WORLD, 1);

    if (m_track) {
        auto proxy = std::static_pointer_cast<ProxyBodySet>(m_proxies[0]);  // proxy for first object
        ChVector<> cam_point = proxy->bodies[0]->GetPos();                  // position of first body in proxy set
        m_vsys->UpdateCamera(m_cam_pos, cam_point);
    }

    m_vsys->BeginScene();
    m_vsys->Render();
    m_vsys->EndScene();
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeRigid::OutputVisualizationData(int frame) {
    auto filename = OutputFilename(m_node_out_dir + "/visualization", "vis", "dat", frame, 5);
    // Include only main body and obstacles
    utils::WriteVisualizationAssets(
        m_system, filename, [](const ChBody& b) -> bool { return b.GetIdentifier() >= body_id_terrain; }, true);
}

void ChVehicleCosimTerrainNodeRigid::PrintMeshProxiesUpdateData(unsigned int i, const MeshState& mesh_state) {
    auto proxy = std::static_pointer_cast<ProxyBodySet>(m_proxies[i]);

    auto lowest = std::min_element(
        proxy->bodies.begin(), proxy->bodies.end(),
        [](std::shared_ptr<ChBody> a, std::shared_ptr<ChBody> b) { return a->GetPos().z() < b->GetPos().z(); });
    double height = (*lowest)->GetPos().z();
    const ChVector<>& vel = (*lowest)->GetPos_dt();
    cout << "[Terrain node] object: " << i << "  lowest proxy:  height = " << height << "  velocity = " << vel << endl;
}

}  // end namespace vehicle
}  // end namespace chrono
