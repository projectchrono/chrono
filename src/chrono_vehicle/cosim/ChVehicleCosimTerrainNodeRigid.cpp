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
// Definition of the rigid TERRAIN NODE (using Chrono).
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <algorithm>
#include <cmath>
#include <set>

#include <mpi.h>

#include "chrono_vehicle/cosim/ChVehicleCosimTerrainNodeRigid.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

using std::cout;
using std::endl;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Construction of the terrain node:
// - create the Chrono system and set solver parameters
// - create the OpenGL visualization window
// -----------------------------------------------------------------------------
ChVehicleCosimTerrainNodeRigid::ChVehicleCosimTerrainNodeRigid(ChContactMethod method, bool render)
    : ChVehicleCosimTerrainNode(Type::RIGID, method, render) {
    cout << "[Terrain node] RIGID "
         << " method = " << static_cast<std::underlying_type<ChContactMethod>::type>(method) << endl;

    // ------------------------
    // Default model parameters
    // ------------------------

    // Default container dimensions
    m_hdimX = 1.0;
    m_hdimY = 0.25;
    m_hdimZ = 0.5;
    m_hthick = 0.1;

    // --------------------------
    // Create the parallel system
    // --------------------------

    // Create system and set default method-specific solver settings
    switch (m_method) {
        case ChContactMethod::SMC: {
            ChSystemSMC* sys = new ChSystemSMC;
            sys->SetContactForceModel(ChSystemSMC::Hertz);
            sys->SetTangentialDisplacementModel(ChSystemSMC::TangentialDisplacementModel::OneStep);
            sys->UseMaterialProperties(true);
            m_system = sys;
            break;
        }
        case ChContactMethod::NSC: {
            ChSystemNSC* sys = new ChSystemNSC;
            collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.001);
            sys->SetSolverType(ChSolver::Type::APGD);
            m_system = sys;
            break;
        }
    }

    // Solver settings independent of method type
    m_system->Set_G_acc(ChVector<>(0, 0, m_gacc));

    // Set number of threads
    m_system->SetNumThreads(1);

#ifdef CHRONO_OPENGL
    // -------------------------------
    // Create the visualization window
    // -------------------------------

    if (m_render) {
        opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
        gl_window.Initialize(1280, 720, "Terrain Node", m_system);
        gl_window.SetCamera(ChVector<>(0, -1, 0), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), 0.05f);
        gl_window.SetRenderMode(opengl::WIREFRAME);
    }
#endif
}

ChVehicleCosimTerrainNodeRigid::~ChVehicleCosimTerrainNodeRigid() {
    delete m_system;
}

// -----------------------------------------------------------------------------

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
// - create the container body
// - if specified, create the granular material
// -----------------------------------------------------------------------------
void ChVehicleCosimTerrainNodeRigid::Construct() {
    // ---------------------
    // Create container body
    // ---------------------

    auto container = std::shared_ptr<ChBody>(m_system->NewBody());
    m_system->AddBody(container);
    container->SetIdentifier(-1);
    container->SetMass(1);
    container->SetBodyFixed(true);
    container->SetCollide(true);

    container->GetCollisionModel()->ClearModel();
    // Bottom box
    utils::AddBoxGeometry(container.get(), m_material_terrain, ChVector<>(m_hdimX, m_hdimY, m_hthick),
                          ChVector<>(0, 0, -m_hthick), ChQuaternion<>(1, 0, 0, 0), true);

    // If using RIGID terrain, the contact will be between the container and proxy bodies.
    // Since collision between two bodies fixed to ground is ignored, if the proxy bodies
    // are fixed, we make the container a free body connected through a weld joint to ground.
    // If using GRANULAR terrain, this is not an issue as the proxy bodies do not interact
    // with the container, but rather with the granular material.
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

    // --------------------------------------
    // Write file with terrain node settings
    // --------------------------------------

    std::ofstream outf;
    outf.open(m_node_out_dir + "/settings.dat", std::ios::out);
    outf << "System settings" << endl;
    outf << "   Integration step size = " << m_step_size << endl;
    outf << "   Contact method = " << (m_method == ChContactMethod::SMC ? "SMC" : "NSC") << endl;
    m_system->GetCollisionSystem();
    outf << "   Collision envelope = " << collision::ChCollisionModel::GetDefaultSuggestedEnvelope() << endl;
    outf << "Container dimensions" << endl;
    outf << "   X = " << 2 * m_hdimX << "  Y = " << 2 * m_hdimY << "  Z = " << 2 * m_hdimZ << endl;
    outf << "   wall thickness = " << 2 * m_hthick << endl;
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
    outf << "   proxy radius = " << m_radius_p << endl;
    outf << "   proxy mass = " << m_mass_p << endl;
}

// Create bodies with spherical contact geometry as proxies for the tire mesh vertices.
// Assign to each body an identifier equal to the index of its corresponding mesh vertex.
// Maintain a list of all bodies associated with the tire.
// Add all proxy bodies to the same collision family and disable collision between any
// two members of this family.
void ChVehicleCosimTerrainNodeRigid::CreateProxies() {
    ChVector<> inertia_p = 0.4 * m_mass_p * m_radius_p * m_radius_p * ChVector<>(1, 1, 1);
    for (unsigned int iv = 0; iv < m_mesh_data.nv; iv++) {
        auto body = std::shared_ptr<ChBody>(m_system->NewBody());
        m_system->AddBody(body);
        body->SetIdentifier(iv);
        body->SetMass(m_mass_p);
        body->SetInertiaXX(inertia_p);
        body->SetBodyFixed(false);
        body->SetCollide(true);

        body->GetCollisionModel()->ClearModel();
        utils::AddSphereGeometry(body.get(), m_material_tire, m_radius_p, ChVector<>(0, 0, 0),
                                 ChQuaternion<>(1, 0, 0, 0), true);
        body->GetCollisionModel()->SetFamily(1);
        body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
        body->GetCollisionModel()->BuildModel();

        m_proxies.push_back(ProxyBody(body, iv));
    }
}

// Set position and velocity of proxy bodies based on tire mesh vertices.
// Set orientation to identity and angular velocity to zero.
void ChVehicleCosimTerrainNodeRigid::UpdateProxies() {
    for (unsigned int iv = 0; iv < m_mesh_data.nv; iv++) {
        m_proxies[iv].m_body->SetPos(m_mesh_state.vpos[iv]);
        m_proxies[iv].m_body->SetPos_dt(m_mesh_state.vvel[iv]);
        m_proxies[iv].m_body->SetRot(ChQuaternion<>(1, 0, 0, 0));
        m_proxies[iv].m_body->SetRot_dt(ChQuaternion<>(0, 0, 0, 0));
    }
}

// Collect contact forces on the (node) proxy bodies that are in contact.
// Load mesh vertex forces and corresponding indices.
void ChVehicleCosimTerrainNodeRigid::ForcesProxies(std::vector<double>& vert_forces, std::vector<int>& vert_indices) {
    for (unsigned int iv = 0; iv < m_mesh_data.nv; iv++) {
        ChVector<> force = m_proxies[iv].m_body->GetContactForce();
        if (force.Length() > 1e-15) {
            vert_forces.push_back(force.x());
            vert_forces.push_back(force.y());
            vert_forces.push_back(force.z());
            vert_indices.push_back(m_proxies[iv].m_index);
        }
    }
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeRigid::OnSynchronize(int step_number, double time) {
    // Calculate cumulative contact forces for all bodies in system.

    //// RADU TODO:  NSC systems?

    ////m_system->CalculateContactForces();
}

void ChVehicleCosimTerrainNodeRigid::OnAdvance(double step_size) {
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
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeRigid::OutputTerrainData(int frame) {
    // Nothing to do here
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChVehicleCosimTerrainNodeRigid::PrintProxiesContactData() {
    //// RADU TODO

    /*
    // Information on all contacts.
    // Note that proxy body identifiers match the index of the associated mesh vertex.
    auto bodies = m_system->Get_bodylist();
    auto dm = m_system->data_manager;
    auto& bids = dm->host_data.bids_rigid_rigid;
    auto& cpta = dm->host_data.cpta_rigid_rigid;
    auto& cptb = dm->host_data.cptb_rigid_rigid;
    auto& dpth = dm->host_data.dpth_rigid_rigid;
    auto& norm = dm->host_data.norm_rigid_rigid;
    std::set<int> vertices_in_contact;
    cout << "[Terrain node] contact information (" << dm->num_rigid_contacts << ")" << endl;
    for (uint ic = 0; ic < dm->num_rigid_contacts; ic++) {
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
    for (unsigned int iv = 0; iv < m_num_vert; iv++) {
        if (vertices_in_contact.find(iv) != vertices_in_contact.end()) {
            real3 force = m_system->GetBodyContactForce(m_proxies[iv].m_body);
            cout << "  id = " << m_proxies[iv].m_index << "  force = " << force.x << "  " << force.y << "  " << force.z
                 << endl;
        }
    }
    */

    ////auto container = std::static_pointer_cast<ChContactContainerParallel>(m_system->GetContactContainer());
    ////auto contacts = container->GetContactList();

    ////for (auto it = contacts.begin(); it != contacts.end(); ++it) {
    ////    ChBody* bodyA = static_cast<ChBody*>((*it)->GetObjA());
    ////    ChBody* bodyB = static_cast<ChBody*>((*it)->GetObjA());

    ////    cout << " id1 = " << bodyA->GetIdentifier() << "  id2 = " << bodyB->GetIdentifier() << endl;
    ////}
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChVehicleCosimTerrainNodeRigid::PrintProxiesUpdateData() {
    auto lowest = std::min_element(m_proxies.begin(), m_proxies.end(), [](const ProxyBody& a, const ProxyBody& b) {
        return a.m_body->GetPos().z() < b.m_body->GetPos().z();
    });
    const ChVector<>& vel = (*lowest).m_body->GetPos_dt();
    double height = (*lowest).m_body->GetPos().z();
    cout << "[Terrain node] lowest proxy:  index = " << (*lowest).m_index << "  height = " << height
         << "  velocity = " << vel.x() << "  " << vel.y() << "  " << vel.z() << endl;
}

}  // end namespace vehicle
}  // end namespace chrono
