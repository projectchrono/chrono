// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
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
// Definition of the vehicle co-simulation flexible TIRE NODE class.
// This type of tire communicates with the terrain node through a MESH
// communication interface.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono/solver/ChDirectSolverLS.h"
#include "chrono/timestepper/ChTimestepperHHT.h"

#include "chrono_vehicle/cosim/tire/ChVehicleCosimTireNodeFlexible.h"

#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

////#ifdef CHRONO_MUMPS
////    #include "chrono_mumps/ChSolverMumps.h"
////#endif

#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#endif
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
#endif

using std::cout;
using std::endl;

namespace chrono {
namespace vehicle {

ChVehicleCosimTireNodeFlexible::ChVehicleCosimTireNodeFlexible(int index, const std::string& tire_json)
    : ChVehicleCosimTireNode(index, tire_json) {
    assert(GetTireTypeFromSpecfile(tire_json) == TireType::FLEXIBLE);
    assert(m_tire);
    m_tire_def = std::static_pointer_cast<ChDeformableTire>(m_tire);  // cache tire as ChDeformableTire

    // Overwrite default solver and integrator
#if defined(CHRONO_PARDISO_MKL)
    auto solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    solver->LockSparsityPattern(true);
    m_system->SetSolver(solver);
////#elif defined(CHRONO_MUMPS)
////    auto solver = chrono_types::make_shared<ChSolverMumps>();
////    solver->LockSparsityPattern(true);
////    m_system->SetSolver(solver);
#else
    auto solver = chrono_types::make_shared<ChSolverSparseQR>();
    solver->LockSparsityPattern(true);
    m_system->SetSolver(solver);
#endif

    auto integrator = chrono_types::make_shared<ChTimestepperHHT>(m_system);
    integrator->SetAlpha(-0.2);
    integrator->SetMaxIters(50);
    integrator->SetAbsTolerances(1e-04, 1e2);
    integrator->SetStepControl(false);
    integrator->SetModifiedNewton(false);
    integrator->SetVerbose(false);
    m_system->SetTimestepper(integrator);

    // Overwrite default number of threads
    m_system->SetNumThreads(std::min(4, ChOMP::GetNumProcs()), 0, std::min(4, ChOMP::GetNumProcs()));
}

void ChVehicleCosimTireNodeFlexible::Advance(double step_size) {
    m_timer.reset();
    m_timer.start();
    double t = 0;
    while (t < step_size) {
        m_tire_def->GetMesh()->ResetCounters();
        m_tire_def->GetMesh()->ResetTimers();
        double h = std::min<>(m_step_size, step_size - t);
        if (h < 1e-8)
            break;
        m_system->DoStepDynamics(h);
        t += h;
    }
    m_timer.stop();
    m_cum_sim_time += m_timer();

    // Possible rendering
    Render(step_size);
}

void ChVehicleCosimTireNodeFlexible::AddVisualShapeFEA(std::shared_ptr<ChVisualShapeFEA> shape) {
    m_tire_def->AddVisualShapeFEA(shape);
}

void ChVehicleCosimTireNodeFlexible::OnRender() {
    if (!m_vsys)
        return;
    if (!m_vsys->Run())
        MPI_Abort(MPI_COMM_WORLD, 1);

    if (m_track) {
        ChVector3d cam_point = m_spindle->GetPos();
        m_vsys->UpdateCamera(cam_point + ChVector3d(1, 2, -0.6), cam_point);
    }

    m_vsys->BeginScene();
    m_vsys->Render();
    m_vsys->EndScene();
}

void ChVehicleCosimTireNodeFlexible::InitializeTire(std::shared_ptr<ChWheel> wheel, const ChVector3d& init_loc) {
    m_tire_def->EnablePressure(m_tire_pressure);
    m_tire_def->EnableContact(true);
    m_tire_def->EnableRimConnection(true);
    m_tire_def->SetContactSurfaceType(ChTire::ContactSurfaceType::TRIANGLE_MESH);

    m_spindle->SetPos(init_loc);
    wheel->SetTire(m_tire);
    m_tire->Initialize(wheel);
    m_tire->SetVisualizationType(VisualizationType::MESH);

    // Create a mesh load for contact forces and add it to the tire's load container
    auto contact_surface = std::static_pointer_cast<fea::ChContactSurfaceMesh>(m_tire_def->GetContactSurface());
    m_contact_load = chrono_types::make_shared<fea::ChLoadContactSurfaceMesh>(contact_surface);
    m_tire_def->GetLoadContainer()->Add(m_contact_load);

    // Set mesh data (initial configuration, vertex positions in local frame)
    //// TODO: vertex normals?
    auto trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    auto& verts = trimesh->GetCoordsVertices();
    ////auto& norms = trimesh->GetCoordsNormals();
    auto& idx_verts = trimesh->GetIndicesVertexes();
    auto& idx_norms = trimesh->GetIndicesNormals();

    std::vector<ChVector3d> vvel;
    m_contact_load->OutputSimpleMesh(verts, vvel, idx_verts);

    //// RADU TODO
    std::copy(idx_verts.begin(), idx_verts.end(), std::back_inserter(idx_norms));
    idx_norms.resize(idx_verts.size(), ChVector3d(0, 0, 1));

    // Tire geometry and contact material
    auto cmat = m_tire_def->GetContactMaterial();
    m_geometry.coll_meshes.push_back(utils::ChBodyGeometry::TrimeshShape(VNULL, trimesh, 0.0, 0));
    m_geometry.materials.push_back(ChContactMaterialData(cmat->GetSlidingFriction(), cmat->GetRestitution(),
                                                         cmat->GetYoungModulus(), cmat->GetPoissonRatio(),
                                                         cmat->GetKn(), cmat->GetGn(), cmat->GetKt(), cmat->GetGt()));

    // Preprocess the tire mesh and store neighbor element information for each vertex
    // and vertex indices for each element. This data is used in output.
    auto mesh = m_tire_def->GetMesh();
    m_adjElements.resize(mesh->GetNumNodes());
    m_adjVertices.resize(mesh->GetNumElements());

    int nodeOrder[] = {0, 1, 2, 3};
    for (unsigned int ie = 0; ie < mesh->GetNumElements(); ie++) {
        auto element = mesh->GetElement(ie);
        for (int in = 0; in < 4; in++) {
            auto node = element->GetNode(nodeOrder[in]);
            auto node_itr = std::find(mesh->GetNodes().begin(), mesh->GetNodes().end(), node);
            auto iv = std::distance(mesh->GetNodes().begin(), node_itr);
            m_adjElements[iv].push_back(ie);
            m_adjVertices[ie].push_back((unsigned int)iv);
        }
    }

    // Create the visualization window
    if (m_renderRT) {
        std::string title = "Tire " + std::to_string(m_index) + " Node";

#if defined(CHRONO_VSG)
        auto vsys_vsg = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        vsys_vsg->AttachSystem(m_system);
        vsys_vsg->SetWindowTitle(title);
        vsys_vsg->SetWindowSize(ChVector2i(1280, 720));
        vsys_vsg->SetWindowPosition(ChVector2i(100, 100));
        vsys_vsg->SetClearColor(ChColor(0.455f, 0.525f, 0.640f));
        vsys_vsg->AddCamera(m_cam_pos, ChVector3d(0, 0, 0));
        vsys_vsg->SetCameraAngleDeg(40);
        vsys_vsg->SetLightIntensity(1.0f);
        vsys_vsg->AddGrid(0.1, 0.1, 40, 20, ChCoordsys<>(init_loc, QuatFromAngleX(CH_PI_2)), ChColor(0.1f, 0.1f, 0.1f));
        vsys_vsg->SetImageOutputDirectory(m_node_out_dir + "/images");
        vsys_vsg->SetImageOutput(m_writeRT);
        vsys_vsg->Initialize();

        vsys_vsg->ToggleCOMFrameVisibility();

        m_vsys = vsys_vsg;
#elif defined(CHRONO_IRRLICHT)
        auto vsys_irr = chrono_types::make_shared<irrlicht::ChVisualSystemIrrlicht>();
        vsys_irr->AttachSystem(m_system);
        vsys_irr->SetWindowTitle(title);
        vsys_irr->SetCameraVertical(CameraVerticalDir::Z);
        vsys_irr->SetWindowSize(1280, 720);
        vsys_irr->Initialize();
        vsys_irr->AddLogo();
        vsys_irr->AddSkyBox();
        vsys_irr->AddTypicalLights();
        vsys_irr->AddCamera(m_cam_pos, ChVector3d(0, 0, 0));

        m_vsys = vsys_irr;
#endif
    }
}

void ChVehicleCosimTireNodeFlexible::LoadMeshState(MeshState& mesh_state) {
    // Extract tire mesh vertex locations and velocites
    std::vector<ChVector3i> triangles;
    m_contact_load->OutputSimpleMesh(mesh_state.vpos, mesh_state.vvel, triangles);

    // Display information on lowest mesh node
    if (m_verbose)
        PrintLowestNode();
}

void ChVehicleCosimTireNodeFlexible::LoadSpindleForce(TerrainForce& spindle_force) {
    spindle_force = m_tire_def->ReportTireForce(nullptr);
}

void ChVehicleCosimTireNodeFlexible::ApplySpindleState(const BodyState& spindle_state) {
    m_spindle->SetPos(spindle_state.pos);
    m_spindle->SetPosDt(spindle_state.lin_vel);
    m_spindle->SetRot(spindle_state.rot);
    m_spindle->SetAngVelParent(spindle_state.ang_vel);
}

void ChVehicleCosimTireNodeFlexible::ApplyMeshForces(const MeshContact& mesh_contact) {
    // Cache mesh nodal contact forces for reporting
    m_forces = mesh_contact;  

    // Load contact forces
    m_contact_load->InputSimpleForces(mesh_contact.vforce, mesh_contact.vidx);

    if (m_verbose) {
        ////PrintContactData(mesh_contact.vforce, mesh_contact.vidx);
        
        ////if (m_index == 0) {
        ////    if (mesh_contact.vforce.size() > 0) {
        ////        double sum = 0;
        ////        for (auto f : mesh_contact.vforce)
        ////            sum += f.Length();
        ////        std::cout << " vforce size = " << mesh_contact.vforce.size() << "   sum = " << sum << std::endl;
        ////    }
        ////    if (mesh_contact.vidx.size() > 0)
        ////        std::cout << " vidx size   = " << mesh_contact.vidx.size() << std::endl;
        ////}
    }
}

void ChVehicleCosimTireNodeFlexible::OnOutputData(int frame) {

    // Write fixed mesh information
    if (frame == 0) {
        utils::ChWriterCSV csv(" ");
        WriteTireMeshInformation(csv);
        std::string filename = m_node_out_dir + "/mesh_info.dat";
        csv.WriteToFile(filename);
    }

    // Write current mesh state
    {
        utils::ChWriterCSV csv(" ");
        csv << m_system->GetChTime() << endl;
        WriteTireStateInformation(csv);
        std::string filename = OutputFilename(m_node_out_dir + "/simulation", "mesh_state", "dat", frame + 1, 5);
        csv.WriteToFile(filename);
        if (m_verbose)
            cout << "[Tire node   ] write output file ==> " << filename << endl;
    }

    // Write current terrain forces
    {
        utils::ChWriterCSV csv(" ");
        csv << m_system->GetChTime() << endl;
        WriteTireTerrainForces(csv);
        std::string filename = OutputFilename(m_node_out_dir + "/simulation", "terrain_force", "dat", frame + 1, 5);
        csv.WriteToFile(filename);
    }
}

void ChVehicleCosimTireNodeFlexible::OutputVisualizationData(int frame) {
    //// TODO (format?)
}

void ChVehicleCosimTireNodeFlexible::WriteTireMeshInformation(utils::ChWriterCSV& csv) {
    // Extract mesh
    auto mesh = m_tire_def->GetMesh();

    // Print tire mesh connectivity
    csv << "\n Connectivity " << mesh->GetNumElements() << 5 * mesh->GetNumElements() << endl;

    for (unsigned int ie = 0; ie < mesh->GetNumElements(); ie++) {
        for (unsigned int in = 0; in < m_adjVertices[ie].size(); in++) {
            csv << m_adjVertices[ie][in];
        }
        csv << endl;
    }
}

void ChVehicleCosimTireNodeFlexible::WriteTireStateInformation(utils::ChWriterCSV& csv) {
    // Extract vertex states from mesh
    auto mesh = m_tire_def->GetMesh();
    ChState x(mesh->GetNumCoordsPosLevel(), NULL);
    ChStateDelta v(mesh->GetNumCoordsVelLevel(), NULL);
    unsigned int offset_x = 0;
    unsigned int offset_v = 0;
    double t;
    for (unsigned int in = 0; in < mesh->GetNumNodes(); in++) {
        auto node = mesh->GetNode(in);
        node->NodeIntStateGather(offset_x, x, offset_v, v, t);
        offset_x += node->GetNumCoordsPosLevel();
        offset_v += node->GetNumCoordsVelLevel();
    }

    // Write number of vertices, number of DOFs
    csv << mesh->GetNumNodes() << mesh->GetNumCoordsPosLevel() << mesh->GetNumCoordsVelLevel() << endl;

    // Write mesh vertex positions and velocities
    for (int ix = 0; ix < x.size(); ix++)
        csv << x(ix) << endl;
    for (int iv = 0; iv < v.size(); iv++)
        csv << v(iv) << endl;

    // Print strain information: eps_xx, eps_yy, eps_xy averaged over surrounding elements
    /*
    csv << "\n Vectors of Strains \n";
    for (unsigned int in = 0; in < mesh->GetNumNodes(); in++) {
        double areaX = 0, areaY = 0, areaZ = 0;
        double area = 0;
        for (unsigned int ie = 0; ie < m_adjElements[in].size(); ie++) {
            auto element =
                std::static_pointer_cast<fea::ChElementShellANCF_3423>(mesh->GetElement(m_adjElements[in][ie]));
            auto StrainStress = element->EvaluateSectionStrainStress(ChVector3d(0, 0, 0), 0);
            ChVector3d StrainVector = StrainStress.strain;
            double dx = element->GetLengthX();
            double dy = element->GetLengthY();
            area += dx * dy / 4;
            areaX += StrainVector.x() * dx * dy / 4;
            areaY += StrainVector.y() * dx * dy / 4;
            areaZ += StrainVector.z() * dx * dy / 4;
        }
        csv << areaX / area << " " << areaY / area << " " << areaZ / area << endl;
    }
    */
}

void ChVehicleCosimTireNodeFlexible::WriteTireTerrainForces(utils::ChWriterCSV& csv) {
    // Write forces reduced to the spindle body
    auto s_force = m_tire_def->ReportTireForce(nullptr);
    csv << s_force.point << endl;
    csv << s_force.force << endl;
    csv << s_force.moment << endl;

    // Write mesh nodal forces
    csv << m_forces.nv << endl;
    for (int i = 0; i < m_forces.nv; i++) {
        csv << m_forces.vidx[i] << m_forces.vforce[i] << endl;
    }
}

void ChVehicleCosimTireNodeFlexible::PrintLowestNode() {
    // Unfortunately, we do not have access to the node container of a mesh, so we cannot use some nice algorithm here.
    unsigned int num_nodes = m_tire_def->GetMesh()->GetNumNodes();
    unsigned int index = 0;
    double zmin = 1e10;
    for (unsigned int i = 0; i < num_nodes; ++i) {
        double z = 1e10;
        // Ugly castings here!!!
        if (auto nodeXYZ = std::dynamic_pointer_cast<fea::ChNodeFEAxyz>(m_tire_def->GetMesh()->GetNode(i))) {
            z = nodeXYZ->GetPos().z();
        } else if (auto nodeXYZrot =
                       std::dynamic_pointer_cast<fea::ChNodeFEAxyzrot>(m_tire_def->GetMesh()->GetNode(i))) {
            z = nodeXYZrot->GetPos().z();
        }

        if (z < zmin) {
            zmin = z;
            index = i;
        }
    }

    cout << "[Tire node   ] lowest node:    index = " << index << "  height = " << zmin << endl;
}

void ChVehicleCosimTireNodeFlexible::PrintContactData(const std::vector<ChVector3d>& forces,
                                                      const std::vector<int>& indices) {
    cout << "[Tire node   ] contact forces" << endl;
    for (int i = 0; i < indices.size(); i++) {
        cout << "  id = " << indices[i] << "  force = " << forces[i].x() << "  " << forces[i].y() << "  "
             << forces[i].z() << endl;
    }
}

}  // namespace vehicle
}  // namespace chrono
