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

#include "chrono_vehicle/cosim/tire/ChVehicleCosimTireNodeFlexible.h"

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

    // Overwrite default integrator and solver types
    ////m_int_type = ChTimestepper::Type::HHT;
    m_int_type = ChTimestepper::Type::EULER_IMPLICIT_PROJECTED;

#if defined(CHRONO_PARDISO_MKL)
    m_slv_type = ChSolver::Type::PARDISO_MKL;
#elif defined(CHRONO_MUMPS)
    m_lsv_type = ChSolver::Type::MUMPS;
#else
    m_slv_type = ChSolver::Type::SPARSE_QR;
#endif
}

void ChVehicleCosimTireNodeFlexible::Advance(double step_size) {
    m_timer.reset();
    m_timer.start();
    double t = 0;
    while (t < step_size) {
        m_tire_def->GetMesh()->ResetCounters();
        m_tire_def->GetMesh()->ResetTimers();
        double h = std::min<>(m_step_size, step_size - t);
        m_system->DoStepDynamics(h);
        t += h;
    }
    m_timer.stop();
    m_cum_sim_time += m_timer();

    // Possible rendering
    Render(step_size);
}

void ChVehicleCosimTireNodeFlexible::OnRender() {
    if (!m_vsys)
        return;
    if (!m_vsys->Run())
        MPI_Abort(MPI_COMM_WORLD, 1);

    if (m_track) {
        ChVector<> cam_point = m_spindle->GetPos();
        m_vsys->UpdateCamera(cam_point + ChVector<>(1, 2, -0.6), cam_point);
    }

    m_vsys->BeginScene();
    m_vsys->Render();
    m_vsys->EndScene();
}

void ChVehicleCosimTireNodeFlexible::InitializeTire(std::shared_ptr<ChWheel> wheel, const ChVector<>& init_loc) {
    m_tire_def->EnablePressure(m_tire_pressure);
    m_tire_def->EnableContact(true);
    m_tire_def->EnableRimConnection(true);
    m_tire_def->SetContactSurfaceType(ChDeformableTire::ContactSurfaceType::TRIANGLE_MESH);

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
    auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
    auto& verts = trimesh->getCoordsVertices();
    ////auto& norms = trimesh->getCoordsNormals();
    auto& idx_verts = trimesh->getIndicesVertexes();
    auto& idx_norms = trimesh->getIndicesNormals();

    std::vector<ChVector<>> vvel;
    m_contact_load->OutputSimpleMesh(verts, vvel, idx_verts);

    //// RADU TODO
    std::copy(idx_verts.begin(), idx_verts.end(), std::back_inserter(idx_norms));
    idx_norms.resize(idx_verts.size(), ChVector<>(0,0,1));

    // Tire geometry and contact material
    auto cmat = m_tire_def->GetContactMaterial();
    m_geometry.m_coll_meshes.push_back(ChVehicleGeometry::TrimeshShape(VNULL, trimesh, 0.0, 0));
    m_geometry.m_materials.push_back(ChContactMaterialData(cmat->GetKfriction(), cmat->GetRestitution(),
                                                           cmat->GetYoungModulus(), cmat->GetPoissonRatio(),
                                                           cmat->GetKn(), cmat->GetGn(), cmat->GetKt(), cmat->GetGt()));

    // Preprocess the tire mesh and store neighbor element information for each vertex
    // and vertex indices for each element. This data is used in output.
    auto mesh = m_tire_def->GetMesh();
    m_adjElements.resize(mesh->GetNnodes());
    m_adjVertices.resize(mesh->GetNelements());

    int nodeOrder[] = {0, 1, 2, 3};
    for (unsigned int ie = 0; ie < mesh->GetNelements(); ie++) {
        auto element = mesh->GetElement(ie);
        for (int in = 0; in < 4; in++) {
            auto node = element->GetNodeN(nodeOrder[in]);
            auto node_itr = std::find(mesh->GetNodes().begin(), mesh->GetNodes().end(), node);
            auto iv = std::distance(mesh->GetNodes().begin(), node_itr);
            m_adjElements[iv].push_back(ie);
            m_adjVertices[ie].push_back((unsigned int)iv);
        }
    }

    // Create the visualization window (only for the first tire)
    if (m_renderRT && m_index == 0) {
#if defined(CHRONO_VSG)
        auto vsys_vsg = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        vsys_vsg->AttachSystem(m_system);
        vsys_vsg->SetWindowTitle("Tire 0 Node");
        vsys_vsg->SetWindowSize(ChVector2<int>(1280, 720));
        vsys_vsg->SetWindowPosition(ChVector2<int>(100, 100));
        vsys_vsg->SetUseSkyBox(false);
        vsys_vsg->SetClearColor(ChColor(0.455f, 0.525f, 0.640f));
        vsys_vsg->AddCamera(m_cam_pos, ChVector<>(0, 0, 0));
        vsys_vsg->SetCameraAngleDeg(40);
        vsys_vsg->SetLightIntensity(1.0f);
        vsys_vsg->AddGrid(0.1, 0.1, 40, 20, ChCoordsys<>(init_loc, Q_from_AngX(CH_C_PI_2)),
                         ChColor(0.1f, 0.1f, 0.1f));
        vsys_vsg->SetImageOutputDirectory(m_node_out_dir + "/images");
        vsys_vsg->SetImageOutput(m_writeRT);
        vsys_vsg->Initialize();

        vsys_vsg->ToggleCOGFrameVisibility();

        m_vsys = vsys_vsg;
#elif defined(CHRONO_IRRLICHT)
        auto vsys_irr = chrono_types::make_shared<irrlicht::ChVisualSystemIrrlicht>();
        vsys_irr->AttachSystem(m_system);
        vsys_irr->SetWindowTitle("Tire 0 Node");
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

void ChVehicleCosimTireNodeFlexible::LoadMeshState(MeshState& mesh_state) {
    // Extract tire mesh vertex locations and velocites
    std::vector<ChVector<int>> triangles;
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
    m_spindle->SetPos_dt(spindle_state.lin_vel);
    m_spindle->SetRot(spindle_state.rot);
    m_spindle->SetWvel_par(spindle_state.ang_vel);
}

void ChVehicleCosimTireNodeFlexible::ApplyMeshForces(const MeshContact& mesh_contact) {
    m_contact_load->InputSimpleForces(mesh_contact.vforce, mesh_contact.vidx);

    if (m_verbose)
        PrintContactData(mesh_contact.vforce, mesh_contact.vidx);
}

void ChVehicleCosimTireNodeFlexible::OnOutputData(int frame) {
    // Create and write frame output file.
    utils::CSV_writer csv(" ");
    csv << m_system->GetChTime() << endl;
    WriteTireStateInformation(csv);
    WriteTireMeshInformation(csv);

    std::string filename = OutputFilename(m_node_out_dir + "/simulation", "data", "dat", frame + 1, 5);
    csv.write_to_file(filename);

    if (m_verbose)
        cout << "[Tire node   ] write output file ==> " << filename << endl;
}

void ChVehicleCosimTireNodeFlexible::OutputVisualizationData(int frame) {
    //// TODO (format?)
}

void ChVehicleCosimTireNodeFlexible::WriteTireStateInformation(utils::CSV_writer& csv) {
    // Extract vertex states from mesh
    auto mesh = m_tire_def->GetMesh();
    ChState x(mesh->GetDOF(), NULL);
    ChStateDelta v(mesh->GetDOF_w(), NULL);
    unsigned int offset_x = 0;
    unsigned int offset_v = 0;
    double t;
    for (unsigned int in = 0; in < mesh->GetNnodes(); in++) {
        auto node = mesh->GetNode(in);
        node->NodeIntStateGather(offset_x, x, offset_v, v, t);
        offset_x += node->GetNdofX();
        offset_v += node->GetNdofW();
    }

    // Write number of vertices, number of DOFs
    csv << mesh->GetNnodes() << mesh->GetDOF() << mesh->GetDOF_w() << endl;

    // Write mesh vertex positions and velocities
    for (int ix = 0; ix < x.size(); ix++)
        csv << x(ix) << endl;
    for (int iv = 0; iv < v.size(); iv++)
        csv << v(iv) << endl;
}

void ChVehicleCosimTireNodeFlexible::WriteTireMeshInformation(utils::CSV_writer& csv) {
    // Extract mesh
    auto mesh = m_tire_def->GetMesh();

    // Print tire mesh connectivity
    csv << "\n Connectivity " << mesh->GetNelements() << 5 * mesh->GetNelements() << endl;

    for (unsigned int ie = 0; ie < mesh->GetNelements(); ie++) {
        for (unsigned int in = 0; in < m_adjVertices[ie].size(); in++) {
            csv << m_adjVertices[ie][in];
        }
        csv << endl;
    }

    // Print strain information: eps_xx, eps_yy, eps_xy averaged over surrounding elements
    /*
    csv << "\n Vectors of Strains \n";
    for (unsigned int in = 0; in < mesh->GetNnodes(); in++) {
        double areaX = 0, areaY = 0, areaZ = 0;
        double area = 0;
        for (unsigned int ie = 0; ie < m_adjElements[in].size(); ie++) {
            auto element =
                std::static_pointer_cast<fea::ChElementShellANCF_3423>(mesh->GetElement(m_adjElements[in][ie]));
            auto StrainStress = element->EvaluateSectionStrainStress(ChVector<>(0, 0, 0), 0);
            ChVector<> StrainVector = StrainStress.strain;
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

void ChVehicleCosimTireNodeFlexible::PrintLowestNode() {
    // Unfortunately, we do not have access to the node container of a mesh, so we cannot use some nice algorithm here.
    unsigned int num_nodes = m_tire_def->GetMesh()->GetNnodes();
    unsigned int index = 0;
    double zmin = 1e10;
    for (unsigned int i = 0; i < num_nodes; ++i) {
        double z = 1e10;
        // Ugly castings here!!!
        if (auto nodeXYZ = std::dynamic_pointer_cast<fea::ChNodeFEAxyz>(m_tire_def->GetMesh()->GetNode(i))) {
            z = nodeXYZ->GetPos().z();
        } else if (auto nodeXYZrot = std::dynamic_pointer_cast<fea::ChNodeFEAxyzrot>(m_tire_def->GetMesh()->GetNode(i))) {
            z = nodeXYZrot->GetPos().z();
        }

        if (z < zmin) {
            zmin = z;
            index = i;
        }
    }

    cout << "[Tire node   ] lowest node:    index = " << index << "  height = " << zmin << endl;
}

void ChVehicleCosimTireNodeFlexible::PrintContactData(const std::vector<ChVector<>>& forces,
                                                      const std::vector<int>& indices) {
    cout << "[Tire node   ] contact forces" << endl;
    for (int i = 0; i < indices.size(); i++) {
        cout << "  id = " << indices[i] << "  force = " << forces[i].x() << "  " << forces[i].y() << "  "
             << forces[i].z() << endl;
    }
}

}  // namespace vehicle
}  // namespace chrono
