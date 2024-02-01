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
// Base class for a co-simulation node.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <iomanip>

#include "chrono_vehicle/cosim/ChVehicleCosimBaseNode.h"

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Free functions in the cosim namespace
// -----------------------------------------------------------------------------

namespace cosim {

static MPI_Comm terrain_comm = MPI_COMM_NULL;

int InitializeFramework(int num_tires) {
    int world_size;
    MPI_Comm_size(MPI_COMM_WORLD, &world_size);
    if (world_size < 2 + num_tires) {
        return MPI_ERR_OTHER;
    }

    // Get the group for MPI_COMM_WORLD
    MPI_Group world_group;
    MPI_Comm_group(MPI_COMM_WORLD, &world_group);

    // Set list of excluded ranks (vehicle and tire nodes)
    std::vector<int> excluded;
    excluded.push_back(MBS_NODE_RANK);
    for (int i = 0; i < num_tires; i++)
        excluded.push_back(TIRE_NODE_RANK(i));

    // Create the group of ranks for terrain simulation
    MPI_Group terrain_group;
    MPI_Group_excl(world_group, 1 + num_tires, excluded.data(), &terrain_group);

    // Create and return a communicator from the terrain group
    MPI_Comm_create(MPI_COMM_WORLD, terrain_group, &terrain_comm);

    return MPI_SUCCESS;
}

bool IsFrameworkInitialized() {
    return terrain_comm != MPI_COMM_NULL;
}

MPI_Comm GetTerrainIntracommunicator() {
    return terrain_comm;
}

}  // end namespace cosim

// -----------------------------------------------------------------------------

const double ChVehicleCosimBaseNode::m_gacc = -9.81;

ChVehicleCosimBaseNode::ChVehicleCosimBaseNode(const std::string& name)
    : m_name(name),
      m_step_size(1e-4),
      m_cum_sim_time(0),
      m_verbose(true),
      m_renderRT(false),
      m_renderRT_step(0.01),
      m_writeRT(false),
      m_renderPP(false),
      m_renderPP_step(0.01),
      m_track(true),
      m_cam_pos({1, 0, 0}),
      m_cam_target({0, 0, 0}),
      m_num_wheeled_mbs_nodes(0),
      m_num_tracked_mbs_nodes(0),
      m_num_terrain_nodes(0),
      m_num_tire_nodes(0),
      m_rank(-1) {
    MPI_Comm_rank(MPI_COMM_WORLD, &m_rank);
}

void ChVehicleCosimBaseNode::Initialize() {
    int size;
    MPI_Comm_size(MPI_COMM_WORLD, &size);

    // Gather node types from all ranks
    int type = -1;
    switch (GetNodeType()) {
        case NodeType::MBS_WHEELED:
            type = 0;
            break;
        case NodeType::MBS_TRACKED:
            type = 1;
            break;
        case NodeType::TERRAIN:
            type = 2;
            break;
        case NodeType::TIRE:
            type = 3;
            break;
    }
    int* type_all = new int[size];
    MPI_Allgather(&type, 1, MPI_INT, type_all, 1, MPI_INT, MPI_COMM_WORLD);

    // Calculate number of different node types
    for (int i = 0; i < size; i++) {
        switch (type_all[i]) {
            case 0:
                m_num_wheeled_mbs_nodes++;
                break;
            case 1:
                m_num_tracked_mbs_nodes++;
                break;
            case 2:
                m_num_terrain_nodes++;
                break;
            case 3:
                m_num_tire_nodes++;
                break;
        }
    }

    if (m_verbose && m_rank == 0) {
        cout << "Num nodes: " << size                               //
             << "  WHEELED MBS nodes: " << m_num_wheeled_mbs_nodes  //
             << "  TRACKED MBS nodes: " << m_num_tracked_mbs_nodes  //
             << "  TERRAIN nodes: " << m_num_terrain_nodes          //
             << "  TIRE nodes: " << m_num_tire_nodes << endl;       //
    }

    // Error checks
    bool err = false;

    if (m_num_wheeled_mbs_nodes + m_num_tracked_mbs_nodes != 1) {
        if (m_rank == 0)
            cerr << "Error: Exactly one MBS node must be present." << endl;
        err = true;
    }

    if (m_num_tracked_mbs_nodes == 1 && m_num_tire_nodes > 0) {
        if (m_rank == 0)
            cerr << "Error: Tire nodes cannot be present for a tracked MBS simulation." << endl;
        err = true;
    }

    if (type_all[MBS_NODE_RANK] != 0 && type_all[MBS_NODE_RANK] != 1) {
        if (m_rank == 0)
            cerr << "Error: rank " << MBS_NODE_RANK << " is not running an MBS node." << endl;
        err = true;
    }

    if (type_all[TERRAIN_NODE_RANK] != 2) {
        if (m_rank == 0)
            cerr << "Error: rank " << TERRAIN_NODE_RANK << " is not running a TERRAIN node." << endl;
        err = true;
    }

    for (unsigned int i = 0; i < m_num_tire_nodes; i++) {
        if (type_all[TIRE_NODE_RANK(i)] != 3) {
            if (m_rank == 0)
                cerr << "Error: rank " << TIRE_NODE_RANK(i) << " is not running a TIRE node." << endl;
            err = true;
        }
    }

    if (err) {
        MPI_Abort(MPI_COMM_WORLD, 1);
    }
}

void ChVehicleCosimBaseNode::SetOutDir(const std::string& dir_name, const std::string& suffix) {
    m_out_dir = dir_name;
    m_node_out_dir = dir_name + "/" + m_name + suffix;

    // Create node-specific output directory
    if (!filesystem::create_directory(filesystem::path(m_node_out_dir))) {
        std::cout << "Error creating directory " << m_node_out_dir << std::endl;
        return;
    }

    // Create subdirectories for simulation and visualization outputs
    if (!filesystem::create_directory(filesystem::path(m_node_out_dir + "/simulation"))) {
        std::cout << "Error creating directory " << m_node_out_dir + "/simulation" << std::endl;
        return;
    }
    if (!filesystem::create_directory(filesystem::path(m_node_out_dir + "/visualization"))) {
        std::cout << "Error creating directory " << m_node_out_dir + "/visualization" << std::endl;
        return;
    }
    if (!filesystem::create_directory(filesystem::path(m_node_out_dir + "/images"))) {
        std::cout << "Error creating directory " << m_node_out_dir + "/images" << std::endl;
        return;
    }

    // Create results output file
    m_outf.open(m_node_out_dir + "/results.dat", std::ios::out);
    m_outf.precision(7);
    m_outf << std::scientific;
}

std::string ChVehicleCosimBaseNode::OutputFilename(const std::string& dir,
                                                   const std::string& root,
                                                   const std::string& ext,
                                                   int frame,
                                                   int frame_digits) {
    // Frame number is zero padded for postprocessing
    std::ostringstream filename;
    filename << dir << "/" << root << "_" << std::setw(frame_digits) << std::setfill('0') << frame << "." << ext;
    return filename.str();
}

bool ChVehicleCosimBaseNode::IsCosimNode() const {
    if (m_num_terrain_nodes == 1)
        return true;
    if (m_rank == TERRAIN_NODE_RANK)
        return true;
    return GetNodeType() != NodeType::TERRAIN;
}

std::string ChVehicleCosimBaseNode::GetNodeTypeString() const {
    switch (GetNodeType()) {
        case NodeType::MBS_WHEELED:
            return "MBS wheeled";
        case NodeType::MBS_TRACKED:
            return "MBS tracked";
        case NodeType::TIRE:
            return "Tire";
        case NodeType::TERRAIN:
            return "Terrain";
        default:
            return "Unknown";
    }
}

void ChVehicleCosimBaseNode::EnableRuntimeVisualization(double render_fps, bool save_img) {
    m_renderRT = true;
    m_writeRT = save_img;
    if (render_fps <= 0) {
        m_renderRT_all = true;
        m_renderRT_step = 0;
    } else {
        m_renderRT_all = false;
        m_renderRT_step = 1.0 / render_fps;
    }
}

void ChVehicleCosimBaseNode::EnablePostprocessVisualization(double render_fps) {
    m_renderPP = true;
    if (render_fps <= 0) {
        m_renderPP_all = true;
        m_renderPP_step = 0;
    } else {
        m_renderPP_all = false;
        m_renderPP_step = 1.0 / render_fps;
    }
}

void ChVehicleCosimBaseNode::SetCameraPosition(const ChVector<>& cam_pos, const ChVector<>& cam_target) {
    m_cam_pos = cam_pos;
    m_cam_target = cam_target;
}

void ChVehicleCosimBaseNode::Render(double step_size) {
    static double sim_time = 0;
    static double renderRT_time = 0;
    static double renderPP_time = 0;
    static bool renderPP_initialized = false;

    sim_time += step_size;

    if (m_renderRT) {
        if (m_renderRT_all || sim_time >= renderRT_time) {
            OnRender();
            renderRT_time += m_renderRT_step;
        }
    }

    if (m_renderPP && GetSystemPostprocess()) {
#ifdef CHRONO_POSTPROCESS
        if (!renderPP_initialized) {
            m_blender = chrono_types::make_shared<postprocess::ChBlender>(GetSystemPostprocess());
            m_blender->SetBlenderUp_is_ChronoZ();
            m_blender->SetBasePath(m_node_out_dir + "/blender");
            m_blender->SetRank(m_rank);
            m_blender->AddAll();
            m_blender->ExportScript();
            renderPP_initialized = true;
        }
        if (m_renderPP_all || sim_time >= renderPP_time) {
            if (m_verbose)
                cout << "[" << GetNodeTypeString() << "] Blender export at t = " << sim_time << endl;
            m_blender->ExportData();
            renderPP_time += m_renderPP_step;
        }
#endif
    }
}

void ChVehicleCosimBaseNode::SendGeometry(const ChVehicleGeometry& geom, int dest) const {
    // Send information on number of contact materials and collision shapes of each type
    int dims[] = {
        static_cast<int>(geom.m_materials.size()),       //
        static_cast<int>(geom.m_coll_boxes.size()),      //
        static_cast<int>(geom.m_coll_spheres.size()),    //
        static_cast<int>(geom.m_coll_cylinders.size()),  //
        static_cast<int>(geom.m_coll_hulls.size()),      //
        static_cast<int>(geom.m_coll_meshes.size())      //
    };
    MPI_Send(dims, 6, MPI_INT, dest, 0, MPI_COMM_WORLD);

    // Send contact materials
    for (const auto& mat : geom.m_materials) {
        float props[] = {mat.mu, mat.cr, mat.Y, mat.nu, mat.kn, mat.gn, mat.kt, mat.gt};
        MPI_Send(props, 8, MPI_FLOAT, dest, 0, MPI_COMM_WORLD);
    }

    // Send shape geometry
    for (const auto& box : geom.m_coll_boxes) {
        double data[] = {
            box.m_pos.x(),                    //
            box.m_pos.y(),                    //
            box.m_pos.z(),                    //
            box.m_rot.e0(),                   //
            box.m_rot.e1(),                   //
            box.m_rot.e2(),                   //
            box.m_rot.e3(),                   //
            box.m_dims.x(),                   //
            box.m_dims.y(),                   //
            box.m_dims.z(),                   //
            static_cast<double>(box.m_matID)  //
        };
        MPI_Send(data, 11, MPI_DOUBLE, dest, 0, MPI_COMM_WORLD);
    }

    for (const auto& sph : geom.m_coll_spheres) {
        double data[] = {
            sph.m_pos.x(),                    //
            sph.m_pos.y(),                    //
            sph.m_pos.z(),                    //
            sph.m_radius,                     //
            static_cast<double>(sph.m_matID)  //
        };
        MPI_Send(data, 5, MPI_DOUBLE, dest, 0, MPI_COMM_WORLD);
    }

    for (const auto& cyl : geom.m_coll_cylinders) {
        double data[] = {
            cyl.m_pos.x(),                    //
            cyl.m_pos.y(),                    //
            cyl.m_pos.z(),                    //
            cyl.m_rot.e0(),                   //
            cyl.m_rot.e1(),                   //
            cyl.m_rot.e2(),                   //
            cyl.m_rot.e3(),                   //
            cyl.m_radius,                     //
            cyl.m_length,                     //
            static_cast<double>(cyl.m_matID)  //
        };
        MPI_Send(data, 10, MPI_DOUBLE, dest, 0, MPI_COMM_WORLD);
    }

    /*
    for (const auto& hull : geom.m_coll_hulls) {
        //// RADU TODO
    }
    */

    for (const auto& mesh : geom.m_coll_meshes) {
        double data[] = {mesh.m_pos.x(), mesh.m_pos.y(), mesh.m_pos.z()};
        MPI_Send(data, 3, MPI_DOUBLE, dest, 0, MPI_COMM_WORLD);

        const auto& trimesh = mesh.m_trimesh;
        const auto& vertices = trimesh->getCoordsVertices();
        const auto& normals = trimesh->getCoordsNormals();
        const auto& idx_vertices = trimesh->getIndicesVertexes();
        const auto& idx_normals = trimesh->getIndicesNormals();
        int nv = trimesh->getNumVertices();
        int nn = trimesh->getNumNormals();
        int nt = trimesh->getNumTriangles();

        int surf_props[] = {nv, nn, nt, mesh.m_matID};
        MPI_Send(surf_props, 4, MPI_INT, dest, 0, MPI_COMM_WORLD);
        if (m_verbose)
            cout << "[" << GetNodeTypeString() << "] Send: vertices = " << surf_props[0]
                 << "  triangles = " << surf_props[2] << endl;

        double* vert_data = new double[3 * nv + 3 * nn];
        int* tri_data = new int[3 * nt + 3 * nt];
        for (int iv = 0; iv < nv; iv++) {
            vert_data[3 * iv + 0] = vertices[iv].x();
            vert_data[3 * iv + 1] = vertices[iv].y();
            vert_data[3 * iv + 2] = vertices[iv].z();
        }
        for (int in = 0; in < nn; in++) {
            vert_data[3 * nv + 3 * in + 0] = normals[in].x();
            vert_data[3 * nv + 3 * in + 1] = normals[in].y();
            vert_data[3 * nv + 3 * in + 2] = normals[in].z();
        }
        for (int it = 0; it < nt; it++) {
            tri_data[6 * it + 0] = idx_vertices[it].x();
            tri_data[6 * it + 1] = idx_vertices[it].y();
            tri_data[6 * it + 2] = idx_vertices[it].z();
            tri_data[6 * it + 3] = idx_normals[it].x();
            tri_data[6 * it + 4] = idx_normals[it].y();
            tri_data[6 * it + 5] = idx_normals[it].z();
        }
        MPI_Send(vert_data, 3 * nv + 3 * nn, MPI_DOUBLE, TERRAIN_NODE_RANK, 0, MPI_COMM_WORLD);
        MPI_Send(tri_data, 3 * nt + 3 * nt, MPI_INT, TERRAIN_NODE_RANK, 0, MPI_COMM_WORLD);
    }
}

void ChVehicleCosimBaseNode::RecvGeometry(ChVehicleGeometry& geom, int source) const {
    MPI_Status status;

    // Receive information on number of contact materials and collision shapes of each type
    int dims[6];
    MPI_Recv(dims, 6, MPI_INT, source, 0, MPI_COMM_WORLD, &status);
    int num_materials = dims[0];
    int num_boxes = dims[1];
    int num_spheres = dims[2];
    int num_cylinders = dims[3];
    int num_hulls = dims[4];
    int num_meshes = dims[5];

    // Receive contact materials
    for (int i = 0; i < num_materials; i++) {
        float props[8];
        MPI_Recv(props, 8, MPI_FLOAT, source, 0, MPI_COMM_WORLD, &status);
        geom.m_materials.push_back(
            ChContactMaterialData(props[0], props[1], props[2], props[3], props[4], props[5], props[6], props[7]));
    }

    // Receive shape geometry
    for (int i = 0; i < num_boxes; i++) {
        double data[11];
        MPI_Recv(data, 11, MPI_DOUBLE, source, 0, MPI_COMM_WORLD, &status);
        geom.m_coll_boxes.push_back(                                                         //
            ChVehicleGeometry::BoxShape(ChVector<>(data[0], data[1], data[2]),               //
                                        ChQuaternion<>(data[3], data[4], data[5], data[6]),  //
                                        ChVector<>(data[7], data[8], data[9]),               //
                                        static_cast<int>(data[10]))                          //
        );
    }

    for (int i = 0; i < num_spheres; i++) {
        double data[5];
        MPI_Recv(data, 5, MPI_DOUBLE, source, 0, MPI_COMM_WORLD, &status);
        geom.m_coll_spheres.push_back(                                             //
            ChVehicleGeometry::SphereShape(ChVector<>(data[0], data[1], data[2]),  //
                                           data[3],                                //
                                           static_cast<int>(data[4]))              //
        );
    }

    for (int i = 0; i < num_cylinders; i++) {
        double data[10];
        MPI_Recv(data, 10, MPI_DOUBLE, source, 0, MPI_COMM_WORLD, &status);
        geom.m_coll_cylinders.push_back(                                                          //
            ChVehicleGeometry::CylinderShape(ChVector<>(data[0], data[1], data[2]),               //
                                             ChQuaternion<>(data[3], data[4], data[5], data[6]),  //
                                             data[7], data[8],                                    //
                                             static_cast<int>(data[9]))                           //
        );
    }

    for (int i = 0; i < num_hulls; i++) {
        //// RADU TODO
    }

    for (int i = 0; i < num_meshes; i++) {
        double data[3];
        MPI_Recv(data, 3, MPI_DOUBLE, source, 0, MPI_COMM_WORLD, &status);
        ChVector<> pos(data[0], data[1], data[2]);

        auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
        auto& vertices = trimesh->getCoordsVertices();
        auto& normals = trimesh->getCoordsNormals();
        auto& idx_vertices = trimesh->getIndicesVertexes();
        auto& idx_normals = trimesh->getIndicesNormals();

        int surf_props[4];
        MPI_Recv(surf_props, 4, MPI_INT, source, 0, MPI_COMM_WORLD, &status);
        int nv = surf_props[0];
        int nn = surf_props[1];
        int nt = surf_props[2];
        int matID = surf_props[3];

        trimesh->getCoordsVertices().resize(nv);
        trimesh->getCoordsNormals().resize(nn);
        trimesh->getIndicesVertexes().resize(nt);
        trimesh->getIndicesNormals().resize(nt);

        // Tire mesh vertices & normals and triangle indices
        double* vert_data = new double[3 * nv + 3 * nn];
        int* tri_data = new int[3 * nt + 3 * nt];
        MPI_Recv(vert_data, 3 * nv + 3 * nn, MPI_DOUBLE, source, 0, MPI_COMM_WORLD, &status);
        MPI_Recv(tri_data, 3 * nt + 3 * nt, MPI_INT, source, 0, MPI_COMM_WORLD, &status);

        for (int iv = 0; iv < nv; iv++) {
            vertices[iv].x() = vert_data[3 * iv + 0];
            vertices[iv].y() = vert_data[3 * iv + 1];
            vertices[iv].z() = vert_data[3 * iv + 2];
        }
        for (int in = 0; in < nn; in++) {
            normals[in].x() = vert_data[3 * nv + 3 * in + 0];
            normals[in].y() = vert_data[3 * nv + 3 * in + 1];
            normals[in].z() = vert_data[3 * nv + 3 * in + 2];
        }
        for (int it = 0; it < nt; it++) {
            idx_vertices[it].x() = tri_data[6 * it + 0];
            idx_vertices[it].y() = tri_data[6 * it + 1];
            idx_vertices[it].z() = tri_data[6 * it + 2];
            idx_normals[it].x() = tri_data[6 * it + 3];
            idx_normals[it].y() = tri_data[6 * it + 4];
            idx_normals[it].z() = tri_data[6 * it + 5];
        }

        delete[] vert_data;
        delete[] tri_data;

        geom.m_coll_meshes.push_back(ChVehicleGeometry::TrimeshShape(pos, trimesh, 0.0, matID));
    }
}

void ChVehicleCosimBaseNode::ProgressBar(unsigned int x, unsigned int n, unsigned int w) {
    if ((x != n) && (x % (n / 100 + 1) != 0))
        return;

    float ratio = x / (float)n;
    unsigned int c = (unsigned int)(ratio * w);

    std::cout << std::setw(3) << (int)(ratio * 100) << "% [";
    for (unsigned int ix = 0; ix < c; ix++)
        std::cout << "=";
    for (unsigned int ix = c; ix < w; ix++)
        std::cout << " ";
    std::cout << "]\r" << std::flush;
}

}  // end namespace vehicle
}  // end namespace chrono
