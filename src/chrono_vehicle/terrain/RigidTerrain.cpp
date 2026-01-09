// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Rigid terrain
//
// =============================================================================

#include <limits>
#include <algorithm>
#include <cmath>
#include <cstdio>

#include "chrono/assets/ChVisualSystem.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/physics/ChContactMaterialNSC.h"
#include "chrono/physics/ChContactMaterialSMC.h"
#include "chrono/input_output/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/ChWorldFrame.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_thirdparty/stb/stb.h"
#include "chrono_thirdparty/filesystem/path.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Default constructor.
// -----------------------------------------------------------------------------
RigidTerrain::RigidTerrain(ChSystem* system)
    : m_system(system),
      m_num_patches(0),
      m_use_friction_functor(false),
      m_contact_callback(nullptr),
      m_collision_family(14),
      m_initialized(false) {}

// -----------------------------------------------------------------------------
// Constructor from JSON file
// -----------------------------------------------------------------------------
RigidTerrain::RigidTerrain(ChSystem* system, const std::string& filename)
    : m_system(system),
      m_num_patches(0),
      m_use_friction_functor(false),
      m_contact_callback(nullptr),
      m_collision_family(14),
      m_initialized(false) {
    // Open and parse the input file
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    // Read top-level data
    assert(d.HasMember("Type"));
    assert(d.HasMember("Template"));
    assert(d.HasMember("Name"));

    // Extract number of patches
    assert(d.HasMember("Patches"));
    assert(d["Patches"].IsArray());

    int num_patches = d["Patches"].Size();

    // Create patches
    for (int i = 0; i < num_patches; i++) {
        LoadPatch(d["Patches"][i]);
    }
}

RigidTerrain::~RigidTerrain() {}

void RigidTerrain::LoadPatch(const rapidjson::Value& d) {
    assert(d.IsObject());
    assert(d.HasMember("Location"));
    assert(d.HasMember("Orientation"));
    assert(d.HasMember("Geometry"));
    assert(d.HasMember("Contact Material"));

    // Create patch with specified geometry and contact material
    std::shared_ptr<Patch> patch;
    auto loc = ReadVectorJSON(d["Location"]);
    auto rot = ReadQuaternionJSON(d["Orientation"]);

    // Create a default material (consistent with containing system) and overwrite properties
    assert(d.HasMember("Contact Material"));
    ChContactMaterialData minfo = ReadMaterialInfoJSON(d["Contact Material"]);
    auto material = minfo.CreateMaterial(m_system->GetContactMethod());

    // Create patch geometry (infer type based on existing keys)
    if (d["Geometry"].HasMember("Dimensions")) {
        auto size = ReadVectorJSON(d["Geometry"]["Dimensions"]);
        patch = AddPatch(material, ChCoordsys<>(loc, rot), size.x(), size.y(), size.z());
    } else if (d["Geometry"].HasMember("Mesh Filename")) {
        std::string mesh_file = d["Geometry"]["Mesh Filename"].GetString();
        bool connected_mesh = true;
        if (d["Geometry"].HasMember("Connected Mesh")) {
            connected_mesh = d["Geometry"]["Connected Mesh"].GetBool();
        }
        patch = AddPatch(material, ChCoordsys<>(loc, rot), GetVehicleDataFile(mesh_file), connected_mesh);
    } else if (d["Geometry"].HasMember("Height Map Filename")) {
        std::string bmp_file = d["Geometry"]["Height Map Filename"].GetString();
        double sx = d["Geometry"]["Size"][0u].GetDouble();
        double sy = d["Geometry"]["Size"][1u].GetDouble();
        double hMin = d["Geometry"]["Height Range"][0u].GetDouble();
        double hMax = d["Geometry"]["Height Range"][1u].GetDouble();
        bool connected_mesh = true;
        if (d["Geometry"].HasMember("Connected Mesh")) {
            connected_mesh = d["Geometry"]["Connected Mesh"].GetBool();
        }
        patch = AddPatch(material, ChCoordsys<>(loc, rot), GetVehicleDataFile(bmp_file), sx, sy, hMin, hMax,
                         connected_mesh);
    }

    // Set visualization data
    if (d.HasMember("Visualization")) {
        if (d["Visualization"].HasMember("Color")) {
            ChColor color = ReadColorJSON(d["Visualization"]["Color"]);
            patch->SetColor(color);
        }
        if (d["Visualization"].HasMember("Texture File")) {
            std::string tex_file = d["Visualization"]["Texture File"].GetString();
            float sx = 1;
            float sy = 1;
            if (d["Visualization"].HasMember("Texture Scaling")) {
                sx = d["Visualization"]["Texture Scaling"][0u].GetFloat();
                sy = d["Visualization"]["Texture Scaling"][1u].GetFloat();
            }
            patch->SetTexture(GetVehicleDataFile(tex_file), sx, sy);
        }
        patch->m_visualize = true;
    } else {
        patch->m_visualize = false;
    }
}

// -----------------------------------------------------------------------------

void RigidTerrain::AddPatch(std::shared_ptr<Patch> patch,
                            const ChCoordsys<>& position,
                            std::shared_ptr<ChContactMaterial> material) {
    m_num_patches++;

    // Create the rigid body for this patch (fixed)
    patch->m_body = chrono_types::make_shared<ChBody>();
    patch->m_body->SetName("patch_" + std::to_string(m_num_patches));
    patch->m_body->SetPos(position.pos);
    patch->m_body->SetRot(position.rot);
    patch->m_body->SetFixed(true);
    patch->m_body->EnableCollision(true);
    m_system->AddBody(patch->m_body);

    // Cache coefficient of friction
    patch->m_friction = material->GetStaticFriction();

    m_patches.push_back(patch);
}

void RigidTerrain::InitializePatch(std::shared_ptr<Patch> patch) {
    // Initialize the patch
    patch->Initialize();

    // All patches are added to the same collision family and collision with other models in this family is disabled
    patch->m_body->GetCollisionModel()->SetFamily(m_collision_family);
    patch->m_body->GetCollisionModel()->DisallowCollisionsWith(m_collision_family);
}

// -----------------------------------------------------------------------------
// Functions to add terrain patches with various definitions
// (box, mesh, height-field)
// -----------------------------------------------------------------------------

std::shared_ptr<RigidTerrain::Patch> RigidTerrain::AddPatch(std::shared_ptr<ChContactMaterial> material,
                                                            const ChCoordsys<>& position,
                                                            double length,
                                                            double width,
                                                            double thickness,
                                                            bool tiled,
                                                            double max_tile_size,
                                                            bool visualization) {
    auto patch = chrono_types::make_shared<BoxPatch>();
    AddPatch(patch, position, material);
    patch->m_visualize = visualization;

    // Create the collision model (one or more boxes) attached to the patch body
    if (tiled) {
        int nX = (int)std::ceil(length / max_tile_size);
        int nY = (int)std::ceil(width / max_tile_size);
        double sizeX1 = length / nX;
        double sizeY1 = width / nY;
        auto ct_shape = chrono_types::make_shared<ChCollisionShapeBox>(material, sizeX1, sizeY1, thickness);
        for (int ix = 0; ix < nX; ix++) {
            for (int iy = 0; iy < nY; iy++) {
                ChVector3d loc((sizeX1 - length) / 2 + ix * sizeX1,  //
                               (sizeY1 - width) / 2 + iy * sizeY1,   //
                               -0.5 * thickness);
                patch->m_body->AddCollisionShape(ct_shape, ChFrame<>(loc, QUNIT));
            }
        }
    } else {  // non-tiled terrain creation of a single collision box and centre based on worldframe detection
        auto ct_shape = chrono_types::make_shared<ChCollisionShapeBox>(material, length, width, thickness);
        ChVector3d loc(0, 0, -0.5 * thickness);
        patch->m_body->AddCollisionShape(ct_shape, ChFrame<>(loc, QUNIT));
    }

    // Cache patch parameters
    patch->m_location = position.pos;
    patch->m_normal = position.rot.GetAxisZ();
    patch->m_hlength = length / 2;
    patch->m_hwidth = width / 2;
    patch->m_hthickness = thickness / 2;
    patch->m_radius = ChVector3d(length, width, thickness).Length() / 2;
    patch->m_type = PatchType::BOX;

    return patch;
}

// -----------------------------------------------------------------------------

std::shared_ptr<RigidTerrain::Patch> RigidTerrain::AddPatch(std::shared_ptr<ChContactMaterial> material,
                                                            const ChCoordsys<>& position,
                                                            const std::string& mesh_file,
                                                            bool connected_mesh,
                                                            double sweep_sphere_radius,
                                                            bool visualization) {
    auto patch = chrono_types::make_shared<MeshPatch>();
    AddPatch(patch, position, material);
    patch->m_visualize = visualization;

    // Load mesh from file
    patch->m_trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(mesh_file, true, true);

    // Create the collision model
    if (connected_mesh) {
        auto ct_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(material, patch->m_trimesh, true, false,
                                                                                sweep_sphere_radius);
        patch->m_body->AddCollisionShape(ct_shape);
    } else {
        patch->m_trimesh_s = ChTriangleMeshSoup::CreateFromWavefrontFile(mesh_file);
        auto ct_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(material, patch->m_trimesh_s, true,
                                                                                false, sweep_sphere_radius);
        patch->m_body->AddCollisionShape(ct_shape);
    }

    auto mesh_name = filesystem::path(mesh_file).stem();

    // Cache patch parameters
    patch->m_radius =
        std::max_element(patch->m_trimesh->GetCoordsVertices().begin(),                                      //
                         patch->m_trimesh->GetCoordsVertices().end(),                                        //
                         [](const ChVector3d& a, const ChVector3d& b) { return a.Length2() < b.Length2(); }  //
                         )
            ->Length();

    patch->m_mesh_name = mesh_name;
    patch->m_type = PatchType::MESH;

    return patch;
}

// -----------------------------------------------------------------------------

std::shared_ptr<RigidTerrain::Patch> RigidTerrain::AddPatch(std::shared_ptr<ChContactMaterial> material,
                                                            const ChCoordsys<>& position,
                                                            const std::string& heightmap_file,
                                                            double length,
                                                            double width,
                                                            double hMin,
                                                            double hMax,
                                                            bool connected_mesh,
                                                            double sweep_sphere_radius,
                                                            bool visualization) {
    auto patch = chrono_types::make_shared<MeshPatch>();
    AddPatch(patch, position, material);
    patch->m_visualize = visualization;

    // Read the image file (request only 1 channel) and extract number of pixels
    STB hmap;
    if (!hmap.ReadFromFile(heightmap_file, 1)) {
        throw std::invalid_argument("Cannot open height map image file");
    }
    int nv_x = hmap.GetWidth();
    int nv_y = hmap.GetHeight();

    // Construct a triangular mesh of sizeX x sizeY (as specified in an ISO frame).
    // Each pixel in the BMP represents a vertex.
    // The gray level of a pixel is mapped to the height range, with black corresponding
    // to hMin and white corresponding to hMax.
    // UV coordinates are mapped in [0,1] x [0,1].
    // We use smoothed vertex normals.
    double dx = length / (nv_x - 1);
    double dy = width / (nv_y - 1);
    double h_scale = (hMax - hMin) / hmap.GetRange();
    double x_scale = 1.0 / (nv_x - 1);
    double y_scale = 1.0 / (nv_y - 1);
    unsigned int n_verts = nv_x * nv_y;
    unsigned int n_faces = 2 * (nv_x - 1) * (nv_y - 1);

    // Resize mesh arrays
    patch->m_trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    patch->m_trimesh->GetCoordsVertices().resize(n_verts);
    patch->m_trimesh->GetCoordsNormals().resize(n_verts);
    patch->m_trimesh->GetCoordsUV().resize(n_verts);
    patch->m_trimesh->GetCoordsColors().resize(n_verts);

    patch->m_trimesh->GetIndicesVertexes().resize(n_faces);
    patch->m_trimesh->GetIndicesNormals().resize(n_faces);
    patch->m_trimesh->GetIndicesUV().resize(n_faces);

    // Initialize the array of accumulators (number of adjacent faces to a vertex)
    std::vector<int> accumulators(n_verts, 0);

    // Readability aliases
    std::vector<ChVector3d>& vertices = patch->m_trimesh->GetCoordsVertices();
    std::vector<ChVector3d>& normals = patch->m_trimesh->GetCoordsNormals();
    std::vector<ChColor>& colors = patch->m_trimesh->GetCoordsColors();
    std::vector<ChVector2d>& uvs = patch->m_trimesh->GetCoordsUV();
    std::vector<ChVector3i>& idx_vertices = patch->m_trimesh->GetIndicesVertexes();
    std::vector<ChVector3i>& idx_normals = patch->m_trimesh->GetIndicesNormals();
    std::vector<ChVector3i>& idx_uvs = patch->m_trimesh->GetIndicesUV();

    // Load mesh vertices.
    // Note that pixels in a BMP start at top-left corner.
    // We order the vertices starting at the bottom-left corner, row after row.
    // The bottom-left corner corresponds to the point (-sizeX/2, -sizeY/2).
    unsigned int iv = 0;
    for (int iy = nv_y - 1; iy >= 0; --iy) {
        double y = 0.5 * width - iy * dy;
        for (int ix = 0; ix < nv_x; ++ix) {
            double x = ix * dx - 0.5 * length;
            // Map gray level to vertex height
            double z = hMin + hmap.Gray(ix, iy) * h_scale;
            // Set vertex location
            vertices[iv] = ChWorldFrame::FromISO(ChVector3d(x, y, z));
            // Initialize vertex normal to (0, 0, 0).
            normals[iv] = ChVector3d(0, 0, 0);
            // Assign color white to all vertices
            colors[iv] = ChColor(1, 1, 1);
            // Set UV coordinates in [0,1] x [0,1]
            uvs[iv] = ChVector2d(ix * x_scale, iy * y_scale);
            ++iv;
        }
    }

    // Specify triangular faces (two at a time).
    // Specify the face vertices counter-clockwise.
    // Set the normal indices same as the vertex indices.
    unsigned int it = 0;
    for (int iy = nv_y - 2; iy >= 0; --iy) {
        for (int ix = 0; ix < nv_x - 1; ++ix) {
            int v0 = ix + nv_x * iy;
            idx_vertices[it] = ChVector3i(v0, v0 + nv_x + 1, v0 + nv_x);
            idx_normals[it] = ChVector3i(v0, v0 + nv_x + 1, v0 + nv_x);
            idx_uvs[it] = ChVector3i(v0, v0 + nv_x + 1, v0 + nv_x);
            ++it;
            idx_vertices[it] = ChVector3i(v0, v0 + 1, v0 + nv_x + 1);
            idx_normals[it] = ChVector3i(v0, v0 + 1, v0 + nv_x + 1);
            idx_uvs[it] = ChVector3i(v0, v0 + 1, v0 + nv_x + 1);
            ++it;
        }
    }

    // Calculate normals and then average the normals from all adjacent faces
    for (it = 0; it < n_faces; ++it) {
        // Calculate the triangle normal as a normalized cross product.
        ChVector3d nrm = Vcross(vertices[idx_vertices[it][1]] - vertices[idx_vertices[it][0]],
                                vertices[idx_vertices[it][2]] - vertices[idx_vertices[it][0]]);
        nrm.Normalize();
        // Increment the normals of all incident vertices by the face normal
        normals[idx_normals[it][0]] += nrm;
        normals[idx_normals[it][1]] += nrm;
        normals[idx_normals[it][2]] += nrm;
        // Increment the count of all incident vertices by 1
        accumulators[idx_normals[it][0]] += 1;
        accumulators[idx_normals[it][1]] += 1;
        accumulators[idx_normals[it][2]] += 1;
    }

    // Set the normals to the average values
    for (unsigned int in = 0; in < n_verts; ++in) {
        normals[in] = ChWorldFrame::FromISO(normals[in] / (double)accumulators[in]);
    }

    // Create contact geometry
    if (connected_mesh) {
        auto ct_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(material, patch->m_trimesh, true, false,
                                                                                sweep_sphere_radius);
        patch->m_body->AddCollisionShape(ct_shape);
    } else {
        patch->m_trimesh_s = chrono_types::make_shared<ChTriangleMeshSoup>();
        std::vector<ChTriangle>& triangles = patch->m_trimesh_s->GetTriangles();
        triangles.resize(n_faces);
        for (it = 0; it < n_faces; it++) {
            const ChVector3i& idx = idx_vertices[it];
            triangles[it] = ChTriangle(vertices[idx[0]], vertices[idx[1]], vertices[idx[2]]);
        }
        auto ct_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(material, patch->m_trimesh_s, true,
                                                                                false, sweep_sphere_radius);
        patch->m_body->AddCollisionShape(ct_shape);
    }

    auto mesh_name = filesystem::path(heightmap_file).stem();

    // Cache patch parameters
    patch->m_radius = ChVector3d(length, width, (hMax - hMin)).Length() / 2;
    patch->m_mesh_name = mesh_name;
    patch->m_type = PatchType::HEIGHT_MAP;

    return patch;
}

//----------------------------------------------------------------------------------------------
// Create a 'heightmap' from a group of points by weighted averaging of the nearest the vectors
// within x,y grid with z height. Grid resolution can be increased or decreased and so can the
// number of LEPP iterations and the factor of smoothing (dependent on grid resolution).
// Acts in a way like a pixelated grid with smoothing. Grid x and y axis align with the standard
// RHF world axis.

std::shared_ptr<RigidTerrain::Patch> RigidTerrain::AddPatch(std::shared_ptr<ChContactMaterial> material,
                                                            const ChCoordsys<>& position,
                                                            const std::vector<ChVector3d>& point_cloud,
                                                            double length,
                                                            double width,
                                                            int unrefined_resolution,
                                                            int heightmap_resolution,
                                                            int max_refinements,
                                                            double refine_angle_limit,
                                                            double smoothing_factor,
                                                            double max_edge_length,
                                                            double sweep_sphere_radius,
                                                            bool visualization) {
    //---------------
    // Initialisation
    //---------------

    auto patch = chrono_types::make_shared<MeshPatch>();
    AddPatch(patch, position, material);
    patch->m_visualize = visualization;

    // Initialise the mesh
    patch->m_trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();

    // Calculate coarse grid resolution. Ensure not less than heightmap resolution
    const int coarse_grid_resolution = std::min(heightmap_resolution, unrefined_resolution);

    // Note: typically this method works best with a 1:1 aspect ratio, but it can work with rectangular-type
    // grid cells, and elongated triangles, however, it might run into trouble using RefineMeshEdges if the
    // aspect ratio is too extreme.
    //
    // Adjusted cell sizes to fit within the terrain dimensions
    const double coarse_cell_length = length / coarse_grid_resolution;
    const double coarse_cell_width = width / coarse_grid_resolution;

    // Calculate the number of vertices for the coarse
    const int n_verts_across_down = coarse_grid_resolution + 1;
    int n_verts = n_verts_across_down * n_verts_across_down;

    // Resize vertices, normals, and UVs
    patch->m_trimesh->GetCoordsVertices().resize(n_verts);
    patch->m_trimesh->GetCoordsNormals().resize(n_verts);
    patch->m_trimesh->GetCoordsUV().resize(n_verts);

    //---------------------
    // Refinement variables
    //---------------------

    // Individual cell sizes for each grid
    const double cell_length = length / heightmap_resolution;  // in the x
    const double cell_width = width / heightmap_resolution;    // in the y

    // initialisations for when calling RefineMeshEdges
    std::vector<std::vector<double>*> aux_data_double;
    std::vector<std::vector<int>*> aux_data_int;
    std::vector<std::vector<bool>*> aux_data_bool;
    std::vector<std::vector<ChVector3d>*> aux_data_vect;

    // point cloud/grid variables
    std::vector<std::vector<double>> height_map(heightmap_resolution, std::vector<double>(heightmap_resolution, 0.0));
    std::vector<std::vector<int>> count_map(heightmap_resolution, std::vector<int>(heightmap_resolution, 0));
    const double influence_distance = std::max(cell_length, cell_width);  // Set to the larger of cell size dimensions
    std::vector<std::vector<double>> max_height_map(
        heightmap_resolution, std::vector<double>(heightmap_resolution, std::numeric_limits<double>::lowest()));
    std::vector<std::vector<double>> min_height_map(
        heightmap_resolution, std::vector<double>(heightmap_resolution, std::numeric_limits<double>::max()));

    // Refinement process
    int current_iteration = 0;
    max_refinements = std::max(0, max_refinements);
    bool execute_refine;
    double angle_threshold = 4 * CH_DEG_TO_RAD;
    double horizontal_threshold = std::cos(angle_threshold);
    double vertical_threshold = std::sin(angle_threshold);
    ChVector3d vertical_vector(0, 0, 1);
    std::map<std::pair<int, int>, std::pair<int, int>> winged_edges;  // map for winged edges checking (in normals)

    // Smoothing / postprocessing parameters
    double lambda = 0.63 * smoothing_factor;
    double mu = -0.6300001 * smoothing_factor;  // the absolute of mu must always be greater than lambda
    // Define the boundaries of the mesh
    double boundary_min_x = -length / 2;
    double boundary_max_x = length / 2;
    double boundary_min_y = -width / 2;
    double boundary_max_y = width / 2;
    double edge_threshold = 1e-6;  // threshold distance from the boundary (2D), to not move on x-y in smoothing

    // Patch parameters
    double height_max = -std::numeric_limits<double>::infinity();
    double height_min = std::numeric_limits<double>::infinity();
    for (const auto& vec : point_cloud) {
        double h = vec.z();
        height_max = std::max(height_max, h);
        height_min = std::min(height_min, h);
    }

    //--------------------------------------------------------
    // Stage 1: Point cloud processing and grid initialisation
    //--------------------------------------------------------

    // Populate fine height map directly from point cloud.
    // point cloud is entered in the sense of the x-y grid and z value of the vector is the height
    for (const auto& point : point_cloud) {
        // set the cell indices
        int cell_origin_x = static_cast<int>((point.x() + length / 2) / cell_length);
        int cell_origin_y = static_cast<int>((point.y() + width / 2) / cell_width);
        // Update height_max and height_min
        height_max = std::max(height_max, point.z());
        height_min = std::min(height_min, point.z());
        // run the 3x3 loop of neighbours to consider weighted averaging of heights
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                int nx = cell_origin_x + dx;
                int ny = cell_origin_y + dy;
                // keep within the bounds of the grid cells
                if (nx >= 0 && nx < heightmap_resolution && ny >= 0 && ny < heightmap_resolution) {
                    double cell_central_x = (nx + 0.5) * cell_length - length / 2;
                    double cell_central_y = (ny + 0.5) * cell_width - width / 2;
                    double dist_sq = (cell_central_x - point.x()) * (cell_central_x - point.x()) +
                                     (cell_central_y - point.y()) * (cell_central_y - point.y());
                    // check the distance and influence, add the count and record the max and min of this cell for
                    // clamping later
                    if (dist_sq < influence_distance * influence_distance) {
                        height_map[nx][ny] += point.z();
                        count_map[nx][ny]++;
                        max_height_map[nx][ny] = std::max(max_height_map[nx][ny], point.z());
                        min_height_map[nx][ny] = std::min(min_height_map[nx][ny], point.z());
                    }
                }
            }
        }
    }

    // Normalise, assign and clamp heights (ensure the averaged heights aren't excessively influencing magnitude beyond
    // what's already max from point cloud)
    for (int i = 0; i < heightmap_resolution; ++i) {
        for (int j = 0; j < heightmap_resolution; ++j) {
            if (count_map[i][j] > 0) {
                height_map[i][j] /= count_map[i][j];
                height_map[i][j] = std::clamp(height_map[i][j], min_height_map[i][j], max_height_map[i][j]);
            }
        }
    }

    // Initialise the coarse height map
    // Iterate over the grid and assign heights to mesh vertices - heights are directly assigned from the cells of
    // the height_map (rather than re-average). Bilinear height assignment is given in the refinement process
    for (int i = 0; i < n_verts_across_down; ++i) {
        for (int j = 0; j < n_verts_across_down; ++j) {
            double x = i * coarse_cell_length - length / 2;
            double y = j * coarse_cell_width - width / 2;

            // Calculate the corresponding indices in the height map
            // ensuring correct alignment with the height map
            double relative_x = x + length / 2;  // position relative to heightmap's origin
            double relative_y = y + width / 2;   // position relative to heightmap's origin
            int cellX = static_cast<int>(relative_x / (length / heightmap_resolution));
            int cellY = static_cast<int>(relative_y / (width / heightmap_resolution));

            // Clamp to the bounds of the height map
            cellX = std::clamp(cellX, 0, heightmap_resolution - 1);
            cellY = std::clamp(cellY, 0, heightmap_resolution - 1);

            // Apply the direct height from the height map and generate the vertex
            double z = height_map[cellX][cellY];
            patch->m_trimesh->GetCoordsVertices()[i * n_verts_across_down + j] = ChVector3d(x, y, z);
        }
    }

    // Generate alternating triangles for the coarse mesh
    for (int i = 0; i < coarse_grid_resolution; ++i) {
        for (int j = 0; j < coarse_grid_resolution; ++j) {
            unsigned int v0 = i * n_verts_across_down + j;
            unsigned int v1 = v0 + 1;
            unsigned int v2 = v0 + n_verts_across_down;
            unsigned int v3 = v2 + 1;
            // Alternating triangles for the RHF - i.e. counter-clockwise vertices
            if ((i + j) % 2 == 0) {
                patch->m_trimesh->GetIndicesVertexes().push_back(ChVector3i(v0, v2, v1));
                patch->m_trimesh->GetIndicesVertexes().push_back(ChVector3i(v1, v2, v3));
            } else {
                patch->m_trimesh->GetIndicesVertexes().push_back(ChVector3i(v0, v2, v3));
                patch->m_trimesh->GetIndicesVertexes().push_back(ChVector3i(v0, v3, v1));
            }
        }
    }
    std::cout << "Terrain patch no. of coarse triangles generated:" << patch->m_trimesh->GetNumTriangles() << std::endl;

    //--------------------------------------------------
    // Stage 2: iterative refinement in a two stage pass
    //--------------------------------------------------
    std::vector<int> marked_tris;  // Container for marked triangles
    marked_tris.clear();           // clear the group prior to iteration

    // ITERATIVE LOOP
    // This makes use of the fine grid with normal-angle refinement
    // Remember this is working on the mesh in a Zup context (regardless of the worldframe),
    // and will be rotated to y-up later
    while (current_iteration < max_refinements) {
        execute_refine = false;

        // Normal checking and clearing if near vertical or flat
        auto& vertices = patch->m_trimesh->GetCoordsVertices();  // get the vertices of the mesh
        patch->m_trimesh->ComputeWingedEdges(winged_edges);      // compute the winged edge map

        // Compute the triangle connectivity map
        std::vector<std::array<int, 4>> tri_map;

        /* bool check_pathological = */ patch->m_trimesh->ComputeNeighbouringTriangleMap(tri_map);
        ////if (!check_pathological) break; // TODO: ensure the return is correct from ChTriangleMeshConnected.cpp

        for (const auto& edge : winged_edges) {
            int tri1_index = edge.second.first;
            int tri2_index = edge.second.second;
            // Skip if no adjacent triangle (boundary edge) or indices are out of bounds
            if (tri1_index >= tri_map.size() || tri2_index >= tri_map.size())
                continue;
            // Also skip if the neighboring triangles are boundary triangles (-1 in tri_map)
            if (tri_map[tri1_index][1] == -1 || tri_map[tri1_index][2] == -1 || tri_map[tri1_index][3] == -1 ||
                tri_map[tri2_index][1] == -1 || tri_map[tri2_index][2] == -1 || tri_map[tri2_index][3] == -1)
                continue;

            // get the vertices
            const auto& tri1 = patch->m_trimesh->GetIndicesVertexes()[tri1_index];
            const auto& tri2 = patch->m_trimesh->GetIndicesVertexes()[tri2_index];
            // Calculate normals for each triangle
            ChVector3d normal1 =
                Vcross(vertices[tri1.y()] - vertices[tri1.x()], vertices[tri1.z()] - vertices[tri1.x()])
                    .GetNormalized();
            ChVector3d normal2 =
                Vcross(vertices[tri2.y()] - vertices[tri2.x()], vertices[tri2.z()] - vertices[tri2.x()])
                    .GetNormalized();

            // Skip triangles that are close to horizontal or close to vertical
            auto dot1 = std::abs(normal1 ^ vertical_vector);
            auto dot2 = std::abs(normal2 ^ vertical_vector);
            if ((dot1 < vertical_threshold && dot1 > horizontal_threshold) ||
                (dot2 < vertical_threshold && dot2 > horizontal_threshold)) {
                continue;
            }

            // Calculate the angle between normals
            double dot_product = Vdot(normal1, normal2);
            double angle = std::acos(dot_product) * CH_RAD_TO_DEG;  // convert the angle
            if (angle > refine_angle_limit) {                       // Mark both triangles for refinement
                marked_tris.emplace_back(tri1_index);
                marked_tris.emplace_back(tri2_index);
                execute_refine = true;
            }
        }

        //  Refine all marked triangles and set heights
        if (execute_refine) {  // Refine marked triangles
            // use Chrono's LEPP refine mesh edges method to increase triangle resolution of marked triangles
            patch->m_trimesh->RefineMeshEdges(marked_tris, max_edge_length, 0, &tri_map, aux_data_double, aux_data_int,
                                              aux_data_bool, aux_data_vect);
            // std::cout << "refinement done. bilinear filtering" << std::endl;

            // set height of vertices with bilinear interpolation
            for (auto& vertex : patch->m_trimesh->GetCoordsVertices()) {
                int cell_x = static_cast<int>((vertex.x() + length / 2) / cell_length);
                int cell_y = static_cast<int>((vertex.y() + width / 2) / cell_width);

                // Ensure cell indices are within bounds
                cell_x = std::max(0, std::min(cell_x, heightmap_resolution - 1));
                cell_y = std::max(0, std::min(cell_y, heightmap_resolution - 1));

                // Calculate fractional parts for bilinear interpolation
                double fx = (vertex.x() + length / 2) / cell_length - cell_x;
                double fy = (vertex.y() + width / 2) / cell_width - cell_y;

                // Ensure the next cell indices are also within bounds
                int next_cell_x = std::min(cell_x + 1, heightmap_resolution - 1);
                int next_cell_y = std::min(cell_y + 1, heightmap_resolution - 1);

                // Apply bilinear interpolation
                vertex.z() =
                    height_map[cell_x][cell_y] * (1 - fx) * (1 - fy) + height_map[next_cell_x][cell_y] * fx * (1 - fy) +
                    height_map[cell_x][next_cell_y] * (1 - fx) * fy + height_map[next_cell_x][next_cell_y] * fx * fy;
            }
        }  // End refinement

        // Provide update to console
        std::cout << "Terrain Patch iterative refinement number: " << current_iteration << std::endl;
        std::cout << "Number of triangles refined: " << marked_tris.size() << std::endl;

        ++current_iteration;  // counter
    }

    //---------------------------------------------------------------------------
    // Mesh healing if necessary and post processing with Taubin smoothing method
    //---------------------------------------------------------------------------

    // Repair duplicate vertices
    ////int merged_vertices = patch->m_trimesh->RepairDuplicateVertexes(1e-18);
    ////std::cout << "Number of merged vertices: " << merged_vertices << std::endl;

    // Taubin smoothing approach, controllable with smoothing factor 0-1
    // Note: Does not set height - this is done in the iterative loop
    // Ensure latest values used
    auto& vertices = patch->m_trimesh->GetCoordsVertices();
    auto& triangles = patch->m_trimesh->GetIndicesVertexes();
    n_verts = patch->m_trimesh->GetNumVertices();  // update the vertices again
    // Smoothing loop
    if (smoothing_factor != 0) {  // skip if smoothing is zero
        // Create an adjacency list
        std::vector<std::unordered_set<int>> adjacency_list(n_verts);
        for (const auto& triangle : triangles) {
            for (int i = 0; i < 3; ++i) {
                int vertex_index = triangle[i];
                for (int j = 0; j < 3; ++j) {
                    if (i != j) {
                        adjacency_list[vertex_index].insert(triangle[j]);
                    }
                }
            }
        }
        // push-pull smoothing
        std::vector<ChVector3d> temp_smoothing_vertices(n_verts);  // up to date vector
        // Iterations of smoothing loop
        for (int iter = 0; iter < 10; ++iter) {  // number of iterations. 10 is usually enough
            // Laplacian smoothing step
            for (int i = 0; i < n_verts; ++i) {
                // Check if its along the boundaries. We don't want to pull them in or push them out
                bool is_edge_vertex = vertices[i].x() <= boundary_min_x + edge_threshold ||
                                      vertices[i].x() >= boundary_max_x - edge_threshold ||
                                      vertices[i].y() <= boundary_min_y + edge_threshold ||
                                      vertices[i].y() >= boundary_max_y - edge_threshold;

                ChVector3d laplacian_vector(0, 0, 0);
                for (int adjVertex : adjacency_list[i]) {
                    laplacian_vector += (vertices[adjVertex] - vertices[i]);  // consider adjacency
                }
                temp_smoothing_vertices[i] = vertices[i] + lambda * laplacian_vector / adjacency_list[i].size();
                if (is_edge_vertex) {
                    temp_smoothing_vertices[i].x() = vertices[i].x();  // keep original x value
                    temp_smoothing_vertices[i].y() = vertices[i].y();  // keep original y coord
                }
            }
            // run the inverse laplacian smoothing step
            for (int i = 0; i < n_verts; ++i) {
                bool is_edge_vertex =  // check and mark the boundary vertices
                    vertices[i].x() <= boundary_min_x + edge_threshold ||
                    vertices[i].x() >= boundary_max_x - edge_threshold ||
                    vertices[i].y() <= boundary_min_y + edge_threshold ||
                    vertices[i].y() >= boundary_max_y - edge_threshold;

                ChVector3d laplacian_vector(0, 0, 0);
                for (int adj_vertex : adjacency_list[i]) {
                    laplacian_vector +=
                        (temp_smoothing_vertices[adj_vertex] - temp_smoothing_vertices[i]);  // consider original
                }
                vertices[i] = temp_smoothing_vertices[i] + mu * laplacian_vector / adjacency_list[i].size();
                if (is_edge_vertex) {
                    vertices[i].x() = temp_smoothing_vertices[i].x();  // keep original x coord
                    vertices[i].y() = temp_smoothing_vertices[i].y();  // keep original y coord
                }
            }
        }
        // clean up the lists
        adjacency_list.clear();
        temp_smoothing_vertices.clear();
    }

    // Clean up buffers & memory allocations and other structures used for refinement
    for (auto& buffer : aux_data_double) {
        buffer->clear();
    }
    for (auto& buffer : aux_data_int) {
        buffer->clear();
    }
    for (auto& buffer : aux_data_bool) {
        buffer->clear();
    }
    for (auto& buffer : aux_data_vect) {
        buffer->clear();
    }

    height_map.clear();
    count_map.clear();
    max_height_map.clear();
    min_height_map.clear();

    // ---------------------------------------
    // Final setup, collision body and caching
    // ---------------------------------------

    // Set colour of all vertices to white
    for (auto& vertex_color : patch->m_trimesh->GetCoordsColors()) {
        vertex_color = ChColor(1, 1, 1);
    }
    // No UV mapping as terrain is simply a single colour

    // Build the connected mesh
    auto ct_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(material, patch->m_trimesh, true, false,
                                                                            sweep_sphere_radius);
    // Add collision shape
    patch->m_body->AddCollisionShape(ct_shape);

    patch->m_radius = ChVector3d(length, width, (height_max - height_min)).Length() / 2;
    patch->m_type = PatchType::MESH;

    return patch;
}

// -----------------------------------------------------------------------------
// Functions to modify properties of a patch
// -----------------------------------------------------------------------------

RigidTerrain::Patch::Patch() : m_friction(0.8f), m_visualize(true) {
    m_vis_mat = std::make_shared<ChVisualMaterial>(*ChVisualMaterial::Default());
}

void RigidTerrain::Patch::SetColor(const ChColor& color) {
    m_vis_mat->SetDiffuseColor({color.R, color.G, color.B});
}

void RigidTerrain::Patch::SetTexture(const std::string& filename, float scale_x, float scale_y) {
    m_vis_mat->SetKdTexture(filename);
    m_vis_mat->SetTextureScale(scale_x, scale_y);
}

// -----------------------------------------------------------------------------
// Initialize all terrain patches
// -----------------------------------------------------------------------------
class RTContactCallback : public ChContactContainer::AddContactCallback {
  public:
    virtual void OnAddContact(const ChCollisionInfo& contactinfo, ChContactMaterialComposite* const material) override {
        //// TODO: also accommodate terrain contact with FEA meshes.

        // Loop over all patch bodies and check if this contact involves one of them.
        ChBody* body_patch = nullptr;
        ChBody* body_other = nullptr;
        ChCollisionShape* shape_other = nullptr;
        for (auto patch : m_terrain->GetPatches()) {
            auto model = patch->GetGroundBody()->GetCollisionModel().get();
            if (model == contactinfo.modelA) {
                body_patch = patch->GetGroundBody().get();
                body_other = dynamic_cast<ChBody*>(contactinfo.modelB->GetContactable());
                shape_other = contactinfo.shapeB;
                break;
            }
            if (model == contactinfo.modelB) {
                body_patch = patch->GetGroundBody().get();
                body_other = dynamic_cast<ChBody*>(contactinfo.modelA->GetContactable());
                shape_other = contactinfo.shapeA;
                break;
            }
        }

        // Do nothing if this contact does not involve a terrain body or if the other contactable
        // is not a body or if the collision does not involve a shape (e.g., a contact added by the user)
        if (!body_patch || !body_other || !shape_other)
            return;

        // Containing system and current combination strategy for composite materials
        auto sys = body_patch->GetSystem();
        auto& strategy = sys->GetMaterialCompositionStrategy();

        // Find the terrain coefficient of friction at the location of current contact.
        // Arbitrarily use the collision point on modelA.
        auto friction_terrain = (*m_friction_fun)(contactinfo.vpA);

        // Set friction in composite material based on contact formulation.
        auto friction_other = shape_other->GetMaterial()->sliding_friction;
        auto friction = strategy.CombineFriction(friction_terrain, friction_other);
        switch (sys->GetContactMethod()) {
            case ChContactMethod::NSC: {
                auto mat = static_cast<ChContactMaterialCompositeNSC* const>(material);
                mat->static_friction = friction;
                mat->sliding_friction = friction;
                break;
            }
            case ChContactMethod::SMC: {
                auto mat = static_cast<ChContactMaterialCompositeSMC* const>(material);
                mat->mu_eff = friction;
                break;
            }
        }
    }

    std::shared_ptr<ChTerrain::FrictionFunctor> m_friction_fun;
    RigidTerrain* m_terrain;
};

void RigidTerrain::Initialize() {
    if (m_initialized)
        return;

    if (m_patches.empty())
        return;

    for (auto& patch : m_patches) {
        InitializePatch(patch);
    }

    m_initialized = true;

    if (!m_friction_fun)
        m_use_friction_functor = false;

    if (!m_use_friction_functor)
        return;

    // Create and register a custom callback functor of type ChContactContainer::AddContactCallback
    // and pass to it a list of patch bodies as well as the location-dependent friction functor.
    auto callback = chrono_types::make_shared<RTContactCallback>();
    callback->m_terrain = this;
    callback->m_friction_fun = m_friction_fun;
    m_contact_callback = callback;
    m_system->GetContactContainer()->RegisterAddContactCallback(m_contact_callback);
}

void RigidTerrain::BoxPatch::Initialize() {
    if (m_visualize) {
        m_body->AddVisualModel(chrono_types::make_shared<ChVisualModel>());
        auto box = chrono_types::make_shared<ChVisualShapeBox>(2 * m_hlength, 2 * m_hwidth, 2 * m_hthickness);
        box->AddMaterial(m_vis_mat);
        m_body->AddVisualShape(box, ChFrame<>(ChVector3d(0, 0, -m_hthickness)));
    }
}

void RigidTerrain::MeshPatch::Initialize() {
    if (m_visualize) {
        m_body->AddVisualModel(chrono_types::make_shared<ChVisualModel>());
        auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        trimesh_shape->AddMaterial(m_vis_mat);
        trimesh_shape->SetName(m_mesh_name);
        trimesh_shape->SetMesh(m_trimesh, true);
        m_body->AddVisualShape(trimesh_shape);
    }
}

// -----------------------------------------------------------------------------

void RigidTerrain::BindPatch(std::shared_ptr<Patch> patch) {
    // This function should be called only for patches that are added after the RigidTerrain was initialized
    if (!m_initialized) {
        std::cerr << "RigidTerrain::BindPatch should be explicitly called only for patches added ";
        std::cerr << "*after* the terrain was initialized." << std::endl;
        return;
    }

    // Initialize the patch
    InitializePatch(patch);

    // Bind the patch visual assets to the visual system (if present)
    if (m_system->GetVisualSystem())
        m_system->GetVisualSystem()->BindItem(patch->m_body);

    // Bind the patch collision model to the collision system (if present)
    if (m_system->GetCollisionSystem())
        m_system->GetCollisionSystem()->BindItem(patch->m_body);
}

void RigidTerrain::RemovePatch(std::shared_ptr<Patch> patch) {
    auto pos = std::find(m_patches.begin(), m_patches.end(), patch);
    if (pos != m_patches.end()) {
        // Unbind the patch from the visualization and collision systems (if they exists)
        if (m_system->GetVisualSystem())
            m_system->GetVisualSystem()->UnbindItem((*pos)->m_body);
        if (m_system->GetCollisionSystem())
            m_system->GetCollisionSystem()->UnbindItem((*pos)->m_body);

        // Erase from the list of patches
        m_patches.erase(pos);
        m_num_patches--;
    }
}

// -----------------------------------------------------------------------------
// Functions for obtaining the terrain height, normal, and coefficient of
// friction  at the specified location.
// This is done by casting vertical rays into each patch collision model.
// -----------------------------------------------------------------------------
double RigidTerrain::GetHeight(const ChVector3d& loc) const {
    if (m_height_fun)
        return (*m_height_fun)(loc);

    double height;
    ChVector3d normal;
    float friction;

    bool hit = FindPoint(loc, height, normal, friction);

    return hit ? height : 0.0;
}

ChVector3d RigidTerrain::GetNormal(const ChVector3d& loc) const {
    if (m_normal_fun)
        return (*m_normal_fun)(loc);

    double height;
    ChVector3d normal;
    float friction;

    bool hit = FindPoint(loc, height, normal, friction);

    return hit ? normal : ChWorldFrame::Vertical();
}

float RigidTerrain::GetCoefficientFriction(const ChVector3d& loc) const {
    if (m_friction_fun)
        return (*m_friction_fun)(loc);

    double height;
    ChVector3d normal;
    float friction;

    bool hit = FindPoint(loc, height, normal, friction);

    return hit ? friction : 0.8f;
}

void RigidTerrain::GetProperties(const ChVector3d& loc, double& height, ChVector3d& normal, float& friction) const {
    if (m_height_fun && m_normal_fun && m_friction_fun) {
        height = (*m_height_fun)(loc);
        normal = (*m_normal_fun)(loc);
        friction = (*m_friction_fun)(loc);
        return;
    }

    bool hit = FindPoint(loc, height, normal, friction);
    if (!hit) {
        height = 0;
        normal = ChWorldFrame::Vertical();
        friction = 0.8f;
    }

    if (m_height_fun)
        height = (*m_height_fun)(loc);

    if (m_normal_fun)
        normal = (*m_normal_fun)(loc);

    if (m_friction_fun)
        friction = (*m_friction_fun)(loc);
}

bool RigidTerrain::FindPoint(const ChVector3d loc, double& height, ChVector3d& normal, float& friction) const {
    bool hit = false;
    height = std::numeric_limits<double>::lowest();
    normal = ChWorldFrame::Vertical();
    friction = 0.8f;

    for (auto patch : m_patches) {
        double pheight;
        ChVector3d pnormal;
        bool phit = patch->FindPoint(loc, pheight, pnormal);
        if (phit && pheight > height) {
            hit = true;
            height = pheight;
            normal = pnormal;
            friction = patch->m_friction;
        }
    }

    return hit;
}

bool RigidTerrain::BoxPatch::FindPoint(const ChVector3d& loc, double& height, ChVector3d& normal) const {
    // Ray definition (in global frame)
    ChVector3d A = loc;                        // start point
    ChVector3d v = -ChWorldFrame::Vertical();  // direction (downward)

    // Intersect ray with top plane
    double t = Vdot(m_location - A, m_normal) / Vdot(v, m_normal);
    ChVector3d C = A + t * v;
    height = ChWorldFrame::Height(C);
    normal = m_normal;

    // Check bounds
    ChVector3d Cl = m_body->TransformPointParentToLocal(C);
    return std::abs(Cl.x()) <= m_hlength && std::abs(Cl.y()) <= m_hwidth;
}

bool RigidTerrain::MeshPatch::FindPoint(const ChVector3d& loc, double& height, ChVector3d& normal) const {
    ChVector3d from = loc;
    ChVector3d to = loc - (m_radius + 1000) * ChWorldFrame::Vertical();

    ChCollisionSystem::ChRayhitResult result;
    m_body->GetSystem()->GetCollisionSystem()->RayHit(from, to, m_body->GetCollisionModel().get(), result);
    height = ChWorldFrame::Height(result.abs_hitPoint);
    normal = result.abs_hitNormal;

    return result.hit;
}

// -----------------------------------------------------------------------------
// Export all patch meshes
// -----------------------------------------------------------------------------
void RigidTerrain::ExportMeshPovray(const std::string& out_dir, bool smoothed) {
    for (auto patch : m_patches) {
        patch->ExportMeshPovray(out_dir, smoothed);
    }
}

void RigidTerrain::ExportMeshWavefront(const std::string& out_dir) {
    for (auto patch : m_patches) {
        patch->ExportMeshWavefront(out_dir);
    }
}

void RigidTerrain::MeshPatch::ExportMeshPovray(const std::string& out_dir, bool smoothed) {
    utils::WriteMeshPovray(*m_trimesh, m_mesh_name, out_dir, ChColor(1, 1, 1), ChVector3d(0, 0, 0),
                           ChQuaternion<>(1, 0, 0, 0), smoothed);
}

void RigidTerrain::MeshPatch::ExportMeshWavefront(const std::string& out_dir) {
    std::string obj_filename = out_dir + "/" + m_mesh_name + ".obj";
    std::vector<ChTriangleMeshConnected> meshes = {*m_trimesh};
    std::cout << "Exporting to " << obj_filename << std::endl;
    m_trimesh->WriteWavefront(obj_filename, meshes);
}

}  // end namespace vehicle
}  // end namespace chrono
