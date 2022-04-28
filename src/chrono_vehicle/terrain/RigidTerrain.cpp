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

#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/physics/ChMaterialSurfaceNSC.h"
#include "chrono/physics/ChMaterialSurfaceSMC.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
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
      m_collision_family(14) {}

// -----------------------------------------------------------------------------
// Constructor from JSON file
// -----------------------------------------------------------------------------
RigidTerrain::RigidTerrain(ChSystem* system, const std::string& filename)
    : m_system(system),
      m_num_patches(0),
      m_use_friction_functor(false),
      m_contact_callback(nullptr),
      m_collision_family(14) {
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
    MaterialInfo minfo = ReadMaterialInfoJSON(d["Contact Material"]);
    auto material = minfo.CreateMaterial(m_system->GetContactMethod());

    // Create patch geometry (infer type based on existing keys)
    if (d["Geometry"].HasMember("Dimensions")) {
        auto size = ReadVectorJSON(d["Geometry"]["Dimensions"]);
        patch = AddPatch(material, loc, ChMatrix33<>(rot).Get_A_Zaxis(), size.x(), size.y(), size.z());
    } else if (d["Geometry"].HasMember("Mesh Filename")) {
        std::string mesh_file = d["Geometry"]["Mesh Filename"].GetString();
        bool connected_mesh = true;
        if (d["Geometry"].HasMember("Connected Mesh")) {
            connected_mesh = d["Geometry"]["Connected Mesh"].GetBool();
        }
        patch = AddPatch(material, ChCoordsys<>(loc, rot), vehicle::GetDataFile(mesh_file), connected_mesh);
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
        patch = AddPatch(material, ChCoordsys<>(loc, rot), vehicle::GetDataFile(bmp_file), sx, sy, hMin, hMax,
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
            patch->SetTexture(vehicle::GetDataFile(tex_file), sx, sy);
        }
    }
}

// -----------------------------------------------------------------------------
// Functions to add terrain patches with various definitions
// (box, mesh, height-field)
// -----------------------------------------------------------------------------
void RigidTerrain::AddPatch(std::shared_ptr<Patch> patch,
                            const ChCoordsys<>& position,
                            std::shared_ptr<ChMaterialSurface> material) {
    m_num_patches++;

    // Create the rigid body for this patch (fixed)
    patch->m_body = std::shared_ptr<ChBody>(m_system->NewBody());
    patch->m_body->SetIdentifier(-m_num_patches);
    patch->m_body->SetNameString("patch_" + std::to_string(m_num_patches));
    patch->m_body->SetPos(position.pos);
    patch->m_body->SetRot(position.rot);
    patch->m_body->SetBodyFixed(true);
    patch->m_body->SetCollide(true);
    m_system->AddBody(patch->m_body);

    // Cache coefficient of friction
    patch->m_friction = material->GetSfriction();

    m_patches.push_back(patch);
}

// -----------------------------------------------------------------------------

std::shared_ptr<RigidTerrain::Patch> RigidTerrain::AddPatch(std::shared_ptr<ChMaterialSurface> material,
                                                            const ChVector<>& location,
                                                            const ChVector<>& normal,
                                                            double length,
                                                            double width,
                                                            double thickness,
                                                            bool tiled,
                                                            double max_tile_size,
                                                            bool visualization) {
    ChVector<> up = normal.GetNormalized();
    ChVector<> lateral = Vcross(up, ChWorldFrame::Forward());
    lateral.Normalize();
    ChVector<> forward = Vcross(lateral, up);
    ChMatrix33<> rot;
    rot.Set_A_axis(forward, lateral, up);

    auto patch = chrono_types::make_shared<BoxPatch>();
    AddPatch(patch, ChCoordsys<>(location - 0.5 * thickness * up, rot.Get_A_quaternion()), material);
    patch->m_visualize = visualization;

    // Create the collision model (one or more boxes) attached to the patch body
    patch->m_body->GetCollisionModel()->ClearModel();
    if (tiled) {
        int nX = (int)std::ceil(length / max_tile_size);
        int nY = (int)std::ceil(width / max_tile_size);
        double sizeX1 = length / nX;
        double sizeY1 = width / nY;
        for (int ix = 0; ix < nX; ix++) {
            for (int iy = 0; iy < nY; iy++) {
                patch->m_body->GetCollisionModel()->AddBox(                                                 //
                    material,                                                                               //
                    0.5 * sizeX1, 0.5 * sizeY1, 0.5 * thickness,                                            //
                    ChVector<>((sizeX1 - length) / 2 + ix * sizeX1, (sizeY1 - width) / 2 + iy * sizeY1, 0)  //
                );
            }
        }
    } else {
        patch->m_body->GetCollisionModel()->AddBox(material, 0.5 * length, 0.5 * width, 0.5 * thickness);
    }
    patch->m_body->GetCollisionModel()->BuildModel();

    // Cache patch parameters
    patch->m_location = location;
    patch->m_normal = up;
    patch->m_hlength = length / 2;
    patch->m_hwidth = width / 2; 
    patch->m_hthickness = thickness / 2;
    patch->m_radius = ChVector<>(length, width, thickness).Length() / 2;
    patch->m_type = PatchType::BOX;

    return patch;
}

// -----------------------------------------------------------------------------

std::shared_ptr<RigidTerrain::Patch> RigidTerrain::AddPatch(std::shared_ptr<ChMaterialSurface> material,
                                                            const ChCoordsys<>& position,
                                                            const std::string& mesh_file,
                                                            bool connected_mesh,
                                                            double sweep_sphere_radius,
                                                            bool visualization) {
    auto patch = chrono_types::make_shared<MeshPatch>();
    AddPatch(patch, position, material);
    patch->m_visualize = visualization;

    // Load mesh from file
    patch->m_trimesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(mesh_file, true, true);

    // Create the collision model
    patch->m_body->GetCollisionModel()->ClearModel();
    if (connected_mesh) {
        patch->m_body->GetCollisionModel()->AddTriangleMesh(material, patch->m_trimesh, true, false, VNULL,
                                                            ChMatrix33<>(1), sweep_sphere_radius);
    } else {
        patch->m_trimesh_s = geometry::ChTriangleMeshSoup::CreateFromWavefrontFile(mesh_file);
        patch->m_body->GetCollisionModel()->AddTriangleMesh(material, patch->m_trimesh_s, true, false, VNULL,
                                                            ChMatrix33<>(1), sweep_sphere_radius);
    }
    patch->m_body->GetCollisionModel()->BuildModel();

    auto mesh_name = filesystem::path(mesh_file).stem();

    // Cache patch parameters
    patch->m_radius =
        std::max_element(patch->m_trimesh->getCoordsVertices().begin(),                                      //
                         patch->m_trimesh->getCoordsVertices().end(),                                        //
                         [](const ChVector<>& a, const ChVector<>& b) { return a.Length2() < b.Length2(); }  //
                         )
            ->Length();

    patch->m_mesh_name = mesh_name;
    patch->m_type = PatchType::MESH;

    return patch;
}

// -----------------------------------------------------------------------------

std::shared_ptr<RigidTerrain::Patch> RigidTerrain::AddPatch(std::shared_ptr<ChMaterialSurface> material,
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
        throw ChException("Cannot open height map image file");
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
    patch->m_trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
    patch->m_trimesh->getCoordsVertices().resize(n_verts);
    patch->m_trimesh->getCoordsNormals().resize(n_verts);
    patch->m_trimesh->getCoordsUV().resize(n_verts);
    patch->m_trimesh->getCoordsColors().resize(n_verts);

    patch->m_trimesh->getIndicesVertexes().resize(n_faces);
    patch->m_trimesh->getIndicesNormals().resize(n_faces);

    // Initialize the array of accumulators (number of adjacent faces to a vertex)
    std::vector<int> accumulators(n_verts, 0);

    // Readability aliases
    std::vector<ChVector<> >& vertices = patch->m_trimesh->getCoordsVertices();
    std::vector<ChVector<> >& normals = patch->m_trimesh->getCoordsNormals();
    std::vector<ChVector<int> >& idx_vertices = patch->m_trimesh->getIndicesVertexes();
    std::vector<ChVector<int> >& idx_normals = patch->m_trimesh->getIndicesNormals();

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
            vertices[iv] = ChWorldFrame::FromISO(ChVector<>(x, y, z));
            // Initialize vertex normal to (0, 0, 0).
            normals[iv] = ChVector<>(0, 0, 0);
            // Assign color white to all vertices
            patch->m_trimesh->getCoordsColors()[iv] = ChColor(1, 1, 1);
            // Set UV coordinates in [0,1] x [0,1]
            patch->m_trimesh->getCoordsUV()[iv] = ChVector2<>(ix * x_scale, iy * y_scale);
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
            idx_vertices[it] = ChVector<int>(v0, v0 + nv_x + 1, v0 + nv_x);
            idx_normals[it] = ChVector<int>(v0, v0 + nv_x + 1, v0 + nv_x);
            ++it;
            idx_vertices[it] = ChVector<int>(v0, v0 + 1, v0 + nv_x + 1);
            idx_normals[it] = ChVector<int>(v0, v0 + 1, v0 + nv_x + 1);
            ++it;
        }
    }

    // Calculate normals and then average the normals from all adjacent faces
    for (it = 0; it < n_faces; ++it) {
        // Calculate the triangle normal as a normalized cross product.
        ChVector<> nrm = Vcross(vertices[idx_vertices[it][1]] - vertices[idx_vertices[it][0]],
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
    patch->m_body->GetCollisionModel()->ClearModel();
    if (connected_mesh) {
        patch->m_body->GetCollisionModel()->AddTriangleMesh(material, patch->m_trimesh, true, false, VNULL,
                                                            ChMatrix33<>(1), sweep_sphere_radius);
    } else {
        patch->m_trimesh_s = chrono_types::make_shared<geometry::ChTriangleMeshSoup>();
        std::vector<geometry::ChTriangle>& triangles = patch->m_trimesh_s->getTriangles();
        triangles.resize(n_faces);
        for (it = 0; it < n_faces; it++) {
            const ChVector<int>& idx = idx_vertices[it];
            triangles[it] = geometry::ChTriangle(vertices[idx[0]], vertices[idx[1]], vertices[idx[2]]);
        }
        patch->m_body->GetCollisionModel()->AddTriangleMesh(material, patch->m_trimesh_s, true, false, VNULL,
                                                            ChMatrix33<>(1), sweep_sphere_radius);
    }
    patch->m_body->GetCollisionModel()->BuildModel();

    auto mesh_name = filesystem::path(heightmap_file).stem();

    // Cache patch parameters
    patch->m_radius = ChVector<>(length, width, (hMax - hMin)).Length() / 2;
    patch->m_mesh_name = mesh_name;
    patch->m_type = PatchType::HEIGHT_MAP;

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
    m_vis_mat->SetKdTexture(filename, scale_x, scale_y);
}

// -----------------------------------------------------------------------------
// Initialize all terrain patches
// -----------------------------------------------------------------------------
class RTContactCallback : public ChContactContainer::AddContactCallback {
  public:
    virtual void OnAddContact(const collision::ChCollisionInfo& contactinfo,
                              ChMaterialComposite* const material) override {
        //// TODO: also accomodate terrain contact with FEA meshes.

        // Loop over all patch bodies and check if this contact involves one of them.
        ChBody* body_patch = nullptr;
        ChBody* body_other = nullptr;
        collision::ChCollisionShape* shape_other = nullptr;
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
        // is not a body or if the collsion does not involve a shape (e.g., a contact added by the user)
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
                auto mat = static_cast<ChMaterialCompositeNSC* const>(material);
                mat->static_friction = friction;
                mat->sliding_friction = friction;
                break;
            }
            case ChContactMethod::SMC: {
                auto mat = static_cast<ChMaterialCompositeSMC* const>(material);
                mat->mu_eff = friction;
                break;
            }
        }
    }

    ChTerrain::FrictionFunctor* m_friction_fun;
    RigidTerrain* m_terrain;
};

void RigidTerrain::Initialize() {
    if (m_patches.empty())
        return;

    for (auto& patch : m_patches) {
        // Initialize the patch (create visualization)
        patch->Initialize();

        // Add all patches to the same collision family
        // and disable collision with other collision models in this family.
        patch->m_body->GetCollisionModel()->SetFamily(m_collision_family);
        patch->m_body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(m_collision_family);
    }

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
    m_patches[0]->m_body->GetSystem()->GetContactContainer()->RegisterAddContactCallback(m_contact_callback);
}

void RigidTerrain::BoxPatch::Initialize() {
    if (m_visualize) {
        m_body->AddVisualModel(chrono_types::make_shared<ChVisualModel>());
        auto box = chrono_types::make_shared<ChBoxShape>();
        box->AddMaterial(m_vis_mat);
        box->GetBoxGeometry().Size = (ChVector<>(m_hlength, m_hwidth, m_hthickness));
        m_body->AddVisualShape(box);
    }
}

void RigidTerrain::MeshPatch::Initialize() {
    if (m_visualize) {
        m_body->AddVisualModel(chrono_types::make_shared<ChVisualModel>());
        auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        trimesh_shape->AddMaterial(m_vis_mat);
        trimesh_shape->SetName(m_mesh_name);
        trimesh_shape->SetMutable(false);
        trimesh_shape->SetMesh(m_trimesh, true);
        m_body->AddVisualShape(trimesh_shape);
    }
}

// -----------------------------------------------------------------------------
// Functions for obtaining the terrain height, normal, and coefficient of
// friction  at the specified location.
// This is done by casting vertical rays into each patch collision model.
// -----------------------------------------------------------------------------
double RigidTerrain::GetHeight(const ChVector<>& loc) const {
    double height;
    ChVector<> normal;
    float friction;

    bool hit = FindPoint(loc, height, normal, friction);

    return hit ? height : 0.0;
}

ChVector<> RigidTerrain::GetNormal(const ChVector<>& loc) const {
    double height;
    ChVector<> normal;
    float friction;

    bool hit = FindPoint(loc, height, normal, friction);

    return hit ? normal : ChWorldFrame::Vertical();
}

float RigidTerrain::GetCoefficientFriction(const ChVector<>& loc) const {
    if (m_friction_fun)
        return (*m_friction_fun)(loc);

    double height;
    ChVector<> normal;
    float friction;

    bool hit = FindPoint(loc, height, normal, friction);

    return hit ? friction : 0.8f;
}

bool RigidTerrain::FindPoint(const ChVector<> loc, double& height, ChVector<>& normal, float& friction) const {
    bool hit = false;
    height = std::numeric_limits<double>::lowest();
    normal = ChWorldFrame::Vertical();
    friction = 0.8f;

    for (auto patch : m_patches) {
        double pheight;
        ChVector<> pnormal;
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

bool RigidTerrain::BoxPatch::FindPoint(const ChVector<>& loc, double& height, ChVector<>& normal) const {
    // Ray definition (in global frame)
    ChVector<> A = loc + (m_radius + 1000) * ChWorldFrame::Vertical();  // start point
    ChVector<> v = -ChWorldFrame::Vertical();                           // direction (downward)

    // Intersect ray with top plane
    double t = Vdot(m_location - A, m_normal) / Vdot(v, m_normal);
    ChVector<> C = A + t * v;
    height = ChWorldFrame::Height(C);
    normal = m_normal;

    // Check bounds
    ChVector<> Cl = m_body->TransformPointParentToLocal(C);
    return std::abs(Cl.x()) <= m_hlength && std::abs(Cl.y()) <= m_hwidth;
}

bool RigidTerrain::MeshPatch::FindPoint(const ChVector<>& loc, double& height, ChVector<>& normal) const {
    ChVector<> from = loc + (m_radius + 1000) * ChWorldFrame::Vertical();
    ChVector<> to = loc - (m_radius + 1000) * ChWorldFrame::Vertical();

    collision::ChCollisionSystem::ChRayhitResult result;
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
    utils::WriteMeshPovray(*m_trimesh, m_mesh_name, out_dir, ChColor(1, 1, 1), ChVector<>(0, 0, 0),
                           ChQuaternion<>(1, 0, 0, 0), smoothed);
}

void RigidTerrain::MeshPatch::ExportMeshWavefront(const std::string& out_dir) {
    std::string obj_filename = out_dir + "/" + m_mesh_name + ".obj";
    std::vector<geometry::ChTriangleMeshConnected> meshes = {*m_trimesh};
    std::cout << "Exporting to " << obj_filename << std::endl;
    m_trimesh->WriteWavefront(obj_filename, meshes);
}

}  // end namespace vehicle
}  // end namespace chrono
