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

#include <cmath>
#include <cstdio>

#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/physics/ChMaterialSurfaceNSC.h"
#include "chrono/physics/ChMaterialSurfaceSMC.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_thirdparty/Easy_BMP/EasyBMP.h"
#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Default constructor.
// -----------------------------------------------------------------------------
RigidTerrain::RigidTerrain(ChSystem* system)
    : m_system(system), m_num_patches(0), m_use_friction_functor(false), m_contact_callback(nullptr) {}

// -----------------------------------------------------------------------------
// Constructor from JSON file
// -----------------------------------------------------------------------------
RigidTerrain::RigidTerrain(ChSystem* system, const std::string& filename)
    : m_system(system), m_num_patches(0), m_use_friction_functor(false), m_contact_callback(nullptr) {
    // Open the JSON file and read data
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

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

RigidTerrain::~RigidTerrain() {
    delete m_contact_callback;
}

void RigidTerrain::LoadPatch(const rapidjson::Value& d) {
    assert(d.IsObject());
    assert(d.HasMember("Location"));
    assert(d.HasMember("Orientation"));
    assert(d.HasMember("Geometry"));
    assert(d.HasMember("Contact Material"));

    // Create patch with specified geometry
    std::shared_ptr<Patch> patch;
    auto loc = ReadVectorJSON(d["Location"]);
    auto rot = ReadQuaternionJSON(d["Orientation"]);

    if (d["Geometry"].HasMember("Dimensions")) {
        auto size = ReadVectorJSON(d["Geometry"]["Dimensions"]);
        patch = AddPatch(ChCoordsys<>(loc, rot), size);
    } else if (d["Geometry"].HasMember("Mesh Filename")) {
        std::string mesh_file = d["Geometry"]["Mesh Filename"].GetString();
        std::string mesh_name = d["Geometry"]["Mesh Name"].GetString();
        patch = AddPatch(ChCoordsys<>(loc, rot), vehicle::GetDataFile(mesh_file), mesh_name);
    } else if (d["Geometry"].HasMember("Height Map Filename")) {
        std::string bmp_file = d["Geometry"]["Height Map Filename"].GetString();
        std::string mesh_name = d["Geometry"]["Mesh Name"].GetString();
        double sx = d["Geometry"]["Size"][0u].GetDouble();
        double sy = d["Geometry"]["Size"][1u].GetDouble();
        double hMin = d["Geometry"]["Height Range"][0u].GetDouble();
        double hMax = d["Geometry"]["Height Range"][1u].GetDouble();
        patch = AddPatch(ChCoordsys<>(loc, rot), vehicle::GetDataFile(bmp_file), mesh_name, sx, sy, hMin, hMax);
    }

    // Set contact material properties
    float mu = d["Contact Material"]["Coefficient of Friction"].GetFloat();
    float cr = d["Contact Material"]["Coefficient of Restitution"].GetFloat();
    patch->SetContactFrictionCoefficient(mu);
    patch->SetContactRestitutionCoefficient(cr);
    if (d["Contact Material"].HasMember("Properties")) {
        float ym = d["Contact Material"]["Properties"]["Young Modulus"].GetFloat();
        float pr = d["Contact Material"]["Properties"]["Poisson Ratio"].GetFloat();
        patch->SetContactMaterialProperties(ym, pr);
    }
    if (d["Contact Material"].HasMember("Coefficients")) {
        float kn = d["Contact Material"]["Coefficients"]["Normal Stiffness"].GetFloat();
        float gn = d["Contact Material"]["Coefficients"]["Normal Damping"].GetFloat();
        float kt = d["Contact Material"]["Coefficients"]["Tangential Stiffness"].GetFloat();
        float gt = d["Contact Material"]["Coefficients"]["Tangential Damping"].GetFloat();
        patch->SetContactMaterialCoefficients(kn, gn, kt, gt);
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
std::shared_ptr<RigidTerrain::Patch> RigidTerrain::AddPatch(const ChCoordsys<>& position) {
    m_num_patches++;
    auto patch = std::make_shared<Patch>();

    // Create the rigid body for this patch (fixed)
    patch->m_body = std::shared_ptr<ChBody>(m_system->NewBody());
    patch->m_body->SetIdentifier(-m_num_patches);
    patch->m_body->SetNameString("patch_" + std::to_string(m_num_patches));
    patch->m_body->SetPos(position.pos);
    patch->m_body->SetRot(position.rot);
    patch->m_body->SetBodyFixed(true);
    patch->m_body->SetCollide(true);
    m_system->AddBody(patch->m_body);

    // Initialize contact material properties
    patch->m_friction = 0.7f;
    switch (m_system->GetContactMethod()) {
        case ChMaterialSurface::NSC:
            patch->m_body->GetMaterialSurfaceNSC()->SetFriction(patch->m_friction);
            patch->m_body->GetMaterialSurfaceNSC()->SetRestitution(0.1f);
            break;
        case ChMaterialSurface::SMC:
            patch->m_body->GetMaterialSurfaceSMC()->SetFriction(patch->m_friction);
            patch->m_body->GetMaterialSurfaceSMC()->SetRestitution(0.1f);
            patch->m_body->GetMaterialSurfaceSMC()->SetYoungModulus(2e5f);
            patch->m_body->GetMaterialSurfaceSMC()->SetPoissonRatio(0.3f);
            patch->m_body->GetMaterialSurfaceSMC()->SetKn(2e5f);
            patch->m_body->GetMaterialSurfaceSMC()->SetGn(40.0f);
            patch->m_body->GetMaterialSurfaceSMC()->SetKt(2e5f);
            patch->m_body->GetMaterialSurfaceSMC()->SetGt(20.0f);
            break;
    }

    // Insert in vector and return a reference to the patch
    m_patches.push_back(patch);

    return patch;
}

// -----------------------------------------------------------------------------

std::shared_ptr<RigidTerrain::Patch> RigidTerrain::AddPatch(const ChCoordsys<>& position,
                                                            const ChVector<>& size,
                                                            bool tiled,
                                                            double max_tile_size,
                                                            bool visualization) {
    auto patch = AddPatch(position);

    // Create collision model (box) attached to the patch body
    patch->m_body->GetCollisionModel()->ClearModel();
    if (tiled) {
        int nX = (int)std::ceil(size.x() / max_tile_size);
        int nY = (int)std::ceil(size.y() / max_tile_size);
        double sizeX1 = size.x() / nX;
        double sizeY1 = size.y() / nY;
        for (int ix = 0; ix < nX; ix++) {
            for (int iy = 0; iy < nY; iy++) {
                patch->m_body->GetCollisionModel()->AddBox(
                    0.5 * sizeX1, 0.5 * sizeY1, 0.5 * size.z(),
                    ChVector<>((sizeX1 - size.x()) / 2 + ix * sizeX1, (sizeY1 - size.y()) / 2 + iy * sizeY1, 0));
            }
        }
    } else {
        patch->m_body->GetCollisionModel()->AddBox(0.5 * size.x(), 0.5 * size.y(), 0.5 * size.z());
    }
    patch->m_body->GetCollisionModel()->BuildModel();

    // Create visualization asset
    if (visualization) {
        auto box = std::make_shared<ChBoxShape>();
        box->GetBoxGeometry().SetLengths(size);
        box->Pos = VNULL;
        patch->m_body->AddAsset(box);
    }

    patch->m_type = BOX;

    return patch;
}

// -----------------------------------------------------------------------------

std::shared_ptr<RigidTerrain::Patch> RigidTerrain::AddPatch(const ChCoordsys<>& position,
                                                            const std::string& mesh_file,
                                                            const std::string& mesh_name,
                                                            double sweep_sphere_radius,
                                                            bool visualization) {
    auto patch = AddPatch(position);

    // Load mesh from file
    patch->m_trimesh = std::make_shared<geometry::ChTriangleMeshConnected>();
    patch->m_trimesh->LoadWavefrontMesh(mesh_file, true, true);

    // Create the collision model
    patch->m_body->GetCollisionModel()->ClearModel();
    patch->m_body->GetCollisionModel()->AddTriangleMesh(patch->m_trimesh, true, false, VNULL, ChMatrix33<>(1),
                                                        sweep_sphere_radius);
    patch->m_body->GetCollisionModel()->BuildModel();

    // Create the visualization asset.
    if (visualization) {
        auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(patch->m_trimesh);
        trimesh_shape->SetName(mesh_name);
        trimesh_shape->SetStatic(true);
        patch->m_body->AddAsset(trimesh_shape);
    }

    patch->m_mesh_name = mesh_name;
    patch->m_type = MESH;

    return patch;
}

// -----------------------------------------------------------------------------

std::shared_ptr<RigidTerrain::Patch> RigidTerrain::AddPatch(const ChCoordsys<>& position,
                                                            const std::string& heightmap_file,
                                                            const std::string& mesh_name,
                                                            double sizeX,
                                                            double sizeY,
                                                            double hMin,
                                                            double hMax,
                                                            bool visualization) {
    auto patch = AddPatch(position);

    // Read the BMP file and extract number of pixels.
    BMP hmap;
    if (!hmap.ReadFromFile(heightmap_file.c_str())) {
        throw ChException("Cannot open height map BMP file");
    }
    int nv_x = hmap.TellWidth();
    int nv_y = hmap.TellHeight();

    // Construct a triangular mesh of sizeX x sizeY.
    // Each pixel in the BMP represents a vertex.
    // The gray level of a pixel is mapped to the height range, with black corresponding
    // to hMin and white corresponding to hMax.
    // UV coordinates are mapped in [0,1] x [0,1].
    // We use smoothed vertex normals.
    double dx = sizeX / (nv_x - 1);
    double dy = sizeY / (nv_y - 1);
    double h_scale = (hMax - hMin) / 255;
    double x_scale = 1.0 / (nv_x - 1);
    double y_scale = 1.0 / (nv_y - 1);
    unsigned int n_verts = nv_x * nv_y;
    unsigned int n_faces = 2 * (nv_x - 1) * (nv_y - 1);

    // Resize mesh arrays.
    patch->m_trimesh = std::make_shared<geometry::ChTriangleMeshConnected>();
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
        double y = 0.5 * sizeY - iy * dy;
        for (int ix = 0; ix < nv_x; ++ix) {
            double x = ix * dx - 0.5 * sizeX;
            // Calculate equivalent gray level (RGB -> YUV)
            ebmpBYTE red = hmap(ix, iy)->Red;
            ebmpBYTE green = hmap(ix, iy)->Green;
            ebmpBYTE blue = hmap(ix, iy)->Blue;
            double gray = 0.299 * red + 0.587 * green + 0.114 * blue;
            // Map gray level to vertex height
            double z = hMin + gray * h_scale;
            // Set vertex location
            vertices[iv] = ChVector<>(x, y, z);
            // Initialize vertex normal to (0, 0, 0).
            normals[iv] = ChVector<>(0, 0, 0);
            // Assign color white to all vertices
            patch->m_trimesh->getCoordsColors()[iv] = ChVector<float>(1, 1, 1);
            // Set UV coordinates in [0,1] x [0,1]
            patch->m_trimesh->getCoordsUV()[iv] = ChVector<>(ix * x_scale, iy * y_scale, 0.0);
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

    // Calculate normals and then average the normals from all adjacent faces.
    for (unsigned int it = 0; it < n_faces; ++it) {
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

    // Set the normals to the average values.
    for (unsigned int in = 0; in < n_verts; ++in) {
        normals[in] /= (double)accumulators[in];
    }

    // Create contact geometry.
    patch->m_body->GetCollisionModel()->ClearModel();
    patch->m_body->GetCollisionModel()->AddTriangleMesh(patch->m_trimesh, true, false, ChVector<>(0, 0, 0));
    patch->m_body->GetCollisionModel()->BuildModel();

    // Create the visualization asset.
    if (visualization) {
        auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(patch->m_trimesh);
        trimesh_shape->SetName(mesh_name);
        patch->m_body->AddAsset(trimesh_shape);
    }

    patch->m_mesh_name = mesh_name;
    patch->m_type = HEIGHT_MAP;

    return patch;
}

// -----------------------------------------------------------------------------
// Functions to modify properties of a patch
// -----------------------------------------------------------------------------
void RigidTerrain::Patch::SetContactFrictionCoefficient(float friction_coefficient) {
    switch (m_body->GetContactMethod()) {
        case ChMaterialSurface::NSC:
            m_body->GetMaterialSurfaceNSC()->SetFriction(friction_coefficient);
            break;
        case ChMaterialSurface::SMC:
            m_body->GetMaterialSurfaceSMC()->SetFriction(friction_coefficient);
            break;
    }

    m_friction = friction_coefficient;
}

void RigidTerrain::Patch::SetContactRestitutionCoefficient(float restitution_coefficient) {
    switch (m_body->GetContactMethod()) {
        case ChMaterialSurface::NSC:
            m_body->GetMaterialSurfaceNSC()->SetRestitution(restitution_coefficient);
            break;
        case ChMaterialSurface::SMC:
            m_body->GetMaterialSurfaceSMC()->SetRestitution(restitution_coefficient);
            break;
    }
}

void RigidTerrain::Patch::SetContactMaterialProperties(float young_modulus, float poisson_ratio) {
    if (m_body->GetContactMethod() == ChMaterialSurface::SMC) {
        m_body->GetMaterialSurfaceSMC()->SetYoungModulus(young_modulus);
        m_body->GetMaterialSurfaceSMC()->SetPoissonRatio(poisson_ratio);
    }
}

void RigidTerrain::Patch::SetContactMaterialCoefficients(float kn, float gn, float kt, float gt) {
    if (m_body->GetContactMethod() == ChMaterialSurface::SMC) {
        m_body->GetMaterialSurfaceSMC()->SetKn(kn);
        m_body->GetMaterialSurfaceSMC()->SetGn(gn);
        m_body->GetMaterialSurfaceSMC()->SetKt(kt);
        m_body->GetMaterialSurfaceSMC()->SetGt(gt);
    }
}

void RigidTerrain::Patch::SetColor(const ChColor& color) {
    auto acolor = std::make_shared<ChColorAsset>(color);
    m_body->AddAsset(acolor);
}

void RigidTerrain::Patch::SetTexture(const std::string& tex_file, float tex_scale_x, float tex_scale_y) {
    auto texture = std::make_shared<ChTexture>();
    texture->SetTextureFilename(tex_file);
    texture->SetTextureScale(tex_scale_x, tex_scale_y);
    m_body->AddAsset(texture);
}

// -----------------------------------------------------------------------------
// Export the patch mesh (if any) as a macro in a PovRay include file.
// -----------------------------------------------------------------------------
void RigidTerrain::Patch::ExportMeshPovray(const std::string& out_dir) {
    switch (m_type) {
        case MESH:
            utils::WriteMeshPovray(*m_trimesh, m_mesh_name, out_dir, ChColor(1, 1, 1));
            break;
        case HEIGHT_MAP:
            utils::WriteMeshPovray(*m_trimesh, m_mesh_name, out_dir, ChColor(1, 1, 1), ChVector<>(0, 0, 0),
                                   ChQuaternion<>(1, 0, 0, 0), true);
            break;
        default:
            break;
    }
}

// -----------------------------------------------------------------------------
// Return the underlying patch body.
// -----------------------------------------------------------------------------
std::shared_ptr<ChBody> RigidTerrain::Patch::GetGroundBody() const {
    return m_body;
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
        ChBody* body_other = nullptr;
        bool process = false;
        for (auto patch : m_terrain->GetPatches()) {
            auto model = patch->GetGroundBody()->GetCollisionModel().get();
            if (model == contactinfo.modelA) {
                body_other = dynamic_cast<ChBody*>(contactinfo.modelB->GetContactable());
                process = true;
                break;
            }
            if (model == contactinfo.modelB) {
                body_other = dynamic_cast<ChBody*>(contactinfo.modelA->GetContactable());
                process = true;
                break;
            }
        }

        // Do nothing if this contact does not involve a terrain body or if the other contactable
        // is not a body.
        if (!process || !body_other)
            return;

        // Find the terrain coefficient of friction at the location of current contact.
        // Arbitrarily use the collision point on modelA.
        auto friction_terrain = (*m_friction_fun)(contactinfo.vpA.x(), contactinfo.vpA.y());

        // Get the current combination strategy for composite materials.
        auto& strategy = body_other->GetSystem()->GetMaterialCompositionStrategy();

        // Set friction in composite material based on contact formulation.
        switch (body_other->GetContactMethod()) {
            case ChMaterialSurface::NSC: {
                auto mat_other = std::static_pointer_cast<ChMaterialSurfaceNSC>(body_other->GetMaterialSurface());
                auto friction_other = mat_other->sliding_friction;
                auto friction = strategy.CombineFriction(friction_terrain, friction_other);
                auto mat = static_cast<ChMaterialCompositeNSC* const>(material);
                mat->static_friction = friction;
                mat->sliding_friction = friction;
                break;
            }
            case ChMaterialSurface::SMC: {
                auto mat_other = std::static_pointer_cast<ChMaterialSurfaceSMC>(body_other->GetMaterialSurface());
                auto friction_other = mat_other->sliding_friction;
                auto friction = strategy.CombineFriction(friction_terrain, friction_other);
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
    if (!m_friction_fun)
        m_use_friction_functor = false;
    if (!m_use_friction_functor)
        return;
    if (m_patches.empty())
        return;

    // Create and register a custom callback functor of type ChContactContainer::AddContactCallback
    // and pass to it a list of patch bodies as well as the location-dependent friction functor.
    auto callback = new RTContactCallback;
    callback->m_terrain = this;
    callback->m_friction_fun = m_friction_fun;
    m_contact_callback = callback;
    m_patches[0]->m_body->GetSystem()->GetContactContainer()->RegisterAddContactCallback(m_contact_callback);
}

// -----------------------------------------------------------------------------
// Functions for obtaining the terrain height, normal, and coefficient of
// friction  at the specified location.
// This is done by casting vertical rays into each patch collision model.
// -----------------------------------------------------------------------------
bool RigidTerrain::FindPoint(double x, double y, double& height, ChVector<>& normal, float& friction) const {
    bool hit = false;
    height = -1000;
    normal = ChVector<>(0, 0, 1);
    friction = 0.8f;

    ChVector<> from(x, y, 1000);
    ChVector<> to(x, y, -1000);

    for (auto patch : m_patches) {
        collision::ChCollisionSystem::ChRayhitResult result;
        m_system->GetCollisionSystem()->RayHit(from, to, patch->m_body->GetCollisionModel().get(), result);
        if (result.hit && result.abs_hitPoint.z() > height) {
            hit = true;
            height = result.abs_hitPoint.z();
            normal = result.abs_hitNormal;
            friction = patch->m_friction;
        }
    }

    return hit;
}

double RigidTerrain::GetHeight(double x, double y) const {
    double height;
    ChVector<> normal;
    float friction;

    bool hit = FindPoint(x, y, height, normal, friction);

    return hit ? height : 0.0;
}

ChVector<> RigidTerrain::GetNormal(double x, double y) const {
    double height;
    ChVector<> normal;
    float friction;

    bool hit = FindPoint(x, y, height, normal, friction);

    return normal;
}

float RigidTerrain::GetCoefficientFriction(double x, double y) const {
    if (m_friction_fun)
        return (*m_friction_fun)(x, y);

    double height;
    ChVector<> normal;
    float friction;

    bool hit = FindPoint(x, y, height, normal, friction);

    return friction;
}

// -----------------------------------------------------------------------------
// Export all patch meshes as macros in PovRay include files.
// -----------------------------------------------------------------------------
void RigidTerrain::ExportMeshPovray(const std::string& out_dir) {
    for (auto patch : m_patches) {
        patch->ExportMeshPovray(out_dir);
    }
}

}  // end namespace vehicle
}  // end namespace chrono
