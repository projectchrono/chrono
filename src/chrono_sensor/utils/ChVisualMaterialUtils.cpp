// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Asher Elmquist
// =============================================================================
//
// utils file translating Chrono assets to higher quality assets
//
// =============================================================================

#include "chrono_sensor/utils/ChVisualMaterialUtils.h"

#include <iostream>
#include "chrono_thirdparty/tinyobjloader/tiny_obj_loader.h"

namespace chrono {
namespace sensor {

void CreateModernMeshAssets(std::shared_ptr<ChTriangleMeshShape> mesh_shape) {
    if (mesh_shape->GetMesh()->GetFileName() == "") {
        return;
    }

    tinyobj::attrib_t att;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warn;
    std::string err;

    std::string mtl_base = "";
    std::string file_name = mesh_shape->GetMesh()->GetFileName();

    int slash_location = (int)file_name.rfind('/');

    if (std::string::npos != slash_location) {
        mtl_base = file_name.substr(0, slash_location + 1);
    }

    bool success = LoadObj(&att, &shapes, &materials, &warn, &err, file_name.c_str(), mtl_base.c_str());

    if (!success) {
        std::cout << "Unable to load OBJ file: " << file_name << std::endl;
    }

    // go through each shape and add the material as an asset. Also add the material id list to the
    // ChVisualization asset along with a list of triangle-to-face id list to mesh

    std::vector<std::shared_ptr<ChVisualMaterial>> material_list = std::vector<std::shared_ptr<ChVisualMaterial>>();

    std::vector<ChVector<double>> vertex_buffer;       // = std::vector<ChVector<double>>();
    std::vector<ChVector<double>> normal_buffer;       // = std::vector<ChVector<double>>();
    std::vector<ChVector<double>> tex_coords;          // = std::vector<ChVector<double>>();
    std::vector<ChVector<int>> vertex_index_buffer;    // = std::vector<ChVector<int>>();
    std::vector<ChVector<int>> normal_index_buffer;    // = std::vector<ChVector<int>>();
    std::vector<ChVector<int>> texcoord_index_buffer;  // = std::vector<ChVector<int>>();
    std::vector<ChVector<int>> material_index_buffer;  // = std::vector<ChVector<int>>();

    unsigned int previous_vertices = 0;

    // copy in vertices
    // vertex_buffer.resize(att.vertices.size() / 3);
    for (int i = 0; i < att.vertices.size() / 3; i++) {
        vertex_buffer.push_back(
            ChVector<double>(att.vertices[3 * i + 0], att.vertices[3 * i + 1], att.vertices[3 * i + 2]));
    }

    // copy in normals
    // normal_buffer.resize(att.normals.size() / 3);
    for (int i = 0; i < att.normals.size() / 3; i++) {
        normal_buffer.push_back(
            ChVector<double>(att.normals[3 * i + 0], att.normals[3 * i + 1], att.normals[3 * i + 2]));
    }

    // copy in tex coords (uvs)
    for (int i = 0; i < att.texcoords.size() / 2; i++) {
        tex_coords.push_back(ChVector<double>(att.texcoords[2 * i + 0], att.texcoords[2 * i + 1], 0));
    }

    // copy in materials
    for (int i = 0; i < materials.size(); i++) {
        std::shared_ptr<ChVisualMaterial> mat = chrono_types::make_shared<ChVisualMaterial>();
        mat->SetAmbientColor({materials[i].ambient[0], materials[i].ambient[1], materials[i].ambient[2]});

        mat->SetDiffuseColor({materials[i].diffuse[0], materials[i].diffuse[1], materials[i].diffuse[2]});
        mat->SetSpecularColor({materials[i].specular[0], materials[i].specular[1], materials[i].specular[2]});
        // mat->SetSpecularExponent(float exponent);
        mat->SetTransparency(materials[i].dissolve);

        if (materials[i].diffuse_texname != "") {
            mat->SetKdTexture(mtl_base + materials[i].diffuse_texname);
            // std::cout << "Kd Map: " << mtl_base + materials[i].diffuse_texname << std::endl;
        }

        if (materials[i].bump_texname != "") {
            mat->SetNormalMapTexture(mtl_base + materials[i].bump_texname);
            // std::cout << "Normal Map: " << mtl_base + materials[i].bump_texname << std::endl;
        }
        // mat->SetFresnelExp(float exp);
        // mat->SetFresnelMax(float max);
        // mat->SetFresnelMin(float min);
        mat->SetRoughness(materials[i].roughness);

        material_list.push_back(mat);
    }

    // for each shape, copy in vertex_indices, normal_indices, and tex_coord_indices, material_indices
    for (int i = 0; i < shapes.size(); i++) {
        for (int j = 0; j < shapes[i].mesh.indices.size() / 3; j++) {
            // vertex indices
            vertex_index_buffer.push_back(ChVector<int>(shapes[i].mesh.indices[3 * j + 0].vertex_index,
                                                        shapes[i].mesh.indices[3 * j + 1].vertex_index,
                                                        shapes[i].mesh.indices[3 * j + 2].vertex_index));

            // normal indices. TODO: what if there are no normal indices?
            if (normal_buffer.size() > 0) {
                normal_index_buffer.push_back(ChVector<int>(shapes[i].mesh.indices[3 * j + 0].normal_index,
                                                            shapes[i].mesh.indices[3 * j + 1].normal_index,
                                                            shapes[i].mesh.indices[3 * j + 2].normal_index));
            }

            // texcoord indices. TODO: what if there are no texcoord indices?
            if (tex_coords.size() > 0) {
                texcoord_index_buffer.push_back(ChVector<int>(shapes[i].mesh.indices[3 * j + 0].texcoord_index,
                                                              shapes[i].mesh.indices[3 * j + 1].texcoord_index,
                                                              shapes[i].mesh.indices[3 * j + 2].texcoord_index));
            }
            material_index_buffer.push_back(ChVector<int>(shapes[i].mesh.material_ids[j], 0, 0));
        }
    }

    mesh_shape->GetMesh()->m_vertices = vertex_buffer;
    mesh_shape->GetMesh()->m_normals = normal_buffer;
    mesh_shape->GetMesh()->m_UV = tex_coords;
    mesh_shape->GetMesh()->m_face_v_indices = vertex_index_buffer;
    mesh_shape->GetMesh()->m_face_n_indices = normal_index_buffer;
    mesh_shape->GetMesh()->m_face_uv_indices = texcoord_index_buffer;
    mesh_shape->GetMesh()->m_face_col_indices = material_index_buffer;

    mesh_shape->material_list = material_list;

    // std::cout << "Vertices: " << vertex_buffer.size() << std::endl;
    // std::cout << "Normals: " << normal_buffer.size() << std::endl;
    // std::cout << "Texcoords: " << tex_coords.size() << std::endl;
    // std::cout << "Vertex Indices: " << vertex_index_buffer.size() << std::endl;
    // std::cout << "Normal Indices: " << normal_index_buffer.size() << std::endl;
    // std::cout << "UV Indices: " << texcoord_index_buffer.size() << std::endl;
    // std::cout << "Mat Indices: " << material_index_buffer.size() << std::endl;
    // std::cout << "Materials: " << material_list.size() << std::endl;

    // std::cout << "Reloaded mesh with materials: " << file_name << std::endl;
}

void ConvertToModernAssets(std::shared_ptr<ChBody> body) {
    if (body->GetAssets().size() > 0) {
        // iterate through all assets in the body
        // std::cout << "Number of assets: " << body->GetAssets().size() << std::endl;
        for (auto asset : body->GetAssets()) {
            // check if the asset is a ChVisualization
            if (std::shared_ptr<ChVisualization> visual_asset = std::dynamic_pointer_cast<ChVisualization>(asset)) {
                // collect relative position and orientation of the asset

                if (std::shared_ptr<ChTriangleMeshShape> trimesh_shape =
                        std::dynamic_pointer_cast<ChTriangleMeshShape>(asset)) {
                    CreateModernMeshAssets(trimesh_shape);
                }

            }
            // check if the asset is a ChColorAsset
            else if (std::shared_ptr<ChColorAsset> visual_asset = std::dynamic_pointer_cast<ChColorAsset>(asset)) {
                // std::cout << "Asset was color\n";
                // CreateVisualMaterial(body, visual_asset);
            }

            // check if the asset is a ChTexture
            else if (std::shared_ptr<ChTexture> visual_asset = std::dynamic_pointer_cast<ChTexture>(asset)) {
                // std::cout << "Asset was texture\n";
            }
        }
    }
}

void ConvertToModernAssets(ChSystem* sys) {
    for (auto body : sys->Get_bodylist()) {
        ConvertToModernAssets(body);
    }
}

}  // namespace sensor
}  // namespace chrono
