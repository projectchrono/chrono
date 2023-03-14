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
// Authors: Hammad Mazhar
// =============================================================================
// Uses the tiny_obj_loader library to load an OBJ file in the proper format
// =============================================================================

#include <iostream>
#include <sstream>
#include <string>

#include "chrono_opengl/shapes/obj/ChOpenGLOBJLoader.h"

using namespace glm;

namespace chrono {
namespace opengl {

ChOpenGLOBJLoader::ChOpenGLOBJLoader() {}

// load an obj mesh. Each mesh can have multiple sub meshes
void ChOpenGLOBJLoader::LoadObject(const char* mesh_file,
                                   std::vector<std::vector<glm::vec3>>& vertices,
                                   std::vector<std::vector<glm::vec3>>& normals,
                                   std::vector<std::vector<glm::vec2>>& texcoords,
                                   std::vector<std::vector<GLuint>>& indices,
                                   std::vector<std::string>& names) {
    std::vector<tinyobj::shape_t> shapes;
    tinyobj::attrib_t att;
    std::vector<tinyobj::material_t> materials;
    std::string warn;
    std::string err;

    std::stringstream ifs;
    ifs << mesh_file;

    LoadObj(&att, &shapes, &materials, &warn, &err, &ifs);

    vertices.resize(shapes.size());
    normals.resize(shapes.size());
    texcoords.resize(shapes.size());
    indices.resize(shapes.size());
    names.resize(shapes.size());

    // convert between mesh loader data structure and vector data structure
    // Since Chrono::OpenGL expects each vertex to have a single normal, we need to duplicate any vertices that
    // have a different normal. If we duplicate, we need to change everyone's index reference to that vertex
    for (size_t i = 0; i < shapes.size(); i++) {
        names[i] = shapes[i].name;
        vertices[i].clear();
        normals[i].clear();
        texcoords[i].clear();
        indices[i].clear();

        // 1. go through every index
        // 2. if the vertex_id, normal_id pair hasn't been added yet add the vertex, normal, texcoord (if applicable)
        // and index. Then add our hashed index to the map to tell future indices what they should be changed to
        // 3. if the vertex_id, normal pair exists, look up in the map what the new index is for this pair

        std::map<int, GLuint> mapped_ids;

        for (unsigned int f = 0; f < shapes[i].mesh.indices.size(); f++) {
            int vertex_id = shapes[i].mesh.indices[f].vertex_index;
            int normal_id = shapes[i].mesh.indices[f].normal_index;
            int texcood_id = shapes[i].mesh.indices[f].texcoord_index;

            // hashed as vertex_id * indices + normal_id to make sure it is unique among the indices: vertex_id,
            // normal_id pair
            int hashed_id = vertex_id * (int)(shapes[i].mesh.indices.size()) + normal_id;

            // vertex_id normal_id combo not yet in index buffer
            if (mapped_ids.find(hashed_id) == mapped_ids.end()) {
                //
                GLuint new_index = (GLuint)(vertices[i].size());

                vertices[i].push_back(glm::vec3(att.vertices[3 * vertex_id + 0], att.vertices[3 * vertex_id + 1],
                                                att.vertices[3 * vertex_id + 2]));

                normals[i].push_back(glm::vec3(att.normals[3 * normal_id + 0], att.normals[3 * normal_id + 1],
                                               att.normals[3 * normal_id + 2]));

                if (texcood_id > 0)
                    texcoords[i].push_back(
                        glm::vec2(att.texcoords[2 * texcood_id + 0], -1 * att.texcoords[2 * texcood_id + 1]));

                indices[i].push_back(new_index);

                mapped_ids[hashed_id] = new_index;
            } else {
                // vertex_id normal_id combo already in index buffer
                indices[i].push_back(mapped_ids[hashed_id]);
            }
        }
    }
}
}  // namespace opengl
}  // namespace chrono
