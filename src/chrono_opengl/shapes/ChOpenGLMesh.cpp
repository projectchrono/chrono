// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================
// Generic renderable triangle mesh.
// =============================================================================

#include <iostream>
#include <glm/gtc/type_ptr.hpp>

#include "chrono_opengl/shapes/ChOpenGLMesh.h"

using namespace glm;

namespace chrono {
namespace opengl {

ChOpenGLMesh::ChOpenGLMesh() : ChOpenGLObject() {}

bool ChOpenGLMesh::Initialize(std::vector<glm::vec3>& vertices,
                              std::vector<glm::vec3>& normals,
                              std::vector<glm::vec2>& texcoords,
                              std::vector<GLuint>& indices,
                              ChOpenGLMaterial mat) {
    if (GLReturnedError("Mesh::Initialize - on entry")) {
        return false;
    }

    if (!super::Initialize()) {
        return false;
    }

    this->vertex_indices = indices;
    ambient = mat.ambient_color;
    diffuse = mat.diffuse_color;
    specular = mat.specular_color;

    for (unsigned int i = 0; i < vertices.size(); i++) {
        this->data.push_back(ChOpenGLVertexAttributesPN(vertices[i], normals[i]));
    }

    PostInitialize();

    if (GLReturnedError("ChOpenGLMesh::Initialize - on exit")) {
        return false;
    }

    return true;
}

bool ChOpenGLMesh::Initialize(chrono::ChTriangleMeshShape* tri_mesh, ChOpenGLMaterial mat) {
    if (GLReturnedError("Mesh::Initialize - on entry")) {
        return false;
    }

    if (!super::Initialize()) {
        return false;
    }
    int num_triangles = tri_mesh->GetMesh().getNumTriangles();

    for (unsigned int i = 0; i < (unsigned)num_triangles; i++) {
        chrono::geometry::ChTriangle tri = tri_mesh->GetMesh().getTriangle(i);
        ChVector<> norm = tri.GetNormal();
        ChVector<> v1 = tri.p1;
        ChVector<> v2 = tri.p2;
        ChVector<> v3 = tri.p3;
        this->vertex_indices.push_back(i * 3 + 0);
        this->vertex_indices.push_back(i * 3 + 1);
        this->vertex_indices.push_back(i * 3 + 2);
        glm::vec3 v, n;
        v = glm::vec3(v1.x(), v1.y(), v1.z());
        n = glm::vec3(norm.x(), norm.y(), norm.z());
        this->data.push_back(ChOpenGLVertexAttributesPN(v, n));
        v = glm::vec3(v2.x(), v2.y(), v2.z());
        this->data.push_back(ChOpenGLVertexAttributesPN(v, n));
        v = glm::vec3(v3.x(), v3.y(), v3.z());
        this->data.push_back(ChOpenGLVertexAttributesPN(v, n));
    }
    ambient = mat.ambient_color;
    diffuse = mat.diffuse_color;
    specular = mat.specular_color;
    //
    //   std::vector<ChVector<int> >& indices =
    //   tri_mesh->GetMesh().getIndicesVertexes();
    //   std::vector<ChVector<double> >& vertices =
    //   tri_mesh->GetMesh().getCoordsVertices();
    //   std::vector<ChVector<double> >& normals =
    //   tri_mesh->GetMesh().getCoordsNormals();
    //

    //   this->vertex_indices.resize(indices.size() * 3);
    //   for (unsigned int i = 0; i < indices.size(); i++) {
    //
    //      this->vertex_indices[i * 3 + 0] = indices[i].x;
    //      this->vertex_indices[i * 3 + 1] = indices[i].y;
    //      this->vertex_indices[i * 3 + 2] = indices[i].z;
    //   }
    //   for (unsigned int i = 0; i < vertices.size(); i++) {
    //      glm::vec3 n;
    //      glm::vec3 v(vertices[i].x, vertices[i].y, vertices[i].z);
    //      if (normals.size() > 0) {
    //         n = glm::vec3(normals[i].x, normals[i].y, normals[i].z);
    //      } else {
    //         n = glm::vec3(1, 0, 0);
    //      }
    //
    //      this->data.push_back(ChOpenGLVertexAttributesPN(v, n));
    //   }

    PostInitialize();

    if (GLReturnedError("ChOpenGLMesh::Initialize - on exit")) {
        return false;
    }

    return true;
}

bool ChOpenGLMesh::PostInitialize() {
    if (GLReturnedError("ChOpenGLMesh::PostInitialize - on entry"))
        return false;
    // Generation complete bind everything!
    if (!this->PostGLInitialize((GLuint*)(&this->data[0]), this->data.size() * sizeof(ChOpenGLVertexAttributesPN))) {
        return false;
    }

    // mat.ambient_color, mat.diffuse_color, mat.specular_color,

    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);
    glEnableVertexAttribArray(2);
    glEnableVertexAttribArray(3);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(ChOpenGLVertexAttributesPN),
                          (GLvoid*)(sizeof(vec3) * 0));  // Position
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(ChOpenGLVertexAttributesPN),
                          (GLvoid*)(sizeof(vec3) * 1));  // Normal

    glGenBuffers(1, &vertex_ambient_handle);
    glBindBuffer(GL_ARRAY_BUFFER, vertex_ambient_handle);
    // glBufferData(GL_ARRAY_BUFFER, sizeof(vec3), &this->ambient,
    // GL_STATIC_DRAW);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, NULL);
    glVertexAttribDivisor(2, 1);

    glGenBuffers(1, &vertex_diffuse_handle);
    glBindBuffer(GL_ARRAY_BUFFER, vertex_diffuse_handle);
    // glBufferData(GL_ARRAY_BUFFER, sizeof(vec3), &this->diffuse,
    // GL_STATIC_DRAW);
    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 0, NULL);
    glVertexAttribDivisor(3, 1);

    int model_loc = 4;
    glGenBuffers(1, &vertex_model_handle);
    glBindBuffer(GL_ARRAY_BUFFER, vertex_model_handle);
    for (int i = 0; i < 4; i++) {
        // Set up the vertex attribute
        glVertexAttribPointer(model_loc + i,  // Location
                              4, GL_FLOAT,
                              GL_FALSE,                    // vec4
                              sizeof(mat4),                // Stride
                              (void*)(sizeof(vec4) * i));  // Start offset
        // Enable it
        glEnableVertexAttribArray(model_loc + i);
        // Make it instanced
        glVertexAttribDivisor(model_loc + i, 1);
    }

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    if (GLReturnedError("ChOpenGLMesh::PostInitialize - on exit"))
        return false;

    return true;
}
void ChOpenGLMesh::TakeDown() {
    // Clean up the vertex arrtibute data
    this->data.clear();

    super::TakeDown();
}

/*  A note about the index arrays.

 In this example, the index arrays are unsigned ints. If you know
 for certain that the number of vertices will be small enough, you
 can change the index array type to shorts or bytes. This will have
 the two fold benefit of using less storage and transferring fewer
 bytes.
 */
void ChOpenGLMesh::Update(std::vector<glm::mat4>& model) {
    std::vector<glm::vec3> a(model.size()), d(model.size()), s(model.size());
    for (int i = 0; i < model.size(); i++) {
        a[i] = ambient;
        d[i] = diffuse;
    }

    glBindBuffer(GL_ARRAY_BUFFER, vertex_ambient_handle);
    glBufferData(GL_ARRAY_BUFFER, a.size() * sizeof(vec3), &a[0], GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, vertex_diffuse_handle);
    glBufferData(GL_ARRAY_BUFFER, d.size() * sizeof(vec3), &d[0], GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, vertex_model_handle);
    glBufferData(GL_ARRAY_BUFFER, model.size() * sizeof(mat4), &model[0], GL_DYNAMIC_DRAW);

    size = (int)model.size();
}
void ChOpenGLMesh::Draw(const mat4& projection, const mat4& view) {
    if (GLReturnedError("ChOpenGLMesh::Draw - on entry"))
        return;

    // Enable the shader
    shader->Use();
    GLReturnedError("ChOpenGLMesh::Draw - after use");
    // Send our common uniforms to the shader
    shader->CommonSetup(value_ptr(projection), value_ptr(view));

    GLReturnedError("ChOpenGLMesh::Draw - after common setup");
    // Bind and draw! (in this case we draw as triangles)
    glBindVertexArray(this->vertex_array_handle);
    glBindBuffer(GL_ARRAY_BUFFER, vertex_data_handle);
    glBindBuffer(GL_ARRAY_BUFFER, vertex_ambient_handle);
    glBindBuffer(GL_ARRAY_BUFFER, vertex_diffuse_handle);
    glBindBuffer(GL_ARRAY_BUFFER, vertex_model_handle);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->vertex_element_handle);

    GLReturnedError("ChOpenGLMesh::Draw - before draw");
    glDrawElementsInstanced(GL_TRIANGLES, (GLsizei)this->vertex_indices.size(), GL_UNSIGNED_INT, (void*)0, size);
    GLReturnedError("ChOpenGLMesh::Draw - after draw");
    // unbind everything and cleanup
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glUseProgram(0);

    if (GLReturnedError("ChOpenGLMesh::Draw - on exit"))
        return;
}
}
}
