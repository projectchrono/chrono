// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2022 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Rainer Gericke, Radu Serban
// =============================================================================

#include <cmath>

#include "chrono_vsg/shapes/ShapeBuilder.h"
#include "chrono_vsg/shapes/ShaderUtils.h"

#include "chrono_vsg/utils/ChConversionsVSG.h"

#include "chrono_thirdparty/stb/stb_image.h"

using std::sin;
using std::cos;

namespace chrono {
namespace vsg3d {

ShapeBuilder::ShapeBuilder(vsg::ref_ptr<vsg::Options> options, int num_divs) : m_options(options) {
    // Create the primitive shape builders
    m_box_data = std::make_unique<BoxShapeData>();
    m_die_data = std::make_unique<DieShapeData>();
    m_sphere_data = std::make_unique<SphereShapeData>(num_divs);
    m_cylinder_data = std::make_unique<CylinderShapeData>(num_divs);
    m_cone_data = std::make_unique<ConeShapeData>(num_divs);
    m_capsule_data = std::make_unique<CapsuleShapeData>(num_divs);
}

void ShapeBuilder::assignCompileTraversal(vsg::ref_ptr<vsg::CompileTraversal> ct) {
    m_compileTraversal = ct;
}

vsg::ref_ptr<vsg::Group> ShapeBuilder::CreatePbrShape(vsg::ref_ptr<vsg::vec3Array>& vertices,
                                                      vsg::ref_ptr<vsg::vec3Array>& normals,
                                                      vsg::ref_ptr<vsg::vec2Array>& texcoords,
                                                      vsg::ref_ptr<vsg::ushortArray>& indices,
                                                      std::shared_ptr<ChVisualMaterial> material,
                                                      vsg::ref_ptr<vsg::MatrixTransform> transform,
                                                      bool wireframe) {
    const uint32_t instanceCount = 1;

    // apply texture scaling
    for (size_t i = 0; i < texcoords->size(); i++) {
        vsg::vec2 tx = texcoords->at(i);
        tx = vsg::vec2(tx.x * material->GetTextureScale().x(), tx.y * material->GetTextureScale().y());
        texcoords->set(i, tx);
    }

    auto colors =
        vsg::vec4Array::create(vertices->size(), vsg::vec4CH(material->GetDiffuseColor(), material->GetOpacity()));
    auto scenegraph = vsg::Group::create();
    auto stategraph = createPbrStateGroup(m_options, material, wireframe);
    transform->subgraphRequiresLocalFrustum = false;

    // setup geometry
    auto vid = vsg::VertexIndexDraw::create();

    vsg::DataList arrays;
    arrays.push_back(vertices);
    if (normals)
        arrays.push_back(normals);
    if (texcoords)
        arrays.push_back(texcoords);
    if (colors)
        arrays.push_back(colors);
    vid->assignArrays(arrays);

    vid->assignIndices(indices);
    vid->indexCount = static_cast<uint32_t>(indices->size());
    vid->instanceCount = instanceCount;

    stategraph->addChild(vid);
    transform->addChild(stategraph);
    scenegraph->addChild(transform);

    if (m_compileTraversal)
        m_compileTraversal->compile(scenegraph);

    return scenegraph;
}

vsg::ref_ptr<vsg::Group> ShapeBuilder::CreatePbrShape(ShapeType shape_type,
                                                      std::shared_ptr<ChVisualMaterial> material,
                                                      vsg::ref_ptr<vsg::MatrixTransform> transform,
                                                      bool wireframe) {
    vsg::ref_ptr<vsg::vec3Array> vertices;
    vsg::ref_ptr<vsg::vec3Array> normals;
    vsg::ref_ptr<vsg::vec2Array> texcoords;
    vsg::ref_ptr<vsg::ushortArray> indices;

    // Important:
    // the unique texcoords cannot be used directly to allow individual scaling
    // a copy is taken therefore
    switch (shape_type) {
        case ShapeType::BOX:
            vertices = m_box_data->vertices;
            normals = m_box_data->normals;
            texcoords = vsg::vec2Array::create(m_box_data->texcoords->size());
            for (size_t i = 0; i < m_box_data->texcoords->size(); i++) {
                texcoords->set(i, m_box_data->texcoords->at(i));
            }
            indices = m_box_data->indices;
            break;
        case ShapeType::DIE:
            vertices = m_die_data->vertices;
            normals = m_die_data->normals;
            texcoords = vsg::vec2Array::create(m_die_data->texcoords->size());
            for (size_t i = 0; i < m_die_data->texcoords->size(); i++) {
                texcoords->set(i, m_die_data->texcoords->at(i));
            }
            indices = m_die_data->indices;
            break;
        case ShapeType::SPHERE:
            vertices = m_sphere_data->vertices;
            normals = m_sphere_data->normals;
            texcoords = vsg::vec2Array::create(m_sphere_data->texcoords->size());
            for (size_t i = 0; i < m_sphere_data->texcoords->size(); i++) {
                texcoords->set(i, m_sphere_data->texcoords->at(i));
            }
            indices = m_sphere_data->indices;
            break;
        case ShapeType::CYLINDER:
            vertices = m_cylinder_data->vertices;
            normals = m_cylinder_data->normals;
            texcoords = vsg::vec2Array::create(m_cylinder_data->texcoords->size());
            for (size_t i = 0; i < m_cylinder_data->texcoords->size(); i++) {
                texcoords->set(i, m_cylinder_data->texcoords->at(i));
            }
            indices = m_cylinder_data->indices;
            break;
        case ShapeType::CAPSULE:
            vertices = m_capsule_data->vertices;
            normals = m_capsule_data->normals;
            texcoords = vsg::vec2Array::create(m_capsule_data->texcoords->size());
            for (size_t i = 0; i < m_capsule_data->texcoords->size(); i++) {
                texcoords->set(i, m_capsule_data->texcoords->at(i));
            }
            indices = m_capsule_data->indices;
            break;
        case ShapeType::CONE:
            vertices = m_cone_data->vertices;
            normals = m_cone_data->normals;
            texcoords = vsg::vec2Array::create(m_cone_data->texcoords->size());
            for (size_t i = 0; i < m_cone_data->texcoords->size(); i++) {
                texcoords->set(i, m_cone_data->texcoords->at(i));
            }
            indices = m_cone_data->indices;
            break;
    }
    auto scenegraph = CreatePbrShape(vertices, normals, texcoords, indices, material, transform, wireframe);
    return scenegraph;
}

// -----------------------------------------------------------------------------

void GetSurfaceShapeData(std::shared_ptr<ChSurface> geometry,
                         int resolution_u,
                         int resolution_v,
                         vsg::ref_ptr<vsg::vec3Array>& vertices,
                         vsg::ref_ptr<vsg::vec3Array>& normals,
                         vsg::ref_ptr<vsg::vec2Array>& texcoords,
                         vsg::ref_ptr<vsg::ushortArray>& indices) {
    auto sections_u = resolution_u * 4;
    auto sections_v = resolution_v * 4;
    auto nvertices = (sections_u + 1) * (sections_v + 1);
    auto ntriangles = (sections_u) * (sections_v) * 2;
    auto nindices = ntriangles * 3;

    vertices = vsg::vec3Array::create(nvertices);
    normals = vsg::vec3Array::create(nvertices);
    texcoords = vsg::vec2Array::create(nvertices);
    indices = vsg::ushortArray::create(nindices);

    int itri = 0;

    for (auto iv = 0; iv <= sections_v; ++iv) {
        double mV = iv / (double)sections_v;  // v abscissa

        for (auto iu = 0; iu <= sections_u; ++iu) {
            double mU = iu / (double)sections_u;  // u abscissa

            ChVector3d P = geometry->Evaluate(mU, mV);
            ////P = vis->Pos + vis->Rot * P;

            ChVector3d N = geometry->GetNormal(mU, mV);
            ////N = vis->Rot * N;

            // create two triangles per uv increment
            vertices->set(iu + iv * (sections_u + 1), vsg::vec3(P.x(), P.y(), P.z()));
            normals->set(iu + iv * (sections_u + 1), vsg::vec3(N.x(), N.y(), N.z()));
            texcoords->set(iu + iv * (sections_u + 1), vsg::vec2(mU, mV));

            if (iu > 0 && iv > 0) {
                indices->set(0 + itri * 3, iu - 1 + iv * (sections_u + 1));
                indices->set(1 + itri * 3, iu - 1 + (iv - 1) * (sections_u + 1));
                indices->set(2 + itri * 3, iu + iv * (sections_u + 1));
                ++itri;
                indices->set(0 + itri * 3, iu - 1 + (iv - 1) * (sections_u + 1));
                indices->set(1 + itri * 3, iu + (iv - 1) * (sections_u + 1));
                indices->set(2 + itri * 3, iu + iv * (sections_u + 1));
                ++itri;
            }
        }
    }
}

vsg::ref_ptr<vsg::Group> ShapeBuilder::CreatePbrSurfaceShape(std::shared_ptr<ChSurface> geometry,
                                                             std::shared_ptr<ChVisualMaterial> material,
                                                             vsg::ref_ptr<vsg::MatrixTransform> transform,
                                                             int resolution_u,
                                                             int resolution_v,
                                                             bool wireframe) {
    vsg::ref_ptr<vsg::vec3Array> vertices;
    vsg::ref_ptr<vsg::vec3Array> normals;
    vsg::ref_ptr<vsg::vec2Array> texcoords;
    vsg::ref_ptr<vsg::ushortArray> indices;
    GetSurfaceShapeData(geometry, resolution_u, resolution_v, vertices, normals, texcoords, indices);
    auto scenegraph = CreatePbrShape(vertices, normals, texcoords, indices, material, transform, wireframe);
    return scenegraph;
}

// -----------------------------------------------------------------------------

vsg::ref_ptr<vsg::Group> ShapeBuilder::CreateTrimeshColShape(std::shared_ptr<ChTriangleMeshConnected> mesh,
                                                             vsg::ref_ptr<vsg::MatrixTransform> transform,
                                                             const ChColor& default_color,
                                                             bool wireframe) {
    auto scenegraph = vsg::Group::create();
    auto chronoMat = chrono_types::make_shared<ChVisualMaterial>();

    const auto& vertices = mesh->GetCoordsVertices();
    const auto& normals = mesh->GetCoordsNormals();
    const auto& uvs = mesh->GetCoordsUV();
    const auto& colors = mesh->GetCoordsColors();

    const auto& v_indices = mesh->GetIndicesVertexes();
    const auto& n_indices = mesh->GetIndicesNormals();
    const auto& uv_indices = mesh->GetIndicesUV();
    const auto& c_indices = mesh->GetIndicesColors();

    unsigned int ntriangles = (unsigned int)v_indices.size();

    // Set the Irrlicht vertex and index buffers for the mesh buffer
    ChVector3d t[3];   // positions of triangle vertices
    ChVector3d n[3];   // normals at the triangle vertices
    ChVector2d uv[3];  // UV coordinates at the triangle vertices
    ChColor col[3];    // color coordinates at the triangle vertices

    std::vector<ChVector3d> tmp_vertices;
    std::vector<ChVector3d> tmp_normals;
    std::vector<ChVector2d> tmp_texcoords;
    std::vector<ChColor> tmp_colors;

    for (unsigned int itri = 0; itri < ntriangles; itri++) {
        for (int iv = 0; iv < 3; iv++)
            t[iv] = vertices[v_indices[itri][iv]];

        if (n_indices.size() == ntriangles) {
            for (int iv = 0; iv < 3; iv++)
                n[iv] = normals[n_indices[itri][iv]];
        } else {
            n[0] = Vcross(t[1] - t[0], t[2] - t[0]).GetNormalized();
            n[1] = n[0];
            n[2] = n[0];
        }

        if (uv_indices.size() == ntriangles) {
            for (int iv = 0; iv < 3; iv++)
                uv[iv] = uvs[uv_indices[itri][iv]];
        } else if (uv_indices.size() == 0 && uvs.size() == vertices.size()) {
            for (int iv = 0; iv < 3; iv++)
                uv[iv] = uvs[v_indices[itri][iv]];
        }

        if (c_indices.size() == ntriangles) {
            for (int iv = 0; iv < 3; iv++)
                col[iv] = colors[c_indices[itri][iv]];
        } else if (c_indices.size() == 0 && colors.size() == vertices.size()) {
            for (int iv = 0; iv < 3; iv++)
                col[iv] = colors[v_indices[itri][iv]];
        } else {
            for (int iv = 0; iv < 3; iv++)
                col[iv] = default_color;
        }

        for (int iv = 0; iv < 3; iv++) {
            tmp_vertices.push_back(t[iv]);
            tmp_normals.push_back(n[iv]);
            tmp_texcoords.push_back(uv[iv]);
            tmp_colors.push_back(col[iv]);
        }
    }
    // create and fill the vsg buffers
    size_t nVert = tmp_vertices.size();
    vsg::ref_ptr<vsg::vec3Array> vsg_vertices = vsg::vec3Array::create(nVert);
    vsg::ref_ptr<vsg::vec3Array> vsg_normals = vsg::vec3Array::create(nVert);
    vsg::ref_ptr<vsg::vec2Array> vsg_texcoords = vsg::vec2Array::create(nVert);
    vsg::ref_ptr<vsg::uintArray> vsg_indices = vsg::uintArray::create(nVert);
    vsg::ref_ptr<vsg::vec4Array> vsg_colors = vsg::vec4Array::create(nVert);
    for (size_t k = 0; k < nVert; k++) {
        vsg_vertices->set(k, vsg::vec3CH(tmp_vertices[k]));
        vsg_normals->set(k, vsg::vec3CH(tmp_normals[k]));
        // seems to work with v-coordinate flipped on VSG
        vsg_texcoords->set(k, vsg::vec2(tmp_texcoords[k].x(), tmp_texcoords[k].y()));
        vsg_colors->set(k, vsg::vec4CH(tmp_colors[k]));
        vsg_indices->set(k, (unsigned int)k);
    }

    vsg::DataList arrays;
    // setup geometry
    auto vid = vsg::VertexIndexDraw::create();

    arrays.push_back(vsg_vertices);
    if (vsg_normals)
        arrays.push_back(vsg_normals);
    if (vsg_texcoords)
        arrays.push_back(vsg_texcoords);
    if (vsg_colors)
        arrays.push_back(vsg_colors);
    vid->assignArrays(arrays);

    vid->assignIndices(vsg_indices);
    vid->indexCount = static_cast<uint32_t>(vsg_indices->size());
    vid->instanceCount = 1;

    auto stategraph = createPbrStateGroup(m_options, chronoMat, wireframe);
    stategraph->addChild(vid);
    transform->addChild(stategraph);

    scenegraph->addChild(transform);

    if (m_compileTraversal)
        m_compileTraversal->compile(scenegraph);

    return scenegraph;
}

vsg::ref_ptr<vsg::Group> ShapeBuilder::CreateTrimeshColAvgShape(std::shared_ptr<ChTriangleMeshConnected> mesh,
                                                                vsg::ref_ptr<vsg::MatrixTransform> transform,
                                                                const ChColor& default_color,
                                                                bool wireframe) {
    auto scenegraph = vsg::Group::create();
    auto chronoMat = chrono_types::make_shared<ChVisualMaterial>();

    const auto& vertices = mesh->GetCoordsVertices();
    const auto& normals = mesh->GetCoordsNormals();
    const auto& uvs = mesh->GetCoordsUV();
    const auto& colors = mesh->GetCoordsColors();

    size_t nvertices = vertices.size();
    bool normals_ok = true;
    std::vector<ChVector3d> avg_normals;
    if (nvertices != normals.size()) {
        avg_normals = mesh->getAverageNormals();
        normals_ok = false;
    }
    bool texcoords_ok = true;
    if (nvertices != uvs.size()) {
        texcoords_ok = false;
    }
    bool colors_ok = true;
    if (nvertices != colors.size()) {
        colors_ok = false;
    }

    const auto& v_indices = mesh->GetIndicesVertexes();

    // create and fill the vsg buffers
    vsg::ref_ptr<vsg::vec3Array> vsg_vertices = vsg::vec3Array::create(nvertices);
    vsg::ref_ptr<vsg::vec3Array> vsg_normals = vsg::vec3Array::create(nvertices);
    vsg::ref_ptr<vsg::vec2Array> vsg_texcoords = vsg::vec2Array::create(nvertices);
    vsg::ref_ptr<vsg::uintArray> vsg_indices = vsg::uintArray::create(v_indices.size() * 3);
    vsg::ref_ptr<vsg::vec4Array> vsg_colors = vsg::vec4Array::create(nvertices);
    for (size_t k = 0; k < nvertices; k++) {
        vsg_vertices->set(k, vsg::vec3CH(vertices[k]));
        vsg_normals->set(k, normals_ok ? vsg::vec3CH(normals[k]) : vsg::vec3CH(avg_normals[k]));
        // seems to work with v-coordinate flipped on VSG (??)
        vsg_texcoords->set(k, texcoords_ok ? vsg::vec2(uvs[k].x(), uvs[k].y()) : vsg::vec2CH({0, 0}));
        vsg_colors->set(k, colors_ok ? vsg::vec4CH(colors[k]) : vsg::vec4CH(default_color));
    }
    size_t kk = 0;
    for (size_t k = 0; k < v_indices.size() * 3; k += 3) {
        vsg_indices->set(k, v_indices[kk][0]);
        vsg_indices->set(k + 1, v_indices[kk][1]);
        vsg_indices->set(k + 2, v_indices[kk++][2]);
    }

    vsg::DataList arrays;
    // setup geometry
    auto vid = vsg::VertexIndexDraw::create();

    arrays.push_back(vsg_vertices);
    if (vsg_normals)
        arrays.push_back(vsg_normals);
    if (vsg_texcoords)
        arrays.push_back(vsg_texcoords);
    if (vsg_colors)
        arrays.push_back(vsg_colors);
    vid->assignArrays(arrays);

    vid->assignIndices(vsg_indices);
    vid->indexCount = static_cast<uint32_t>(vsg_indices->size());
    vid->instanceCount = 1;

    auto stategraph = createPbrStateGroup(m_options, chronoMat, wireframe);
    stategraph->addChild(vid);
    transform->addChild(stategraph);

    scenegraph->addChild(transform);

    if (m_compileTraversal)
        m_compileTraversal->compile(scenegraph);

    return scenegraph;
}

vsg::ref_ptr<vsg::Group> ShapeBuilder::CreateTrimeshPbrMatShape(std::shared_ptr<ChTriangleMeshConnected> mesh,
                                                                vsg::ref_ptr<vsg::MatrixTransform> transform,
                                                                const std::vector<ChVisualMaterialSharedPtr>& materials,
                                                                bool wireframe) {
    auto scenegraph = vsg::Group::create();
    transform->subgraphRequiresLocalFrustum = false;
    scenegraph->addChild(transform);

    int num_materials = (int)materials.size();

    const auto& vertices = mesh->GetCoordsVertices();
    const auto& normals = mesh->GetCoordsNormals();
    const auto& uvs = mesh->GetCoordsUV();

    const auto& v_indices = mesh->GetIndicesVertexes();
    const auto& n_indices = mesh->GetIndicesNormals();
    const auto& uv_indices = mesh->GetIndicesUV();
    const auto& m_indices = mesh->GetIndicesMaterials();

    size_t ntriangles_all = (unsigned int)v_indices.size();

    // Count number of faces assigned to each material (buffer)
    std::vector<size_t> nfaces_per_buffer;
    if (m_indices.empty()) {
        assert(num_materials == 1);
        nfaces_per_buffer.push_back(ntriangles_all);
    } else {
        for (size_t imat = 0; imat < num_materials; imat++) {
            auto count = std::count(m_indices.begin(), m_indices.end(), imat);
            nfaces_per_buffer.push_back(count);
        }
    }

    for (size_t imat = 0; imat < num_materials; imat++) {
        const auto& chronoMat = materials[imat];
        vsg::ref_ptr<vsg::ShaderSet> shaderSet = createPbrShaderSet(m_options, chronoMat);

        std::vector<ChVector3d> tmp_vertices;
        std::vector<ChVector3d> tmp_normals;
        std::vector<ChVector2d> tmp_texcoords;
        // Set the VSG vertex and index buffers for this material
        ChVector3d t[3];   // positions of triangle vertices
        ChVector3d n[3];   // normals at the triangle vertices
        ChVector2d uv[3];  // UV coordinates at the triangle vertices
        for (size_t itri = 0; itri < ntriangles_all; itri++) {
            if (!m_indices.empty() && m_indices[itri] != imat)
                continue;

            for (int iv = 0; iv < 3; iv++)
                t[iv] = vertices[v_indices[itri][iv]];

            if (n_indices.size() == ntriangles_all) {
                for (int iv = 0; iv < 3; iv++)
                    n[iv] = normals[n_indices[itri][iv]];
            } else {
                n[0] = Vcross(t[1] - t[0], t[2] - t[0]).GetNormalized();
                n[1] = n[0];
                n[2] = n[0];
            }

            if (uv_indices.size() == ntriangles_all) {
                for (int iv = 0; iv < 3; iv++)
                    uv[iv] = uvs[uv_indices[itri][iv]];
            } else {
                for (int iv = 0; iv < 3; iv++)
                    uv[iv] = 0.0;
            }
            for (int j = 0; j < 3; j++) {
                tmp_vertices.push_back(t[j]);
                tmp_normals.push_back(n[j]);
                tmp_texcoords.push_back(uv[j]);
            }
        }

        // create and fill the vsg buffers
        size_t nVert = tmp_vertices.size();
        vsg::ref_ptr<vsg::vec3Array> vsg_vertices = vsg::vec3Array::create(nVert);
        vsg::ref_ptr<vsg::vec3Array> vsg_normals = vsg::vec3Array::create(nVert);
        vsg::ref_ptr<vsg::vec2Array> vsg_texcoords = vsg::vec2Array::create(nVert);
        vsg::ref_ptr<vsg::uintArray> vsg_indices = vsg::uintArray::create(nVert);
        for (size_t k = 0; k < nVert; k++) {
            vsg_vertices->set(k, vsg::vec3CH(tmp_vertices[k]));
            vsg_normals->set(k, vsg::vec3CH(tmp_normals[k]));
            // apply texture scale
            vsg_texcoords->set(k, vsg::vec2(tmp_texcoords[k].x() * chronoMat->GetTextureScale().x(),
                                            (1.0 - tmp_texcoords[k].y()) * chronoMat->GetTextureScale().y()));
            vsg_indices->set(k, (unsigned int)k);
        }
        auto colors = vsg::vec4Array::create(vsg_vertices->size(), vsg::vec4{1.0f, 1.0f, 1.0f, 1.0f});

        vsg::DataList arrays;
        // setup geometry
        auto vid = vsg::VertexIndexDraw::create();

        arrays.push_back(vsg_vertices);
        if (vsg_normals)
            arrays.push_back(vsg_normals);
        if (vsg_texcoords)
            arrays.push_back(vsg_texcoords);
        if (colors)
            arrays.push_back(colors);
        vid->assignArrays(arrays);

        vid->assignIndices(vsg_indices);
        vid->indexCount = static_cast<uint32_t>(vsg_indices->size());
        vid->instanceCount = 1;

        auto stategraph = createPbrStateGroup(m_options, chronoMat, wireframe);
        stategraph->addChild(vid);
        transform->addChild(stategraph);
    }  // imat

    if (m_compileTraversal)
        m_compileTraversal->compile(scenegraph);

    return scenegraph;
}

vsg::ref_ptr<vsg::Group> ShapeBuilder::createFrameSymbol(vsg::ref_ptr<vsg::MatrixTransform> transform,
                                                         float color_factor,
                                                         float line_width,
                                                         bool skipZbuffer) {
    auto scenegraph = vsg::Group::create();
    scenegraph->addChild(transform);

    // Set red, green, and blue colors at specified darkness level
    ChColor R(1, 0, 0);
    ChColor G(0, 1, 0);
    ChColor B(0, 0, 1);

    auto hsvR = ChColor::RGB2HSV(R);
    hsvR[2] *= color_factor;
    R = ChColor::HSV2RGB(hsvR);

    auto hsvG = ChColor::RGB2HSV(G);
    hsvG[2] *= color_factor;
    G = ChColor::HSV2RGB(hsvG);

    auto hsvB = ChColor::RGB2HSV(B);
    hsvB[2] *= color_factor;
    B = ChColor::HSV2RGB(hsvB);

    // calculate vertices
    const int num_points = 6;
    auto vertices = vsg::vec3Array::create(num_points);
    auto colors = vsg::vec3Array::create(num_points);

    vertices->set(0, vsg::vec3(0, 0, 0));
    vertices->set(1, vsg::vec3(1, 0, 0));
    vertices->set(2, vsg::vec3(0, 0, 0));
    vertices->set(3, vsg::vec3(0, 1, 0));
    vertices->set(4, vsg::vec3(0, 0, 0));
    vertices->set(5, vsg::vec3(0, 0, 1));

    colors->set(0, vsg::vec3CH(R));
    colors->set(1, vsg::vec3CH(R));
    colors->set(2, vsg::vec3CH(G));
    colors->set(3, vsg::vec3CH(G));
    colors->set(4, vsg::vec3CH(B));
    colors->set(5, vsg::vec3CH(B));

    auto stategraph = createLineStateGroup(m_options, VK_PRIMITIVE_TOPOLOGY_LINE_LIST, line_width, skipZbuffer);

    // setup vertex index draw
    auto vd = vsg::VertexDraw::create();

    vsg::DataList arrays;
    arrays.push_back(vertices);
    arrays.push_back(colors);

    vd->assignArrays(arrays);
    vd->vertexCount = num_points;
    vd->instanceCount = 1;

    stategraph->addChild(vd);
    transform->addChild(stategraph);

    if (m_compileTraversal)
        m_compileTraversal->compile(scenegraph);
    return scenegraph;
}

vsg::ref_ptr<vsg::Group> ShapeBuilder::CreateLineShape(std::shared_ptr<ChLine> geometry,
                                                       std::shared_ptr<ChVisualMaterial> material,
                                                       vsg::ref_ptr<vsg::MatrixTransform> transform,
                                                       unsigned int num_points) {
    auto scenegraph = vsg::Group::create();
    transform->subgraphRequiresLocalFrustum = false;
    scenegraph->addChild(transform);

    // calculate vertices
    double maxU = 1;
    if (auto mline_path = std::dynamic_pointer_cast<ChLinePath>(geometry))
        maxU = mline_path->GetPathDuration();
    assert(num_points > 2);
    double ustep = maxU / (num_points - 1);
    auto vertices = vsg::vec3Array::create(num_points);
    auto colors = vsg::vec3Array::create(num_points);
    for (unsigned int i = 0; i < num_points; i++) {
        double u = i * ustep;
        auto pos = geometry->Evaluate(u);
        vertices->set(i, vsg::vec3CH(pos));
        colors->set(i, vsg::vec3CH(material->GetDiffuseColor()));
    }

    auto stategraph = createLineStateGroup(m_options, VK_PRIMITIVE_TOPOLOGY_LINE_STRIP, 1.0f);

    // setup vertex index draw
    auto vd = vsg::VertexDraw::create();

    vsg::DataList arrays;
    arrays.push_back(vertices);
    arrays.push_back(colors);

    vd->assignArrays(arrays);
    vd->vertexCount = (uint32_t)num_points;
    vd->instanceCount = 1;

    stategraph->addChild(vd);

    transform->addChild(stategraph);

    if (m_compileTraversal)
        m_compileTraversal->compile(scenegraph);
    return scenegraph;
}

vsg::ref_ptr<vsg::Group> ShapeBuilder::CreatePathShape(std::shared_ptr<ChLinePath> geometry,
                                                       std::shared_ptr<ChVisualMaterial> material,
                                                       vsg::ref_ptr<vsg::MatrixTransform> transform,
                                                       unsigned int num_points) {
    auto scenegraph = vsg::Group::create();
    transform->subgraphRequiresLocalFrustum = false;
    scenegraph->addChild(transform);

    // calculate vertices
    assert(num_points > 2);
    double maxU = geometry->GetPathDuration();
    double ustep = maxU / (num_points - 1);
    auto vertices = vsg::vec3Array::create(num_points);
    auto colors = vsg::vec3Array::create(num_points);
    for (unsigned int i = 0; i < num_points; i++) {
        double u = i * ustep;
        auto pos = geometry->Evaluate(u);
        vertices->set(i, vsg::vec3CH(pos));
        colors->set(i, vsg::vec3CH(material->GetDiffuseColor()));
    }

    auto stategraph = createLineStateGroup(m_options, VK_PRIMITIVE_TOPOLOGY_LINE_STRIP, 1.0f);

    // setup vertex index draw
    auto vd = vsg::VertexDraw::create();

    vsg::DataList arrays;
    arrays.push_back(vertices);
    arrays.push_back(colors);

    vd->assignArrays(arrays);
    vd->vertexCount = (uint32_t)num_points;
    vd->instanceCount = 1;

    stategraph->addChild(vd);

    transform->addChild(stategraph);

    if (m_compileTraversal)
        m_compileTraversal->compile(scenegraph);
    return scenegraph;
}

vsg::ref_ptr<vsg::Group> ShapeBuilder::CreateSpringShape(std::shared_ptr<ChVisualMaterial> material,
                                                         vsg::ref_ptr<vsg::MatrixTransform> transform,
                                                         size_t num_points,
                                                         double turns,
                                                         float line_width) {
    auto scenegraph = vsg::Group::create();
    scenegraph->addChild(transform);

    // calculate vertices
    assert(num_points > 2);
    auto vertices = vsg::vec3Array::create(num_points);
    auto colors = vsg::vec3Array::create(num_points);
    auto cv = vsg::vec3CH(material->GetDiffuseColor());
    double length = 1;
    double phase = 0.0;
    double height = 0.0;
    vsg::vec3 p(0, -length / 2, 0);
    for (int iu = 0; iu < num_points; iu++) {
        phase = turns * CH_2PI * (double)iu / (double)num_points;
        height = length * ((double)iu / (double)num_points);
        vsg::vec3 pos;
        pos = p + vsg::vec3(cos(phase), height, sin(phase));
        vertices->set(iu, pos);
        colors->set(iu, cv);
    }

    auto stategraph = createLineStateGroup(m_options, VK_PRIMITIVE_TOPOLOGY_LINE_STRIP, line_width);

    // setup vertex index draw
    auto vd = vsg::VertexDraw::create();

    vsg::DataList arrays;
    arrays.push_back(vertices);
    arrays.push_back(colors);

    vd->assignArrays(arrays);
    vd->vertexCount = (uint32_t)num_points;
    vd->instanceCount = 1;

    stategraph->addChild(vd);

    transform->addChild(stategraph);

    if (m_compileTraversal)
        m_compileTraversal->compile(scenegraph);

    return scenegraph;
}

vsg::ref_ptr<vsg::Group> ShapeBuilder::CreateUnitSegment(std::shared_ptr<ChVisualMaterial> material,
                                                         vsg::ref_ptr<vsg::MatrixTransform> transform,
                                                         float line_width,
                                                         bool skipZbuffer) {
    auto scenegraph = vsg::Group::create();
    scenegraph->addChild(transform);

    // calculate vertices
    uint32_t num_points = 2;
    auto vertices = vsg::vec3Array::create(num_points);
    auto colors = vsg::vec3Array::create(num_points);
    auto cv = vsg::vec3CH(material->GetDiffuseColor());
    double length = 1;
    vertices->set(0, vsg::vec3(0, -length / 2, 0));
    vertices->set(1, vsg::vec3(0, +length / 2, 0));
    colors->set(0, cv);
    colors->set(1, cv);

    auto stategraph = createLineStateGroup(m_options, VK_PRIMITIVE_TOPOLOGY_LINE_STRIP, line_width, skipZbuffer);

    // setup vertex index draw
    auto vd = vsg::VertexDraw::create();

    vsg::DataList arrays;
    arrays.push_back(vertices);
    arrays.push_back(colors);

    vd->assignArrays(arrays);
    vd->vertexCount = num_points;
    vd->instanceCount = 1;

    stategraph->addChild(vd);

    transform->addChild(stategraph);

    if (m_compileTraversal)
        m_compileTraversal->compile(scenegraph);

    return scenegraph;
}

vsg::ref_ptr<vsg::Group> ShapeBuilder::CreateGrid(double ustep,
                                                  double vstep,
                                                  int nu,
                                                  int nv,
                                                  ChCoordsys<> pos,
                                                  ChColor col) {
    auto scenegraph = vsg::Group::create();
    auto transform = vsg::MatrixTransform::create();

    auto p = pos.pos;
    auto r = pos.rot;
    double rotAngle;
    ChVector3d rotAxis;
    r.GetAngleAxis(rotAngle, rotAxis);
    transform->matrix =
        vsg::translate(p.x(), p.y(), p.z()) * vsg::rotate(rotAngle, rotAxis.x(), rotAxis.y(), rotAxis.z());

    scenegraph->addChild(transform);
    // calculate vertices
    std::vector<ChVector3d> v;
    for (int iu = -nu / 2; iu <= nu / 2; iu++) {
        ChVector3d V1(iu * ustep, vstep * (nv / 2), 0);
        ChVector3d V2(iu * ustep, -vstep * (nv / 2), 0);
        v.push_back(V1);
        v.push_back(V2);
    }

    for (int iv = -nv / 2; iv <= nv / 2; iv++) {
        ChVector3d V1(ustep * (nu / 2), iv * vstep, 0);
        ChVector3d V2(-ustep * (nu / 2), iv * vstep, 0);
        v.push_back(V1);
        v.push_back(V2);
    }

    auto num_points = v.size();
    auto vertices = vsg::vec3Array::create(num_points);
    auto colors = vsg::vec3Array::create(num_points);
    auto cv = vsg::vec3(col.R, col.G, col.B);
    colors->set(0, cv);
    for (size_t i = 0; i < num_points; i++) {
        vertices->set(i, vsg::vec3CH(v[i]));
        colors->set(i, cv);
    }

    auto stategraph = createLineStateGroup(m_options, VK_PRIMITIVE_TOPOLOGY_LINE_LIST, 1.0f);

    // setup vertex index draw
    auto vd = vsg::VertexDraw::create();

    vsg::DataList arrays;
    arrays.push_back(vertices);
    arrays.push_back(colors);

    vd->assignArrays(arrays);
    vd->vertexCount = (uint32_t)num_points;
    vd->instanceCount = 1;

    stategraph->addChild(vd);

    transform->addChild(stategraph);

    if (m_compileTraversal)
        m_compileTraversal->compile(scenegraph);
    return scenegraph;
}

// -----------------------------------------------------------------------------
// Tesselation data for primitive shapes
// -----------------------------------------------------------------------------

ShapeBuilder::BoxShapeData::BoxShapeData() {
    const float a = 1.0;
    vertices = vsg::vec3Array::create({{-a, -a, -a}, {a, -a, -a}, {a, -a, a},  {-a, -a, a},  {a, a, -a},  {-a, a, -a},
                                       {-a, a, a},   {a, a, a},   {-a, a, -a}, {-a, -a, -a}, {-a, -a, a}, {-a, a, a},
                                       {a, -a, -a},  {a, a, -a},  {a, a, a},   {a, -a, a},   {a, -a, -a}, {-a, -a, -a},
                                       {-a, a, -a},  {a, a, -a},  {-a, -a, a}, {a, -a, a},   {a, a, a},   {-a, a, a}});

    normals = vsg::vec3Array::create({{0, -1, 0}, {0, -1, 0}, {0, -1, 0}, {0, -1, 0}, {0, 1, 0},  {0, 1, 0},
                                      {0, 1, 0},  {0, 1, 0},  {-1, 0, 0}, {-1, 0, 0}, {-1, 0, 0}, {-1, 0, 0},
                                      {1, 0, 0},  {1, 0, 0},  {1, 0, 0},  {1, 0, 0},  {0, 0, -1}, {0, 0, -1},
                                      {0, 0, -1}, {0, 0, -1}, {0, 0, 1},  {0, 0, 1},  {0, 0, 1},  {0, 0, 1}});

    texcoords = vsg::vec2Array::create({{0, 1}, {1, 1}, {1, 0}, {0, 0}, {0, 1}, {1, 1}, {1, 0}, {0, 0},
                                        {0, 1}, {1, 1}, {1, 0}, {0, 0}, {0, 1}, {1, 1}, {1, 0}, {0, 0},
                                        {0, 1}, {1, 1}, {1, 0}, {0, 0}, {0, 1}, {1, 1}, {1, 0}, {0, 0}});

    indices = vsg::ushortArray::create({0,  1,  2,  0,  2,  3,  4,  5,  6,  4,  6,  7,  8,  9,  10, 8,  10, 11,
                                        12, 13, 14, 12, 14, 15, 16, 17, 18, 16, 18, 19, 20, 21, 22, 20, 22, 23});
}

ShapeBuilder::DieShapeData::DieShapeData() {
    const float a = 1.0;
    vertices = vsg::vec3Array::create({{-a, -a, -a}, {a, -a, -a}, {a, -a, a},  {-a, -a, a},  {a, a, -a},  {-a, a, -a},
                                       {-a, a, a},   {a, a, a},   {-a, a, -a}, {-a, -a, -a}, {-a, -a, a}, {-a, a, a},
                                       {a, -a, -a},  {a, a, -a},  {a, a, a},   {a, -a, a},   {a, -a, -a}, {-a, -a, -a},
                                       {-a, a, -a},  {a, a, -a},  {-a, -a, a}, {a, -a, a},   {a, a, a},   {-a, a, a}});

    normals = vsg::vec3Array::create({{0, -1, 0}, {0, -1, 0}, {0, -1, 0}, {0, -1, 0}, {0, 1, 0},  {0, 1, 0},
                                      {0, 1, 0},  {0, 1, 0},  {-1, 0, 0}, {-1, 0, 0}, {-1, 0, 0}, {-1, 0, 0},
                                      {1, 0, 0},  {1, 0, 0},  {1, 0, 0},  {1, 0, 0},  {0, 0, -1}, {0, 0, -1},
                                      {0, 0, -1}, {0, 0, -1}, {0, 0, 1},  {0, 0, 1},  {0, 0, 1},  {0, 0, 1}});

    texcoords = vsg::vec2Array::create(
        {{0.25f, 0},      {0.5f, 0},        {0.5f, 0.3333f},  {0.25f, 0.3333f}, {0.25f, 0.6666f}, {0.5f, 0.6666f},
         {0.5f, 1},       {0.25f, 1},       {0, 0.3333f},     {0.25f, 0.3333f}, {0.25f, 0.6666f}, {0, 0.6666f},
         {0.5f, 0.3333f}, {0.75f, 0.3333f}, {0.75f, 0.6666f}, {0.5f, 0.6666f},  {0.25f, 0.3333f}, {0.5f, 0.3333f},
         {0.5f, 0.6666f}, {0.25f, 0.6666f}, {0.75f, 0.3333f}, {1, 0.3333f},     {1, 0.6666f},     {0.75f, 0.6666f}});

    indices = vsg::ushortArray::create({0,  1,  2,  0,  2,  3,  4,  5,  6,  4,  6,  7,  8,  9,  10, 8,  10, 11,
                                        12, 13, 14, 12, 14, 15, 16, 17, 18, 16, 18, 19, 20, 21, 22, 20, 22, 23});
}

// sphere
// radius = 1
// center at {0,0,0}
ShapeBuilder::SphereShapeData::SphereShapeData(int num_divs) {
    int nTheta = num_divs / 2;
    int nPhi = num_divs;

    double r = 1.0;
    double dTheta = CH_PI / nTheta;
    double dPhi = CH_2PI / nPhi;

    size_t nv = (nPhi + 1) * (nTheta + 1);
    vertices = vsg::vec3Array::create(nv);
    normals = vsg::vec3Array::create(nv);
    texcoords = vsg::vec2Array::create(nv);

    size_t nf = 2 * nPhi * nTheta;
    indices = vsg::ushortArray::create(3 * nf);

    int v = 0;  // current vertex counter
    for (int iPhi = 0; iPhi <= nPhi; iPhi++) {
        auto phi = iPhi * dPhi;

        for (int iTheta = 0; iTheta <= nTheta; iTheta++) {
            auto theta = iTheta * dTheta;

            double x = r * sin(theta) * cos(phi);
            double y = r * sin(theta) * sin(phi);
            double z = r * cos(theta);
            auto vertex = ChVector3d(x, y, z);
            vertices->set(v, vsg::vec3CH(vertex));
            normals->set(v, vsg::vec3CH(vertex.GetNormalized()));

            ChVector2d t(phi / CH_2PI, theta / CH_PI);
            texcoords->set(v, vsg::vec2CH(t));

            v++;
        }
    }

    int i = 0;  // current index counter
    for (int iPhi = 0; iPhi < nPhi; iPhi++) {
        for (int iTheta = 0; iTheta < nTheta; iTheta++) {
            int k1 = (nTheta + 1) * (iPhi + 0) + (iTheta + 0);
            int k2 = (nTheta + 1) * (iPhi + 0) + (iTheta + 1);
            int k3 = (nTheta + 1) * (iPhi + 1) + (iTheta + 1);
            int k4 = (nTheta + 1) * (iPhi + 1) + (iTheta + 0);

            indices->set(i + 0, k1);
            indices->set(i + 1, k2);
            indices->set(i + 2, k3);

            indices->set(i + 3, k1);
            indices->set(i + 4, k3);
            indices->set(i + 5, k4);

            i += 6;
        }
    }

    ////for (size_t j = 0; j < vertices->size(); j++)
    ////    std::cout << vertices->at(j) << std::endl;
    ////std::cout << std::endl;
    ////for (size_t j = 0; j < normals->size(); j++)
    ////    std::cout << normals->at(j) << std::endl;
    ////std::cout << std::endl;
    ////for (size_t j = 0; j < texcoords->size(); j++)
    ////    std::cout << texcoords->at(j) << std::endl;
    ////std::cout << std::endl;
    ////for (size_t j = 0; j < indices->size(); j++)
    ////    std::cout << indices->at(j) << std::endl;
}

// cylinder
// height = 1
// radius = 1
// bottom at {0,0,-0.5}
// top at    {0,0,+0.5}
ShapeBuilder::CylinderShapeData::CylinderShapeData(int num_divs) {
    int nPhi = num_divs;

    double r = 1.0;
    double h = 0.5;
    double dPhi = CH_2PI / nPhi;

    size_t nv = 4 * (nPhi + 1);
    vertices = vsg::vec3Array::create(nv);
    normals = vsg::vec3Array::create(nv);
    texcoords = vsg::vec2Array::create(nv);

    size_t nf = 2 * nPhi + 2 * (nPhi - 1);
    indices = vsg::ushortArray::create(3 * nf);

    int v = 0;  // current vertex counter
    int i = 0;  // current index counter

    // Cylinder side

    for (int iPhi = 0; iPhi <= nPhi; iPhi++) {
        auto phi = iPhi * dPhi;
        double x = r * cos(phi);
        double y = -r * sin(phi);
        double utex = 1 - phi / CH_PI;

        // top vertices
        vertices->set(nPhi + 1 + v, vsg::vec3(x, y, +h));
        normals->set(nPhi + 1 + v, vsg::vec3(x, y, 0));
        texcoords->set(nPhi + 1 + v, vsg::vec2(utex, 0));

        // bottom vertices
        vertices->set(v, vsg::vec3(x, y, -h));
        normals->set(v, vsg::vec3(x, y, 0));
        texcoords->set(v, vsg::vec2(utex, h / (2 * r)));

        v++;
    }

    for (int iPhi = 0; iPhi < nPhi; iPhi++) {
        int k1 = iPhi;
        int k2 = iPhi + 1;
        int k3 = k1 + nPhi + 1;
        int k4 = k3 + 1;

        indices->set(i + 0, k1);
        indices->set(i + 1, k3);
        indices->set(i + 2, k2);

        indices->set(i + 3, k2);
        indices->set(i + 4, k3);
        indices->set(i + 5, k4);

        i += 6;
    }

    // Cylinder caps

    v = 2 * (nPhi + 1);
    for (int iPhi = 0; iPhi <= nPhi; iPhi++) {
        auto phi = iPhi * dPhi;
        double x = r * cos(phi);
        double y = -r * sin(phi);
        double utex = (sin(phi) + 1) / 2;
        double vtex = (cos(phi) + 1) / 2;

        // bottom vertices
        vertices->set(v, vsg::vec3(x, y, -h));
        normals->set(v, vsg::vec3(0, 0, -1));
        texcoords->set(v, vsg::vec2(utex, vtex));

        // top vertices
        vertices->set(nPhi + 1 + v, vsg::vec3(x, y, +h));
        normals->set(nPhi + 1 + v, vsg::vec3(0, 0, +1));
        texcoords->set(nPhi + 1 + v, vsg::vec2(vtex, utex));

        v++;
    }

    int v_offset = 2 * (nPhi + 1);

    i = 3 * (2 * nPhi);
    for (int iPhi = 1; iPhi < nPhi; iPhi++) {
        int k1 = 0;
        int k2 = iPhi;
        int k3 = iPhi + 1;

        indices->set(i + 0, v_offset + k1);
        indices->set(i + 1, v_offset + k2);
        indices->set(i + 2, v_offset + k3);

        i += 3;
    }

    v_offset = 3 * (nPhi + 1);
    i = 3 * (2 * nPhi + (nPhi - 1));
    for (int iPhi = 1; iPhi < nPhi; iPhi++) {
        int k1 = 0;
        int k2 = iPhi + 1;
        int k3 = iPhi;

        indices->set(i + 0, v_offset + k1);
        indices->set(i + 1, v_offset + k2);
        indices->set(i + 2, v_offset + k3);

        i += 3;
    }

    ////for (size_t j = 0; j < vertices->size(); j++)
    ////    std::cout << vertices->at(j) << std::endl;
    ////std::cout << std::endl;
    ////for (size_t j = 0; j < normals->size(); j++)
    ////    std::cout << normals->at(j) << std::endl;
    ////std::cout << std::endl;
    ////for (size_t j = 0; j < texcoords->size(); j++)
    ////    std::cout << texcoords->at(j) << std::endl;
    ////std::cout << std::endl;
    ////for (size_t j = 0; j < indices->size(); j++)
    ////    std::cout << indices->at(j) << std::endl;
}

// cone
// height = 1
// radius = 1
// bottom at {0,0,0}
// apex at   {0,0,1}
ShapeBuilder::ConeShapeData::ConeShapeData(int num_divs) {
    int nPhi = num_divs;

    double r = 1.0;
    double h = 1.0;
    double dPhi = CH_2PI / nPhi;

    size_t nv = 3 * (nPhi + 1);
    vertices = vsg::vec3Array::create(nv);
    normals = vsg::vec3Array::create(nv);
    texcoords = vsg::vec2Array::create(nv);

    size_t nf = nPhi + (nPhi - 1);
    indices = vsg::ushortArray::create(3 * nf);

    int v = 0;  // current vertex counter
    int i = 0;  // current index counter

    // Cone side

    for (int iPhi = 0; iPhi <= nPhi; iPhi++) {
        auto phi = iPhi * dPhi;
        double x = r * cos(phi);
        double y = -r * sin(phi);
        double utex = 1 - (iPhi * 1.0) / nPhi;

        auto normal = ChVector3d(x, y, r * r / h).GetNormalized();

        // bottom vertices
        vertices->set(v, vsg::vec3(x, y, 0));
        normals->set(v, vsg::vec3CH(normal));
        texcoords->set(v, vsg::vec2(utex, 1));

        // top vertices (all at cone apex)
        vertices->set(nPhi + 1 + v, vsg::vec3(0, 0, h));
        normals->set(nPhi + 1 + v, vsg::vec3CH(normal));
        texcoords->set(nPhi + 1 + v, vsg::vec2(utex, 0));

        v++;
    }

    for (int iPhi = 0; iPhi < nPhi; iPhi++) {
        int k1 = iPhi;
        int k2 = iPhi + 1;
        int k3 = k1 + nPhi + 1;

        indices->set(i + 0, k1);
        indices->set(i + 1, k3);
        indices->set(i + 2, k2);

        i += 3;
    }

    // Cone base

    v = 2 * (nPhi + 1);
    for (int iPhi = 0; iPhi <= nPhi; iPhi++) {
        auto phi = iPhi * dPhi;
        double x = r * cos(phi);
        double y = -r * sin(phi);
        double utex = (sin(phi) + 1) / CH_2PI;
        double vtex = (cos(phi) + 1) / CH_2PI;

        // bottom vertices
        vertices->set(v, vsg::vec3(x, y, 0));
        normals->set(v, vsg::vec3(0, 0, -1));
        texcoords->set(v, vsg::vec2(utex, vtex));

        v++;
    }

    int v_offset = 2 * (nPhi + 1);

    i = 3 * nPhi;
    for (int iPhi = 1; iPhi < nPhi; iPhi++) {
        int k1 = 0;
        int k2 = iPhi;
        int k3 = iPhi + 1;

        indices->set(i + 0, v_offset + k1);
        indices->set(i + 1, v_offset + k2);
        indices->set(i + 2, v_offset + k3);

        i += 3;
    }

    ////for (size_t j = 0; j < vertices->size(); j++)
    ////    std::cout << vertices->at(j) << std::endl;
    ////std::cout << std::endl;
    ////for (size_t j = 0; j < normals->size(); j++)
    ////    std::cout << normals->at(j) << std::endl;
    ////std::cout << std::endl;
    ////for (size_t j = 0; j < texcoords->size(); j++)
    ////    std::cout << texcoords->at(j) << std::endl;
    ////std::cout << std::endl;
    ////for (size_t j = 0; j < indices->size(); j++)
    ////    std::cout << indices->at(j) << std::endl;
}

// capsule
// height = 1 (cylindrical portion)
// radius = 1
// bottom at {0,0,-h-r}
// top at    {0,0,+h+r}
ShapeBuilder::CapsuleShapeData::CapsuleShapeData(int num_divs) {
    int nTheta = num_divs / 4;
    int nPhi = num_divs;

    double r = 1.0;
    double h = 1;

    double dTheta = CH_PI_2 / nTheta;
    double dPhi = CH_2PI / nPhi;

    size_t nv = 2 * (nPhi + 1) + (nPhi + 1) * (nTheta + 1) + (nPhi + 1) * (nTheta + 1);
    vertices = vsg::vec3Array::create(nv);
    normals = vsg::vec3Array::create(nv);
    texcoords = vsg::vec2Array::create(nv);

    size_t nf = 2 * nPhi + 2 * nTheta * nPhi + 2 * nTheta * nPhi;
    indices = vsg::ushortArray::create(3 * nf);

    int v = 0;  // current vertex counter
    int i = 0;  // current index counter

    // Cylindrical section
    for (int iPhi = 0; iPhi <= nPhi; iPhi++) {
        auto phi = iPhi * dPhi;
        double x = r * cos(phi);
        double y = -r * sin(phi);
        double utex = 1 - phi / CH_2PI;

        // top vertices
        vertices->set(nPhi + 1 + v, vsg::vec3(x, y, +h));
        normals->set(nPhi + 1 + v, vsg::vec3(x, y, 0));
        texcoords->set(nPhi + 1 + v, vsg::vec2(utex, 0));

        // bottom vertices
        vertices->set(v, vsg::vec3(x, y, -h));
        normals->set(v, vsg::vec3(x, y, 0));
        texcoords->set(v, vsg::vec2(utex, 1));

        v++;
    }

    for (int iPhi = 0; iPhi < nPhi; iPhi++) {
        int k1 = iPhi;
        int k2 = iPhi + 1;
        int k3 = k1 + nPhi + 1;
        int k4 = k3 + 1;

        indices->set(i + 0, k1);
        indices->set(i + 1, k3);
        indices->set(i + 2, k2);

        indices->set(i + 3, k2);
        indices->set(i + 4, k3);
        indices->set(i + 5, k4);

        i += 6;
    }

    // Top capsule cap

    v = 2 * (nPhi + 1);
    for (int iPhi = 0; iPhi <= nPhi; iPhi++) {
        auto phi = iPhi * dPhi;

        for (int iTheta = 0; iTheta <= nTheta; iTheta++) {
            auto theta = iTheta * dTheta;

            double x = r * cos(theta) * cos(phi);
            double y = r * cos(theta) * sin(phi);
            double z = r * sin(theta);
            auto vertex = ChVector3d(x, y, z);
            vertices->set(v, vsg::vec3CH(vertex + ChVector3d(0, 0, h)));
            normals->set(v, vsg::vec3CH(vertex.GetNormalized()));

            double utex = phi / CH_2PI;
            double vtex = 1 - theta / CH_PI;
            ChVector2d t(utex, vtex);
            texcoords->set(v, vsg::vec2CH(t));

            v++;
        }
    }

    int v_offset = 2 * (nPhi + 1);

    i = 3 * (2 * nPhi);
    for (int iPhi = 0; iPhi < nPhi; iPhi++) {
        for (int iTheta = 0; iTheta < nTheta; iTheta++) {
            int k1 = (nTheta + 1) * (iPhi + 0) + (iTheta + 0);
            int k2 = (nTheta + 1) * (iPhi + 0) + (iTheta + 1);
            int k3 = (nTheta + 1) * (iPhi + 1) + (iTheta + 1);
            int k4 = (nTheta + 1) * (iPhi + 1) + (iTheta + 0);

            indices->set(i + 0, v_offset + k1);
            indices->set(i + 1, v_offset + k3);
            indices->set(i + 2, v_offset + k2);

            indices->set(i + 3, v_offset + k1);
            indices->set(i + 4, v_offset + k4);
            indices->set(i + 5, v_offset + k3);

            i += 6;
        }
    }

    // Bottom capsule cap

    v = 2 * (nPhi + 1) + (nPhi + 1) * (nTheta + 1);
    for (int iPhi = 0; iPhi <= nPhi; iPhi++) {
        auto phi = iPhi * dPhi;

        for (int iTheta = 0; iTheta <= nTheta; iTheta++) {
            auto theta = -iTheta * dTheta;

            double x = r * cos(theta) * cos(phi);
            double y = r * cos(theta) * sin(phi);
            double z = r * sin(theta);
            auto vertex = ChVector3d(x, y, z);
            vertices->set(v, vsg::vec3CH(vertex + ChVector3d(0, 0, -h)));
            normals->set(v, vsg::vec3CH(vertex.GetNormalized()));

            double utex = phi / CH_2PI;
            double vtex = 1 - theta / CH_PI;
            ChVector2d t(utex, vtex);
            texcoords->set(v, vsg::vec2CH(t));

            v++;
        }
    }

    v_offset = 2 * (nPhi + 1) + (nPhi + 1) * (nTheta + 1);

    i = 3 * (2 * nPhi) + 3 * (2 * nPhi * nTheta);
    for (int iPhi = 0; iPhi < nPhi; iPhi++) {
        for (int iTheta = 0; iTheta < nTheta; iTheta++) {
            int k1 = (nTheta + 1) * (iPhi + 0) + (iTheta + 0);
            int k2 = (nTheta + 1) * (iPhi + 0) + (iTheta + 1);
            int k3 = (nTheta + 1) * (iPhi + 1) + (iTheta + 1);
            int k4 = (nTheta + 1) * (iPhi + 1) + (iTheta + 0);

            indices->set(i + 0, v_offset + k1);
            indices->set(i + 1, v_offset + k2);
            indices->set(i + 2, v_offset + k3);

            indices->set(i + 3, v_offset + k1);
            indices->set(i + 4, v_offset + k3);
            indices->set(i + 5, v_offset + k4);

            i += 6;
        }
    }

    ////for (size_t j = 0; j < vertices->size(); j++)
    ////    std::cout << vertices->at(j) << std::endl;
    ////std::cout << std::endl;
    ////for (size_t j = 0; j < normals->size(); j++)
    ////    std::cout << normals->at(j) << std::endl;
    ////std::cout << std::endl;
    ////for (size_t j = 0; j < texcoords->size(); j++)
    ////    std::cout << texcoords->at(j) << std::endl;
    ////std::cout << std::endl;
    ////for (size_t j = 0; j < indices->size(); j++)
    ////    std::cout << indices->at(j) << std::endl;
}

}  // namespace vsg3d
}  // namespace chrono
