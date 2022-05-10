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

#include "chrono/core/ChVector.h"

#include "chrono_irrlicht/ChIrrNodeShape.h"
#include "chrono_irrlicht/ChIrrTools.h"

namespace chrono {
namespace irrlicht {

using namespace irr;

ChIrrNodeShape::ChIrrNodeShape(std::shared_ptr<ChVisualShape> shape, ISceneNode* parent)
    : ISceneNode(parent, parent->getSceneManager(), 0), m_shape(shape), m_initial_update(true) {}

scene::ISceneNode* ChIrrNodeShape::clone(ISceneNode* newParent, scene::ISceneManager* newManager) {
    if (!newParent)
        newParent = Parent;
    if (!newManager)
        newManager = SceneManager;

    ChIrrNodeShape* nb = new ChIrrNodeShape(m_shape, newParent);

    nb->cloneMembers(this, newManager);
    nb->m_box = m_box;
    nb->m_shape = m_shape;

    if (newParent)
        nb->drop();
    return nb;
}

scene::ESCENE_NODE_TYPE ChIrrNodeShape::getType() const {
    return (scene::ESCENE_NODE_TYPE)ESNT_CHIRRNODE_SHAPE;
}

void ChIrrNodeShape::Update() {
    if (!m_shape)
        return;

    if (!m_initial_update) {
        auto asset = std::dynamic_pointer_cast<ChVisualShape>(m_shape);
        if (asset && !asset->IsMutable())
            return;
    }

    if (auto trianglemesh = std::dynamic_pointer_cast<ChTriangleMeshShape>(m_shape)) {
        if (trianglemesh->FixedConnectivity())
            UpdateTriangleMeshFixedConnectivity(trianglemesh);
        else
            UpdateTriangleMesh(trianglemesh);
    } else if (auto glyphs = std::dynamic_pointer_cast<ChGlyphs>(m_shape)) {
        UpdateGlyphs(glyphs);
    } else if (auto surface = std::dynamic_pointer_cast<ChSurfaceShape>(m_shape)) {
        UpdateSurface(surface);
    } else if (auto path_shape = std::dynamic_pointer_cast<ChPathShape>(m_shape)) {
        UpdateLine(path_shape->GetPathGeometry(), path_shape->GetNumRenderPoints());
    } else if (auto line_shape = std::dynamic_pointer_cast<ChLineShape>(m_shape)) {
        UpdateLine(line_shape->GetLineGeometry(), line_shape->GetNumRenderPoints());
    }

    m_initial_update = false;
}

static video::S3DVertex ToIrrlichtVertex(const ChVector<>& pos,
                                         const ChVector<>& nrm,
                                         const ChVector2<>& uv,
                                         const ChColor& col) {
    video::S3DVertex vertex;
    vertex.Pos = core::vector3df((f32)pos.x(), (f32)pos.y(), (f32)pos.z());
    vertex.Normal = core::vector3df((f32)nrm.x(), (f32)nrm.y(), (f32)nrm.z());
    vertex.TCoords = core::vector2df((f32)uv.x(), 1 - (f32)uv.y());
    vertex.Color = tools::ToIrrlichtSColor(col);
    return vertex;
}

static video::S3DVertex ToIrrlichtVertex(const ChVector<>& pos,
                                         const ChVector<>& nrm,
                                         const ChVector2<>& uv,
                                         const video::SColor& col) {
    video::S3DVertex vertex;
    vertex.Pos = core::vector3df((f32)pos.x(), (f32)pos.y(), (f32)pos.z());
    vertex.Normal = core::vector3df((f32)nrm.x(), (f32)nrm.y(), (f32)nrm.z());
    vertex.TCoords = core::vector2df((f32)uv.x(), 1 - (f32)uv.y());  // change handedness for Irrlicht
    vertex.Color = col;
    return vertex;
}

void ChIrrNodeShape::UpdateTriangleMesh(std::shared_ptr<ChTriangleMeshShape> trianglemesh) {
    if (trianglemesh->GetNumMaterials() == 0)
        UpdateTriangleMesh_col(trianglemesh);
    else
        UpdateTriangleMesh_mat(trianglemesh);
}

void ChIrrNodeShape::UpdateTriangleMesh_col(std::shared_ptr<ChTriangleMeshShape> trianglemesh) {
    // Fetch the 1st child, i.e. the mesh
    ISceneNode* irr_node = *(getChildren().begin());
    if (!irr_node || irr_node->getType() != scene::ESNT_MESH)
        return;

    scene::IMeshSceneNode* meshnode = (scene::IMeshSceneNode*)irr_node;
    scene::SMesh* smesh = (scene::SMesh*)meshnode->getMesh();
    int nbuffers = (int)smesh->getMeshBufferCount();

    const auto& mesh = trianglemesh->GetMesh();

    if (nbuffers == 0)
        return;

    const auto& vertices = mesh->getCoordsVertices();
    const auto& normals = mesh->getCoordsNormals();
    const auto& uvs = mesh->getCoordsUV();
    const auto& colors = mesh->getCoordsColors();

    const auto& v_indices = mesh->getIndicesVertexes();
    const auto& n_indices = mesh->getIndicesNormals();
    const auto& uv_indices = mesh->getIndicesUV();
    const auto& c_indices = mesh->getIndicesColors();

    unsigned int ntriangles = (unsigned int)v_indices.size();
    unsigned int nvertexes = ntriangles * 3;

    // Load the one and only mesh buffer
    assert(nbuffers == 1);
    scene::CDynamicMeshBuffer* irr_meshbuffer = (scene::CDynamicMeshBuffer*)smesh->getMeshBuffer(0);
    auto& irr_vertices = irr_meshbuffer->getVertexBuffer();
    auto& irr_indices = irr_meshbuffer->getIndexBuffer();

    // smart inflating of allocated buffers, only if necessary, and once in a while shrinking
    if (irr_indices.allocated_size() > (ntriangles * 3) * 1.5)
        irr_indices.reallocate(0);  // clear();
    if (irr_vertices.allocated_size() > nvertexes * 1.5)
        irr_vertices.reallocate(0);

    irr_indices.set_used(ntriangles * 3);
    irr_vertices.set_used(nvertexes);

    // Set the Irrlicht vertex and index buffers for the mesh buffer
    ChVector<> t[3];    // positions of trianlge vertices
    ChVector<> n[3];    // normals at the triangle vertices
    ChVector2<> uv[3];  // UV coordinates at the triangle vertices
    ChColor col[3];     // color coordinates at the triangle vertices

    auto default_color = trianglemesh->GetColor();

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
            irr_vertices[iv + itri * 3] = ToIrrlichtVertex(t[iv], n[iv], uv[iv], col[iv]);
            irr_indices.setValue(iv + itri * 3, 2 - iv + itri * 3);
        }
    }

    irr_meshbuffer->setDirty();                                  // to force update of hardware buffers
    irr_meshbuffer->setHardwareMappingHint(scene::EHM_DYNAMIC);  // EHM_DYNAMIC for faster hw mapping
    irr_meshbuffer->recalculateBoundingBox();

    meshnode->setAutomaticCulling(scene::EAC_OFF);

    meshnode->setMaterialFlag(video::EMF_WIREFRAME, trianglemesh->IsWireframe());
    meshnode->setMaterialFlag(video::EMF_LIGHTING,
                              !trianglemesh->IsWireframe());  // avoid shading for wireframes
    meshnode->setMaterialFlag(video::EMF_BACK_FACE_CULLING, trianglemesh->IsBackfaceCull());

    meshnode->setMaterialFlag(video::EMF_COLOR_MATERIAL, true);  // so color shading = vertexes  color
}

void ChIrrNodeShape::UpdateTriangleMesh_mat(std::shared_ptr<ChTriangleMeshShape> trianglemesh) {
    // Fetch the 1st child, i.e. the mesh
    ISceneNode* mchildnode = *(getChildren().begin());
    if (!mchildnode)
        return;
    if (!(mchildnode->getType() == scene::ESNT_MESH))
        return;

    auto meshnode = (scene::IMeshSceneNode*)mchildnode;
    scene::SMesh* smesh = (scene::SMesh*)meshnode->getMesh();

    const auto& mesh = trianglemesh->GetMesh();
    const auto& materials = trianglemesh->GetMaterials();

    int nbuffers = (int)smesh->getMeshBufferCount();
    int nmaterials = (int)materials.size();
    assert(nbuffers == nmaterials);

    if (nbuffers == 0)
        return;

    const auto& vertices = mesh->getCoordsVertices();
    const auto& normals = mesh->getCoordsNormals();
    const auto& uvs = mesh->getCoordsUV();

    const auto& v_indices = mesh->getIndicesVertexes();
    const auto& n_indices = mesh->getIndicesNormals();
    const auto& uv_indices = mesh->getIndicesUV();
    const auto& m_indices = mesh->getIndicesMaterials();

    unsigned int ntriangles_all = (unsigned int)v_indices.size();

    // Count number of faces assigned to each material (buffer)
    std::vector<unsigned int> nfaces_per_buffer;
    if (m_indices.empty()) {
        assert(nbuffers == 1);
        nfaces_per_buffer.push_back(ntriangles_all);
    } else {
        for (int imat = 0; imat < nmaterials; imat++) {
            auto count = std::count(m_indices.begin(), m_indices.end(), imat);
            nfaces_per_buffer.push_back(count);
        }
    }

    // Load each mesh buffer one at a time
    for (int i = 0; i < nbuffers; i++) {
        // Get the current Irrlicht mesh buffer
        scene::CDynamicMeshBuffer* irr_meshbuffer = (scene::CDynamicMeshBuffer*)smesh->getMeshBuffer(i);
        auto& irr_vertices = irr_meshbuffer->getVertexBuffer();
        auto& irr_indices = irr_meshbuffer->getIndexBuffer();
        auto& irr_mat = irr_meshbuffer->getMaterial();

        irr_vertices.reallocate(0);
        irr_indices.reallocate(0);

        // Set the Irrlicht material for this mesh buffer
        irr_mat = tools::ToIrrlichtMaterial(materials[i], getSceneManager()->getVideoDriver());
        irr_mat.ColorMaterial = irr::video::ECM_NONE;
        ////irr_mat.setFlag(video::EMF_TEXTURE_WRAP, video::ETC_CLAMP);

        // Map from vertex indices in full Chrono mesh to vertex indices in current Irrlicht mesh buffer.
        core::map<video::S3DVertex, int> vertex_map;

        // Set the Irrlicht vertex and index buffers for this mesh buffer
        ChVector<> t[3];    // positions of trianlge vertices
        ChVector<> n[3];    // normals at the triangle vertices
        ChVector2<> uv[3];  // UV coordinates at the triangle vertices
        unsigned int num_added_tri = 0;
        for (unsigned int itri = 0; itri < ntriangles_all; itri++) {
            if (!m_indices.empty() && m_indices[itri] != i)
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
            }

            u32 v_id[3];
            for (int iv = 0; iv < 3; iv++) {
                auto vertex = ToIrrlichtVertex(t[iv], n[iv], uv[iv], irr_mat.DiffuseColor);
                auto search = vertex_map.find(vertex);
                if (search) {
                    // Already stored Irrlicht vertex
                    v_id[iv] = search->getValue();
                } else {
                    // Create a new Irrlicht vertex
                    irr_vertices.push_back(vertex);
                    v_id[iv] = (u32)irr_vertices.size() - 1;
                    vertex_map.insert(vertex, v_id[iv]);
                }
            }

            irr_indices.push_back(v_id[2]);
            irr_indices.push_back(v_id[1]);
            irr_indices.push_back(v_id[0]);

            num_added_tri++;
        }

        assert(irr_meshbuffer->getVertexCount() == vertex_map.size());
        assert(irr_meshbuffer->getIndexCount() == 3 * num_added_tri);
        assert(nfaces_per_buffer[i] == num_added_tri);

        irr_meshbuffer->setDirty();                                  // to force update of hardware buffers
        irr_meshbuffer->setHardwareMappingHint(scene::EHM_DYNAMIC);  // EHM_DYNAMIC for faster hw mapping
        irr_meshbuffer->recalculateBoundingBox();
    }

    smesh->recalculateBoundingBox();

    //// RADU TODO: what is and is not needed here?

    ////meshnode->setMaterialType(video::EMT_DETAIL_MAP);
    meshnode->setMaterialType(video::EMT_SOLID);

    ////meshnode->setAutomaticCulling(scene::EAC_OFF);
    ////meshnode->setMaterialFlag(video::EMF_WIREFRAME, trianglemesh->IsWireframe());
    ////meshnode->setMaterialFlag(video::EMF_LIGHTING,
    ////                          !trianglemesh->IsWireframe());  // avoid shading for wireframes
    ////meshnode->setMaterialFlag(video::EMF_BACK_FACE_CULLING, trianglemesh->IsBackfaceCull());
    ////meshnode->setMaterialFlag(video::EMF_BACK_FACE_CULLING, false);
    ////meshnode->setMaterialFlag(video::EMF_ANTI_ALIASING, true);
    ////meshnode->setMaterialFlag(video::EMF_COLOR_MATERIAL, true);  // so color shading = vertexes  color
}

// Update a trimesh by keeping fixed the connectivity and only touching the specified modified vertices.
void ChIrrNodeShape::UpdateTriangleMeshFixedConnectivity(std::shared_ptr<ChTriangleMeshShape> trianglemesh) {
    // Access the Irrlicht mesh (first child node)
    ISceneNode* childnode = *(getChildren().begin());
    if (!childnode)
        return;

    if (!(childnode->getType() == scene::ESNT_MESH))
        return;

    scene::IMeshSceneNode* meshnode = (scene::IMeshSceneNode*)childnode;
    if (meshnode->getMesh()->getMeshBufferCount() == 0)
        return;

    scene::CDynamicMeshBuffer* irrmesh = (scene::CDynamicMeshBuffer*)meshnode->getMesh()->getMeshBuffer(0);
    auto& vertexbuffer = irrmesh->getVertexBuffer();
    auto& indexbuffer = irrmesh->getIndexBuffer();

    // Access Chrono triangle mesh
    const auto& mesh = trianglemesh->GetMesh();
    std::vector<ChVector<>>& vertices = mesh->getCoordsVertices();
    std::vector<ChVector<>>& normals = mesh->getCoordsNormals();
    std::vector<ChVector2<>>& uvs = mesh->getCoordsUV();
    std::vector<ChColor>& colors = mesh->getCoordsColors();
    std::vector<ChVector<int>>& idx_vertices = mesh->getIndicesVertexes();

    unsigned int ntriangles = (unsigned int)mesh->getIndicesVertexes().size();
    unsigned int nvertices = (unsigned int)mesh->getCoordsVertices().size();
    unsigned int nuvs = (unsigned int)mesh->getCoordsUV().size();
    unsigned int ncolors = (unsigned int)mesh->getCoordsColors().size();

    bool has_colors = (ncolors > 0);
    bool has_uvs = (nuvs > 0);

    assert(mesh->getCoordsNormals().size() == mesh->getCoordsVertices().size());
    assert(ncolors == 0 || ncolors == nvertices);
    assert(nuvs == 0 || nuvs == nvertices);

    if (m_initial_update) {
        // Full setup of the Irrlicht mesh
        vertexbuffer.set_used(nvertices);
        indexbuffer.set_used(ntriangles * 3);

        for (unsigned int i = 0; i < nvertices; i++) {
            video::SColor color(255, 255, 255, 255);
            if (has_colors) {
                color.setRed((u32)(colors[i].R * 255));
                color.setGreen((u32)(colors[i].G * 255));
                color.setBlue((u32)(colors[i].B * 255));
            }
            f32 u = 0;
            f32 v = 0;
            if (has_uvs) {
                u = (f32)uvs[i].x();
                v = (f32)uvs[i].y();
            }

            vertexbuffer[i] = video::S3DVertex(                                    //
                (f32)vertices[i].x(), (f32)vertices[i].y(), (f32)vertices[i].z(),  //
                (f32)normals[i].x(), (f32)normals[i].y(), (f32)normals[i].z(),     //
                color,                                                             //
                u, v                                                               //
            );
        }

        for (unsigned int i = 0; i < ntriangles; i++) {
            indexbuffer.setValue(3 * i + 0, idx_vertices[i].x());
            indexbuffer.setValue(3 * i + 1, idx_vertices[i].y());
            indexbuffer.setValue(3 * i + 2, idx_vertices[i].z());
        }

        irrmesh->setHardwareMappingHint(scene::EHM_DYNAMIC);  // EHM_DYNAMIC for faster hw mapping
        irrmesh->recalculateBoundingBox();

        meshnode->setAutomaticCulling(scene::EAC_OFF);                                 // disable automatic culling
        meshnode->setMaterialFlag(video::EMF_WIREFRAME, trianglemesh->IsWireframe());  // set as wireframe?
        meshnode->setMaterialFlag(video::EMF_LIGHTING, !trianglemesh->IsWireframe());  // no shading if wireframe
        meshnode->setMaterialFlag(video::EMF_BACK_FACE_CULLING, trianglemesh->IsBackfaceCull());

    } else {
        // Incremental update of the Irrlicht mesh
        for (auto i : trianglemesh->GetModifiedVertices()) {
            vertexbuffer[i].Pos = core::vector3df((f32)vertices[i].x(), (f32)vertices[i].y(), (f32)vertices[i].z());
            vertexbuffer[i].Normal = core::vector3df((f32)normals[i].x(), (f32)normals[i].y(), (f32)normals[i].z());
            if (has_colors) {
                vertexbuffer[i].Color = tools::ToIrrlichtSColor(colors[i]);
            }
        }
    }

    irrmesh->setDirty();                                         // to force update of hardware buffers
    meshnode->setMaterialFlag(video::EMF_COLOR_MATERIAL, true);  // color shading = vertexes  color
}

void ChIrrNodeShape::UpdateGlyphs(std::shared_ptr<ChGlyphs> glyphs) {
    // Fetch the 1st child, i.e. the mesh
    ISceneNode* mchildnode = *(getChildren().begin());
    if (!mchildnode || mchildnode->getType() != scene::ESNT_MESH)
        return;

    scene::IMeshSceneNode* meshnode = (scene::IMeshSceneNode*)mchildnode;
    scene::IMesh* amesh = meshnode->getMesh();
    if (amesh->getMeshBufferCount() == 0)
        return;

    scene::CDynamicMeshBuffer* irrmesh = (scene::CDynamicMeshBuffer*)amesh->getMeshBuffer(0);

    size_t ntriangles = 0;
    size_t nvertexes = 0;

    switch (glyphs->GetDrawMode()) {
        case ChGlyphs::GLYPH_POINT:
            ntriangles = 12 * glyphs->GetNumberOfGlyphs();
            nvertexes = 24 * glyphs->GetNumberOfGlyphs();
            break;
        case ChGlyphs::GLYPH_VECTOR:
            ntriangles = 1 * glyphs->GetNumberOfGlyphs();
            nvertexes = 3 * glyphs->GetNumberOfGlyphs();
            break;
        case ChGlyphs::GLYPH_COORDSYS:
            ntriangles = 3 * glyphs->GetNumberOfGlyphs();
            nvertexes = 9 * glyphs->GetNumberOfGlyphs();
            break;
    }

    // smart inflating of allocated buffers, only if necessary, and once in a while shrinking
    if (irrmesh->getIndexBuffer().allocated_size() > (ntriangles * 3) * 1.5)
        irrmesh->getIndexBuffer().reallocate(0);  // clear();
    if (irrmesh->getVertexBuffer().allocated_size() > nvertexes * 1.5)
        irrmesh->getVertexBuffer().reallocate(0);

    irrmesh->getIndexBuffer().set_used((u32)ntriangles * 3);
    irrmesh->getVertexBuffer().set_used((u32)nvertexes);

    // set buffers

    if (glyphs->GetDrawMode() == ChGlyphs::GLYPH_POINT) {
        const u32 u[36] = {0,  1,  2,  0,  2,  3,  4,  6,  5,  4,  7,  6,  8,  9,  10, 8,  10, 11,
                           12, 14, 13, 12, 15, 14, 16, 18, 17, 16, 19, 18, 20, 21, 22, 20, 22, 23};

        int itri = 0;

        for (unsigned int ig = 0; ig < glyphs->points.size(); ++ig) {
            ChVector<> t1 = glyphs->points[ig];
            ChColor mcol = glyphs->colors[ig];
            video::SColor clr(255, (u32)(mcol.R * 255), (u32)(mcol.G * 255), (u32)(mcol.B * 255));

            // create a small cube per each vertex
            unsigned int voffs = ig * 24;
            f32 s = (f32)(glyphs->GetGlyphsSize() * 0.5);

            irrmesh->getVertexBuffer()[0 + voffs] = video::S3DVertex(-s, -s, -s, 0, 0, -1, clr, 0, 0);
            irrmesh->getVertexBuffer()[1 + voffs] = video::S3DVertex(-s, s, -s, 0, 0, -1, clr, 0, 1);
            irrmesh->getVertexBuffer()[2 + voffs] = video::S3DVertex(s, s, -s, 0, 0, -1, clr, 1, 1);
            irrmesh->getVertexBuffer()[3 + voffs] = video::S3DVertex(s, -s, -s, 0, 0, -1, clr, 1, 0);

            irrmesh->getVertexBuffer()[4 + voffs] = video::S3DVertex(-s, -s, s, 0, 0, 1, clr, 0, 0);
            irrmesh->getVertexBuffer()[5 + voffs] = video::S3DVertex(-s, s, s, 0, 0, 1, clr, 0, 1);
            irrmesh->getVertexBuffer()[6 + voffs] = video::S3DVertex(s, s, s, 0, 0, 1, clr, 1, 1);
            irrmesh->getVertexBuffer()[7 + voffs] = video::S3DVertex(s, -s, s, 0, 0, 1, clr, 1, 0);

            irrmesh->getVertexBuffer()[8 + voffs] = video::S3DVertex(-s, -s, -s, -1, 0, 0, clr, 0, 0);
            irrmesh->getVertexBuffer()[9 + voffs] = video::S3DVertex(-s, -s, s, -1, 0, 0, clr, 0, 1);
            irrmesh->getVertexBuffer()[10 + voffs] = video::S3DVertex(-s, s, s, -1, 0, 0, clr, 1, 1);
            irrmesh->getVertexBuffer()[11 + voffs] = video::S3DVertex(-s, s, -s, -1, 0, 0, clr, 1, 0);

            irrmesh->getVertexBuffer()[12 + voffs] = video::S3DVertex(s, -s, -s, 1, 0, 0, clr, 0, 0);
            irrmesh->getVertexBuffer()[13 + voffs] = video::S3DVertex(s, -s, s, 1, 0, 0, clr, 0, 1);
            irrmesh->getVertexBuffer()[14 + voffs] = video::S3DVertex(s, s, s, 1, 0, 0, clr, 1, 1);
            irrmesh->getVertexBuffer()[15 + voffs] = video::S3DVertex(s, s, -s, 1, 0, 0, clr, 1, 0);

            irrmesh->getVertexBuffer()[16 + voffs] = video::S3DVertex(-s, -s, -s, 0, -1, 0, clr, 0, 0);
            irrmesh->getVertexBuffer()[17 + voffs] = video::S3DVertex(-s, -s, s, 0, -1, 0, clr, 0, 1);
            irrmesh->getVertexBuffer()[18 + voffs] = video::S3DVertex(s, -s, s, 0, -1, 0, clr, 1, 1);
            irrmesh->getVertexBuffer()[19 + voffs] = video::S3DVertex(s, -s, -s, 0, -1, 0, clr, 1, 0);

            irrmesh->getVertexBuffer()[20 + voffs] = video::S3DVertex(-s, s, -s, 0, 1, 0, clr, 0, 0);
            irrmesh->getVertexBuffer()[21 + voffs] = video::S3DVertex(-s, s, s, 0, 1, 0, clr, 0, 1);
            irrmesh->getVertexBuffer()[22 + voffs] = video::S3DVertex(s, s, s, 0, 1, 0, clr, 1, 1);
            irrmesh->getVertexBuffer()[23 + voffs] = video::S3DVertex(s, s, -s, 0, 1, 0, clr, 1, 0);

            for (u32 i = 0; i < 24; ++i) {
                irrmesh->getVertexBuffer()[i + voffs].Pos += core::vector3df((f32)t1.x(), (f32)t1.y(), (f32)t1.z());
                // buffer->BoundingBox.addInternalPoint(buffer->Vertices[i].Pos);
            }

            for (u32 i = 0; i < 36; ++i)
                irrmesh->getIndexBuffer().setValue(i + itri, u[i] + voffs);

            itri += 36;
        }
    }

    if (glyphs->GetDrawMode() == ChGlyphs::GLYPH_VECTOR) {
        int itri = 0;
        for (unsigned int ig = 0; ig < glyphs->points.size(); ++ig) {
            ChVector<> t1 = glyphs->points[ig];
            ChVector<> t2 = glyphs->vectors[ig] + t1;
            ChColor mcol = glyphs->colors[ig];
            video::SColor clr(255, (u32)(mcol.R * 255), (u32)(mcol.G * 255), (u32)(mcol.B * 255));

            // create a  small line (a degenerate triangle) per each vector
            irrmesh->getVertexBuffer()[0 + ig * 3] =
                video::S3DVertex((f32)t1.x(), (f32)t1.y(), (f32)t1.z(), 1, 0, 0, clr, 0, 0);

            irrmesh->getVertexBuffer()[1 + ig * 3] =
                video::S3DVertex((f32)t2.x(), (f32)t2.y(), (f32)t2.z(), 1, 0, 0, clr, 0, 0);

            irrmesh->getVertexBuffer()[2 + ig * 3] =
                video::S3DVertex((f32)t2.x(), (f32)t2.y(), (f32)t2.z(), 1, 0, 0, clr, 0, 0);

            irrmesh->getIndexBuffer().setValue(0 + itri * 3, 0 + ig * 3);
            irrmesh->getIndexBuffer().setValue(1 + itri * 3, 1 + ig * 3);
            irrmesh->getIndexBuffer().setValue(2 + itri * 3, 2 + ig * 3);

            ++itri;
        }
    }

    if (glyphs->GetDrawMode() == ChGlyphs::GLYPH_COORDSYS) {
        int itri = 0;

        for (unsigned int ig = 0; ig < glyphs->points.size(); ++ig) {
            ChVector<> t1 = glyphs->points[ig];
            ChVector<> t2;

            // X axis - create a  small line (a degenerate triangle) per each vector
            t2 = glyphs->rotations[ig].Rotate(ChVector<>(1, 0, 0) * glyphs->GetGlyphsSize()) + t1;

            irrmesh->getVertexBuffer()[0 + ig * 9] =
                video::S3DVertex((f32)t1.x(), (f32)t1.y(), (f32)t1.z(), 1, 0, 0, video::SColor(255, 255, 0, 0), 0, 0);
            irrmesh->getVertexBuffer()[1 + ig * 9] =
                video::S3DVertex((f32)t2.x(), (f32)t2.y(), (f32)t2.z(), 1, 0, 0, video::SColor(255, 255, 0, 0), 0, 0);
            irrmesh->getVertexBuffer()[2 + ig * 9] =
                video::S3DVertex((f32)t2.x(), (f32)t2.y(), (f32)t2.z(), 1, 0, 0, video::SColor(255, 255, 0, 0), 0, 0);

            irrmesh->getIndexBuffer().setValue(0 + itri * 3, 0 + ig * 9);
            irrmesh->getIndexBuffer().setValue(1 + itri * 3, 1 + ig * 9);
            irrmesh->getIndexBuffer().setValue(2 + itri * 3, 2 + ig * 9);

            ++itri;

            // Y axis
            t2 = glyphs->rotations[ig].Rotate(ChVector<>(0, 1, 0) * glyphs->GetGlyphsSize()) + t1;

            irrmesh->getVertexBuffer()[3 + ig * 9] =
                video::S3DVertex((f32)t1.x(), (f32)t1.y(), (f32)t1.z(), 1, 0, 0, video::SColor(255, 0, 255, 0), 0, 0);
            irrmesh->getVertexBuffer()[4 + ig * 9] =
                video::S3DVertex((f32)t2.x(), (f32)t2.y(), (f32)t2.z(), 1, 0, 0, video::SColor(255, 0, 255, 0), 0, 0);
            irrmesh->getVertexBuffer()[5 + ig * 9] =
                video::S3DVertex((f32)t2.x(), (f32)t2.y(), (f32)t2.z(), 1, 0, 0, video::SColor(255, 0, 255, 0), 0, 0);

            irrmesh->getIndexBuffer().setValue(0 + itri * 3, 3 + ig * 9);
            irrmesh->getIndexBuffer().setValue(1 + itri * 3, 4 + ig * 9);
            irrmesh->getIndexBuffer().setValue(2 + itri * 3, 5 + ig * 9);

            ++itri;

            // Z axis
            t2 = glyphs->rotations[ig].Rotate(ChVector<>(0, 0, 1) * glyphs->GetGlyphsSize()) + t1;

            irrmesh->getVertexBuffer()[6 + ig * 9] =
                video::S3DVertex((f32)t1.x(), (f32)t1.y(), (f32)t1.z(), 1, 0, 0, video::SColor(255, 0, 0, 255), 0, 0);
            irrmesh->getVertexBuffer()[7 + ig * 9] =
                video::S3DVertex((f32)t2.x(), (f32)t2.y(), (f32)t2.z(), 1, 0, 0, video::SColor(255, 0, 0, 255), 0, 0);
            irrmesh->getVertexBuffer()[8 + ig * 9] =
                video::S3DVertex((f32)t2.x(), (f32)t2.y(), (f32)t2.z(), 1, 0, 0, video::SColor(255, 0, 0, 255), 0, 0);

            irrmesh->getIndexBuffer().setValue(0 + itri * 3, 6 + ig * 9);
            irrmesh->getIndexBuffer().setValue(1 + itri * 3, 7 + ig * 9);
            irrmesh->getIndexBuffer().setValue(2 + itri * 3, 8 + ig * 9);

            ++itri;
        }
    }

    irrmesh->setDirty();                                  // to force update of hardware buffers
    irrmesh->setHardwareMappingHint(scene::EHM_DYNAMIC);  // EHM_NEVER); //EHM_DYNAMIC for faster hw mapping
    irrmesh->recalculateBoundingBox();

    if (glyphs->GetDrawMode() == ChGlyphs::GLYPH_VECTOR || glyphs->GetDrawMode() == ChGlyphs::GLYPH_COORDSYS) {
        meshnode->setMaterialFlag(video::EMF_WIREFRAME, true);
        meshnode->setMaterialFlag(video::EMF_LIGHTING, false);  // avoid shading for wireframe
        meshnode->setMaterialFlag(video::EMF_BACK_FACE_CULLING, false);
    } else {
        meshnode->setMaterialFlag(video::EMF_WIREFRAME, false);
        meshnode->setMaterialFlag(video::EMF_LIGHTING, true);
        meshnode->setMaterialFlag(video::EMF_BACK_FACE_CULLING, true);
    }

    if (glyphs->GetZbufferHide() == true)
        meshnode->setMaterialFlag(video::EMF_ZBUFFER, true);
    else
        meshnode->setMaterialFlag(video::EMF_ZBUFFER, false);

    meshnode->setMaterialFlag(video::EMF_COLOR_MATERIAL, true);  // so color shading = vertexes  color
}

void ChIrrNodeShape::UpdateSurface(std::shared_ptr<ChSurfaceShape> surface) {
    std::shared_ptr<geometry::ChSurface> msurface = surface->GetSurfaceGeometry();

    // Set color.
    auto vis = std::static_pointer_cast<ChVisualShape>(m_shape);
    video::SColor clr = tools::ToIrrlichtSColor(vis->GetColor());

    // Fetch the 1st child, i.e. the mesh
    ISceneNode* mchildnode = *(getChildren().begin());
    if (!mchildnode || mchildnode->getType() != scene::ESNT_MESH)
        return;

    scene::IMeshSceneNode* meshnode = (scene::IMeshSceneNode*)mchildnode;
    scene::IMesh* amesh = meshnode->getMesh();
    if (amesh->getMeshBufferCount() == 0)
        return;

    scene::CDynamicMeshBuffer* irrmesh = (scene::CDynamicMeshBuffer*)amesh->getMeshBuffer(0);

    if (!surface->IsWireframe()) {
        auto sections_u = surface->GetResolutionU() * 4;  //***TEST***
        auto sections_v = surface->GetResolutionV() * 4;  //***TEST***
        auto nvertexes = (sections_u + 1) * (sections_v + 1);
        auto ntriangles = (sections_u) * (sections_v)*2;

        // smart inflating of allocated buffers, only if necessary, and once in a while shrinking
        if (irrmesh->getIndexBuffer().allocated_size() > (ntriangles * 3) * 1.5)
            irrmesh->getIndexBuffer().reallocate(0);  // clear();
        if (irrmesh->getVertexBuffer().allocated_size() > nvertexes * 1.5)
            irrmesh->getVertexBuffer().reallocate(0);

        irrmesh->getIndexBuffer().set_used((u32)ntriangles * 3);
        irrmesh->getVertexBuffer().set_used((u32)nvertexes);

        int itri = 0;

        for (auto iv = 0; iv <= sections_v; ++iv) {
            double mV = 1.0 * ((double)iv / (double)(sections_v));  // v abscissa

            for (auto iu = 0; iu <= sections_u; ++iu) {
                double mU = 1.0 * ((double)iu / (double)(sections_u));  // u abscissa

                ChVector<> P;
                msurface->Evaluate(P, mU, mV);
                ////P = vis->Pos + vis->Rot * P;

                ChVector<> N;
                msurface->Normal(N, mU, mV);
                ////N = vis->Rot * N;

                // create two triangles per uv increment

                irrmesh->getVertexBuffer()[iu + iv * (sections_u + 1)] =
                    video::S3DVertex((f32)P.x(), (f32)P.y(), (f32)P.z(), (f32)N.x(), (f32)N.y(), (f32)N.z(), clr, 0, 0);

                if (iu > 0 && iv > 0) {
                    irrmesh->getIndexBuffer().setValue(0 + itri * 3, iu - 1 + iv * (sections_u + 1));
                    irrmesh->getIndexBuffer().setValue(1 + itri * 3, iu - 1 + (iv - 1) * (sections_u + 1));
                    irrmesh->getIndexBuffer().setValue(2 + itri * 3, iu + iv * (sections_u + 1));
                    ++itri;
                    irrmesh->getIndexBuffer().setValue(0 + itri * 3, iu - 1 + (iv - 1) * (sections_u + 1));
                    irrmesh->getIndexBuffer().setValue(1 + itri * 3, iu + (iv - 1) * (sections_u + 1));
                    irrmesh->getIndexBuffer().setValue(2 + itri * 3, iu + iv * (sections_u + 1));
                    ++itri;
                }
            }
        }

        irrmesh->setDirty();                                  // to force update of hardware buffers
        irrmesh->setHardwareMappingHint(scene::EHM_DYNAMIC);  // EHM_NEVER); //EHM_DYNAMIC for faster hw mapping
        irrmesh->recalculateBoundingBox();

        meshnode->setAutomaticCulling(scene::EAC_OFF);

        meshnode->setMaterialFlag(video::EMF_WIREFRAME, false);
        meshnode->setMaterialFlag(video::EMF_LIGHTING, true);  // avoid shading for wireframe
        meshnode->setMaterialFlag(video::EMF_BACK_FACE_CULLING, false);
        meshnode->setMaterialFlag(video::EMF_COLOR_MATERIAL, true);
    } else {                  // if wirewrame u v isolines
        auto isolines_u = 4;  //***TEST***
        auto isolines_v = 3;  //***TEST***
        auto sections_u = surface->GetResolutionU() * isolines_u;
        auto sections_v = surface->GetResolutionV() * isolines_v;

        auto nvertexes = (sections_u + 1) * (isolines_v) + (sections_v + 1) * (isolines_u);
        auto ntriangles = (sections_u) * (isolines_v) + (sections_v) * (isolines_u);

        // smart inflating of allocated buffers, only if necessary, and once in a while shrinking
        if (irrmesh->getIndexBuffer().allocated_size() > (ntriangles * 3) * 1.5)
            irrmesh->getIndexBuffer().reallocate(0);  // clear();
        if (irrmesh->getVertexBuffer().allocated_size() > nvertexes * 1.5)
            irrmesh->getVertexBuffer().reallocate(0);

        irrmesh->getIndexBuffer().set_used((u32)ntriangles * 3);
        irrmesh->getVertexBuffer().set_used((u32)nvertexes);

        int itri = 0;

        for (auto iv = 0; iv < isolines_v; ++iv) {
            double mV = 1.0 * ((double)iv / (double)(isolines_v - 1));  // v abscissa

            for (auto iu = 0; iu <= sections_u; ++iu) {
                double mU = 1.0 * ((double)iu / (double)(sections_u));  // u abscissa

                ChVector<> P;
                msurface->Evaluate(P, mU, mV);
                ////P = vis->Pos + vis->Rot * P;

                irrmesh->getVertexBuffer()[iu + iv * (sections_u + 1)] =
                    video::S3DVertex((f32)P.x(), (f32)P.y(), (f32)P.z(), 1, 0, 0, clr, 0, 0);

                if (iu > 0) {
                    irrmesh->getIndexBuffer().setValue(0 + itri * 3, iu - 1 + iv * (sections_u + 1));
                    irrmesh->getIndexBuffer().setValue(1 + itri * 3, iu - 1 + iv * (sections_u + 1));
                    irrmesh->getIndexBuffer().setValue(2 + itri * 3, iu + iv * (sections_u + 1));
                    ++itri;
                }
            }
        }
        auto stride = (sections_u + 1) * isolines_v;
        for (auto iu = 0; iu < isolines_u; ++iu) {
            double mU = 1.0 * ((double)iu / (double)(isolines_u - 1));  // u abscissa

            for (auto iv = 0; iv <= sections_v; ++iv) {
                double mV = 1.0 * ((double)iv / (double)(sections_v));  // v abscissa

                ChVector<> P;
                msurface->Evaluate(P, mU, mV);
                ////P = vis->Pos + vis->Rot * P;

                irrmesh->getVertexBuffer()[iv + iu * (sections_v + 1) + stride] =
                    video::S3DVertex((f32)P.x(), (f32)P.y(), (f32)P.z(), 1, 0, 0, clr, 0, 0);

                if (iv > 0) {
                    irrmesh->getIndexBuffer().setValue(0 + itri * 3, iv - 1 + iu * (sections_v + 1) + stride);
                    irrmesh->getIndexBuffer().setValue(1 + itri * 3, iv - 1 + iu * (sections_v + 1) + stride);
                    irrmesh->getIndexBuffer().setValue(2 + itri * 3, iv + iu * (sections_v + 1) + stride);
                    ++itri;
                }
            }
        }

        irrmesh->setDirty();                                  // to force update of hardware buffers
        irrmesh->setHardwareMappingHint(scene::EHM_DYNAMIC);  // EHM_NEVER); //EHM_DYNAMIC for faster hw mapping
        irrmesh->recalculateBoundingBox();

        meshnode->setAutomaticCulling(scene::EAC_OFF);

        meshnode->setMaterialFlag(video::EMF_WIREFRAME, true);
        meshnode->setMaterialFlag(video::EMF_LIGHTING, false);  // avoid shading for wireframe
        meshnode->setMaterialFlag(video::EMF_BACK_FACE_CULLING, false);
    }
}

void ChIrrNodeShape::UpdateLine(std::shared_ptr<geometry::ChLine> line, unsigned int nvertexes) {
    unsigned int ntriangles = nvertexes - 1;

    // Set color.
    auto vis = std::static_pointer_cast<ChVisualShape>(m_shape);
    video::SColor clr = tools::ToIrrlichtSColor(vis->GetColor());

    // Fetch the 1st child, i.e. the mesh
    ISceneNode* mchildnode = *(getChildren().begin());
    if (!mchildnode || mchildnode->getType() != scene::ESNT_MESH)
        return;

    scene::IMeshSceneNode* meshnode = (scene::IMeshSceneNode*)mchildnode;
    scene::IMesh* amesh = meshnode->getMesh();
    if (amesh->getMeshBufferCount() == 0)
        return;

    // SMeshBuffer* irrmesh = (SMeshBuffer*)amesh->getMeshBuffer(0);
    scene::CDynamicMeshBuffer* irrmesh = (scene::CDynamicMeshBuffer*)amesh->getMeshBuffer(0);

    // smart inflating of allocated buffers, only if necessary, and once in a while shrinking
    if (irrmesh->getIndexBuffer().allocated_size() > (ntriangles * 3) * 1.5)
        irrmesh->getIndexBuffer().reallocate(0);  // clear();
    if (irrmesh->getVertexBuffer().allocated_size() > nvertexes * 1.5)
        irrmesh->getVertexBuffer().reallocate(0);

    irrmesh->getIndexBuffer().set_used((u32)ntriangles * 3);
    irrmesh->getVertexBuffer().set_used((u32)nvertexes);

    int itri = 0;

    ChVector<> t1;
    line->Evaluate(t1, 0);
    ////t1 = vis->Pos + vis->Rot * t1;

    irrmesh->getVertexBuffer()[0] = video::S3DVertex((f32)t1.x(), (f32)t1.y(), (f32)t1.z(), 1, 0, 0, clr, 0, 0);

    double maxU = 1;
    if (auto mline_path = std::dynamic_pointer_cast<geometry::ChLinePath>(line))
        maxU = mline_path->GetPathDuration();

    for (unsigned int ig = 0; ig < ntriangles; ++ig) {
        double mU = maxU * ((double)ig / (double)(ntriangles - 1));  // abscissa

        ChVector<> t2;
        line->Evaluate(t2, mU);
        ////t2 = vis->Pos + vis->Rot * t2;

        // create a  small line (a degenerate triangle) per each vector

        irrmesh->getVertexBuffer()[1 + ig] =
            video::S3DVertex((f32)t2.x(), (f32)t2.y(), (f32)t2.z(), 1, 0, 0, clr, 0, 0);

        irrmesh->getIndexBuffer().setValue(0 + itri * 3, 0 + ig);
        irrmesh->getIndexBuffer().setValue(1 + itri * 3, 1 + ig);
        irrmesh->getIndexBuffer().setValue(2 + itri * 3, 1 + ig);

        ++itri;

        t1 = t2;
    }
    irrmesh->setDirty();                                  // to force update of hardware buffers
    irrmesh->setHardwareMappingHint(scene::EHM_DYNAMIC);  // EHM_NEVER); //EHM_DYNAMIC for faster hw mapping
    irrmesh->recalculateBoundingBox();

    meshnode->setAutomaticCulling(scene::EAC_OFF);

    meshnode->setMaterialFlag(video::EMF_WIREFRAME, true);
    meshnode->setMaterialFlag(video::EMF_LIGHTING, false);  // avoid shading for wireframe
    meshnode->setMaterialFlag(video::EMF_BACK_FACE_CULLING, false);

    // meshnode->setMaterialFlag(video::EMF_COLOR_MATERIAL, true); // so color shading = vertexes  color
}

}  // end namespace irrlicht
}  // end namespace chrono
