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

#include "chrono_irrlicht/ChIrrNodeProxyToAsset.h"

namespace chrono {
namespace irrlicht {

using namespace irr;

ChIrrNodeProxyToAsset::ChIrrNodeProxyToAsset(std::shared_ptr<ChAsset> asset, ISceneNode* parent)
    : ISceneNode(parent, parent->getSceneManager(), 0), visualization_asset(asset), initial_update(true) {}

scene::ISceneNode* ChIrrNodeProxyToAsset::clone(ISceneNode* newParent, scene::ISceneManager* newManager) {
    if (!newParent)
        newParent = Parent;
    if (!newManager)
        newManager = SceneManager;

    ChIrrNodeProxyToAsset* nb = new ChIrrNodeProxyToAsset(visualization_asset, newParent);

    nb->cloneMembers(this, newManager);
    nb->Box = Box;
    nb->visualization_asset = visualization_asset;

    if (newParent)
        nb->drop();
    return nb;
}

void ChIrrNodeProxyToAsset::Update() {
    if (!visualization_asset)
        return;

    if (!initial_update) {
        auto asset = std::dynamic_pointer_cast<ChVisualization>(visualization_asset);
        if (asset && asset->IsStatic())
            return;
    }

    if (auto trianglemesh = std::dynamic_pointer_cast<ChTriangleMeshShape>(visualization_asset)) {
        if (trianglemesh->FixedConnectivity())
            UpdateTriangleMeshFixedConnectivity(trianglemesh);
        else
            UpdateTriangleMesh(trianglemesh);
    } else if (auto glyphs = std::dynamic_pointer_cast<ChGlyphs>(visualization_asset)) {
        UpdateGlyphs(glyphs);
    } else if (auto surface = std::dynamic_pointer_cast<ChSurfaceShape>(visualization_asset)) {
        UpdateSurface(surface);
    } else if (auto path_shape = std::dynamic_pointer_cast<ChPathShape>(visualization_asset)) {
        UpdateLine(path_shape->GetPathGeometry(), path_shape->GetNumRenderPoints());
    } else if (auto line_shape = std::dynamic_pointer_cast<ChLineShape>(visualization_asset)) {
        UpdateLine(line_shape->GetLineGeometry(), line_shape->GetNumRenderPoints());
    }

    initial_update = false;
}

void ChIrrNodeProxyToAsset::UpdateTriangleMesh(std::shared_ptr<ChTriangleMeshShape> trianglemesh) {
    // Fetch the 1st child, i.e. the mesh
    ISceneNode* mchildnode = *(getChildren().begin());
    if (!mchildnode)
        return;

    if (!(mchildnode->getType() == scene::ESNT_MESH))
        return;
    scene::IMeshSceneNode* meshnode = (scene::IMeshSceneNode*)mchildnode;

    scene::IMesh* amesh = meshnode->getMesh();
    if (amesh->getMeshBufferCount() == 0)
        return;

    geometry::ChTriangleMeshConnected* mmesh = trianglemesh->GetMesh().get();
    unsigned int ntriangles = (unsigned int)mmesh->getIndicesVertexes().size();
    unsigned int nvertexes = ntriangles * 3;  // suboptimal, because some vertexes might be shared

    scene::CDynamicMeshBuffer* irrmesh = (scene::CDynamicMeshBuffer*)amesh->getMeshBuffer(0);

    // smart inflating of allocated buffers, only if necessary, and once in a while shrinking
    if (irrmesh->getIndexBuffer().allocated_size() > (ntriangles * 3) * 1.5)
        irrmesh->getIndexBuffer().reallocate(0);  // clear();
    if (irrmesh->getVertexBuffer().allocated_size() > nvertexes * 1.5)
        irrmesh->getVertexBuffer().reallocate(0);

    irrmesh->getIndexBuffer().set_used(ntriangles * 3);
    irrmesh->getVertexBuffer().set_used(nvertexes);

    // set buffers
    for (unsigned int itri = 0; itri < ntriangles; itri++) {
        ChVector<> t1 = mmesh->getCoordsVertices()[mmesh->getIndicesVertexes()[itri].x()];
        ChVector<> t2 = mmesh->getCoordsVertices()[mmesh->getIndicesVertexes()[itri].y()];
        ChVector<> t3 = mmesh->getCoordsVertices()[mmesh->getIndicesVertexes()[itri].z()];
        ChVector<> n1, n2, n3;
        if (mmesh->getIndicesNormals().size() == mmesh->getIndicesVertexes().size()) {
            n1 = mmesh->getCoordsNormals()[mmesh->getIndicesNormals()[itri].x()];
            n2 = mmesh->getCoordsNormals()[mmesh->getIndicesNormals()[itri].y()];
            n3 = mmesh->getCoordsNormals()[mmesh->getIndicesNormals()[itri].z()];
        } else {
            n1 = Vcross(t2 - t1, t3 - t1).GetNormalized();
            n2 = n1;
            n3 = n1;
        }

        ChVector<> uv1, uv2, uv3;
        if (mmesh->getIndicesUV().size() == mmesh->getIndicesVertexes().size()) {
            uv1 = mmesh->getCoordsUV()[mmesh->getIndicesUV()[itri].x()];
            uv2 = mmesh->getCoordsUV()[mmesh->getIndicesUV()[itri].y()];
            uv3 = mmesh->getCoordsUV()[mmesh->getIndicesUV()[itri].z()];
        } else if (mmesh->getIndicesUV().size() == 0 &&
                   mmesh->getCoordsUV().size() == mmesh->getCoordsVertices().size()) {
            uv1 = mmesh->getCoordsUV()[mmesh->getIndicesVertexes()[itri].x()];
            uv2 = mmesh->getCoordsUV()[mmesh->getIndicesVertexes()[itri].y()];
            uv3 = mmesh->getCoordsUV()[mmesh->getIndicesVertexes()[itri].z()];
        } else {
            uv1 = uv2 = uv3 = VNULL;
        }

        ChVector<float> col1, col2, col3;
        if (mmesh->getIndicesColors().size() == mmesh->getIndicesVertexes().size()) {
            col1 = mmesh->getCoordsColors()[mmesh->getIndicesColors()[itri].x()];
            col2 = mmesh->getCoordsColors()[mmesh->getIndicesColors()[itri].y()];
            col3 = mmesh->getCoordsColors()[mmesh->getIndicesColors()[itri].z()];
        } else if (mmesh->getIndicesColors().size() == 0 &&
                   mmesh->getCoordsColors().size() == mmesh->getCoordsVertices().size()) {
            col1 = mmesh->getCoordsColors()[mmesh->getIndicesVertexes()[itri].x()];
            col2 = mmesh->getCoordsColors()[mmesh->getIndicesVertexes()[itri].y()];
            col3 = mmesh->getCoordsColors()[mmesh->getIndicesVertexes()[itri].z()];
        } else {
            col1 = col2 = col3 =
                ChVector<float>(trianglemesh->GetColor().R, trianglemesh->GetColor().G, trianglemesh->GetColor().B);
        }

        irrmesh->getVertexBuffer()[0 + itri * 3] =
            video::S3DVertex((f32)t1.x(), (f32)t1.y(), (f32)t1.z(), (f32)n1.x(), (f32)n1.y(), (f32)n1.z(),
                             video::SColor(255, (u32)(col1.x() * 255), (u32)(col1.y() * 255), (u32)(col1.z() * 255)),
                             (f32)uv1.x(), (f32)uv1.y());

        irrmesh->getVertexBuffer()[1 + itri * 3] =
            video::S3DVertex((f32)t2.x(), (f32)t2.y(), (f32)t2.z(), (f32)n2.x(), (f32)n2.y(), (f32)n2.z(),
                             video::SColor(255, (u32)(col2.x() * 255), (u32)(col2.y() * 255), (u32)(col2.z() * 255)),
                             (f32)uv2.x(), (f32)uv2.y());

        irrmesh->getVertexBuffer()[2 + itri * 3] =
            video::S3DVertex((f32)t3.x(), (f32)t3.y(), (f32)t3.z(), (f32)n3.x(), (f32)n3.y(), (f32)n3.z(),
                             video::SColor(255, (u32)(col3.x() * 255), (u32)(col3.y() * 255), (u32)(col3.z() * 255)),
                             (f32)uv3.x(), (f32)uv3.y());

        irrmesh->getIndexBuffer().setValue(0 + itri * 3, 0 + itri * 3);
        irrmesh->getIndexBuffer().setValue(1 + itri * 3, 1 + itri * 3);
        irrmesh->getIndexBuffer().setValue(2 + itri * 3, 2 + itri * 3);
    }

    irrmesh->setDirty();                                  // to force update of hardware buffers
    irrmesh->setHardwareMappingHint(scene::EHM_DYNAMIC);  // EHM_NEVER); //EHM_DYNAMIC for faster hw mapping
    irrmesh->recalculateBoundingBox();

    meshnode->setAutomaticCulling(scene::EAC_OFF);

    meshnode->setMaterialFlag(video::EMF_WIREFRAME, trianglemesh->IsWireframe());
    meshnode->setMaterialFlag(video::EMF_LIGHTING,
                              !trianglemesh->IsWireframe());  // avoid shading for wireframes
    meshnode->setMaterialFlag(video::EMF_BACK_FACE_CULLING, trianglemesh->IsBackfaceCull());

    meshnode->setMaterialFlag(video::EMF_COLOR_MATERIAL, true);  // so color shading = vertexes  color
}

// Update a trimesh by keeping fixed the connectivity and only touching the specified modified vertices.
void ChIrrNodeProxyToAsset::UpdateTriangleMeshFixedConnectivity(std::shared_ptr<ChTriangleMeshShape> trianglemesh) {
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
    geometry::ChTriangleMeshConnected* mesh = trianglemesh->GetMesh().get();
    std::vector<ChVector<>>& vertices = mesh->getCoordsVertices();
    std::vector<ChVector<>>& normals = mesh->getCoordsNormals();
    std::vector<ChVector<int>>& idx_vertices = mesh->getIndicesVertexes();
    std::vector<ChVector<int>>& idx_normals = mesh->getIndicesNormals();
    std::vector<ChVector<>>& uv_coords = mesh->getCoordsUV();
    std::vector<ChVector<float>>& cols = mesh->getCoordsColors();

    // Chrono mesh -> Irrlicht mesh
    if (initial_update) {
        // Full setup of the Irrlicht mesh
        unsigned int ntriangles = (unsigned int)mesh->getIndicesVertexes().size();
        unsigned int nvertices = (unsigned int)mesh->getCoordsVertices().size();

        vertexbuffer.set_used(nvertices);
        indexbuffer.set_used(ntriangles * 3);

        for (unsigned int i = 0; i < nvertices; i++) {
            vertexbuffer[i] = video::S3DVertex(                                    //
                (f32)vertices[i].x(), (f32)vertices[i].y(), (f32)vertices[i].z(),  //
                (f32)normals[i].x(), (f32)normals[i].y(), (f32)normals[i].z(),     //
                video::SColor(255, (u32)(cols[i].x() * 255), (u32)(cols[i].y() * 255),
                              (u32)(cols[i].z() * 255)),      //
                (f32)uv_coords[i].x(), (f32)uv_coords[i].y()  //
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
            vertexbuffer[i].Color =
                video::SColor(255, (u32)(cols[i].x() * 255), (u32)(cols[i].y() * 255), (u32)(cols[i].z() * 255));
        }
    }

    irrmesh->setDirty();                                         // to force update of hardware buffers
    meshnode->setMaterialFlag(video::EMF_COLOR_MATERIAL, true);  // color shading = vertexes  color
}

void ChIrrNodeProxyToAsset::UpdateGlyphs(std::shared_ptr<ChGlyphs> glyphs) {
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

void ChIrrNodeProxyToAsset::UpdateSurface(std::shared_ptr<ChSurfaceShape> surface) {
    std::shared_ptr<geometry::ChSurface> msurface = surface->GetSurfaceGeometry();

    // Set color.
    auto vis = std::static_pointer_cast<ChVisualization>(visualization_asset);
    ChColor col = vis->GetColor();
    video::SColor clr((u32)(col.A * 255), (u32)(col.R * 255), (u32)(col.G * 255), (u32)(col.B * 255));

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
                P = vis->Pos + vis->Rot * P;

                ChVector<> N;
                msurface->Normal(N, mU, mV);
                N = vis->Rot * N;

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
                P = vis->Pos + vis->Rot * P;

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
                P = vis->Pos + vis->Rot * P;

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

void ChIrrNodeProxyToAsset::UpdateLine(std::shared_ptr<geometry::ChLine> line, unsigned int nvertexes) {
    unsigned int ntriangles = nvertexes - 1;

    // Set color.
    auto vis = std::static_pointer_cast<ChVisualization>(visualization_asset);
    ChColor col = vis->GetColor();
    video::SColor clr((u32)(col.A * 255), (u32)(col.R * 255), (u32)(col.G * 255), (u32)(col.B * 255));

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
    t1 = vis->Pos + vis->Rot * t1;

    irrmesh->getVertexBuffer()[0] = video::S3DVertex((f32)t1.x(), (f32)t1.y(), (f32)t1.z(), 1, 0, 0, clr, 0, 0);

    double maxU = 1;
    if (auto mline_path = std::dynamic_pointer_cast<geometry::ChLinePath>(line))
        maxU = mline_path->GetPathDuration();

    for (unsigned int ig = 0; ig < ntriangles; ++ig) {
        double mU = maxU * ((double)ig / (double)(ntriangles - 1));  // abscissa

        ChVector<> t2;
        line->Evaluate(t2, mU);
        t2 = vis->Pos + vis->Rot * t2;

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
