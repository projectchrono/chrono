// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================

#include "chrono/assets/ChAsset.h"
#include "chrono/assets/ChGlyphs.h"
#include "chrono/assets/ChLineShape.h"
#include "chrono/assets/ChPathShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/core/ChVector.h"

#include "chrono_irrlicht/ChIrrNodeProxyToAsset.h"

namespace chrono {
namespace irrlicht {

using namespace irr;

ChIrrNodeProxyToAsset::ChIrrNodeProxyToAsset(std::shared_ptr<ChAsset> myvisualization, ISceneNode* parent)
    : ISceneNode(parent, parent->getSceneManager(), 0), do_update(true) {
#ifdef _DEBUG
    setDebugName("ChIrrNodeProxyToAsset");
#endif

    // Set the shared pointer to the asset
    visualization_asset = myvisualization;
}

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

// Updates the child mesh to reflect the ChAsset
void ChIrrNodeProxyToAsset::Update() {
    if (!do_update)
        return;

    if (!visualization_asset)
        return;

    if (auto trianglemesh = std::dynamic_pointer_cast<ChTriangleMeshShape>(visualization_asset)) {
        // Fetch the 1st child, i.e. the mesh
        ISceneNode* mchildnode = *(getChildren().begin());
        if (!mchildnode)
            return;

        if (!(mchildnode->getType() == scene::ESNT_MESH))
            return;
        scene::IMeshSceneNode* meshnode =
            (scene::IMeshSceneNode*)mchildnode;  // dynamic_cast not enabled in Irrlicht dll

        scene::IMesh* amesh = meshnode->getMesh();
        if (amesh->getMeshBufferCount() == 0)
            return;

        geometry::ChTriangleMeshConnected* mmesh = &trianglemesh->GetMesh();
        unsigned int ntriangles = (unsigned int)mmesh->getIndicesVertexes().size();
        unsigned int nvertexes =
            ntriangles * 3;  // this is suboptimal because some vertexes might be shared, but easier now..

        // SMeshBuffer* irrmesh = (SMeshBuffer*)amesh->getMeshBuffer(0);
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

            irrmesh->getVertexBuffer()[0 + itri * 3] = irr::video::S3DVertex(
                (f32)t1.x(), (f32)t1.y(), (f32)t1.z(), (f32)n1.x(), (f32)n1.y(), (f32)n1.z(),
                irr::video::SColor(255, (u32)(col1.x() * 255), (u32)(col1.y() * 255), (u32)(col1.z() * 255)), (f32)uv1.x(),
                (f32)uv1.y());

            irrmesh->getVertexBuffer()[1 + itri * 3] = irr::video::S3DVertex(
                (f32)t2.x(), (f32)t2.y(), (f32)t2.z(), (f32)n2.x(), (f32)n2.y(), (f32)n2.z(),
                irr::video::SColor(255, (u32)(col2.x() * 255), (u32)(col2.y() * 255), (u32)(col2.z() * 255)), (f32)uv2.x(),
                (f32)uv2.y());

            irrmesh->getVertexBuffer()[2 + itri * 3] = irr::video::S3DVertex(
                (f32)t3.x(), (f32)t3.y(), (f32)t3.z(), (f32)n3.x(), (f32)n3.y(), (f32)n3.z(),
                irr::video::SColor(255, (u32)(col3.x() * 255), (u32)(col3.y() * 255), (u32)(col3.z() * 255)), (f32)uv3.x(),
                (f32)uv3.y());

            irrmesh->getIndexBuffer().setValue(0 + itri * 3, 0 + itri * 3);
            irrmesh->getIndexBuffer().setValue(1 + itri * 3, 1 + itri * 3);
            irrmesh->getIndexBuffer().setValue(2 + itri * 3, 2 + itri * 3);
        }

        irrmesh->setDirty();                                  // to force update of hardware buffers
        irrmesh->setHardwareMappingHint(scene::EHM_DYNAMIC);  // EHM_NEVER); //EHM_DYNAMIC for faster hw mapping
        irrmesh->recalculateBoundingBox();

        meshnode->setAutomaticCulling(scene::EAC_OFF);

        meshnode->setMaterialFlag(irr::video::EMF_WIREFRAME, trianglemesh->IsWireframe());
        meshnode->setMaterialFlag(irr::video::EMF_LIGHTING,
                                  !trianglemesh->IsWireframe());  // avoid shading for wireframes
        meshnode->setMaterialFlag(irr::video::EMF_BACK_FACE_CULLING, trianglemesh->IsBackfaceCull());

        meshnode->setMaterialFlag(irr::video::EMF_COLOR_MATERIAL, true);  // so color shading = vertexes  color
    }

    if (auto mglyphs = std::dynamic_pointer_cast<ChGlyphs>(visualization_asset)) {
        // Fetch the 1st child, i.e. the mesh
        ISceneNode* mchildnode = *(getChildren().begin());
        if (!mchildnode || mchildnode->getType() != scene::ESNT_MESH)
            return;

        scene::IMeshSceneNode* meshnode =
            (scene::IMeshSceneNode*)mchildnode;  // dynamic_cast not enabled in Irrlicht dll
        scene::IMesh* amesh = meshnode->getMesh();
        if (amesh->getMeshBufferCount() == 0)
            return;

        // SMeshBuffer* irrmesh = (SMeshBuffer*)amesh->getMeshBuffer(0);
        scene::CDynamicMeshBuffer* irrmesh = (scene::CDynamicMeshBuffer*)amesh->getMeshBuffer(0);

        size_t ntriangles = 0;
        size_t nvertexes = 0;

        switch (mglyphs->GetDrawMode()) {
            case ChGlyphs::GLYPH_POINT:
                ntriangles = 12 * mglyphs->GetNumberOfGlyphs();
                nvertexes = 24 * mglyphs->GetNumberOfGlyphs();
                break;
            case ChGlyphs::GLYPH_VECTOR:
                ntriangles = 1 * mglyphs->GetNumberOfGlyphs();
                nvertexes = 3 * mglyphs->GetNumberOfGlyphs();
                break;
            case ChGlyphs::GLYPH_COORDSYS:
                ntriangles = 3 * mglyphs->GetNumberOfGlyphs();
                nvertexes = 9 * mglyphs->GetNumberOfGlyphs();
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

        if (mglyphs->GetDrawMode() == ChGlyphs::GLYPH_POINT) {
            const u32 u[36] = {0,  1,  2,  0,  2,  3,  4,  6,  5,  4,  7,  6,  8,  9,  10, 8,  10, 11,
                               12, 14, 13, 12, 15, 14, 16, 18, 17, 16, 19, 18, 20, 21, 22, 20, 22, 23};

            int itri = 0;

            for (unsigned int ig = 0; ig < mglyphs->points.size(); ++ig) {
                ChVector<> t1 = mglyphs->points[ig];
                ChColor mcol = mglyphs->colors[ig];
                irr::video::SColor clr(255, (u32)(mcol.R * 255), (u32)(mcol.G * 255), (u32)(mcol.B * 255));

                // create a small cube per each vertex
                unsigned int voffs = ig * 24;
                f32 s = (f32)(mglyphs->GetGlyphsSize() * 0.5);

                irrmesh->getVertexBuffer()[0 + voffs] = irr::video::S3DVertex(-s, -s, -s, 0, 0, -1, clr, 0, 0);
                irrmesh->getVertexBuffer()[1 + voffs] = irr::video::S3DVertex(-s, s, -s, 0, 0, -1, clr, 0, 1);
                irrmesh->getVertexBuffer()[2 + voffs] = irr::video::S3DVertex(s, s, -s, 0, 0, -1, clr, 1, 1);
                irrmesh->getVertexBuffer()[3 + voffs] = irr::video::S3DVertex(s, -s, -s, 0, 0, -1, clr, 1, 0);

                irrmesh->getVertexBuffer()[4 + voffs] = irr::video::S3DVertex(-s, -s, s, 0, 0, 1, clr, 0, 0);
                irrmesh->getVertexBuffer()[5 + voffs] = irr::video::S3DVertex(-s, s, s, 0, 0, 1, clr, 0, 1);
                irrmesh->getVertexBuffer()[6 + voffs] = irr::video::S3DVertex(s, s, s, 0, 0, 1, clr, 1, 1);
                irrmesh->getVertexBuffer()[7 + voffs] = irr::video::S3DVertex(s, -s, s, 0, 0, 1, clr, 1, 0);

                irrmesh->getVertexBuffer()[8 + voffs] = irr::video::S3DVertex(-s, -s, -s, -1, 0, 0, clr, 0, 0);
                irrmesh->getVertexBuffer()[9 + voffs] = irr::video::S3DVertex(-s, -s, s, -1, 0, 0, clr, 0, 1);
                irrmesh->getVertexBuffer()[10 + voffs] = irr::video::S3DVertex(-s, s, s, -1, 0, 0, clr, 1, 1);
                irrmesh->getVertexBuffer()[11 + voffs] = irr::video::S3DVertex(-s, s, -s, -1, 0, 0, clr, 1, 0);

                irrmesh->getVertexBuffer()[12 + voffs] = irr::video::S3DVertex(s, -s, -s, 1, 0, 0, clr, 0, 0);
                irrmesh->getVertexBuffer()[13 + voffs] = irr::video::S3DVertex(s, -s, s, 1, 0, 0, clr, 0, 1);
                irrmesh->getVertexBuffer()[14 + voffs] = irr::video::S3DVertex(s, s, s, 1, 0, 0, clr, 1, 1);
                irrmesh->getVertexBuffer()[15 + voffs] = irr::video::S3DVertex(s, s, -s, 1, 0, 0, clr, 1, 0);

                irrmesh->getVertexBuffer()[16 + voffs] = irr::video::S3DVertex(-s, -s, -s, 0, -1, 0, clr, 0, 0);
                irrmesh->getVertexBuffer()[17 + voffs] = irr::video::S3DVertex(-s, -s, s, 0, -1, 0, clr, 0, 1);
                irrmesh->getVertexBuffer()[18 + voffs] = irr::video::S3DVertex(s, -s, s, 0, -1, 0, clr, 1, 1);
                irrmesh->getVertexBuffer()[19 + voffs] = irr::video::S3DVertex(s, -s, -s, 0, -1, 0, clr, 1, 0);

                irrmesh->getVertexBuffer()[20 + voffs] = irr::video::S3DVertex(-s, s, -s, 0, 1, 0, clr, 0, 0);
                irrmesh->getVertexBuffer()[21 + voffs] = irr::video::S3DVertex(-s, s, s, 0, 1, 0, clr, 0, 1);
                irrmesh->getVertexBuffer()[22 + voffs] = irr::video::S3DVertex(s, s, s, 0, 1, 0, clr, 1, 1);
                irrmesh->getVertexBuffer()[23 + voffs] = irr::video::S3DVertex(s, s, -s, 0, 1, 0, clr, 1, 0);

                for (u32 i = 0; i < 24; ++i) {
                    irrmesh->getVertexBuffer()[i + voffs].Pos += irr::core::vector3df((f32)t1.x(), (f32)t1.y(), (f32)t1.z());
                    // buffer->BoundingBox.addInternalPoint(buffer->Vertices[i].Pos);
                }

                for (u32 i = 0; i < 36; ++i)
                    irrmesh->getIndexBuffer().setValue(i + itri, u[i] + voffs);

                itri += 36;
            }
        }

        if (mglyphs->GetDrawMode() == ChGlyphs::GLYPH_VECTOR) {
            int itri = 0;
            for (unsigned int ig = 0; ig < mglyphs->points.size(); ++ig) {
                ChVector<> t1 = mglyphs->points[ig];
                ChVector<> t2 = mglyphs->vectors[ig] + t1;
                ChColor mcol = mglyphs->colors[ig];
                irr::video::SColor clr(255, (u32)(mcol.R * 255), (u32)(mcol.G * 255), (u32)(mcol.B * 255));

                // create a  small line (a degenerate triangle) per each vector
                irrmesh->getVertexBuffer()[0 + ig * 3] =
                    irr::video::S3DVertex((f32)t1.x(), (f32)t1.y(), (f32)t1.z(), 1, 0, 0, clr, 0, 0);

                irrmesh->getVertexBuffer()[1 + ig * 3] =
                    irr::video::S3DVertex((f32)t2.x(), (f32)t2.y(), (f32)t2.z(), 1, 0, 0, clr, 0, 0);

                irrmesh->getVertexBuffer()[2 + ig * 3] =
                    irr::video::S3DVertex((f32)t2.x(), (f32)t2.y(), (f32)t2.z(), 1, 0, 0, clr, 0, 0);

                irrmesh->getIndexBuffer().setValue(0 + itri * 3, 0 + ig * 3);
                irrmesh->getIndexBuffer().setValue(1 + itri * 3, 1 + ig * 3);
                irrmesh->getIndexBuffer().setValue(2 + itri * 3, 2 + ig * 3);

                ++itri;
            }
        }

        if (mglyphs->GetDrawMode() == ChGlyphs::GLYPH_COORDSYS) {
            int itri = 0;

            for (unsigned int ig = 0; ig < mglyphs->points.size(); ++ig) {
                ChVector<> t1 = mglyphs->points[ig];
                ChVector<> t2;

                // X axis - create a  small line (a degenerate triangle) per each vector
                t2 = mglyphs->rotations[ig].Rotate(ChVector<>(1, 0, 0) * mglyphs->GetGlyphsSize()) + t1;

                irrmesh->getVertexBuffer()[0 + ig * 9] = irr::video::S3DVertex(
                    (f32)t1.x(), (f32)t1.y(), (f32)t1.z(), 1, 0, 0, irr::video::SColor(255, 255, 0, 0), 0, 0);
                irrmesh->getVertexBuffer()[1 + ig * 9] = irr::video::S3DVertex(
                    (f32)t2.x(), (f32)t2.y(), (f32)t2.z(), 1, 0, 0, irr::video::SColor(255, 255, 0, 0), 0, 0);
                irrmesh->getVertexBuffer()[2 + ig * 9] = irr::video::S3DVertex(
                    (f32)t2.x(), (f32)t2.y(), (f32)t2.z(), 1, 0, 0, irr::video::SColor(255, 255, 0, 0), 0, 0);

                irrmesh->getIndexBuffer().setValue(0 + itri * 3, 0 + ig * 9);
                irrmesh->getIndexBuffer().setValue(1 + itri * 3, 1 + ig * 9);
                irrmesh->getIndexBuffer().setValue(2 + itri * 3, 2 + ig * 9);

                ++itri;

                // Y axis
                t2 = mglyphs->rotations[ig].Rotate(ChVector<>(0, 1, 0) * mglyphs->GetGlyphsSize()) + t1;

                irrmesh->getVertexBuffer()[3 + ig * 9] = irr::video::S3DVertex(
                    (f32)t1.x(), (f32)t1.y(), (f32)t1.z(), 1, 0, 0, irr::video::SColor(255, 0, 255, 0), 0, 0);
                irrmesh->getVertexBuffer()[4 + ig * 9] = irr::video::S3DVertex(
                    (f32)t2.x(), (f32)t2.y(), (f32)t2.z(), 1, 0, 0, irr::video::SColor(255, 0, 255, 0), 0, 0);
                irrmesh->getVertexBuffer()[5 + ig * 9] = irr::video::S3DVertex(
                    (f32)t2.x(), (f32)t2.y(), (f32)t2.z(), 1, 0, 0, irr::video::SColor(255, 0, 255, 0), 0, 0);

                irrmesh->getIndexBuffer().setValue(0 + itri * 3, 3 + ig * 9);
                irrmesh->getIndexBuffer().setValue(1 + itri * 3, 4 + ig * 9);
                irrmesh->getIndexBuffer().setValue(2 + itri * 3, 5 + ig * 9);

                ++itri;

                // Z axis
                t2 = mglyphs->rotations[ig].Rotate(ChVector<>(0, 0, 1) * mglyphs->GetGlyphsSize()) + t1;

                irrmesh->getVertexBuffer()[6 + ig * 9] = irr::video::S3DVertex(
                    (f32)t1.x(), (f32)t1.y(), (f32)t1.z(), 1, 0, 0, irr::video::SColor(255, 0, 0, 255), 0, 0);
                irrmesh->getVertexBuffer()[7 + ig * 9] = irr::video::S3DVertex(
                    (f32)t2.x(), (f32)t2.y(), (f32)t2.z(), 1, 0, 0, irr::video::SColor(255, 0, 0, 255), 0, 0);
                irrmesh->getVertexBuffer()[8 + ig * 9] = irr::video::S3DVertex(
                    (f32)t2.x(), (f32)t2.y(), (f32)t2.z(), 1, 0, 0, irr::video::SColor(255, 0, 0, 255), 0, 0);

                irrmesh->getIndexBuffer().setValue(0 + itri * 3, 6 + ig * 9);
                irrmesh->getIndexBuffer().setValue(1 + itri * 3, 7 + ig * 9);
                irrmesh->getIndexBuffer().setValue(2 + itri * 3, 8 + ig * 9);

                ++itri;
            }
        }

        irrmesh->setDirty();                                  // to force update of hardware buffers
        irrmesh->setHardwareMappingHint(scene::EHM_DYNAMIC);  // EHM_NEVER); //EHM_DYNAMIC for faster hw mapping
        irrmesh->recalculateBoundingBox();

        if (mglyphs->GetDrawMode() == ChGlyphs::GLYPH_VECTOR || mglyphs->GetDrawMode() == ChGlyphs::GLYPH_COORDSYS) {
            meshnode->setMaterialFlag(irr::video::EMF_WIREFRAME, true);
            meshnode->setMaterialFlag(irr::video::EMF_LIGHTING, false);  // avoid shading for wireframe
            meshnode->setMaterialFlag(irr::video::EMF_BACK_FACE_CULLING, false);
        } else {
            meshnode->setMaterialFlag(irr::video::EMF_WIREFRAME, false);
            meshnode->setMaterialFlag(irr::video::EMF_LIGHTING, true);
            meshnode->setMaterialFlag(irr::video::EMF_BACK_FACE_CULLING, true);
        }

        if (mglyphs->GetZbufferHide() == true)
            meshnode->setMaterialFlag(irr::video::EMF_ZBUFFER, true);
        else
            meshnode->setMaterialFlag(irr::video::EMF_ZBUFFER, false);

        meshnode->setMaterialFlag(irr::video::EMF_COLOR_MATERIAL, true);  // so color shading = vertexes  color
    }

    auto path_shape = std::dynamic_pointer_cast<ChPathShape>(visualization_asset);
    auto line_shape = std::dynamic_pointer_cast<ChLineShape>(visualization_asset);
    if (path_shape || line_shape) {
        std::shared_ptr<geometry::ChLine> mline;
        if (path_shape)
            mline = path_shape->GetPathGeometry();
        if (line_shape)
            mline = line_shape->GetLineGeometry();

        // Set color.
        auto vis = std::static_pointer_cast<ChVisualization>(visualization_asset);
        ChColor col = vis->GetColor();
        irr::video::SColor clr((u32)(col.A * 255), (u32)(col.R * 255), (u32)(col.G * 255), (u32)(col.B * 255));

        // Fetch the 1st child, i.e. the mesh
        ISceneNode* mchildnode = *(getChildren().begin());
        if (!mchildnode || mchildnode->getType() != scene::ESNT_MESH)
            return;

        scene::IMeshSceneNode* meshnode =
            (scene::IMeshSceneNode*)mchildnode;  // dynamic_cast not enabled in Irrlicht dll
        scene::IMesh* amesh = meshnode->getMesh();
        if (amesh->getMeshBufferCount() == 0)
            return;

        // SMeshBuffer* irrmesh = (SMeshBuffer*)amesh->getMeshBuffer(0);
        scene::CDynamicMeshBuffer* irrmesh = (scene::CDynamicMeshBuffer*)amesh->getMeshBuffer(0);

        size_t nvertexes = 200;
        size_t ntriangles = nvertexes - 1;

        // smart inflating of allocated buffers, only if necessary, and once in a while shrinking
        if (irrmesh->getIndexBuffer().allocated_size() > (ntriangles * 3) * 1.5)
            irrmesh->getIndexBuffer().reallocate(0);  // clear();
        if (irrmesh->getVertexBuffer().allocated_size() > nvertexes * 1.5)
            irrmesh->getVertexBuffer().reallocate(0);

        irrmesh->getIndexBuffer().set_used((u32)ntriangles * 3);
        irrmesh->getVertexBuffer().set_used((u32)nvertexes);

        int itri = 0;

        ChVector<> t1;
        mline->Evaluate(t1, 0, 0, 0);
        t1 = vis->Pos + vis->Rot * t1;

        irrmesh->getVertexBuffer()[0] = irr::video::S3DVertex((f32)t1.x(), (f32)t1.y(), (f32)t1.z(), 1, 0, 0, clr, 0, 0);

        double maxU = 1;
        if (auto mline_path = std::dynamic_pointer_cast<geometry::ChLinePath>(mline))
            maxU = mline_path->GetPathDuration();

        for (unsigned int ig = 0; ig < ntriangles; ++ig) {
            double mU = maxU * ((double)ig / (double)(ntriangles - 1));  // abscyssa

            ChVector<> t2;
            mline->Evaluate(t2, mU, 0, 0);
            t2 = vis->Pos + vis->Rot * t2;

            // create a  small line (a degenerate triangle) per each vector

            irrmesh->getVertexBuffer()[1 + ig] =
                irr::video::S3DVertex((f32)t2.x(), (f32)t2.y(), (f32)t2.z(), 1, 0, 0, clr, 0, 0);

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

        meshnode->setMaterialFlag(irr::video::EMF_WIREFRAME, true);
        meshnode->setMaterialFlag(irr::video::EMF_LIGHTING, false);  // avoid shading for wireframe
        meshnode->setMaterialFlag(irr::video::EMF_BACK_FACE_CULLING, false);

        // meshnode->setMaterialFlag(irr::video::EMF_COLOR_MATERIAL, true); // so color shading = vertexes  color
    }
}

}  // end namespace irrlicht
}  // end namespace chrono
