//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//
// File author: A.Tasora

#include "core/ChVector.h"
#include "assets/ChAsset.h"
#include "assets/ChTriangleMeshShape.h"
#include "assets/ChGlyphs.h"

#include "unit_IRRLICHT/ChIrrNodeProxyToAsset.h"


namespace irr {
namespace scene {


ChIrrNodeProxyToAsset::ChIrrNodeProxyToAsset(chrono::ChSharedPtr<chrono::ChAsset>  myvisualization,
                                             ISceneNode*                           parent)
: ISceneNode(parent, parent->getSceneManager(), 0),
  do_update(true)
{
#ifdef _DEBUG
  setDebugName("ChIrrNodeProxyToAsset");
#endif

  // Set the shared pointer to the asset
  visualization_asset = myvisualization;
}


ISceneNode* ChIrrNodeProxyToAsset::clone(ISceneNode*    newParent,
                                         ISceneManager* newManager)
{
  if (!newParent)
    newParent = Parent;
  if (!newManager)
    newManager = SceneManager;

  ChIrrNodeProxyToAsset* nb = new ChIrrNodeProxyToAsset(visualization_asset, newParent);

  nb->cloneMembers(this, newManager);
  nb->Box = Box;
  nb->visualization_asset = visualization_asset;

  if ( newParent )
    nb->drop();
  return nb;
}


// Updates the child mesh to reflect the ChAsset
void ChIrrNodeProxyToAsset::Update()
{
  if (!do_update)
    return;

  if (visualization_asset.IsNull())
    return;

  if (visualization_asset.IsType<chrono::ChTriangleMeshShape>())
  {
    chrono::ChSharedPtr<chrono::ChTriangleMeshShape> trianglemesh = visualization_asset.DynamicCastTo<chrono::ChTriangleMeshShape>();

    // Fetch the 1st child, i.e. the mesh
    ISceneNode* mchildnode = *(getChildren().begin());
    if (!mchildnode) 
      return;

    if (!(mchildnode->getType() == ESNT_MESH))
      return;
    IMeshSceneNode* meshnode = (IMeshSceneNode*)mchildnode; // dynamic_cast not enabled in Irrlicht dll

    IMesh* amesh = meshnode->getMesh();
    if (amesh->getMeshBufferCount() == 0)
      return;

    chrono::geometry::ChTriangleMeshConnected* mmesh = &trianglemesh->GetMesh();
    unsigned int ntriangles = mmesh->getIndicesVertexes().size();
    unsigned int nvertexes = ntriangles * 3; // this is suboptimal because some vertexes might be shared, but easier now..

    //SMeshBuffer* irrmesh = (SMeshBuffer*)amesh->getMeshBuffer(0);
    CDynamicMeshBuffer* irrmesh = (CDynamicMeshBuffer*)amesh->getMeshBuffer(0);

    // smart inflating of allocated buffers, only if necessary, and once in a while shrinking
    if (irrmesh->getIndexBuffer().allocated_size() > (ntriangles*3) * 1.5)
      irrmesh->getIndexBuffer().reallocate(0);//clear();
    if (irrmesh->getVertexBuffer().allocated_size() > nvertexes * 1.5)
      irrmesh->getVertexBuffer().reallocate(0);

    irrmesh->getIndexBuffer().set_used(ntriangles*3);
    irrmesh->getVertexBuffer().set_used(nvertexes); 

    // set buffers
    for (unsigned int itri = 0; itri < ntriangles; itri++)
    {
      chrono::ChVector<> t1 = mmesh->getCoordsVertices()[ mmesh->getIndicesVertexes()[itri].x ];
      chrono::ChVector<> t2 = mmesh->getCoordsVertices()[ mmesh->getIndicesVertexes()[itri].y ];
      chrono::ChVector<> t3 = mmesh->getCoordsVertices()[ mmesh->getIndicesVertexes()[itri].z ];
      chrono::ChVector<> n1, n2, n3;
      if ( mmesh->getIndicesNormals().size() == mmesh->getIndicesVertexes().size() ) {
        n1 = mmesh->getCoordsNormals()[ mmesh->getIndicesNormals()[itri].x ];
        n2 = mmesh->getCoordsNormals()[ mmesh->getIndicesNormals()[itri].y ];
        n3 = mmesh->getCoordsNormals()[ mmesh->getIndicesNormals()[itri].z ];
      } else {
        n1 = Vcross(t2-t1, t3-t1).GetNormalized();
        n2 = n1; 
        n3 = n1;
      }

      chrono::ChVector<> uv1,uv2,uv3;
      if ( mmesh->getIndicesUV().size() == mmesh->getIndicesVertexes().size() ) {
        uv1 = mmesh->getCoordsUV()[ mmesh->getIndicesUV()[itri].x ];
        uv2 = mmesh->getCoordsUV()[ mmesh->getIndicesUV()[itri].y ];
        uv3 = mmesh->getCoordsUV()[ mmesh->getIndicesUV()[itri].z ];
      } else if ( mmesh->getIndicesUV().size() == 0 && 
                  mmesh->getCoordsUV().size()  == mmesh->getCoordsVertices().size() ) {
        uv1 = mmesh->getCoordsUV()[ mmesh->getIndicesVertexes()[itri].x ];
        uv2 = mmesh->getCoordsUV()[ mmesh->getIndicesVertexes()[itri].y ];
        uv3 = mmesh->getCoordsUV()[ mmesh->getIndicesVertexes()[itri].z ];
      } else {
        uv1=uv2=uv3= chrono::VNULL;
      }

      chrono::ChVector<float> col1,col2,col3;
      if ( mmesh->getIndicesColors().size() == mmesh->getIndicesVertexes().size() ) {
        col1 = mmesh->getCoordsColors()[ mmesh->getIndicesColors()[itri].x ];
        col2 = mmesh->getCoordsColors()[ mmesh->getIndicesColors()[itri].y ];
        col3 = mmesh->getCoordsColors()[ mmesh->getIndicesColors()[itri].z ];
      } else if ( mmesh->getIndicesColors().size() == 0 && 
                  mmesh->getCoordsColors().size() == mmesh->getCoordsVertices().size() ) {
        col1 = mmesh->getCoordsColors()[ mmesh->getIndicesVertexes()[itri].x ];
        col2 = mmesh->getCoordsColors()[ mmesh->getIndicesVertexes()[itri].y ];
        col3 = mmesh->getCoordsColors()[ mmesh->getIndicesVertexes()[itri].z ];
      } else {
        col1=col2=col3= chrono::ChVector<float>(trianglemesh->GetColor().R,
                                                trianglemesh->GetColor().G,
                                                trianglemesh->GetColor().B);
      }

      irrmesh->getVertexBuffer()[0+itri*3] = 
        video::S3DVertex((f32)t1.x, (f32)t1.y, (f32)t1.z,
                         (f32)n1.x, (f32)n1.y, (f32)n1.z,
                         video::SColor(255, (u32)(col1.x*255), (u32)(col1.y*255), (u32)(col1.z*255)),
                         (f32)uv1.x, (f32)uv1.y);

      irrmesh->getVertexBuffer()[1+itri*3] = 
        video::S3DVertex((f32)t2.x, (f32)t2.y, (f32)t2.z,
                         (f32)n2.x, (f32)n2.y, (f32)n2.z,
                         video::SColor(255, (u32)(col2.x*255), (u32)(col2.y*255), (u32)(col2.z*255)),
                         (f32)uv2.x, (f32)uv2.y);

      irrmesh->getVertexBuffer()[2+itri*3] = 
        video::S3DVertex((f32)t3.x, (f32)t3.y, (f32)t3.z,
                         (f32)n3.x, (f32)n3.y, (f32)n3.z,
                         video::SColor(255, (u32)(col3.x*255), (u32)(col3.y*255), (u32)(col3.z*255)),
                         (f32)uv3.x, (f32)uv3.y);

      irrmesh->getIndexBuffer().setValue(0+itri*3, 0+itri*3);
      irrmesh->getIndexBuffer().setValue(1+itri*3, 1+itri*3);
      irrmesh->getIndexBuffer().setValue(2+itri*3, 2+itri*3);
    }

    irrmesh->setDirty(); // to force update of hardware buffers
    irrmesh->setHardwareMappingHint(EHM_DYNAMIC);//EHM_NEVER); //EHM_DYNAMIC for faster hw mapping
    irrmesh->recalculateBoundingBox();

    meshnode->setMaterialFlag(video::EMF_WIREFRAME,         trianglemesh->IsWireframe() );
    meshnode->setMaterialFlag(video::EMF_LIGHTING,          !trianglemesh->IsWireframe() ); // avoid shading for wireframes
    meshnode->setMaterialFlag(video::EMF_BACK_FACE_CULLING, trianglemesh->IsBackfaceCull() );

    meshnode->setMaterialFlag(video::EMF_COLOR_MATERIAL, true); // so color shading = vertexes  color

  }

  if (visualization_asset.IsType<chrono::ChGlyphs>())
  {
    chrono::ChSharedPtr<chrono::ChGlyphs> mglyphs = visualization_asset.DynamicCastTo<chrono::ChGlyphs>();

    // Fetch the 1st child, i.e. the mesh
    ISceneNode* mchildnode = *(getChildren().begin()); 
    if (!mchildnode || mchildnode->getType() != ESNT_MESH) 
      return;

    IMeshSceneNode* meshnode = (IMeshSceneNode*)mchildnode; // dynamic_cast not enabled in Irrlicht dll
    IMesh*          amesh    = meshnode->getMesh();
    if (amesh->getMeshBufferCount() == 0)
      return;

    //SMeshBuffer* irrmesh = (SMeshBuffer*)amesh->getMeshBuffer(0);
    CDynamicMeshBuffer* irrmesh = (CDynamicMeshBuffer*)amesh->getMeshBuffer(0);

    unsigned int ntriangles = 0;
    unsigned int nvertexes = 0;

    switch (mglyphs->GetDrawMode()) {
    case chrono::ChGlyphs::GLYPH_POINT:
      ntriangles =  12 * mglyphs->GetNumberOfGlyphs();
      nvertexes =   24 * mglyphs->GetNumberOfGlyphs();
      break;
    case chrono::ChGlyphs::GLYPH_VECTOR:
      ntriangles =  1 * mglyphs->GetNumberOfGlyphs();
      nvertexes =   3 * mglyphs->GetNumberOfGlyphs();
      break;
    case chrono::ChGlyphs::GLYPH_COORDSYS:
      ntriangles =  3 * mglyphs->GetNumberOfGlyphs();
      nvertexes =   9 * mglyphs->GetNumberOfGlyphs();
      break;
    }

    // smart inflating of allocated buffers, only if necessary, and once in a while shrinking
    if (irrmesh->getIndexBuffer().allocated_size() > (ntriangles*3) * 1.5)
      irrmesh->getIndexBuffer().reallocate(0);//clear();
    if (irrmesh->getVertexBuffer().allocated_size() > nvertexes * 1.5)
      irrmesh->getVertexBuffer().reallocate(0);

    irrmesh->getIndexBuffer().set_used(ntriangles*3);
    irrmesh->getVertexBuffer().set_used(nvertexes);

    // set buffers

    if (mglyphs->GetDrawMode() == chrono::ChGlyphs::GLYPH_POINT)
    {
      const u32 u[36] = {   0, 1, 2,    0, 2, 3,
                            4, 6, 5,    4, 7, 6,
                            8, 9,10,    8,10,11,
                           12,14,13,   12,15,14,
                           16,18,17,   16,19,18,
                           20,21,22,   20,22,23   };

      int itri = 0;

      for (unsigned int ig= 0; ig< mglyphs->points.size(); ++ig)
      {
        chrono::ChVector<> t1 =  mglyphs->points[ig];
        chrono::ChColor mcol  =  mglyphs->colors[ig];
        video::SColor clr(255, (u32)(mcol.R *255), (u32)(mcol.G *255), (u32)(mcol.B *255) );

        // create a small cube per each vertex
        unsigned int voffs = ig*24;
        f32 s = (f32) (mglyphs->GetGlyphsSize() * 0.5);

        irrmesh->getVertexBuffer()[0+voffs]  = video::S3DVertex(-s,-s,-s,  0, 0,-1, clr, 0, 0);
        irrmesh->getVertexBuffer()[1+voffs]  = video::S3DVertex(-s, s,-s,  0, 0,-1, clr, 0, 1);
        irrmesh->getVertexBuffer()[2+voffs]  = video::S3DVertex( s, s,-s,  0, 0,-1, clr, 1, 1);
        irrmesh->getVertexBuffer()[3+voffs]  = video::S3DVertex( s,-s,-s,  0, 0,-1, clr, 1, 0);

        irrmesh->getVertexBuffer()[4+voffs]  = video::S3DVertex(-s,-s, s,  0, 0, 1, clr, 0, 0);
        irrmesh->getVertexBuffer()[5+voffs]  = video::S3DVertex(-s, s, s,  0, 0, 1, clr, 0, 1);
        irrmesh->getVertexBuffer()[6+voffs]  = video::S3DVertex( s, s, s,  0, 0, 1, clr, 1, 1);
        irrmesh->getVertexBuffer()[7+voffs]  = video::S3DVertex( s,-s, s,  0, 0, 1, clr, 1, 0);

        irrmesh->getVertexBuffer()[8+voffs]  = video::S3DVertex(-s,-s,-s, -1, 0, 0, clr, 0, 0);
        irrmesh->getVertexBuffer()[9+voffs]  = video::S3DVertex(-s,-s, s, -1, 0, 0, clr, 0, 1);
        irrmesh->getVertexBuffer()[10+voffs] = video::S3DVertex(-s, s, s, -1, 0, 0, clr, 1, 1);
        irrmesh->getVertexBuffer()[11+voffs] = video::S3DVertex(-s, s,-s, -1, 0, 0, clr, 1, 0);

        irrmesh->getVertexBuffer()[12+voffs] = video::S3DVertex( s,-s,-s,  1, 0, 0, clr, 0, 0);
        irrmesh->getVertexBuffer()[13+voffs] = video::S3DVertex( s,-s, s,  1, 0, 0, clr, 0, 1);
        irrmesh->getVertexBuffer()[14+voffs] = video::S3DVertex( s, s, s,  1, 0, 0, clr, 1, 1);
        irrmesh->getVertexBuffer()[15+voffs] = video::S3DVertex( s, s,-s,  1, 0, 0, clr, 1, 0);

        irrmesh->getVertexBuffer()[16+voffs] = video::S3DVertex(-s,-s,-s,  0,-1, 0, clr, 0, 0);
        irrmesh->getVertexBuffer()[17+voffs] = video::S3DVertex(-s,-s, s,  0,-1, 0, clr, 0, 1);
        irrmesh->getVertexBuffer()[18+voffs] = video::S3DVertex( s,-s, s,  0,-1, 0, clr, 1, 1);
        irrmesh->getVertexBuffer()[19+voffs] = video::S3DVertex( s,-s,-s,  0,-1, 0, clr, 1, 0);

        irrmesh->getVertexBuffer()[20+voffs] = video::S3DVertex(-s, s,-s,  0, 1, 0, clr, 0, 0);
        irrmesh->getVertexBuffer()[21+voffs] = video::S3DVertex(-s, s, s,  0, 1, 0, clr, 0, 1);
        irrmesh->getVertexBuffer()[22+voffs] = video::S3DVertex( s, s, s,  0, 1, 0, clr, 1, 1);
        irrmesh->getVertexBuffer()[23+voffs] = video::S3DVertex( s, s,-s,  0, 1, 0, clr, 1, 0);

        for (u32 i=0; i<24; ++i) {
          irrmesh->getVertexBuffer()[i+voffs].Pos += core::vector3df((f32)t1.x, (f32)t1.y, (f32)t1.z);
          //buffer->BoundingBox.addInternalPoint(buffer->Vertices[i].Pos);
        }

        for (u32 i=0; i<36; ++i)
          irrmesh->getIndexBuffer().setValue(i+itri,  u[i]+voffs);

        itri += 36;

      }
    }


    if (mglyphs->GetDrawMode() == chrono::ChGlyphs::GLYPH_VECTOR)
    {
      int itri = 0;
      for (unsigned int ig= 0; ig< mglyphs->points.size(); ++ig)
      {
        chrono::ChVector<> t1 =  mglyphs->points [ig];
        chrono::ChVector<> t2 =  mglyphs->vectors[ig] + t1;
        chrono::ChColor mcol  =  mglyphs->colors [ig];
        video::SColor clr(255, (u32)(mcol.R *255), (u32)(mcol.G *255), (u32)(mcol.B *255) );

        // create a  small line (a degenerate triangle) per each vector
        irrmesh->getVertexBuffer()[0+ig*3] = 
          video::S3DVertex((f32)t1.x, (f32)t1.y, (f32)t1.z,
                           1, 0, 0,
                           clr,
                           0, 0);

        irrmesh->getVertexBuffer()[1+ig*3] = 
          video::S3DVertex((f32)t2.x, (f32)t2.y, (f32)t2.z,
                           1, 0, 0,
                           clr,
                           0, 0);

        irrmesh->getVertexBuffer()[2+ig*3] = 
          video::S3DVertex((f32)t2.x, (f32)t2.y, (f32)t2.z, 
                           1, 0, 0,
                           clr,
                           0, 0);

        irrmesh->getIndexBuffer().setValue(0+itri*3, 0+ig*3);
        irrmesh->getIndexBuffer().setValue(1+itri*3, 1+ig*3);
        irrmesh->getIndexBuffer().setValue(2+itri*3, 2+ig*3);

        ++itri;
      }
    }


    if (mglyphs->GetDrawMode() == chrono::ChGlyphs::GLYPH_COORDSYS)
    {
      int itri = 0;

      for (unsigned int ig= 0; ig< mglyphs->points.size(); ++ig)
      {
        chrono::ChVector<> t1 =  mglyphs->points [ig];
        chrono::ChVector<> t2;

        // X axis - create a  small line (a degenerate triangle) per each vector
        t2 =  mglyphs->rotations[ig].Rotate( chrono::ChVector<>(1,0,0)*mglyphs->GetGlyphsSize() ) + t1;

        irrmesh->getVertexBuffer()[0+ig*9] = 
          video::S3DVertex((f32)t1.x, (f32)t1.y, (f32)t1.z, 1, 0, 0, video::SColor(255,255,0,0), 0, 0);
        irrmesh->getVertexBuffer()[1+ig*9] = 
          video::S3DVertex((f32)t2.x, (f32)t2.y, (f32)t2.z, 1, 0, 0, video::SColor(255,255,0,0), 0, 0);
        irrmesh->getVertexBuffer()[2+ig*9] = 
          video::S3DVertex((f32)t2.x, (f32)t2.y, (f32)t2.z, 1, 0, 0, video::SColor(255,255,0,0), 0, 0);

        irrmesh->getIndexBuffer().setValue(0+itri*3, 0+ig*9);
        irrmesh->getIndexBuffer().setValue(1+itri*3, 1+ig*9);
        irrmesh->getIndexBuffer().setValue(2+itri*3, 2+ig*9);

        ++itri;

        // Y axis
        t2 =  mglyphs->rotations[ig].Rotate( chrono::ChVector<>(0,1,0)*mglyphs->GetGlyphsSize() ) + t1;

        irrmesh->getVertexBuffer()[3+ig*9] = 
          video::S3DVertex((f32)t1.x, (f32)t1.y, (f32)t1.z, 1, 0, 0, video::SColor(255,0,255,0), 0, 0);
        irrmesh->getVertexBuffer()[4+ig*9] = 
          video::S3DVertex((f32)t2.x, (f32)t2.y, (f32)t2.z, 1, 0, 0, video::SColor(255,0,255,0), 0, 0);
        irrmesh->getVertexBuffer()[5+ig*9] = 
          video::S3DVertex((f32)t2.x, (f32)t2.y, (f32)t2.z, 1, 0, 0, video::SColor(255,0,255,0), 0, 0);

        irrmesh->getIndexBuffer().setValue(0+itri*3, 3+ig*9);
        irrmesh->getIndexBuffer().setValue(1+itri*3, 4+ig*9);
        irrmesh->getIndexBuffer().setValue(2+itri*3, 5+ig*9);

        ++itri;

        // Z axis
        t2 =  mglyphs->rotations[ig].Rotate( chrono::ChVector<>(0,0,1)*mglyphs->GetGlyphsSize() ) + t1;

        irrmesh->getVertexBuffer()[6+ig*9] = 
          video::S3DVertex((f32)t1.x, (f32)t1.y, (f32)t1.z, 1, 0, 0, video::SColor(255,0,0,255), 0, 0);
        irrmesh->getVertexBuffer()[7+ig*9] = 
          video::S3DVertex((f32)t2.x, (f32)t2.y, (f32)t2.z, 1, 0, 0, video::SColor(255,0,0,255), 0, 0);
        irrmesh->getVertexBuffer()[8+ig*9] = 
          video::S3DVertex((f32)t2.x, (f32)t2.y, (f32)t2.z, 1, 0, 0, video::SColor(255,0,0,255), 0, 0);

        irrmesh->getIndexBuffer().setValue(0+itri*3, 6+ig*9);
        irrmesh->getIndexBuffer().setValue(1+itri*3, 7+ig*9);
        irrmesh->getIndexBuffer().setValue(2+itri*3, 8+ig*9);

        ++itri;
      }
    }

    irrmesh->setDirty(); // to force update of hardware buffers
    irrmesh->setHardwareMappingHint(EHM_DYNAMIC);//EHM_NEVER); //EHM_DYNAMIC for faster hw mapping
    irrmesh->recalculateBoundingBox();

    if (mglyphs->GetDrawMode() == chrono::ChGlyphs::GLYPH_VECTOR ||
        mglyphs->GetDrawMode() == chrono::ChGlyphs::GLYPH_COORDSYS ) {
      meshnode->setMaterialFlag(video::EMF_WIREFRAME,         true );
      meshnode->setMaterialFlag(video::EMF_LIGHTING,          false ); // avoid shading for wireframe
      meshnode->setMaterialFlag(video::EMF_BACK_FACE_CULLING, false );
    } else {
      meshnode->setMaterialFlag(video::EMF_WIREFRAME,         false );
      meshnode->setMaterialFlag(video::EMF_LIGHTING,          true );
      meshnode->setMaterialFlag(video::EMF_BACK_FACE_CULLING, true );
    }

    if (mglyphs->GetZbufferHide() == true)
      meshnode->setMaterialFlag(video::EMF_ZBUFFER,   true );
    else
      meshnode->setMaterialFlag(video::EMF_ZBUFFER,   false );

    meshnode->setMaterialFlag(video::EMF_COLOR_MATERIAL, true); // so color shading = vertexes  color

  }

}


} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


