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


#include <vector>

#include "geometry/ChCSphere.h"
#include "geometry/ChCBox.h"
#include "geometry/ChCTriangleMeshSoup.h"

#include "unit_IRRLICHT/ChIrrAssetConverter.h"
#include "unit_IRRLICHT/ChIrrTools.h"
#include "unit_IRRLICHT/ChIrrMeshTools.h"
#include "unit_IRRLICHT/ChIrrAppInterface.h"


namespace irr {
namespace scene {


ChIrrAssetConverter::ChIrrAssetConverter(ChIrrAppInterface& ainterface)
{
  minterface   = &ainterface;
  scenemanager = ainterface.GetSceneManager();
  mdevice      = ainterface.GetDevice();

  sphereMesh   = createEllipticalMesh(1.0,1.0,-2,+2,0, 15, 8);
  cubeMesh     = createCubeMesh(core::vector3df(2,2,2)); // -/+ 1 unit each xyz axis  
  cylinderMesh = createCylinderMesh(1,1,32); 

  //if (sphereMesh)
  //  sphereMesh->grab();
  if (cubeMesh)
    cubeMesh->grab();
  if (cylinderMesh)
    cylinderMesh->grab();
}


ChIrrAssetConverter::~ChIrrAssetConverter()
{
  if (sphereMesh)
    sphereMesh->drop();
  if (cubeMesh)
    cubeMesh->drop();
  if (cylinderMesh)
    cylinderMesh->drop();
}


chrono::ChSharedPtr<chrono::ChIrrNodeAsset> 
ChIrrAssetConverter::GetIrrNodeAsset(chrono::ChSharedPtr<chrono::ChPhysicsItem> mitem)
{
  chrono::ChSharedPtr<chrono::ChIrrNodeAsset> myirrasset; // default: IsNull() will return true
  std::vector< chrono::ChSharedPtr<chrono::ChAsset> > assetlist = mitem->GetAssets();

  for (unsigned int k = 0; k < assetlist.size(); k++)
  {
    chrono::ChSharedPtr<chrono::ChAsset> k_asset = assetlist[k];
    // asset k of object i references a proxy to an irrlicht node?
    if ( k_asset.IsType<chrono::ChIrrNodeAsset>() )
    {
      myirrasset = k_asset.DynamicCastTo<chrono::ChIrrNodeAsset>();
    }
  }
  return myirrasset;
}


void ChIrrAssetConverter::Bind(chrono::ChSharedPtr<chrono::ChPhysicsItem> mitem)
{
  // find a ChIrrNodeAsset if there is already one...
  chrono::ChSharedPtr<chrono::ChIrrNodeAsset> irrasset;
  irrasset = GetIrrNodeAsset(mitem);

  if (irrasset.IsNull())
  {
    // add the ChIrrNodeAsset because it was not already there
    chrono::ChSharedPtr<chrono::ChIrrNodeAsset> mirr_assetpart(new chrono::ChIrrNodeAsset);
    mirr_assetpart->Bind(mitem, *minterface);
    mitem->AddAsset(mirr_assetpart);
  }
}


void ChIrrAssetConverter::BindAll()
{
  chrono::ChSystem* msystem = minterface->GetSystem();

  chrono::ChSystem::IteratorBodies myiter = msystem->IterBeginBodies();
  while (myiter != msystem->IterEndBodies())
  {
    Bind(*myiter);
    ++myiter;
  }
  chrono::ChSystem::IteratorOtherPhysicsItems myiterB = msystem->IterBeginOtherPhysicsItems();
  while (myiterB != msystem->IterEndOtherPhysicsItems())
  {
    Bind(*myiterB);
    ++myiterB;
  }
  chrono::ChSystem::IteratorLinks myiterC = msystem->IterBeginLinks();
  while (myiterC != msystem->IterEndLinks())
  {
    Bind(*myiterC);
    ++myiterC;
  }
}


void ChIrrAssetConverter::Update(chrono::ChSharedPtr<chrono::ChPhysicsItem> mitem)
{
  CleanIrrlicht(mitem);
  PopulateIrrlicht(mitem);
}


void ChIrrAssetConverter::UpdateAll()
{
  chrono::ChSystem* msystem = minterface->GetSystem();

  chrono::ChSystem::IteratorBodies myiter = msystem->IterBeginBodies();
  while (myiter != msystem->IterEndBodies())
  {
    Update(*myiter);
    ++myiter;
  }
  chrono::ChSystem::IteratorOtherPhysicsItems myiterB = msystem->IterBeginOtherPhysicsItems();
  while (myiterB != msystem->IterEndOtherPhysicsItems())
  {
    Update(*myiterB);
    ++myiterB;
  }
  chrono::ChSystem::IteratorLinks myiterC = msystem->IterBeginLinks();
  while (myiterC != msystem->IterEndLinks())
  {
    Update(*myiterC);
    ++myiterC;
  }
}


void ChIrrAssetConverter::CleanIrrlicht(chrono::ChSharedPtr<chrono::ChPhysicsItem> mitem)
{
  chrono::ChSharedPtr<chrono::ChIrrNodeAsset> irrasset;
  irrasset = GetIrrNodeAsset(mitem);

  if (!irrasset.IsNull())
  {
    irrasset->GetIrrlichtNode()->removeAll();
  }
}


void ChIrrAssetConverter::PopulateIrrlicht(chrono::ChSharedPtr<chrono::ChPhysicsItem> mitem)
{
  camera_found_in_assets = 0;
  mcamera = 0;
  std::vector< chrono::ChSharedPtr<chrono::ChAsset> > assetlist = mitem->GetAssets();
  chrono::ChSharedPtr<chrono::ChIrrNodeAsset> myirrasset;

  // 1- Clean the ChIrrNode
  CleanIrrlicht(mitem);

  // 2- Find the ChIrrNodeAsset proxy
  myirrasset = GetIrrNodeAsset(mitem);

  if (myirrasset.IsNull())
    return;

  // 3- If shapes must be 'clones', put all them inside an intermediate level
  // (that will be cloned in ChIrrNode::OnAnimate), if necessary. Otherwise put shapes
  // normally inside the ChIrrNode

  ISceneNode* fillnode = myirrasset->GetIrrlichtNode();

  if (!fillnode)
    return;

  if(mitem->GetAssetsFrameNclones()>0)
  {
    ISceneNode* clonecontainer = scenemanager->addEmptySceneNode(myirrasset->GetIrrlichtNode());
    fillnode = clonecontainer;
  }

  // 4- populate the ChIrrNode with conversions 
  // of the geometric assets in this ChPhysicsItem

  // ISceneNode* mnode = myirrasset->GetIrrlichtNode(); TO REMOVE

  chrono::ChFrame<> bodyframe; // begin with no rotation/translation respect to ChPhysicsItem (ex. a body)

  _recursePopulateIrrlicht(assetlist, bodyframe, fillnode); 
}


void ChIrrAssetConverter::mflipSurfacesOnX(scene::IMesh* mesh) const
{
  if (!mesh)
    return;

  const u32 bcount = mesh->getMeshBufferCount();
  for (u32 b=0; b<bcount; ++b)
  {
    IMeshBuffer* buffer = mesh->getMeshBuffer(b);
    const u32 idxcnt = buffer->getIndexCount();
    u16* idx = buffer->getIndices();
    s32 tmp;

    for (u32 i=0; i<idxcnt; i+=3)
    {
      tmp = idx[i+1];
      idx[i+1] = idx[i+2];
      idx[i+2] = tmp;
    }
    const u32 vertcnt = buffer->getVertexCount();
    for (u32 i=0; i<vertcnt; i++)
    {
      core::vector3df oldnorm = buffer->getNormal(i);
      buffer->getNormal(i).X = -oldnorm.X; //mirrors normal on X 
    }
  }
}


void ChIrrAssetConverter::_recursePopulateIrrlicht(
            std::vector< chrono::ChSharedPtr<chrono::ChAsset> >&  assetlist,
            chrono::ChFrame<>                                     parentframe,
            ISceneNode*                                           mnode)
{
  chrono::ChSharedPtr<chrono::ChTexture> mtexture; // def no texture in level
  chrono::ChSharedPtr<chrono::ChColorAsset> mcolor; // def no visualiz. settings in level

  // Scan assets in object and copy them as Irrlicht meshes in the ISceneNode
  for (unsigned int k = 0; k < assetlist.size(); k++)
  {
    chrono::ChSharedPtr<chrono::ChAsset> k_asset = assetlist[k];

    if ( k_asset.IsType<chrono::ChObjShapeFile>() )
    {
      chrono::ChSharedPtr<chrono::ChObjShapeFile> myobj(k_asset.DynamicCastTo<chrono::ChObjShapeFile>());
      IAnimatedMesh* genericMesh = scenemanager->getMesh(myobj->GetFilename().c_str());
      if (genericMesh)
      {
        ISceneNode* mproxynode = new ChIrrNodeProxyToAsset(myobj, mnode);
        ISceneNode* mchildnode = scenemanager->addAnimatedMeshSceneNode(genericMesh,mproxynode);

        //mchildnode->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, false);

        mchildnode->setScale(irr::core::vector3df(-1,1,1)); // because of Irrlicht being left handed!!!
        mchildnode->setMaterialFlag(video::EMF_BACK_FACE_CULLING, true); // because of Irrlicht being left handed!!!
        mflipSurfacesOnX(((IAnimatedMeshSceneNode*)mchildnode)->getMesh()); // this wold be better than disabling back culling, but it does not work!
      }
    }
    if ( k_asset.IsType<chrono::ChTriangleMeshShape>() )
    {
      chrono::ChSharedPtr<chrono::ChTriangleMeshShape> mytrimesh(k_asset.DynamicCastTo<chrono::ChTriangleMeshShape>());

      CDynamicMeshBuffer * buffer = new CDynamicMeshBuffer(irr::video::EVT_STANDARD, irr::video::EIT_32BIT);
      //	SMeshBuffer* buffer = new SMeshBuffer();
      SMesh* newmesh = new SMesh;
      newmesh->addMeshBuffer(buffer);
      buffer->drop();

      ChIrrNodeProxyToAsset* mproxynode = new ChIrrNodeProxyToAsset(mytrimesh, mnode);
      ISceneNode* mchildnode = scenemanager->addMeshSceneNode(newmesh,mproxynode);
      newmesh->drop();
      mproxynode->Update(); // force syncing of triangle positions & face indexes

      mchildnode->setMaterialFlag(video::EMF_WIREFRAME, mytrimesh->IsWireframe() ); 
      mchildnode->setMaterialFlag(video::EMF_BACK_FACE_CULLING, mytrimesh->IsBackfaceCull() );
    }
    if ( k_asset.IsType<chrono::ChGlyphs>() )
    {
      chrono::ChSharedPtr<chrono::ChGlyphs> myglyphs(k_asset.DynamicCastTo<chrono::ChGlyphs>());

      CDynamicMeshBuffer * buffer = new CDynamicMeshBuffer(irr::video::EVT_STANDARD, irr::video::EIT_32BIT);
      //  SMeshBuffer* buffer = new SMeshBuffer();
      SMesh* newmesh = new SMesh;
      newmesh->addMeshBuffer(buffer);
      buffer->drop();

      ChIrrNodeProxyToAsset* mproxynode = new ChIrrNodeProxyToAsset(myglyphs, mnode);
      ISceneNode* mchildnode = scenemanager->addMeshSceneNode(newmesh,mproxynode);
      newmesh->drop();
      mproxynode->Update(); // force syncing of triangle positions & face indexes

      //mchildnode->setMaterialFlag(video::EMF_WIREFRAME,  mytrimesh->IsWireframe() ); 
      //mchildnode->setMaterialFlag(video::EMF_BACK_FACE_CULLING, mytrimesh->IsBackfaceCull() );
    }
    if ( k_asset.IsType<chrono::ChSphereShape>() && sphereMesh)
    {
      chrono::ChSharedPtr<chrono::ChSphereShape> mysphere(k_asset.DynamicCastTo<chrono::ChSphereShape>());
      ISceneNode* mproxynode = new ChIrrNodeProxyToAsset(mysphere, mnode);
      ISceneNode* mchildnode = scenemanager->addMeshSceneNode(sphereMesh,mproxynode);

      // Calculate transform from node to geometry
      // (concatenate node - asset and asset - geometry)
      chrono::ChVector<> pos = mysphere->Pos + mysphere->Rot * mysphere->GetSphereGeometry().center;
      chrono::ChCoordsys<> irrspherecoords(pos, mysphere->Rot.Get_A_quaternion());

      double mradius = mysphere->GetSphereGeometry().rad;
      mchildnode->setScale(core::vector3dfCH(chrono::ChVector<>(mradius,mradius,mradius)));
      ChIrrTools::alignIrrlichtNodeToChronoCsys(mchildnode, irrspherecoords);
      mchildnode->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
    }
    if ( k_asset.IsType<chrono::ChCylinderShape>() && cylinderMesh)
    {
      chrono::ChSharedPtr<chrono::ChCylinderShape> mycylinder(k_asset.DynamicCastTo<chrono::ChCylinderShape>());
      ISceneNode* mproxynode = new ChIrrNodeProxyToAsset(mycylinder, mnode);
      ISceneNode* mchildnode = scenemanager->addMeshSceneNode(cylinderMesh,mproxynode);

      double rad = mycylinder->GetCylinderGeometry().rad;
      chrono::ChVector<> dir = mycylinder->GetCylinderGeometry().p2 - mycylinder->GetCylinderGeometry().p1;
      double height = dir.Length();

      // Calculate transform from asset to geometry
      dir.Normalize();
      chrono::ChVector<> mx, my, mz;
      dir.DirToDxDyDz(my,mz,mx); // y is axis, in cylinder.obj frame
      chrono::ChMatrix33<> mrot;
      mrot.Set_A_axis(mx,my,mz);
      chrono::ChVector<> mpos = 0.5 * (mycylinder->GetCylinderGeometry().p2 + mycylinder->GetCylinderGeometry().p1);

      // Calculate transform from node to geometry
      // (concatenate node - asset and asset - geometry)
      chrono::ChVector<> pos = mycylinder->Pos + mycylinder->Rot * mpos;
      chrono::ChMatrix33<> rot = mycylinder->Rot * mrot;
      chrono::ChCoordsys<> irrcylindercoords(pos, rot.Get_A_quaternion());

      ChIrrTools::alignIrrlichtNodeToChronoCsys(mchildnode, irrcylindercoords);
      core::vector3df irrsize((f32)rad, (f32)(0.5*height), (f32)rad);
      mchildnode->setScale(irrsize);
      mchildnode->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
    }
    if ( k_asset.IsType<chrono::ChBoxShape>() && cubeMesh)
    {
      chrono::ChSharedPtr<chrono::ChBoxShape> mybox(k_asset.DynamicCastTo<chrono::ChBoxShape>());
      ISceneNode* mproxynode = new ChIrrNodeProxyToAsset(mybox, mnode);
      ISceneNode* mchildnode = scenemanager->addMeshSceneNode(cubeMesh,mproxynode);

      // Calculate transform from node to geometry
      // (concatenate node - asset and asset - geometry)
      chrono::ChVector<> pos = mybox->Pos + mybox->Rot * mybox->GetBoxGeometry().Pos;
      chrono::ChMatrix33<> rot = mybox->Rot * mybox->GetBoxGeometry().Rot;
      chrono::ChCoordsys<> irrboxcoords(pos, rot.Get_A_quaternion());

      mchildnode->setScale(core::vector3dfCH(mybox->GetBoxGeometry().Size));
      ChIrrTools::alignIrrlichtNodeToChronoCsys(mchildnode, irrboxcoords);
      mchildnode->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
    }

    if ( k_asset.IsType<chrono::ChTexture>() )
    {
      mtexture = k_asset.DynamicCastTo<chrono::ChTexture>();
    }
    if ( k_asset.IsType<chrono::ChColorAsset>() )
    {
      mcolor = k_asset.DynamicCastTo<chrono::ChColorAsset>();
    }

    if ( k_asset.IsType<chrono::ChCamera>() )
    {
      camera_found_in_assets = true;
      chrono::ChSharedPtr<chrono::ChCamera> mycamera(k_asset.DynamicCastTo<chrono::ChCamera>());
      ISceneNode* mproxynode = new ChIrrNodeProxyToAsset(mycamera, mnode);
      scene::RTSCamera* irrcamera = new scene::RTSCamera(mdevice, mproxynode, scenemanager,-1, -160.0f, 1.0f, 0.003f); 

      irrcamera->setPosition(core::vector3dfCH(mycamera->GetPosition()));
      irrcamera->setTarget(core::vector3dfCH(mycamera->GetAimPoint()));
      double fov_rad = mycamera->GetAngle() * chrono::CH_C_DEG_TO_RAD;
      irrcamera->setFOV((irr::f32)fov_rad); 
      irrcamera->setNearValue(0.3f);
      irrcamera->setMinZoom(0.6f);
    }
    if ( k_asset.IsType<chrono::ChAssetLevel>() )
    {
      chrono::ChSharedPtr<chrono::ChAssetLevel> mylevel(k_asset.DynamicCastTo<chrono::ChAssetLevel>());
      std::vector< chrono::ChSharedPtr<chrono::ChAsset> >& subassetlist = mylevel->GetAssets();
      chrono::ChFrame<> subassetframe = mylevel->GetFrame();
      ISceneNode* mproxynode = new ChIrrNodeProxyToAsset(mylevel, mnode);
      ISceneNode* subassetnode = scenemanager->addEmptySceneNode(mproxynode);
      // recurse level...
      _recursePopulateIrrlicht(subassetlist, subassetframe, subassetnode);
    }

  } // end loop on assets


  // if a texture has been found, apply it to all nodes of this level
  if (!mtexture.IsNull())
  {
    video::ITexture* mtextureMap = mdevice->getVideoDriver()->getTexture(mtexture->GetTextureFilename().c_str());
    ISceneNodeList::ConstIterator it = mnode->getChildren().begin();
    for (; it != mnode->getChildren().end(); ++it)
    {
      ISceneNode* mproxynode = (*it); // the ChIrrNodeProxyToAsset contains..
      ISceneNode* meshnode = *(mproxynode->getChildren().begin()); // ..one child ISceneNode with a mesh
      if (meshnode) 
        meshnode->setMaterialTexture(0,mtextureMap);
    }
  }

  // if a visualization setting (color) has been set, apply it to all nodes of this level
  if (!mcolor.IsNull())
  {
    ISceneNodeList::ConstIterator it = mnode->getChildren().begin();
    for (; it != mnode->getChildren().end(); ++it)
    {
      ISceneNode* mproxynode = (*it); // the ChIrrNodeProxyToAsset contains..
      ISceneNode* meshnode = *(mproxynode->getChildren().begin()); // ..one child ISceneNode with a mesh
      if (meshnode) 
        for (unsigned int im =0; im < meshnode->getMaterialCount(); ++im)
        {
          meshnode->getMaterial(im).ColorMaterial = irr::video::ECM_NONE;
          meshnode->getMaterial(im).DiffuseColor.set((irr::u32)(255*mcolor->GetColor().A), 
            (irr::u32)(255*mcolor->GetColor().R),
            (irr::u32)(255*mcolor->GetColor().G), 
            (irr::u32)(255*mcolor->GetColor().B));
        }
    }
  }

  // Set the rotation and position of the node container
  if (!(parentframe.GetCoord() == chrono::CSYSNORM))
  {
    ChIrrTools::alignIrrlichtNodeToChronoCsys(mnode, parentframe.GetCoord());
  }
}


} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

