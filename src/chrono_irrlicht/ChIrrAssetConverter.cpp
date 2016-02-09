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

#include "chrono/geometry/ChCSphere.h"
#include "chrono/geometry/ChCBox.h"
#include "chrono/geometry/ChCTriangleMeshSoup.h"
#include "chrono/geometry/ChCLinePath.h"

#include "chrono_irrlicht/ChIrrAssetConverter.h"
#include "chrono_irrlicht/ChIrrTools.h"
#include "chrono_irrlicht/ChIrrMeshTools.h"
#include "chrono_irrlicht/ChIrrAppInterface.h"

namespace chrono {
namespace irrlicht {

using namespace irr;
using namespace irr::scene;

ChIrrAssetConverter::ChIrrAssetConverter(ChIrrAppInterface& ainterface) {
    minterface = &ainterface;
    scenemanager = ainterface.GetSceneManager();
    mdevice = ainterface.GetDevice();

    sphereMesh = createEllipticalMesh(1.0, 1.0, -2, +2, 0, 15, 8);
    cubeMesh = createCubeMesh(core::vector3df(2, 2, 2));  // -/+ 1 unit each xyz axis
    cylinderMesh = createCylinderMesh(1, 1, 32);

    // if (sphereMesh)
    //  sphereMesh->grab();
    if (cubeMesh)
        cubeMesh->grab();
    if (cylinderMesh)
        cylinderMesh->grab();
}

ChIrrAssetConverter::~ChIrrAssetConverter() {
    if (sphereMesh)
        sphereMesh->drop();
    if (cubeMesh)
        cubeMesh->drop();
    if (cylinderMesh)
        cylinderMesh->drop();
}

ChSharedPtr<ChIrrNodeAsset> ChIrrAssetConverter::GetIrrNodeAsset(
    ChSharedPtr<ChPhysicsItem> mitem) {
    ChSharedPtr<ChIrrNodeAsset> myirrasset;  // default: IsNull() will return true
    std::vector<ChSharedPtr<ChAsset> > assetlist = mitem->GetAssets();

    for (unsigned int k = 0; k < assetlist.size(); k++) {
        ChSharedPtr<ChAsset> k_asset = assetlist[k];
        // asset k of object i references a proxy to an irrlicht node?
        if (k_asset.IsType<ChIrrNodeAsset>()) {
            myirrasset = k_asset.DynamicCastTo<ChIrrNodeAsset>();
        }
    }
    return myirrasset;
}

void ChIrrAssetConverter::Bind(ChSharedPtr<ChPhysicsItem> mitem) {
    // find a ChIrrNodeAsset if there is already one...
    ChSharedPtr<ChIrrNodeAsset> irrasset;
    irrasset = GetIrrNodeAsset(mitem);

    if (irrasset.IsNull()) {
        // add the ChIrrNodeAsset because it was not already there
        ChSharedPtr<ChIrrNodeAsset> mirr_assetpart(new ChIrrNodeAsset);
        mirr_assetpart->Bind(mitem, *minterface);
        mitem->AddAsset(mirr_assetpart);
    }
}

void ChIrrAssetConverter::BindAll() {
    ChSystem* msystem = minterface->GetSystem();

    ChSystem::IteratorBodies myiter = msystem->IterBeginBodies();
    while (myiter != msystem->IterEndBodies()) {
        Bind(*myiter);
        ++myiter;
    }
    ChSystem::IteratorOtherPhysicsItems myiterB = msystem->IterBeginOtherPhysicsItems();
    while (myiterB != msystem->IterEndOtherPhysicsItems()) {
        Bind(*myiterB);
        ++myiterB;
    }
    ChSystem::IteratorLinks myiterC = msystem->IterBeginLinks();
    while (myiterC != msystem->IterEndLinks()) {
        Bind(*myiterC);
        ++myiterC;
    }
}

void ChIrrAssetConverter::Update(ChSharedPtr<ChPhysicsItem> mitem) {
    CleanIrrlicht(mitem);
    PopulateIrrlicht(mitem);
}

void ChIrrAssetConverter::UpdateAll() {
    ChSystem* msystem = minterface->GetSystem();

    ChSystem::IteratorBodies myiter = msystem->IterBeginBodies();
    while (myiter != msystem->IterEndBodies()) {
        Update(*myiter);
        ++myiter;
    }
    ChSystem::IteratorOtherPhysicsItems myiterB = msystem->IterBeginOtherPhysicsItems();
    while (myiterB != msystem->IterEndOtherPhysicsItems()) {
        Update(*myiterB);
        ++myiterB;
    }
    ChSystem::IteratorLinks myiterC = msystem->IterBeginLinks();
    while (myiterC != msystem->IterEndLinks()) {
        Update(*myiterC);
        ++myiterC;
    }
}

void ChIrrAssetConverter::CleanIrrlicht(ChSharedPtr<ChPhysicsItem> mitem) {
    ChSharedPtr<ChIrrNodeAsset> irrasset;
    irrasset = GetIrrNodeAsset(mitem);

    if (!irrasset.IsNull()) {
        irrasset->GetIrrlichtNode()->removeAll();
    }
}

void ChIrrAssetConverter::PopulateIrrlicht(ChSharedPtr<ChPhysicsItem> mitem) {
    camera_found_in_assets = 0;
    mcamera = 0;
    std::vector<ChSharedPtr<ChAsset> > assetlist = mitem->GetAssets();
    ChSharedPtr<ChIrrNodeAsset> myirrasset;

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

    if (mitem->GetAssetsFrameNclones() > 0) {
        ISceneNode* clonecontainer = scenemanager->addEmptySceneNode(myirrasset->GetIrrlichtNode());
        fillnode = clonecontainer;
    }

    // 4- populate the ChIrrNode with conversions
    // of the geometric assets in this ChPhysicsItem

    // ISceneNode* mnode = myirrasset->GetIrrlichtNode(); TO REMOVE

    ChFrame<> bodyframe;  // begin with no rotation/translation respect to ChPhysicsItem (ex. a body)

    _recursePopulateIrrlicht(assetlist, bodyframe, fillnode);
}

void ChIrrAssetConverter::mflipSurfacesOnX(IMesh* mesh) const {
    if (!mesh)
        return;

    const u32 bcount = mesh->getMeshBufferCount();
    for (u32 b = 0; b < bcount; ++b) {
        IMeshBuffer* buffer = mesh->getMeshBuffer(b);
        const u32 idxcnt = buffer->getIndexCount();
        u16* idx = buffer->getIndices();
        s32 tmp;

        for (u32 i = 0; i < idxcnt; i += 3) {
            tmp = idx[i + 1];
            idx[i + 1] = idx[i + 2];
            idx[i + 2] = tmp;
        }
        const u32 vertcnt = buffer->getVertexCount();
        for (u32 i = 0; i < vertcnt; i++) {
            core::vector3df oldnorm = buffer->getNormal(i);
            buffer->getNormal(i).X = -oldnorm.X;  // mirrors normal on X
        }
    }
}

void ChIrrAssetConverter::_recursePopulateIrrlicht(std::vector<ChSharedPtr<ChAsset> >& assetlist,
                                                   ChFrame<> parentframe,
                                                   ISceneNode* mnode) {
    ChSharedPtr<ChTexture> mtexture;   // def no texture in level
    ChSharedPtr<ChColorAsset> mcolor;  // def no visualiz. settings in level

    // Scan assets in object and copy them as Irrlicht meshes in the ISceneNode
    for (unsigned int k = 0; k < assetlist.size(); k++) {
        ChSharedPtr<ChAsset> k_asset = assetlist[k];

        if (ChSharedPtr<ChVisualization> v_asset = k_asset.DynamicCastTo<ChVisualization>())
         if (v_asset->IsVisible()) {

            if (k_asset.IsType<ChObjShapeFile>()) {
                ChSharedPtr<ChObjShapeFile> myobj(k_asset.DynamicCastTo<ChObjShapeFile>());
                IAnimatedMesh* genericMesh = scenemanager->getMesh(myobj->GetFilename().c_str());
                if (genericMesh) {
                    ISceneNode* mproxynode = new ChIrrNodeProxyToAsset(myobj, mnode);
                    ISceneNode* mchildnode = scenemanager->addAnimatedMeshSceneNode(genericMesh, mproxynode);
                    mproxynode->drop();

                    // mchildnode->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, false);

                    //mchildnode->setScale(irr::core::vector3df(1, 1, 1));  // because of Irrlicht being left handed!!! NO-ACCEPT IRRLICHT IS MIRRORED
                    mchildnode->setMaterialFlag(video::EMF_BACK_FACE_CULLING,
                                                true);  // because of Irrlicht being left handed!!!
                    //mflipSurfacesOnX(((IAnimatedMeshSceneNode*)mchildnode)->getMesh());  // this wold be better than disabling back culling, but it does not work!
                }
            }
            if (k_asset.IsType<ChTriangleMeshShape>()) {
                ChSharedPtr<ChTriangleMeshShape> mytrimesh(
                    k_asset.DynamicCastTo<ChTriangleMeshShape>());

                CDynamicMeshBuffer* buffer = new CDynamicMeshBuffer(irr::video::EVT_STANDARD, irr::video::EIT_32BIT);
                //	SMeshBuffer* buffer = new SMeshBuffer();
                SMesh* newmesh = new SMesh;
                newmesh->addMeshBuffer(buffer);
                buffer->drop();

                ChIrrNodeProxyToAsset* mproxynode = new ChIrrNodeProxyToAsset(mytrimesh, mnode);
                ISceneNode* mchildnode = scenemanager->addMeshSceneNode(newmesh, mproxynode);
                newmesh->drop();
                mproxynode->Update();  // force syncing of triangle positions & face indexes
                mproxynode->drop();

                mchildnode->setMaterialFlag(video::EMF_WIREFRAME, mytrimesh->IsWireframe());
                mchildnode->setMaterialFlag(video::EMF_BACK_FACE_CULLING, mytrimesh->IsBackfaceCull());
            }
            if (k_asset.IsType<ChGlyphs>()) {
                ChSharedPtr<ChGlyphs> myglyphs(k_asset.DynamicCastTo<ChGlyphs>());

                CDynamicMeshBuffer* buffer = new CDynamicMeshBuffer(irr::video::EVT_STANDARD, irr::video::EIT_32BIT);
                //  SMeshBuffer* buffer = new SMeshBuffer();
                SMesh* newmesh = new SMesh;
                newmesh->addMeshBuffer(buffer);
                buffer->drop();

                ChIrrNodeProxyToAsset* mproxynode = new ChIrrNodeProxyToAsset(myglyphs, mnode);
                ISceneNode* mchildnode = scenemanager->addMeshSceneNode(newmesh, mproxynode);
                newmesh->drop();
                mproxynode->Update();  // force syncing of triangle positions & face indexes
                mproxynode->drop();

                // mchildnode->setMaterialFlag(video::EMF_WIREFRAME,  mytrimesh->IsWireframe() );
                // mchildnode->setMaterialFlag(video::EMF_BACK_FACE_CULLING, mytrimesh->IsBackfaceCull() );
            }
            if (k_asset.IsType<ChPathShape>() || k_asset.IsType<ChLineShape>()) {
                // ChSharedPtr<ChA> mypath(k_asset.DynamicCastTo<ChPathShape>());

                CDynamicMeshBuffer* buffer = new CDynamicMeshBuffer(irr::video::EVT_STANDARD, irr::video::EIT_32BIT);
                SMesh* newmesh = new SMesh;
                newmesh->addMeshBuffer(buffer);
                buffer->drop();

                ChIrrNodeProxyToAsset* mproxynode = new ChIrrNodeProxyToAsset(k_asset, mnode);
                ISceneNode* mchildnode = scenemanager->addMeshSceneNode(newmesh, mproxynode);
                newmesh->drop();
                mproxynode->Update();  // force syncing of triangle positions & face indexes
                mproxynode->drop();

                // mchildnode->setMaterialFlag(video::EMF_WIREFRAME,  mytrimesh->IsWireframe() );
                // mchildnode->setMaterialFlag(video::EMF_BACK_FACE_CULLING, mytrimesh->IsBackfaceCull() );
            }
            if (k_asset.IsType<ChSphereShape>() && sphereMesh) {
                ChSharedPtr<ChSphereShape> mysphere(k_asset.DynamicCastTo<ChSphereShape>());
                ISceneNode* mproxynode = new ChIrrNodeProxyToAsset(mysphere, mnode);
                ISceneNode* mchildnode = scenemanager->addMeshSceneNode(sphereMesh, mproxynode);
                mproxynode->drop();

                // Calculate transform from node to geometry
                // (concatenate node - asset and asset - geometry)
                ChVector<> pos = mysphere->Pos + mysphere->Rot * mysphere->GetSphereGeometry().center;
                ChCoordsys<> irrspherecoords(pos, mysphere->Rot.Get_A_quaternion());

                double mradius = mysphere->GetSphereGeometry().rad;
                mchildnode->setScale(core::vector3dfCH(ChVector<>(mradius, mradius, mradius)));
                ChIrrTools::alignIrrlichtNodeToChronoCsys(mchildnode, irrspherecoords);
                mchildnode->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
            }
            if (k_asset.IsType<ChCylinderShape>() && cylinderMesh) {
                ChSharedPtr<ChCylinderShape> mycylinder(k_asset.DynamicCastTo<ChCylinderShape>());
                ISceneNode* mproxynode = new ChIrrNodeProxyToAsset(mycylinder, mnode);
                ISceneNode* mchildnode = scenemanager->addMeshSceneNode(cylinderMesh, mproxynode);
                mproxynode->drop();

                double rad = mycylinder->GetCylinderGeometry().rad;
                ChVector<> dir = mycylinder->GetCylinderGeometry().p2 - mycylinder->GetCylinderGeometry().p1;
                double height = dir.Length();

                // Calculate transform from asset to geometry
                dir.Normalize();
                ChVector<> mx, my, mz;
                dir.DirToDxDyDz(my, mz, mx);  // y is axis, in cylinder.obj frame
                ChMatrix33<> mrot;
                mrot.Set_A_axis(mx, my, mz);
                ChVector<> mpos =
                    0.5 * (mycylinder->GetCylinderGeometry().p2 + mycylinder->GetCylinderGeometry().p1);

                // Calculate transform from node to geometry
                // (concatenate node - asset and asset - geometry)
                ChVector<> pos = mycylinder->Pos + mycylinder->Rot * mpos;
                ChMatrix33<> rot = mycylinder->Rot * mrot;
                ChCoordsys<> irrcylindercoords(pos, rot.Get_A_quaternion());

                ChIrrTools::alignIrrlichtNodeToChronoCsys(mchildnode, irrcylindercoords);
                core::vector3df irrsize((f32)rad, (f32)(0.5 * height), (f32)rad);
                mchildnode->setScale(irrsize);
                mchildnode->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
            }
            if (k_asset.IsType<ChBoxShape>() && cubeMesh) {
                ChSharedPtr<ChBoxShape> mybox(k_asset.DynamicCastTo<ChBoxShape>());
                ISceneNode* mproxynode = new ChIrrNodeProxyToAsset(mybox, mnode);
                ISceneNode* mchildnode = scenemanager->addMeshSceneNode(cubeMesh, mproxynode);
                mproxynode->drop();

                // Calculate transform from node to geometry
                // (concatenate node - asset and asset - geometry)
                ChVector<> pos = mybox->Pos + mybox->Rot * mybox->GetBoxGeometry().Pos;
                ChMatrix33<> rot = mybox->Rot * mybox->GetBoxGeometry().Rot;
                ChCoordsys<> irrboxcoords(pos, rot.Get_A_quaternion());

                mchildnode->setScale(core::vector3dfCH(mybox->GetBoxGeometry().Size));
                ChIrrTools::alignIrrlichtNodeToChronoCsys(mchildnode, irrboxcoords);
                mchildnode->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
            }
            if (k_asset.IsType<ChCamera>()) {
                camera_found_in_assets = true;
                ChSharedPtr<ChCamera> mycamera(k_asset.DynamicCastTo<ChCamera>());
                ISceneNode* mproxynode = new ChIrrNodeProxyToAsset(mycamera, mnode);
                RTSCamera* irrcamera =
                    new RTSCamera(mdevice, mproxynode, scenemanager, -1, -160.0f, 1.0f, 0.003f);
                mproxynode->drop();

                irrcamera->setPosition(core::vector3dfCH(mycamera->GetPosition()));
                irrcamera->setTarget(core::vector3dfCH(mycamera->GetAimPoint()));
                double fov_rad = mycamera->GetAngle() * CH_C_DEG_TO_RAD;
                irrcamera->setFOV((irr::f32)fov_rad);
                irrcamera->setNearValue(0.3f);
                irrcamera->setMinZoom(0.6f);
            }

        } // end if visible asset

        if (k_asset.IsType<ChAssetLevel>()) {
            ChSharedPtr<ChAssetLevel> mylevel(k_asset.DynamicCastTo<ChAssetLevel>());
            std::vector<ChSharedPtr<ChAsset> >& subassetlist = mylevel->GetAssets();
            ChFrame<> subassetframe = mylevel->GetFrame();
            ISceneNode* mproxynode = new ChIrrNodeProxyToAsset(mylevel, mnode);
            ISceneNode* subassetnode = scenemanager->addEmptySceneNode(mproxynode);
            mproxynode->drop();
            // recurse level...
            _recursePopulateIrrlicht(subassetlist, subassetframe, subassetnode);
        }

        if (k_asset.IsType<ChTexture>()) {
            mtexture = k_asset.DynamicCastTo<ChTexture>();
        }
        if (k_asset.IsType<ChColorAsset>()) {
            mcolor = k_asset.DynamicCastTo<ChColorAsset>();
        }

    }  // end loop on assets

    // if a texture has been found, apply it to all nodes of this level
    if (!mtexture.IsNull()) {
        video::ITexture* mtextureMap = mdevice->getVideoDriver()->getTexture(mtexture->GetTextureFilename().c_str());
        ISceneNodeList::ConstIterator it = mnode->getChildren().begin();
        for (; it != mnode->getChildren().end(); ++it) {
            ISceneNode* mproxynode = (*it);                               // the ChIrrNodeProxyToAsset contains..
            ISceneNode* meshnode = *(mproxynode->getChildren().begin());  // ..one child ISceneNode with a mesh
            if (meshnode) {
                meshnode->setMaterialTexture(0, mtextureMap);
                meshnode->getMaterial(0).getTextureMatrix(0).setTextureScale(mtexture->GetTextureScaleX(), mtexture->GetTextureScaleY());
            }
        }
    }

    // if a visualization setting (color) has been set, apply it to all nodes of this level
    if (!mcolor.IsNull()) {
        ISceneNodeList::ConstIterator it = mnode->getChildren().begin();
        for (; it != mnode->getChildren().end(); ++it) {
            ISceneNode* mproxynode = (*it);                               // the ChIrrNodeProxyToAsset contains..
            ISceneNode* meshnode = *(mproxynode->getChildren().begin());  // ..one child ISceneNode with a mesh
            if (meshnode)
                for (unsigned int im = 0; im < meshnode->getMaterialCount(); ++im) {
                    meshnode->getMaterial(im).ColorMaterial = irr::video::ECM_NONE;
                    meshnode->getMaterial(im).DiffuseColor.set(
                        (irr::u32)(255 * mcolor->GetColor().A), (irr::u32)(255 * mcolor->GetColor().R),
                        (irr::u32)(255 * mcolor->GetColor().G), (irr::u32)(255 * mcolor->GetColor().B));
                }
        }
    }

    // if a visualization setting (color) has been set, force the color attribute of all assets in same level
    if (!mcolor.IsNull()) {
        for (unsigned int k = 0; k < assetlist.size(); k++) {
            if (ChSharedPtr<ChVisualization> k_visasset = assetlist[k].DynamicCastTo<ChVisualization>()) {
                k_visasset->SetColor(mcolor->GetColor());
            }
        }
    }

    // Set the rotation and position of the node container
    if (!(parentframe.GetCoord() == CSYSNORM)) {
        ChIrrTools::alignIrrlichtNodeToChronoCsys(mnode, parentframe.GetCoord());
    }
}

}  // end namespace irrlicht
}  // end namespace chrono
