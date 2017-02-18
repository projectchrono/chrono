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

#include "chrono_irrlicht/ChBodySceneNode.h"

namespace chrono {
namespace irrlicht {

using namespace irr;
using namespace irr::scene;

// Initialize static variables
int ChBodySceneNode::body_identifier = 0;

// Constructors
ChBodySceneNode::ChBodySceneNode(ChSystem* msystem, IAnimatedMesh* mesh, ISceneNode* parent, ISceneManager* mgr, s32 id)
    : scene::ISceneNode(parent, mgr, id), ChronoControlled(true) {
    assert(msystem);

#ifdef _DEBUG
    setDebugName("ChBodySceneNode");
#endif

    child_mesh = 0;

    if (mesh)
        child_mesh = mgr->addAnimatedMeshSceneNode(mesh, this);

    if (child_mesh)
        child_mesh->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);

    // Create the shared pointer, and the ChBody object pointed by the shared
    // pointer. Creating dynamically the shared pointer from heap is not nice
    // to see, but it must be managed dynamically in this wrapper node.

    bodyp = new std::shared_ptr<ChBody>(new ChBody);

    // set an unique identifier
    body_identifier++;
    GetBody()->SetIdentifier(body_identifier);

    // Automatically add to the Chrono::Engine system.
    msystem->AddBody(GetBody());
}

ChBodySceneNode::ChBodySceneNode(ChSystem* msystem,
                                 IAnimatedMesh* mesh,
                                 ISceneNode* parent,
                                 ISceneManager* mgr,
                                 s32 id,
                                 const ChVector<>& offset)
    : scene::ISceneNode(parent, mgr, id), ChronoControlled(true) {
    assert(msystem);

#ifdef _DEBUG
    setDebugName("ChBodySceneNode");
#endif

    child_mesh = 0;

    if (mesh)
        child_mesh = mgr->addAnimatedMeshSceneNode(mesh, this, -1,
                                                   core::vector3df(-(f32)offset.x(), -(f32)offset.y(), -(f32)offset.z()));

    if (child_mesh)
        child_mesh->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);

    bodyp = new std::shared_ptr<ChBody>(new ChBody);

    body_identifier++;
    GetBody()->SetIdentifier(body_identifier);

    msystem->AddBody(GetBody());
}

/// Destructor.
ChBodySceneNode::~ChBodySceneNode() {
    // Automatically remove from the Chrono::Engine system, if currently inserted
    // in a system.
    if (GetBody()->GetSystem())
        GetBody()->GetSystem()->RemoveBody(GetBody());

    // Deleting the shared pointer will also automatically delete the pointed
    // ChBody, if needed (ie. if none else is referencing it).
    delete bodyp;
    bodyp = 0;
}

void ChBodySceneNode::OnRegisterSceneNode() {
    if (IsVisible)
        SceneManager->registerNodeForRendering(this);

    ISceneNode::OnRegisterSceneNode();
}

void ChBodySceneNode::render() {
    if (child_mesh)
        child_mesh->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
}

const core::aabbox3d<f32>& ChBodySceneNode::getBoundingBox() const {
    if (child_mesh)
        return child_mesh->getBoundingBox();
    else
        return Box;
}

void ChBodySceneNode::setMaterialTexture(s32 textureLayer, video::ITexture* texture) {
    if (child_mesh)
        return child_mesh->setMaterialTexture(textureLayer, texture);
}

u32 ChBodySceneNode::getMaterialCount() {
    if (child_mesh)
        return child_mesh->getMaterialCount();
    else
        return 0;
}

video::SMaterial& ChBodySceneNode::getMaterial(u32 i) {
    assert(child_mesh);
    return child_mesh->getMaterial(i);
}

void ChBodySceneNode::OnAnimate(u32 timeMs) {
    if (child_mesh)
        child_mesh->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);

    if (IsVisible) {
        // reorient/reposition the scene node every frame
        if (bodyp && ChronoControlled) {
            // Output: will be an Irrlicht 4x4 matrix
            core::matrix4 irrMat;

            // Get the rigid body actual rotation, as a 3x3 matrix [A]
            const ChMatrix33<>& chMat = GetBody()->GetFrame_REF_to_abs().GetA();

            // Fill the upper 3x3 submatrix with the [A] matrix transposed, since
            // Irrlicht uses the row-major style as in D3D
            irrMat[0] = (irr::f32)chMat.GetElementN(0);
            irrMat[1] = (irr::f32)chMat.GetElementN(3);
            irrMat[2] = (irr::f32)chMat.GetElementN(6);

            irrMat[4] = (irr::f32)chMat.GetElementN(1);
            irrMat[5] = (irr::f32)chMat.GetElementN(4);
            irrMat[6] = (irr::f32)chMat.GetElementN(7);

            irrMat[8] = (irr::f32)chMat.GetElementN(2);
            irrMat[9] = (irr::f32)chMat.GetElementN(5);
            irrMat[10] = (irr::f32)chMat.GetElementN(8);

            irrMat[12] = (irr::f32)GetBody()->GetFrame_REF_to_abs().GetPos().x();
            irrMat[13] = (irr::f32)GetBody()->GetFrame_REF_to_abs().GetPos().y();
            irrMat[14] = (irr::f32)GetBody()->GetFrame_REF_to_abs().GetPos().z();

            // Clear the last column to 0 and set low-right corner to 1 as in
            // Denavitt-Hartemberg matrices, transposed.
            irrMat[3] = irrMat[7] = irrMat[11] = 0.0f;
            irrMat[15] = 1.0f;

            // Set position and rotation of node using the 4x4 Irrlicht matrix.
            setPosition(irrMat.getTranslation());
            setRotation(irrMat.getRotationDegrees());
        }
    }

    ISceneNode::OnAnimate(timeMs);
}

IShadowVolumeSceneNode* ChBodySceneNode::addShadowVolumeSceneNode(const IMesh* shadowMesh,
                                                                  s32 id,
                                                                  bool zfailmethod,
                                                                  f32 infinity) {
    if (child_mesh)
        return child_mesh->addShadowVolumeSceneNode(shadowMesh, id, zfailmethod, infinity);

    return NULL;
};

}  // end namespace irrlicht
}  // end namespace chrono
