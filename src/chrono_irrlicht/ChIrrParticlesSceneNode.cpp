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

#include "chrono_irrlicht/ChIrrParticlesSceneNode.h"

namespace chrono {
namespace irrlicht {

using namespace irr;

// Initialize static variables
int ChIrrParticlesSceneNode::particles_identifier = 0;

// Constructor
ChIrrParticlesSceneNode::ChIrrParticlesSceneNode(ChSystem* msystem,
                                                 scene::IAnimatedMesh* mesh,
                                                 irr::core::vector3df mmesh_scale,
                                                 scene::ISceneNode* parent,
                                                 scene::ISceneManager* mgr,
                                                 s32 id)
    : scene::ISceneNode(parent, mgr, id), ChronoControlled(true) {
    assert(msystem);

#ifdef _DEBUG
    setDebugName("ChIrrParticlesSceneNode");
#endif

    child_mesh = 0;

    sample_mesh = mesh;
    sample_mesh->grab();

    mesh_scale = mmesh_scale;

    /*
    if (mesh)
    {
    for (int j = 0; j<num; j++)
    {
    child_mesh = mgr->addAnimatedMeshSceneNode(sample_mesh, this);
    child_mesh->setScale(mesh_scale);

    if (child_mesh)
    child_mesh->setMaterialFlag(irr::video::EMF_NORMALIZE_NORMALS, true);
    }
    }

    Nchildren = num;
    */

    Nchildren = 0;

    // Create the shared pointer, and the ChParticlesClones object
    // pointed by the shared pointer.
    // Creating dynamically the shared pointer from heap is not
    // nice to see, but it must be managed dynamically in this wrapper node..

    particlep = new std::shared_ptr<ChParticlesClones>(new ChParticlesClones);

    // set an unique identifier
    particles_identifier++;
    GetParticles()->SetIdentifier(particles_identifier);

    // Automatically add to the Chrono::Engine system.
    msystem->Add(GetParticles());

    sample_texture = 0;
}

// Destructor
ChIrrParticlesSceneNode::~ChIrrParticlesSceneNode() {
    // Automatically remove from the Chrono::Engine system, if currently inserted
    // in a system.
    if (GetParticles()->GetSystem()) {
        GetParticles()->GetSystem()->Remove(GetParticles());
    }

    // Deleting the shared pointer will automatically delete also the pointed
    // ChParticlesClones, if needed (ie. if none else is referencing it).
    delete particlep;
    particlep = 0;

    if (sample_mesh)
        sample_mesh->drop();
}

void ChIrrParticlesSceneNode::OnRegisterSceneNode() {
    if (IsVisible)
        SceneManager->registerNodeForRendering(this);

    ISceneNode::OnRegisterSceneNode();
}

void ChIrrParticlesSceneNode::render() {
    // if (child_mesh)
    //	child_mesh->setMaterialFlag(irr::video::EMF_NORMALIZE_NORMALS, true);
}

const irr::core::aabbox3d<f32>& ChIrrParticlesSceneNode::getBoundingBox() const {
    if (child_mesh)
        return child_mesh->getBoundingBox();
    else
        return Box;
}

void ChIrrParticlesSceneNode::setMaterialTexture(s32 textureLayer, irr::video::ITexture* texture) {
    sample_texture = texture;
    // if (child_mesh)
    //	return child_mesh->setMaterialTexture (textureLayer, texture);
}

u32 ChIrrParticlesSceneNode::getMaterialCount() {
    if (child_mesh)
        return child_mesh->getMaterialCount();
    else
        return 0;
}

irr::video::SMaterial& ChIrrParticlesSceneNode::getMaterial(u32 i) {
    assert(child_mesh);
    return child_mesh->getMaterial(i);
}

void ChIrrParticlesSceneNode::OnAnimate(u32 timeMs) {
    UpdateChildrenHierarchy();

    // setMaterialFlag(irr::video::EMF_NORMALIZE_NORMALS, true);

    // if (child_mesh)
    //  child_mesh->setMaterialFlag(irr::video::EMF_NORMALIZE_NORMALS, true);

    if (IsVisible) {
        // reorient/reposition the particle nodes every frame
        if (particlep && ChronoControlled) {
            irr::core::list<ISceneNode*>::ConstIterator it = getChildren().begin();

            for (unsigned int j = 0; j < (*particlep)->GetNparticles(); j++) {
                // Output: will be an Irrlicht 4x4 matrix
                irr::core::matrix4 irrMat;

                // Get the particle actual rotation, as a 3x3 matrix [A]
                const ChMatrix33<>& chMat = (*particlep)->GetParticle(j).GetA();

                // Fill the upper 3x3 submatrix with the [A] matrix
                // transposed, since Irrlicht uses the row-major style as in D3D
                irrMat[0] = (f32)chMat.GetElementN(0);
                irrMat[1] = (f32)chMat.GetElementN(3);
                irrMat[2] = (f32)chMat.GetElementN(6);

                irrMat[4] = (f32)chMat.GetElementN(1);
                irrMat[5] = (f32)chMat.GetElementN(4);
                irrMat[6] = (f32)chMat.GetElementN(7);

                irrMat[8] = (f32)chMat.GetElementN(2);
                irrMat[9] = (f32)chMat.GetElementN(5);
                irrMat[10] = (f32)chMat.GetElementN(8);

                irrMat[12] = (f32)(*particlep)->GetParticle(j).GetPos().x();
                irrMat[13] = (f32)(*particlep)->GetParticle(j).GetPos().y();
                irrMat[14] = (f32)(*particlep)->GetParticle(j).GetPos().z();

                // Clear the last column to 0 and set low-right corner to 1
                // as in Denavitt-Hartemberg matrices, transposed.
                irrMat[3] = irrMat[7] = irrMat[11] = 0.0f;
                irrMat[15] = 1.0f;

                // Set position and rotation of node using the 4x4 Irrlicht matrix.
                (*it)->setPosition(irrMat.getTranslation());
                (*it)->setRotation(irrMat.getRotationDegrees());

                ++it;
                if (it == getChildren().end())
                    break;
            }
        }
    }

    ISceneNode::OnAnimate(timeMs);
}

scene::IShadowVolumeSceneNode* ChIrrParticlesSceneNode::addShadowVolumeSceneNode(const scene::IMesh* shadowMesh,
                                                                                 s32 id,
                                                                                 bool zfailmethod,
                                                                                 f32 infinity) {
    if (child_mesh)
        return child_mesh->addShadowVolumeSceneNode(shadowMesh, id, zfailmethod, infinity);
    return NULL;
}

void ChIrrParticlesSceneNode::UpdateChildrenHierarchy() {
    size_t npart = (*particlep)->GetNparticles();
    if (this->Nchildren != npart) {
        // delete all children mesh nodes
        this->removeAll();
        // add the new amount of mesh nodes (suboptimal update, but enough for demos)
        if (sample_mesh) {
            for (unsigned int j = 0; j < npart; j++) {
                child_mesh = SceneManager->addAnimatedMeshSceneNode(sample_mesh, this);
                child_mesh->setScale(mesh_scale);
                if (this->sample_texture)
                    child_mesh->setMaterialTexture(0, this->sample_texture);

                if (child_mesh)
                    child_mesh->setMaterialFlag(irr::video::EMF_NORMALIZE_NORMALS, true);
            }
        }
        this->Nchildren = (s32)npart;
    }
}

// -----------------------------------------------------------------------------
// Utility free functions
// -----------------------------------------------------------------------------

scene::ISceneNode* addChParticlesSceneNode(ChSystem* asystem,
                                           scene::ISceneManager* amanager,
                                           scene::IAnimatedMesh* amesh,
                                           irr::core::vector3df amesh_scale,
                                           double mmass,
                                           scene::ISceneNode* aparent,
                                           s32 mid) {
    if (!aparent)
        aparent = amanager->getRootSceneNode();

    // create a ChronoENGINE rigid body
    ChIrrParticlesSceneNode* particleObj =
        new ChIrrParticlesSceneNode(asystem, amesh, amesh_scale, aparent, amanager, mid);
    // set some ChronoENGINE specific properties for the particle cluster...
    particleObj->GetParticles()->SetMass(mmass);

    particleObj->drop();

    return particleObj;
}

scene::ISceneNode* addChParticlesSceneNode_easySpheres(ChSystem* asystem,
                                                       scene::ISceneManager* amanager,
                                                       double mmass,
                                                       double mradius,
                                                       int Hslices,
                                                       int Vslices,
                                                       scene::ISceneNode* aparent,
                                                       s32 mid) {
    static scene::IAnimatedMesh* sphereMesh = 0;

    if (!sphereMesh)
        sphereMesh = createEllipticalMesh(1.0, 1.0, -2, +2, 0, Hslices, Vslices);

    irr::core::vector3df mmeshscale((f32)mradius, (f32)mradius, (f32)mradius);

    // create a ChronoENGINE rigid body
    ChIrrParticlesSceneNode* particleObj = (ChIrrParticlesSceneNode*)addChParticlesSceneNode(
        asystem, amanager, sphereMesh, mmeshscale, mmass, aparent, mid);

    particleObj->GetParticles()->SetCollide(
        false);  // make sure you are not defining the 'sample' collision model with collide =on

    particleObj->GetParticles()->GetCollisionModel()->ClearModel();
    particleObj->GetParticles()->GetCollisionModel()->AddSphere(mradius);
    particleObj->GetParticles()->GetCollisionModel()->BuildModel();

    particleObj->GetParticles()->SetCollide(true);

    return particleObj;
}

}  // end namespace irrlicht
}  // end namespace chrono
