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

#include "chrono/geometry/ChSphere.h"
#include "chrono/geometry/ChBox.h"
#include "chrono/geometry/ChTriangleMeshSoup.h"

#include "chrono_irrlicht/ChBodySceneNodeTools.h"

namespace chrono {
namespace irrlicht {

using namespace irr;
using namespace irr::scene;

// -----------------------------------------------------------------------------

ISceneNode* addChBodySceneNode(ChSystem* asystem,
                               ISceneManager* amanager,
                               IAnimatedMesh* amesh,
                               double mmass,
                               const ChVector<>& position,
                               const ChQuaternion<>& rotation,
                               ISceneNode* aparent,
                               s32 mid) {
    if (!aparent)
        aparent = amanager->getRootSceneNode();

    // create a ChronoENGINE rigid body
    ChBodySceneNode* rigidBodyZ = new ChBodySceneNode(asystem, amesh, aparent, amanager, mid);
    // set some ChronoENGINE specific properties for the body...
    rigidBodyZ->GetBody()->SetPos(position);
    rigidBodyZ->GetBody()->SetRot(rotation);
    rigidBodyZ->GetBody()->SetMass(mmass);

    rigidBodyZ->drop();

    return rigidBodyZ;
}

// -----------------------------------------------------------------------------

ISceneNode* addChBodySceneNode_offsetCOG(ChSystem* asystem,
                                         ISceneManager* amanager,
                                         IAnimatedMesh* amesh,
                                         double mmass,
                                         const ChVector<>& mesh_position,
                                         const ChQuaternion<>& rotation,
                                         const ChVector<>& COG_offset,
                                         ISceneNode* aparent,
                                         s32 mid) {
    if (!aparent)
        aparent = amanager->getRootSceneNode();

    // create a ChronoENGINE rigid body
    ChBodySceneNode* rigidBodyZ = new ChBodySceneNode(asystem, amesh, aparent, amanager, mid, COG_offset);
    // set some ChronoENGINE specific properties for the body...
    rigidBodyZ->GetBody()->SetPos(mesh_position);
    rigidBodyZ->GetBody()->SetRot(rotation);
    rigidBodyZ->GetBody()->SetMass(mmass);

    rigidBodyZ->drop();

    return rigidBodyZ;
}

// -----------------------------------------------------------------------------

ISceneNode* addChBodySceneNode_easySphere(ChSystem* asystem,
                                          ISceneManager* amanager,
                                          double mmass,
                                          const ChVector<>& position,
                                          double mradius,
                                          int Hslices,
                                          int Vslices,
                                          ISceneNode* aparent,
                                          s32 mid) {
    static IAnimatedMesh* sphereMesh = 0;

    if (!sphereMesh)
        sphereMesh = createEllipticalMesh(1.0, 1.0, -2, +2, 0, Hslices, Vslices);

    // create a ChronoENGINE rigid body
    ChBodySceneNode* rigidBodyZ = (ChBodySceneNode*)addChBodySceneNode(asystem, amanager, sphereMesh, mmass, position,
                                                                       ChQuaternion<>(1, 0, 0, 0), aparent, mid);

    rigidBodyZ->setScale(core::vector3df((f32)mradius, (f32)mradius, (f32)mradius));

    rigidBodyZ->GetBody()->GetCollisionModel()->ClearModel();
    rigidBodyZ->GetBody()->GetCollisionModel()->AddSphere(mradius);
    rigidBodyZ->GetBody()->GetCollisionModel()->BuildModel();
    rigidBodyZ->GetBody()->SetCollide(true);

    return rigidBodyZ;
}

// -----------------------------------------------------------------------------

ISceneNode* addChBodySceneNode_easyBox(ChSystem* asystem,
                                       ISceneManager* amanager,
                                       double mmass,
                                       const ChVector<>& position,
                                       const ChQuaternion<>& rotation,
                                       const ChVector<>& size,
                                       ISceneNode* aparent,
                                       s32 mid) {
    static IAnimatedMesh* cubeMesh = 0;

    if (!cubeMesh)
        cubeMesh = amanager->getMesh(GetChronoDataFile("cube.obj").c_str());

    // create a ChronoENGINE rigid body
    ChBodySceneNode* rigidBodyZ =
        (ChBodySceneNode*)addChBodySceneNode(asystem, amanager, cubeMesh, mmass, position, rotation, aparent, mid);

    ChVector<> hsize = size * 0.5;

    core::vector3df irrsize((f32)hsize.x(), (f32)hsize.y(), (f32)hsize.z());
    rigidBodyZ->setScale(irrsize);

    rigidBodyZ->GetBody()->GetCollisionModel()->ClearModel();
    rigidBodyZ->GetBody()->GetCollisionModel()->AddBox(hsize.x(), hsize.y(), hsize.z());
    rigidBodyZ->GetBody()->GetCollisionModel()->BuildModel();
    rigidBodyZ->GetBody()->SetCollide(true);

    return rigidBodyZ;
}

// -----------------------------------------------------------------------------

ISceneNode* addChBodySceneNode_easyCylinder(ChSystem* asystem,
                                            ISceneManager* amanager,
                                            double mmass,
                                            const ChVector<>& position,
                                            const ChQuaternion<>& rotation,
                                            const ChVector<>& size,
                                            ISceneNode* aparent,
                                            s32 mid) {
    static IAnimatedMesh* cylinderMesh = 0;

    if (!cylinderMesh)
        cylinderMesh = amanager->getMesh(GetChronoDataFile("cylinder.obj").c_str());

    // create a ChronoENGINE rigid body
    ChBodySceneNode* rigidBodyZ =
        (ChBodySceneNode*)addChBodySceneNode(asystem, amanager, cylinderMesh, mmass, position, rotation, aparent, mid);

    ChVector<> hsize = size * 0.5;

    core::vector3df irrsize((f32)hsize.x(), (f32)hsize.y(), (f32)hsize.z());
    rigidBodyZ->setScale(irrsize);

    rigidBodyZ->GetBody()->GetCollisionModel()->ClearModel();
    rigidBodyZ->GetBody()->GetCollisionModel()->AddCylinder(hsize.x(), hsize.z(), hsize.y());  // radius, radius, height on y
    rigidBodyZ->GetBody()->GetCollisionModel()->BuildModel();
    rigidBodyZ->GetBody()->SetCollide(true);

    return rigidBodyZ;
}

// -----------------------------------------------------------------------------

ISceneNode* addChBodySceneNode_easyBarrel(ChSystem* asystem,
                                          ISceneManager* amanager,
                                          double mmass,
                                          const ChVector<>& position,
                                          double mradiusH,
                                          double mradiusV,
                                          double mYlow,
                                          double mYhigh,
                                          double mOffset,
                                          int Hslices,
                                          int Vslices,
                                          ISceneNode* aparent,
                                          s32 mid) {
    IAnimatedMesh* barrellMesh =
        createEllipticalMesh((f32)mradiusH, (f32)mradiusV, (f32)mYlow, (f32)mYhigh, (f32)mOffset, Hslices, Vslices);

    // create a ChronoENGINE rigid body
    ChBodySceneNode* rigidBodyZ = (ChBodySceneNode*)addChBodySceneNode(asystem, amanager, barrellMesh, mmass, position,
                                                                       ChQuaternion<>(1, 0, 0, 0), aparent, mid);

    rigidBodyZ->GetBody()->GetCollisionModel()->ClearModel();
    rigidBodyZ->GetBody()->GetCollisionModel()->AddBarrel(mYlow, mYhigh, mradiusV, mradiusH, mOffset);
    rigidBodyZ->GetBody()->GetCollisionModel()->BuildModel();
    rigidBodyZ->GetBody()->SetCollide(true);

    return rigidBodyZ;
}

// -----------------------------------------------------------------------------

ISceneNode* addChBodySceneNode_easyClone(ChSystem* asystem,
                                         ISceneManager* amanager,
                                         ChBodySceneNode* source,
                                         const ChVector<>& position,
                                         const ChQuaternion<>& rotation,
                                         ISceneNode* aparent,
                                         s32 mid) {
    // create a ChronoENGINE rigid body
    ChBodySceneNode* rigidBodyZ = (ChBodySceneNode*)addChBodySceneNode(asystem, amanager,
                                                                       0,  // source->GetChildMesh()->getMesh(),
                                                                       1, position, rotation, aparent, mid);

    // copy all settings of the original body (masses, inertia, etc.)
    rigidBodyZ->GetBody() = std::shared_ptr<ChBody>(source->GetBody().get()->Clone());
    rigidBodyZ->GetBody()->SetSystem(source->GetBody()->GetSystem());
    rigidBodyZ->GetBody()->SetPos(position);
    rigidBodyZ->GetBody()->SetRot(rotation);

    rigidBodyZ->setScale(source->getScale());

    rigidBodyZ->GetBody()->GetCollisionModel()->ClearModel();
    rigidBodyZ->GetBody()->GetCollisionModel()->AddCopyOfAnotherModel(source->GetBody()->GetCollisionModel().get());
    rigidBodyZ->GetBody()->GetCollisionModel()->BuildModel();
    rigidBodyZ->GetBody()->SetCollide(true);

    return rigidBodyZ;
}

// -----------------------------------------------------------------------------

ISceneNode* addChBodySceneNode_easyGenericMesh(ChSystem* asystem,
                                               ISceneManager* amanager,
                                               double mmass,
                                               const ChVector<>& position,
                                               const ChQuaternion<>& rotation,
                                               const char* mesh_filemane,
                                               bool is_static,
                                               bool is_convex,
                                               ISceneNode* aparent,
                                               s32 mid) {
    IAnimatedMesh* genericMesh = 0;

    genericMesh = amanager->getMesh(mesh_filemane);

    assert(genericMesh);

    geometry::ChTriangleMeshSoup temp_trianglemesh;  // temp., only in function scope, since AddTriangleMesh
                                                     // doesn't reference by striding interface -just copy
    fillChTrimeshFromIrlichtMesh(&temp_trianglemesh, genericMesh->getMesh(0));

    // create a ChronoENGINE rigid body
    ChBodySceneNode* rigidBodyZ =
        (ChBodySceneNode*)addChBodySceneNode(asystem, amanager, genericMesh, mmass, position, rotation, aparent, mid);

    rigidBodyZ->GetBody()->GetCollisionModel()->ClearModel();
    rigidBodyZ->GetBody()->GetCollisionModel()->AddTriangleMesh(temp_trianglemesh, is_static, is_convex);
    rigidBodyZ->GetBody()->GetCollisionModel()->BuildModel();
    rigidBodyZ->GetBody()->SetCollide(true);

    rigidBodyZ->GetBody()->SetBodyFixed(is_static);

    return rigidBodyZ;
}

// -----------------------------------------------------------------------------

ISceneNode* addChBodySceneNode_easyStaticMesh(ChSystem* asystem,
                                              ISceneManager* amanager,
                                              const char* mesh_filename,
                                              const ChVector<>& position,
                                              const ChQuaternion<>& rotation,
                                              ISceneNode* aparent,
                                              s32 mid) {
    return addChBodySceneNode_easyGenericMesh(asystem, amanager, 1.0, position, rotation, mesh_filename, true, false,
                                              aparent, mid);
}

// -----------------------------------------------------------------------------

ISceneNode* addChBodySceneNode_easyConvexMesh(ChSystem* asystem,
                                              ISceneManager* amanager,
                                              const char* mesh_filename,
                                              double mmass,
                                              const ChVector<>& position,
                                              const ChQuaternion<>& rotation,
                                              ISceneNode* aparent,
                                              s32 mid) {
    return addChBodySceneNode_easyGenericMesh(asystem, amanager, mmass, position, rotation, mesh_filename, false, true,
                                              aparent, mid);
}

// -----------------------------------------------------------------------------

ISceneNode* addChBodySceneNode_easyConcaveMesh(ChSystem* asystem,
                                               ISceneManager* amanager,
                                               const char* mesh_filename,
                                               double mmass,
                                               const ChVector<>& position,
                                               const ChQuaternion<>& rotation,
                                               ISceneNode* aparent,
                                               s32 mid) {
    return addChBodySceneNode_easyGenericMesh(asystem, amanager, mmass, position, rotation, mesh_filename, false, false,
                                              aparent, mid);
}

}  // end namespace irrlicht
}  // end namespace chrono
