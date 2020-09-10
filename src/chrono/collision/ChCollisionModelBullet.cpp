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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include <memory>
#include <array>

#include "chrono/collision/ChCollisionSystemBullet.h"
#include "chrono/collision/ChCollisionUtils.h"
#include "chrono/collision/ChConvexDecomposition.h"
#include "chrono/collision/ChCollisionModelBullet.h"
#include "chrono/collision/bullet/BulletCollision/CollisionShapes/bt2DShape.h"
#include "chrono/collision/bullet/BulletCollision/CollisionShapes/btBarrelShape.h"
#include "chrono/collision/bullet/BulletCollision/CollisionShapes/btCEtriangleShape.h"
#include "chrono/collision/bullet/btBulletCollisionCommon.h"
#include "chrono/collision/gimpact/GIMPACT/Bullet/btGImpactCollisionAlgorithm.h"
#include "chrono/collision/gimpact/GIMPACTUtils/btGImpactConvexDecompositionShape.h"
#include "chrono/geometry/ChLineArc.h"
#include "chrono/geometry/ChLineSegment.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChPhysicsItem.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {
namespace collision {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChCollisionModelBullet)

ChCollisionModelBullet::ChCollisionModelBullet() {
    bt_collision_object = std::unique_ptr<btCollisionObject>(new btCollisionObject);
    bt_collision_object->setCollisionShape(nullptr);
    bt_collision_object->setUserPointer((void*)this);
}

ChCollisionModelBullet::~ChCollisionModelBullet() {}

int ChCollisionModelBullet::ClearModel() {
    // Delete any existing collision shapes and remove the model from the collision system, if applicable.
    if (m_shapes.size() > 0) {
        m_shapes.clear();

        if (mcontactable &&                                 //
            mcontactable->GetPhysicsItem() &&               //
            mcontactable->GetPhysicsItem()->GetSystem() &&  //
            mcontactable->GetPhysicsItem()->GetCollide()    //
        )
            mcontactable->GetPhysicsItem()->GetSystem()->GetCollisionSystem()->Remove(this);
    }

    bt_collision_object->setCollisionShape(nullptr);
    bt_compound_shape.reset();

    return 1;
}

int ChCollisionModelBullet::BuildModel() {
    // Insert model into collision system, if applicable.
    if (mcontactable &&                                 //
        mcontactable->GetPhysicsItem() &&               //
        mcontactable->GetPhysicsItem()->GetSystem() &&  //
        mcontactable->GetPhysicsItem()->GetCollide()    //
    )
        mcontactable->GetPhysicsItem()->GetSystem()->GetCollisionSystem()->Add(this);

    return 1;
}

static btVector3 ChVectToBullet(const ChVector<>& pos) {
    return btVector3((btScalar)pos.x(), (btScalar)pos.y(), (btScalar)pos.z());
}

static ChVector<> ChBulletToVect(const btVector3& vec) {
    return ChVector<>((double)vec.x(), (double)vec.y(), (double)vec.z());
}

static void ChPosMatrToBullet(const ChVector<>& pos, const ChMatrix33<>& rA, btTransform& mtransform) {
    btMatrix3x3 basisA((btScalar)rA(0, 0), (btScalar)rA(0, 1), (btScalar)rA(0, 2), (btScalar)rA(1, 0),
                       (btScalar)rA(1, 1), (btScalar)rA(1, 2), (btScalar)rA(2, 0), (btScalar)rA(2, 1),
                       (btScalar)rA(2, 2));
    mtransform.setBasis(basisA);
    mtransform.setOrigin(btVector3((btScalar)pos.x(), (btScalar)pos.y(), (btScalar)pos.z()));
}

static void ChCoordsToBullet(const ChCoordsys<>& mcoords, btTransform& mtransform) {
    ChMatrix33<> rA;
    rA.Set_A_quaternion(mcoords.rot);
    btMatrix3x3 basisA((btScalar)rA(0, 0), (btScalar)rA(0, 1), (btScalar)rA(0, 2), (btScalar)rA(1, 0),
                       (btScalar)rA(1, 1), (btScalar)rA(1, 2), (btScalar)rA(2, 0), (btScalar)rA(2, 1),
                       (btScalar)rA(2, 2));
    mtransform.setBasis(basisA);
    mtransform.setOrigin(btVector3((btScalar)mcoords.pos.x(), (btScalar)mcoords.pos.y(), (btScalar)mcoords.pos.z()));
}

static void ChBulletToCoords(const btTransform& mtransform, ChCoordsys<>& mcoords) {
    const btVector3& p = mtransform.getOrigin();
    btQuaternion q = mtransform.getRotation();
    mcoords.pos = ChVector<>((double)p.x(), (double)p.y(), (double)p.z());
    mcoords.rot = ChQuaternion<>((double)q.w(), (double)q.x(), (double)q.y(), (double)q.z());
}

void ChCollisionModelBullet::injectShape(const ChVector<>& pos,
                                         const ChMatrix33<>& rot,
                                         ChCollisionShapeBullet* shape) {
    bool centered = (pos.IsNull() && rot.isIdentity());

    // This is needed so one can later access the model's GetSafeMargin() and GetEnvelope()
    shape->m_bt_shape->setUserPointer(this);

    // If this is the first shape added to the model...
    if (m_shapes.size() == 0) {
        // shape vector: {}
        m_shapes.push_back(std::shared_ptr<ChCollisionShape>(shape));
        if (centered) {
            bt_collision_object->setCollisionShape(shape->m_bt_shape);
            return;
            // shape vector: {centered shape}
        } else {
            bt_compound_shape = chrono_types::make_shared<btCompoundShape>(true);
            bt_compound_shape->setMargin(GetSuggestedFullMargin());
            btTransform mtransform;
            ChPosMatrToBullet(pos, rot, mtransform);
            bt_compound_shape->addChildShape(mtransform, shape->m_bt_shape);
            bt_collision_object->setCollisionShape(bt_compound_shape.get());
            return;
            // shape vector: {off-center shape}
        }
    }

    // If the model currently has only one centered shape...
    if (!bt_compound_shape && m_shapes.size() == 1) {
        // shape vector: {centered shape}
        m_shapes.push_back(std::shared_ptr<ChCollisionShape>(shape));
        bt_compound_shape = chrono_types::make_shared<btCompoundShape>(true);
        bt_compound_shape->setMargin(GetSuggestedFullMargin());
        btTransform mtransform;
        mtransform.setIdentity();
        bt_compound_shape->addChildShape(mtransform, ((ChCollisionShapeBullet*)m_shapes[0].get())->m_bt_shape);
        ChPosMatrToBullet(pos, rot, mtransform);
        bt_compound_shape->addChildShape(mtransform, shape->m_bt_shape);
        bt_collision_object->setCollisionShape(bt_compound_shape.get());
        return;
        // shape vector: {old centered shape | shape}
    }

    // Already working with a compound...
    // shape vector: {old shape | old shape | ...}
    m_shapes.push_back(std::shared_ptr<ChCollisionShape>(shape));
    btTransform mtransform;
    ChPosMatrToBullet(pos, rot, mtransform);
    bt_compound_shape->addChildShape(mtransform, shape->m_bt_shape);
    return;
    // shape vector: {old shape | old shape | ... | new shape}
}

bool ChCollisionModelBullet::AddSphere(std::shared_ptr<ChMaterialSurface> material,
                                       double radius,
                                       const ChVector<>& pos) {
    // adjust default inward 'safe' margin (always as radius)
    SetSafeMargin(radius);

    auto shape = new ChCollisionShapeBullet(ChCollisionShape::Type::SPHERE, material);

    shape->m_bt_shape = new btSphereShape((btScalar)(radius + GetEnvelope()));
    shape->m_bt_shape->setMargin((btScalar)GetSuggestedFullMargin());

    injectShape(pos, ChMatrix33<>(1), shape);
    return true;
}

bool ChCollisionModelBullet::AddEllipsoid(std::shared_ptr<ChMaterialSurface> material,
                                          double rx,
                                          double ry,
                                          double rz,
                                          const ChVector<>& pos,
                                          const ChMatrix33<>& rot) {
    auto shape = new ChCollisionShapeBullet(ChCollisionShape::Type::ELLIPSOID, material);

    btScalar rad = 1.0;
    btVector3 spos(0, 0, 0);
    double arx = rx + GetEnvelope();
    double ary = ry + GetEnvelope();
    double arz = rz + GetEnvelope();
    shape->m_bt_shape = new btMultiSphereShape(&spos, &rad, 1);
    shape->m_bt_shape->setLocalScaling(btVector3((btScalar)arx, (btScalar)ary, (btScalar)arz));
    shape->m_bt_shape->setMargin((btScalar)ChMin(GetSuggestedFullMargin(), 0.9 * ChMin(ChMin(arx, ary), arz)));

    injectShape(pos, rot, shape);
    return true;
}

bool ChCollisionModelBullet::AddBox(std::shared_ptr<ChMaterialSurface> material,
                                    double hx,
                                    double hy,
                                    double hz,
                                    const ChVector<>& pos,
                                    const ChMatrix33<>& rot) {
    // adjust default inward margin (if object too thin)
    SetSafeMargin(ChMin(GetSafeMargin(), 0.2 * ChMin(ChMin(hx, hy), hz)));

    auto shape = new ChCollisionShapeBullet(ChCollisionShape::Type::BOX, material);

    btScalar ahx = (btScalar)(hx + GetEnvelope());
    btScalar ahy = (btScalar)(hy + GetEnvelope());
    btScalar ahz = (btScalar)(hz + GetEnvelope());
    shape->m_bt_shape = new btBoxShape(btVector3(ahx, ahy, ahz));
    shape->m_bt_shape->setMargin((btScalar)GetSuggestedFullMargin());

    injectShape(pos, rot, shape);
    return true;
}

bool ChCollisionModelBullet::AddCylinder(std::shared_ptr<ChMaterialSurface> material,
                                         double rx,
                                         double rz,
                                         double hy,
                                         const ChVector<>& pos,
                                         const ChMatrix33<>& rot) {
    // adjust default inward margin (if object too thin)
    SetSafeMargin(ChMin(GetSafeMargin(), 0.2 * ChMin(ChMin(rx, rz), 0.5 * hy)));

    auto shape = new ChCollisionShapeBullet(ChCollisionShape::Type::CYLINDER, material);

    btScalar arx = (btScalar)(rx + GetEnvelope());
    btScalar arz = (btScalar)(rz + GetEnvelope());
    btScalar ahy = (btScalar)(hy + GetEnvelope());
    shape->m_bt_shape = new btCylinderShape(btVector3(arx, ahy, arz));
    shape->m_bt_shape->setMargin((btScalar)GetSuggestedFullMargin());

    injectShape(pos, rot, shape);
    return true;
}

bool ChCollisionModelBullet::AddBarrel(std::shared_ptr<ChMaterialSurface> material,
                                       double Y_low,
                                       double Y_high,
                                       double R_vert,
                                       double R_hor,
                                       double R_offset,
                                       const ChVector<>& pos,
                                       const ChMatrix33<>& rot) {
    // adjust default inward margin (if object too thin)
    SetSafeMargin(ChMin(GetSafeMargin(), 0.15 * ChMin(ChMin(R_vert, R_hor), Y_high - Y_low)));

    auto shape = new ChCollisionShapeBullet(ChCollisionShape::Type::BARREL, material);

    btScalar sY_low = (btScalar)(Y_low - model_envelope);
    btScalar sY_high = (btScalar)(Y_high + model_envelope);
    btScalar sR_vert = (btScalar)(R_vert + model_envelope);
    btScalar sR_hor = (btScalar)(R_hor + model_envelope);
    btScalar sR_offset = (btScalar)(R_offset);
    shape->m_bt_shape = new btBarrelShape(sY_low, sY_high, sR_vert, sR_hor, sR_offset);
    shape->m_bt_shape->setMargin((btScalar)GetSuggestedFullMargin());

    injectShape(pos, rot, shape);
    return true;
}

bool ChCollisionModelBullet::Add2Dpath(std::shared_ptr<ChMaterialSurface> material,
                                       std::shared_ptr<geometry::ChLinePath> mpath,
                                       const ChVector<>& pos,
                                       const ChMatrix33<>& rot,
                                       const double mthickness) {
    // The envelope is not used in this type of collision primitive.
    SetEnvelope(0);

    // if (!mpath.Get_closed())
    //    throw ChException("Error! Add2Dpath requires a CLOSED ChLinePath!");

    for (size_t i = 0; i < mpath->GetSubLinesCount(); ++i) {
        if (auto msegment = std::dynamic_pointer_cast<geometry::ChLineSegment>(mpath->GetSubLineN(i))) {
            if (msegment->pA.z() != msegment->pB.z())
                throw ChException("Error! Add2Dpath: sub segment of ChLinePath not parallel to XY plane!");

            btVector3 pa((btScalar)msegment->pA.x(), (btScalar)msegment->pA.y(), (btScalar)0);
            btVector3 pb((btScalar)msegment->pB.x(), (btScalar)msegment->pB.y(), (btScalar)0);

            auto shape = new ChCollisionShapeBullet(ChCollisionShape::Type::PATH2D, material);
            shape->m_bt_shape = new bt2DsegmentShape(pa, pb, (btScalar)mthickness);
            shape->m_bt_shape->setMargin((btScalar)GetSuggestedFullMargin());
            injectShape(pos, rot, shape);
        } else if (auto marc = std::dynamic_pointer_cast<geometry::ChLineArc>(mpath->GetSubLineN(i))) {
            if ((marc->origin.rot.e1() != 0) ||  (marc->origin.rot.e2() != 0))
                throw ChException("Error! Add2Dpath: a sub arc of ChLinePath not parallel to XY plane!");
            double mangle1 = marc->angle1;
            double mangle2 = marc->angle2;
            if (mangle1 - mangle2 == CH_C_2PI)
                mangle1 -= 1e-7;
            auto shape = new ChCollisionShapeBullet(ChCollisionShape::Type::PATH2D, material);
            shape->m_bt_shape =
                new bt2DarcShape((btScalar)marc->origin.pos.x(), (btScalar)marc->origin.pos.y(), (btScalar)marc->radius,
                                 (btScalar)mangle1, (btScalar)mangle2, marc->counterclockwise, (btScalar)mthickness);
            shape->m_bt_shape->setMargin((btScalar)GetSuggestedFullMargin());
            injectShape(pos, rot, shape);
        } else {
            throw ChException("Error! Add2Dpath: ChLinePath must contain only ChLineArc and/or ChLineSegment.");
        }

        size_t i_prev = i;
        size_t i_next = i + 1;
        if (i_next >= mpath->GetSubLinesCount())
            if ((mpath->GetEndA() - mpath->GetEndB()).Length() <
                1e-9)        // can't use Get_closed() that is user preference via Set_closed()
                i_next = 0;  // closed path
        if (i_next < mpath->GetSubLinesCount()) {
            std::shared_ptr<geometry::ChLine> mline_prev = mpath->GetSubLineN(i_prev);
            std::shared_ptr<geometry::ChLine> mline_next = mpath->GetSubLineN(i_next);
            ChVector<> pos_prev, pos_next;
            ChVector<> dir_prev, dir_next;
            mline_prev->Evaluate(pos_prev, 1);
            mline_next->Evaluate(pos_next, 0);
            mline_prev->Derive(dir_prev, 1);
            mline_next->Derive(dir_next, 0);
            dir_prev.Normalize();
            dir_next.Normalize();

            // check if connected segments
            if ((pos_prev - pos_next).Length() > 1e-9)
                throw ChException(
                    "Error! Add2Dpath: ChLinePath must contain sequence of connected segments/arcs, with no gaps");

            // insert a 0-radius fillet arc at sharp corners, to allow for sharp-corner vs arc/segment
            if (Vcross(dir_prev, dir_next).z() < -1e-9) {
                double mangle1 = atan2(dir_prev.y(), dir_prev.x()) + CH_C_PI_2;
                double mangle2 = atan2(dir_next.y(), dir_next.x()) + CH_C_PI_2;

                auto shape = new ChCollisionShapeBullet(ChCollisionShape::Type::PATH2D, material);
                shape->m_bt_shape = new bt2DarcShape((btScalar)pos_prev.x(), (btScalar)pos_prev.y(), (btScalar)0,
                                                     (btScalar)mangle1, (btScalar)mangle2, false, (btScalar)mthickness);
                shape->m_bt_shape->setMargin((btScalar)GetSuggestedFullMargin());
                injectShape(pos, rot, shape);
                // GetLog() << "convex corner between " << i_next << " and " << i_next << " w.angles: " << mangle1 << "
                // " << mangle2 << "\n";
            } else {
                // GetLog() << "concave corner between " << i_next << " and " << i_next << "\n";
            }
        }
    }

    return true;
}

bool ChCollisionModelBullet::AddPoint(std::shared_ptr<ChMaterialSurface> material,
                                      double radius,
                                      const ChVector<>& pos) {
    // adjust default inward 'safe' margin (always as radius)
    SetSafeMargin(radius);

    auto shape = new ChCollisionShapeBullet(ChCollisionShape::Type::POINT, material);

    shape->m_bt_shape = new btPointShape((btScalar)(radius + GetEnvelope()));
    shape->m_bt_shape->setMargin((btScalar)GetSuggestedFullMargin());

    injectShape(pos, ChMatrix33<>(1), shape);
    return true;
}

bool ChCollisionModelBullet::AddTriangleProxy(std::shared_ptr<ChMaterialSurface> material,
                                              ChVector<>* p1,
                                              ChVector<>* p2,
                                              ChVector<>* p3,
                                              ChVector<>* ep1,
                                              ChVector<>* ep2,
                                              ChVector<>* ep3,
                                              bool mowns_vertex_1,
                                              bool mowns_vertex_2,
                                              bool mowns_vertex_3,
                                              bool mowns_edge_1,
                                              bool mowns_edge_2,
                                              bool mowns_edge_3,
                                              double msphereswept_rad) {
    // adjust default inward 'safe' margin (always as radius)
    SetSafeMargin(msphereswept_rad);

    auto shape = new ChCollisionShapeBullet(ChCollisionShape::Type::TRIANGLE, material);

    shape->m_bt_shape = new btCEtriangleShape(p1, p2, p3, ep1, ep2, ep3, mowns_vertex_1, mowns_vertex_2, mowns_vertex_3,
                                              mowns_edge_1, mowns_edge_2, mowns_edge_3, msphereswept_rad);
    shape->m_bt_shape->setMargin((btScalar)GetSuggestedFullMargin());

    injectShape(VNULL, ChMatrix33<>(1), shape);
    return true;
}

bool ChCollisionModelBullet::AddConvexHull(std::shared_ptr<ChMaterialSurface> material,
                                           const std::vector<ChVector<double>>& pointlist,
                                           const ChVector<>& pos,
                                           const ChMatrix33<>& rot) {
    // adjust default inward margin (if object too thin)
    ChVector<> aabbMax(-1e9, -1e9, -1e9);
    ChVector<> aabbMin(1e9, 1e9, 1e9);
    for (size_t i = 0; i < pointlist.size(); ++i) {
        aabbMax.x() = ChMax(aabbMax.x(), pointlist[i].x());
        aabbMax.y() = ChMax(aabbMax.y(), pointlist[i].y());
        aabbMax.z() = ChMax(aabbMax.z(), pointlist[i].z());
        aabbMin.x() = ChMin(aabbMin.x(), pointlist[i].x());
        aabbMin.y() = ChMin(aabbMin.y(), pointlist[i].y());
        aabbMin.z() = ChMin(aabbMin.z(), pointlist[i].z());
    }
    ChVector<> aabbsize = aabbMax - aabbMin;
    double approx_chord = ChMin(ChMin(aabbsize.x(), aabbsize.y()), aabbsize.z());
    // override the inward margin if larger than 0.2 chord:
    SetSafeMargin((btScalar)ChMin(GetSafeMargin(), approx_chord * 0.2));

    // shrink the convex hull by GetSafeMargin()
    collision::ChConvexHullLibraryWrapper lh;
    geometry::ChTriangleMeshConnected mmesh;
    lh.ComputeHull(pointlist, mmesh);
    mmesh.MakeOffset(-GetSafeMargin());

    auto shape = new ChCollisionShapeBullet(ChCollisionShape::Type::CONVEXHULL, material);

    auto bt_shape = new btConvexHullShape;
    for (unsigned int i = 0; i < mmesh.m_vertices.size(); i++) {
        bt_shape->addPoint(btVector3((btScalar)mmesh.m_vertices[i].x(), (btScalar)mmesh.m_vertices[i].y(),
                                     (btScalar)mmesh.m_vertices[i].z()));
    }
    bt_shape->setMargin((btScalar)GetSuggestedFullMargin());
    bt_shape->recalcLocalAabb();
    shape->m_bt_shape = bt_shape;

    injectShape(pos, rot, shape);
    return true;
}

// These classes inherit the Bullet triangle mesh and add just a single feature: when this shape is deleted, also delete
// the referenced triangle mesh interface. Hence, when a btBvhTriangleMeshShape_handlemesh is added to the list of
// shapes of this ChCollisionModelBullet, there's no need to remember to delete the mesh interface because it dies with
// the model, when shapes are deleted. This is just to avoid adding a pointer to a triangle interface in all collision
// models, when not needed.

class btBvhTriangleMeshShape_handlemesh : public btBvhTriangleMeshShape {
    btStridingMeshInterface* minterface;

  public:
    btBvhTriangleMeshShape_handlemesh(btStridingMeshInterface* meshInterface)
        : btBvhTriangleMeshShape(meshInterface, true), minterface(meshInterface){};

    ~btBvhTriangleMeshShape_handlemesh() {
        if (minterface)
            delete minterface;
        minterface = 0;  // also delete the mesh interface
    }
};

class btConvexTriangleMeshShape_handlemesh : public btConvexTriangleMeshShape {
    btStridingMeshInterface* minterface;

  public:
    btConvexTriangleMeshShape_handlemesh(btStridingMeshInterface* meshInterface)
        : btConvexTriangleMeshShape(meshInterface), minterface(meshInterface){};

    ~btConvexTriangleMeshShape_handlemesh() {
        if (minterface)
            delete minterface;
        minterface = 0;  // also delete the mesh interface
    }
};

class btGImpactConvexDecompositionShape_handlemesh : public btGImpactConvexDecompositionShape {
    btStridingMeshInterface* minterface;

  public:
    btGImpactConvexDecompositionShape_handlemesh(btStridingMeshInterface* meshInterface)
        : btGImpactConvexDecompositionShape(meshInterface, btVector3(1.f, 1.f, 1.f), btScalar(0.01)),
          minterface(meshInterface){
              // setLocalScaling(btVector3(1.f,1.f,1.f));
          };

    virtual ~btGImpactConvexDecompositionShape_handlemesh() {
        if (minterface)
            delete minterface;
        minterface = 0;  // also delete the mesh interface
    }
};

class btGImpactMeshShape_handlemesh : public btGImpactMeshShape {
    btStridingMeshInterface* minterface;

  public:
    btGImpactMeshShape_handlemesh(btStridingMeshInterface* meshInterface)
        : btGImpactMeshShape(meshInterface),
          minterface(meshInterface){
              // setLocalScaling(btVector3(1.f,1.f,1.f));
          };

    virtual ~btGImpactMeshShape_handlemesh() {
        if (minterface)
            delete minterface;
        minterface = 0;  // also delete the mesh interface
    }
};

bool ChCollisionModelBullet::AddTriangleMesh(std::shared_ptr<ChMaterialSurface> material,
                                             std::shared_ptr<geometry::ChTriangleMesh> trimesh,
                                             bool is_static,
                                             bool is_convex,
                                             const ChVector<>& pos,
                                             const ChMatrix33<>& rot,
                                             double sphereswept_thickness) {
    if (!trimesh->getNumTriangles())
        return false;

    m_trimeshes.push_back(trimesh);  // cache pointer to triangle mesh

    if (auto mesh = std::dynamic_pointer_cast<geometry::ChTriangleMeshConnected>(trimesh)) {
        std::vector<std::array<int, 4>> trimap;
        mesh->ComputeNeighbouringTriangleMap(trimap);

        std::map<std::pair<int, int>, std::pair<int, int>> winged_edges;
        mesh->ComputeWingedEdges(winged_edges, true);

        std::vector<bool> added_vertexes(mesh->m_vertices.size());

        // iterate on triangles
        for (int it = 0; it < mesh->m_face_v_indices.size(); ++it) {
            // edges = pairs of vertexes indexes
            std::pair<int, int> medgeA(mesh->m_face_v_indices[it].x(), mesh->m_face_v_indices[it].y());
            std::pair<int, int> medgeB(mesh->m_face_v_indices[it].y(), mesh->m_face_v_indices[it].z());
            std::pair<int, int> medgeC(mesh->m_face_v_indices[it].z(), mesh->m_face_v_indices[it].x());
            // vertex indexes in edges: always in increasing order to avoid ambiguous duplicated edges
            if (medgeA.first > medgeA.second)
                medgeA = std::pair<int, int>(medgeA.second, medgeA.first);
            if (medgeB.first > medgeB.second)
                medgeB = std::pair<int, int>(medgeB.second, medgeB.first);
            if (medgeC.first > medgeC.second)
                medgeC = std::pair<int, int>(medgeC.second, medgeC.first);
            auto wingedgeA = winged_edges.find(medgeA);
            auto wingedgeB = winged_edges.find(medgeB);
            auto wingedgeC = winged_edges.find(medgeC);

            int i_wingvertex_A = -1;
            int i_wingvertex_B = -1;
            int i_wingvertex_C = -1;

            if (trimap[it][1] != -1) {
                i_wingvertex_A = mesh->m_face_v_indices[trimap[it][1]].x();
                if (mesh->m_face_v_indices[trimap[it][1]].y() != wingedgeA->first.first &&
                    mesh->m_face_v_indices[trimap[it][1]].y() != wingedgeA->first.second)
                    i_wingvertex_A = mesh->m_face_v_indices[trimap[it][1]].y();
                if (mesh->m_face_v_indices[trimap[it][1]].z() != wingedgeA->first.first &&
                    mesh->m_face_v_indices[trimap[it][1]].z() != wingedgeA->first.second)
                    i_wingvertex_A = mesh->m_face_v_indices[trimap[it][1]].z();
            }

            if (trimap[it][2] != -1) {
                i_wingvertex_B = mesh->m_face_v_indices[trimap[it][2]].x();
                if (mesh->m_face_v_indices[trimap[it][2]].y() != wingedgeB->first.first &&
                    mesh->m_face_v_indices[trimap[it][2]].y() != wingedgeB->first.second)
                    i_wingvertex_B = mesh->m_face_v_indices[trimap[it][2]].y();
                if (mesh->m_face_v_indices[trimap[it][2]].z() != wingedgeB->first.first &&
                    mesh->m_face_v_indices[trimap[it][2]].z() != wingedgeB->first.second)
                    i_wingvertex_B = mesh->m_face_v_indices[trimap[it][2]].z();
            }

            if (trimap[it][3] != -1) {
                i_wingvertex_C = mesh->m_face_v_indices[trimap[it][3]].x();
                if (mesh->m_face_v_indices[trimap[it][3]].y() != wingedgeC->first.first &&
                    mesh->m_face_v_indices[trimap[it][3]].y() != wingedgeC->first.second)
                    i_wingvertex_C = mesh->m_face_v_indices[trimap[it][3]].y();
                if (mesh->m_face_v_indices[trimap[it][3]].z() != wingedgeC->first.first &&
                    mesh->m_face_v_indices[trimap[it][3]].z() != wingedgeC->first.second)
                    i_wingvertex_C = mesh->m_face_v_indices[trimap[it][3]].z();
            }

            // Add a triangle proxy collision shape.
            // For a non-wing vertex (i.e. 'free' edge), point to opposite vertex, that is the vertex in triangle not
            // belonging to edge. Indicate is an edge is owned by this triangle. Otherwise, they belong to a neighboring
            // triangle.
            AddTriangleProxy(
                material, &mesh->m_vertices[mesh->m_face_v_indices[it].x()],
                &mesh->m_vertices[mesh->m_face_v_indices[it].y()], &mesh->m_vertices[mesh->m_face_v_indices[it].z()],
                wingedgeA->second.second != -1 ? &mesh->m_vertices[i_wingvertex_A]
                                               : &mesh->m_vertices[mesh->m_face_v_indices[it].z()],
                wingedgeB->second.second != -1 ? &mesh->m_vertices[i_wingvertex_B]
                                               : &mesh->m_vertices[mesh->m_face_v_indices[it].x()],
                wingedgeC->second.second != -1 ? &mesh->m_vertices[i_wingvertex_C]
                                               : &mesh->m_vertices[mesh->m_face_v_indices[it].y()],
                !added_vertexes[mesh->m_face_v_indices[it].x()], !added_vertexes[mesh->m_face_v_indices[it].y()],
                !added_vertexes[mesh->m_face_v_indices[it].z()], wingedgeA->second.first != -1,
                wingedgeB->second.first != -1, wingedgeC->second.first != -1, sphereswept_thickness);

            // Mark added vertexes
            added_vertexes[mesh->m_face_v_indices[it].x()] = true;
            added_vertexes[mesh->m_face_v_indices[it].y()] = true;
            added_vertexes[mesh->m_face_v_indices[it].z()] = true;
            // Mark added edges, setting to -1 the 'ti' id of 1st triangle in winged edge {{vi,vj}{ti,tj}}
            wingedgeA->second.first = -1;
            wingedgeB->second.first = -1;
            wingedgeC->second.first = -1;
        }
        return true;
    }

    btTriangleMesh* bulletMesh = new btTriangleMesh;
    for (int i = 0; i < trimesh->getNumTriangles(); i++) {
        // bulletMesh->m_weldingThreshold = ...
        bulletMesh->addTriangle(ChVectToBullet(trimesh->getTriangle(i).p1), ChVectToBullet(trimesh->getTriangle(i).p2),
                                ChVectToBullet(trimesh->getTriangle(i).p3),
                                true);  // try to remove duplicate vertices
    }

    if (is_static) {
        // Here a static btBvhTriangleMeshShape suffices, but btGImpactMeshShape might work better?
        auto shape = new ChCollisionShapeBullet(ChCollisionShape::Type::TRIANGLEMESH, material);
        shape->m_bt_shape = (btBvhTriangleMeshShape*)new btBvhTriangleMeshShape_handlemesh(bulletMesh);
        shape->m_bt_shape->setMargin((btScalar)GetSafeMargin());
        injectShape(pos, rot, shape);
    } else {
        if (is_convex) {
            auto shape = new ChCollisionShapeBullet(ChCollisionShape::Type::TRIANGLEMESH, material);
            shape->m_bt_shape = (btConvexTriangleMeshShape*)new btConvexTriangleMeshShape_handlemesh(bulletMesh);
            shape->m_bt_shape->setMargin((btScalar)GetEnvelope());
            injectShape(pos, rot, shape);
        } else {
            // Note: currently there's no 'perfect' convex decomposition method, so code here is a bit experimental...

            /*
            // using the HACD convex decomposition
            auto mydecompositionHACD = chrono_types::make_shared<ChConvexDecompositionHACD>();
            mydecompositionHACD->AddTriangleMesh(*trimesh);
            mydecompositionHACD->SetParameters(2,      // clusters
                                               0,      // no decimation
                                               0.0,    // small cluster threshold
                                               false,  // add faces points
                                               false,  // add extra dist points
                                               100.0,  // max concavity
                                               30,     // cc connect dist
                                               0.0,    // volume weight beta
                                               0.0,    // compacity alpha
                                               50      // vertices per cc
            );
            mydecompositionHACD->ComputeConvexDecomposition();
            AddTriangleMeshConcaveDecomposed(material, mydecompositionHACD, pos, rot);
            */

            // using HACDv2 convex decomposition
            auto mydecompositionHACDv2 = chrono_types::make_shared<ChConvexDecompositionHACDv2>();
            mydecompositionHACDv2->Reset();
            mydecompositionHACDv2->AddTriangleMesh(*trimesh);
            mydecompositionHACDv2->SetParameters(  //
                512,                               // max hull count
                256,                               // max hull merge
                64,                                // max hull vettices
                0.2f,                              // concavity
                0.0f,                              // small cluster threshold
                1e-9f                              // fuse tolerance
            );
            mydecompositionHACDv2->ComputeConvexDecomposition();
            AddTriangleMeshConcaveDecomposed(material, mydecompositionHACDv2, pos, rot);
        }
    }

    return true;
}

bool ChCollisionModelBullet::AddTriangleMeshConcave(std::shared_ptr<ChMaterialSurface> material,
                                                    std::shared_ptr<geometry::ChTriangleMesh> trimesh,
                                                    const ChVector<>& pos,
                                                    const ChMatrix33<>& rot) {
    if (!trimesh->getNumTriangles())
        return false;

    btTriangleMesh* bulletMesh = new btTriangleMesh;
    for (int i = 0; i < trimesh->getNumTriangles(); i++) {
        // bulletMesh->m_weldingThreshold = ...
        bulletMesh->addTriangle(ChVectToBullet(trimesh->getTriangle(i).p1), ChVectToBullet(trimesh->getTriangle(i).p2),
                                ChVectToBullet(trimesh->getTriangle(i).p3),
                                true);  // try to remove duplicate vertices
    }

    SetSafeMargin(0);

    auto shape = new ChCollisionShapeBullet(ChCollisionShape::Type::TRIANGLEMESH, material);

    // Use the GImpact custom mesh-mesh algorithm
    auto bt_shape = new btGImpactMeshShape_handlemesh(bulletMesh);
    bt_shape->setMargin((btScalar)GetEnvelope());
    bt_shape->updateBound();
    shape->m_bt_shape = bt_shape;

    injectShape(pos, rot, shape);
    return true;
}

bool ChCollisionModelBullet::AddTriangleMeshConcaveDecomposed(std::shared_ptr<ChMaterialSurface> material,
                                                              std::shared_ptr<ChConvexDecomposition> mydecomposition,
                                                              const ChVector<>& pos,
                                                              const ChMatrix33<>& rot) {
    // note: since the convex hulls are ot shrunk, the safe margin will be set to zero (must be set before adding them)
    SetSafeMargin(0);

    for (unsigned int j = 0; j < mydecomposition->GetHullCount(); j++) {
        std::vector<ChVector<double>> ptlist;
        mydecomposition->GetConvexHullResult(j, ptlist);

        if (ptlist.size())
            AddConvexHull(material, ptlist, pos, rot);
    }

    return true;
}

bool ChCollisionModelBullet::AddCopyOfAnotherModel(ChCollisionModel* other) {
    SetSafeMargin(other->GetSafeMargin());
    SetEnvelope(other->GetEnvelope());

    m_shapes.clear();
    CopyShapes(other);

    auto other_bt = static_cast<ChCollisionModelBullet*>(other);
    bt_collision_object->setCollisionShape(other_bt->GetBulletModel()->getCollisionShape());
    bt_compound_shape = other_bt->bt_compound_shape;

    return true;
}

void ChCollisionModelBullet::SetFamily(int mfamily) {
    ChCollisionModel::SetFamily(mfamily);
    onFamilyChange();
}

int ChCollisionModelBullet::GetFamily() {
    int fam;
    if (!bt_collision_object->getBroadphaseHandle())
        return -1;
    for (fam = 0; fam < 16; fam++)
        if (((short int)0x1 << fam) & bt_collision_object->getBroadphaseHandle()->m_collisionFilterGroup)
            return fam;
    return fam;
}

void ChCollisionModelBullet::SetFamilyMaskNoCollisionWithFamily(int mfamily) {
    ChCollisionModel::SetFamilyMaskNoCollisionWithFamily(mfamily);
    onFamilyChange();
}

void ChCollisionModelBullet::SetFamilyMaskDoCollisionWithFamily(int mfamily) {
    ChCollisionModel::SetFamilyMaskDoCollisionWithFamily(mfamily);
    onFamilyChange();
}

bool ChCollisionModelBullet::GetFamilyMaskDoesCollisionWithFamily(int mfamily) {
    assert(mfamily < 16);
    if (!bt_collision_object->getBroadphaseHandle())
        return false;
    short familyflag = (1 << mfamily);
    return (bt_collision_object->getBroadphaseHandle()->m_collisionFilterMask & familyflag) != 0;
}

void ChCollisionModelBullet::SetFamilyGroup(short int group) {
    ChCollisionModel::SetFamilyGroup(group);
    onFamilyChange();
}

void ChCollisionModelBullet::SetFamilyMask(short int mask) {
    ChCollisionModel::SetFamilyMask(mask);
    onFamilyChange();
}

void ChCollisionModelBullet::onFamilyChange() {
    if (!bt_collision_object->getBroadphaseHandle())
        return;

    SyncPosition();

    // Trick to avoid troubles if setting mask or family when model is already overlapping to some other model
    auto mcosys = mcontactable->GetPhysicsItem()->GetSystem()->GetCollisionSystem();
    mcosys->Remove(this);

    auto mcs = std::static_pointer_cast<ChCollisionSystemBullet>(mcosys);
    mcs->GetBulletCollisionWorld()->addCollisionObject(bt_collision_object.get(), family_group, family_mask);
}

void ChCollisionModelBullet::GetAABB(ChVector<>& bbmin, ChVector<>& bbmax) const {
    btVector3 btmin;
    btVector3 btmax;
    if (bt_collision_object->getCollisionShape())
        bt_collision_object->getCollisionShape()->getAabb(bt_collision_object->getWorldTransform(), btmin, btmax);
    bbmin.Set(btmin.x(), btmin.y(), btmin.z());
    bbmax.Set(btmax.x(), btmax.y(), btmax.z());
}

void ChCollisionModelBullet::SyncPosition() {
    ChCoordsys<> mcsys = mcontactable->GetCsysForCollisionModel();

    bt_collision_object->getWorldTransform().setOrigin(
        btVector3((btScalar)mcsys.pos.x(), (btScalar)mcsys.pos.y(), (btScalar)mcsys.pos.z()));
    const ChMatrix33<>& rA(mcsys.rot);
    btMatrix3x3 basisA((btScalar)rA(0, 0), (btScalar)rA(0, 1), (btScalar)rA(0, 2), (btScalar)rA(1, 0),
                       (btScalar)rA(1, 1), (btScalar)rA(1, 2), (btScalar)rA(2, 0), (btScalar)rA(2, 1),
                       (btScalar)rA(2, 2));
    bt_collision_object->getWorldTransform().setBasis(basisA);
}

bool ChCollisionModelBullet::SetSphereRadius(double coll_radius, double out_envelope) {
    if (m_shapes.size() != 1)
        return false;

    auto bt_shape = ((ChCollisionShapeBullet*)m_shapes[0].get())->m_bt_shape;
    if (btSphereShape* bt_sphere_shape = dynamic_cast<btSphereShape*>(bt_shape)) {
        SetSafeMargin(coll_radius);
        SetEnvelope(out_envelope);
        bt_sphere_shape->setUnscaledRadius((btScalar)(coll_radius + out_envelope));
        ////bt_sphere_shape->setMargin((btScalar)(coll_radius + out_envelope));
        return true;
    }

    return false;
}

ChCoordsys<> ChCollisionModelBullet::GetShapePos(int index) const {
    if (!bt_compound_shape) {
        // if not using a compound, there must be a single centered shape
        assert(m_shapes.size() == 1);
        assert(index == 0);
        return ChCoordsys<>();
    }

    ChCoordsys<> csys;
    ChBulletToCoords(bt_compound_shape->getChildTransform(index), csys);

    return csys;
}

std::vector<double> ChCollisionModelBullet::GetShapeDimensions(int index) const {
    assert(index < GetNumShapes());

    auto shape = std::static_pointer_cast<ChCollisionShapeBullet>(m_shapes[index]);

    std::vector<double> dims;
    switch (m_shapes[index]->GetType()) {
        case ChCollisionShape::Type::SPHERE: {
            auto bt_sphere = static_cast<btSphereShape*>(shape->m_bt_shape);
            auto radius = (double)bt_sphere->getImplicitShapeDimensions().getX();
            dims = {radius};
            break;
        }
        case ChCollisionShape::Type::BOX: {
            auto bt_box = static_cast<btBoxShape*>(shape->m_bt_shape);
            auto hdims = ChBulletToVect(bt_box->getHalfExtentsWithoutMargin());
            dims = {hdims.x(), hdims.y(), hdims.z()};
            break;
        }
        case ChCollisionShape::Type::ELLIPSOID: {
            auto bt_ell = static_cast<btMultiSphereShape*>(shape->m_bt_shape);
            auto rx = (double)bt_ell->getSphereRadius(0);
            auto ry = (double)bt_ell->getSphereRadius(1);
            auto rz = (double)bt_ell->getSphereRadius(2);
            dims = {rx, ry, rz};
            break;
        }
        case ChCollisionShape::Type::CYLINDER: {
            auto bt_cyl = static_cast<btCylinderShape*>(shape->m_bt_shape);
            auto hdims = ChBulletToVect(bt_cyl->getHalfExtentsWithoutMargin());
            dims = {hdims.x(), hdims.z(), hdims.y()};
            break;
        }
        default:
            break;
    }

    return dims;
}

void ChCollisionModelBullet::ArchiveOUT(ChArchiveOut& marchive) {
    //// RADU TODO
}

void ChCollisionModelBullet::ArchiveIN(ChArchiveIn& marchive) {
    //// RADU TODO
}

/*
void __recurse_add_newcollshapes(btCollisionShape* ashape, std::vector<std::shared_ptr<btCollisionShape>>& shapes) {
    if (ashape) {
        shapes.push_back(std::shared_ptr<btCollisionShape>(ashape));

        if (ashape->getShapeType() == COMPOUND_SHAPE_PROXYTYPE) {
            btCompoundShape* compoundShape = (btCompoundShape*)ashape;
            for (int shi = 0; shi < compoundShape->getNumChildShapes(); shi++) {
                __recurse_add_newcollshapes(compoundShape->getChildShape(shi), shapes);
            }
        }
    }
}

void ChCollisionModelBullet::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChCollisionModelBullet>();
    // serialize parent class
    ChCollisionModel::ArchiveOUT(marchive);

    // serialize all member data:
    std::vector<char> serialized(0);

    if (this->bt_collision_object->getCollisionShape()) {
        // serialize all member data:
        int maxSerializeBufferSize = 1024 * 1024 * 5;  // ***TO DO*** make this more efficient
        btDefaultSerializer* serializer = new btDefaultSerializer(maxSerializeBufferSize);

        serializer->startSerialization();

        this->bt_collision_object->getCollisionShape()->serializeSingleShape(serializer);

        serializer->finishSerialization();

        serialized.resize(serializer->getCurrentBufferSize());
        for (int mpt = 0; mpt < serializer->getCurrentBufferSize(); mpt++)
            serialized[mpt] = (char)(*(serializer->getBufferPointer() + mpt));

        delete serializer;
    }

    marchive << CHNVP(serialized, "bullet_serialized_bytes");
}

void ChCollisionModelBullet::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChCollisionModelBullet>();
    // deserialize parent class
    ChCollisionModel::ArchiveIN(marchive);

    // stream in all member data:

    this->ClearModel();  // remove shape

    std::vector<char> serialized;

    marchive >> CHNVP(serialized, "bullet_serialized_bytes");

    if (serialized.size()) {
        // convert to char array (maybe just cast from std::vector data ptr might be sufficient)
        char* mbuffer = new char[serialized.size()];
        for (int mpt = 0; mpt < serialized.size(); mpt++)
            mbuffer[mpt] = serialized[mpt];

        btBulletWorldImporter import(0);  // don't store info into the world
        import.setVerboseMode(false);

        if (import.loadFileFromMemory(mbuffer, (int)serialized.size())) {
            int numShape = import.getNumCollisionShapes();
            if (numShape) {
                btCollisionShape* mshape = import.getCollisionShapeByIndex(0);
                if (mshape)
                    bt_collision_object->setCollisionShape(mshape);

                // Update the list of shared pointers to newly created shapes, so that
                // the deletion will be automatic
                __recurse_add_newcollshapes(mshape, this->shapes);
            }
        }

        delete[] mbuffer;
    }
}
*/

}  // end namespace collision
}  // end namespace chrono
