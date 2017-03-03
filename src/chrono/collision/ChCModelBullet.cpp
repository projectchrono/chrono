//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#include <memory>
#include <array>

#include "chrono/collision/ChCCollisionSystemBullet.h"
#include "chrono/collision/ChCCollisionUtils.h"
#include "chrono/collision/ChCConvexDecomposition.h"
#include "chrono/collision/ChCModelBullet.h"
#include "chrono/collision/bullet/BulletCollision/CollisionShapes/bt2DShape.h"
#include "chrono/collision/bullet/BulletCollision/CollisionShapes/btBarrelShape.h"
#include "chrono/collision/bullet/BulletCollision/CollisionShapes/btCEtriangleShape.h"
#include "chrono/collision/bullet/BulletWorldImporter/btBulletWorldImporter.h"
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

// Register into the object factory, to enable run-time
// dynamic creation and persistence
CH_FACTORY_REGISTER(ChModelBullet)


ChModelBullet::ChModelBullet() {
    bt_collision_object = new btCollisionObject;
    bt_collision_object->setCollisionShape(0);
    bt_collision_object->setUserPointer((void*)this);

    shapes.clear();
}

ChModelBullet::~ChModelBullet() {
    // ClearModel(); not possible, would call GetPhysicsItem() that is pure virtual, enough to use instead..
    shapes.clear();

    bt_collision_object->setCollisionShape(0);

    if (bt_collision_object)
        delete bt_collision_object;
    bt_collision_object = 0;
}

int ChModelBullet::ClearModel() {
    // delete previously added shapes, if collision shape(s) used by collision object
    if (shapes.size() > 0) {
        // deletes shared pointers, so also deletes shapes if uniquely referenced
        shapes.clear();

        // tell to the parent collision system to remove this from collision system,
        // if still connected to a physical system
        if (GetContactable())
            if (GetPhysicsItem())
                if (GetPhysicsItem()->GetSystem())
                    if (GetPhysicsItem()->GetCollide())
                        GetPhysicsItem()->GetSystem()->GetCollisionSystem()->Remove(this);

        // at the end, no collision shape
        bt_collision_object->setCollisionShape(0);
    }

    return 1;
}

int ChModelBullet::BuildModel() {

    // insert again (we assume it was removed by ClearModel!!!)
    if (GetContactable())
        if (GetPhysicsItem())
            if (GetPhysicsItem()->GetSystem())
                if (GetPhysicsItem()->GetCollide())
                    GetPhysicsItem()->GetSystem()->GetCollisionSystem()->Add(this);

    return 1;
}

static btVector3 ChVectToBullet(const ChVector<>& pos) {
    return btVector3((btScalar)pos.x(), (btScalar)pos.y(), (btScalar)pos.z());
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

void ChModelBullet::_injectShape(const ChVector<>& pos, const ChMatrix33<>& rot, btCollisionShape* mshape) {
    bool centered = (pos.IsNull() && rot.IsIdentity());
    
    // This is needed so later one can access ChModelBullet::GetSafeMargin and ChModelBullet::GetEnvelope
    mshape->setUserPointer(this);
    
    // start_vector = ||    -- description is still empty
    if (shapes.size() == 0) {
        if (centered) {
            shapes.push_back(std::shared_ptr<btCollisionShape>(mshape));
            bt_collision_object->setCollisionShape(mshape);
            // end_vector=  | centered shape |
            return;
        } else {
            btCompoundShape* mcompound = new btCompoundShape(true);
            shapes.push_back(std::shared_ptr<btCollisionShape>(mcompound));
            shapes.push_back(std::shared_ptr<btCollisionShape>(mshape));
            bt_collision_object->setCollisionShape(mcompound);
            btTransform mtransform;
            ChPosMatrToBullet(pos, rot, mtransform);
            mcompound->addChildShape(mtransform, mshape);
            // vector=  | compound | not centered shape |
            return;
        }
    }
    // start_vector = | centered shape |    ----just a single centered shape was added
    if (shapes.size() == 1) {
        btTransform mtransform;
        shapes.push_back(shapes[0]);
        shapes.push_back(std::shared_ptr<btCollisionShape>(mshape));
        btCompoundShape* mcompound = new btCompoundShape(true);
        shapes[0] = std::shared_ptr<btCollisionShape>(mcompound);
        bt_collision_object->setCollisionShape(mcompound);
        mtransform.setIdentity();
        mcompound->addChildShape(mtransform, shapes[1].get());
        ChPosMatrToBullet(pos, rot, mtransform);
        mcompound->addChildShape(mtransform, shapes[2].get());
        // vector=  | compound | old centered shape | new shape | ...
        return;
    }
    // vector=  | compound | old | old.. |   ----already working with compounds..
    if (shapes.size() > 1) {
        btTransform mtransform;
        shapes.push_back(std::shared_ptr<btCollisionShape>(mshape));
        ChPosMatrToBullet(pos, rot, mtransform);
        btCollisionShape* mcom = shapes[0].get();
        ((btCompoundShape*)mcom)->addChildShape(mtransform, mshape);
        // vector=  | compound | old | old.. | new shape | ...
        return;
    }
}

bool ChModelBullet::AddSphere(double radius, const ChVector<>& pos) {
    // adjust default inward 'safe' margin (always as radius)
    this->SetSafeMargin(radius);

    btSphereShape* mshape = new btSphereShape((btScalar)(radius + this->GetEnvelope()));

    mshape->setMargin((btScalar) this->GetSuggestedFullMargin());

    _injectShape(pos, ChMatrix33<>(1), mshape);

    return true;
}

bool ChModelBullet::AddEllipsoid(double rx, double ry, double rz, const ChVector<>& pos, const ChMatrix33<>& rot) {
    btScalar rad = 1.0;
    btVector3 spos(0, 0, 0);
    double arx = rx + this->GetEnvelope();
    double ary = ry + this->GetEnvelope();
    double arz = rz + this->GetEnvelope();
    double mmargin = GetSuggestedFullMargin();
    btMultiSphereShape* mshape = new btMultiSphereShape(&spos, &rad, 1);
    mshape->setLocalScaling(btVector3((btScalar)arx, (btScalar)ary, (btScalar)arz));

    mshape->setMargin((btScalar)ChMin(mmargin, 0.9 * ChMin(ChMin(arx, ary), arz)));

    _injectShape(pos, rot, mshape);

    return true;
}

bool ChModelBullet::AddBox(double hx, double hy, double hz, const ChVector<>& pos, const ChMatrix33<>& rot) {
    // adjust default inward margin (if object too thin)
    this->SetSafeMargin(ChMin(this->GetSafeMargin(), 0.2 * ChMin(ChMin(hx, hy), hz)));

    btScalar ahx = (btScalar)(hx + this->GetEnvelope());
    btScalar ahy = (btScalar)(hy + this->GetEnvelope());
    btScalar ahz = (btScalar)(hz + this->GetEnvelope());
    btBoxShape* mshape = new btBoxShape(btVector3(ahx, ahy, ahz));

    mshape->setMargin((btScalar) this->GetSuggestedFullMargin());

    _injectShape(pos, rot, mshape);

    return true;
}

/// Add a cylinder to this model (default axis on Y direction), for collision purposes
bool ChModelBullet::AddCylinder(double rx, double rz, double hy, const ChVector<>& pos, const ChMatrix33<>& rot) {
    // adjust default inward margin (if object too thin)
    this->SetSafeMargin(ChMin(this->GetSafeMargin(), 0.2 * ChMin(ChMin(rx, rz), 0.5 * hy)));

    btScalar arx = (btScalar)(rx + this->GetEnvelope());
    btScalar arz = (btScalar)(rz + this->GetEnvelope());
    btScalar ahy = (btScalar)(hy + this->GetEnvelope());
    btCylinderShape* mshape = new btCylinderShape(btVector3(arx, ahy, arz));

    mshape->setMargin((btScalar) this->GetSuggestedFullMargin());

    _injectShape(pos, rot, mshape);

    return true;
}

bool ChModelBullet::AddBarrel(double Y_low,
                              double Y_high,
                              double R_vert,
                              double R_hor,
                              double R_offset,
                              const ChVector<>& pos,
                              const ChMatrix33<>& rot) {
    // adjust default inward margin (if object too thin)
    this->SetSafeMargin(ChMin(this->GetSafeMargin(), 0.15 * ChMin(ChMin(R_vert, R_hor), Y_high - Y_low)));

    btBarrelShape* mshape = new btBarrelShape(
        (btScalar)(Y_low - this->model_envelope), (btScalar)(Y_high + this->model_envelope),
        (btScalar)(R_vert + this->model_envelope), (btScalar)(R_hor + this->model_envelope), (btScalar)(R_offset));

    mshape->setMargin((btScalar) this->GetSuggestedFullMargin());

    _injectShape(pos, rot, mshape);

    return true;
}


bool ChModelBullet::Add2Dpath(geometry::ChLinePath& mpath,
                           const ChVector<>& pos,
                           const ChMatrix33<>& rot,
                           const double mthickness)
{
    // The envelope is not used in this type of collision primitive. 
    this->SetEnvelope(0); 

    //if (!mpath.Get_closed()) 
    //    throw ChException("Error! Add2Dpath requires a CLOSED ChLinePath!");

    for (size_t i = 0; i < mpath.GetSubLinesCount(); ++i) {
        if (auto msegment = std::dynamic_pointer_cast<geometry::ChLineSegment>(mpath.GetSubLineN(i))) {
            if ((msegment->pA.z() != 0) || (msegment->pB.z() != 0))
                throw ChException("Error! Add2Dpath: a sub segment of the ChLinePath had non-zero z coordinate!");

            btVector3 pa((btScalar)msegment->pA.x(), (btScalar)msegment->pA.y(), (btScalar)0);
            btVector3 pb((btScalar)msegment->pB.x(), (btScalar)msegment->pB.y(), (btScalar)0);
            bt2DsegmentShape* mshape = new bt2DsegmentShape(pa, pb, (btScalar)mthickness);

            mshape->setMargin((btScalar) this->GetSuggestedFullMargin());
            _injectShape(pos, rot, mshape);
        } else if (auto marc = std::dynamic_pointer_cast<geometry::ChLineArc>(mpath.GetSubLineN(i))) {
            if ((marc->origin.pos.z() != 0))
                throw ChException(
                    "Error! Add2Dpath: a sub arc of the ChLinePath had center with non-zero z coordinate! It must be "
                    "flat on xy.");
            double mangle1 = marc->angle1;
            double mangle2 = marc->angle2;
            if (mangle1 - mangle2 == CH_C_2PI)
                mangle1 -= 1e-7;
            bt2DarcShape* mshape =
                new bt2DarcShape((btScalar)marc->origin.pos.x(), (btScalar)marc->origin.pos.y(), (btScalar)marc->radius,
                                 (btScalar)mangle1, (btScalar)mangle2, marc->counterclockwise, (btScalar)mthickness);

            mshape->setMargin((btScalar) this->GetSuggestedFullMargin());
            _injectShape(pos, rot, mshape);
        } else {
            throw ChException("Error! Add2Dpath: ChLinePath must contain only ChLineArc and/or ChLineSegment.");
        }

        size_t i_prev = i;
        size_t i_next = i + 1;
        if (i_next >= mpath.GetSubLinesCount())
            if ((mpath.GetEndA() - mpath.GetEndB()).Length() <
                1e-9)        // can't use Get_closed() that is user preference via Set_closed()
                i_next = 0;  // closed path
        if (i_next < mpath.GetSubLinesCount()) {
            std::shared_ptr<geometry::ChLine> mline_prev = mpath.GetSubLineN(i_prev);
            std::shared_ptr<geometry::ChLine> mline_next = mpath.GetSubLineN(i_next);
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
                bt2DarcShape* mshape =
                    new bt2DarcShape((btScalar)pos_prev.x(), (btScalar)pos_prev.y(), (btScalar)0, (btScalar)mangle1,
                                     (btScalar)mangle2, false, (btScalar)mthickness);

                mshape->setMargin((btScalar) this->GetSuggestedFullMargin());
                _injectShape(pos, rot, mshape);
                // GetLog() << "convex corner between " << i_next << " and " << i_next << " w.angles: " << mangle1 << "
                // " << mangle2 << "\n";
            } else {
                // GetLog() << "concave corner between " << i_next << " and " << i_next << "\n";
            }
        }
    }

    return true;
}


bool ChModelBullet::AddPoint(double radius, const ChVector<>& pos) {
    // adjust default inward 'safe' margin (always as radius)
    this->SetSafeMargin(radius);

    btPointShape* mshape = new btPointShape((btScalar)(radius + this->GetEnvelope()));

    mshape->setMargin((btScalar) this->GetSuggestedFullMargin());

    _injectShape(pos, ChMatrix33<>(1), mshape);

    return true;
}


bool ChModelBullet::AddTriangleProxy(ChVector<>* p1,                ///< points to vertex1 coords
                                    ChVector<>* p2,                 ///< points to vertex2 coords
                                    ChVector<>* p3,                 ///< points to vertex3 coords
                                    ChVector<>* ep1,                ///< points to neighbouring vertex at edge1 if any
                                    ChVector<>* ep2,                ///< points to neighbouring vertex at edge1 if any
                                    ChVector<>* ep3,                ///< points to neighbouring vertex at edge1 if any
                                    bool mowns_vertex_1,            ///< vertex is owned by this triangle (otherwise, owned by neighbour)
                                    bool mowns_vertex_2,
                                    bool mowns_vertex_3,
                                    bool mowns_edge_1,              ///< edge is owned by this triangle (otherwise, owned by neighbour)
                                    bool mowns_edge_2,
                                    bool mowns_edge_3,
                                    double msphereswept_rad       ///< sphere swept triangle ('fat' triangle, improves robustness)
                                    ) {
    // adjust default inward 'safe' margin (always as radius)
    this->SetSafeMargin(msphereswept_rad);

    btCEtriangleShape* mshape = new btCEtriangleShape(p1,p2,p3,ep1,ep2,ep3,
        mowns_vertex_1, mowns_vertex_2, mowns_vertex_3, 
        mowns_edge_1, mowns_edge_2, mowns_edge_3, msphereswept_rad);

    mshape->setMargin((btScalar)  this->GetSuggestedFullMargin()); // this->GetSafeMargin());  // not this->GetSuggestedFullMargin() given the way that btCEtriangleShape works.
    
    _injectShape(VNULL, ChMatrix33<>(1), mshape);

    return true;
}


bool ChModelBullet::AddConvexHull(std::vector<ChVector<double> >& pointlist,
                                  const ChVector<>& pos,
                                  const ChMatrix33<>& rot) {

    // adjust default inward margin (if object too thin)
    ChVector<> aabbMax(-1e9,-1e9,-1e9);
    ChVector<> aabbMin(1e9,1e9,1e9);
    for (size_t i=0; i< pointlist.size(); ++i) {
        aabbMax.x()=ChMax(aabbMax.x(),pointlist[i].x());
        aabbMax.y()=ChMax(aabbMax.y(),pointlist[i].y());
        aabbMax.z()=ChMax(aabbMax.z(),pointlist[i].z());
        aabbMin.x()=ChMin(aabbMin.x(),pointlist[i].x());
        aabbMin.y()=ChMin(aabbMin.y(),pointlist[i].y());
        aabbMin.z()=ChMin(aabbMin.z(),pointlist[i].z());
    }
    ChVector<>aabbsize = aabbMax -aabbMin;
    double approx_chord = ChMin( ChMin(aabbsize.x(), aabbsize.y()), aabbsize.z() );
    // override the inward margin if larger than 0.2 chord:
    this->SetSafeMargin((btScalar)ChMin(this->GetSafeMargin(), approx_chord*0.2));


    btConvexHullShape* mshape = new btConvexHullShape;

    // shrink the convex hull by GetSafeMargin()
    collision::ChConvexHullLibraryWrapper lh;
    geometry::ChTriangleMeshConnected mmesh;
    lh.ComputeHull(pointlist, mmesh);
    mmesh.MakeOffset(-this->GetSafeMargin());

    for (unsigned int i = 0; i < mmesh.m_vertices.size(); i++) {
        mshape->addPoint(btVector3((btScalar)mmesh.m_vertices[i].x(), (btScalar)mmesh.m_vertices[i].y(), (btScalar)mmesh.m_vertices[i].z()));
    }

    mshape->setMargin((btScalar) this->GetSuggestedFullMargin());
    mshape->recalcLocalAabb();

    _injectShape(pos, rot, mshape);

    return true;
}

// These classes inherits the Bullet triangle mesh, but adds just a single feature:
// when this shape is deleted, also the referenced triangle mesh interface is deleted.
// Hence, when a btBvhTriangleMeshShape_handlemesh is added to the list of shapes of this ChModelBullet,
// there's no need to remember to delete the mesh interface because it dies with the model, when shapes are deleted.
// This is just to avoid adding a pointer to a triangle interface in all collision models, when not needed.

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
          minterface(meshInterface){// this->setLocalScaling(btVector3(1.f,1.f,1.f));
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
          minterface(meshInterface){// this->setLocalScaling(btVector3(1.f,1.f,1.f));
          };

    virtual ~btGImpactMeshShape_handlemesh() {
        if (minterface)
            delete minterface;
        minterface = 0;  // also delete the mesh interface
    }
};

/// Add a triangle mesh to this model
bool ChModelBullet::AddTriangleMesh(const geometry::ChTriangleMesh& trimesh,
                                    bool is_static,
                                    bool is_convex,
                                    const ChVector<>& pos,
                                    const ChMatrix33<>& rot,
                                    double sphereswept_thickness) {
    if (!trimesh.getNumTriangles())
        return false;

    if (geometry::ChTriangleMeshConnected* mesh = dynamic_cast<geometry::ChTriangleMeshConnected*> ( const_cast<geometry::ChTriangleMesh*>(&trimesh))) {

        std::vector<std::array<int,4>> trimap;
        mesh->ComputeNeighbouringTriangleMap(trimap);
        
        std::map<std::pair<int,int>, std::pair<int, int>> winged_edges;
        mesh->ComputeWingedEdges(winged_edges, true);

        std::vector<bool> added_vertexes (mesh->m_vertices.size());
        
        // iterate on triangles
        for (int it = 0; it < mesh->m_face_v_indices.size(); ++it) {
            // edges = pairs of vertexes indexes
            std::pair<int, int> medgeA(mesh->m_face_v_indices[it].x(), mesh->m_face_v_indices[it].y());
            std::pair<int, int> medgeB(mesh->m_face_v_indices[it].y(), mesh->m_face_v_indices[it].z());
            std::pair<int, int> medgeC(mesh->m_face_v_indices[it].z(), mesh->m_face_v_indices[it].x());
            // vertex indexes in edges: always in increasing order to avoid ambiguous duplicated edges
            if (medgeA.first>medgeA.second) 
                medgeA = std::pair<int, int>(medgeA.second, medgeA.first);
            if (medgeB.first>medgeB.second) 
                medgeB = std::pair<int, int>(medgeB.second, medgeB.first);
            if (medgeC.first>medgeC.second) 
                medgeC = std::pair<int, int>(medgeC.second, medgeC.first);  
            auto wingedgeA = winged_edges.find(medgeA);
            auto wingedgeB = winged_edges.find(medgeB);
            auto wingedgeC = winged_edges.find(medgeC);

            int i_wingvertex_A = -1;
            int i_wingvertex_B = -1;
            int i_wingvertex_C = -1;

            if (trimap[it][1] != -1) {
                i_wingvertex_A = mesh->m_face_v_indices[trimap[it][1]].x();
                if (mesh->m_face_v_indices[trimap[it][1]].y() != wingedgeA->first.first && mesh->m_face_v_indices[trimap[it][1]].y() != wingedgeA->first.second)
                    i_wingvertex_A = mesh->m_face_v_indices[trimap[it][1]].y();
                if (mesh->m_face_v_indices[trimap[it][1]].z() != wingedgeA->first.first && mesh->m_face_v_indices[trimap[it][1]].z() != wingedgeA->first.second)
                    i_wingvertex_A = mesh->m_face_v_indices[trimap[it][1]].z();
            }

            if (trimap[it][2] != -1) {
                i_wingvertex_B = mesh->m_face_v_indices[trimap[it][2]].x();
                if (mesh->m_face_v_indices[trimap[it][2]].y() != wingedgeB->first.first && mesh->m_face_v_indices[trimap[it][2]].y() != wingedgeB->first.second)
                    i_wingvertex_B = mesh->m_face_v_indices[trimap[it][2]].y();
                if (mesh->m_face_v_indices[trimap[it][2]].z() != wingedgeB->first.first && mesh->m_face_v_indices[trimap[it][2]].z() != wingedgeB->first.second)
                    i_wingvertex_B = mesh->m_face_v_indices[trimap[it][2]].z();
            }

            if (trimap[it][3] != -1) {
                i_wingvertex_C = mesh->m_face_v_indices[trimap[it][3]].x();
                if (mesh->m_face_v_indices[trimap[it][3]].y() != wingedgeC->first.first && mesh->m_face_v_indices[trimap[it][3]].y() != wingedgeC->first.second)
                    i_wingvertex_C = mesh->m_face_v_indices[trimap[it][3]].y();
                if (mesh->m_face_v_indices[trimap[it][3]].z() != wingedgeC->first.first && mesh->m_face_v_indices[trimap[it][3]].z() != wingedgeC->first.second)
                    i_wingvertex_C = mesh->m_face_v_indices[trimap[it][3]].z();
            }

            this->AddTriangleProxy(&mesh->m_vertices[mesh->m_face_v_indices[it].x()], 
                                   &mesh->m_vertices[mesh->m_face_v_indices[it].y()],
                                   &mesh->m_vertices[mesh->m_face_v_indices[it].z()],
                                   // if no wing vertex (ie. 'free' edge), point to opposite vertex, ie vertex in triangle not belonging to edge
                                   wingedgeA->second.second != -1 ? &mesh->m_vertices[i_wingvertex_A] : &mesh->m_vertices[mesh->m_face_v_indices[it].z()], 
                                   wingedgeB->second.second != -1 ? &mesh->m_vertices[i_wingvertex_B] : &mesh->m_vertices[mesh->m_face_v_indices[it].x()],
                                   wingedgeC->second.second != -1 ? &mesh->m_vertices[i_wingvertex_C] : &mesh->m_vertices[mesh->m_face_v_indices[it].y()],
                                   !added_vertexes[mesh->m_face_v_indices[it].x()],
                                   !added_vertexes[mesh->m_face_v_indices[it].y()],
                                   !added_vertexes[mesh->m_face_v_indices[it].z()],
                                   // are edges owned by this triangle? (if not, they belong to a neighbouring triangle)
                                   wingedgeA->second.first != -1,
                                   wingedgeB->second.first != -1,
                                   wingedgeC->second.first != -1,
                                   sphereswept_thickness);
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
    for (int i = 0; i < trimesh.getNumTriangles(); i++) {
        // bulletMesh->m_weldingThreshold = ...
        bulletMesh->addTriangle(ChVectToBullet(trimesh.getTriangle(i).p1), ChVectToBullet(trimesh.getTriangle(i).p2),
                                ChVectToBullet(trimesh.getTriangle(i).p3),
                                true);  // try to remove duplicate vertices
    }

    if (is_static) {
        // Here a static btBvhTriangleMeshShape should suffice, but looks like that the btGImpactMeshShape works
        // better..
        btCollisionShape* pShape = (btBvhTriangleMeshShape*)new btBvhTriangleMeshShape_handlemesh(bulletMesh);
        pShape->setMargin((btScalar) this->GetSafeMargin());
        // ((btBvhTriangleMeshShape*)pShape)->refitTree();
        // btCollisionShape* pShape = new btGImpactMeshShape_handlemesh(bulletMesh);
        // pShape->setMargin((btScalar) this->GetSafeMargin() );
        //((btGImpactMeshShape_handlemesh*)pShape)->updateBound();
        _injectShape(pos, rot, pShape);
    } else {
        if (is_convex) {
            btCollisionShape* pShape = (btConvexTriangleMeshShape*)new btConvexTriangleMeshShape_handlemesh(bulletMesh);
            pShape->setMargin((btScalar) this->GetEnvelope());
            _injectShape(pos, rot, pShape);
        } else {
            // Note: currently there's no 'perfect' convex decomposition method,
            // so here the code is a bit experimental...

            /*
            // ----- use this? (using GImpact collision method without decomposition) :
            this->AddTriangleMeshConcave(trimesh,pos,rot);
            */

            // ----- ..or use this? (using the JR convex decomposition) :
            ChConvexDecompositionJR mydecompositionJR;
            mydecompositionJR.AddTriangleMesh(trimesh);
            mydecompositionJR.SetParameters(0,      // skin width
                                            9, 64,  // depht, max vertices in hull
                                            5,      // concavity percent
                                            5,      // merge treshold percent
                                            5,      // split threshold percent
                                            true,   // use initial island generation
                                            false   // use island generation (unsupported-disabled)
                                            );
            mydecompositionJR.ComputeConvexDecomposition();
            GetLog() << " found n.hulls=" << mydecompositionJR.GetHullCount() << "\n";
            this->AddTriangleMeshConcaveDecomposed(mydecompositionJR, pos, rot);

            /*
            // ----- ..or use this? (using the HACD convex decomposition) :
            ChConvexDecompositionHACD mydecompositionHACD;
            mydecompositionHACD.AddTriangleMesh(trimesh);
            mydecompositionHACD.SetParameters          (2, // clusters
                                                        0, // no decimation
                                                        0.0, // small cluster threshold
                                                        false, // add faces points
                                                        false, // add extra dist points
                                                        100.0, // max concavity
                                                        30, // cc connect dist
                                                        0.0, // volume weight beta
                                                        0.0, // compacity alpha
                                                        50 // vertices per cc
                                                        );
            mydecompositionHACD.ComputeConvexDecomposition();
            this->AddTriangleMeshConcaveDecomposed(mydecompositionHACD, pos, rot);
            */
        }
    }

    return true;
}

bool ChModelBullet::AddTriangleMeshConcave(const geometry::ChTriangleMesh& trimesh,
                                           const ChVector<>& pos,
                                           const ChMatrix33<>& rot) {
    if (!trimesh.getNumTriangles())
        return false;

    btTriangleMesh* bulletMesh = new btTriangleMesh;
    for (int i = 0; i < trimesh.getNumTriangles(); i++) {
        // bulletMesh->m_weldingThreshold = ...
        bulletMesh->addTriangle(ChVectToBullet(trimesh.getTriangle(i).p1), ChVectToBullet(trimesh.getTriangle(i).p2),
                                ChVectToBullet(trimesh.getTriangle(i).p3),
                                true);  // try to remove duplicate vertices
    }

    // Use the GImpact custom mesh-mesh algorithm
    btCollisionShape* pShape = (btGImpactMeshShape*)new btGImpactMeshShape_handlemesh(bulletMesh);
    pShape->setMargin((btScalar) this->GetEnvelope());
    this->SetSafeMargin(0);

    ((btGImpactMeshShape_handlemesh*)pShape)->updateBound();
    _injectShape(pos, rot, pShape);

    return true;
}

bool ChModelBullet::AddTriangleMeshConcaveDecomposed(ChConvexDecomposition& mydecomposition,
                                                     const ChVector<>& pos,
                                                     const ChMatrix33<>& rot) {
    // note: since the convex hulls are ot shrunk, the safe margin will be set to zero (must be set before adding them)
    this->SetSafeMargin(0);

    for (unsigned int j = 0; j < mydecomposition.GetHullCount(); j++) {
        std::vector<ChVector<double> > ptlist;
        mydecomposition.GetConvexHullResult(j, ptlist);

        if (ptlist.size())
            this->AddConvexHull(ptlist, pos, rot);
    }

    return true;
}

bool ChModelBullet::AddCopyOfAnotherModel(ChCollisionModel* another) {
    // this->ClearModel();
    this->shapes.clear();  // this will also delete owned shapes, if any, thank to shared pointers in 'shapes' vector

    this->SetSafeMargin(another->GetSafeMargin());
    this->SetEnvelope(another->GetEnvelope());

    this->bt_collision_object->setCollisionShape(((ChModelBullet*)another)->GetBulletModel()->getCollisionShape());
    this->shapes = ((ChModelBullet*)another)->shapes;

    return true;
}

void ChModelBullet::SetFamily(int mfamily) {
    ChCollisionModel::SetFamily(mfamily);
    onFamilyChange();
}

int ChModelBullet::GetFamily() {
    int fam;
    if (!bt_collision_object->getBroadphaseHandle())
        return -1;
    for (fam = 0; fam < 16; fam++)
        if (((short int)0x1 << fam) & bt_collision_object->getBroadphaseHandle()->m_collisionFilterGroup)
            return fam;
    return fam;
}

void ChModelBullet::SetFamilyMaskNoCollisionWithFamily(int mfamily) {
    ChCollisionModel::SetFamilyMaskNoCollisionWithFamily(mfamily);
    onFamilyChange();
}

void ChModelBullet::SetFamilyMaskDoCollisionWithFamily(int mfamily) {
    ChCollisionModel::SetFamilyMaskDoCollisionWithFamily(mfamily);
    onFamilyChange();
}

bool ChModelBullet::GetFamilyMaskDoesCollisionWithFamily(int mfamily) {
    assert(mfamily < 16);
    if (!bt_collision_object->getBroadphaseHandle())
        return false;
    short familyflag = (1 << mfamily);
    return (bt_collision_object->getBroadphaseHandle()->m_collisionFilterMask & familyflag) != 0;
}

void ChModelBullet::SetFamilyGroup(short int group) {
    ChCollisionModel::SetFamilyGroup(group);
    onFamilyChange();
}

void ChModelBullet::SetFamilyMask(short int mask) {
    ChCollisionModel::SetFamilyMask(mask);
    onFamilyChange();
}

void ChModelBullet::onFamilyChange() {
    if (!bt_collision_object->getBroadphaseHandle())
        return;

    this->SyncPosition();

    // Trick to avoid troubles if setting mask or family when model is already overlapping to some other model
    auto mcosys = this->GetPhysicsItem()->GetSystem()->GetCollisionSystem();
    mcosys->Remove(this);

    auto mcs = std::static_pointer_cast<ChCollisionSystemBullet>(mcosys);
    mcs->GetBulletCollisionWorld()->addCollisionObject(bt_collision_object, family_group, family_mask);
}

void ChModelBullet::GetAABB(ChVector<>& bbmin, ChVector<>& bbmax) const {
    btVector3 btmin;
    btVector3 btmax;
    if (bt_collision_object->getCollisionShape())
        bt_collision_object->getCollisionShape()->getAabb(bt_collision_object->getWorldTransform(), btmin, btmax);
    bbmin.Set(btmin.x(), btmin.y(), btmin.z());
    bbmax.Set(btmax.x(), btmax.y(), btmax.z());
}

void __recurse_add_newcollshapes(btCollisionShape* ashape,
    std::vector<std::shared_ptr<btCollisionShape> >& shapes) {
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



void ChModelBullet::SyncPosition()
{
    ChCoordsys<> mcsys = this->mcontactable->GetCsysForCollisionModel();

    bt_collision_object->getWorldTransform().setOrigin(btVector3(
        (btScalar)mcsys.pos.x(), (btScalar)mcsys.pos.y(), (btScalar)mcsys.pos.z()));
    const ChMatrix33<>& rA(mcsys.rot);
    btMatrix3x3 basisA((btScalar)rA(0, 0), (btScalar)rA(0, 1), (btScalar)rA(0, 2), (btScalar)rA(1, 0),
                       (btScalar)rA(1, 1), (btScalar)rA(1, 2), (btScalar)rA(2, 0), (btScalar)rA(2, 1),
                       (btScalar)rA(2, 2));
    bt_collision_object->getWorldTransform().setBasis(basisA);
}


bool ChModelBullet::SetSphereRadius(double coll_radius, double out_envelope) {
    if (this->shapes.size() != 1)
        return false;
    if (btSphereShape* mshape = dynamic_cast<btSphereShape*>(this->shapes[0].get())) {
        this->SetSafeMargin(coll_radius);
        this->SetEnvelope(out_envelope);
        mshape->setUnscaledRadius((btScalar)(coll_radius + out_envelope));
        // mshape->setMargin((btScalar) (coll_radius+out_envelope));
    } else
        return false;
    return true;
}

void ChModelBullet::ArchiveOUT(ChArchiveOut& marchive)
{
    // version number
    marchive.VersionWrite<ChModelBullet>();
    // serialize parent class
    ChCollisionModel::ArchiveOUT(marchive);

    // serialize all member data:
    std::vector< char > serialized(0);

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

void ChModelBullet::ArchiveIN(ChArchiveIn& marchive) 
{
    // version number
    int version = marchive.VersionRead<ChModelBullet>();
    // deserialize parent class
    ChCollisionModel::ArchiveIN(marchive);

    // stream in all member data:

    this->ClearModel();  // remove shape

    std::vector<char> serialized;

    marchive >> CHNVP(serialized, "bullet_serialized_bytes");

    if(serialized.size()) {
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

                // Update the list of sharedpointers to newly created shapes, so that
                // the deletion will be automatic
                __recurse_add_newcollshapes(mshape, this->shapes);
            }
        }

        delete[] mbuffer;
    }
}

//***OBSOLETE***
/*
void ChModelBullet::StreamIN(ChStreamInBinary& mstream) {
    // class version number
    int version = mstream.VersionRead();

    // parent class deserialize
    //ChCollisionModel::StreamIN(mstream);

    // deserialize custom data:

    this->ClearModel();  // remove shape

    int buffer_len = 0;

    mstream >> buffer_len;

    char* mbuffer = new char[buffer_len];

    for (int mpt = 0; mpt < buffer_len; mpt++)
        mstream >> mbuffer[mpt];

    btBulletWorldImporter import(0);  // don't store info into the world
    import.setVerboseMode(false);

    if (import.loadFileFromMemory(mbuffer, buffer_len)) {
        int numShape = import.getNumCollisionShapes();
        if (numShape) {
            btCollisionShape* mshape = import.getCollisionShapeByIndex(0);
            if (mshape)
                bt_collision_object->setCollisionShape(mshape);

            // Update the list of sharedpointers to newly created shapes, so that
            // the deletion will be automatic
            __recurse_add_newcollshapes(mshape, this->shapes);
        }
    }

    delete[] mbuffer;
}
*/
//***OBSOLETE***
/*
void ChModelBullet::StreamOUT(ChStreamOutBinary& mstream) {
    // class version number
    mstream.VersionWrite(1);

    // parent class serialize
    //ChCollisionModel::StreamOUT(mstream);

    // serialize custom data:

    int maxSerializeBufferSize = 1024 * 1024 * 5;  // ***TO DO*** make this more efficient
    btDefaultSerializer* serializer = new btDefaultSerializer(maxSerializeBufferSize);

    serializer->startSerialization();

    this->bt_collision_object->getCollisionShape()->serializeSingleShape(serializer);

    serializer->finishSerialization();

    mstream << serializer->getCurrentBufferSize();

    for (int mpt = 0; mpt < serializer->getCurrentBufferSize(); mpt++)
        mstream << (char)(*(serializer->getBufferPointer() + mpt));

    delete serializer;
}
*/

}  // end namespace collision
}  // end namespace chrono
