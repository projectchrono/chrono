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

#include "chrono/collision/bullet/ChCollisionSystemBullet.h"
#include "chrono/collision/bullet/ChCollisionUtilsBullet.h"
#include "chrono/collision/bullet/ChCollisionModelBullet.h"
#include "chrono/collision/bullet/BulletCollision/CollisionShapes/cbt2DShape.h"
#include "chrono/collision/bullet/BulletCollision/CollisionShapes/cbtBarrelShape.h"
#include "chrono/collision/bullet/BulletCollision/CollisionShapes/cbtCEtriangleShape.h"
#include "chrono/collision/bullet/cbtBulletCollisionCommon.h"
#include "chrono/collision/gimpact/GIMPACT/Bullet/cbtGImpactCollisionAlgorithm.h"
#include "chrono/collision/gimpact/GIMPACTUtils/cbtGImpactConvexDecompositionShape.h"
#include "chrono/collision/ChConvexDecomposition.h"
#include "chrono/geometry/ChLineArc.h"
#include "chrono/geometry/ChLineSegment.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChPhysicsItem.h"
#include "chrono/physics/ChSystem.h"

#include "chrono/collision/ChCollisionShapes.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChCollisionModelBullet)
CH_UPCASTING(ChCollisionModelBullet, ChCollisionModelImpl)

// -----------------------------------------------------------------------------

ChCollisionModelBullet::ChCollisionModelBullet(ChCollisionModel* collision_model)
    : ChCollisionModelImpl(collision_model) {
    bt_collision_object = std::unique_ptr<cbtCollisionObject>(new cbtCollisionObject);
    bt_collision_object->setCollisionShape(nullptr);
    bt_collision_object->setUserPointer((void*)this);
}

ChCollisionModelBullet::~ChCollisionModelBullet() {
    m_shapes.clear();
    m_bt_shapes.clear();
}

// -----------------------------------------------------------------------------

// Utility class to convert a Chrono ChVector into a Bullet vector3.
class cbtVector3CH : public cbtVector3 {
  public:
    cbtVector3CH(const chrono::ChVector<>& p) { setValue((cbtScalar)p.x(), (cbtScalar)p.y(), (cbtScalar)p.z()); }
};

// Utility class to convert a Chrono frame into a Bullet transform.
class cbtTransformCH : public cbtTransform {
  public:
    cbtTransformCH(const chrono::ChFrame<>& frame) {
        const auto& p = frame.GetPos();
        const auto& q = frame.GetRot();
        cbtVector3 bt_p((cbtScalar)p.x(), (cbtScalar)p.y(), (cbtScalar)p.z());
        cbtQuaternion bt_q((cbtScalar)q.e1(), (cbtScalar)q.e2(), (cbtScalar)q.e3(), (cbtScalar)q.e0());
        setOrigin(bt_p);
        setRotation(bt_q);
    }
};

// Utility class to convert a Bullet transform to a ChCoordsys.
class ChCoordsysBT : public ChCoordsys<double> {
  public:
    ChCoordsysBT(const cbtTransform& transform) {
        const cbtVector3& p = transform.getOrigin();
        cbtQuaternion q = transform.getRotation();
        pos = ChVector<>((double)p.x(), (double)p.y(), (double)p.z());
        rot = ChQuaternion<>((double)q.w(), (double)q.x(), (double)q.y(), (double)q.z());
    }
};

// -----------------------------------------------------------------------------

cbtScalar ChCollisionModelBullet::GetSuggestedFullMargin() {
    return (cbtScalar)(GetEnvelope() + GetSafeMargin());
}

void ChCollisionModelBullet::Populate() {
    auto envelope = GetEnvelope();
    auto safe_margin = GetSafeMargin();
    auto full_margin = GetSuggestedFullMargin();

    for (const auto& shape_instance : model->GetShapes()) {
        const auto& shape = shape_instance.first;
        const auto& frame = shape_instance.second;

        switch (shape->GetType()) {
            case ChCollisionShape::Type::SPHERE: {
                auto shape_sphere = std::static_pointer_cast<ChCollisionShapeSphere>(shape);
                auto radius = shape_sphere->GetRadius();
                model->SetSafeMargin(radius);
                auto bt_shape = chrono_types::make_shared<cbtSphereShape>((cbtScalar)(radius + envelope));
                bt_shape->setMargin((cbtScalar)full_margin);
                injectShape(shape, bt_shape, frame);
                break;
            }
            case ChCollisionShape::Type::ELLIPSOID: {
                auto shape_ell = std::static_pointer_cast<ChCollisionShapeEllipsoid>(shape);
                auto haxes = shape_ell->GetSemiaxes();
                auto bt_axes = cbtVector3CH(haxes + envelope);
                cbtScalar rad = 1.0;
                cbtVector3 spos(0, 0, 0);
                auto bt_shape = chrono_types::make_shared<cbtMultiSphereShape>(&spos, &rad, 1);
                bt_shape->setLocalScaling(bt_axes);
                bt_shape->setMargin((cbtScalar)ChMin(full_margin, 0.9 * ChMin(ChMin(haxes.x(), haxes.y()), haxes.z())));
                injectShape(shape, bt_shape, frame);
                break;
            }
            case ChCollisionShape::Type::BOX: {
                auto shape_box = std::static_pointer_cast<ChCollisionShapeBox>(shape);
                auto len = shape_box->GetLengths();
                model->SetSafeMargin(ChMin(safe_margin, 0.1 * ChMin(ChMin(len.x(), len.y()), len.z())));
                auto bt_shape = chrono_types::make_shared<cbtBoxShape>(cbtVector3CH(len / 2 + envelope));
                bt_shape->setMargin((cbtScalar)full_margin);
                injectShape(shape, bt_shape, frame);
                break;
            }
            case ChCollisionShape::Type::CYLINDER: {
                auto shape_cylinder = std::static_pointer_cast<ChCollisionShapeCylinder>(shape);
                auto height = shape_cylinder->GetHeight();
                auto radius = shape_cylinder->GetRadius();
                model->SetSafeMargin(ChMin(safe_margin, 0.2 * ChMin(radius, height / 2)));
                ChVector<> size(radius, radius, height / 2);
                auto bt_shape = chrono_types::make_shared<cbtCylinderShapeZ>(cbtVector3CH(size + envelope));
                bt_shape->setMargin((cbtScalar)full_margin);
                injectShape(shape, bt_shape, frame);
                break;
            }
            case ChCollisionShape::Type::CAPSULE: {
                auto shape_capsule = std::static_pointer_cast<ChCollisionShapeCapsule>(shape);
                auto height = shape_capsule->GetHeight();
                auto radius = shape_capsule->GetRadius();
                model->SetSafeMargin(ChMin(safe_margin, 0.2 * ChMin(radius, height / 2)));
                auto bt_shape = chrono_types::make_shared<cbtCapsuleShapeZ>((cbtScalar)(radius + envelope),
                                                                            (cbtScalar)(height + 2 * envelope));
                bt_shape->setMargin((cbtScalar)full_margin);
                injectShape(shape, bt_shape, frame);
                break;
            }
            case ChCollisionShape::Type::CYLSHELL: {
                auto shape_cylshell = std::static_pointer_cast<ChCollisionShapeCylindricalShell>(shape);
                auto height = shape_cylshell->GetHeight();
                auto radius = shape_cylshell->GetRadius();
                model->SetSafeMargin(ChMin(safe_margin, 0.2 * ChMin(radius, height)));
                auto bt_shape = chrono_types::make_shared<cbtCylindricalShellShape>((cbtScalar)(radius + envelope),
                                                                                    (cbtScalar)(height / 2 + envelope));
                bt_shape->setMargin((cbtScalar)full_margin);
                injectShape(shape, bt_shape, frame);
                break;
            }
            case ChCollisionShape::Type::BARREL: {
                auto shape_barrel = std::static_pointer_cast<ChCollisionShapeBarrel>(shape);
                auto Y_low = shape_barrel->Y_low;
                auto Y_high = shape_barrel->Y_high;
                auto axis_vert = shape_barrel->axis_vert;
                auto axis_hor = shape_barrel->axis_hor;
                auto R_offset = shape_barrel->R_offset;
                model->SetSafeMargin(
                    ChMin(safe_margin, 0.15 * ChMin(ChMin(axis_vert / 2, axis_hor / 2), Y_high - Y_low)));
                auto sY_low = (cbtScalar)(Y_low - envelope);
                auto sY_high = (cbtScalar)(Y_high + envelope);
                auto sR_vert = (cbtScalar)(axis_vert / 2 + envelope);
                auto sR_hor = (cbtScalar)(axis_hor / 2 + envelope);
                auto sR_offset = (cbtScalar)(R_offset);
                auto bt_shape = chrono_types::make_shared<cbtBarrelShape>(sY_low, sY_high, sR_vert, sR_hor, sR_offset);
                bt_shape->setMargin((cbtScalar)full_margin);
                injectShape(shape, bt_shape, frame);
                break;
            }
            case ChCollisionShape::Type::POINT: {
                auto shape_point = std::static_pointer_cast<ChCollisionShapePoint>(shape);
                auto radius = shape_point->GetRadius();
                model->SetSafeMargin(radius);
                auto bt_shape = chrono_types::make_shared<cbtPointShape>((cbtScalar)(radius + envelope));
                bt_shape->setMargin((cbtScalar)full_margin);
                injectShape(shape, bt_shape, frame);
                break;
            }
            case ChCollisionShape::Type::PATH2D: {
                auto shape_path = std::static_pointer_cast<ChCollisionShapePath2D>(shape);
                injectPath2D(shape_path, frame);
                break;
            }
            case ChCollisionShape::Type::CONVEXHULL: {
                auto shape_hull = std::static_pointer_cast<ChCollisionShapeConvexHull>(shape);
                injectConvexHull(shape_hull, frame);
                break;
            }
            case ChCollisionShape::Type::TRIANGLEMESH: {
                auto shape_trimesh = std::static_pointer_cast<ChCollisionShapeTriangleMesh>(shape);
                injectTriangleMesh(shape_trimesh, frame);
                break;
            }
            case ChCollisionShape::Type::MESHTRIANGLE: {
                auto shape_triangle = std::static_pointer_cast<ChCollisionShapeMeshTriangle>(shape);
                injectTriangleProxy(shape_triangle);
                break;
            }
            default:
                // Shape type not supported
                break;
        }
    }

    // The number of total collision shapes must match the number of Bullet collision shapes
    assert(m_shapes.size() == m_bt_shapes.size());
}

void ChCollisionModelBullet::injectShape(std::shared_ptr<ChCollisionShape> shape,
                                         std::shared_ptr<cbtCollisionShape> bt_shape,
                                         const ChFrame<>& frame) {
    auto full_margin = GetSuggestedFullMargin();

    bool centered = (frame.GetPos().IsNull() && frame.GetRot().IsIdentity());

    // This is needed so one can later access the model's GetSafeMargin() and GetEnvelope()
    bt_shape->setUserPointer(this);

    if (m_bt_shapes.size() == 0) {  // ----------------------------------- this is the first shape added to the model

        // [in] shape vector: {}
        // [out] shape vector: {centered shape} OR {off-center shape}

        if (centered) {
            bt_collision_object->setCollisionShape(bt_shape.get());
        } else {
            bt_compound_shape = chrono_types::make_shared<cbtCompoundShape>(true);
            bt_compound_shape->setMargin((cbtScalar)full_margin);
            bt_compound_shape->addChildShape(cbtTransformCH(frame), bt_shape.get());
            bt_collision_object->setCollisionShape(bt_compound_shape.get());
        }

    } else if (!bt_compound_shape && m_bt_shapes.size() == 1) {  // ------ the model has only one centered shape

        // [in] shape vector: {centered shape}
        // [out] shape vector: {old centered shape | shape}

        bt_compound_shape = chrono_types::make_shared<cbtCompoundShape>(true);
        bt_compound_shape->setMargin((cbtScalar)full_margin);
        cbtTransform identity;
        identity.setIdentity();
        bt_compound_shape->addChildShape(identity, m_bt_shapes[0].get());
        bt_compound_shape->addChildShape(cbtTransformCH(frame), bt_shape.get());
        bt_collision_object->setCollisionShape(bt_compound_shape.get());

    } else {  // --------------------------------------------------------- already working with a compound

        // [in] shape vector: {old shape | old shape | ...}
        // [out] shape vector: {old shape | old shape | ... | new shape}

        bt_compound_shape->addChildShape(cbtTransformCH(frame), bt_shape.get());
    }

    m_shapes.push_back(shape);
    m_bt_shapes.push_back(bt_shape);
}

void ChCollisionModelBullet::injectPath2D(std::shared_ptr<ChCollisionShapePath2D> shape_path, const ChFrame<>& frame) {
    const auto& path = *shape_path->GetGeometry();
    const auto& material = shape_path->GetMaterial();
    auto thickness = shape_path->GetSRadius();

    auto full_margin = GetSuggestedFullMargin();

    // The envelope is not used in this type of collision primitive.
    model->SetEnvelope(0);

    for (size_t i = 0; i < path.GetSubLinesCount(); ++i) {
        if (auto segment = std::dynamic_pointer_cast<geometry::ChLineSegment>(path.GetSubLineN(i))) {
            if (segment->pA.z() != segment->pB.z())
                throw ChException("Error! injectPath2D: sub segment of ChLinePath not parallel to XY plane!");
            ChVector<> pA(segment->pA.x(), segment->pA.y(), 0);
            ChVector<> pB(segment->pB.x(), segment->pB.y(), 0);
            auto shape_seg = chrono_types::make_shared<ChCollisionShapeSegment2D>(material, *segment, thickness);
            auto bt_shape =
                chrono_types::make_shared<cbt2DsegmentShape>(cbtVector3CH(pA), cbtVector3CH(pB), (cbtScalar)thickness);
            bt_shape->setMargin((cbtScalar)full_margin);
            injectShape(shape_seg, bt_shape, frame);
        } else if (auto arc = std::dynamic_pointer_cast<geometry::ChLineArc>(path.GetSubLineN(i))) {
            if ((arc->origin.rot.e1() != 0) || (arc->origin.rot.e2() != 0))
                throw ChException("Error! injectPath2D: a sub arc of ChLinePath not parallel to XY plane!");

            double angle1 = arc->angle1;
            double angle2 = arc->angle2;
            if (angle1 - angle2 == CH_C_2PI)
                angle1 -= 1e-7;
            auto shape_arc = chrono_types::make_shared<ChCollisionShapeArc2D>(material, *arc, thickness);
            auto bt_shape = chrono_types::make_shared<cbt2DarcShape>(
                (cbtScalar)arc->origin.pos.x(), (cbtScalar)arc->origin.pos.y(), (cbtScalar)arc->radius,
                (cbtScalar)angle1, (cbtScalar)angle2, arc->counterclockwise, (cbtScalar)thickness);
            bt_shape->setMargin((cbtScalar)full_margin);
            injectShape(shape_arc, bt_shape, frame);
        } else {
            throw ChException("Error! injectPath2D: ChLinePath must contain only ChLineArc and/or ChLineSegment.");
        }

        size_t i_prev = i;
        size_t i_next = i + 1;
        if (i_next >= path.GetSubLinesCount())
            if ((path.GetEndA() - path.GetEndB()).Length() < 1e-9)
                i_next = 0;  // closed path
        if (i_next < path.GetSubLinesCount()) {
            std::shared_ptr<geometry::ChLine> line_prev = path.GetSubLineN(i_prev);
            std::shared_ptr<geometry::ChLine> line_next = path.GetSubLineN(i_next);
            auto pos_prev = line_prev->Evaluate(1);
            auto pos_next = line_next->Evaluate(0);
            auto dir_prev = line_prev->GetTangent(1);
            auto dir_next = line_next->GetTangent(0);
            dir_prev.Normalize();
            dir_next.Normalize();

            // check if connected segments
            if ((pos_prev - pos_next).Length() > 1e-9)
                throw ChException(
                    "Error! injectPath2D: ChLinePath must contain sequence of connected segments/arcs, with no gaps");

            // insert a 0-radius fillet arc at sharp corners, to allow for sharp-corner vs arc/segment
            if (Vcross(dir_prev, dir_next).z() < -1e-9) {
                double angle1 = atan2(dir_prev.y(), dir_prev.x()) + CH_C_PI_2;
                double angle2 = atan2(dir_next.y(), dir_next.x()) + CH_C_PI_2;
                geometry::ChLineArc arc(ChCoordsys<>(pos_prev, QUNIT), 0, angle1, angle2, false);
                auto shape_arc = chrono_types::make_shared<ChCollisionShapeArc2D>(material, arc, thickness);
                auto bt_shape = chrono_types::make_shared<cbt2DarcShape>(
                    (cbtScalar)pos_prev.x(), (cbtScalar)pos_prev.y(), (cbtScalar)0, (cbtScalar)angle1,
                    (cbtScalar)angle2, false, (cbtScalar)thickness);
                bt_shape->setMargin((cbtScalar)full_margin);
                injectShape(shape_arc, bt_shape, frame);
            } else {
                // GetLog() << "concave corner between " << i_next << " and " << i_next << "\n";
            }
        }
    }
}

void ChCollisionModelBullet::injectConvexHull(std::shared_ptr<ChCollisionShapeConvexHull> shape_hull,
                                              const ChFrame<>& frame) {
    const auto& points = shape_hull->GetPoints();

    auto safe_margin = GetSafeMargin();
    auto full_margin = GetSuggestedFullMargin();

    // adjust default inward margin (if object too thin)
    geometry::ChAABB aabb;
    for (size_t i = 0; i < points.size(); ++i) {
        aabb.min = Vmin(aabb.min, points[i]);
        aabb.max = Vmax(aabb.max, points[i]);
    }
    auto aabb_size = aabb.Size();
    double approx_chord = ChMin(ChMin(aabb_size.x(), aabb_size.y()), aabb_size.z());

    // override the inward margin if larger than 0.2 chord:
    model->SetSafeMargin((cbtScalar)ChMin(safe_margin, approx_chord * 0.2));

    // shrink the convex hull by GetSafeMargin()
    bt_utils::ChConvexHullLibraryWrapper lh;
    geometry::ChTriangleMeshConnected mmesh;
    lh.ComputeHull(points, mmesh);
    mmesh.MakeOffset(-safe_margin);

    auto bt_shape = chrono_types::make_shared<cbtConvexHullShape>();
    for (unsigned int i = 0; i < mmesh.m_vertices.size(); i++) {
        bt_shape->addPoint(cbtVector3((cbtScalar)mmesh.m_vertices[i].x(), (cbtScalar)mmesh.m_vertices[i].y(),
                                      (cbtScalar)mmesh.m_vertices[i].z()));
    }
    bt_shape->setMargin((cbtScalar)full_margin);
    bt_shape->recalcLocalAabb();

    injectShape(shape_hull, bt_shape, frame);
}

// -----------------------------------------------------------------------------

// These classes inherit the Bullet triangle mesh and add just a single feature: when this shape is deleted, also delete
// the referenced triangle mesh interface. Hence, when a cbtBvhTriangleMeshShape_handlemesh is added to the list of
// shapes of this ChCollisionModelBullet, there's no need to remember to delete the mesh interface because it dies with
// the model, when shapes are deleted. This is just to avoid adding a pointer to a triangle interface in all collision
// models, when not needed.
class cbtBvhTriangleMeshShape_handlemesh : public cbtBvhTriangleMeshShape {
    cbtStridingMeshInterface* minterface;

  public:
    cbtBvhTriangleMeshShape_handlemesh(cbtStridingMeshInterface* meshInterface)
        : cbtBvhTriangleMeshShape(meshInterface, true), minterface(meshInterface){};

    ~cbtBvhTriangleMeshShape_handlemesh() {
        if (minterface)
            delete minterface;
        minterface = 0;  // also delete the mesh interface
    }
};

class cbtConvexTriangleMeshShape_handlemesh : public cbtConvexTriangleMeshShape {
    cbtStridingMeshInterface* minterface;

  public:
    cbtConvexTriangleMeshShape_handlemesh(cbtStridingMeshInterface* meshInterface)
        : cbtConvexTriangleMeshShape(meshInterface), minterface(meshInterface){};

    ~cbtConvexTriangleMeshShape_handlemesh() {
        if (minterface)
            delete minterface;
        minterface = 0;  // also delete the mesh interface
    }
};

class cbtGImpactMeshShape_handlemesh : public cbtGImpactMeshShape {
    cbtStridingMeshInterface* minterface;

  public:
    cbtGImpactMeshShape_handlemesh(cbtStridingMeshInterface* meshInterface)
        : cbtGImpactMeshShape(meshInterface),
          minterface(meshInterface){
              // setLocalScaling(cbtVector3(1.f,1.f,1.f));
          };

    virtual ~cbtGImpactMeshShape_handlemesh() {
        if (minterface)
            delete minterface;
        minterface = 0;  // also delete the mesh interface
    }
};

void ChCollisionModelBullet::injectTriangleMesh(std::shared_ptr<ChCollisionShapeTriangleMesh> shape_trimesh,
                                                const ChFrame<>& frame) {
    auto envelope = GetEnvelope();
    auto safe_margin = GetSafeMargin();

    auto trimesh = shape_trimesh->GetMesh();
    auto is_static = shape_trimesh->IsStatic();
    auto is_convex = shape_trimesh->IsConvex();
    auto radius = shape_trimesh->GetRadius();

    if (!trimesh->getNumTriangles())
        return;

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

            // Add a mesh triangle collision shape (triangle with connectivity information).
            // For a non-wing vertex (i.e. 'free' edge), point to opposite vertex, that is the vertex in triangle not
            // belonging to edge. Indicate is an edge is owned by this triangle. Otherwise, they belong to a neighboring
            // triangle.
            auto shape_triangle = chrono_types::make_shared<ChCollisionShapeMeshTriangle>(
                shape_trimesh->GetMaterial(),                                                        // contact material
                &mesh->m_vertices[mesh->m_face_v_indices[it].x()],                                   // face nodes
                &mesh->m_vertices[mesh->m_face_v_indices[it].y()],                                   //
                &mesh->m_vertices[mesh->m_face_v_indices[it].z()],                                   //
                wingedgeA->second.second != -1 ? &mesh->m_vertices[i_wingvertex_A]                   // edge node 1
                                               : &mesh->m_vertices[mesh->m_face_v_indices[it].z()],  //
                wingedgeB->second.second != -1 ? &mesh->m_vertices[i_wingvertex_B]                   // edge node 2
                                               : &mesh->m_vertices[mesh->m_face_v_indices[it].x()],  //
                wingedgeC->second.second != -1 ? &mesh->m_vertices[i_wingvertex_C]                   // edge node 3
                                               : &mesh->m_vertices[mesh->m_face_v_indices[it].y()],  //
                !added_vertexes[mesh->m_face_v_indices[it].x()],                                     // face owns nodes?
                !added_vertexes[mesh->m_face_v_indices[it].y()],                                     //
                !added_vertexes[mesh->m_face_v_indices[it].z()],                                     //
                wingedgeA->second.first != -1,                                                       // face owns edges?
                wingedgeB->second.first != -1,                                                       //
                wingedgeC->second.first != -1,                                                       //
                radius                                                                               // thickness
            );

            injectTriangleProxy(shape_triangle);

            // Mark added vertexes
            added_vertexes[mesh->m_face_v_indices[it].x()] = true;
            added_vertexes[mesh->m_face_v_indices[it].y()] = true;
            added_vertexes[mesh->m_face_v_indices[it].z()] = true;
            // Mark added edges, setting to -1 the 'ti' id of 1st triangle in winged edge {{vi,vj}{ti,tj}}
            wingedgeA->second.first = -1;
            wingedgeB->second.first = -1;
            wingedgeC->second.first = -1;
        }
        return;
    }

    cbtTriangleMesh* bulletMesh = new cbtTriangleMesh;
    for (int i = 0; i < trimesh->getNumTriangles(); i++) {
        bulletMesh->addTriangle(cbtVector3CH(trimesh->getTriangle(i).p1),  //
                                cbtVector3CH(trimesh->getTriangle(i).p2),  //
                                cbtVector3CH(trimesh->getTriangle(i).p3),  //
                                true                                       // try to remove duplicate vertices
        );
    }

    if (is_static) {
        // Here a static cbtBvhTriangleMeshShape suffices, but cbtGImpactMeshShape might work better?
        auto bt_shape = chrono_types::make_shared<cbtBvhTriangleMeshShape_handlemesh>(bulletMesh);
        bt_shape->setMargin((cbtScalar)safe_margin);
        injectShape(shape_trimesh, bt_shape, frame);
        return;
    }

    if (is_convex) {
        auto bt_shape = chrono_types::make_shared<cbtConvexTriangleMeshShape_handlemesh>(bulletMesh);
        bt_shape->setMargin((cbtScalar)envelope);
        injectShape(shape_trimesh, bt_shape, frame);
    } else {
        // Note: currently there's no 'perfect' convex decomposition method, so code here is a bit experimental...

        /*
        // using the HACD convex decomposition
        auto decomposition = chrono_types::make_shared<ChConvexDecompositionHACD>();
        decomposition->AddTriangleMesh(*trimesh);
        decomposition->SetParameters(2,      // clusters
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
        */

        // using HACDv2 convex decomposition
        auto decomposition = chrono_types::make_shared<ChConvexDecompositionHACDv2>();
        decomposition->Reset();
        decomposition->AddTriangleMesh(*trimesh);
        decomposition->SetParameters(512,   // max hull count
                                     256,   // max hull merge
                                     64,    // max hull vettices
                                     0.2f,  // concavity
                                     0.0f,  // small cluster threshold
                                     1e-9f  // fuse tolerance
        );

        decomposition->ComputeConvexDecomposition();

        model->SetSafeMargin(0);
        for (unsigned int j = 0; j < decomposition->GetHullCount(); j++) {
            std::vector<ChVector<double>> ptlist;
            decomposition->GetConvexHullResult(j, ptlist);
            if (ptlist.size() > 0) {
                auto shape_hull =
                    chrono_types::make_shared<ChCollisionShapeConvexHull>(shape_trimesh->GetMaterial(), ptlist);
                injectConvexHull(shape_hull, frame);
            }
        }
    }
}

void ChCollisionModelBullet::injectTriangleProxy(std::shared_ptr<ChCollisionShapeMeshTriangle> shape_triangle) {
    model->SetSafeMargin(shape_triangle->sradius);

    auto bt_shape = chrono_types::make_shared<cbtCEtriangleShape>(
        shape_triangle->V1, shape_triangle->V2, shape_triangle->V3,              //
        shape_triangle->eP1, shape_triangle->eP2, shape_triangle->eP3,           //
        shape_triangle->ownsV1, shape_triangle->ownsV2, shape_triangle->ownsV3,  //
        shape_triangle->ownsE1, shape_triangle->ownsE2, shape_triangle->ownsE3,  //
        shape_triangle->sradius);
    bt_shape->setMargin((cbtScalar)GetSuggestedFullMargin());

    injectShape(shape_triangle, bt_shape, ChFrame<>());
}

// -----------------------------------------------------------------------------

void ChCollisionModelBullet::OnFamilyChange(short int family_group, short int family_mask) {
    // This function can only be executed for collision models already processed by the collision system.
    if (!bt_collision_object->getBroadphaseHandle())
        return;

    SyncPosition();

    auto coll_sys = std::static_pointer_cast<ChCollisionSystemBullet>(
        model->GetContactable()->GetPhysicsItem()->GetSystem()->GetCollisionSystem());

    // The only way to change the collision filters in Bullet is to remove the object and add it back in!
    // No need to remove association with the owning ChCollisionModel.
    coll_sys->Remove(this);
    coll_sys->GetBulletCollisionWorld()->addCollisionObject(bt_collision_object.get(), family_group, family_mask);
}

geometry::ChAABB ChCollisionModelBullet::GetBoundingBox() const {
    if (bt_collision_object->getCollisionShape()) {
        cbtVector3 btmin;
        cbtVector3 btmax;
        bt_collision_object->getCollisionShape()->getAabb(bt_collision_object->getWorldTransform(), btmin, btmax);
        return geometry::ChAABB(ChVector<>((double)btmin.x(), (double)btmin.y(), (double)btmin.z()),
                                ChVector<>((double)btmax.x(), (double)btmax.y(), (double)btmax.z()));
    }

    return geometry::ChAABB();
}

void ChCollisionModelBullet::SyncPosition() {
    ChCoordsys<> mcsys = GetContactable()->GetCsysForCollisionModel();

    bt_collision_object->getWorldTransform().setOrigin(
        cbtVector3((cbtScalar)mcsys.pos.x(), (cbtScalar)mcsys.pos.y(), (cbtScalar)mcsys.pos.z()));
    const ChMatrix33<>& rA(mcsys.rot);
    cbtMatrix3x3 basisA((cbtScalar)rA(0, 0), (cbtScalar)rA(0, 1), (cbtScalar)rA(0, 2), (cbtScalar)rA(1, 0),
                        (cbtScalar)rA(1, 1), (cbtScalar)rA(1, 2), (cbtScalar)rA(2, 0), (cbtScalar)rA(2, 1),
                        (cbtScalar)rA(2, 2));
    bt_collision_object->getWorldTransform().setBasis(basisA);
}

bool ChCollisionModelBullet::SetSphereRadius(double coll_radius, double out_envelope) {
    if (m_bt_shapes.size() != 1)
        return false;

    if (cbtSphereShape* bt_sphere_shape = dynamic_cast<cbtSphereShape*>(m_bt_shapes[0].get())) {
        model->SetSafeMargin(coll_radius);
        model->SetEnvelope(out_envelope);
        bt_sphere_shape->setUnscaledRadius((cbtScalar)(coll_radius + out_envelope));
        ////bt_sphere_shape->setMargin((cbtScalar)(coll_radius + out_envelope));
        return true;
    }

    return false;
}

}  // end namespace chrono
