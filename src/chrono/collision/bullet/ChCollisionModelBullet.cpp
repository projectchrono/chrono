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
#include <algorithm>

#include "chrono/collision/bullet/ChCollisionSystemBullet.h"
#include "chrono/collision/bullet/ChCollisionUtilsBullet.h"
#include "chrono/collision/bullet/ChCollisionModelBullet.h"
#include "chrono/collision/bullet/BulletCollision/CollisionShapes/cbt2DShape.h"
#include "chrono/collision/bullet/BulletCollision/CollisionShapes/cbtBarrelShape.h"
#include "chrono/collision/bullet/BulletCollision/CollisionShapes/cbtChTriangleShape.h"
#include "chrono/collision/bullet/BulletCollision/CollisionShapes/cbtPointShape.h"
#include "chrono/collision/bullet/BulletCollision/CollisionShapes/cbtSegmentShape.h"
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

/// Utility function to populate a Bullet triangle mesh from a Chrono one
static void PopulateBulletMesh(cbtTriangleMesh* bullet_mesh, std::shared_ptr<ChTriangleMesh> trimesh) {
    for (unsigned int i = 0; i < trimesh->GetNumTriangles(); i++) {
        bullet_mesh->addTriangle(cbtVector3CH(trimesh->GetTriangle(i).p1),  //
                                 cbtVector3CH(trimesh->GetTriangle(i).p2),  //
                                 cbtVector3CH(trimesh->GetTriangle(i).p3),  //
                                 true                                       // try to remove duplicate vertices
        );
    }
}

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
        pos = ChVector3d((double)p.x(), (double)p.y(), (double)p.z());
        rot = ChQuaternion<>((double)q.w(), (double)q.x(), (double)q.y(), (double)q.z());
    }
};

// -----------------------------------------------------------------------------

ChCollisionModelBullet::ChCollisionModelBullet(ChCollisionModel* collision_model) : ChCollisionModelImpl(collision_model) {
    bt_collision_object = std::unique_ptr<cbtCollisionObject>(new cbtCollisionObject);
    bt_collision_object->setCollisionShape(nullptr);
    bt_collision_object->setUserPointer((void*)this);
}

ChCollisionModelBullet::~ChCollisionModelBullet() {
    m_shapes.clear();
    m_bt_shapes.clear();
}

// -----------------------------------------------------------------------------

cbtScalar ChCollisionModelBullet::GetSuggestedFullMargin() {
    return (cbtScalar)(GetEnvelope() + GetSafeMargin());
}

void ChCollisionModelBullet::Populate() {
    auto envelope = GetEnvelope();
    auto safe_margin = GetSafeMargin();
    auto full_margin = GetSuggestedFullMargin();

    for (const auto& shape_instance : model->GetShapeInstances()) {
        const auto& shape = shape_instance.shape;
        const auto& frame = shape_instance.frame;

        switch (shape->GetType()) {
            case ChCollisionShape::Type::SPHERE: {
                auto shape_sphere = std::static_pointer_cast<ChCollisionShapeSphere>(shape);
                auto radius = shape_sphere->GetRadius();
                model->SetSafeMargin(radius);
                auto bt_shape = chrono_types::make_shared<cbtSphereShape>((cbtScalar)(radius + envelope));
                bt_shape->setMargin((cbtScalar)full_margin);
                InjectShape(shape, bt_shape, frame);
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
                bt_shape->setMargin((cbtScalar)std::min((double)full_margin, 0.9 * std::min(std::min(haxes.x(), haxes.y()), haxes.z())));
                InjectShape(shape, bt_shape, frame);
                break;
            }
            case ChCollisionShape::Type::BOX: {
                auto shape_box = std::static_pointer_cast<ChCollisionShapeBox>(shape);
                auto len = shape_box->GetLengths();
                model->SetSafeMargin(std::min((double)safe_margin, 0.1 * std::min(std::min(len.x(), len.y()), len.z())));
                auto bt_shape = chrono_types::make_shared<cbtBoxShape>(cbtVector3CH(len / 2 + envelope));
                bt_shape->setMargin((cbtScalar)full_margin);
                InjectShape(shape, bt_shape, frame);
                break;
            }
            case ChCollisionShape::Type::CYLINDER: {
                auto shape_cylinder = std::static_pointer_cast<ChCollisionShapeCylinder>(shape);
                auto height = shape_cylinder->GetHeight();
                auto radius = shape_cylinder->GetRadius();
                model->SetSafeMargin(std::min((double)safe_margin, 0.2 * std::min(radius, height / 2)));
                ChVector3d size(radius, radius, height / 2);
                auto bt_shape = chrono_types::make_shared<cbtCylinderShapeZ>(cbtVector3CH(size + envelope));
                bt_shape->setMargin((cbtScalar)full_margin);
                InjectShape(shape, bt_shape, frame);
                break;
            }
            case ChCollisionShape::Type::CAPSULE: {
                auto shape_capsule = std::static_pointer_cast<ChCollisionShapeCapsule>(shape);
                auto height = shape_capsule->GetHeight();
                auto radius = shape_capsule->GetRadius();
                model->SetSafeMargin(std::min((double)safe_margin, 0.2 * std::min(radius, height / 2)));
                auto bt_shape = chrono_types::make_shared<cbtCapsuleShapeZ>((cbtScalar)(radius + envelope), (cbtScalar)(height + 2 * envelope));
                bt_shape->setMargin((cbtScalar)full_margin);
                InjectShape(shape, bt_shape, frame);
                break;
            }
            case ChCollisionShape::Type::CYLSHELL: {
                auto shape_cylshell = std::static_pointer_cast<ChCollisionShapeCylindricalShell>(shape);
                auto height = shape_cylshell->GetHeight();
                auto radius = shape_cylshell->GetRadius();
                model->SetSafeMargin(std::min((double)safe_margin, 0.2 * std::min(radius, height / 2)));
                auto bt_shape = chrono_types::make_shared<cbtCylindricalShellShape>((cbtScalar)(radius + envelope), (cbtScalar)(height / 2 + envelope));
                bt_shape->setMargin((cbtScalar)full_margin);
                InjectShape(shape, bt_shape, frame);
                break;
            }
            case ChCollisionShape::Type::BARREL: {
                auto shape_barrel = std::static_pointer_cast<ChCollisionShapeBarrel>(shape);
                auto Y_low = shape_barrel->Y_low;
                auto Y_high = shape_barrel->Y_high;
                auto axis_vert = shape_barrel->axis_vert;
                auto axis_hor = shape_barrel->axis_hor;
                auto R_offset = shape_barrel->R_offset;
                model->SetSafeMargin(std::min((double)safe_margin, 0.15 * std::min(std::min(axis_vert / 2, axis_hor / 2), Y_high - Y_low)));
                auto sY_low = (cbtScalar)(Y_low - envelope);
                auto sY_high = (cbtScalar)(Y_high + envelope);
                auto sR_vert = (cbtScalar)(axis_vert / 2 + envelope);
                auto sR_hor = (cbtScalar)(axis_hor / 2 + envelope);
                auto sR_offset = (cbtScalar)(R_offset);
                auto bt_shape = chrono_types::make_shared<cbtBarrelShape>(sY_low, sY_high, sR_vert, sR_hor, sR_offset);
                bt_shape->setMargin((cbtScalar)full_margin);
                InjectShape(shape, bt_shape, frame);
                break;
            }
            case ChCollisionShape::CONE: {
                auto shape_cone = std::static_pointer_cast<ChCollisionShapeCone>(shape);
                auto height = shape_cone->GetHeight();
                auto radius = shape_cone->GetRadius();
                model->SetSafeMargin(std::min((double)safe_margin, 0.2 * std::min(radius, height)));
                auto bt_shape = chrono_types::make_shared<cbtConeShapeZ>(radius + envelope, height + envelope);
                bt_shape->setMargin((cbtScalar)full_margin);
                InjectShape(shape, bt_shape, frame);
                break;
            }
            case ChCollisionShape::Type::POINT: {
                auto shape_point = std::static_pointer_cast<ChCollisionShapePoint>(shape);
                auto radius = shape_point->GetRadius();
                model->SetSafeMargin(radius);
                auto bt_shape = chrono_types::make_shared<cbtPointShape>((cbtScalar)(radius + envelope));
                bt_shape->setMargin((cbtScalar)full_margin);
                InjectShape(shape, bt_shape, frame);
                break;
            }
            case ChCollisionShape::Type::SEGMENT: {
                auto shape_seg = std::static_pointer_cast<ChCollisionShapeSegment>(shape);
                InjectSegmentProxy(shape_seg);
                break;
            }
            case ChCollisionShape::Type::PATH2D: {
                auto shape_path = std::static_pointer_cast<ChCollisionShapePath2D>(shape);
                InjectPath2D(shape_path, frame);
                break;
            }
            case ChCollisionShape::Type::CONVEXHULL: {
                auto shape_hull = std::static_pointer_cast<ChCollisionShapeConvexHull>(shape);
                InjectConvexHull(shape_hull, frame);
                break;
            }
            case ChCollisionShape::Type::TRIANGLEMESH: {
                auto shape_trimesh = std::static_pointer_cast<ChCollisionShapeTriangleMesh>(shape);
                InjectTriangleMesh(shape_trimesh, frame);
                break;
            }
            case ChCollisionShape::Type::CONNECTEDTRIANGLE: {
                auto shape_triangle = std::static_pointer_cast<ChCollisionShapeConnectedTriangle>(shape);
                InjectTriangleProxy(shape_triangle);
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

void ChCollisionModelBullet::InjectShape(std::shared_ptr<ChCollisionShape> shape, std::shared_ptr<cbtCollisionShape> bt_shape, const ChFrame<>& frame) {
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

void ChCollisionModelBullet::InjectPath2D(std::shared_ptr<ChCollisionShapePath2D> shape_path, const ChFrame<>& frame) {
    const auto& path = *shape_path->GetGeometry();
    const auto& material = shape_path->GetMaterial();
    auto thickness = shape_path->GetSRadius();

    auto full_margin = GetSuggestedFullMargin();

    // The envelope is not used in this type of collision primitive.
    model->SetEnvelope(0);

    for (size_t i = 0; i < path.GetSubLinesCount(); ++i) {
        if (auto segment = std::dynamic_pointer_cast<ChLineSegment>(path.GetSubLineN(i))) {
            if (segment->pA.z() != segment->pB.z())
                throw std::runtime_error("Error! InjectPath2D: sub segment of ChLinePath not parallel to XY plane!");
            ChVector3d pA(segment->pA.x(), segment->pA.y(), 0);
            ChVector3d pB(segment->pB.x(), segment->pB.y(), 0);
            auto shape_seg = chrono_types::make_shared<ChCollisionShapeSegment2D>(material, *segment, thickness);
            shape_seg->SetParentShape(shape_path);
            auto bt_shape = chrono_types::make_shared<cbt2DsegmentShape>(cbtVector3CH(pA), cbtVector3CH(pB), (cbtScalar)thickness);
            bt_shape->setMargin((cbtScalar)full_margin);
            InjectShape(shape_seg, bt_shape, frame);
        } else if (auto arc = std::dynamic_pointer_cast<ChLineArc>(path.GetSubLineN(i))) {
            if ((arc->origin.rot.e1() != 0) || (arc->origin.rot.e2() != 0))
                throw std::invalid_argument("Error! InjectPath2D: a sub arc of ChLinePath not parallel to XY plane!");

            double angle1 = arc->angle1;
            double angle2 = arc->angle2;
            if (angle1 - angle2 == CH_2PI)
                angle1 -= 1e-7;
            auto shape_arc = chrono_types::make_shared<ChCollisionShapeArc2D>(material, *arc, thickness);
            shape_arc->SetParentShape(shape_path);
            auto bt_shape = chrono_types::make_shared<cbt2DarcShape>((cbtScalar)arc->origin.pos.x(), (cbtScalar)arc->origin.pos.y(), (cbtScalar)arc->radius, (cbtScalar)angle1,
                                                                     (cbtScalar)angle2, arc->counterclockwise, (cbtScalar)thickness);
            bt_shape->setMargin((cbtScalar)full_margin);
            InjectShape(shape_arc, bt_shape, frame);
        } else {
            throw std::invalid_argument("Error! InjectPath2D: ChLinePath must contain only ChLineArc and/or ChLineSegment.");
        }

        size_t i_prev = i;
        size_t i_next = i + 1;
        if (i_next >= path.GetSubLinesCount())
            if ((path.GetEndA() - path.GetEndB()).Length() < 1e-9)
                i_next = 0;  // closed path
        if (i_next < path.GetSubLinesCount()) {
            std::shared_ptr<ChLine> line_prev = path.GetSubLineN(i_prev);
            std::shared_ptr<ChLine> line_next = path.GetSubLineN(i_next);
            auto pos_prev = line_prev->Evaluate(1);
            auto pos_next = line_next->Evaluate(0);
            auto dir_prev = line_prev->GetTangent(1);
            auto dir_next = line_next->GetTangent(0);
            dir_prev.Normalize();
            dir_next.Normalize();

            // check if connected segments
            if ((pos_prev - pos_next).Length() > 1e-9)
                throw std::runtime_error("Error! InjectPath2D: ChLinePath must contain sequence of connected segments/arcs, with no gaps");

            // insert a 0-radius fillet arc at sharp corners, to allow for sharp-corner vs arc/segment
            if (Vcross(dir_prev, dir_next).z() < -1e-9) {
                double angle1 = atan2(dir_prev.y(), dir_prev.x()) + CH_PI_2;
                double angle2 = atan2(dir_next.y(), dir_next.x()) + CH_PI_2;
                ChLineArc arc(ChCoordsys<>(pos_prev, QUNIT), 0, angle1, angle2, false);
                auto shape_arc = chrono_types::make_shared<ChCollisionShapeArc2D>(material, arc, thickness);
                shape_arc->SetParentShape(shape_path);
                auto bt_shape = chrono_types::make_shared<cbt2DarcShape>((cbtScalar)pos_prev.x(), (cbtScalar)pos_prev.y(), (cbtScalar)0, (cbtScalar)angle1, (cbtScalar)angle2,
                                                                         false, (cbtScalar)thickness);
                bt_shape->setMargin((cbtScalar)full_margin);
                InjectShape(shape_arc, bt_shape, frame);
            } else {
                // std::cout << "concave corner between " << i_next << " and " << i_next << std::endl;
            }
        }
    }
}

void ChCollisionModelBullet::InjectConvexHull(std::shared_ptr<ChCollisionShapeConvexHull> shape_hull, const ChFrame<>& frame) {
    const auto& points = shape_hull->GetPoints();

    auto safe_margin = GetSafeMargin();
    auto full_margin = GetSuggestedFullMargin();

    // adjust default inward margin (if object too thin)
    ChAABB aabb;
    for (size_t i = 0; i < points.size(); ++i) {
        aabb.min = Vmin(aabb.min, points[i]);
        aabb.max = Vmax(aabb.max, points[i]);
    }
    auto aabb_size = aabb.Size();
    double approx_chord = std::min(std::min(aabb_size.x(), aabb_size.y()), aabb_size.z());

    // override the inward margin if larger than 0.2 chord:
    model->SetSafeMargin((cbtScalar)std::min((double)safe_margin, approx_chord * 0.2));

    // shrink the convex hull by GetSafeMargin()
    ChTriangleMeshConnected mmesh;
    bt_utils::ChConvexHullLibraryWrapper::ComputeHull(points, mmesh);
    mmesh.MakeOffset(-safe_margin);

    auto bt_shape = chrono_types::make_shared<cbtConvexHullShape>();
    for (unsigned int i = 0; i < mmesh.m_vertices.size(); i++) {
        bt_shape->addPoint(cbtVector3((cbtScalar)mmesh.m_vertices[i].x(), (cbtScalar)mmesh.m_vertices[i].y(), (cbtScalar)mmesh.m_vertices[i].z()));
    }
    bt_shape->setMargin((cbtScalar)full_margin);
    bt_shape->recalcLocalAabb();

    InjectShape(shape_hull, bt_shape, frame);
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
    cbtBvhTriangleMeshShape_handlemesh(cbtStridingMeshInterface* meshInterface) : cbtBvhTriangleMeshShape(meshInterface, true), minterface(meshInterface) {};

    ~cbtBvhTriangleMeshShape_handlemesh() {
        delete minterface;
        minterface = 0;  // also delete the mesh interface
    }
};

class cbtConvexTriangleMeshShape_handlemesh : public cbtConvexTriangleMeshShape {
    cbtStridingMeshInterface* minterface;

  public:
    cbtConvexTriangleMeshShape_handlemesh(cbtStridingMeshInterface* meshInterface) : cbtConvexTriangleMeshShape(meshInterface), minterface(meshInterface) {};

    ~cbtConvexTriangleMeshShape_handlemesh() {
        delete minterface;
        minterface = 0;  // also delete the mesh interface
    }
};

class cbtGImpactMeshShape_handlemesh : public cbtGImpactMeshShape {
    cbtStridingMeshInterface* minterface;

  public:
    cbtGImpactMeshShape_handlemesh(cbtStridingMeshInterface* meshInterface)
        : cbtGImpactMeshShape(meshInterface), minterface(meshInterface) {
              // setLocalScaling(cbtVector3(1.f,1.f,1.f));
          };

    virtual ~cbtGImpactMeshShape_handlemesh() {
        delete minterface;
        minterface = 0;  // also delete the mesh interface
    }
};

void ChCollisionModelBullet::InjectTriangleMesh(std::shared_ptr<ChCollisionShapeTriangleMesh> shape_trimesh, const ChFrame<>& frame) {
    const float envelope = GetEnvelope();
    const float safe_margin = GetSafeMargin();
    const auto trimesh = shape_trimesh->GetMesh();
    const bool is_static = shape_trimesh->IsStatic();
    const bool is_convex = shape_trimesh->IsConvex();
    const double radius = shape_trimesh->GetRadius();

    if (trimesh->GetNumTriangles() == 0) {
        return;
    }

    // Triangle mesh with connectivity ------------------------
    if (auto mesh = std::dynamic_pointer_cast<ChTriangleMeshConnected>(trimesh)) {
        std::vector<std::array<int, 4>> neighb_trimap;  // [Ti, TAi, TBi, TCi]
        bool ok_trimap = mesh->ComputeNeighbouringTriangleMap(neighb_trimap);

        std::map<std::pair<int, int>, std::pair<int, int>> winged_edges;  // {v1i, v2i}->{T1i, T2i}
        bool ok_wingedge = mesh->ComputeWingedEdges(winged_edges, true);

        std::vector<bool> added_vertices(mesh->m_vertices.size(), false);

        // Iterate on triangles
        for (size_t it = 0; it < mesh->m_face_v_indices.size(); ++it) {
            const ChVector3i& tri_verts_indices = mesh->m_face_v_indices[it];

            // Edges: pairs of vertices indices
            std::pair<int, int> edgeA(tri_verts_indices.x(), tri_verts_indices.y());
            std::pair<int, int> edgeB(tri_verts_indices.y(), tri_verts_indices.z());
            std::pair<int, int> edgeC(tri_verts_indices.z(), tri_verts_indices.x());

            // Vertex indices in edges: always in increasing order to avoid ambiguous duplicated edges
            if (edgeA.first > edgeA.second)
                std::swap(edgeA.first, edgeA.second);
            if (edgeB.first > edgeB.second)
                std::swap(edgeB.first, edgeB.second);
            if (edgeC.first > edgeC.second)
                std::swap(edgeC.first, edgeC.second);

            // Collect wing vertices
            auto wingedgeA = winged_edges.find(edgeA);
            auto wingedgeB = winged_edges.find(edgeB);
            auto wingedgeC = winged_edges.find(edgeC);

            auto get_wingvertex_idx = [&mesh](int edgetriangle_idx, const std::map<std::pair<int, int>, std::pair<int, int>>::iterator& wingedge) -> int {
                int wingvertex_idx = -1;  // no wing edge exists
                if (edgetriangle_idx != -1) {
                    const ChVector3i& edgetri_verts_indices = mesh->m_face_v_indices[edgetriangle_idx];
                    wingvertex_idx = edgetri_verts_indices.x();
                    if (edgetri_verts_indices.y() != wingedge->first.first && edgetri_verts_indices.y() != wingedge->first.second)
                        wingvertex_idx = edgetri_verts_indices.y();
                    if (edgetri_verts_indices.z() != wingedge->first.first && edgetri_verts_indices.z() != wingedge->first.second)
                        wingvertex_idx = edgetri_verts_indices.z();
                }
                return wingvertex_idx;
            };
            int wingvertexA_idx = get_wingvertex_idx(neighb_trimap[it][1], wingedgeA);
            int wingvertexB_idx = get_wingvertex_idx(neighb_trimap[it][2], wingedgeB);
            int wingvertexC_idx = get_wingvertex_idx(neighb_trimap[it][3], wingedgeC);

            // Add a mesh triangle collision shape (triangle with connectivity information).
            // For a non-wing vertex (i.e. 'free' edge), point to opposite vertex, that is the vertex in triangle not belonging to edge.
            // Indicate if an edge is owned by this triangle. Otherwise, they belong to a neighboring triangle.
            auto shape_triangle = chrono_types::make_shared<ChCollisionShapeConnectedTriangle>(
                shape_trimesh->GetMaterial(),                                                                                    // contact material
                &mesh->m_vertices[tri_verts_indices.x()],                                                                        // vertex 1 coords
                &mesh->m_vertices[tri_verts_indices.y()],                                                                        // vertex 2 coords
                &mesh->m_vertices[tri_verts_indices.z()],                                                                        // vertex 3 coords
                wingedgeA->second.second != -1 ? &mesh->m_vertices[wingvertexA_idx] : &mesh->m_vertices[tri_verts_indices.z()],  // edge 1 neighbor vertex
                wingedgeB->second.second != -1 ? &mesh->m_vertices[wingvertexB_idx] : &mesh->m_vertices[tri_verts_indices.x()],  // edge 2 neighbor vertex
                wingedgeC->second.second != -1 ? &mesh->m_vertices[wingvertexC_idx] : &mesh->m_vertices[tri_verts_indices.y()],  // edge 3 neighbor vertex
                !added_vertices[tri_verts_indices.x()],                                                                          // face owns vertex 1?
                !added_vertices[tri_verts_indices.y()],                                                                          // face owns vertex 2?
                !added_vertices[tri_verts_indices.z()],                                                                          // face owns vertex 3?
                wingedgeA->second.first != -1,                                                                                   // face owns edge 1?
                wingedgeB->second.first != -1,                                                                                   // face owns edge 2?
                wingedgeC->second.first != -1,                                                                                   // face owns edge 3?
                radius                                                                                                           // thickness
            );

            // Inject connected triangle
            shape_triangle->SetParentShape(shape_trimesh);
            InjectTriangleProxy(shape_triangle);

            // Mark added vertices
            added_vertices[tri_verts_indices.x()] = true;
            added_vertices[tri_verts_indices.y()] = true;
            added_vertices[tri_verts_indices.z()] = true;

            // Mark added edges, setting to -1 the 'Ti' id of first triangle in winged edge {vi,vj}->{Ti,Tj}
            wingedgeA->second.first = -1;
            wingedgeB->second.first = -1;
            wingedgeC->second.first = -1;
        }

        return;
    }

    // Triangle mesh without connectivity ---------------------
    if (is_static) {
        // Here a static cbtBvhTriangleMeshShape suffices, but cbtGImpactMeshShape might work better?
        cbtTriangleMesh* bullet_mesh = new cbtTriangleMesh;
        PopulateBulletMesh(bullet_mesh, trimesh);
        auto bt_shape = chrono_types::make_shared<cbtBvhTriangleMeshShape_handlemesh>(bullet_mesh);
        bt_shape->setMargin(static_cast<cbtScalar>(safe_margin));
        InjectShape(shape_trimesh, bt_shape, frame);
        return;
    }

    if (is_convex) {
        cbtTriangleMesh* bullet_mesh = new cbtTriangleMesh;
        PopulateBulletMesh(bullet_mesh, trimesh);
        auto bt_shape = chrono_types::make_shared<cbtConvexTriangleMeshShape_handlemesh>(bullet_mesh);
        bt_shape->setMargin(static_cast<cbtScalar>(envelope));
        InjectShape(shape_trimesh, bt_shape, frame);
        return;
    }

    // Fallback -----------------------------------------------
    // If none of the above conditions are met, resort to algorithmic convex decomposition

    /*
    // Use HACD convex decomposition
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

    // Use HACDv2 convex decomposition
    auto decomposition = chrono_types::make_shared<ChConvexDecompositionHACDv2>();
    decomposition->Reset();
    decomposition->AddTriangleMesh(*trimesh);
    decomposition->SetParameters(512,   // max hull count
                                 256,   // max hull merge
                                 64,    // max hull vertices
                                 0.2f,  // concavity
                                 0.0f,  // small cluster threshold
                                 1e-9f  // fuse tolerance
    );
    decomposition->ComputeConvexDecomposition();

    model->SetSafeMargin(0);
    for (unsigned int j = 0; j < decomposition->GetHullCount(); ++j) {
        std::vector<ChVector3d> ptlist;
        decomposition->GetConvexHullResult(j, ptlist);
        if (ptlist.size() > 0) {
            auto shape_hull = chrono_types::make_shared<ChCollisionShapeConvexHull>(shape_trimesh->GetMaterial(), ptlist);
            shape_hull->SetParentShape(shape_trimesh);
            InjectConvexHull(shape_hull, frame);
        }
    }
}

void ChCollisionModelBullet::InjectTriangleProxy(std::shared_ptr<ChCollisionShapeConnectedTriangle> shape_triangle) {
    model->SetSafeMargin(shape_triangle->sradius);

    auto bt_shape = chrono_types::make_shared<cbtChTriangleShape>(shape_triangle->V1, shape_triangle->V2, shape_triangle->V3,              //
                                                                  shape_triangle->eP1, shape_triangle->eP2, shape_triangle->eP3,           //
                                                                  shape_triangle->ownsV1, shape_triangle->ownsV2, shape_triangle->ownsV3,  //
                                                                  shape_triangle->ownsE1, shape_triangle->ownsE2, shape_triangle->ownsE3,  //
                                                                  shape_triangle->sradius);
    bt_shape->setMargin((cbtScalar)GetSuggestedFullMargin());

    InjectShape(shape_triangle, bt_shape, ChFrame<>());
}

void ChCollisionModelBullet::InjectSegmentProxy(std::shared_ptr<ChCollisionShapeSegment> shape_seg) {
    model->SetSafeMargin(shape_seg->radius);

    auto bt_shape = chrono_types::make_shared<cbtSegmentShape>(shape_seg->P1, shape_seg->P2, shape_seg->ownsP1, shape_seg->ownsP2, shape_seg->radius);
    bt_shape->setMargin((cbtScalar)GetSuggestedFullMargin());

    InjectShape(shape_seg, bt_shape, ChFrame<>());
}

// -----------------------------------------------------------------------------

void ChCollisionModelBullet::OnFamilyChange(short int family_group, short int family_mask) {
    // This function can only be executed for collision models already processed by the collision system.
    if (!bt_collision_object->getBroadphaseHandle())
        return;

    SyncPosition();

    auto coll_sys = std::static_pointer_cast<ChCollisionSystemBullet>(model->GetContactable()->GetPhysicsItem()->GetSystem()->GetCollisionSystem());

    // The only way to change the collision filters in Bullet is to remove the object and add it back in!
    // No need to remove association with the owning ChCollisionModel.
    coll_sys->Remove(this, false);
    coll_sys->GetBulletCollisionWorld()->addCollisionObject(bt_collision_object.get(), family_group, family_mask);
}

ChAABB ChCollisionModelBullet::GetBoundingBox() const {
    if (bt_collision_object->getCollisionShape()) {
        cbtVector3 btmin;
        cbtVector3 btmax;
        bt_collision_object->getCollisionShape()->getAabb(bt_collision_object->getWorldTransform(), btmin, btmax);
        return ChAABB(ChVector3d((double)btmin.x(), (double)btmin.y(), (double)btmin.z()), ChVector3d((double)btmax.x(), (double)btmax.y(), (double)btmax.z()));
    }

    return ChAABB();
}

void ChCollisionModelBullet::SyncPosition() {
    ChFramed frame = GetContactable()->GetCollisionModelFrame();
    const ChMatrix33d& R = frame.GetRotMat();
    cbtMatrix3x3 basisA((cbtScalar)R(0, 0), (cbtScalar)R(0, 1), (cbtScalar)R(0, 2),   //
                        (cbtScalar)R(1, 0), (cbtScalar)R(1, 1), (cbtScalar)R(1, 2),   //
                        (cbtScalar)R(2, 0), (cbtScalar)R(2, 1), (cbtScalar)R(2, 2));  //
    bt_collision_object->getWorldTransform().setOrigin(cbtVector3CH(frame.GetPos()));
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
