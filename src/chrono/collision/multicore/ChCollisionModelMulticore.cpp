// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Hammad Mazhar
// =============================================================================
// Geometric model for the custom multicore Chrono collision system
//
// TODO: Collision family information must currently be set BEFORE the collision
//       model is added to the collision system!   Can this be relaxed?
//       Note that we can figure out if the model was added by checking whether
//       the associated body has a ChSystem (mbody->GetSystem())
// =============================================================================

#include "chrono/collision/multicore/ChCollisionModelMulticore.h"

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChCollisionModelMulticore)
CH_UPCASTING(ChCollisionModelMulticore, ChCollisionModelImpl)

ChCollisionModelMulticore::ChCollisionModelMulticore(ChCollisionModel* collision_model)
    : ChCollisionModelImpl(collision_model), aabb_min(C_REAL_MAX), aabb_max(-C_REAL_MAX) {
    collision_model->SetSafeMargin(0);

    assert(collision_model->GetContactable());

    // Currently, a ChCollisionModelMulticore can only be a associated with a rigid body.
    mbody = dynamic_cast<ChBody*>(collision_model->GetContactable());
    assert(mbody);
}

ChCollisionModelMulticore::~ChCollisionModelMulticore() {
    m_shapes.clear();
    m_ct_shapes.clear();
}

void ChCollisionModelMulticore::Populate() {
    for (const auto& shape_instance : model->GetShapes()) {
        const auto& shape = shape_instance.first;
        const auto& material = shape->GetMaterial();

        // Create collision shapes relative to the body COG frame
        auto frame = shape_instance.second;
        if (ChBodyAuxRef* body_ar = dynamic_cast<ChBodyAuxRef*>(GetBody())) {
            frame = frame >> body_ar->GetFrame_REF_to_COG();
        }
        const ChVector<>& position = frame.GetPos();
        const ChQuaternion<>& rotation = frame.GetRot();

        switch (shape->GetType()) {
            case ChCollisionShape::Type::SPHERE: {
                auto shape_sphere = std::static_pointer_cast<ChCollisionShapeSphere>(shape);
                auto radius = shape_sphere->GetRadius();

                auto ct_shape = chrono_types::make_shared<ctCollisionShape>();
                ct_shape->A = real3(position.x(), position.y(), position.z());
                ct_shape->B = real3(radius, 0, 0);
                ct_shape->C = real3(0, 0, 0);
                ct_shape->R = quaternion(1, 0, 0, 0);

                m_shapes.push_back(shape);
                m_ct_shapes.push_back(ct_shape);
                break;
            }
            case ChCollisionShape::Type::ELLIPSOID: {
                auto shape_ell = std::static_pointer_cast<ChCollisionShapeEllipsoid>(shape);
                const auto& haxes = shape_ell->GetSemiaxes();

                auto ct_shape = chrono_types::make_shared<ctCollisionShape>();
                ct_shape->A = real3(position.x(), position.y(), position.z());
                ct_shape->B = real3(haxes.x(), haxes.y(), haxes.z());
                ct_shape->C = real3(0, 0, 0);
                ct_shape->R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());

                m_shapes.push_back(shape);
                m_ct_shapes.push_back(ct_shape);
                break;
            }
            case ChCollisionShape::Type::BOX: {
                auto shape_box = std::static_pointer_cast<ChCollisionShapeBox>(shape);
                const auto& hlen = shape_box->GetHalflengths();

                auto ct_shape = chrono_types::make_shared<ctCollisionShape>();
                ct_shape->A = real3(position.x(), position.y(), position.z());
                ct_shape->B = real3(hlen.x(), hlen.y(), hlen.z());
                ct_shape->C = real3(0, 0, 0);
                ct_shape->R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());

                m_shapes.push_back(shape);
                m_ct_shapes.push_back(ct_shape);
                break;
            }
            case ChCollisionShape::Type::CYLINDER: {
                auto shape_cylinder = std::static_pointer_cast<ChCollisionShapeCylinder>(shape);
                auto height = shape_cylinder->GetHeight();
                auto radius = shape_cylinder->GetRadius();

                auto ct_shape = chrono_types::make_shared<ctCollisionShape>();
                ct_shape->A = real3(position.x(), position.y(), position.z());
                ct_shape->B = real3(radius, radius, height / 2);
                ct_shape->C = real3(0, 0, 0);
                ct_shape->R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());

                m_shapes.push_back(shape);
                m_ct_shapes.push_back(ct_shape);
                break;
            }
            case ChCollisionShape::Type::CAPSULE: {
                auto shape_capsule = std::static_pointer_cast<ChCollisionShapeCapsule>(shape);
                auto height = shape_capsule->GetHeight();
                auto radius = shape_capsule->GetRadius();

                auto ct_shape = chrono_types::make_shared<ctCollisionShape>();
                ct_shape->A = real3(position.x(), position.y(), position.z());
                ct_shape->B = real3(radius, radius, height / 2);
                ct_shape->C = real3(0, 0, 0);
                ct_shape->R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());

                m_shapes.push_back(shape);
                m_ct_shapes.push_back(ct_shape);
                break;
            }
            case ChCollisionShape::Type::CYLSHELL: {
                auto shape_cylshell = std::static_pointer_cast<ChCollisionShapeCylindricalShell>(shape);
                auto height = shape_cylshell->GetHeight();
                auto radius = shape_cylshell->GetRadius();

                auto ct_shape = chrono_types::make_shared<ctCollisionShape>();
                ct_shape->A = real3(position.x(), position.y(), position.z());
                ct_shape->B = real3(radius, radius, height / 2);
                ct_shape->C = real3(0, 0, 0);
                ct_shape->R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());

                m_shapes.push_back(shape);
                m_ct_shapes.push_back(ct_shape);
                break;
            }
            case ChCollisionShape::CONE: {
                auto shape_cone = std::static_pointer_cast<ChCollisionShapeCone>(shape);
                auto height = shape_cone->GetHeight();
                auto radius = shape_cone->GetRadius();

                auto ct_shape = chrono_types::make_shared<ctCollisionShape>();
                ct_shape->A = real3(position.x(), position.y(), position.z());
                ct_shape->B = real3(radius, radius, height / 2);
                ct_shape->C = real3(0, 0, 0);
                ct_shape->R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());

                m_shapes.push_back(shape);
                m_ct_shapes.push_back(ct_shape);
                break;
            }
            case ChCollisionShape::Type::CONVEXHULL: {
                auto shape_hull = std::static_pointer_cast<ChCollisionShapeConvexHull>(shape);
                const auto& points = shape_hull->GetPoints();

                auto ct_shape = chrono_types::make_shared<ctCollisionShape>();
                ct_shape->A = real3(position.x(), position.y(), position.z());
                ct_shape->B = real3((chrono::real)points.size(), (chrono::real)local_convex_data.size(), 0);
                ct_shape->C = real3(0, 0, 0);
                ct_shape->R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
                for (const auto& p : points) {
                    local_convex_data.push_back(real3(p.x(), p.y(), p.z()));
                }

                m_shapes.push_back(shape);
                m_ct_shapes.push_back(ct_shape);
                break;
            }
            case ChCollisionShape::Type::ROUNDEDBOX: {
                auto shape_rbox = std::static_pointer_cast<ChCollisionShapeRoundedBox>(shape);
                const auto& hlen = shape_rbox->GetHalflengths();
                auto srad = shape_rbox->GetSRadius();

                auto ct_shape = chrono_types::make_shared<ctCollisionShape>();
                ct_shape->A = real3(position.x(), position.y(), position.z());
                ct_shape->B = real3(hlen.x(), hlen.y(), hlen.z());
                ct_shape->C = real3(srad, 0, 0);
                ct_shape->R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());

                m_shapes.push_back(shape);
                m_ct_shapes.push_back(ct_shape);
                break;
            }
            case ChCollisionShape::Type::ROUNDEDCYL: {
                auto shape_rcyl = std::static_pointer_cast<ChCollisionShapeRoundedCylinder>(shape);
                auto height = shape_rcyl->GetHeight();
                auto radius = shape_rcyl->GetRadius();
                auto srad = shape_rcyl->GetSRadius();

                auto ct_shape = chrono_types::make_shared<ctCollisionShape>();
                ct_shape->A = real3(position.x(), position.y(), position.z());
                ct_shape->B = real3(radius, radius, height / 2);
                ct_shape->C = real3(srad, 0, 0);
                ct_shape->R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());

                m_shapes.push_back(shape);
                m_ct_shapes.push_back(ct_shape);
                break;
            }
            case ChCollisionShape::Type::TRIANGLE: {
                auto shape_tri = std::static_pointer_cast<ChCollisionShapeTriangle>(shape);
                const auto& p1 = shape_tri->GetGeometry().p1;
                const auto& p2 = shape_tri->GetGeometry().p1;
                const auto& p3 = shape_tri->GetGeometry().p1;

                auto ct_shape = chrono_types::make_shared<ctCollisionShape>();
                ct_shape->A = real3(p1.x() + position.x(), p1.y() + position.y(), p1.z() + position.z());
                ct_shape->B = real3(p2.x() + position.x(), p2.y() + position.y(), p2.z() + position.z());
                ct_shape->C = real3(p3.x() + position.x(), p3.y() + position.y(), p3.z() + position.z());
                ct_shape->R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());

                m_shapes.push_back(shape);
                m_ct_shapes.push_back(ct_shape);
                break;
            }
            case ChCollisionShape::Type::TRIANGLEMESH: {
                auto shape_trimesh = std::static_pointer_cast<ChCollisionShapeTriangleMesh>(shape);
                auto trimesh = shape_trimesh->GetMesh();

                for (int i = 0; i < trimesh->getNumTriangles(); i++) {
                    geometry::ChTriangle tri = trimesh->getTriangle(i);
                    ChVector<> p1 = tri.p1 + position;
                    ChVector<> p2 = tri.p2 + position;
                    ChVector<> p3 = tri.p3 + position;
                    auto shape_triangle = chrono_types::make_shared<ChCollisionShapeTriangle>(material, p1, p2, p3);
                    auto ct_shape = chrono_types::make_shared<ctCollisionShape>();
                    ct_shape->A = real3(p1.x(), p1.y(), p1.z());
                    ct_shape->B = real3(p2.x(), p2.y(), p2.z());
                    ct_shape->C = real3(p3.x(), p3.y(), p3.z());
                    ct_shape->R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
                    m_shapes.push_back(shape_triangle);
                    m_ct_shapes.push_back(ct_shape);
                }
                break;
            }
            default:
                // Shape type not supported
                break;
        }
    }

    // The number of total collision shapes must match the number of Chrono collision shapes
    assert(m_shapes.size() == m_ct_shapes.size());
}

// TransformToCOG
// This utility function converts a given position and orientation, specified
// with respect to a body's reference frame, into a frame defined with respect
// to the body's centroidal frame.  Note that by default, a body's reference
// frame is the centroidal frame. This is not true for a ChBodyAuxRef.
void TransformToCOG(ChBody* body, const ChVector<>& pos, const ChMatrix33<>& rot, ChFrame<>& frame) {
    frame = ChFrame<>(pos, rot);
    if (ChBodyAuxRef* body_ar = dynamic_cast<ChBodyAuxRef*>(body)) {
        frame = frame >> body_ar->GetFrame_REF_to_COG();
    }
}

geometry::ChAABB ChCollisionModelMulticore::GetBoundingBox() const {
    return geometry::ChAABB(aabb_min, aabb_max);
}

void ChCollisionModelMulticore::SyncPosition() {
#if !defined(NDEBUG)
    ChBody* bpointer = GetBody();
    assert(bpointer);
    assert(bpointer->GetSystem());
#endif
}

}  // end namespace chrono
