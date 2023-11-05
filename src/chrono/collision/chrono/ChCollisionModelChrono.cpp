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

#include "chrono/collision/chrono/ChCollisionModelChrono.h"

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {
namespace collision {

ChCollisionModelChrono::ChCollisionModelChrono() : aabb_min(C_REAL_MAX), aabb_max(-C_REAL_MAX) {
    model_safe_margin = 0;
}

ChCollisionModelChrono::~ChCollisionModelChrono() {
    m_shapes.clear();
    m_ct_shapes.clear();
}

void ChCollisionModelChrono::Dissociate() {
    if (GetPhysicsItem()->GetSystem() && GetPhysicsItem()->GetCollide())
        GetPhysicsItem()->GetSystem()->GetCollisionSystem()->Remove(this);

    local_convex_data.clear();
    aabb_min = ChVector<>(C_REAL_MAX);
    aabb_max = ChVector<>(-C_REAL_MAX);
    family_group = 1;
    family_mask = 0x7FFF;

    m_shapes.clear();
    m_ct_shapes.clear();
}

void ChCollisionModelChrono::Associate() {
    if (GetPhysicsItem()->GetSystem() && GetPhysicsItem()->GetCollide()) {
        GetPhysicsItem()->GetSystem()->GetCollisionSystem()->Add(this);
    }
}

void ChCollisionModelChrono::Populate() {
    //// TODO
    for (const auto& shape_instance : m_shape_instances) {
        const auto& shape = shape_instance.first;

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
                    auto ct_shape = chrono_types::make_shared<ctCollisionShape>();
                    geometry::ChTriangle tri = trimesh->getTriangle(i);
                    ct_shape->A =
                        real3(tri.p1.x() + position.x(), tri.p1.y() + position.y(), tri.p1.z() + position.z());
                    ct_shape->B =
                        real3(tri.p2.x() + position.x(), tri.p2.y() + position.y(), tri.p2.z() + position.z());
                    ct_shape->C =
                        real3(tri.p3.x() + position.x(), tri.p3.y() + position.y(), tri.p3.z() + position.z());
                    ct_shape->R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
                    m_shapes.push_back(shape);
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

bool ChCollisionModelChrono::AddCopyOfAnotherModel(ChCollisionModel* another) {
    // NOT SUPPORTED
    return false;
}

void ChCollisionModelChrono::GetAABB(ChVector<>& bbmin, ChVector<>& bbmax) const {
    bbmin = aabb_min;
    bbmax = aabb_max;
}

ChCoordsys<> ChCollisionModelChrono::GetShapePos(int index) const {
    const auto& ct_shape = m_ct_shapes[index];
    const auto& p = ct_shape->A;
    const auto& q = ct_shape->R;
    return ChCoordsys<>(ChVector<>((double)p.x, (double)p.y, (double)p.z),
                        ChQuaternion<>((double)q.w, (double)q.x, (double)q.y, (double)q.z));
}

void ChCollisionModelChrono::SyncPosition() {
#if !defined(NDEBUG)
    ChBody* bpointer = GetBody();
    assert(bpointer);
    assert(bpointer->GetSystem());
#endif
}

void ChCollisionModelChrono::SetContactable(ChContactable* mc) {
    // Invoke the base class method.
    ChCollisionModel::SetContactable(mc);

    // Currently, a ChCollisionModelChrono can only be a associated with a rigid body.
    mbody = dynamic_cast<ChBody*>(mc);
    assert(mbody);
}

}  // end namespace collision
}  // end namespace chrono
