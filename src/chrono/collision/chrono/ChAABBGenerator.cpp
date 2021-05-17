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
// Authors: Hammad Mazhar, Radu Serban
// =============================================================================

#include <algorithm>
#include <climits>

#include "chrono/collision/chrono/ChAABBGenerator.h"
#include "chrono/collision/chrono/ChConvexShape.h"

using namespace chrono;
using namespace chrono::collision;

// -----------------------------------------------------------------------------

static void ComputeAABBSphere(const real& radius,
                              const real3& lpositon,
                              const real3& position,
                              const quaternion& body_rotation,
                              real3& minp,
                              real3& maxp) {
    real3 pos = Rotate(lpositon, body_rotation) + position;
    minp = pos - radius;
    maxp = pos + radius;
}

static void ComputeAABBTriangle(const real3& A, const real3& B, const real3& C, real3& minp, real3& maxp) {
    minp.x = Min(A.x, Min(B.x, C.x));
    minp.y = Min(A.y, Min(B.y, C.y));
    minp.z = Min(A.z, Min(B.z, C.z));
    maxp.x = Max(A.x, Max(B.x, C.x));
    maxp.y = Max(A.y, Max(B.y, C.y));
    maxp.z = Max(A.z, Max(B.z, C.z));
}

static void ComputeAABBBox(const real3& dim,
                           const real3& lpositon,
                           const real3& position,
                           const quaternion& rotation,
                           const quaternion& body_rotation,
                           real3& minp,
                           real3& maxp) {
    real3 temp = AbsRotate(rotation, dim);
    real3 pos = Rotate(lpositon, body_rotation) + position;
    minp = pos - temp;
    maxp = pos + temp;
}

/*
static void ComputeAABBCone(const real3& dim,
                            const real3& lpositon,
                            const real3& positon,
                            const quaternion& rotation,
                            const quaternion& body_rotation,
                            real3& minp,
                            real3& maxp) {
    real3 temp = AbsRotate(rotation, real3(dim.x, dim.y, dim.z / 2.0));
    real3 pos = Rotate(lpositon - real3(0, 0, dim.z / 2.0), body_rotation) + positon;
    minp = pos - temp;
    maxp = pos + temp;
}
*/

static void ComputeAABBConvex(const real3* convex_points,
                              const int start,
                              const int size,
                              const real3& lpos,
                              const real3& pos,
                              const quaternion& rot,
                              real3& minp,
                              real3& maxp) {
    real3 point_0 = Rotate(convex_points[start] + lpos, rot) + pos;

    minp = maxp = point_0;
    for (int i = start; i < start + size; i++) {
        real3 p = Rotate(convex_points[i] + lpos, rot) + pos;
        if (minp.x > p.x) {
            minp.x = p.x;
        }
        if (minp.y > p.y) {
            minp.y = p.y;
        }
        if (minp.z > p.z) {
            minp.z = p.z;
        }
        if (maxp.x < p.x) {
            maxp.x = p.x;
        }
        if (maxp.y < p.y) {
            maxp.y = p.y;
        }
        if (maxp.z < p.z) {
            maxp.z = p.z;
        }
    }
}

// -----------------------------------------------------------------------------

ChAABBGenerator::ChAABBGenerator() : data_manager(nullptr) {}

void ChAABBGenerator::GenerateAABB(real envelope) {
    if (data_manager->num_rigid_shapes > 0) {
        const custom_vector<shape_type>& typ_rigid = data_manager->shape_data.typ_rigid;
        const custom_vector<int>& start_rigid = data_manager->shape_data.start_rigid;
        const custom_vector<uint>& id_rigid = data_manager->shape_data.id_rigid;
        const custom_vector<real3>& obj_data_A = data_manager->shape_data.ObA_rigid;
        const custom_vector<quaternion>& obj_data_R = data_manager->shape_data.ObR_rigid;
        const custom_vector<real3>& convex_rigid = data_manager->shape_data.convex_rigid;
        const custom_vector<real3>& pos_rigid = data_manager->state_data.pos_rigid;
        const custom_vector<quaternion>& body_rot = data_manager->state_data.rot_rigid;
        const uint num_rigid_shapes = data_manager->num_rigid_shapes;
        custom_vector<real3>& aabb_min = data_manager->host_data.aabb_min;
        custom_vector<real3>& aabb_max = data_manager->host_data.aabb_max;

        aabb_min.resize(num_rigid_shapes);
        aabb_max.resize(num_rigid_shapes);

#pragma omp parallel for
        for (int index = 0; index < (signed)num_rigid_shapes; index++) {
            // Shape data
            shape_type type = typ_rigid[index];
            real3 local_pos = obj_data_A[index];
            quaternion local_rot = obj_data_R[index];
            uint id = id_rigid[index];  // The rigid body corresponding to this shape
            int start = start_rigid[index];

            // Body data
            if (id == UINT_MAX)
                continue;

            real3 position = pos_rigid[id];
            quaternion rotation = Mult(body_rot[id], local_rot);
            real3 temp_min;
            real3 temp_max;

            if (type == ChCollisionShape::Type::SPHERE) {
                real radius = data_manager->shape_data.sphere_rigid[start];
                ComputeAABBSphere(radius + envelope, local_pos, position, body_rot[id], temp_min, temp_max);

            } else if (type == ChCollisionShape::Type::ELLIPSOID || type == ChCollisionShape::Type::BOX ||
                       type == ChCollisionShape::Type::CYLINDER || type == ChCollisionShape::Type::CYLSHELL ||
                       type == ChCollisionShape::Type::CONE) {
                real3 B = data_manager->shape_data.box_like_rigid[start];
                ComputeAABBBox(B + envelope, local_pos, position, rotation, body_rot[id], temp_min, temp_max);

            } else if (type == ChCollisionShape::Type::ROUNDEDBOX || type == ChCollisionShape::Type::ROUNDEDCYL ||
                       type == ChCollisionShape::Type::ROUNDEDCONE) {
                real4 T = data_manager->shape_data.rbox_like_rigid[start];
                real3 B = real3(T.x, T.y, T.z) + T.w + envelope;
                ComputeAABBBox(B, local_pos, position, rotation, body_rot[id], temp_min, temp_max);

            } else if (type == ChCollisionShape::Type::CAPSULE) {
                real2 T = data_manager->shape_data.capsule_rigid[start];
                real3 B_ = real3(T.x, T.x + T.y, T.x) + envelope;
                ComputeAABBBox(B_, local_pos, position, rotation, body_rot[id], temp_min, temp_max);

            } else if (type == ChCollisionShape::Type::CONVEX) {
                int length = data_manager->shape_data.length_rigid[index];
                ComputeAABBConvex(convex_rigid.data(), start, length, local_pos, position, rotation, temp_min,
                                  temp_max);
                temp_min -= envelope;
                temp_max += envelope;

            } else if (type == ChCollisionShape::Type::TRIANGLE) {
                real3 A, B, C;

                A = data_manager->shape_data.triangle_rigid[start + 0];
                B = data_manager->shape_data.triangle_rigid[start + 1];
                C = data_manager->shape_data.triangle_rigid[start + 2];

                A = Rotate(A, body_rot[id]) + position;
                B = Rotate(B, body_rot[id]) + position;
                C = Rotate(C, body_rot[id]) + position;

                ComputeAABBTriangle(A, B, C, temp_min, temp_max);

            } else {
                continue;
            }

            aabb_min[index] = temp_min;
            aabb_max[index] = temp_max;
        }
    }
}
