// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#include "chrono/physics/ChBody.h"
#include "chrono_distributed/collision/ChCollisionModelDistributed.h"

using namespace chrono;

ChCollisionModelDistributed::ChCollisionModelDistributed() {
    aabb_valid = false;
}

ChCollisionModelDistributed::~ChCollisionModelDistributed() {}

void ChCollisionModelDistributed::Dissociate() {
    ChCollisionModelChrono::Dissociate();
    aabb_valid = false;
}

void ChCollisionModelDistributed::Associate() {
    ChCollisionModelChrono::Associate();
}

void ChCollisionModelDistributed::Populate() {
    // Populate the base Chrono collision model
    ChCollisionModelChrono::Populate();

    // Traverse all collision shapes and attach additional data.
    // Curently, only support for spheres and boxes
    for (size_t is = 0; is < m_shapes.size(); is++) {
        const auto& shape = m_shapes[is];
        auto& ct_shape = m_ct_shapes[is];

        switch (shape->GetType()) {
            case ChCollisionShape::Type::SPHERE: {
                const auto& pos = ct_shape->A;
                const auto& radius = ct_shape->B[0];

                real3 shape_aabb_min(pos.x - radius, pos.y - radius, pos.z - radius);
                real3 shape_aabb_max(pos.x + radius, pos.y + radius, pos.z + radius);

                // If this is the first shape being added to the model, set the first reference points
                if (!aabb_valid) {
                    aabb_min = shape_aabb_min;
                    aabb_max = shape_aabb_max;
                    aabb_valid = true;
                } else {
                    aabb_min = Min(aabb_min, shape_aabb_min);
                    aabb_max = Max(aabb_max, shape_aabb_max);
                }

                ct_shape->aabb_min = shape_aabb_min;
                ct_shape->aabb_max = shape_aabb_max;

                break;
            }
            case ChCollisionShape::Type::BOX: {
                const auto& pos = ct_shape->A;
                const auto& hlen = ct_shape->B;
                const auto& rot = ct_shape->R;

                // Generate 8 corners of the box
                auto rx = rot * real3(hlen.x, 0, 0);
                auto ry = rot * real3(0, hlen.y, 0);
                auto rz = rot * real3(0, 0, hlen.z);

                // Vertices of collision box
                real3 v[8];
                v[0] = pos + rx + ry + rz;
                v[1] = pos + rx + ry - rz;
                v[2] = pos + rx - ry + rz;
                v[3] = pos + rx - ry - rz;
                v[4] = pos - rx + ry + rz;
                v[5] = pos - rx + ry - rz;
                v[6] = pos - rx - ry + rz;
                v[7] = pos - rx - ry - rz;

                // If this is the first shape being added to the model, set the first reference points
                if (!aabb_valid) {
                    aabb_min = v[0];
                    aabb_max = v[0];
                    aabb_valid = true;
                }

                real3 shape_aabb_min = v[0];
                real3 shape_aabb_max = v[0];
                for (int i = 0; i < 8; i++) {
                    aabb_min = Min(aabb_min, v[i]);
                    aabb_max = Max(aabb_max, v[i]);
                    shape_aabb_min = Min(shape_aabb_min, v[i]);
                    shape_aabb_max = Max(shape_aabb_max, v[i]);
                }

                ct_shape->aabb_min = shape_aabb_min;
                ct_shape->aabb_max = shape_aabb_max;

                break;
            }
            default:
                std::cout << "*** Collision shape type NOT SUPPORTED in Chrono::Distributed! ***" << std::endl;
                break;
        }
    }
}

void ChCollisionModelDistributed::GetAABB(ChVector<>& bbmin, ChVector<>& bbmax) const {
    bbmin = ChVector<>((double)aabb_min.x, (double)aabb_min.y, (double)aabb_min.z);
    bbmax = ChVector<>((double)aabb_max.x, (double)aabb_max.y, (double)aabb_max.z);
}
