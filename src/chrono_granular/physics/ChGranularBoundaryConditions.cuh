// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2018 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Dan Negrut, Nic Olsen
// =============================================================================
/*! \file */

#pragma once

#include "chrono_granular/ChGranularDefines.h"
#include "chrono_granular/utils/ChCudaMathUtils.cuh"
#include "chrono_granular/physics/ChGranularBoundaryConditions.h"

using chrono::granular::BC_type;
using chrono::granular::BC_params_t;
using chrono::granular::AABox_BC_params_t;
using chrono::granular::sphere_BC_params_t;
using chrono::granular::cone_BC_params_t;
using chrono::granular::Plane_BC_params_t;

//// DO NOT USE, IS BROKEN
inline __device__ bool addBCForces_AABox(const int sphXpos,
                                         const int sphYpos,
                                         const int sphZpos,
                                         const float sphXvel,
                                         const float sphYvel,
                                         const float sphZvel,
                                         float& X_force,
                                         float& Y_force,
                                         float& Z_force,
                                         ParamsPtr gran_params,
                                         const BC_params_t<int, int3>& bc_params) {
    return false;  // TODO implement
    // AABox_BC_params_t<int, int3> AABox_params = bc_params.AABox_params;
    // bool contact = false;
    // // classic radius grab, this must be signed to avoid false conversions
    // const signed int sphereRadius_SU = (signed int)gran_params->sphereRadius_SU;
    //
    // // Compute spring factor in force
    // float scalingFactor = AABox_params.normal_sign * gran_params->K_n_s2w_SU;
    //
    // // penetration into wall
    // signed int pen = 0;
    // // Are we touching wall?
    // int touchingWall = 0;
    //
    // // Do x direction
    // // penetration of sphere into bottom x wall
    // pen = sphXpos - sphereRadius_SU - AABox_params.min_corner.x;
    // // true if sphere touching wall
    // touchingWall = (pen < 0) && abs(pen) < sphereRadius_SU;
    // contact &= touchingWall;
    // // in this case, pen is negative and we want a positive restorative force
    // // Need to get relative velocity
    // X_force += touchingWall * (scalingFactor * abs(pen));
    //
    // // Do top X wall
    // pen = AABox_params.max_corner.x - (sphXpos + (signed int)sphereRadius_SU);
    // touchingWall = (pen < 0) && abs(pen) < sphereRadius_SU;
    // contact &= touchingWall;
    //
    // // in this case, pen is positive and we want a negative restorative force
    // X_force += touchingWall * (-1 * scalingFactor * abs(pen));
    //
    // // penetration of sphere into relevant wall
    // pen = sphYpos - (signed int)sphereRadius_SU - AABox_params.min_corner.y;
    // // true if sphere touching wall
    // touchingWall = (pen < 0) && abs(pen) < sphereRadius_SU;
    // contact &= touchingWall;
    //
    // // in this case, pen is negative and we want a positive restorative force
    // Y_force += touchingWall * (scalingFactor * abs(pen));
    //
    // // Do top y wall
    // pen = AABox_params.max_corner.y - (sphYpos + (signed int)sphereRadius_SU);
    // touchingWall = (pen < 0) && abs(pen) < sphereRadius_SU;
    // contact &= touchingWall;
    //
    // // in this case, pen is positive and we want a negative restorative force
    // Y_force += touchingWall * (-1 * scalingFactor * abs(pen));
    //
    // // penetration of sphere into relevant wall
    // pen = sphZpos - (signed int)sphereRadius_SU - AABox_params.min_corner.z;
    // // true if sphere touching wall
    // touchingWall = (pen < 0) && abs(pen) < sphereRadius_SU;
    // contact &= touchingWall;
    //
    // // in this case, pen is negative and we want a positive restorative force
    // Z_force += touchingWall * (scalingFactor * abs(pen));
    //
    // // Do top z wall
    // pen = AABox_params.max_corner.z - (sphZpos + (signed int)sphereRadius_SU);
    // touchingWall = (pen < 0) && abs(pen) < sphereRadius_SU;
    // contact &= touchingWall;
    //
    // // in this case, pen is positive and we want a negative restorative force
    // Z_force += touchingWall * (-1 * scalingFactor * abs(pen));
    // return contact;
}

inline __device__ bool addBCForces_Sphere(const int sphXpos,
                                          const int sphYpos,
                                          const int sphZpos,
                                          const float sphXvel,
                                          const float sphYvel,
                                          const float sphZvel,
                                          float& X_force,
                                          float& Y_force,
                                          float& Z_force,
                                          ParamsPtr gran_params,
                                          const BC_params_t<int, int3>& bc_params) {
    sphere_BC_params_t<int, int3> sphere_params = bc_params.sphere_params;
    bool contact = false;
    // classic radius grab, this must be signed to avoid false conversions
    const signed int sphereRadius_SU = (signed int)gran_params->sphereRadius_SU;
    // sum of two radii
    float total_diameter = (sphere_params.radius + sphereRadius_SU);
    float inv_diameter = 1.f / (total_diameter);

    double dX = (sphXpos - sphere_params.sphere_center.x) * inv_diameter;
    double dY = (sphYpos - sphere_params.sphere_center.y) * inv_diameter;
    double dZ = (sphZpos - sphere_params.sphere_center.z) * inv_diameter;

    double d2 = dX * dX + dY * dY + dZ * dZ;
    float reciplength = rsqrt(d2);

    float penetration = reciplength - 1.;
    // contact
    if (d2 < 1) {
        contact = true;
        // spring term
        X_force += sphere_params.normal_sign * gran_params->K_n_s2w_SU * dX * total_diameter * penetration;
        Y_force += sphere_params.normal_sign * gran_params->K_n_s2w_SU * dY * total_diameter * penetration;
        Z_force += sphere_params.normal_sign * gran_params->K_n_s2w_SU * dZ * total_diameter * penetration;

        // damping term
        // Compute force updates for damping term
        // Project relative velocity to the normal
        // n = delta * reciplength
        // proj = Dot(delta_dot, n)
        // TODO this assumes walls at rest
        float projection = (sphXvel * dX + sphYvel * dY + sphZvel * dZ) * reciplength;

        constexpr float m_eff = 0.5;

        X_force += -gran_params->Gamma_n_s2w_SU * projection * dX * reciplength * m_eff;
        X_force += -gran_params->Gamma_n_s2w_SU * projection * dY * reciplength * m_eff;
        X_force += -gran_params->Gamma_n_s2w_SU * projection * dZ * reciplength * m_eff;
    }
    return contact;
}
/// TODO check damping, adhesion
inline __device__ bool addBCForces_Cone(const int sphXpos,
                                        const int sphYpos,
                                        const int sphZpos,
                                        const float sphXvel,
                                        const float sphYvel,
                                        const float sphZvel,
                                        float& X_force,
                                        float& Y_force,
                                        float& Z_force,
                                        ParamsPtr gran_params,
                                        const BC_params_t<int, int3>& bc_params) {
    cone_BC_params_t<int, int3> cone_params = bc_params.cone_params;
    bool contact = false;
    // classic radius grab, this must be signed to avoid false conversions
    const signed int sphereRadius_SU = (signed int)gran_params->sphereRadius_SU;

    // Point on cone directly below sphere
    float3 sphere_pos_rel = make_float3(sphXpos - cone_params.cone_tip.x, sphYpos - cone_params.cone_tip.y,
                                        sphZpos - cone_params.cone_tip.z);

    float Px = sphere_pos_rel.x;
    float Py = sphere_pos_rel.y;
    float Pz = cone_params.slope * sqrt(Px * Px + Py * Py);

    // line from tip to P
    float3 l = make_float3(Px, Py, Pz);
    // get line from P to sphere
    float3 delta = sphere_pos_rel - l;

    // vector from contact point to sphere
    float3 contact_vector = delta - Dot(delta, l) * l / Dot(l, l);

    float dist = Length(contact_vector);

    // put contact vector in tip-relative frame, then find
    float contact_height = sphere_pos_rel.z - contact_vector.z;

    // give us a contact normal
    contact_vector = contact_vector / dist;

    // positive means we are penetrating
    float pen = sphereRadius_SU - dist;

    // if penetrating and the material is inside (not above or below) the cone, add forces
    if (pen > 0 && contact_height >= cone_params.hmin - cone_params.cone_tip.z &&
        contact_height <= cone_params.hmax - cone_params.cone_tip.z) {
        // add spring term
        X_force += cone_params.normal_sign * gran_params->K_n_s2w_SU * pen * contact_vector.x;
        Y_force += cone_params.normal_sign * gran_params->K_n_s2w_SU * pen * contact_vector.y;
        Z_force += cone_params.normal_sign * gran_params->K_n_s2w_SU * pen * contact_vector.z;

        // damping term
        // Compute force updates for damping term
        // Project relative velocity to the normal
        // assume static BC
        float projection = (sphXvel * contact_vector.x + sphYvel * contact_vector.y + sphZvel * contact_vector.z);

        constexpr float m_eff = 0.5;

        X_force += -gran_params->Gamma_n_s2w_SU * projection * contact_vector.x * m_eff;
        Y_force += -gran_params->Gamma_n_s2w_SU * projection * contact_vector.y * m_eff;
        Z_force += -gran_params->Gamma_n_s2w_SU * projection * contact_vector.z * m_eff;
        contact = true;
    }
    return contact;
}

/// TODO check damping, adhesion
inline __device__ bool addBCForces_Plane(const int sphXpos,
                                         const int sphYpos,
                                         const int sphZpos,
                                         const float sphXvel,
                                         const float sphYvel,
                                         const float sphZvel,
                                         float& X_force,
                                         float& Y_force,
                                         float& Z_force,
                                         ParamsPtr gran_params,
                                         const BC_params_t<int, int3>& bc_params) {
    Plane_BC_params_t<int3> plane_params = bc_params.plane_params;
    bool contact = false;
    // classic radius grab, this must be signed to avoid false conversions
    const signed int sphereRadius_SU = (signed int)gran_params->sphereRadius_SU;

    // Vector from point on plane to sphere center
    float3 delta_r = make_float3(sphXpos - plane_params.position.x, sphYpos - plane_params.position.y,
                                 sphZpos - plane_params.position.z);

    // projection displacement onto plane normal
    float proj = Dot(plane_params.normal, delta_r);
    // positive implies radius is bigger than distance, so there is penetration
    float penetration = sphereRadius_SU - proj;

    if (penetration > 0) {
        contact = true;
        X_force += gran_params->K_n_s2w_SU * penetration * plane_params.normal.x;
        Y_force += gran_params->K_n_s2w_SU * penetration * plane_params.normal.y;
        Z_force += gran_params->K_n_s2w_SU * penetration * plane_params.normal.z;

        // damping term
        // Compute force updates for damping term
        // Project relative velocity to the normal
        // assume static BC
        float projection =
            (sphXvel * plane_params.normal.x + sphYvel * plane_params.normal.y + sphZvel * plane_params.normal.z);

        constexpr float m_eff = 0.5;

        X_force += -gran_params->Gamma_n_s2w_SU * projection * plane_params.normal.x * m_eff;
        Y_force += -gran_params->Gamma_n_s2w_SU * projection * plane_params.normal.y * m_eff;
        Z_force += -gran_params->Gamma_n_s2w_SU * projection * plane_params.normal.z * m_eff;
    }

    return contact;
}