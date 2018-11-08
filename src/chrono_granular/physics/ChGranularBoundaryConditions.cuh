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
using chrono::granular::AABox_BC_params_t;  // todo implement
using chrono::granular::sphere_BC_params_t;
using chrono::granular::cone_BC_params_t;
using chrono::granular::Plane_BC_params_t;

inline __device__ bool addBCForces_Sphere(const int sphXpos,
                                          const int sphYpos,
                                          const int sphZpos,
                                          const float sphXvel,
                                          const float sphYvel,
                                          const float sphZvel,
                                          const float sphOmegaX,
                                          const float sphOmegaY,
                                          const float sphOmegaZ,
                                          float3 force_from_BCs,
                                          float3 ang_acc_from_BCs,
                                          ParamsPtr gran_params,
                                          const BC_params_t<int, int3>& bc_params) {
    sphere_BC_params_t<int, int3> sphere_params = bc_params.sphere_params;
    bool contact = false;
    // classic radius grab, this must be signed to avoid false conversions
    const signed int sphereRadius_SU = (signed int)gran_params->sphereRadius_SU;
    // sum of two radii
    float total_diameter = (sphere_params.radius + sphereRadius_SU);
    float inv_diameter = 1.f / (total_diameter);

    float reciplength = 0;

    {
        double dX = (sphXpos - sphere_params.sphere_center.x) * inv_diameter;
        double dY = (sphYpos - sphere_params.sphere_center.y) * inv_diameter;
        double dZ = (sphZpos - sphere_params.sphere_center.z) * inv_diameter;
        double d2 = dX * dX + dY * dY + dZ * dZ;
        // this needs to be computed in double, then cast to float
        reciplength = (float)rsqrt(d2);
    }
    // recompute in float to be cheaper
    float dX = (sphXpos - sphere_params.sphere_center.x) * inv_diameter;
    float dY = (sphYpos - sphere_params.sphere_center.y) * inv_diameter;
    float dZ = (sphZpos - sphere_params.sphere_center.z) * inv_diameter;

    float penetration = reciplength - 1.;
    // contact means d2 <1, so 1/d2 > 1, reciplength > 1, penetration > 0
    if (penetration > 0) {
        contact = true;
        // spring term
        force_from_BCs.x += sphere_params.normal_sign * gran_params->K_n_s2w_SU * dX * total_diameter * penetration;
        force_from_BCs.y += sphere_params.normal_sign * gran_params->K_n_s2w_SU * dY * total_diameter * penetration;
        force_from_BCs.z += sphere_params.normal_sign * gran_params->K_n_s2w_SU * dZ * total_diameter * penetration;

        // damping term
        // Compute force updates for damping term
        // Project relative velocity to the normal
        // n = delta * reciplength
        // proj = Dot(delta_dot, n)
        // TODO this assumes walls at rest
        float projection = (sphXvel * dX + sphYvel * dY + sphZvel * dZ) * reciplength;

        constexpr float m_eff = 0.5;

        force_from_BCs.x += -gran_params->Gamma_n_s2w_SU * projection * dX * reciplength * m_eff;
        force_from_BCs.y += -gran_params->Gamma_n_s2w_SU * projection * dY * reciplength * m_eff;
        force_from_BCs.z += -gran_params->Gamma_n_s2w_SU * projection * dZ * reciplength * m_eff;
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
                                        const float sphOmegaX,
                                        const float sphOmegaY,
                                        const float sphOmegaZ,
                                        float3 force_from_BCs,
                                        float3 ang_acc_from_BCs,
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
        force_from_BCs = force_from_BCs + cone_params.normal_sign * gran_params->K_n_s2w_SU * pen * contact_vector;

        // damping term
        // Compute force updates for damping term
        // Project relative velocity to the normal
        // assume static BC
        float projection = (sphXvel * contact_vector.x + sphYvel * contact_vector.y + sphZvel * contact_vector.z);

        constexpr float m_eff = 0.5;

        force_from_BCs = force_from_BCs + -gran_params->Gamma_n_s2w_SU * projection * contact_vector * m_eff;
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
                                         const float sphOmegaX,
                                         const float sphOmegaY,
                                         const float sphOmegaZ,
                                         float3 force_from_BCs,
                                         float3 ang_acc_from_BCs,
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
        force_from_BCs = force_from_BCs + gran_params->K_n_s2w_SU * penetration * plane_params.normal;

        float3 sphere_vel = make_float3(sphXvel, sphYvel, sphZvel);

        // add sphere rotation
        if (gran_params->friction_mode != chrono::granular::GRAN_FRICTION_MODE::FRICTIONLESS) {
            // v = v_COM + omega cross r
            sphere_vel = sphere_vel + Cross(make_float3(sphOmegaX, sphOmegaY, sphOmegaZ), -1 * delta_r);
        }

        // damping term
        // Compute force updates for damping term
        // Project relative velocity to the normal
        // assume static BC
        float projection = Dot(sphere_vel, plane_params.normal);

        // now this is just the tangential component
        sphere_vel = sphere_vel - projection * plane_params.normal;

        constexpr float m_eff = 0.5;

        // add tangent forces
        if (gran_params->friction_mode == chrono::granular::GRAN_FRICTION_MODE::SINGLE_STEP) {
            const float combined_tangent_coeff =
                gran_params->K_t_s2s_SU * gran_params->alpha_h_bar + gran_params->Gamma_t_s2s_SU * m_eff;

            // float3 tangent_force = -combined_tangent_coeff * sphere_vel * force_model_multiplier;
            // TODO ADD hertz!

            // v = v_COM + omega cross r
            sphere_vel = sphere_vel + Cross(make_float3(sphOmegaX, sphOmegaY, sphOmegaZ), -1 * delta_r);
        }

        force_from_BCs = force_from_BCs + -1 * gran_params->Gamma_n_s2w_SU * projection * plane_params.normal * m_eff;
    }

    return contact;
}