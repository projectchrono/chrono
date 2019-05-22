// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Conlain Kelly, Nic Olsen, Dan Negrut
// =============================================================================

#pragma once

#include "chrono_granular/ChGranularDefines.h"
#include "chrono_granular/utils/ChCudaMathUtils.cuh"
#include "chrono_granular/physics/ChGranularBoundaryConditions.h"
#include "chrono_granular/physics/ChGranularHelpers.cuh"

using chrono::granular::GRAN_TIME_INTEGRATOR;
using chrono::granular::GRAN_FRICTION_MODE;
using chrono::granular::GRAN_ROLLING_MODE;

using chrono::granular::BC_type;
using chrono::granular::BC_params_t;
using chrono::granular::Sphere_BC_params_t;
using chrono::granular::Z_Cone_BC_params_t;
using chrono::granular::Z_Cylinder_BC_params_t;
using chrono::granular::Plane_BC_params_t;

inline __device__ bool addBCForces_Sphere_frictionless(const int64_t3& sphPos,
                                                       const float3& sphVel,
                                                       float3& force_from_BCs,
                                                       GranParamsPtr gran_params,
                                                       BC_params_t<int64_t, int64_t3>& bc_params,
                                                       bool track_forces) {
    Sphere_BC_params_t<int64_t, int64_t3> sphere_params = bc_params.sphere_params;
    bool contact = false;
    // classic radius grab, this must be signed to avoid false conversions
    const signed int sphereRadius_SU = (signed int)gran_params->sphereRadius_SU;

    float reciplength = 0;

    // precompute the int offset
    int64_t3 delta_int = sphPos - sphere_params.sphere_center;

    {
        // TODO is double even necessary
        double3 delta = int64_t3_to_double3(delta_int) / (sphere_params.radius + sphereRadius_SU);
        double d2 = Dot(delta, delta);
        // this needs to be computed in double, then cast to float
        reciplength = (float)rsqrt(d2);
    }
    // recompute in float to be cheaper
    float3 delta = int64_t3_to_float3(delta_int) / (sphere_params.radius + sphereRadius_SU);

    float3 contact_normal = delta * reciplength;

    float penetration_over_R = 2. - 2. / reciplength;
    contact = (penetration_over_R > 0);
    // contact means d2 <1, so 1/d2 > 1, reciplength > 1, penetration_over_R > 0
    if (contact) {
        float3 force_accum = {0, 0, 0};

        float force_model_multiplier = sqrt(penetration_over_R);

        // spring term
        force_accum = force_accum + sphere_params.normal_sign * gran_params->K_n_s2w_SU * contact_normal * 0.5 *
                                        (sphere_params.radius + sphereRadius_SU) * penetration_over_R *
                                        force_model_multiplier;

        // Project relative velocity to the normal
        float3 rel_vel = sphVel - bc_params.vel_SU;
        float projection = Dot(rel_vel, contact_normal);

        // assume bc mass is infinite
        constexpr float m_eff = gran_params->sphere_mass_SU;
        // add damping term
        force_accum =
            force_accum + -gran_params->Gamma_n_s2w_SU * projection * contact_normal * m_eff * force_model_multiplier;

        force_from_BCs = force_from_BCs + force_accum;
        if (track_forces) {
            atomicAdd(&(bc_params.reaction_forces.x), -force_accum.x);
            atomicAdd(&(bc_params.reaction_forces.y), -force_accum.y);
            atomicAdd(&(bc_params.reaction_forces.z), -force_accum.z);
        }
    }

    return contact;
}

/// compute frictionless cone normal forces
// NOTE: overloaded below
inline __device__ bool addBCForces_ZCone_frictionless(const int64_t3& sphPos,
                                                      const float3& sphVel,
                                                      float3& force_from_BCs,
                                                      GranParamsPtr gran_params,
                                                      BC_params_t<int64_t, int64_t3>& bc_params,
                                                      bool track_forces,
                                                      float3& contact_normal,
                                                      float& dist) {
    Z_Cone_BC_params_t<int64_t, int64_t3> cone_params = bc_params.cone_params;
    bool contact = false;
    // classic radius grab, this must be signed to avoid false conversions
    const signed int sphereRadius_SU = (signed int)gran_params->sphereRadius_SU;

    // no contact here
    if (sphPos.z >= cone_params.hmax || sphPos.z <= cone_params.hmin) {
        return false;
    }

    // Get vector from cone tip to sphere center
    // TODO are we concerned about large floats here???
    float3 sphere_pos_rel = int64_t3_to_float3(sphPos - cone_params.cone_tip);

    // NOTE that this could get ugly if Px, Py are very small
    // get point P on cone directly below sphere
    float Px = sphere_pos_rel.x;
    float Py = sphere_pos_rel.y;
    float Pz = cone_params.slope * sqrt(Px * Px + Py * Py);

    // line from tip to P
    float3 l = make_float3(Px, Py, Pz);

    // component of sphere_pos_rel tangent to cone along line l
    float3 contact_tangent = l * Dot(sphere_pos_rel, l) / Dot(l, l);

    // vector from contact point to sphere
    float3 contact_vector = sphere_pos_rel - contact_tangent;

    dist = Length(contact_vector);

    // give us a contact normal
    contact_normal = contact_vector / dist;

    // positive means we are penetrating
    float penetration = sphereRadius_SU - dist;

    contact = (penetration > 0);

    // if penetrating and the material is inside (not above or below) the cone, add forces
    if (contact) {
        float3 force_accum = {0, 0, 0};
        float force_model_multiplier = sqrt(penetration / sphereRadius_SU);

        // add spring term
        force_accum = force_accum + cone_params.normal_sign * gran_params->K_n_s2w_SU * penetration * contact_normal *
                                        force_model_multiplier;

        // damping term
        // Project relative velocity to the normal
        float3 rel_vel = sphVel - bc_params.vel_SU;
        float projection = Dot(rel_vel, contact_normal);

        // assume bc mass is infinite
        constexpr float m_eff = gran_params->sphere_mass_SU;

        // Compute force updates for damping term
        force_accum =
            force_accum + -gran_params->Gamma_n_s2w_SU * projection * contact_normal * m_eff * force_model_multiplier;
        force_from_BCs = force_from_BCs + force_accum;
        if (track_forces) {
            atomicAdd(&(bc_params.reaction_forces.x), -force_accum.x);
            atomicAdd(&(bc_params.reaction_forces.y), -force_accum.y);
            atomicAdd(&(bc_params.reaction_forces.z), -force_accum.z);
        }
    }

    return contact;
}
// overload of above if we don't care about dist and contact normal
inline __device__ bool addBCForces_ZCone_frictionless(const int64_t3& sphPos,
                                                      const float3& sphVel,
                                                      float3& force_from_BCs,
                                                      GranParamsPtr gran_params,
                                                      BC_params_t<int64_t, int64_t3>& bc_params,
                                                      bool track_forces) {
    float3 contact_normal = {0, 0, 0};
    float dist;
    return addBCForces_ZCone_frictionless(sphPos, sphVel, force_from_BCs, gran_params, bc_params, track_forces,
                                          contact_normal, dist);
}

/// TODO check damping, adhesion
inline __device__ bool addBCForces_ZCone(unsigned int sphID,
                                         unsigned int BC_id,
                                         const int64_t3& sphPos,
                                         const float3& sphVel,
                                         const float3& sphOmega,
                                         float3& force_from_BCs,
                                         float3& ang_acc_from_BCs,
                                         GranParamsPtr gran_params,
                                         GranSphereDataPtr sphere_data,
                                         BC_params_t<int64_t, int64_t3>& bc_params,
                                         bool track_forces) {
    // determine these from frictionless helper
    float3 force_accum = {0, 0, 0};
    float3 contact_normal = {0, 0, 0};

    const signed int sphereRadius_SU = (signed int)gran_params->sphereRadius_SU;

    // distance of penetration
    float dist = 0;
    // determine whether we are in contact
    bool contact = addBCForces_ZCone_frictionless(sphPos, sphVel, force_accum, gran_params, bc_params, false,
                                                  contact_normal, dist);

    if (contact) {
        // add tangent forces
        if (gran_params->friction_mode != chrono::granular::GRAN_FRICTION_MODE::FRICTIONLESS) {
            float projection = Dot(sphVel, contact_normal);
            float3 sphere_vel_rel =
                sphVel - bc_params.vel_SU - contact_normal * projection + Cross(sphOmega, -1. * dist * contact_normal);

            float force_model_multiplier = sqrt((sphereRadius_SU - dist) / sphereRadius_SU);
            unsigned int BC_histmap_label = gran_params->nSpheres + BC_id + 1;

            float3 roll_acc = computeRollingAngAcc(sphere_data, gran_params, gran_params->rolling_coeff_s2w_SU,
                                                   force_accum, sphOmega, make_float3(0, 0, 0), dist * contact_normal);

            // assume bc mass is infinite
            constexpr float m_eff = gran_params->sphere_mass_SU;

            // compute tangent force
            float3 tangent_force = computeFrictionForces(
                gran_params, sphere_data, sphID, BC_histmap_label, gran_params->static_friction_coeff_s2w,
                gran_params->K_t_s2w_SU, gran_params->Gamma_t_s2w_SU, force_model_multiplier, m_eff, force_accum,
                sphere_vel_rel, contact_normal);

            ang_acc_from_BCs =
                ang_acc_from_BCs + (Cross(-1 * contact_normal, tangent_force) / gran_params->sphereInertia_by_r);
            ang_acc_from_BCs = ang_acc_from_BCs + roll_acc;
            force_accum = force_accum + tangent_force;
        }

        force_from_BCs = force_from_BCs + force_accum;
        if (track_forces) {
            atomicAdd(&(bc_params.reaction_forces.x), -force_accum.x);
            atomicAdd(&(bc_params.reaction_forces.y), -force_accum.y);
            atomicAdd(&(bc_params.reaction_forces.z), -force_accum.z);
        }
    }

    return contact;
}

/// TODO check damping, adhesion
inline __device__ bool addBCForces_Plane_frictionless(const int64_t3& sphPos,
                                                      const float3& sphVel,
                                                      float3& force_from_BCs,
                                                      GranParamsPtr gran_params,
                                                      BC_params_t<int64_t, int64_t3>& bc_params,
                                                      bool track_forces,
                                                      float& dist) {
    Plane_BC_params_t<int64_t3> plane_params = bc_params.plane_params;
    bool contact = false;
    // classic radius grab, this must be signed to avoid false conversions
    const signed int sphereRadius_SU = (signed int)gran_params->sphereRadius_SU;

    // Vector from point on plane to sphere center
    float3 delta_r = int64_t3_to_float3(sphPos - plane_params.position);

    // projection displacement onto plane normal
    dist = Dot(plane_params.normal, delta_r);
    // positive implies radius is bigger than distance, so there is penetration
    float penetration = sphereRadius_SU - dist;
    contact = (penetration > 0);

    if (contact) {
        float3 force_accum = {0, 0, 0};

        float3 contact_normal = plane_params.normal;
        float force_model_multiplier = sqrt(penetration / sphereRadius_SU);
        force_accum = gran_params->K_n_s2w_SU * penetration * contact_normal;

        float3 rel_vel = sphVel - bc_params.vel_SU;

        // project velocity onto the normal
        float projection = Dot(rel_vel, contact_normal);

        // assume bc mass is infinite
        constexpr float m_eff = gran_params->sphere_mass_SU;

        // damping term
        force_accum = force_accum + -1. * gran_params->Gamma_n_s2w_SU * projection * contact_normal * m_eff;
        force_accum = force_accum * force_model_multiplier;

        force_from_BCs = force_from_BCs + force_accum;
        if (track_forces) {
            atomicAdd(&(bc_params.reaction_forces.x), -force_accum.x);
            atomicAdd(&(bc_params.reaction_forces.y), -force_accum.y);
            atomicAdd(&(bc_params.reaction_forces.z), -force_accum.z);
        }
    }

    return contact;
}

/// overload of above in case we don't care about dist
inline __device__ bool addBCForces_Plane_frictionless(const int64_t3& sphPos,
                                                      const float3& sphVel,
                                                      float3& force_from_BCs,
                                                      GranParamsPtr gran_params,
                                                      BC_params_t<int64_t, int64_t3>& bc_params,
                                                      bool track_forces) {
    float dist;
    return addBCForces_Plane_frictionless(sphPos, sphVel, force_from_BCs, gran_params, bc_params, track_forces, dist);
}

/// TODO check damping, adhesion
inline __device__ bool addBCForces_Plane(unsigned int sphID,
                                         unsigned int BC_id,
                                         const int64_t3& sphPos,
                                         const float3& sphVel,
                                         const float3& sphOmega,
                                         float3& force_from_BCs,
                                         float3& ang_acc_from_BCs,
                                         GranParamsPtr gran_params,
                                         GranSphereDataPtr sphere_data,
                                         BC_params_t<int64_t, int64_t3>& bc_params,
                                         bool track_forces) {
    float3 force_accum = {0, 0, 0};
    float3 contact_normal = bc_params.plane_params.normal;

    const signed int sphereRadius_SU = (signed int)gran_params->sphereRadius_SU;

    float dist = 0;

    bool contact = addBCForces_Plane_frictionless(sphPos, sphVel, force_accum, gran_params, bc_params, false, dist);

    // if we had normal forces, and friction is on, compute tangential forces
    if (contact) {
        float penetration = sphereRadius_SU - dist;
        float projection = Dot(sphVel, contact_normal);
        float3 rel_vel = sphVel - contact_normal * projection + Cross(sphOmega, -1. * dist * contact_normal);
        float force_model_multiplier = sqrt(penetration / sphereRadius_SU);

        // add tangent forces
        if (gran_params->friction_mode != chrono::granular::GRAN_FRICTION_MODE::FRICTIONLESS) {
            unsigned int BC_histmap_label = gran_params->nSpheres + BC_id + 1;

            // assume bc mass is infinite
            constexpr float m_eff = gran_params->sphere_mass_SU;

            // compute tangent force
            float3 tangent_force = computeFrictionForces(gran_params, sphere_data, sphID, BC_histmap_label,
                                                         gran_params->rolling_coeff_s2w_SU, gran_params->K_t_s2w_SU,
                                                         gran_params->Gamma_t_s2w_SU, force_model_multiplier, m_eff,
                                                         force_accum, rel_vel, contact_normal);

            float3 roll_acc = computeRollingAngAcc(sphere_data, gran_params, gran_params->static_friction_coeff_s2w,
                                                   force_accum, sphOmega, make_float3(0, 0, 0), dist * contact_normal);

            ang_acc_from_BCs =
                ang_acc_from_BCs + (Cross(-1 * contact_normal, tangent_force) / gran_params->sphereInertia_by_r);
            ang_acc_from_BCs = ang_acc_from_BCs + roll_acc;
            force_accum = force_accum + tangent_force;
        }

        force_from_BCs = force_from_BCs + force_accum;
        if (track_forces) {
            atomicAdd(&(bc_params.reaction_forces.x), -force_accum.x);
            atomicAdd(&(bc_params.reaction_forces.y), -force_accum.y);
            atomicAdd(&(bc_params.reaction_forces.z), -force_accum.z);
        }
    }

    return contact;
}

/// TODO check damping, adhesion
inline __device__ bool addBCForces_Zcyl_frictionless(const int64_t3& sphPos,
                                                     const float3& sphVel,
                                                     float3& force_from_BCs,
                                                     GranParamsPtr gran_params,
                                                     BC_params_t<int64_t, int64_t3>& bc_params,
                                                     bool track_forces,
                                                     float3& contact_normal,
                                                     float& dist) {
    Z_Cylinder_BC_params_t<int64_t, int64_t3> cyl_params = bc_params.cyl_params;
    bool contact = false;
    // classic radius grab
    signed int sphereRadius_SU = (signed int)gran_params->sphereRadius_SU;

    // Radial vector from cylinder center to sphere center, along inward direction
    float3 delta_r = make_float3(cyl_params.center.x - sphPos.x, cyl_params.center.y - sphPos.y, 0.f);
    dist = Length(delta_r);

    // directional normal
    contact_normal = cyl_params.normal_sign * delta_r / dist;

    // get penetration into cylinder
    float penetration = sphereRadius_SU - abs(cyl_params.radius - dist);
    contact = (penetration > 0);

    // if penetrating and the material is inside (not above or below) the cone, add forces
    if (contact) {
        float force_model_multiplier = sqrt(penetration / sphereRadius_SU);

        // add spring term
        float3 force_accum = gran_params->K_n_s2w_SU * penetration * contact_normal * force_model_multiplier;

        // damping term
        // Compute force updates for damping term
        // Project relative velocity to the normal
        // assume static BC
        float3 rel_vel = {sphVel.x - bc_params.vel_SU.x, sphVel.y - bc_params.vel_SU.y, 0};
        float projection = Dot(rel_vel, contact_normal);

        // assume bc mass is infinite
        constexpr float m_eff = gran_params->sphere_mass_SU;

        force_accum =
            force_accum + -gran_params->Gamma_n_s2w_SU * projection * contact_normal * m_eff * force_model_multiplier;

        force_from_BCs = force_from_BCs + force_accum;
        if (track_forces) {
            atomicAdd(&(bc_params.reaction_forces.x), -force_accum.x);
            atomicAdd(&(bc_params.reaction_forces.y), -force_accum.y);
            atomicAdd(&(bc_params.reaction_forces.z), -force_accum.z);
        }
    }
    return contact;
}

/// minimal overload for dist and contact_normal params
inline __device__ bool addBCForces_Zcyl_frictionless(const int64_t3& sphPos,
                                                     const float3& sphVel,
                                                     float3& force_from_BCs,
                                                     GranParamsPtr gran_params,
                                                     BC_params_t<int64_t, int64_t3>& bc_params,
                                                     bool track_forces) {
    float3 contact_normal = {0, 0, 0};
    float dist;
    return addBCForces_Zcyl_frictionless(sphPos, sphVel, force_from_BCs, gran_params, bc_params, track_forces,
                                         contact_normal, dist);
}

/// TODO check damping, adhesion
inline __device__ bool addBCForces_Zcyl(unsigned int sphID,
                                        unsigned int BC_id,
                                        const int64_t3& sphPos,
                                        const float3& sphVel,
                                        const float3& sphOmega,
                                        float3& force_from_BCs,
                                        float3& ang_acc_from_BCs,
                                        GranParamsPtr gran_params,
                                        GranSphereDataPtr sphere_data,
                                        BC_params_t<int64_t, int64_t3>& bc_params,
                                        bool track_forces) {
    float3 force_accum = {0, 0, 0};
    float3 contact_normal;

    const signed int sphereRadius_SU = (signed int)gran_params->sphereRadius_SU;

    float dist = 0;

    bool contact =
        addBCForces_Zcyl_frictionless(sphPos, sphVel, force_accum, gran_params, bc_params, false, contact_normal, dist);

    // if we had normal forces, and friction is on, compute tangential forces
    if (contact) {
        float penetration = sphereRadius_SU - dist;
        float projection = Dot(sphVel, contact_normal);
        float3 rel_vel = sphVel - contact_normal * projection + Cross(sphOmega, -1. * dist * contact_normal);
        float force_model_multiplier = sqrt(penetration / sphereRadius_SU);

        // add tangent forces
        if (gran_params->friction_mode != chrono::granular::GRAN_FRICTION_MODE::FRICTIONLESS) {
            unsigned int BC_histmap_label = gran_params->nSpheres + BC_id + 1;

            float3 roll_acc = computeRollingAngAcc(sphere_data, gran_params, gran_params->rolling_coeff_s2w_SU,
                                                   force_accum, sphOmega, make_float3(0, 0, 0), dist * contact_normal);

            // assume bc mass is infinite
            constexpr float m_eff = gran_params->sphere_mass_SU;

            // compute tangent force
            float3 tangent_force = computeFrictionForces(
                gran_params, sphere_data, sphID, BC_histmap_label, gran_params->static_friction_coeff_s2w,
                gran_params->K_t_s2w_SU, gran_params->Gamma_t_s2w_SU, force_model_multiplier, m_eff, force_accum,
                rel_vel, contact_normal);

            ang_acc_from_BCs =
                ang_acc_from_BCs + (Cross(-1 * contact_normal, tangent_force) / gran_params->sphereInertia_by_r);
            ang_acc_from_BCs = ang_acc_from_BCs + roll_acc;

            force_accum = force_accum + tangent_force;
        }

        force_from_BCs = force_from_BCs + force_accum;
        if (track_forces) {
            atomicAdd(&(bc_params.reaction_forces.x), -force_accum.x);
            atomicAdd(&(bc_params.reaction_forces.y), -force_accum.y);
            atomicAdd(&(bc_params.reaction_forces.z), -force_accum.z);
        }
    }
    return contact;
}