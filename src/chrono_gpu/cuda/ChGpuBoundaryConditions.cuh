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

#include "chrono_gpu/ChGpuDefines.h"
#include "chrono_gpu/physics/ChSystemGpu_impl.h"
#include "chrono_gpu/physics/ChGpuBoundaryConditions.h"
#include "chrono_gpu/cuda/ChCudaMathUtils.cuh"
#include "chrono_gpu/cuda/ChGpuHelpers.cuh"
//#include "chrono/core/ChMathematics.h"
#include <math_constants.h>
using chrono::gpu::CHGPU_TIME_INTEGRATOR;
using chrono::gpu::CHGPU_FRICTION_MODE;
using chrono::gpu::CHGPU_ROLLING_MODE;

using chrono::gpu::BC_type;
using chrono::gpu::BC_params_t;
using chrono::gpu::Sphere_BC_params_t;
using chrono::gpu::Z_Cone_BC_params_t;
using chrono::gpu::Z_Cylinder_BC_params_t;
using chrono::gpu::Plane_BC_params_t;

using chrono::gpu::ChSystemGpu_impl;

// add bc forces material based only
inline __device__ bool addBCForces_Sphere_matBased(unsigned int sphID,
                                                   unsigned int BC_id,
                                                   const int64_t3& sphPos,
                                                   const float3& sphVel,
                                                   const float3& sphOmega,
                                                   float3& force_from_BCs,
                                                   float3& ang_acc_from_BCs,
                                                   ChSystemGpu_impl::GranParamsPtr gran_params,
                                                   ChSystemGpu_impl::GranSphereDataPtr sphere_data,
                                                   BC_params_t<int64_t, int64_t3>& bc_params,
                                                   bool track_forces) {
    Sphere_BC_params_t<int64_t, int64_t3> sphere_params = bc_params.sphere_params;
    bool contact = false;

    const signed int sphereRadius_SU = (signed int)gran_params->sphereRadius_SU;

    float penetration = 0;

    // precompute the int offset
    int64_t3 delta_int = sphPos - sphere_params.sphere_center;

    double sph_center_dist = sqrt(Dot(int64_t3_to_double3(delta_int), int64_t3_to_double3(delta_int)));

    float3 contact_normal = int64_t3_to_float3(delta_int) / sph_center_dist;

    if (sph_center_dist < (double)(sphere_params.radius + sphereRadius_SU)) {
        contact = true;
        penetration = (double)(sphere_params.radius + sphereRadius_SU) - sph_center_dist;
    }

    if (contact) {
        const float m_eff =
            (gran_params->sphere_mass_SU * sphere_params.mass) / (gran_params->sphere_mass_SU + sphere_params.mass);

        // normal force part
        float sqrt_Rd = sqrt(penetration * (float)sphereRadius_SU);
        float Sn = 2 * gran_params->E_eff_s2w_SU * sqrt_Rd;
        float loge = (gran_params->COR_s2w_SU < EPSILON) ? log(EPSILON) : log(gran_params->COR_s2w_SU);
        float beta = loge / sqrt(loge * loge + CUDART_PI_F * CUDART_PI_F);

        float kn = (2.0 / 3.0) * Sn;
        float gn = -2 * sqrt(5.0 / 6.0) * beta * sqrt(Sn * m_eff);

        // project velocity onto the normal
        float3 v_rel = sphVel - sphere_params.sphere_velo;
        float projection = Dot(v_rel, contact_normal);

        // tangential component of relative velocity
        float3 vrel_n = projection * contact_normal;
        float3 vrel_t = v_rel - vrel_n;

        float forceN_mag = kn * penetration - gn * projection;

        // damping term
        float3 force_accum = forceN_mag * contact_normal;

        // tangential force component
        unsigned int BC_histmap_label = gran_params->nSpheres + BC_id + 1;

        // tangential component without angular velocity component
        vrel_t = vrel_t + Cross(sphereRadius_SU * sphVel + sphere_params.radius * sphere_params.sphere_angularVelo,
                                contact_normal);

        // parameter force_accum as normal force, returned val as tangent force
        float3 tangent_force = computeFrictionForces_matBased(
            gran_params, sphere_data, sphID, BC_histmap_label, gran_params->static_friction_coeff_s2w,
            gran_params->E_eff_s2w_SU, gran_params->G_eff_s2w_SU, sqrt_Rd, beta, force_accum, vrel_t, contact_normal,
            m_eff);

        // TODO: use collision time to check whether or not to apply rolling friction
        // size_t contact_id = findContactPairInfo(sphere_data, gran_params, sphID, BC_histmap_label);

        // sphere_data->contact_duration[contact_id] += gran_params->stepSize_SU;

        // bool calc_rolling_fr = EvaluateRollingFriction(gran_params, gran_params->E_eff_s2w_SU, sphereRadius_SU, beta,
        // m_eff, sphere_data->contact_duration[contact_id]);

        float3 normalized_bc_omega = sphere_params.radius / (float)sphereRadius_SU * sphere_params.sphere_angularVelo;
        // // force_accum is normal force
        // if (calc_rolling_fr == true){
        float3 roll_acc = computeRollingAngAcc(sphere_data, gran_params, gran_params->rolling_coeff_s2w_SU,
                                               gran_params->spinning_coeff_s2w_SU, force_accum, sphOmega,
                                               normalized_bc_omega, (float)sphereRadius_SU * contact_normal);

        // } else {
        //     roll_acc = make_float3(0.0f, 0.0f, 0.0f);
        // }

        ang_acc_from_BCs =
            ang_acc_from_BCs + (Cross(-1 * contact_normal, tangent_force) / gran_params->sphereInertia_by_r);
        ang_acc_from_BCs = ang_acc_from_BCs + roll_acc;

        force_accum = force_accum + tangent_force;
        force_from_BCs = force_from_BCs + force_accum;

        float3 torque_accum = Cross(contact_normal, tangent_force) * sphere_params.radius -
                              roll_acc * gran_params->sphereInertia_by_r * sphere_params.radius;

        if (track_forces) {
            // accumulate force
            atomicAdd(&(bc_params.reaction_forces.x), -force_accum.x);
            atomicAdd(&(bc_params.reaction_forces.y), -force_accum.y);
            atomicAdd(&(bc_params.reaction_forces.z), -force_accum.z);

            // accumulate torque
            atomicAdd(&(bc_params.sphere_params.reaction_torques.x), torque_accum.x);
            atomicAdd(&(bc_params.sphere_params.reaction_torques.y), torque_accum.y);
            atomicAdd(&(bc_params.sphere_params.reaction_torques.z), torque_accum.z);
        }

        return true;
    }

    return false;
}

inline __device__ bool addBCForces_Sphere_frictionless(const int64_t3& sphPos,
                                                       const float3& sphVel,
                                                       float3& force_from_BCs,
                                                       ChSystemGpu_impl::GranParamsPtr gran_params,
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
        const float m_eff = gran_params->sphere_mass_SU;
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
                                                      ChSystemGpu_impl::GranParamsPtr gran_params,
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
        const float m_eff = gran_params->sphere_mass_SU;

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
                                                      ChSystemGpu_impl::GranParamsPtr gran_params,
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
                                         ChSystemGpu_impl::GranParamsPtr gran_params,
                                         ChSystemGpu_impl::GranSphereDataPtr sphere_data,
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
        if (gran_params->friction_mode != chrono::gpu::CHGPU_FRICTION_MODE::FRICTIONLESS) {
            float projection = Dot(sphVel, contact_normal);
            float3 sphere_vel_rel =
                sphVel - bc_params.vel_SU - contact_normal * projection + Cross(sphOmega, -1. * dist * contact_normal);

            float force_model_multiplier = sqrt((sphereRadius_SU - dist) / sphereRadius_SU);
            unsigned int BC_histmap_label = gran_params->nSpheres + BC_id + 1;

            float3 roll_acc = computeRollingAngAcc(sphere_data, gran_params, gran_params->rolling_coeff_s2w_SU,
                                                   gran_params->spinning_coeff_s2w_SU, force_accum, sphOmega,
                                                   make_float3(0, 0, 0), dist * contact_normal);
            // assume bc mass is infinite
            const float m_eff = gran_params->sphere_mass_SU;

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
                                                      ChSystemGpu_impl::GranParamsPtr gran_params,
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

        // point of contact
        float3 ct_point = (int64_t3_to_float3)(sphPos)-contact_normal * (float)(sphereRadius_SU);
        float3 bc_velo = bc_params.vel_SU + Cross(plane_params.angular_acc,
                                                  (ct_point - (int64_t3_to_float3)(plane_params.rotation_center)));

        float3 rel_vel = sphVel - bc_velo;

        // project velocity onto the normal
        float projection = Dot(rel_vel, contact_normal);

        // assume bc mass is infinite
        const float m_eff = gran_params->sphere_mass_SU;

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

/// LULUTODO: material_based model
inline __device__ bool addBCForces_Plane_frictionless_mbased(const int64_t3& sphPos,
                                                             const float3& sphVel,
                                                             float3& force_from_BCs,
                                                             ChSystemGpu_impl::GranParamsPtr gran_params,
                                                             BC_params_t<int64_t, int64_t3>& bc_params,
                                                             bool track_forces,
                                                             float& dist,
                                                             float& sqrt_Rd,
                                                             float& beta) {
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
        // assume bc mass is infinite
        const float m_eff = gran_params->sphere_mass_SU;

        // normal force part
        sqrt_Rd = sqrt(penetration * sphereRadius_SU);

        float Sn = 2 * gran_params->E_eff_s2w_SU * sqrt_Rd;
        float loge = (gran_params->COR_s2w_SU < EPSILON) ? log(EPSILON) : log(gran_params->COR_s2w_SU);
        beta = loge / sqrt(loge * loge + CUDART_PI_F * CUDART_PI_F);

        float kn = (2.0 / 3.0) * Sn;
        float gn = -2 * sqrt(5.0 / 6.0) * beta * sqrt(Sn * m_eff);

        float3 contact_normal = plane_params.normal;

        float3 ct_point = (int64_t3_to_float3)(sphPos)-contact_normal * (float)(sphereRadius_SU);
        float3 bc_velo = bc_params.vel_SU + Cross(plane_params.angular_acc,
                                                  (ct_point - (int64_t3_to_float3)(plane_params.rotation_center)));

        float3 rel_vel = sphVel - bc_velo;

        // project velocity onto the normal
        float projection = Dot(rel_vel, contact_normal);

        float forceN_mag = kn * penetration - gn * projection;

        // damping term
        force_accum = forceN_mag * contact_normal;

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
                                                      ChSystemGpu_impl::GranParamsPtr gran_params,
                                                      BC_params_t<int64_t, int64_t3>& bc_params,
                                                      bool track_forces) {
    float dist;
    return addBCForces_Plane_frictionless(sphPos, sphVel, force_from_BCs, gran_params, bc_params, track_forces, dist);
}

/// overload of above in case we don't care about dist, sqrt_Rd and beta
inline __device__ bool addBCForces_Plane_frictionless_mbased(const int64_t3& sphPos,
                                                             const float3& sphVel,
                                                             float3& force_from_BCs,
                                                             ChSystemGpu_impl::GranParamsPtr gran_params,
                                                             BC_params_t<int64_t, int64_t3>& bc_params,
                                                             bool track_forces) {
    float dist, sqrt_Rd, beta;
    return addBCForces_Plane_frictionless_mbased(sphPos, sphVel, force_from_BCs, gran_params, bc_params, track_forces,
                                                 dist, sqrt_Rd, beta);
}

inline __device__ bool EvaluateRollingFriction(ChSystemGpu_impl::GranParamsPtr gran_params,
                                               const float& E_eff,
                                               const float& R_eff,
                                               const float& beta,
                                               const float& m_eff,
                                               const float& time_contact) {
    float kn_simple = 4.f / 3.f * E_eff * sqrtf(R_eff);
    float gn_simple = -2.f * sqrtf(5.f / 3.f * m_eff * E_eff) * beta * powf(R_eff, 1.f / 4.f);

    float d_coeff = gn_simple / (2.f * sqrtf(kn_simple * m_eff));

    if (d_coeff < 1) {
        float t_collision = CUDART_PI_F * sqrtf(m_eff / (kn_simple * (1.f - d_coeff * d_coeff)));
        if (time_contact <= t_collision * powf(gran_params->LENGTH_UNIT, 0.25f)) {

            return false;
        }
    }
    return true;
}

inline __device__ bool addBCForces_Plane(unsigned int sphID,
                                         unsigned int BC_id,
                                         const int64_t3& sphPos,
                                         const float3& sphVel,
                                         const float3& sphOmega,
                                         float3& force_from_BCs,
                                         float3& ang_acc_from_BCs,
                                         ChSystemGpu_impl::GranParamsPtr gran_params,
                                         ChSystemGpu_impl::GranSphereDataPtr sphere_data,
                                         BC_params_t<int64_t, int64_t3>& bc_params,
                                         bool track_forces) {
    float3 force_accum = {0, 0, 0};
    float3 contact_normal = bc_params.plane_params.normal;

    const signed int sphereRadius_SU = (signed int)gran_params->sphereRadius_SU;

    float dist = 0;
    float beta;
    float sqrt_Rd;

    bool contact;
    if (gran_params->use_mat_based == true) {
        contact = addBCForces_Plane_frictionless_mbased(sphPos, sphVel, force_accum, gran_params, bc_params, false,
                                                        dist, sqrt_Rd, beta);
    } else {
        contact = addBCForces_Plane_frictionless(sphPos, sphVel, force_accum, gran_params, bc_params, false, dist);
    }

    // if we had normal forces, and friction is on, compute tangential forces
    if (contact) {
        float3 ct_point = (int64_t3_to_float3)(sphPos)-contact_normal * (float)(sphereRadius_SU);
        float3 bc_velo = Cross(bc_params.plane_params.angular_acc,
                               (ct_point - (int64_t3_to_float3)(bc_params.plane_params.rotation_center)));

        // float penetration = sphereRadius_SU - dist;
        float projection = Dot(sphVel - bc_velo, contact_normal);
        float3 rel_vel = sphVel - bc_velo - contact_normal * projection + Cross(sphOmega, -1. * dist * contact_normal);

        // add tangent forces
        if (gran_params->friction_mode != chrono::gpu::CHGPU_FRICTION_MODE::FRICTIONLESS) {
            unsigned int BC_histmap_label = gran_params->nSpheres + BC_id + 1;

            // assume bc mass is infinite
            const float m_eff = gran_params->sphere_mass_SU;

            // compute tangent force
            float3 tangent_force;
            float3 roll_acc;
            if (gran_params->use_mat_based == true) {
                tangent_force = computeFrictionForces_matBased(
                    gran_params, sphere_data, sphID, BC_histmap_label, gran_params->static_friction_coeff_s2w,
                    gran_params->E_eff_s2w_SU, gran_params->G_eff_s2w_SU, sqrt_Rd, beta, force_accum, rel_vel,
                    contact_normal, m_eff);

                size_t contact_id = findContactPairInfo(sphere_data, gran_params, sphID, BC_histmap_label);

                sphere_data->contact_duration[contact_id] += gran_params->stepSize_SU;

                bool calc_rolling_fr = EvaluateRollingFriction(gran_params, gran_params->E_eff_s2w_SU, sphereRadius_SU,
                                                               beta, m_eff, sphere_data->contact_duration[contact_id]);

                if (calc_rolling_fr == true) {
                    roll_acc = computeRollingAngAcc(sphere_data, gran_params, gran_params->rolling_coeff_s2w_SU,
                                                    gran_params->spinning_coeff_s2w_SU, force_accum, sphOmega,
                                                    make_float3(0, 0, 0), dist * contact_normal);

                } else {
                    roll_acc = make_float3(0.0f, 0.0f, 0.0f);
                }

                // write normal and tangential force for recording
                if (gran_params->recording_contactInfo == true) {
                    sphere_data->normal_contact_force[contact_id] = force_accum;
                    sphere_data->tangential_friction_force[contact_id] = tangent_force;
                }

            } else {
                float penetration = sphereRadius_SU - dist;
                float force_model_multiplier = sqrt(penetration / sphereRadius_SU);

                tangent_force = computeFrictionForces(gran_params, sphere_data, sphID, BC_histmap_label,
                                                      gran_params->static_friction_coeff_s2w, gran_params->K_t_s2w_SU,
                                                      gran_params->Gamma_t_s2w_SU, force_model_multiplier, m_eff,
                                                      force_accum, rel_vel, contact_normal);

                roll_acc = computeRollingAngAcc(sphere_data, gran_params, gran_params->rolling_coeff_s2w_SU,
                                                gran_params->spinning_coeff_s2w_SU, force_accum, sphOmega,
                                                make_float3(0, 0, 0), dist * contact_normal);
            }

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

inline __device__ bool addBCForces_Zcyl_frictionless(const int64_t3& sphPos,
                                                     const float3& sphVel,
                                                     float3& force_from_BCs,
                                                     ChSystemGpu_impl::GranParamsPtr gran_params,
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
    float dist_delta_r = Length(delta_r);

    // directional normal
    contact_normal = cyl_params.normal_sign * delta_r / dist_delta_r;

    // get penetration into cylinder
    float penetration = sphereRadius_SU - abs(cyl_params.radius - dist_delta_r);

    contact = (penetration > 0);

    // if penetrating and the material is inside the cylinder, add forces
    if (contact) {
        dist = cyl_params.radius - dist_delta_r;
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
        const float m_eff = gran_params->sphere_mass_SU;

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

inline __device__ bool addBCForces_Zcyl_frictionless_mbased(const int64_t3& sphPos,
                                                            const float3& sphVel,
                                                            float3& force_from_BCs,
                                                            ChSystemGpu_impl::GranParamsPtr gran_params,
                                                            BC_params_t<int64_t, int64_t3>& bc_params,
                                                            bool track_forces,
                                                            float& dist,
                                                            float& sqrt_Rd,
                                                            float& beta) {
    Z_Cylinder_BC_params_t<int64_t, int64_t3> cyl_params = bc_params.cyl_params;
    bool contact = false;
    // classic radius grab
    signed int sphereRadius_SU = (signed int)gran_params->sphereRadius_SU;

    // Radial vector from cylinder center to sphere center, along inward direction
    float3 delta_r = make_float3(cyl_params.center.x - sphPos.x, cyl_params.center.y - sphPos.y, 0.f);
    float dist_delta_r = Length(delta_r);

    // directional normal
    float3 contact_normal = cyl_params.normal_sign * delta_r / dist_delta_r;

    // get penetration into cylinder
    float penetration = sphereRadius_SU - abs(cyl_params.radius - dist_delta_r);

    contact = (penetration > 0);

    // if penetrating and the material is inside the cylinder, add forces
    if (contact) {
        dist = cyl_params.radius - dist_delta_r;

        const float m_eff = gran_params->sphere_mass_SU;

        // normal force part
        sqrt_Rd = sqrt(penetration * sphereRadius_SU);

        float Sn = 2 * gran_params->E_eff_s2w_SU * sqrt_Rd;
        float loge = (gran_params->COR_s2w_SU < EPSILON) ? log(EPSILON) : log(gran_params->COR_s2w_SU);
        beta = loge / sqrt(loge * loge + CUDART_PI_F * CUDART_PI_F);

        float kn = (2.0 / 3.0) * Sn;
        float gn = -2 * sqrt(5.0 / 6.0) * beta * sqrt(Sn * m_eff);

        // project velocity onto the normal
        float projection = Dot(sphVel, contact_normal);
        float forceN_mag = kn * penetration - gn * projection;

        // damping term
        float3 force_accum = forceN_mag * contact_normal;
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
                                                     ChSystemGpu_impl::GranParamsPtr gran_params,
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
                                        ChSystemGpu_impl::GranParamsPtr gran_params,
                                        ChSystemGpu_impl::GranSphereDataPtr sphere_data,
                                        BC_params_t<int64_t, int64_t3>& bc_params,
                                        bool track_forces) {
    float3 force_accum = {0, 0, 0};
    float3 contact_normal = {0, 0, 0};

    const signed int sphereRadius_SU = (signed int)gran_params->sphereRadius_SU;

    float dist = 0;
    float beta = 0;
    float sqrt_Rd = 0;

    bool contact;
    if (gran_params->use_mat_based == true) {
        contact = addBCForces_Zcyl_frictionless_mbased(sphPos, sphVel, force_accum, gran_params, bc_params,
                                                       track_forces, dist, sqrt_Rd, beta);

    } else {
        contact = addBCForces_Zcyl_frictionless(sphPos, sphVel, force_accum, gran_params, bc_params, false,
                                                contact_normal, dist);
    }

    // if we had normal forces, and friction is on, compute tangential forces
    if (contact) {
        float projection = Dot(sphVel, contact_normal);
        float3 rel_vel = sphVel - contact_normal * projection + Cross(sphOmega, -1. * dist * contact_normal);

        // add tangent forces
        if (gran_params->friction_mode != chrono::gpu::CHGPU_FRICTION_MODE::FRICTIONLESS) {
            unsigned int BC_histmap_label = gran_params->nSpheres + BC_id + 1;
            // assume bc mass is infinite
            const float m_eff = gran_params->sphere_mass_SU;

            // compute tangent force
            float3 tangent_force = {0.f, 0.f, 0.f};
            float3 roll_acc = {0.f, 0.f, 0.f};

            if (gran_params->use_mat_based == true) {
                tangent_force = computeFrictionForces_matBased(
                    gran_params, sphere_data, sphID, BC_histmap_label, gran_params->static_friction_coeff_s2w,
                    gran_params->E_eff_s2w_SU, gran_params->G_eff_s2w_SU, sqrt_Rd, beta, force_accum, rel_vel,
                    contact_normal, m_eff);

                size_t contact_id = findContactPairInfo(sphere_data, gran_params, sphID, BC_histmap_label);

                sphere_data->contact_duration[contact_id] += gran_params->stepSize_SU;

                bool calc_rolling_fr = EvaluateRollingFriction(gran_params, gran_params->E_eff_s2w_SU, sphereRadius_SU,
                                                               beta, m_eff, sphere_data->contact_duration[contact_id]);

                if (calc_rolling_fr == true) {
                    roll_acc = computeRollingAngAcc(sphere_data, gran_params, gran_params->rolling_coeff_s2w_SU,
                                                    gran_params->spinning_coeff_s2w_SU, force_accum, sphOmega,
                                                    make_float3(0, 0, 0), dist * contact_normal);

                } else {
                    roll_acc = make_float3(0.0f, 0.0f, 0.0f);
                }

            } else {
                float penetration = sphereRadius_SU - dist;
                float force_model_multiplier = sqrt(penetration / sphereRadius_SU);

                roll_acc = computeRollingAngAcc(sphere_data, gran_params, gran_params->rolling_coeff_s2w_SU,
                                                gran_params->spinning_coeff_s2w_SU, force_accum, sphOmega,
                                                make_float3(0, 0, 0), dist * contact_normal);

                // compute tangent force
                tangent_force = computeFrictionForces(gran_params, sphere_data, sphID, BC_histmap_label,
                                                      gran_params->static_friction_coeff_s2w, gran_params->K_t_s2w_SU,
                                                      gran_params->Gamma_t_s2w_SU, force_model_multiplier, m_eff,
                                                      force_accum, rel_vel, contact_normal);
            }

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
