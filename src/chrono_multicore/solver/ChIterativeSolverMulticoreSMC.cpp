// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Arman Pazouki
// =============================================================================
//
// Implementation of methods specific to the multicore smooth-contact solver.
//
// These functions implement the basic time update for a multibody system using
// a penalty-based approach for including frictional contact. It is assumed that
// geometric contact information has been already computed and is available.
// The current algorithm is based on a semi-implicit Euler scheme and projection
// on the velocity manifold of the bilateral constraints.
//
// =============================================================================

//// RADU
//// When using the MultiStep tangential displacement mode, we need to calculate the
//// indeices of the shape in collision.  This conflicts with cases where contacts
//// are defined by a user custom callback as there are no shapes defined in that
//// case. Is there a solution?

#include <algorithm>
#include <stdexcept>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChMaterialSurfaceSMC.h"
#include "chrono_multicore/solver/ChIterativeSolverMulticore.h"

#include <thrust/sort.h>

#if defined _WIN32
    #include <cstdint>
#endif

using namespace chrono;

// -----------------------------------------------------------------------------
// Main worker function for calculating contact forces. Calculates the contact
// force and torque for the contact pair identified by 'index' and stores them
// in the 'extended' output arrays. The calculated force and torque vectors are
// therefore duplicated in the output arrays, once for each body involved in the
// contact (with opposite signs for the two bodies).
// -----------------------------------------------------------------------------
void function_CalcContactForces(
    int index,                                            // index of this contact pair
    vec2* body_pairs,                                     // indices of the body pair in contact
    vec2* shape_pairs,                                    // indices of the shape pair in contact
    ChSystemSMC::ContactForceModel contact_model,         // contact force model
    ChSystemSMC::AdhesionForceModel adhesion_model,       // adhesion force model
    ChSystemSMC::TangentialDisplacementModel displ_mode,  // type of tangential displacement history
    bool use_mat_props,                                   // flag specifying how coefficients are obtained
    real char_vel,                                        // characteristic velocity (Hooke)
    real min_slip_vel,                                    // threshold tangential velocity
    real min_roll_vel,                                    // threshold rolling velocity
    real min_spin_vel,                                    // threshold spinning velocity
    real dT,                                              // integration time step
    real* body_mass,                                      // body masses (per body)
    real3* pos,                                           // body positions
    quaternion* rot,                                      // body orientations
    real* vel,                                            // body linear and angular velocities
    real3* friction,                                      // eff. coefficients of friction (per contact)
    real2* modulus,                                       // eff. elasticity and shear modulus (per contact)
    real3* adhesion,                                      // eff. adhesion paramters (per contact)
    real* cr,                                             // eff. coefficient of restitution (per contact)
    real4* smc_params,                                    // eff. SMC parameters k and g (per contact)
    real3* pt1,                                           // point on shape 1 (per contact)
    real3* pt2,                                           // point on shape 2 (per contact)
    real3* normal,                                        // contact normal (per contact)
    real* depth,                                          // penetration depth (per contact)
    real* eff_radius,                                     // effective contact radius (per contact)
    vec3* shear_neigh,                                    // neighbor list of contacting bodies and shapes (per body)
    char* shear_touch,                                    // flag if contact in neighbor list is persistent (per body)
    real3* shear_disp,                                    // accumulated shear displacement for each neighbor (per body)
    real* contact_relvel_init,                            // initial relative normal velocity per contact pair
    real* contact_duration,                               // duration of persistent contact between contact pairs
    int* ct_bid,                                          // [output] body IDs (two per contact)
    real3* ct_force,                                      // [output] body force (two per contact)
    real3* ct_torque                                      // [output] body torque (two per contact)
) {
    // Identify the two bodies in contact (global body IDs).
    int b1 = body_pairs[index].x;
    int b2 = body_pairs[index].y;

    // If the two contact shapes are actually separated, set zero forces and torques.
    if (depth[index] >= 0) {
        ct_bid[2 * index] = b1;
        ct_bid[2 * index + 1] = b2;
        ct_force[2 * index] = real3(0);
        ct_force[2 * index + 1] = real3(0);
        ct_torque[2 * index] = real3(0);
        ct_torque[2 * index + 1] = real3(0);
        return;
    }

    // Kinematic information
    // ---------------------

    // Express contact point locations in local frames
    //   s' = At * s = At * (rP - r)
    real3 pt1_loc = TransformParentToLocal(pos[b1], rot[b1], pt1[index]);
    real3 pt2_loc = TransformParentToLocal(pos[b2], rot[b2], pt2[index]);

    // Calculate velocities of the contact points (in global frame)
    //   vP = v + omg x s = v + A * (omg' x s')
    real3 v_body1 = real3(vel[b1 * 6 + 0], vel[b1 * 6 + 1], vel[b1 * 6 + 2]);
    real3 v_body2 = real3(vel[b2 * 6 + 0], vel[b2 * 6 + 1], vel[b2 * 6 + 2]);

    real3 o_body1 = real3(vel[b1 * 6 + 3], vel[b1 * 6 + 4], vel[b1 * 6 + 5]);
    real3 o_body2 = real3(vel[b2 * 6 + 3], vel[b2 * 6 + 4], vel[b2 * 6 + 5]);

    real3 vel1 = v_body1 + Rotate(Cross(o_body1, pt1_loc), rot[b1]);
    real3 vel2 = v_body2 + Rotate(Cross(o_body2, pt2_loc), rot[b2]);

    // Calculate relative velocity (in global frame)
    // Note that relvel_n_mag is a signed quantity, while relvel_t_mag is an
    // actual magnitude (always positive).
    real3 relvel = vel2 - vel1;
    real relvel_n_mag = Dot(relvel, normal[index]);
    real3 relvel_n = relvel_n_mag * normal[index];
    real3 relvel_t = relvel - relvel_n;
    real relvel_t_mag = Length(relvel_t);

    // Extract composite material properties
    // -------------------------------------

    real m_eff = body_mass[b1] * body_mass[b2] / (body_mass[b1] + body_mass[b2]);

    real mu_eff = friction[index].x;
    real muRoll_eff = friction[index].y;
    real muSpin_eff = friction[index].z;

    real E_eff = modulus[index].x;
    real G_eff = modulus[index].y;

    real adhesion_eff = adhesion[index].x;
    real adhesionMultDMT_eff = adhesion[index].y;
    real adhesionSPerko_eff = adhesion[index].z;

    real cr_eff = cr[index];

    real user_kn = smc_params[index].x;
    real user_kt = smc_params[index].y;
    real user_gn = smc_params[index].z;
    real user_gt = smc_params[index].w;

    // Contact force
    // -------------

    // All models use the following formulas for normal and tangential forces:
    //     Fn = kn * delta_n - gn * v_n
    //     Ft = kt * delta_t - gt * v_t
    // The stiffness and damping coefficients are obtained differently, based
    // on the force model and on how coefficients are specified.
    real kn = 0;
    real kt = 0;
    real gn = 0;
    real gt = 0;
    real kn_simple = 0;
    real gn_simple = 0;

    real t_contact = 0;
    real relvel_init = abs(relvel_n_mag);
    real delta_n = -depth[index];
    real3 delta_t = real3(0);

    int i;
    int contact_id = -1;
    int shear_body1 = -1;
    int shear_body2;
    int shear_shape1;
    int shear_shape2;
    bool newcontact = true;

    if (displ_mode == ChSystemSMC::TangentialDisplacementModel::OneStep) {
        delta_t = relvel_t * dT;
    } else if (displ_mode == ChSystemSMC::TangentialDisplacementModel::MultiStep) {
        delta_t = relvel_t * dT;

        // Identify the two shapes in contact (global shape IDs).
        int s1 = shape_pairs[index].x;
        int s2 = shape_pairs[index].y;

        // Contact history information stored on the body with the smaller shape or else the body with larger index.
        // Currently, it is assumed that the smaller shape is on the body with larger ID. We call this body shear_body1.
        shear_body1 = std::max(b1, b2);
        shear_body2 = std::min(b1, b2);
        shear_shape1 = std::max(s1, s2);
        shear_shape2 = std::min(s1, s2);

        // Check if contact history already exists. If not, initialize new contact history.
        for (i = 0; i < max_shear; i++) {
            int ctIdUnrolled = max_shear * shear_body1 + i;
            if (shear_neigh[ctIdUnrolled].x == shear_body2 && shear_neigh[ctIdUnrolled].y == shear_shape1 &&
                shear_neigh[ctIdUnrolled].z == shear_shape2) {
                contact_duration[ctIdUnrolled] += dT;
                contact_id = i;
                newcontact = false;
                break;
            }
        }
        if (newcontact == true) {
            for (i = 0; i < max_shear; i++) {
                int ctIdUnrolled = max_shear * shear_body1 + i;
                if (shear_neigh[ctIdUnrolled].x == -1) {
                    contact_id = i;
                    shear_neigh[ctIdUnrolled].x = shear_body2;
                    shear_neigh[ctIdUnrolled].y = shear_shape1;
                    shear_neigh[ctIdUnrolled].z = shear_shape2;
                    shear_disp[ctIdUnrolled].x = 0;
                    shear_disp[ctIdUnrolled].y = 0;
                    shear_disp[ctIdUnrolled].z = 0;
                    contact_relvel_init[ctIdUnrolled] = relvel_init;
                    contact_duration[ctIdUnrolled] = 0;
                    break;
                }
            }
        }

        // Record that these two bodies are really in contact at this time.
        int ctSaveId = max_shear * shear_body1 + contact_id;
        shear_touch[ctSaveId] = true;

        // Increment stored contact history tangential (shear) displacement vector and project it onto the current
        // contact plane.
        if (shear_body1 == b1) {
            shear_disp[ctSaveId] += delta_t;
            shear_disp[ctSaveId] -= Dot(shear_disp[ctSaveId], normal[index]) * normal[index];
            delta_t = shear_disp[ctSaveId];
        } else {
            shear_disp[ctSaveId] -= delta_t;
            shear_disp[ctSaveId] -= Dot(shear_disp[ctSaveId], normal[index]) * normal[index];
            delta_t = -shear_disp[ctSaveId];
        }

        // Load the initial collision velocity and accumulated contact duration from the contact history.
        relvel_init = (contact_relvel_init[ctSaveId] < char_vel) ? char_vel : contact_relvel_init[ctSaveId];
        t_contact = contact_duration[ctSaveId];
    }

    auto eps = std::numeric_limits<double>::epsilon();

    switch (contact_model) {
        case ChSystemSMC::ContactForceModel::Hooke:
            if (use_mat_props) {
                real tmp_k = (16.0 / 15) * Sqrt(eff_radius[index]) * E_eff;
                char_vel = (displ_mode == ChSystemSMC::TangentialDisplacementModel::MultiStep) ? relvel_init : char_vel;
                real v2 = char_vel * char_vel;
                real loge = (cr_eff < eps) ? Log(eps) : Log(cr_eff);
                loge = (cr_eff > 1 - eps) ? Log(1 - eps) : loge;
                real tmp_g = 1 + Pow(CH_C_PI / loge, 2);
                kn = tmp_k * Pow(m_eff * v2 / tmp_k, 1.0 / 5);
                kt = kn;
                gn = Sqrt(4 * m_eff * kn / tmp_g);
                gt = gn;
            } else {
                kn = user_kn;
                kt = user_kt;
                gn = m_eff * user_gn;
                gt = m_eff * user_gt;
            }

            kn_simple = kn;
            gn_simple = gn;

            break;

        case ChSystemSMC::ContactForceModel::Hertz:
            if (use_mat_props) {
                real sqrt_Rd = Sqrt(eff_radius[index] * delta_n);
                real Sn = 2 * E_eff * sqrt_Rd;
                real St = 8 * G_eff * sqrt_Rd;
                real loge = (cr_eff < eps) ? Log(eps) : Log(cr_eff);
                real beta = loge / Sqrt(loge * loge + CH_C_PI * CH_C_PI);
                kn = (2.0 / 3) * Sn;
                kt = St;
                gn = -2 * Sqrt(5.0 / 6) * beta * Sqrt(Sn * m_eff);
                gt = -2 * Sqrt(5.0 / 6) * beta * Sqrt(St * m_eff);
            } else {
                real tmp = eff_radius[index] * Sqrt(delta_n);
                kn = tmp * user_kn;
                kt = tmp * user_kt;
                gn = tmp * m_eff * user_gn;
                gt = tmp * m_eff * user_gt;
            }

            kn_simple = kn / Sqrt(delta_n);
            gn_simple = gn / Pow(delta_n, 1.0 / 4.0);

            break;

        case ChSystemSMC::Flores:
            if (use_mat_props) {
                real sqrt_Rd = Sqrt(eff_radius[index] * delta_n);
                real Sn = 2 * E_eff * sqrt_Rd;
                real St = 8 * G_eff * sqrt_Rd;
                cr_eff = (cr_eff < 0.01) ? 0.01 : cr_eff;
                cr_eff = (cr_eff > 1.0 - eps) ? 1.0 - eps : cr_eff;
                real loge = Log(cr_eff);
                real beta = loge / Sqrt(loge * loge + CH_C_PI * CH_C_PI);
                char_vel = (displ_mode == ChSystemSMC::TangentialDisplacementModel::MultiStep) ? relvel_init : char_vel;
                kn = (2.0 / 3.0) * Sn;
                kt = (2.0 / 3.0) * St;
                gn = 8.0 * (1.0 - cr_eff) * kn * delta_n / (5.0 * cr_eff * char_vel);
                gt = -2 * Sqrt(5.0 / 6) * beta * Sqrt(St * m_eff);  // Need to multiply St by 2/3 here as well ?
            } else {
                real tmp = eff_radius[index] * Sqrt(delta_n);
                kn = tmp * user_kn;
                kt = tmp * user_kt;
                gn = tmp * m_eff * user_gn * delta_n;
                gt = tmp * m_eff * user_gt;
            }

            kn_simple = kn / Sqrt(delta_n);
            gn_simple = gn / Pow(delta_n, 3.0 / 2.0);

            break;

        case ChSystemSMC::ContactForceModel::PlainCoulomb:
            if (use_mat_props) {
                real sqrt_Rd = Sqrt(delta_n);
                real Sn = 2 * E_eff * sqrt_Rd;
                real loge = (cr_eff < eps) ? Log(eps) : Log(cr_eff);
                real beta = loge / Sqrt(loge * loge + CH_C_PI * CH_C_PI);
                kn = (2.0 / 3) * Sn;
                gn = -2 * Sqrt(5.0 / 6) * beta * Sqrt(Sn * m_eff);
            } else {
                real tmp = Sqrt(delta_n);
                kn = tmp * user_kn;
                gn = tmp * user_gn;
            }

            kn_simple = kn / Sqrt(delta_n);
            gn_simple = gn / Pow(delta_n, 1.0 / 4.0);

            kt = 0;
            gt = 0;

            {
                real forceN_mag = kn * delta_n - gn * relvel_n_mag;
                real forceT_mag = mu_eff * Tanh(5.0 * relvel_t_mag) * forceN_mag;

                // Accumulate normal and tangential forces
                real3 force = forceN_mag * normal[index];
                if (relvel_t_mag >= min_slip_vel)
                    force -= (forceT_mag / relvel_t_mag) * relvel_t;

                // Convert force into the local body frames and calculate induced torques
                real3 torque1_loc = Cross(pt1_loc, RotateT(force, rot[b1]));
                real3 torque2_loc = Cross(pt2_loc, RotateT(force, rot[b2]));

                // If the duration of the current contact is less than the durration of a typical collision,
                // do not apply friction. Rolling and spinning friction should only be applied to persistant contacts
                // Rolling and spinning friction are applied right away for critically damped or over-damped systems
                real d_coeff = gn_simple / (2.0 * m_eff * Sqrt(kn_simple / m_eff));
                if (d_coeff < 1.0) {
                    real t_collision = CH_C_PI * Sqrt(m_eff / (kn_simple * (1 - d_coeff * d_coeff)));
                    if (t_contact <= t_collision) {
                        muRoll_eff = 0.0;
                        muSpin_eff = 0.0;
                    }
                }

                // Compute some additional vales needed for the rolling and spinning friction calculations
                real3 v_rot = Rotate(Cross(o_body2, pt2_loc), rot[b2]) - Rotate(Cross(o_body1, pt1_loc), rot[b1]);
                real3 rel_o = Rotate(o_body2, rot[b2]) - Rotate(o_body1, rot[b1]);

                // Calculate rolling friction torque as M_roll = mu_r * R * (F_N x v_rot) / |v_rot| (Schwartz et al.
                // 2012)
                real3 m_roll1 = real3(0);
                real3 m_roll2 = real3(0);

                if (Length(v_rot) > min_roll_vel && muRoll_eff > eps) {
                    m_roll1 = muRoll_eff * Cross(forceN_mag * pt1_loc, RotateT(v_rot, rot[b1])) / Length(v_rot);
                    m_roll2 = muRoll_eff * Cross(forceN_mag * pt2_loc, RotateT(v_rot, rot[b2])) / Length(v_rot);
                }

                // Calculate spinning friction torque as M_spin = -mu_t * r_c * ((w_n - w_p) . F_n / |w_n - w_p|) * n
                // r_c is the radius of the circle resulting from the intersecting body surfaces (Schwartz et al. 2012)
                //
                // TODO: The spinning moment calculation is only valid for sphere-sphere collisions because of the
                // r1 and r2 terms. In order for the calculation to be valid for sphere-wall collisions, the wall
                // must be ~100x particle diameters in thickness
                real3 m_spin1 = real3(0);
                real3 m_spin2 = real3(0);

                if (Length(rel_o) > min_spin_vel && muSpin_eff > eps) {
                    real r1 = Length(pt1_loc);
                    real r2 = Length(pt2_loc);
                    real xc = (r1 * r1 - r2 * r2) / (2 * (r1 + r2 - delta_n)) + 0.5 * (r1 + r2 - delta_n);
                    real rc = r1 * r1 - xc * xc;
                    rc = (rc < eps) ? eps : Sqrt(rc);

                    m_spin1 = muSpin_eff * rc *
                              RotateT(Dot(rel_o, forceN_mag * normal[index]) * normal[index], rot[b1]) / Length(rel_o);
                    m_spin2 = muSpin_eff * rc *
                              RotateT(Dot(rel_o, forceN_mag * normal[index]) * normal[index], rot[b2]) / Length(rel_o);
                }

                // Account for adhesion
                switch (adhesion_model) {
                    case ChSystemSMC::AdhesionForceModel::Constant:
                        force -= adhesion_eff * normal[index];
                        break;
                    case ChSystemSMC::AdhesionForceModel::DMT:
                        force -= adhesionMultDMT_eff * Sqrt(eff_radius[index]) * normal[index];
                        break;
                    case ChSystemSMC::AdhesionForceModel::Perko:
                        force -= adhesionSPerko_eff * eff_radius[index] * normal[index];
                        break;
                }

                ct_bid[2 * index] = b1;
                ct_bid[2 * index + 1] = b2;
                ct_force[2 * index] = -force;
                ct_force[2 * index + 1] = force;
                ct_torque[2 * index] = -torque1_loc + m_roll1 + m_spin1;
                ct_torque[2 * index + 1] = torque2_loc - m_roll2 - m_spin2;
            }

            return;
    }

    // Calculate the the normal and tangential contact forces.
    // The normal force is a magnitude, and it will be applied along the contact
    // normal direction (negative relative to body1 & positive relative to body2).
    // The tangential force is a vector with two parts: one depends on the stored
    // contact history tangential (or shear) displacement vector delta_t, and the
    // other depends on the current relative velocity vector (for viscous damping).
    real forceN_mag = kn * delta_n - gn * relvel_n_mag;
    real3 forceT_stiff = kt * delta_t;
    real3 forceT_damp = gt * relvel_t;

    // Apply Coulomb friction law.
    // We must enforce force_T_mag <= mu_eff * |forceN_mag|.
    // If force_T_mag > mu_eff * |forceN_mag| and there is shear displacement
    // due to contact history, then the shear displacement is scaled so that
    // the tangential force will be correct if force_T_mag subsequently drops
    // below the Coulomb limit.
    //
    // TODO: This implementation currently assumes that mu_slip and mu_k are equal
    real3 forceT = forceT_stiff + forceT_damp;
    real forceT_mag = Length(forceT);
    real delta_t_mag = Length(delta_t);
    real forceT_slide = mu_eff * Abs(forceN_mag);
    if (forceT_mag > forceT_slide) {
        if (delta_t_mag > eps) {
            real ratio = forceT_slide / forceT_mag;
            forceT *= ratio;
            if (displ_mode == ChSystemSMC::TangentialDisplacementModel::MultiStep) {
                delta_t = (forceT - forceT_damp) / kt;
                if (shear_body1 == b1) {
                    shear_disp[max_shear * shear_body1 + contact_id] = delta_t;
                } else {
                    shear_disp[max_shear * shear_body1 + contact_id] = -delta_t;
                }
            }
        } else {
            forceT = real3(0);
        }
    }

    // Accumulate normal and tangential forces
    real3 force = forceN_mag * normal[index] - forceT;

    // Body forces (in global frame) & torques (in local frame)
    // --------------------------------------------------------

    // Convert force into the local body frames and calculate induced torques
    //    n' = s' x F' = s' x (A*F)
    real3 torque1_loc = Cross(pt1_loc, RotateT(force, rot[b1]));
    real3 torque2_loc = Cross(pt2_loc, RotateT(force, rot[b2]));

    // If the duration of the current contact is less than the durration of a typical collision,
    // do not apply friction. Rolling and spinning friction should only be applied to persistant contacts.
    // Rolling and spinning friction are applied right away for critically damped or over-damped systems.
    real d_coeff = gn_simple / (2.0 * m_eff * Sqrt(kn_simple / m_eff));
    if (d_coeff < 1.0) {
        real t_collision = CH_C_PI * Sqrt(m_eff / (kn_simple * (1 - d_coeff * d_coeff)));
        if (t_contact <= t_collision) {
            muRoll_eff = 0.0;
            muSpin_eff = 0.0;
        }
    }

    // Compute some additional vales needed for the rolling and spinning friction calculations
    real3 v_rot = Rotate(Cross(o_body2, pt2_loc), rot[b2]) - Rotate(Cross(o_body1, pt1_loc), rot[b1]);
    real3 rel_o = Rotate(o_body2, rot[b2]) - Rotate(o_body1, rot[b1]);

    // Calculate rolling friction torque as M_roll = mu_r * R * (F_N x v_rot) / |v_rot| (Schwartz et al. 2012)
    real3 m_roll1 = real3(0);
    real3 m_roll2 = real3(0);

    if (Length(v_rot) > min_roll_vel && muRoll_eff > eps) {
        m_roll1 = muRoll_eff * Cross(forceN_mag * pt1_loc, RotateT(v_rot, rot[b1])) / Length(v_rot);
        m_roll2 = muRoll_eff * Cross(forceN_mag * pt2_loc, RotateT(v_rot, rot[b2])) / Length(v_rot);
    }

    // Calculate spinning friction torque as M_spin = -mu_t * r_c * ((w_n - w_p) . F_n / |w_n - w_p|) * n
    // r_c is the radius of the circle resulting from the intersecting body surfaces (Schwartz et al. 2012)
    //
    // TODO: The spinning moment calculation is only valid for sphere-sphere collisions because of the
    // r1 and r2 terms. In order for the calculation to be valid for sphere-wall collisions, the wall
    // must be ~100x particle diameters in thickness
    real3 m_spin1 = real3(0);
    real3 m_spin2 = real3(0);

    if (Length(rel_o) > min_spin_vel && muSpin_eff > eps) {
        real r1 = Length(pt1_loc);
        real r2 = Length(pt2_loc);
        real xc = (r1 * r1 - r2 * r2) / (2 * (r1 + r2 - delta_n)) + 0.5 * (r1 + r2 - delta_n);
        real rc = r1 * r1 - xc * xc;
        rc = (rc < eps) ? eps : Sqrt(rc);

        m_spin1 =
            muSpin_eff * rc * RotateT(Dot(rel_o, forceN_mag * normal[index]) * normal[index], rot[b1]) / Length(rel_o);
        m_spin2 =
            muSpin_eff * rc * RotateT(Dot(rel_o, forceN_mag * normal[index]) * normal[index], rot[b2]) / Length(rel_o);
    }

    // Account for adhesion
    switch (adhesion_model) {
        case ChSystemSMC::AdhesionForceModel::Constant:
            force -= adhesion_eff * normal[index];
            break;
        case ChSystemSMC::AdhesionForceModel::DMT:
            force -= adhesionMultDMT_eff * Sqrt(eff_radius[index]) * normal[index];
            break;
        case ChSystemSMC::AdhesionForceModel::Perko:
            force -= adhesionSPerko_eff * eff_radius[index] * normal[index];
            break;
    }

    // Store body forces and torques, duplicated for the two bodies.
    ct_bid[2 * index] = b1;
    ct_bid[2 * index + 1] = b2;
    ct_force[2 * index] = -force;
    ct_force[2 * index + 1] = force;
    ct_torque[2 * index] = -torque1_loc + m_roll1 + m_spin1;
    ct_torque[2 * index + 1] = torque2_loc - m_roll2 - m_spin2;
}

// -----------------------------------------------------------------------------
// Calculate contact forces and torques for all contact pairs.
// -----------------------------------------------------------------------------

void ChIterativeSolverMulticoreSMC::host_CalcContactForces(custom_vector<int>& ct_bid,
                                                           custom_vector<real3>& ct_force,
                                                           custom_vector<real3>& ct_torque,
                                                           custom_vector<vec2>& shape_pairs,
                                                           custom_vector<char>& shear_touch) {
#pragma omp parallel for
    for (int index = 0; index < (signed)data_manager->cd_data->num_rigid_contacts; index++) {
        function_CalcContactForces(
            index,                                                  // index of this contact pair
            data_manager->cd_data->bids_rigid_rigid.data(),         // indices of the body pair in contact
            shape_pairs.data(),                                     // indices of the shape pair in contact
            data_manager->settings.solver.contact_force_model,      // contact force model
            data_manager->settings.solver.adhesion_force_model,     // adhesion force model
            data_manager->settings.solver.tangential_displ_mode,    // type of tangential displacement history
            data_manager->settings.solver.use_material_properties,  // flag specifying how coefficients are obtained
            data_manager->settings.solver.characteristic_vel,       // characteristic velocity (Hooke)
            data_manager->settings.solver.min_slip_vel,             // threshold tangential velocity
            data_manager->settings.solver.min_roll_vel,             // threshold rolling velocity
            data_manager->settings.solver.min_spin_vel,             // threshold spinning velocity
            data_manager->settings.step_size,                       // integration time step
            data_manager->host_data.mass_rigid.data(),              // body masses
            data_manager->host_data.pos_rigid.data(),               // body positions
            data_manager->host_data.rot_rigid.data(),               // body orientations
            data_manager->host_data.v.data(),                       // body linear and angular velocities
            data_manager->host_data.fric_rigid_rigid.data(),        // eff. coefficients of friction (per contact)
            data_manager->host_data.modulus_rigid_rigid.data(),     // eff. elasticity and shear modulus (per contact)
            data_manager->host_data.adhesion_rigid_rigid.data(),    // eff. adhesion paramters (per contact)
            data_manager->host_data.cr_rigid_rigid.data(),          // eff. coefficient of restitution (per contact)
            data_manager->host_data.smc_rigid_rigid.data(),         // eff. SMC parameters k and g (per contact)
            data_manager->cd_data->cpta_rigid_rigid.data(),         // point on shape 1 (per contact)
            data_manager->cd_data->cptb_rigid_rigid.data(),         // point on shape 2 (per contact)
            data_manager->cd_data->norm_rigid_rigid.data(),         // contact normal (per contact)
            data_manager->cd_data->dpth_rigid_rigid.data(),         // penetration depth (per contact)
            data_manager->cd_data->erad_rigid_rigid.data(),         // effective contact radius (per contact)
            data_manager->host_data.shear_neigh.data(),  // neighbor list of contacting bodies and shapes (per body)
            shear_touch.data(),                          // flag if contact in neighbor list is persistent (per body)
            data_manager->host_data.shear_disp.data(),   // accumulated shear displacement for each neighbor (per body)
            data_manager->host_data.contact_relvel_init.data(),  // initial relative normal velocity per contact pair
            data_manager->host_data.contact_duration.data(),     // duration of persistent contact between contact pairs
            ct_bid.data(),                                       // [output] body IDs (two per contact)
            ct_force.data(),                                     // [output] body force (two per contact)
            ct_torque.data()                                     // [output] body torque (two per contact)
        );
    }
}

// -----------------------------------------------------------------------------
// Include contact impulses (linear and rotational) for all bodies that are
// involved in at least one contact. For each such body, the corresponding
// entries in the arrays 'ct_body_force' and 'ct_body_torque' contain the
// cummulative force and torque, respectively, over all contacts involving that
// body.
// -----------------------------------------------------------------------------
void ChIterativeSolverMulticoreSMC::host_AddContactForces(uint ct_body_count, const custom_vector<int>& ct_body_id) {
    const custom_vector<real3>& ct_body_force = data_manager->host_data.ct_body_force;
    const custom_vector<real3>& ct_body_torque = data_manager->host_data.ct_body_torque;

#pragma omp parallel for
    for (int index = 0; index < (signed)ct_body_count; index++) {
        real3 contact_force = data_manager->settings.step_size * ct_body_force[index];
        real3 contact_torque = data_manager->settings.step_size * ct_body_torque[index];
        data_manager->host_data.hf[ct_body_id[index] * 6 + 0] += contact_force.x;
        data_manager->host_data.hf[ct_body_id[index] * 6 + 1] += contact_force.y;
        data_manager->host_data.hf[ct_body_id[index] * 6 + 2] += contact_force.z;
        data_manager->host_data.hf[ct_body_id[index] * 6 + 3] += contact_torque.x;
        data_manager->host_data.hf[ct_body_id[index] * 6 + 4] += contact_torque.y;
        data_manager->host_data.hf[ct_body_id[index] * 6 + 5] += contact_torque.z;
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChIterativeSolverMulticoreSMC::host_SetContactForcesMap(uint ct_body_count, const custom_vector<int>& ct_body_id) {
    custom_vector<int>& ct_body_map = data_manager->host_data.ct_body_map;

#pragma omp parallel for
    for (int index = 0; index < (signed)ct_body_count; index++) {
        ct_body_map[ct_body_id[index]] = index;
    }
}

// Binary operation for adding two-object tuples
struct sum_tuples {
    thrust::tuple<real3, real3> operator()(const thrust::tuple<real3, real3>& a,
                                           const thrust::tuple<real3, real3>& b) const {
        return thrust::tuple<real3, real3>(thrust::get<0>(a) + thrust::get<0>(b),
                                           thrust::get<1>(a) + thrust::get<1>(b));
    }
};

// -----------------------------------------------------------------------------
// Process contact information reported by the narrowphase collision detection,
// generate contact forces, and update the (linear and rotational) impulses for
// all bodies involved in at least one contact.
// -----------------------------------------------------------------------------
void ChIterativeSolverMulticoreSMC::ProcessContacts() {
    const auto num_rigid_contacts = data_manager->cd_data->num_rigid_contacts;

    // 1. Calculate contact forces and torques - per contact basis
    //    For each pair of contact shapes that overlap, we calculate and store the
    //    IDs of the two corresponding bodies and the resulting contact forces and
    //    torques on the two bodies.

    custom_vector<int> ct_bid(2 * num_rigid_contacts);
    custom_vector<real3> ct_force(2 * num_rigid_contacts);
    custom_vector<real3> ct_torque(2 * num_rigid_contacts);

    // Set up additional vectors for multi-step tangential model
    custom_vector<vec2> shape_pairs;
    custom_vector<char> shear_touch;
    if (data_manager->settings.solver.tangential_displ_mode == ChSystemSMC::TangentialDisplacementModel::MultiStep) {
        shape_pairs.resize(num_rigid_contacts);
        shear_touch.resize(max_shear * data_manager->num_rigid_bodies);
        Thrust_Fill(shear_touch, false);
#pragma omp parallel for
        for (int i = 0; i < (signed)num_rigid_contacts; i++) {
            vec2 pair = I2(int(data_manager->cd_data->contact_shapeIDs[i] >> 32),
                           int(data_manager->cd_data->contact_shapeIDs[i] & 0xffffffff));
            shape_pairs[i] = pair;
        }
    }

    host_CalcContactForces(ct_bid, ct_force, ct_torque, shape_pairs, shear_touch);

    data_manager->host_data.ct_force.resize(2 * num_rigid_contacts);
    data_manager->host_data.ct_torque.resize(2 * num_rigid_contacts);
    thrust::copy(THRUST_PAR ct_force.begin(), ct_force.end(), data_manager->host_data.ct_force.begin());
    thrust::copy(THRUST_PAR ct_torque.begin(), ct_torque.end(), data_manager->host_data.ct_torque.begin());

    if (data_manager->settings.solver.tangential_displ_mode == ChSystemSMC::TangentialDisplacementModel::MultiStep) {
#pragma omp parallel for
        for (int index = 0; index < (signed)data_manager->num_rigid_bodies; index++) {
            for (int i = 0; i < max_shear; i++) {
                if (shear_touch[max_shear * index + i] == false)
                    data_manager->host_data.shear_neigh[max_shear * index + i].x = -1;
            }
        }
    }

    // 2. Calculate contact forces and torques - per body basis
    //    Accumulate the contact forces and torques for all bodies that are
    //    involved in at least one contact, by reducing the contact forces and
    //    torques from all contacts these bodies are involved in. The number of
    //    bodies that experience at least one contact is 'ct_body_count'.
    thrust::sort_by_key(THRUST_PAR ct_bid.begin(), ct_bid.end(),
                        thrust::make_zip_iterator(thrust::make_tuple(ct_force.begin(), ct_torque.begin())));

    custom_vector<int> ct_body_id(data_manager->num_rigid_bodies);
    custom_vector<real3>& ct_body_force = data_manager->host_data.ct_body_force;
    custom_vector<real3>& ct_body_torque = data_manager->host_data.ct_body_torque;

    ct_body_force.resize(data_manager->num_rigid_bodies);
    ct_body_torque.resize(data_manager->num_rigid_bodies);

    // Reduce contact forces from all contacts and count bodies currently involved
    // in contact. We do this simultaneously for contact forces and torques, using
    // zip iterators.
    auto end_range = thrust::reduce_by_key(
        THRUST_PAR ct_bid.begin(), ct_bid.end(),
        thrust::make_zip_iterator(thrust::make_tuple(ct_force.begin(), ct_torque.begin())), ct_body_id.begin(),
        thrust::make_zip_iterator(thrust::make_tuple(ct_body_force.begin(), ct_body_torque.begin())),
#if defined _WIN32
        thrust::equal_to<int64_t>(), sum_tuples()  // Windows compilers require an explicit-width type
#else
        thrust::equal_to<int>(), sum_tuples()
#endif
    );

    uint ct_body_count = (uint)(end_range.first - ct_body_id.begin());

    ct_body_force.resize(ct_body_count);
    ct_body_torque.resize(ct_body_count);

    // 3. Add contact forces and torques to existing forces (impulses):
    //    For all bodies involved in a contact, update the body forces and torques
    //    (scaled by the integration time step).
    host_AddContactForces(ct_body_count, ct_body_id);

    // 4. Set up map from all bodies in the system to bodies involved in a contact.
    host_SetContactForcesMap(ct_body_count, ct_body_id);
}

void ChIterativeSolverMulticoreSMC::ComputeD() {
    uint num_constraints = data_manager->num_constraints;
    if (num_constraints <= 0) {
        return;
    }

    uint num_dof = data_manager->num_dof;
    uint nnz_bilaterals = data_manager->nnz_bilaterals;

    CompressedMatrix<real>& D_T = data_manager->host_data.D_T;
    if (D_T.capacity() > 0) {
        clear(D_T);
    }

    D_T.reserve(nnz_bilaterals);
    D_T.resize(num_constraints, num_dof, false);

    data_manager->bilateral->GenerateSparsity();
    data_manager->bilateral->Build_D();

    data_manager->host_data.D = trans(D_T);
    data_manager->host_data.M_invD = data_manager->host_data.M_inv * data_manager->host_data.D;
}

void ChIterativeSolverMulticoreSMC::ComputeE() {
    if (data_manager->num_constraints <= 0) {
        return;
    }

    data_manager->host_data.E.resize(data_manager->num_constraints);
    reset(data_manager->host_data.E);

    data_manager->bilateral->Build_E();
}

void ChIterativeSolverMulticoreSMC::ComputeR() {
    if (data_manager->num_constraints <= 0) {
        return;
    }

    data_manager->host_data.b.resize(data_manager->num_constraints);
    reset(data_manager->host_data.b);
    data_manager->bilateral->Build_b();

    data_manager->host_data.R_full =
        -data_manager->host_data.b - data_manager->host_data.D_T * data_manager->host_data.M_invk;
}

// -----------------------------------------------------------------------------
// This is the main function for advancing the system state in time. On entry,
// geometric contact information is available as calculated by the narrowphase
// collision detection. This function calculates contact forces, updates the
// generalized velocities, then enforces the velocity-level constraints for any
// bilateral (joint) constraints present in the system.
// -----------------------------------------------------------------------------
void ChIterativeSolverMulticoreSMC::RunTimeStep() {
    // This is the total number of constraints, note that there are no contacts
    data_manager->num_constraints = data_manager->num_bilaterals;
    data_manager->num_unilaterals = 0;

    // Calculate contact forces (impulses) and append them to the body forces
    data_manager->host_data.ct_body_map.resize(data_manager->num_rigid_bodies);
    Thrust_Fill(data_manager->host_data.ct_body_map, -1);

    if (data_manager->cd_data && data_manager->cd_data->num_rigid_contacts > 0) {
        data_manager->system_timer.start("ChIterativeSolverMulticoreSMC_ProcessContact");
        ProcessContacts();
        data_manager->system_timer.stop("ChIterativeSolverMulticoreSMC_ProcessContact");
    }

    // Generate the mass matrix and compute M_inv_k
    ComputeInvMassMatrix();

    // If there are (bilateral) constraints, calculate Lagrange multipliers.
    if (data_manager->num_constraints != 0) {
        data_manager->system_timer.start("ChIterativeSolverMulticore_Setup");

        data_manager->bilateral->Setup(data_manager);

        solver->current_iteration = 0;
        data_manager->measures.solver.total_iteration = 0;
        data_manager->measures.solver.maxd_hist.clear();            ////
        data_manager->measures.solver.maxdeltalambda_hist.clear();  ////  currently not used

        solver->Setup(data_manager);

        data_manager->system_timer.stop("ChIterativeSolverMulticore_Setup");

        // Set the initial guess for the iterative solver to zero.
        data_manager->host_data.gamma.resize(data_manager->num_constraints);
        data_manager->host_data.gamma.reset();

        // Compute the jacobian matrix, the compliance matrix and the right hand side
        data_manager->system_timer.start("ChIterativeSolverMulticore_Matrices");
        ComputeD();
        ComputeE();
        ComputeR();
        data_manager->system_timer.stop("ChIterativeSolverMulticore_Matrices");

        ShurProductBilateral.Setup(data_manager);

        bilateral_solver->Setup(data_manager);

        // Solve for the Lagrange multipliers associated with bilateral constraints.
        PerformStabilization();
    }

    // Update velocity (linear and angular)
    ComputeImpulses();

    for (int i = 0; i < data_manager->measures.solver.maxd_hist.size(); i++) {
        AtIterationEnd(data_manager->measures.solver.maxd_hist[i], data_manager->measures.solver.maxdeltalambda_hist[i],
                       i);
    }
    m_iterations = (int)data_manager->measures.solver.maxd_hist.size();
}

void ChIterativeSolverMulticoreSMC::ComputeImpulses() {
    DynamicVector<real>& v = data_manager->host_data.v;
    const DynamicVector<real>& M_invk = data_manager->host_data.M_invk;
    const DynamicVector<real>& gamma = data_manager->host_data.gamma;

    uint num_unilaterals = data_manager->num_unilaterals;
    uint num_bilaterals = data_manager->num_bilaterals;

    if (data_manager->num_constraints > 0) {
        ConstSubVectorType gamma_b = blaze::subvector(gamma, num_unilaterals, num_bilaterals);
        v = M_invk + data_manager->host_data.M_invD * gamma_b;
    } else {
        v = M_invk;
    }
}
