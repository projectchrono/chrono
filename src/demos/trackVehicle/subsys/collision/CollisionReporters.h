// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2015 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Justin Madsen
// =============================================================================
//
// Contact callback class implementations to report contact info
//
// =============================================================================

#ifndef COLLISIONREPORTERS_H
#define COLLISIONREPORTERS_H

#include "physics/ChContactContainer.h"

namespace chrono {

// scans the contacts, reports the desired info about the gear, including:
//  max, sum and variance of both normal and friction forces
class _gear_report_contact : public ChReportContactCallback {
  public:
    _gear_report_contact(const ChSharedPtr<ChBody> gear)
        : m_gear(gear),
          m_num_gear_contacts(0),
          m_Fn_max(0),
          m_Fn_sum(0),
          m_Fn_var(0),
          m_Ft_max(0),
          m_Ft_sum(0),
          m_Ft_var(0) {}

    virtual bool ReportContactCallback(const ChVector<>& pA,
                                       const ChVector<>& pB,
                                       const ChMatrix33<>& plane_coord,
                                       const double& distance,
                                       const float& mfriction,
                                       const ChVector<>& react_forces,
                                       const ChVector<>& react_torques,
                                       collision::ChCollisionModel* modA,
                                       collision::ChCollisionModel* modB) {
        // does this collision include the gear?
        if (modA == m_gear->GetCollisionModel() || modB == m_gear->GetCollisionModel()) {
            // don't count collisions w/ 0 normal force
            if (react_forces.x > 0) {
                // average value of norm, friction forces from last time scanned contact
                double Fn_bar_n1 = 0;
                double Ft_bar_n1 = 0;
                if (m_num_gear_contacts > 0) {
                    Fn_bar_n1 = m_Fn_sum / m_num_gear_contacts;
                    Ft_bar_n1 = m_Ft_sum / m_num_gear_contacts;
                }
                // increment the data for this contact
                m_num_gear_contacts++;
                // incrase normal force
                m_Fn_sum += react_forces.x;
                // current friction magnitude in local c-syss
                double Ft = ChVector<>(0, react_forces.y, react_forces.z).Length();
                m_Ft_sum += Ft;

                // update max normal and force magnitude
                if (react_forces.x > m_Fn_max)
                    m_Fn_max = react_forces.x;
                if (Ft > m_Ft_max)
                    m_Ft_max = Ft;

                // compute some statistics.
                // update normal force avg, variance
                double Fn_bar = Fn_bar_n1 + (react_forces.x - Fn_bar_n1) / m_num_gear_contacts;
                // Fn_var is from last step, e.g. Fn_var_n1
                double Fn_sig2 =
                    (m_Fn_var * (m_num_gear_contacts - 1) + (react_forces.x - Fn_bar_n1) * (react_forces.x - Fn_bar)) /
                    m_num_gear_contacts;
                // with updated average and variance, update the data
                m_Fn_var = Fn_sig2;

                // similarly for the friction forces
                double Ft_bar = Ft_bar_n1 + (Ft - Ft_bar_n1) / m_num_gear_contacts;
                // like the normal force, the Ft_var has not been updated this step, so it is actually Ft_var_n1
                double Ft_sig2 =
                    ((m_num_gear_contacts - 1) * m_Ft_var + (Ft - Ft_bar_n1) * (Ft - Ft_bar)) / m_num_gear_contacts;
                // with  updated variance, update the data
                m_Ft_var = Ft_sig2;
            }
        }
        return true;  // to continue scanning contacts
    }

    ChSharedPtr<ChBody> m_gear;  // gear body to eval collision with
    // NOTE: must initialize these before use!
    // relevant gear info
    int m_num_gear_contacts;
    double m_Fn_max;  // max normal force
    double m_Fn_sum;  // reaction normal force sum
    double m_Fn_var;  // reaction normal force population variance, sigma_n^2

    double m_Ft_max;  // tangent/friction force max
    double m_Ft_sum;  // friction force sum
    double m_Ft_var;  // friction force variance, sigma_t^2
};


// --------------------------------------------------------------------------------------
// scans the contacts, reports the desired info about any shoe 0 and gear collision pairs
class _shoeGear_report_contact : public ChReportContactCallback {
  public:
    _shoeGear_report_contact(const std::string& shoe_name) : m_num_contacts(0) {
        m_num_contacts_side.resize(2, 0);
        m_PosRel.resize(2, ChVector<>());
        m_PosAbs.resize(2, ChVector<>());
        m_VelRel.resize(2, ChVector<>());
        m_NormDirRel.resize(2, ChVector<>());
        m_NormDirAbs.resize(2, ChVector<>());
        // normal, friction force magnitude
        m_Fn.resize(2, 0);  // normal reaction force
        m_Ft.resize(2, 0);  // friction reaction force{}
    }

    // get the location of the contact, on the specified body type, with specified body COG pos in global coords.
    // If cannot find the collision type for either modA or modB family, just use modB
    const ChVector<> getContactLocRel(const ChVector<>& pA,
                                      const ChVector<>& pB,
                                      collision::ChCollisionModel* modA,
                                      collision::ChCollisionModel* modB,
                                      CollisionFam::Enum fam) {
        ChVector<> pos_abs = ChVector<>();
        ChFrame<> body_frame = ChFrame<>();
        // is the collision body A?
        if (modA->GetFamily() == (int)fam) {
            // A is the gear body
            pos_abs =
                pA - ((ChBody*)modA->GetPhysicsItem())->GetPos();  // abs dist. of contact point from the gear center
            body_frame.SetRot(((ChBody*)modA->GetPhysicsItem())->GetRot());
        } else {
            // assume gear is body B
            pos_abs =
                pB - ((ChBody*)modB->GetPhysicsItem())->GetPos();  // abs dist. of contact point from the gear center
            body_frame.SetRot(((ChBody*)modB->GetPhysicsItem())->GetRot());
        }

        // get the distance from gear center to contact point on the gear, in gear c-sys
        ChVector<> loc_rel = body_frame.TransformParentToLocal(pos_abs);
        return loc_rel;
    }

    // get the normal direction of the contact normal, on the specified body type,
    //  relative to the body c-sys
    // If cannot find the collision type for either modA or modB family, just use modB
    // plane_coord has normal w.r.t. modA
    const ChVector<> getContactNormDirRel(const ChVector<> normDir_abs,
                                          collision::ChCollisionModel* modA,
                                          collision::ChCollisionModel* modB,
                                          CollisionFam::Enum fam) {
        // TODO: does the normal direction switch depending on which collision model ends up being = to fam ???
        ChFrame<> body_frame = ChFrame<>();     // should end up being the gear body orientation
        ChVector<> normDir_rel = ChVector<>();  // relative to gear c-sys
                                                // is the collision body A?
        if (modA->GetFamily() == (int)fam) {
            body_frame.SetRot(((ChBody*)modA->GetPhysicsItem())->GetRot());
            normDir_rel = body_frame.TransformParentToLocal(normDir_abs);
        } else {
            // assume collision is body B
            body_frame.SetRot(((ChBody*)modB->GetPhysicsItem())->GetRot());
            normDir_rel = body_frame.TransformParentToLocal(-normDir_abs);
        }

        return normDir_rel;
    }

    // is this contact point on the gear body close enough to the one we're tracking?
    // NOTE: per_step_len_max needs to be set depending on how much the contact point can
    //    logically move between the two times you called this function.
    bool is_PosRel_match(const ChVector<>& test_PosRel,
                         int idx,
                         double per_step_len_max = 0.01  ///< relative location of contact on gear body is close
                         /// enough to tracked contact to be used as the one we
                         /// follow
                         ) {
        if ((m_PosRel[idx] - test_PosRel).Length() < per_step_len_max) {
            return true;
        }

        return false;
    }

    // based on specified Collision fam, set the contact data for the specified persistent contact index:
    // m_Fn
    // m_Ft
    // m_NormDirRel
    // m_PosAbs
    // mNormDirAbs
    void SetPeristentContactInfo(const ChVector<>& pA,
                                 const ChVector<>& pB,
                                 const ChMatrix33<>& plane_coord,
                                 const ChVector<>& react_forces,
                                 collision::ChCollisionModel* modA,
                                 collision::ChCollisionModel* modB,
                                 CollisionFam::Enum fam,
                                 int idx) {
        m_Fn[idx] = react_forces.x;
        m_Ft[idx] = ChVector<>(0, react_forces.y, react_forces.z).Length();
        if (modA->GetFamily() == (int)fam) {
            // gear body is A, normal will point in the right direction
            m_NormDirRel[idx] = getContactNormDirRel(plane_coord.Get_A_Xaxis(), modA, modB, CollisionFam::Gear);
            m_PosAbs[idx] = pA;
            m_NormDirAbs[idx] = plane_coord.Get_A_Xaxis();
        } else {
            // gear body is B, switch normal dir
            m_NormDirRel[idx] = getContactNormDirRel(-(plane_coord.Get_A_Xaxis()), modA, modB, CollisionFam::Gear);
            m_PosAbs[idx] = pB;
            m_NormDirAbs[idx] = -(plane_coord.Get_A_Xaxis());
        }
    }

    // add the absolute location, absolute normal force vector for the specified
    // collisionFam, for all the contacts between the shoe and gear
    void AddContactInfo_all(const ChVector<>& pA,
                            const ChVector<>& pB,
                            const ChMatrix33<>& plane_coord,
                            const ChVector<>& react_forces,
                            collision::ChCollisionModel* modA,
                            collision::ChCollisionModel* modB,
                            CollisionFam::Enum fam) {
        if (modA->GetFamily() == (int)fam) {
            // gear body is A, normal will point in the right direction
            m_ContactPos_all.push_back(pA);
            m_ContactFn_all.push_back(plane_coord.Get_A_Xaxis() * react_forces.x);
        } else {
            // fam body is B
            m_ContactPos_all.push_back(pB);
            m_ContactFn_all.push_back(-(plane_coord.Get_A_Xaxis()) * react_forces.x);
        }
    }

    virtual bool ReportContactCallback(const ChVector<>& pA,
                                       const ChVector<>& pB,
                                       const ChMatrix33<>& plane_coord,
                                       const double& distance,
                                       const float& mfriction,
                                       const ChVector<>& react_forces,
                                       const ChVector<>& react_torques,
                                       collision::ChCollisionModel* modA,
                                       collision::ChCollisionModel* modB) {
        // if in contact with the gear, and the other body is the specified shoe
        if ((modA->GetFamily() == (int)CollisionFam::Gear && modB->GetPhysicsItem()->GetNameString() == m_shoe_name) ||
            (modB->GetFamily() == (int)CollisionFam::Gear && modA->GetPhysicsItem()->GetNameString() == m_shoe_name)) {
            // don't count collisions w/ normal force = 0
            if (react_forces.x > 0) {
                // this is a non-zero contact force between the shoe and the gear
                m_num_contacts++;

                // get the relative location of the contact point on the gear body
                ChVector<> gearpt_PosRel = getContactLocRel(pA, pB, modA, modB, CollisionFam::Gear);

                // positive z-relative position will be index 0, else index 1.
                int index = (gearpt_PosRel.z > 0) ? 0 : 1;
                // count the number of contats on each side
                m_num_contacts_side[index]++;
                // see if the relative position of the contact point has been set on this side of the gear
                if (!m_is_persistentContact_set[index]) {
                    // set the persistent contact info on this side of the gear
                    m_PosRel[index] = gearpt_PosRel;
                    // use this relative position next time
                    m_is_persistentContact_set[index] = true;

                    // set some other useful data
                    // get the normal force direction relative to the gear
                    SetPeristentContactInfo(pA, pB, plane_coord, react_forces, modA, modB, CollisionFam::Gear, index);

                    if (0) {
                        GetLog() << " \n\n ***********     setting the gear contact point at time : "
                                 << modA->GetPhysicsItem()->GetSystem()->GetChTime() << " , collision index: " << index
                                 << "\n rel pos : " << m_PosRel[index] << "\n";
                    }
                } else {
                    // relative loc has already been set.
                    // See if this relative contact position is close to already set location.
                    // If within some relative distance, assume this is the same contact from last step.
                    if (is_PosRel_match(gearpt_PosRel, index)) {
                        // increment the total time this contact has been persistent
                        m_t_persist[index] += modA->GetPhysicsItem()->GetSystem()->GetStep();
                        // see if the time is larger than the last max
                        if (m_t_persist[index] > m_t_persist_max[index])
                            m_t_persist_max[index] = m_t_persist[index];

                        // set the pos, vel data
                        // velocity is the relative distance between the contact pts between time steps, w.r.t. gear
                        // c-sys
                        m_VelRel[index] =
                            (gearpt_PosRel - m_PosRel[index]) / modA->GetPhysicsItem()->GetSystem()->GetStep();
                        // done using last step relative position of contact pt on gear body, update for current
                        // contact
                        m_PosRel[index] = gearpt_PosRel;

                        // set the other data
                        SetPeristentContactInfo(pA, pB, plane_coord, react_forces, modA, modB, CollisionFam::Gear,
                                                index);

                    } else {
                        // this is a different the point on the shoe than the one tracked,
                        // want to be able to plot it in Irrlicht
                        AddContactInfo_all(pA, pB, plane_coord, react_forces, modA, modB, CollisionFam::Gear);
                    }
                }
            }
        }

        return true;  // to continue scanning contacts
    }

    // NOTE: must initialize
    // relevant gear info
    std::string m_shoe_name;  // string name for the shoe to check for collision with
    int m_num_contacts;       // contacts this step

    // NOTE: for this type of gear/shoe pin contact, two pins on each shoe are in contact with
    //      the gear. EXPECT to have symmetric contact behavior, and a single contact point,
    //      where the contact force magnitude and relative direction vector vary smootly through time.
    // So, track two contacts for the shoe/gear contact, one w/ positive z "Pz", the other negative z, "Nz"
    std::vector<double> m_t_persist;      // time the contacts have been in persistent contact
    std::vector<double> m_t_persist_max;  // max time the shoe stayed in contact with the gear
    std::vector<int> m_num_contacts_side;
    std::vector<ChVector<> > m_PosRel;  // position of a contact to follow, relative to gear c-sys
    std::vector<ChVector<> > m_PosAbs;
    std::vector<ChVector<> > m_VelRel;      // velocity of contact followed, relative to gear c-sys
    std::vector<ChVector<> > m_NormDirRel;  // relative norm. dir of contact
    std::vector<ChVector<> > m_NormDirAbs;
    std::vector<bool> m_is_persistentContact_set;  // has a contact point to follow been chosen? if yes, check to
                                                   // see, else write to above coord

    std::vector<double> m_Fn;  // normal force this step
    std::vector<double> m_Ft;  // max friction force

    // all other contacts
    int m_num_shoeGear_contacts_Pz;             // # of contacts with z-positive, relative to gear c-sys
    int m_num_shoeGear_contacts_Nz;             // # of contacts with z-negative, relative to gear c-sys
    std::vector<ChVector<> > m_ContactPos_all;  // abs. coords, for irrlicht
    std::vector<ChVector<> > m_ContactFn_all;   // abs. coords, for irrlicht vis
};

}  // end namespace chrono

#endif  // COLLISIONREPORTERS_H