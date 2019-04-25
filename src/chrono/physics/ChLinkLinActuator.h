// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHLINKLINACTUATOR_H
#define CHLINKLINACTUATOR_H

#include "chrono/physics/ChLinkLock.h"

namespace chrono {

/// Class for linear actuators between two markers,
/// as the actuator were joined with two spherical
/// bearing at the origin of the two markers.
/// **NOTE! THIS IS OBSOLETE**. Prefer using the new classes 
/// inherited from chrono::ChLinkMotor.

class ChApi ChLinkLinActuator : public ChLinkLockLock {

  protected:
    std::shared_ptr<ChFunction> dist_funct;  ///< distance function
    bool learn;  ///< if true, the actuator does not apply constraint, just records the motion into its dist_function.
    bool learn_torque_rotation;  ///< if true, the actuator records the torque and rotation.
    double offset;               ///< distance offset

    double mot_tau;                          ///< motor: transmission ratio
    double mot_eta;                          ///< motor: transmission efficiency
    double mot_inertia;                      ///< motor: inertia (added to system)
    std::shared_ptr<ChFunction> mot_torque;  ///< motor: recorder of torque
    std::shared_ptr<ChFunction> mot_rot;     ///< motor: recorder of motor rotation

    double mot_rerot;       ///< current rotation (read only)  before reducer
    double mot_rerot_dt;    ///< current ang speed (read only) before reducer
    double mot_rerot_dtdt;  ///< current ang acc  (read only)  before reducer
    double mot_retorque;    ///< current motor torque (read only) before reducer

  public:
    ChLinkLinActuator();
    ChLinkLinActuator(const ChLinkLinActuator& other);
    virtual ~ChLinkLinActuator() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLinActuator* Clone() const override { return new ChLinkLinActuator(*this); }

    // Updates motion laws, marker positions, etc.
    virtual void UpdateTime(double mytime) override;

    // data get/set
    std::shared_ptr<ChFunction> Get_dist_funct() const { return dist_funct; }
    std::shared_ptr<ChFunction> Get_motrot_funct() const { return mot_rot; }
    std::shared_ptr<ChFunction> Get_mottorque_funct() const { return mot_torque; }

    void Set_dist_funct(std::shared_ptr<ChFunction> mf) { dist_funct = mf; }
    void Set_motrot_funct(std::shared_ptr<ChFunction> mf) { mot_rot = mf; }
    void Set_mottorque_funct(std::shared_ptr<ChFunction> mf) { mot_torque = mf; }

    bool Get_learn() const { return learn; }
    void Set_learn(bool mset);
    bool Get_learn_torque_rotaton() const { return learn_torque_rotation; }
    void Set_learn_torque_rotaton(bool mset);
    double Get_lin_offset() const { return offset; };
    void Set_lin_offset(double mset) { offset = mset; }

    void Set_mot_tau(double mtau) { mot_tau = mtau; }
    double Get_mot_tau() const { return mot_tau; }
    void Set_mot_eta(double meta) { mot_eta = meta; }
    double Get_mot_eta() const { return mot_eta; }
    void Set_mot_inertia(double min) { mot_inertia = min; }
    double Get_mot_inertia() const { return mot_inertia; }

    // easy fetching of motor-reduced moments or angle-speed-accel.
    double Get_mot_rerot() const { return mot_rerot; }
    double Get_mot_rerot_dt() const { return mot_rerot_dt; }
    double Get_mot_rerot_dtdt() const { return mot_rerot_dtdt; }
    double Get_mot_retorque() const { return mot_retorque; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

CH_CLASS_VERSION(ChLinkLinActuator,0)


}  // end namespace chrono

#endif
