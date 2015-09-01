//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHLINKLINACTUATOR_H
#define CHLINKLINACTUATOR_H

#include "physics/ChLinkLock.h"

namespace chrono {

///
/// Class for linear actuators between two markers,
/// as the actuator were joined with two spherical
/// bearing at the origin of the two markers.
///
class ChApi ChLinkLinActuator : public ChLinkLock {
    CH_RTTI(ChLinkLinActuator, ChLinkLock);

  protected:
    ChSharedPtr<ChFunction> dist_funct;  // distance function
    int learn;                           // if TRUE, the actuator does not apply constraint, just
                                         // records the motion into its dist_function.
    double offset;                       // distance offset

    double mot_tau;                      // motor: transmission ratio
    double mot_eta;                      // motor: transmission efficiency
    double mot_inertia;                  // motor: inertia (added to system)
    ChSharedPtr<ChFunction> mot_torque;  // motor: recorder of torque
    ChSharedPtr<ChFunction> mot_rot;     // motor: recorder of motor rotation

    double mot_rerot;       // current rotation (read only)  before reducer
    double mot_rerot_dt;    // current ang speed (read only) before reducer
    double mot_rerot_dtdt;  // current ang acc  (read only)  before reducer
    double mot_retorque;    // current motor torque (read only) before reducer

  public:
    // builders and destroyers
    ChLinkLinActuator();
    virtual ~ChLinkLinActuator();
    virtual void Copy(ChLinkLinActuator* source);
    virtual ChLink* new_Duplicate();  // always return base link class pointer

    // UPDATING FUNCTIONS - "lin.act. link" custom implementations

    // Updates motion laws, marker positions, etc.
    virtual void UpdateTime(double mytime);

    // data get/set
    ChSharedPtr<ChFunction> Get_dist_funct() { return dist_funct; }
    ChSharedPtr<ChFunction> Get_motrot_funct() { return mot_rot; }
    ChSharedPtr<ChFunction> Get_mottorque_funct() { return mot_torque; }

    void Set_dist_funct(ChSharedPtr<ChFunction> mf) { dist_funct = mf; }
    void Set_motrot_funct(ChSharedPtr<ChFunction> mf) { mot_rot = mf; }
    void Set_mottorque_funct(ChSharedPtr<ChFunction> mf) { mot_torque = mf; }

    int Get_learn() { return learn; }
    void Set_learn(int mset);
    double Get_lin_offset() { return offset; };
    void Set_lin_offset(double mset) { offset = mset; }

    void Set_mot_tau(double mtau) { mot_tau = mtau; }
    double Get_mot_tau() { return mot_tau; }
    void Set_mot_eta(double meta) { mot_eta = meta; }
    double Get_mot_eta() { return mot_eta; }
    void Set_mot_inertia(double min) { mot_inertia = min; }
    double Get_mot_inertia() { return mot_inertia; }

    // easy fetching of motor-reduced moments or angle-speed-accel.
    double Get_mot_rerot() { return mot_rerot; }
    double Get_mot_rerot_dt() { return mot_rerot_dt; }
    double Get_mot_rerot_dtdt() { return mot_rerot_dtdt; }
    double Get_mot_retorque() { return mot_retorque; }

    // STREAMING
    virtual void StreamIN(ChStreamInBinary& mstream);
    virtual void StreamOUT(ChStreamOutBinary& mstream);
};

}  // END_OF_NAMESPACE____

#endif
