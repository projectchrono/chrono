// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHLINKPNEUMATICACTUATOR_H
#define CHLINKPNEUMATICACTUATOR_H

#include "chrono/physics/ChLinkLock.h"
#include "chrono/pneumatica/assepneumatico.h"

namespace chrono {

/// Class for pneumatic linear actuators between two markers,
/// as the piston were joined with two spherical
/// bearing at the origin of the two markers.

class ChApi ChLinkPneumaticActuator : public ChLinkLock {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChLinkPneumaticActuator)

  protected:
    pneumatics::AssePneumatico* pneuma;  ///< pointer to internal structure with all pneumatic variables

    double offset;  ///< marker distance for zero stroke

    // read-only vars  (updated in UpdateXyz() functions!!)
    double pA;      ///< pressure chamber A
    double pB;      ///< pressure chamber B
    double pA_dt;   ///< d/dt pressure chamber A
    double pB_dt;   ///< d/dt pressure chamber B
    double pneu_F;  ///< applied force

    double last_force_time;  ///< internal

  public:
    ChLinkPneumaticActuator();
    ChLinkPneumaticActuator(const ChLinkPneumaticActuator& other);
    virtual ~ChLinkPneumaticActuator();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkPneumaticActuator* Clone() const override { return new ChLinkPneumaticActuator(*this); }

    /// Updates motion laws, marker positions, etc.
    virtual void UpdateTime(double mytime) override;
    /// Updates forces etc.
    virtual void UpdateForces(double mytime) override;

    /// Direct access to all internal pneumatic data
    pneumatics::AssePneumatico* Get_pneuma() const { return pneuma; }
    /// After setting internal pneumatic datas in 'pneuma'
    void SetupActuator() { pneuma->SetupAssePneumatico(); }

    /// Joints offset for zero length stroke
    double Get_lin_offset() const { return offset; }
    void Set_lin_offset(double mset);
    /// Cylinder stroke
    double Get_pneu_L() const { return pneuma->Get_L(); }
    void Set_pneu_L(double mset);

    /// State
    double Get_pA() const { return pA; }
    double Get_pB() const { return pB; }
    double Get_pA_dt() const { return pA_dt; }
    double Get_pB_dt() const { return pB_dt; }
    /// Actual force
    double Get_pneu_F() const { return pneu_F; }
    /// Actual position & speed
    double Get_pneu_pos() const { return relM.pos.x - offset; }
    double Get_pneu_pos_dt() const { return relM_dt.pos.x; }

    // Shortcuts for valve commands

    void Set_ComA(double ma) { pneuma->Set_ComA(ma); };
    double Get_ComA() const { return pneuma->Get_ComA(); };
    void Set_ComB(double ma) { pneuma->Set_ComB(ma); };
    double Get_ComB() const { return pneuma->Get_ComB(); };

    double Get_pneu_R() const { return sqrt(pneuma->Get_A() / CH_C_PI); }
    void Set_pneu_R(double mr) {
        pneuma->Set_A(mr * mr * CH_C_PI);
        pneuma->SetupAssePneumatico();
    }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive);

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive);
};

CH_CLASS_VERSION(ChLinkPneumaticActuator,0)

}  // end namespace chrono

#endif
