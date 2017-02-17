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

#ifndef CHSHAFTSTORQUECONVERTER_H
#define CHSHAFTSTORQUECONVERTER_H

#include "chrono/motion_functions/ChFunction.h"
#include "chrono/physics/ChShaftsCouple.h"

namespace chrono {

/// Class for defining a torque converter between two one-degree-of-freedom parts;
/// i.e., shafts that can be used to build 1D models of powertrains. Note that is
/// not inherited from ChShaftsTorqueBase, because it requires a third part: the stator.
/// The torque converter multiplies the input torque if there is slippage between input
/// and output, then the multiplicative effect becomes closer to unity when the slippage
/// is almost null; so it is similar to a variable-transmission-ratio gearbox, and just
/// like any gearbox it requires a truss (the 'stator') that gets some torque.
/// Note: it can work only in a given direction.

class ChApi ChShaftsTorqueConverter : public ChPhysicsItem {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChShaftsTorqueConverter)

  private:
    ChShaft* shaft1;
    ChShaft* shaft2;
    ChShaft* shaft_stator;

    double torque_in;
    double torque_out;

    std::shared_ptr<ChFunction> K;
    std::shared_ptr<ChFunction> T;

    bool state_warning_reverseflow;
    bool state_warning_wrongimpellerdirection;

  public:
    ChShaftsTorqueConverter();
    ChShaftsTorqueConverter(const ChShaftsTorqueConverter& other);
    ~ChShaftsTorqueConverter() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChShaftsTorqueConverter* Clone() const override { return new ChShaftsTorqueConverter(*this); }

    /// Number of scalar constraints
    virtual int GetDOC_c() override { return 0; }

    // (override/implement interfaces for global state vectors, see ChPhysicsItem for comments.)
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override;

    // Override/implement system functions of ChPhysicsItem
    // (to assemble/manage data for system solver)

    virtual void VariablesFbLoadForces(double factor) override;

    /// Use this function after torque converter creation, to initialize it, given
    /// input and output shafts to join (plus the stator shaft, that should be fixed).
    /// Each shaft must belong to the same ChSystem.
    bool Initialize(std::shared_ptr<ChShaft> mshaft1,       ///< input shaft
                    std::shared_ptr<ChShaft> mshaft2,       ///< output shaft
                    std::shared_ptr<ChShaft> mshaft_stator  ///< stator shaft (often fixed)
                    );

    /// Get the input shaft
    ChShaft* GetShaftInput() { return shaft1; }
    /// Get the output shaft
    ChShaft* GetShaftOutput() { return shaft2; }
    /// Get the stator shaft (the truss)
    ChShaft* GetShaftStator() { return shaft_stator; }

    /// Set the capacity factor curve, function of speed ratio R.
    /// It is K(R)= input speed / square root of the input torque.
    /// Units: (rad/s) / sqrt(Nm)
    void SetCurveCapacityFactor(std::shared_ptr<ChFunction> mf) { K = mf; }
    /// Get the capacity factor curve.
    std::shared_ptr<ChFunction> GetCurveCapacityFactor() { return K; }

    /// Set the torque ratio curve, function of speed ratio R.
    /// It is T(R) = (output torque) / (input torque)
    void SetCurveTorqueRatio(std::shared_ptr<ChFunction> mf) { T = mf; }
    /// Get the torque ratio curve.
    std::shared_ptr<ChFunction> GetCurveTorqueRatio() { return T; }

    /// Get the torque applied to the input shaft
    double GetTorqueReactionOnInput() const { return torque_in; }

    /// Get the torque applied to the output shaft
    double GetTorqueReactionOnOutput() const { return torque_out; }

    /// Get the torque applied to the stator shaft (the truss)
    double GetTorqueReactionOnStator() const { return -torque_out - torque_in; }

    /// Get the actual peed ratio, as output speed / input speed.
    /// Assumes output has same direction as input, and slower than input
    /// otherwise exchanges input and output.
    /// For speed ratio = 0, complete slippage, for ratio=1 perfect locking.
    double GetSpeedRatio() const;

    /// Get the actual slippage, for slippage = 1 complete slippage,
    /// for slippage = 0 perfect locking.
    double GetSlippage() const { return 1.0 - GetSpeedRatio(); }

    /// State warning, at last update. Tell if the torque converter is working
    /// in reverse power flow, i.e. the output turbine is running faster than
    /// input impeller shaft.
    bool StateWarningReverseFlow() { return state_warning_reverseflow; }

    /// State warning, at last update. Tell if the torque converter is working
    /// with the input shaft in the reverse direction (negative speed).
    /// This is considered an abnormal behavior, and torques are forced to zero.
    bool StateWarningWrongImpellerDirection() { return state_warning_wrongimpellerdirection; }

    /// Update all auxiliary data of the gear transmission at given time
    virtual void Update(double mytime, bool update_assets = true) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

CH_CLASS_VERSION(ChShaftsTorqueConverter,0)


}  // end namespace chrono

#endif
