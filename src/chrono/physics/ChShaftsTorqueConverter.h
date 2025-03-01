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
// Authors: Alessandro Tasora, Radu Serban, Rainer Gericke
// =============================================================================

#ifndef CHSHAFTSTORQUECONVERTER_H
#define CHSHAFTSTORQUECONVERTER_H

#include "chrono/functions/ChFunction.h"
#include "chrono/physics/ChShaftsCouple.h"

namespace chrono {

/// Class for defining a torque converter between two one-degree-of-freedom parts.
/// Note that is not inherited from ChShaftsTorque, because it requires a third part: the stator.
/// The torque converter multiplies the input torque if there is slippage between input and output, then the
/// multiplicative effect becomes closer to unity when the slippage is almost null; so it is similar to a
/// variable-transmission-ratio gearbox, and just like any gearbox it requires a truss (the 'stator') that gets some
/// torque. When the vehicle is coasting, the stator is always inactive, then the output torque cannot be increased.
/// Note: it can work only in a given direction.
class ChApi ChShaftsTorqueConverter : public ChPhysicsItem {
  public:
    ChShaftsTorqueConverter();
    ChShaftsTorqueConverter(const ChShaftsTorqueConverter& other);
    ~ChShaftsTorqueConverter() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChShaftsTorqueConverter* Clone() const override { return new ChShaftsTorqueConverter(*this); }

    /// Initialize the torque converter, given input and output shafts to join.
    /// The third input shaft is the stator shaft, which should be fixed.
    /// All shafts must belong to the same ChSystem.
    bool Initialize(std::shared_ptr<ChShaft> shaft_1,  ///< input shaft
                    std::shared_ptr<ChShaft> shaft_2,  ///< output shaft
                    std::shared_ptr<ChShaft> shaft_st  ///< stator shaft (often fixed)
    );

    /// Get the input shaft.
    ChShaft* GetShaftInput() const { return shaft1; }

    /// Get the output shaft.
    ChShaft* GetShaftOutput() const { return shaft2; }

    /// Get the stator shaft (the truss).
    ChShaft* GetShaftStator() const { return shaft_stator; }

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
    std::shared_ptr<ChFunction> GetCurveTorqueRatio() const { return T; }

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
    virtual void Update(double time, bool update_assets) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

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

    virtual unsigned int GetNumConstraintsBilateral() override { return 0; }
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override;
    virtual void VariablesFbLoadForces(double factor) override;
};

CH_CLASS_VERSION(ChShaftsTorqueConverter, 0)

}  // end namespace chrono

#endif
