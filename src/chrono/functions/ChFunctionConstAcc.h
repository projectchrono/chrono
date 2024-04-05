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

#ifndef CHFUNCT_CONSTACC_H
#define CHFUNCT_CONSTACC_H

#include "chrono/functions/ChFunctionBase.h"

namespace chrono {

/// @addtogroup chrono_functions
/// @{

/// Constant acceleration function.
///
/// The function consists of three segments:
/// - constant acceleration (+acc): 0.0 <= x/duration < acceleration1_end
/// - constant velocity: acceleration1_end <= x/duration < acceleration2_start
/// - constant deceleration (-acc): acceleration2_start <= x/duration <= 1.0
///
/// End of the first acceleration and start of the second acceleration are specified as fractions of the total duration.
///
/// The function takes care of computing the acceleration 'acc' so to achieve the desired final displacement in the
/// given duration.
class ChApi ChFunctionConstAcc : public ChFunction {
  private:
    double m_displacement;
    double m_accel1_end;
    double m_accel2_start;
    double m_duration;

  public:
    ChFunctionConstAcc();
    ChFunctionConstAcc(double displacement, double acceleration1_end, double acceleration2_start, double duration);
    ChFunctionConstAcc(const ChFunctionConstAcc& other);
    ~ChFunctionConstAcc() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChFunctionConstAcc* Clone() const override { return new ChFunctionConstAcc(*this); }

    virtual Type GetType() const override { return ChFunction::Type::CONSTACC; }

    virtual double GetVal(double x) const override;
    virtual double GetDer(double x) const override;
    virtual double GetDer2(double x) const override;

    /// Set duration of the double acceleration.
    void SetDuration(double duration);

    /// Set the end of the first acceleration ramp, as a fraction of the total duration [0, 1)
    /// Must be less than the start of the second acceleration ramp.
    void SetFirstAccelerationEnd(double acceleration1_end);

    /// Set the start of the second acceleration ramp, as a fraction of the total duration [0, 1)
    /// Must be greater than the end of the first acceleration ramp.
    void SetSecondAccelerationStart(double acceleration2_start);

    /// Set the desired displacement.
    /// The function will compute the accelerations so to achieve this displacement in the given duration.
    void SetDisplacement(double displacement) { m_displacement = displacement; }

    /// Set the end of the first acceleration ramp and the start of the second acceleration ramp.
    void SetAccelerationPoints(double acceleration1_end, double acceleration2_start);

    /// Get the duration of the double acceleration.
    double GetDuration() const { return m_duration; }

    /// Get the end of the first acceleration ramp, as a fraction of the total duration.
    double GetFirstAccelerationEnd() const { return m_accel1_end; }

    /// Get the start of the second acceleration ramp, as a fraction of the total duration.
    double GetSecondAccelerationStart() const { return m_accel2_start; }

    /// Get the desired displacement.
    double GetDisplacement() const { return m_displacement; }

    /// Get the positive acceleration coefficient.
    virtual double GetPositiveAccelerationCoeff() const override;

    /// Get the negative acceleration coefficient.
    virtual double GetNegativeAccelerationCoeff() const override;

    /// Get the velocity coefficient.
    virtual double GetVelocityCoefficient() const override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;
};

/// @} chrono_functions

CH_CLASS_VERSION(ChFunctionConstAcc, 0)

}  // end namespace chrono

#endif
