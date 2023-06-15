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
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CHFUNCT_SETPOINT_H
#define CHFUNCT_SETPOINT_H

#include "chrono/motion_functions/ChFunction_Base.h"

namespace chrono {

/// @addtogroup chrono_functions
/// @{

/// Function that returns Y from an externally-provided value,
/// as a ZOH (zero order hold) block. This means that the Y value
/// does NOT change if you call Get_y(double x) with different values
/// of x, unless you keep the setpoint Y updated via multiple
/// calls to SetSetpoint(), for example calling SetSetpoint()
/// at each timestep in the simulation loop.
/// Also first two derivatives (speed, accel.) will persist until
/// next SetSetpoint() call.
/// Function of this class are most often functions of time.
class ChApi ChFunction_Setpoint : public ChFunction {
  private:
    double Y;
    double Y_dx;
    double Y_dxdx;
    double last_x;
    double last_Y;
    double last_Y_dx;

  public:
    ChFunction_Setpoint() {
        Y = 0;
        Y_dx = 0;
        Y_dxdx = 0;
        last_x = 0;
        last_Y = 0;
        last_Y_dx = 0;
    }

    ChFunction_Setpoint(const ChFunction_Setpoint& other) {
        Y = other.Y;
        Y_dx = other.Y_dx;
        Y_dxdx = other.Y_dxdx;
        last_x = other.last_x;
        last_Y = other.last_Y;
        last_Y_dx = other.last_Y_dx;
    }
    virtual ~ChFunction_Setpoint() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChFunction_Setpoint* Clone() const override { return new ChFunction_Setpoint(*this); }

    virtual double Get_y(double x) const override { return Y; }
    virtual double Get_y_dx(double x) const override { return Y_dx; }
    virtual double Get_y_dxdx(double x) const override { return Y_dxdx; }

    /// Set the setpoint, and compute its derivatives (speed, acceleration) automatically
    /// by backward differentiation (only if x is called at increasing small steps).
    /// All values will persist indefinitely until next call.
    virtual void SetSetpoint(double setpoint, double x);

    /// Set the setpoint, and also its derivatives.
    /// All values will persist indefinitely until next call.
    virtual void SetSetpointAndDerivatives(double setpoint, double setpoint_dx, double setpoint_dxdx) {
        Y = setpoint;
        Y_dx = setpoint_dx;
        Y_dxdx = setpoint_dxdx;
    }

    /// Get the last set setpoint
    double GetSetpoint() { return Y; }

    /// Update could be implemented by children classes, ex. to launch callbacks
    virtual void Update(const double x) override {}

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;
};

CH_CLASS_VERSION(ChFunction_Setpoint, 0)

/// Interface for functions that uses a callback to return a Y value,
/// as a ZOH (zero order hold) block. This means that the Y value
/// does NOT change if you call Get_y(double x) with different values
/// of x, unless you keep the setpoint Y updated via multiple
/// callback calls.
/// Also first two derivatives (speed, accel.) will persist until
/// next SetSetpoint() call.
/// To use this: you must inherit from this class and you must implement
/// the SetpointCallback() function.

class ChApi ChFunction_SetpointCallback : public ChFunction_Setpoint {
  public:
    ChFunction_SetpointCallback() {}

    ChFunction_SetpointCallback(const ChFunction_SetpointCallback& other) {}

    virtual ~ChFunction_SetpointCallback() {}

    /// You MUST implement this in inherited classes, by returning Y as a function of x.
    /// Set the setpoint, and compute its derivatives (speed, acceleration) automatically
    /// by backward differentiation (only if x is called at increasing small steps).
    /// All values will persist indefinitely until next call.
    virtual double SetpointCallback(const double x) = 0;

    /// Calling this will invoke the callback
    virtual void Update(const double x) override {
        // invokes callback
        double y = SetpointCallback(x);
        // changes the setpoint and also computes derivatives by BDF
        SetSetpoint(y, x);
    };
};

/// @} chrono_functions

CH_CLASS_VERSION(ChFunction_SetpointCallback, 0)

}  // end namespace chrono

#endif
