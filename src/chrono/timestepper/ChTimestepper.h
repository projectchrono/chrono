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

#ifndef CHTIMESTEPPER_H
#define CHTIMESTEPPER_H

#include <cstdlib>

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChFrame.h"
#include "chrono/serialization/ChArchive.h"
#include "chrono/timestepper/ChIntegrable.h"
#include "chrono/timestepper/ChState.h"

namespace chrono {

/// @addtogroup chrono_timestepper
/// @{

/// Base class for timesteppers, i.e., time integrators that can advance a system state.
class ChApi ChTimestepper {
  public:
    /// Methods for time integration.
    enum class Type {
        EULER_EXPLICIT_I,
        EULER_EXPLICIT_II,
        EULER_SEMI_IMPLICIT,
        RUNGE_KUTTA,
        HEUN,
        LEAPFROG,
        EULER_IMPLICIT,
        EULER_IMPLICIT_LINEARIZED,
        EULER_IMPLICIT_PROJECTED,
        TRAPEZOIDAL,
        TRAPEZOIDAL_LINEARIZED,
        NEWMARK,
        HHT,
        CUSTOM
    };

    virtual ~ChTimestepper() {}

    /// Return type of the integration method.
    /// Default is CUSTOM. Derived classes should override this function.
    virtual Type GetType() const = 0;

    /// Return the associated integrable object.
    virtual ChIntegrable* GetIntegrable() const = 0;

    /// Performs an integration timestep.
    virtual void Advance(double dt) = 0;

    /// Access the Lagrange multipliers, if any.
    virtual ChVectorDynamic<>& GetLagrangeMultipliers() { return L; }

    /// Get the current time.
    virtual double GetTime() const { return T; }

    /// Set the current time.
    virtual void SetTime(double mt) { T = mt; }

    /// Turn on/off logging of messages.
    void SetVerbose(bool verb) { verbose = verb; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive);

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive);

    /// Return the integrator type as a string.
    static std::string GetTypeAsString(Type type);

  protected:
    ChTimestepper(ChIntegrable* intgr = nullptr);

    bool verbose;
    double T;
    ChVectorDynamic<> L;
    bool Qc_do_clamp;
    double Qc_clamping;

    friend class ChSystem;
};

/// Base class for 1st order timesteppers, that is a time integrator for a ChIntegrable.
class ChApi ChTimestepperIorder {
  public:
    virtual ~ChTimestepperIorder() {}

    /// Access the state at current time
    virtual ChState& GetState() { return Y; }

    /// Access the derivative of state at current time
    virtual ChStateDelta& GetStateDt() { return dYdt; }

    /// Set the integrable object
    virtual void SetIntegrable(ChIntegrable* intgr);

    /// Get the integrable object.
    ChIntegrable* GetIntegrableIorder() const { return integrable; }

  protected:
    ChTimestepperIorder(ChIntegrable* intgr = nullptr);

    ChIntegrable* integrable;
    ChState Y;
    ChStateDelta dYdt;
};

/// Base class for 2nd order timesteppers, i.e., a time integrator for a ChIntegrableIIorder.
/// A ChIntegrableIIorder is a special subclass of integrable objects that have a state comprised of position and
/// velocity y={x,v}, and state derivative dy/dt={v,a}, where a=acceleration.
class ChApi ChTimestepperIIorder {
  public:
    virtual ~ChTimestepperIIorder() {}

    /// Access the state, position part, at current time
    virtual ChState& GetStatePos() { return X; }

    /// Access the state, speed part, at current time
    virtual ChStateDelta& GetStateVel() { return V; }

    /// Access the acceleration, at current time
    virtual ChStateDelta& GetStateAcc() { return A; }

    /// Set the integrable object
    virtual void SetIntegrable(ChIntegrableIIorder* intgr);

    /// Get the integrable object.
    ChIntegrableIIorder* GetIntegrableIIorder() const { return integrable; }

  protected:
    ChTimestepperIIorder(ChIntegrableIIorder* intgr = nullptr);

    ChIntegrableIIorder* integrable;
    ChState X;
    ChStateDelta V;
    ChStateDelta A;
};

/// @} chrono_timestepper

}  // end namespace chrono

#endif
