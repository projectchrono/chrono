// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#ifndef CH_TIMESTEPPER_EXPLICIT_H
#define CH_TIMESTEPPER_EXPLICIT_H

#include "chrono/timestepper/ChTimestepper.h"

namespace chrono {

/// @addtogroup chrono_timestepper
/// @{

/// Base class for explicit integrators.
/// Such integrators might require solution of a nonlinear problem if constraints are added, otherwise they can use
/// penalty in constraints and lumped masses to avoid the linear system. Diagonal lumping is off by default.
class ChApi ChTimestepperExplicit : public ChTimestepper {
  public:
    virtual ~ChTimestepperExplicit();

    /// Turn on the diagonal lumping.
    /// This can achieve a large speedup because no linear system is needeed to compute the derivative (i.e.
    /// acceleration in II order systems), but not all Chintegrable might support the diagonal lumping.
    /// - If lumping is not supported because ChIntegrable::LoadLumpedMass_Md() not implemented, throw exception.
    /// - If lumping introduces some approximation, you'll get nonzero in GetLumpingError().
    /// Optionally parameters: the stiffness penalty for constraints, and damping penalty for constraints.
    void SetDiagonalLumpingON(double Ck = 1000, double Cr = 0);

    /// Turn off the diagonal lumping (default is off)
    void SetDiagonalLumpingOFF();

    /// Gets the diagonal lumping error done last time the integrator has been called
    double GetLumpingError();

    /// Resets the diagonal lumping error.
    void ResetLumpingError();

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive);

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive);

  protected:
    ChTimestepperExplicit();

    ChLumpingParms* lumping_parameters;
};

/// Euler explicit timestepper.
/// This performs the typical  y_new = y+ dy/dt * dt integration with Euler formula.
class ChApi ChTimestepperEulerExplicitIorder : public ChTimestepperIorder, public ChTimestepperExplicit {
  public:
    ChTimestepperEulerExplicitIorder(ChIntegrable* intgr = nullptr);

    virtual Type GetType() const override { return Type::EULER_EXPLICIT_I; }

    /// Return the associated integrable object.
    virtual ChIntegrable* GetIntegrable() const override { return integrable; }

    /// Performs an integration timestep.
    virtual void Advance(double dt) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive) override;
};

/// Euler explicit timestepper customized for II order.
/// (It gives the same results of ChTimestepperEulerExplicitIorder, but performs a bit faster because it
/// can exploit the special structure of ChIntegrableIIorder)
/// This integrator implements the typical Euler scheme:
///    x_new = x + v * dt
///    v_new = v + a * dt
class ChApi ChTimestepperEulerExplicitIIorder : public ChTimestepperIIorder, public ChTimestepperExplicit {
  public:
    ChTimestepperEulerExplicitIIorder(ChIntegrableIIorder* intgr = nullptr);

    virtual Type GetType() const override { return Type::EULER_EXPLICIT_II; }

    /// Return the associated integrable object.
    virtual ChIntegrable* GetIntegrable() const override { return integrable; }

    /// Performs an integration timestep.
    virtual void Advance(double dt) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive) override;

  protected:
    ChStateDelta Dv;
};

/// Euler semi-implicit timestepper.
/// This performs the typical
///    v_new = v + a * dt
///    x_new = x + v_new * dt
/// integration with Euler semi-implicit formula.
class ChApi ChTimestepperEulerSemiImplicit : public ChTimestepperIIorder, public ChTimestepperExplicit {
  public:
    ChTimestepperEulerSemiImplicit(ChIntegrableIIorder* intgr = nullptr);

    virtual Type GetType() const override { return Type::EULER_SEMI_IMPLICIT; }

    /// Return the associated integrable object.
    virtual ChIntegrable* GetIntegrable() const override { return integrable; }

    /// Performs an integration timestep.
    virtual void Advance(double dt) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive) override;
};

/// Runke-Kutta 4th order explicit integrator.
class ChApi ChTimestepperRungeKutta : public ChTimestepperIorder, public ChTimestepperExplicit {
  public:
    ChTimestepperRungeKutta(ChIntegrable* intgr = nullptr);

    virtual Type GetType() const override { return Type::RUNGE_KUTTA; }

    /// Return the associated integrable object.
    virtual ChIntegrable* GetIntegrable() const override { return integrable; }

    /// Performs an integration timestep.
    virtual void Advance(double dt) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive) override;

  protected:
    ChState y_new;
    ChStateDelta Dydt1;
    ChStateDelta Dydt2;
    ChStateDelta Dydt3;
    ChStateDelta Dydt4;
};

/// Heun explicit integrator.
/// This scheme is similar to a 2nd order Runge Kutta.
class ChApi ChTimestepperHeun : public ChTimestepperIorder, public ChTimestepperExplicit {
  public:
    ChTimestepperHeun(ChIntegrable* intgr = nullptr);

    /// Return the associated integrable object.
    virtual ChIntegrable* GetIntegrable() const override { return integrable; }

    virtual Type GetType() const override { return Type::HEUN; }

    /// Performs an integration timestep.
    virtual void Advance(double dt) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive) override;

  protected:
    ChState y_new;
    ChStateDelta Dydt1;
    ChStateDelta Dydt2;
};

/// Leapfrog explicit integrator.
/// This is a symplectic method, with 2nd order accuracy, at least when F depends on positions only.
/// Note: uses last step acceleration: changing or resorting  the numbering of DOFs will invalidate it.
/// Suggestion: use the ChTimestepperEulerSemiImplicit, it gives the same accuracy with better performance.
class ChApi ChTimestepperLeapfrog : public ChTimestepperIIorder, public ChTimestepperExplicit {
  public:
    ChTimestepperLeapfrog(ChIntegrableIIorder* intgr = nullptr);

    virtual Type GetType() const override { return Type::LEAPFROG; }

    /// Return the associated integrable object.
    virtual ChIntegrable* GetIntegrable() const override { return integrable; }

    /// Performs an integration timestep.
    virtual void Advance(double dt) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive) override;

  protected:
    ChStateDelta Aold;
};

/// @} chrono_timestepper

}  // end namespace chrono

#endif
