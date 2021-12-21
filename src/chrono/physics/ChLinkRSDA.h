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
// Authors: Radu Serban
// =============================================================================
// TODO:
// - derive directly from ChLink
// - add optional internal dynamics (described by ODEs)
// - add optional support for computing Jacobians of generalized forces
// =============================================================================

#ifndef CH_LINK_RSDA_H
#define CH_LINK_RSDA_H

#include "chrono/physics/ChLinkMarkers.h"

namespace chrono {

/// Class for rotational spring-damper-actuator (RSDA) with the torque specified through a functor object.
/// By default, models a linear RSDA. The torque is applied in the current direction of the relative axis of rotation.
/// While a rotational spring-damper can be associated with any pair of bodies in the system, it is typically used in
/// cases where the mechanism kinematics are such that the two bodies have a single relative rotational degree of
/// freedom (e.g, between two bodies connected through a revolute, cylindrical, or screw joint).
class ChApi ChLinkRSDA : public ChLinkMarkers {
  public:
    ChLinkRSDA();
    ChLinkRSDA(const ChLinkRSDA& other);
    virtual ~ChLinkRSDA() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkRSDA* Clone() const override { return new ChLinkRSDA(*this); }

    /// Set spring coefficient (default: 0).
    /// Used only if no torque functor is provided.
    void SetSpringCoefficient(double k) { m_k = k; }

    /// Set damping coefficient (default: 0).
    /// Used only if no torque functor is provided.
    void SetDampingCoefficient(double r) { m_r = r; }

    /// Set constant actuation torque (default: 0).
    /// Used only if no torque functor is provided.
    void SetActuatorTorque(double t) { m_t = t; }

    /// Get the current rotation angle about the relative rotation axis.
    double GetRotSpringAngle() const { return relAngle; }

    /// Get the current relative axis of rotation.
    const ChVector<>& GetRotSpringAxis() const { return relAxis; }

    /// Get the current relative angular speed about the common rotation axis.
    double GetRotSpringSpeed() const { return Vdot(relWvel, relAxis); }

    /// Get the current generated torque.
    double GetRotSpringTorque() const { return m_torque; }

    /// Get the value of the spring coefficient.
    /// Meaningful only if no torque functor is provided.
    double GetSpringCoefficient() const { return m_k; }

    /// Get the value of the damping coefficient.
    /// Meaningful only if no torque functor is provided.
    double GetDampingCoefficient() const { return m_r; }

    /// Get the constant acutation torque.
    /// Meaningful only if no torque functor is provided.
    double GetActuatorTorque() const { return m_t; }

    /// Class to be used as a callback interface for calculating the general spring-damper torque.
    /// A derived class must implement the virtual operator().
    class ChApi TorqueFunctor {
      public:
        virtual ~TorqueFunctor() {}

        /// Calculate and return the general spring-damper torque at the specified configuration.
        virtual double evaluate(double time,      ///< current time
                                double angle,     ///< relative angle of rotation
                                double vel,       ///< relative angular speed
                                ChLinkRSDA* link  ///< back-pointer to associated link
                                ) = 0;
    };

    /// Specify the callback object for calculating the torque.
    void RegisterTorqueFunctor(std::shared_ptr<TorqueFunctor> functor) { m_torque_fun = functor; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

  protected:
    /// Include the rotational spring-damper custom torque.
    virtual void UpdateForces(double time) override;

    double m_k;  ///< spring coefficient (if no torque functor provided)
    double m_r;  ///< damping coefficient (if no torque functor provided)
    double m_t;  ///< constant actuation (if no torque functor provided)

    std::shared_ptr<TorqueFunctor> m_torque_fun;  ///< functor for torque calculation
    double m_torque;                              ///< resulting torque along relative axis of rotation
};

CH_CLASS_VERSION(ChLinkRSDA, 0)

}  // end namespace chrono

#endif
