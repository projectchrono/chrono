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

#ifndef CH_LINK_ROTSPRING_CB_H
#define CH_LINK_ROTSPRING_CB_H

#include "chrono/physics/ChLinkMarkers.h"

namespace chrono {

/// Class for rotational spring-damper elements with the torque specified through a callback object.
/// The torque is applied in the current direction of the relative axis of rotation.
/// While a rotational spring-damper can be associated with any pair of bodies in the system, it is
/// typically used in cases where the mechanism kinematics are such that the two bodies have a single
/// relative rotational degree of freedom (e.g, between two bodies connected through a revolute,
/// cylindrical, or screw joint). 
class ChApi ChLinkRotSpringCB : public ChLinkMarkers {

  public:
    ChLinkRotSpringCB();
    ChLinkRotSpringCB(const ChLinkRotSpringCB& other);
    virtual ~ChLinkRotSpringCB() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkRotSpringCB* Clone() const override { return new ChLinkRotSpringCB(*this); }

    /// Get the current rotation angle about the relative rotation axis.
    double GetRotSpringAngle() const { return relAngle; }

    /// Get the current relative axis of rotation.
    const ChVector<>& GetRotSpringAxis() const { return relAxis; }

    /// Get the current relative angular speed about the common rotation axis.
    double GetRotSpringSpeed() const { return Vdot(relWvel, relAxis); }

    /// Get the current generated torque.
    double GetRotSpringTorque() const { return m_torque; }

    /// Class to be used as a callback interface for calculating the general spring-damper torque.
    /// A derived class must implement the virtual operator().
    class ChApi TorqueFunctor {
      public:
        virtual ~TorqueFunctor() {}

        /// Calculate and return the general spring-damper torque at the specified configuration.
        virtual double operator()(double time,             ///< current time
                                  double angle,            ///< relative angle of rotation
                                  double vel,              ///< relative angular speed
                                  ChLinkRotSpringCB* link  ///< back-pointer to associated link
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

    std::shared_ptr<TorqueFunctor> m_torque_fun;  ///< functor for torque calculation
    double m_torque;                              ///< resulting torque along relative axis of rotation
};

CH_CLASS_VERSION(ChLinkRotSpringCB, 0)

}  // end namespace chrono

#endif
