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

/// Class for rotational spring-damper systems with the torque specified through a
/// functor object.
/// It is ASSUMED that the two bodies are joined such that they have a rotational
/// degree of freedom about the z axis of the specified link reference frame (i.e.,
/// they are connected through a revolute, cylindrical, or screw joint). The
/// relative angle and relative angular speed of this link are about the common axis.
class ChApi ChLinkRotSpringCB : public ChLinkMarkers {

  public:
    ChLinkRotSpringCB();
    ChLinkRotSpringCB(const ChLinkRotSpringCB& other);
    virtual ~ChLinkRotSpringCB() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkRotSpringCB* Clone() const override { return new ChLinkRotSpringCB(*this); }

    /// Get the current relative angle about the common rotation axis.
    double GetRotSpringAngle() const { return relAngle; }

    /// Get the current relative angular speed about the common rotation axis.
    double GetRotSpringSpeed() const { return Vdot(relWvel, relAxis); }

    /// Get the current generated torque.
    double GetRotSpringTorque() const { return m_torque; }

    /// Class to be used as a functor interface for calculating the general spring-damper torque.
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

    /// Specify the functor object for calculating the torque.
    void RegisterTorqueFunctor(TorqueFunctor* functor) { m_torque_fun = functor; }

    /// Include the rotational spring custom torque.
    virtual void UpdateForces(double time) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

  protected:
    TorqueFunctor* m_torque_fun;  ///< functor for torque calculation
    double m_torque;              ///< resulting torque along relative axis of rotation
};

CH_CLASS_VERSION(ChLinkRotSpringCB, 0)

}  // end namespace chrono

#endif
