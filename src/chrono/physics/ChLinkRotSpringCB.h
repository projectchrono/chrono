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
// Authors: Radu Serban
// =============================================================================

#ifndef CHLINKROTSPRINGCB_H
#define CHLINKROTSPRINGCB_H

#include "chrono/physics/ChLinkMarkers.h"

namespace chrono {

/// Base callback function for implementing a general rotational spring-damper force.
/// A derived class must implement the virtual operator().

class ChRotSpringTorqueCallback {
  public:
    virtual ~ChRotSpringTorqueCallback() {}

    virtual double operator()(double time,   ///< current time
                              double angle,  ///< relative angle of rotation
                              double vel     ///< relative angular speed
                              ) = 0;
};

/// Class for rotational spring-damper systems with the torque specified through a
/// callback object.
/// It is ASSUMED that the two bodies are joined such that they have a rotational
/// degree of freedom about the z axis of the specified link reference frame (i.e.,
/// they are connected through a revolute, cylindrical, or screw joint). The
/// relative angle and relative angular speed of this link are about the common axis.

class ChApi ChLinkRotSpringCB : public ChLinkMarkers {
    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChLinkRotSpringCB)

  protected:
    ChRotSpringTorqueCallback* m_torque_fun;  ///< functor for torque calculation
    double m_torque;                          ///< resulting torque along relative axis of rotation

  public:
    ChLinkRotSpringCB();
    ChLinkRotSpringCB(const ChLinkRotSpringCB& other);
    virtual ~ChLinkRotSpringCB() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkRotSpringCB* Clone() const override { return new ChLinkRotSpringCB(*this); }

    /// Get the current relative angle about the common rotation axis.
    double Get_RotSpringAngle() const { return relAngle; }

    /// Get the current relative angular speed about the common rotation axis.
    double Get_RotSpringSpeed() const { return Vdot(relWvel, relAxis); }

    /// Get the current generated torque.
    double Get_RotSpringTorque() const { return m_torque; }

    /// Set the torque calculation callback functor.
    void Set_RotSpringCallback(ChRotSpringTorqueCallback* torque) { m_torque_fun = torque; }

    /// Include the rotational spring custom torque.
    virtual void UpdateForces(double time) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

CH_CLASS_VERSION(ChLinkRotSpringCB,0)


}  // end namespace chrono

#endif
