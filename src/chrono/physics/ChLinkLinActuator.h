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

#ifndef CH_LINK_LINACTUATOR_H
#define CH_LINK_LINACTUATOR_H

#include "chrono/physics/ChLinkLock.h"

namespace chrono {

/// Linear actuator between two markers on two rigid bodies
/// The distance between the two markers changes in time following a user-provided function.
class ChApi ChLinkLinActuator : public ChLinkLockLock {
  public:
    ChLinkLinActuator();
    ChLinkLinActuator(const ChLinkLinActuator& other);
    virtual ~ChLinkLinActuator() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLinActuator* Clone() const override { return new ChLinkLinActuator(*this); }

    /// Set the actuation function of time d(t).
    void SetActuatorFunction(std::shared_ptr<ChFunction> mf) { dist_funct = mf; }

    /// Get the actuation function of time d(t).
    std::shared_ptr<ChFunction> GetActuatorFunction() const { return dist_funct; }

    /// Set a constant distance offset.
    /// This value may be required to prevent the singular configuration where the two markers coincide (d = 0).  If the
    /// mechanism can reach such a configuration, set a positive offset large enough to ensure that d(t) + offset > 0 at
    /// all times.
    void SetDistanceOffset(double mset) { offset = mset; }

    /// Get the value of the distance offset.
    double GetDistanceOffset() const { return offset; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

  private:
    /// Updates motion laws, marker positions, etc.
    virtual void UpdateTime(double mytime) override;

    std::shared_ptr<ChFunction> dist_funct;  ///< distance function
    double offset;                           ///< distance offset
};

CH_CLASS_VERSION(ChLinkLinActuator, 0)

}  // end namespace chrono

#endif
