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

#ifndef CHFEEDER_H
#define CHFEEDER_H

#include <cmath>
#include "chrono/physics/ChBody.h"

namespace chrono {

/// Class for feeders like vibrating bowls, conveyor belts.
/// Compared to ChConveyor, this class provides more functionality, where the vibrating feeder can have any shape. The
/// trick is that the feeder part needs not to vibrate: this ChFeeder simply modifies the contact data between colliding
/// parts and the part marked as feeder (which can be simply static) so that a tangential velocity is imposed: the
/// overall effect is like what happens for very high frequency of vibration.
class ChApi ChFeeder : public ChPhysicsItem {
  public:
    ChFeeder();
    ChFeeder(const ChFeeder& other);
    ~ChFeeder();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChFeeder* Clone() const override { return new ChFeeder(*this); }

    /// Set the feeder object, defining the surface of the vibrating feeder.
    void SetFeederObject(std::shared_ptr<ChContactable> mfeeder) { feeder = mfeeder; }

    /// Get the feeder object, defining the surface of the vibrating feeder.
    std::shared_ptr<ChContactable> GetFeederObject() { return feeder; }

    ChFrame<> reference;
    double v_x;
    double v_y;
    double v_z;
    double w_x;
    double w_y;
    double w_z;

    /// Set the reference for the (virtual, not simulated) vibration of the feeder, in abs space.
    /// Also set the eigenvector of the vibration mode as x,y,z,rx,ry,rz about that frame. The six values also define
    /// the max speed on that axis.
    void SetFeederVibration(ChFrame<> ref, double vx, double vy, double vz, double wx, double wy, double wz);

    /// Get the reference for the (virtual, not simulated) vibration of the feeder, in abs space.
    void GetFeederVibration(ChFrame<>& ref, double& vx, double& vy, double& vz, double& wx, double& wy, double& wz);

    /// Number of coordinates.
    virtual unsigned int GetNumCoordsPosLevel() override { return 0; }

    /// Number of speed coordinates.
    virtual unsigned int GetNumCoordsVelLevel() override { return 0; }

    /// Get the number of scalar constraints.
    virtual unsigned int GetNumConstraintsBilateral() override { return 0; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    std::shared_ptr<ChContactable> feeder;  ///< the feeder object, defining the surface of the vibrating feeder

    virtual void Update(double time, bool update_assets) override;
    virtual void IntLoadConstraint_Ct(const unsigned int off, ChVectorDynamic<>& Qc, const double c) override;
};

CH_CLASS_VERSION(ChFeeder, 0)

}  // end namespace chrono

#endif
