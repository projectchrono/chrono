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



/// Class for feeders like vibrating bowls, conveyor belts. Respect to ChConveyor, this 
/// provides a more versatile functionality, where the vibrating feeder can have whatever shape.
/// The trick is that the feeder part needs not to vibrate: this ChFeeder simply modifies the
/// contact data between colliding parts and the part marked as feeder (which can be simply static)
/// so that a tangential velocity is imposed: the overall effect is like what happens for very high frequency
/// of vibration.

class ChApi ChFeeder : public ChPhysicsItem {
  private:

    std::shared_ptr<ChContactable> feeder;          ///< the feeder object, defining the surface of the vibrating feeder

  public:

    ChFeeder();
    ChFeeder(const ChFeeder& other);
    ~ChFeeder();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChFeeder* Clone() const override { return new ChFeeder(*this); }

    /// Set the pointer to the parent ChSystem() and
    /// also add to new collision system / remove from old coll.system
    //virtual void SetSystem(ChSystem* m_system) override;


    /// Set the feeder object, defining the surface of the vibrating feeder
    void SetFeederObject( std::shared_ptr<ChContactable> mfeeder) { feeder = mfeeder; }
    /// Get the feeder object, defining the surface of the vibrating feeder
    std::shared_ptr<ChContactable> GetFeederObject() { return feeder; }

    ChFrame<> reference;
    double v_x;
    double v_y;
    double v_z; 
    double w_x;
    double w_y; 
    double w_z;

    /// Set the reference for the (virtual, not simulated) vibration of the feeder, in abs space, 
    /// and the eigenvector of the vibration mode as x,y,z,rx,ry,rz about that frame; the six values also define the max speed on that axis.
    void SetFeederVibration( ChFrame<> mref, double mv_x, double mv_y, double mv_z, double mw_x, double mw_y, double mw_z) { 
        reference = mref;
        v_x = mv_x; 
        v_y = mv_y;
        v_z = mv_z;
        w_x = mw_x;
        w_y = mw_y;
        w_z = mw_z;
    }

    /// Get the reference for the (virtual, not simulated) vibration of the feeder, in abs space
    void GetFeederVibration(ChFrame<>& mref, double& mv_x, double& mv_y, double& mv_z, double& mw_x, double& mw_y, double& mw_z) {  
        mref = reference;
        mv_x = v_x; 
        mv_y = v_y;
        mv_z = v_z;
        mw_x = w_x;
        mw_y = w_y;
        mw_z = w_z;
    }



    //
    // STATE FUNCTIONS
    //

    /// Number of coordinates
    virtual int GetDOF() override { return 0; }
    /// Number of speed coordinates
    virtual int GetDOF_w() override { return 0; }
    /// Get the number of scalar constraints. 
    virtual int GetDOC_c() override { return 0; }

    // Override/implement interfaces for global state vectors (see ChPhysicsItem for details)

    virtual void IntLoadConstraint_Ct(const unsigned int off, ChVectorDynamic<>& Qc, const double c) override;


    //
    // UPDATE FUNCTIONS
    //

    /// Update all auxiliary data of the conveyor at given time
    virtual void Update(double mytime, bool update_assets = true) override;

    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;
};

CH_CLASS_VERSION(ChFeeder,0)



}  // end namespace chrono

#endif
