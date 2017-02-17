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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHSHAFTHERMALENGINE_H
#define CHSHAFTHERMALENGINE_H

#include "chrono/motion_functions/ChFunction.h"
#include "chrono/physics/ChShaftsTorqueBase.h"

namespace chrono {

/// Class for defining a thermal engine between two one-degree-of-freedom parts;
/// i.e., shafts that can be used to build 1D models of power trains.
/// The first shaft is the 'crankshaft' to whom the torque is applied, the second
/// is the botor block, that receives the negative torque.

class ChApi ChShaftsThermalEngine : public ChShaftsTorqueBase {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChShaftsThermalEngine)

  private:
    std::shared_ptr<ChFunction> Tw;  ///< torque as function of angular vel.
    double throttle;

    bool error_backward;

  public:
    ChShaftsThermalEngine();
    ChShaftsThermalEngine(const ChShaftsThermalEngine& other);
    ~ChShaftsThermalEngine() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChShaftsThermalEngine* Clone() const override { return new ChShaftsThermalEngine(*this); }

    /// Set the torque curve T(w), function of angular speed between shaft1 and shaft2.
    /// Output units: [Nm]  , input units: [rad/s]
    void SetTorqueCurve(std::shared_ptr<ChFunction> mf) { Tw = mf; }
    /// Get the torque curve T(w).
    std::shared_ptr<ChFunction> GetTorqueCurve() { return Tw; }

    /// Set the current throttle value 's' in [0,1] range. If s=1,
    /// the torque is exactly T=T(w), otherwise it is linearly
    /// scaled as T=T(w)*s.
    /// This is a simplified model of real torque modulation, but
    /// nough for a basic model. An advanced approach would require a 2D map T(w,s)
    void SetThrottle(double mt) { throttle = mt; }
    /// Get the current throttle value
    double GetThrottle() const { return throttle; }

    /// Tell if the engine is rotating backward.
    /// If so, it's an erroneous situation
    bool IsRotatingBackward() { return error_backward; }

    /// This is the function that actually contains the
    /// formula for computing T=T(w,throttle)
    virtual double ComputeTorque() override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

CH_CLASS_VERSION(ChShaftsThermalEngine,0)

}  // end namespace chrono

#endif
