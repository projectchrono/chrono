// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2015 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Justin Madsen
// =============================================================================
//
// As the name implies, a loop of chains around lots of rolling elements.
//
// =============================================================================

#ifndef LOOPCHAIN_H
#define LOOPCHAIN_H

#include "core/ChCoordsys.h"
// #include "physics/ChSystem.h"
// #include "ModelDefs.h"

#include "subsys/base/ChTrackVehicle.h"
#include "subsys/driveGear/DriveGear.h"
#include "subsys/idler/IdlerSimple.h"
#include "subsys/trackChain/TrackChain.h"
#include "subsys/powertrain/TrackPowertrain.h"
#include "subsys/idler/SupportRoller.h"

namespace chrono {

/// Model a static LoopChain, with a powered Gear and
/// an idler attached to some large inertial load
class CH_SUBSYS_API LoopChain : public ChTrackVehicle {
  public:
    LoopChain(const std::string& name,
              VisualizationType::Enum chassisVis = VisualizationType::Primitives,
              CollisionType::Enum chassisCollide = CollisionType::Primitives,
              double gearMass = 100.0,
              const ChVector<>& gearIxx = ChVector<>(10, 10, 10),
              size_t num_idlers = 2,
              size_t num_rollers = 3);

    ~LoopChain();

    /// Initialize the drive chain system
    virtual void Initialize(const ChCoordsys<>& gear_Csys);  ///< [in] initial config of gear

    /// Update the vehicle with the new settings for throttle and brake
    virtual void Update(double time, double throttle, double braking);

    /// Advance the vehicle (and the ChSystem)
    virtual void Advance(double step);

    /// set the pin friction as a damping value
    virtual void SetShoePinDamping(double damping);

    /// log the constraint violations, w/ and w/o chain body, to either the console or a file
    virtual void LogConstraintViolations(bool include_chain = false);

    /// save the constraint violations, w/ and w/o chain body, to the
    virtual void SaveConstraintViolations(std::stringstream& ss, bool include_chain = false);

    virtual void Log_to_console(int console_what);

    /// Log data to file.
    /// Data types AND filename to be saved should already set in Setup_log_to_file()
    virtual void Log_to_file();

    /// setup class to save the log to a file for python postprocessing.
    /// Usage: call after construction & Initialize(), else no data is saved.
    virtual void Setup_log_to_file(int what, const std::string& out_filename);

    // ---------------------------------------------------------------------------
    // Accessors
    virtual double GetShoePinDamping() const { return m_chain->Get_pin_damping(); }

    /// Get the angular speed of the driveshaft.
    virtual double GetDriveshaftSpeed(size_t idx) const { return m_gear->GetAxle()->GetPos_dt(); }

    /// pointer to the powertrain
    virtual const ChSharedPtr<TrackPowertrain> GetPowertrain(size_t idx) const { return m_ptrain; }

    /// vehicle's driveshaft body.
    const ChSharedPtr<ChShaft> GetDriveshaft(size_t idx) const { return m_gear->GetAxle(); }

    /// current value of the integration step size for the vehicle system.
    double GetStepsize() const { return m_stepsize; }

    /// just return the COG of the gear
    const ChCoordsys<> GetLocalDriverCoordsys() const { return ChCoordsys<>(m_chassis->GetPos(), m_chassis->GetRot()); }

    /// vehicle speed, doesn't matter
    double GetVehicleSpeed() const { return 0; }
    int GetNum_Engines() const { return 1; }

  protected:
    // helper functions for output/Log
    virtual void create_fileHeader(int what);

    // private variables
    // <ChBodyAuxRef> m_chassis   in base class
    ChSharedPtr<DriveGear> m_gear;                    ///< drive gear
    std::vector<ChSharedPtr<IdlerSimple> > m_idlers;  ///< idler wheel
    size_t m_num_idlers;                              ///< number of idlers to create
    ChSharedPtr<TrackChain> m_chain;                  ///< chain

    ChVector<> m_idlerPosRel;  ///< position of idler COG relative to local c-sys

    ChSharedPtr<TrackPowertrain> m_ptrain;  ///< powertrain system

    std::vector<ChSharedPtr<SupportRoller> > m_rollers;  ///< passive support rollers
    size_t m_num_rollers;

    // static variables. hard-coded for now
    static const ChVector<> m_idlerPos;  // relative to chassis frame, which is the same as the gear's (initially)
    static const ChQuaternion<> m_idlerRot;

    friend std::ostream& operator<<(std::ostream& out, const ChVector<double>& vect);
    friend std::ostream& operator<<(std::ostream& out, const ChQuaternion<double>& q);
};

}  // end namespace chrono

#endif
