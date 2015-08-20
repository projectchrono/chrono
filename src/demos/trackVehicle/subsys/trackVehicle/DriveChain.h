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
// Authors: Justin Madsen
// =============================================================================
//
// Use some track vehicle components to model a drive gear, attached to a powertrain
// via a driveline. Wrap a chain around the drive gear and idler, to which an
// inertial load will be applied.
// It would be easy to turn this into a motorcycle dynamometer.
//
// =============================================================================

#ifndef DRIVECHAIN_H
#define DRIVECHAIN_H

#include "core/ChCoordsys.h"
#include "physics/ChSystem.h"
#include "ModelDefs.h"

#include "subsys/ChTrackVehicle.h"
#include "subsys/trackSystem/TrackSystem.h"


namespace chrono {

/// Model a static DriveChain, with a powered Gear and
/// an idler attached to some large inertial load
class CH_SUBSYS_API DriveChain : public ChTrackVehicle {
  public:
    /// chassis is static, so use idler info to create the gear body, instead.
    DriveChain(const std::string& name,
               VisualizationType::Enum GearVis = VisualizationType::Primitives,
               CollisionType::Enum GearCollide = CollisionType::Primitives,
               double pin_damping_coef = 0,     ///< inter-shoe body revolute joint damping coef., [N-s/m]
               double tensioner_preload = 1e5,  ///< idler tensioner-spring preload [N]
               const ChVector<>& right_pos_rel = ChVector<>(0.23644, -0.4780, 0.83475),
               const ChVector<>& COG_to_REF = ChVector<>() );

    ~DriveChain();

    /// Initialize the tracked vehicle REF frame with the specified Coordinate system.
    /// This initial transform is inherited by all vehicle subsystems.
    /// Will add the collision geometry and add modeling elements to the ChSystem.
    virtual void Initialize(const ChCoordsys<>& chassis_Csys);  ///< initial config of vehicle REF frame

    /// Update the vehicle with the new settings for throttle and brake
    virtual void Update(double time,                          ///< [in] current time
        const std::vector<double>& throttle,  ///< [in] current steering input [-1,+1]
        const std::vector<double>& braking    ///< [in] current braking input [0,1]
        );

    /// Advance the vehicle (and the ChSystem)
    virtual void Advance(double step);

    /// set the pin friction as a damping value
    virtual void SetShoePinDamping(double damping);

    /*
    /// Log data to file.
    /// Data types AND filename to be saved should already set in Setup_log_to_file()
    virtual void Log_to_file();

    /// setup class to save the log to a file for python postprocessing.
    /// Usage: call after construction & Initialize(), else no data is saved.
    virtual void Setup_logger(int what_subsys,  /// which vehicle objects (e.g. subsystems) to save data for?
                              int debug_type,   /// data types: _BODY, _CONSTRAINTS, _CONTACTS
                              const std::string& out_filename,
                              const std::string& data_dirname = "data_test");
    */

    // ---------------------------------------------------------------------------
    // Accessors
    virtual double GetDriveshaftSpeed(size_t idx) const { return GetSprocketSpeed(0); }

    /// pointer to the powertrain
    virtual ChSharedPtr<TrackPowertrain> GetPowertrain(size_t idx) const { return m_ptrains[0]; }

    /// return the contant part of the damping (if any)
    virtual double GetShoePinDamping() const { return m_damping; }

    /// current value of the integration step size for the vehicle system.
    double GetStepsize() const { return m_stepsize; }

    /// number of track chain systems attached to the vehicle
    size_t GetNum_TrackSystems() const { return 1; }

    /// drive gear sprocket speed
    virtual double GetSprocketSpeed(const size_t idx) const {
        return m_TrackSystem->GetDriveGear()->GetBody()->GetWvel_loc().z;
    }

    // helper functions, for Irrlicht GUI
    // following variables are populated when DriveChain::reportShoeGearContact() is called
    // absolute pos of the persistent contact point
    virtual ChVector<> Get_SG_Persistent_PosAbs(int track, int idx) const {
        return m_TrackSystem->GetTrackChain()->Get_SG_Persistent_PosAbs(idx);
    }

    // normal force abs. vector of the persistent contact point
    virtual ChVector<> Get_SG_Persistent_Fn(int track, int idx) const {
        return m_TrackSystem->GetTrackChain()->Get_SG_Persistent_Fn(idx);
    }

    // abs. pos. of all shoe-gear contacts found
    virtual const std::vector<ChVector<> >& Get_SG_PosAbs_all(int track) const {
        return m_TrackSystem->GetTrackChain()->Get_SG_PosAbs_all();
    }

    // abs. normal force of all sh oe-gear contacts
    virtual const std::vector<ChVector<> >& Get_SG_Fn_all(int track) const {
        return m_TrackSystem->GetTrackChain()->Get_SG_Fn_all();
    }

  protected:
    /// create files with headers for all specified output data types.
    /// File format is .csv, for easy reading into python pandas scripts for data analysis
    // void create_fileHeaders();

    // private variables
    const ChVector<> m_TrackSystem_loc;  ///< location of tracksystem, relative to chassis REF c-sys
    ChSharedPtr<TrackSystem> m_TrackSystem;  ///< handles to track system

    double m_damping;   ///< damping coef. applied between shoe bodies, to the rev. constraint
    const double m_tensioner_preload;

    // filenames for output data
    std::string m_filename_DBG_CHASSIS;  // vehicle data (chassis motion, driver inputs)
    std::string m_filename_DBG_PTRAIN;   // write powertrain data
    std::string m_filename_DBG_ALL_CONTACTS;

    size_t m_cnt_Log_to_file;  // how many times was Log_to_file called?

    // static variables
    static const ChCoordsys<> m_driverCsys;  // driver position and orientation relative to chassis

    // placeholder
    ChSharedPtr<ChShaft> m_axle;  // dummy shaft

    friend std::ostream& operator<<(std::ostream& out, const ChVector<double>& vect);
    friend std::ostream& operator<<(std::ostream& out, const ChQuaternion<double>& q);
};

}  // end namespace chrono

#endif
