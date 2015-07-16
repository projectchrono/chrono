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
// Base class for a Tracked vehicle system.
//
// Chassis ref frame is Y- up, X-forward.
//
// =============================================================================

#ifndef CH_TRACKVEHICLE_H
#define CH_TRACKVEHICLE_H

#include <vector>

#include "ModelDefs.h"

#include "physics/ChSystem.h"
#include "core/ChVector.h"
#include "physics/ChBodyAuxRef.h"

#include "subsys/powertrain/TrackPowertrain.h"
#include "subsys/ChApiSubsys.h"

// collision callback function
#include "subsys/collision/TrackCollisionCallback.h"
#include "physics/ChContactContainer.h"

namespace chrono {

///
/// Base class for chrono Track vehicle system.
/// This class provides the interface between the vehicle system and a driver
///
class CH_SUBSYS_API ChTrackVehicle : public ChShared {
  public:
    /// Construct by passing in desired system variables.
    /// Default values are for M113 APC vehicle
    ChTrackVehicle(const std::string& name,
                   VisualizationType::Enum vis = VisualizationType::None,
                   CollisionType::Enum collide = CollisionType::None,
                   double mass = 5489.2,
                   const ChVector<>& Ixx = ChVector<>(1786.9, 10449.7, 10721.2),
                   size_t num_engines = 0,
                   double step_size = 1e-3);

    /// Construct using the specified ChSystem, with the desired vehicle system settings.
    ChTrackVehicle(ChSystem* system,
                   const std::string& name,
                   VisualizationType::Enum vis = VisualizationType::None,
                   CollisionType::Enum collide = CollisionType::None,
                   double mass = 5489.2,
                   const ChVector<>& Ixx = ChVector<>(1786.9, 10449.7, 10721.2),
                   size_t num_engines = 0,
                   GearPinCollisionCallback<ChContactContainerBase>* collision_callback = 0);

    /// Destructor.
    virtual ~ChTrackVehicle();

    // pure virtuals
    /// Get the angular speed of the driveshaft.
    virtual double GetDriveshaftSpeed(size_t idx) const = 0;

    /// pointer to the powertrain
    virtual const ChSharedPtr<TrackPowertrain> GetPowertrain(size_t idx) const = 0;

    /// Get the local driver position and orientation, relative to the chassis reference frame.
    virtual const ChCoordsys<> GetLocalDriverCoordsys() const { return ChCoordsys<>(); }

    // accessors
    /// Get a pointer to the Chrono ChSystem.
    ChSystem* GetSystem() const { return m_system; }

    /// Get a handle to the chassis body.
    ChSharedPtr<ChBodyAuxRef> GetChassis() const { return m_chassis; }

    /// Get the global location of the chassis reference frame origin.
    const ChVector<>& GetChassisPos() const { return m_chassis->GetFrame_REF_to_abs().GetPos(); }

    /// Get the orientation of the chassis reference frame.
    const ChQuaternion<>& GetChassisRot() const { return m_chassis->GetFrame_REF_to_abs().GetRot(); }

    /// Get the global location of the chassis center of mass.
    const ChVector<>& GetChassisPosCOM() const { return m_chassis->GetPos(); }

    /// Get the orientation of the chassis centroidal frame.
    const ChQuaternion<>& GetChassisRotCOM() const { return m_chassis->GetRot(); }

    /// Get the vehicle speed, measured at the origin of the chassis reference frame.
    double GetVehicleSpeed() const { return m_chassis->GetFrame_REF_to_abs().GetPos_dt().Length(); }

    /// Get the speed of the chassis COM.
    double GetVehicleSpeedCOM() const { return m_chassis->GetPos_dt().Length(); }

    /// Get the global location of the driver.
    ChVector<> GetDriverPos() const;

    /// number of track chain systems attached to the vehicle
    size_t GetNum_Engines() const { return m_num_engines; }

    /// Initialize at the specified global location and orientation.
    virtual void Initialize(const ChCoordsys<>& chassis_Csys  ///< [in] initial config of vehicle REF frame
                            ) {}

    /// Update the state at the current time, driver inputs between 0 and 1.
    virtual void Update(double time,                          ///< [in] current time
                        const std::vector<double>& throttle,  ///< [in] current steering input [-1,+1]
                        const std::vector<double>& braking    ///< [in] current braking input [0,1]
                        ) {}

    /// Advance the system by the specified time step.
    virtual void Advance(double step);

    /// Set the integration step size
    void SetStepsize(double val) { m_stepsize = val; }

    /// can set the pin friction as a damping value
    virtual void SetShoePinDamping(double damping) {}

    /// add the set of gear and shoe bodies on a chain to be included in the custom collision callback
    void AddGearPinCollisionCallback(const std::vector<ChSharedPtr<ChBody> >& m_shoes,
                                     ChSharedPtr<ChBody> gear,
                                     ChSharedPtr<GearPinGeometry> geom);

    // Accessors
    /// Get the current value of the integration step size
    double GetStepsize() const { return m_stepsize; }

    /// return the contant part of the damping (if any)
    virtual double GetShoePinDamping() const { return 0; }

    virtual const GearPinCollisionCallback<ChContactContainerBase>* GetCollisionCallback() const {
        return m_gearPin_CollisionCallback;
    }

    // Log data, constraint violations, etc. to console (ChLog), or a file (

    /// log the constraint violations of this and all child subsystems to console
    virtual void LogConstraintViolations(bool include_chain = false) {}

    /// save the constraint violations, w/ and w/o chain body, to the
    virtual void SaveConstraintViolations(bool include_chain = false) {}

    virtual void Log_to_console(int console_what) {}

    /// Log data to file.
    /// Data types AND filename to be saved should already set in Setup_log_to_file()
    virtual void Log_to_file() {}

    /// setup class to save the log to a file for python postprocessing.
    /// Usage: call after construction & Initialize(), else no data is saved.
    virtual void Setup_log_to_file(int what,
                                   const std::string& out_filename,
                                   const std::string& data_dirname = "data_test") {}

  protected:
    // private functions
    virtual void AddVisualization();
    virtual void AddCollisionGeometry(double mu = 0.7,
                                      double mu_sliding = 0.6,
                                      double mu_roll = 0.0,
                                      double mu_spin = 0.0);

    const std::string& getMeshName() const { return m_meshName; }
    const std::string& getMeshFile() const { return m_meshFile; }

    ChSystem* m_system;       ///< pointer to the Chrono system
    const bool m_ownsSystem;  ///< true if system created at construction
    double m_stepsize;        ///< integration step-size for the vehicle system

    ChSharedPtr<ChBodyAuxRef> m_chassis;  ///< handle to the chassis body

    VisualizationType::Enum m_vis;  ///< visualize  geometry
    CollisionType::Enum m_collide;  ///< collision geometry

    std::string m_meshName;       ///< name of the mesh, if any
    std::string m_meshFile;       ///< filename of the mesh, if any
    ChVector<> m_chassisBoxSize;  ///< size of chassis box, used for any PRIMITIVES type

    const size_t m_num_engines;                            ///< can support multiple powertrain/drivetrains
    std::vector<ChSharedPtr<TrackPowertrain> > m_ptrains;  ///< powertrain system, one per track system

    // Custom collision class for gear and shoe pin geometry with non-convex analytically defined shapes
    GearPinCollisionCallback<ChContactContainerBase>* m_gearPin_CollisionCallback;

    // output/Log variables
    bool m_save_log_to_file;    ///< save the DebugLog() info to file? default false
    int m_log_what_to_file;     ///< set this in Setup_log_to_file(), if writing to file
    bool m_log_file_exists;     ///< written the headers for log file yet?
    int m_log_what_to_console;  ///< pre-set what to write to console when calling
    std::string m_log_file_name;
};

}  // end namespace chrono

#endif
