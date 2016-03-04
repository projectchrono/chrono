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
//
// Definition of a track testing mechanism (as a vehicle).
// The tested track can be specified through a stand-alone JSON file or as a
// specified track assembly in a vehicle JSON specification file.
//
// The reference frame follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#ifndef CH_TRACK_TEST_RIG_H
#define CH_TRACK_TEST_RIG_H

#include <string>
#include <vector>

#include "chrono/assets/ChColor.h"

#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"

/**
    @addtogroup vehicle_tracked
    @{
        @defgroup vehicle_tracked_utils Utility classes
    @}
*/

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_utils
/// @{

/// Definition of a suspension test rig.
class CH_VEHICLE_API ChTrackTestRig : public ChVehicle {
  public:
    /// Construct a test rig for a specified track assembly of a given vehicle.
    ChTrackTestRig(
        const std::string& filename,  ///< [in] JSON file with vehicle specification
        VehicleSide side,             ///< [in] select left or right track assembly
        ChMaterialSurfaceBase::ContactMethod contact_method = ChMaterialSurfaceBase::DVI  ///< [in] contact method
        );

    /// Construct a test rig from specified file.
    ChTrackTestRig(
        const std::string& filename,  ///< [in] JSON file with test rig specification
        ChMaterialSurfaceBase::ContactMethod contact_method = ChMaterialSurfaceBase::DVI  ///< [in] contact method
        );

    /// Construct a test rig using the specified track assembly and subsystem locations.
    ChTrackTestRig(
        std::shared_ptr<ChTrackAssembly> assembly,     ///< [in] handle to the track assembly
        const ChVector<>& sprocketLoc,                 ///< [in] sprocket location
        const ChVector<>& idlerLoc,                    ///< [in] idler location
        const std::vector<ChVector<> >& suspLocs,      ///< [in] suspension locations
        ChMaterialSurfaceBase::ContactMethod contact_method = ChMaterialSurfaceBase::DVI  ///< [in] contact method
        );

    /// Destructor
    ~ChTrackTestRig() {}

    /// Set the actuator function
    void SetActuator_func(const std::shared_ptr<ChFunction>& func) { m_actuator = func; }

    double GetActuatorDisp();
    double GetActuatorForce();
    double GetActuatorMarkerDist();

    /// Get the rig total mass.
    /// This is simply the mass of the track subsystem.
    virtual double GetVehicleMass() const override;

    /// Return the location of the shaker post.
    const ChVector<>& GetPostPosition() const { return m_post_pos; }

    /// Get the local driver position and orientation.
    /// This is a coordinate system relative to the chassis reference frame.
    virtual ChCoordsys<> GetLocalDriverCoordsys() const override { return ChCoordsys<>(); }

    /// Get a handle to the vehicle's driveshaft body.
    virtual std::shared_ptr<ChShaft> GetDriveshaft() const override { return m_dummy_shaft; }

    /// Get the angular speed of the driveshaft.
    virtual double GetDriveshaftSpeed() const override { return 0; }

    /// Initialize this chassis at the specified global location and orientation.
    virtual void Initialize(const ChCoordsys<>& chassisPos  ///< [in] initial global position and orientation
                            ) override;

    /// Set collision flags for the various subsystems.
    /// By default, collision is enabled for sprocket, idler, road wheels, and
    /// track shoes. To override these default settings, this function must be
    /// called after the call to Initialize().
    void SetCollide(int flags);

    /// Update the state at the current time.
    /// steering between -1 and +1, and no force need be applied if using external actuation
    void Synchronize(double time,                        ///< [in] current time
                     double disp,                        ///< [in] post displacement
                     const TrackShoeForces& shoe_forces  ///< [in] vector of track shoe forces
                     );

    /// Log current constraint violations.
    virtual void LogConstraintViolations() override;

  private:
    static void AddVisualize_post(std::shared_ptr<ChBody> post_body,
                                  std::shared_ptr<ChBody> chassis_body,
                                  double length,
                                  double width,
                                  double height,
                                  const ChColor& color);

    std::shared_ptr<ChTrackAssembly> m_track;  ///< handle to the track assembly

    std::shared_ptr<ChShaft> m_dummy_shaft;  ///< dummy driveshaft

    ChVector<> m_post_pos;                                  ///< location of the shaker post
    std::shared_ptr<ChBody> m_post;                         ///< shaker post
    std::shared_ptr<ChLinkLockPrismatic> m_post_prismatic;  ///< post prismatic joint
    std::shared_ptr<ChLinkLinActuator> m_post_linact;       ///< actuate post
    std::shared_ptr<ChLinkLockPointPlane> m_post_ptPlane;   ///< actuate track to a specified height

    std::shared_ptr<ChFunction> m_actuator;  ///< actuator function applied to post

    double m_displ;  ///< cached left post displacement

    ChVector<> m_sprocketLoc;
    ChVector<> m_idlerLoc;
    std::vector<ChVector<> > m_suspLocs;
};

/// @} vehicle_tracked_utils

}  // end namespace vehicle
}  // end namespace chrono

#endif
