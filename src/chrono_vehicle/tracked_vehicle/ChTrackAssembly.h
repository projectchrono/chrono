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
// Base class for a track assembly which consists of one sprocket, one idler,
// a collection of road wheel assemblies (suspensions), and a collection of
// track shoes.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#ifndef CH_TRACK_ASSEMBLY_H
#define CH_TRACK_ASSEMBLY_H

#include <vector>

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyAuxRef.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"

#include "chrono_vehicle/tracked_vehicle/ChSprocket.h"
#include "chrono_vehicle/tracked_vehicle/ChIdler.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackBrake.h"
#include "chrono_vehicle/tracked_vehicle/ChRoadWheelAssembly.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackShoe.h"

namespace chrono {
namespace vehicle {

///
///
///
class CH_VEHICLE_API ChTrackAssembly {
  public:
    ChTrackAssembly(const std::string& name,  ///< [in] name of the subsystem
                    VehicleSide side          ///< [in] assembly on left/right vehicle side
                    )
        : m_name(name), m_side(side) {}

    ~ChTrackAssembly() {}

    /// Get the name identifier for this track assembly subsystem.
    const std::string& GetName() const { return m_name; }

    /// Set the name identifier for this track assembly subsystem.
    void SetName(const std::string& name) { m_name = name; }

    /// Return the vehicle side for this track assembly.
    VehicleSide GetVehicleSide() const { return m_side; }

    /// Get the number of suspensions.
    size_t GetNumRoadWheelAssemblies() const { return m_suspensions.size(); }

    /// Get the number of track shoes.
    size_t GetNumTrackShoes() const { return m_shoes.size(); }

    /// Get a handle to the sprocket.
    ChSharedPtr<ChSprocket> GetSprocket() const { return m_sprocket; }

    /// Get a handle to the idler subsystem.
    ChSharedPtr<ChIdler> GetIdler() const { return m_idler; }

    /// Get a handle to the brake subsystem.
    ChSharedPtr<ChTrackBrake> GetBrake() const { return m_brake; }

    /// Get a handle to the specified suspension subsystem.
    ChSharedPtr<ChRoadWheelAssembly> GetRoadWheelAssembly(size_t id) const { return m_suspensions[id]; }

    /// Get a handle to the specified road wheel subsystem.
    ChSharedPtr<ChRoadWheel> GetRoadWheel(size_t id) const { return m_suspensions[id]->GetRoadWheel(); }

    /// Get a handle to the specified track shoe subsystem.
    ChSharedPtr<ChTrackShoe> GetTrackShoe(size_t id) const { return m_shoes[id]; }

    /// Get the global location of the specified track shoe.
    const ChVector<>& GetTrackShoePos(size_t id) const { return m_shoes[id]->m_shoe->GetPos(); }

    /// Get the orientation of the the specified track shoe.
    /// The track shoe body orientation is returned as a quaternion representing a
    /// rotation with respect to the global reference frame.
    const ChQuaternion<>& GetTrackShoeRot(size_t id) const { return m_shoes[id]->m_shoe->GetRot(); }

    /// Get the linear velocity of the specified track shoe.
    /// Return the linear velocity of the shoe center, expressed in the global
    /// reference frame.
    const ChVector<>& GetTrackShoeLinVel(size_t id) const { return m_shoes[id]->m_shoe->GetPos_dt(); }

    /// Get the angular velocity of the the specified track shoe.
    /// Return the angular velocity of the shoe frame, expressed in the global
    /// reference frame.
    ChVector<> GetTrackShoeAngVel(size_t id) const { return m_shoes[id]->m_shoe->GetWvel_par(); }

    /// Get the complete state for the specified track shoe.
    /// This includes the location, orientation, linear and angular velocities,
    /// all expressed in the global reference frame.
    BodyState GetTrackShoeState(size_t id) const;

    /// Initialize this track assembly subsystem.
    /// The subsystem is initialized by attaching its constituent subsystems to the
    /// specified chassis body at the specified corresponding locations (with respect
    /// to and expressed in the reference frame of the chassis).  All subsystem reference
    /// frames are assumed to be aligned with the chassis reference frame.
    void Initialize(
        ChSharedPtr<ChBodyAuxRef> chassis,               ///< [in] handle to the chassis body
        const ChVector<>& sprocket_loc,                  ///< [in] sprocket location relative to the chassis frame
        const ChVector<>& idler_loc,                     ///< [in] idler location relative to the chassis frame
        const std::vector<ChVector<> >& suspension_locs  ///< [in] suspension locations relative to the chassis frame
        );

    /// Update the state of this track assembly at the current time.
    void Update(double time,                        ///< [in] current time
                double braking,                     ///< [in] braking driver input
                const TrackShoeForces& shoe_forces  ///< [in] vector of tire force structures
                );

    /// Log current constraint violations.
    void LogConstraintViolations();

  protected:
    std::string m_name;                     ///< name of the subsystem
    VehicleSide m_side;                     ///< assembly on left/right vehicle side
    ChSharedPtr<ChSprocket> m_sprocket;     ///< sprocket subsystem
    ChSharedPtr<ChIdler> m_idler;           ///< idler (and tensioner) subsystem
    ChSharedPtr<ChTrackBrake> m_brake;      ///< sprocket brake
    ChRoadWheelAssemblyList m_suspensions;  ///< road-wheel assemblies
    ChTrackShoeList m_shoes;                ///< track shoes

  private:
    /// Assemble track shoes over wheels.
    /// Return true if the track shoes were initialized in a counter clockwise
    /// direction and false otherwise.
    bool Assemble(ChSharedPtr<ChBodyAuxRef> chassis);
};

}  // end namespace vehicle
}  // end namespace chrono

#endif
