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
// Authors: Radu Serban
// =============================================================================
//
// Base class for a track assembly which consists of one sprocket, one idler,
// a collection of road wheel assemblies (suspensions), a collection of rollers,
// and a collection of track shoes.
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
#include "chrono_vehicle/ChPart.h"

#include "chrono_vehicle/tracked_vehicle/ChSprocket.h"
#include "chrono_vehicle/tracked_vehicle/ChIdler.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackBrake.h"
#include "chrono_vehicle/tracked_vehicle/ChRoadWheelAssembly.h"
#include "chrono_vehicle/tracked_vehicle/ChRoller.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackShoe.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked
/// @{

/// Definition of a track assembly.
/// A track assembly consists of a sprocket, an idler (with tensioner mechanism), a set of
/// rollers, a set of suspensions (road-wheel assemblies), and a collection of track shoes.
class CH_VEHICLE_API ChTrackAssembly : public ChPart {
  public:
    virtual ~ChTrackAssembly() {}

    /// Return the vehicle side for this track assembly.
    VehicleSide GetVehicleSide() const { return m_side; }

    /// Get the number of suspensions.
    size_t GetNumRoadWheelAssemblies() const { return m_suspensions.size(); }

    /// Get the number of rollers.
    size_t GetNumRollers() const { return m_rollers.size(); }

    /// Get the number of track shoes.
    virtual size_t GetNumTrackShoes() const = 0;

    /// Get a handle to the sprocket.
    virtual std::shared_ptr<ChSprocket> GetSprocket() const = 0;

    /// Get a handle to the idler subsystem.
    std::shared_ptr<ChIdler> GetIdler() const { return m_idler; }

    /// Get a handle to the brake subsystem.
    std::shared_ptr<ChTrackBrake> GetBrake() const { return m_brake; }

    /// Get a handle to the specified suspension subsystem.
    std::shared_ptr<ChRoadWheelAssembly> GetRoadWheelAssembly(size_t id) const { return m_suspensions[id]; }

    /// Get a handle to the specified roller subsystem.
    std::shared_ptr<ChRoller> GetRoller(size_t id) const { return m_rollers[id]; }

    /// Get a handle to the specified road wheel subsystem.
    std::shared_ptr<ChRoadWheel> GetRoadWheel(size_t id) const { return m_suspensions[id]->GetRoadWheel(); }

    /// Get a handle to the specified track shoe subsystem.
    virtual std::shared_ptr<ChTrackShoe> GetTrackShoe(size_t id) const = 0;

    /// Get the global location of the specified track shoe.
    /// The returned location is that of the shoe body in the track shoe subsystem.
    const ChVector<>& GetTrackShoePos(size_t id) const { return GetTrackShoe(id)->m_shoe->GetPos(); }

    /// Get the orientation of the specified track shoe.
    /// The track shoe body orientation is returned as a quaternion representing a
    /// rotation with respect to the global reference frame. This is the orientation of
    /// the shoe body in the track shoe subsystem.
    const ChQuaternion<>& GetTrackShoeRot(size_t id) const { return GetTrackShoe(id)->m_shoe->GetRot(); }

    /// Get the linear velocity of the specified track shoe.
    /// Return the linear velocity of the shoe body center, expressed in the global
    /// reference frame.
    const ChVector<>& GetTrackShoeLinVel(size_t id) const { return GetTrackShoe(id)->m_shoe->GetPos_dt(); }

    /// Get the angular velocity of the specified track shoe.
    /// Return the angular velocity of the shoe body frame, expressed in the global
    /// reference frame.
    ChVector<> GetTrackShoeAngVel(size_t id) const { return GetTrackShoe(id)->m_shoe->GetWvel_par(); }

    /// Get the complete state for the specified track shoe.
    /// This includes the location, orientation, linear and angular velocities,
    /// all expressed in the global reference frame.
    BodyState GetTrackShoeState(size_t id) const;

    /// Get the complete states for all track shoes.
    /// These include the locations, orientations, linear and angular velocities for
    /// all track shoes in this track assembly, all expressed in the global reference frame.
    /// It is assumed that the vector of body states was properly sized.
    void GetTrackShoeStates(BodyStates& states) const;

    /// Get the total mass of the track assembly.
    /// This includes the masses of the sprocket, idler, suspensions, and track shoes.
    double GetMass() const;

    /// Get the relative location of the sprocket subsystem.
    /// The track assembly reference frame is ISO, with origin at the sprocket center.
    virtual const ChVector<> GetSprocketLocation() const = 0;

    /// Get the relative location of the idler subsystem.
    /// The track assembly reference frame is ISO, with origin at the sprocket center.
    virtual const ChVector<> GetIdlerLocation() const = 0;

    /// Get the relative location of the specified suspension subsystem.
    /// The track assembly reference frame is ISO, with origin at the sprocket center.
    virtual const ChVector<> GetRoadWhelAssemblyLocation(int which) const = 0;

    /// Get the relative location of the specified roller subsystem.
    /// The track assembly reference frame is ISO, with origin at the sprocket center.
    virtual const ChVector<> GetRollerLocation(int which) const { return ChVector<>(0, 0, 0); }

    /// Initialize this track assembly subsystem.
    /// The subsystem is initialized by attaching it to the specified chassis body
    /// at the specified location (with respect to and expressed in the reference
    /// frame of the chassis). It is assumed that the track assembly reference frame
    /// is always aligned with the chassis reference frame.
    void Initialize(std::shared_ptr<ChBodyAuxRef> chassis,  ///< [in] handle to the chassis body
                    const ChVector<>& location,             ///< [in] location relative to the chassis frame
                    bool create_shoes = true                ///< [in] control creation of the actual track
    );

    /// Set visualization type for the sprocket subsystem.
    void SetSprocketVisualizationType(VisualizationType vis);

    // Set visualization type for the idler subsystem.
    void SetIdlerVisualizationType(VisualizationType vis);

    /// Set visualization type for the suspension subsystems.
    void SetRoadWheelAssemblyVisualizationType(VisualizationType vis);

    /// Set visualization type for the road-wheel subsystems.
    void SetRoadWheelVisualizationType(VisualizationType vis);

    /// Set visualization type for the roller subsystems.
    void SetRollerVisualizationType(VisualizationType vis);

    /// Set visualization type for the track shoe subsystems.
    void SetTrackShoeVisualizationType(VisualizationType vis);

    /// Update the state of this track assembly at the current time.
    void Synchronize(double time,                      ///< [in] current time
                     double braking,                   ///< [in] braking driver input
                     const TerrainForces& shoe_forces  ///< [in] vector of tire force structures
    );

    /// Enable/disable output for this subsystem.
    /// This function overrides the output setting for all components of this track assembly.
    virtual void SetOutput(bool state) override;

    /// Log current constraint violations.
    void LogConstraintViolations();

  protected:
    ChTrackAssembly(const std::string& name,  ///< [in] name of the subsystem
                    VehicleSide side          ///< [in] assembly on left/right vehicle side
                    )
        : ChPart(name), m_side(side) {}

    /// Assemble track shoes over wheels.
    /// Return true if the track shoes were initialized in a counter clockwise
    /// direction and false otherwise.
    virtual bool Assemble(std::shared_ptr<ChBodyAuxRef> chassis) = 0;

    /// Remove all track shoes from assembly.
    virtual void RemoveTrackShoes() = 0;

    virtual void ExportComponentList(rapidjson::Document& jsonDocument) const override;

    virtual void Output(ChVehicleOutput& database) const override;

    VehicleSide m_side;                     ///< assembly on left/right vehicle side
    std::shared_ptr<ChIdler> m_idler;       ///< idler (and tensioner) subsystem
    std::shared_ptr<ChTrackBrake> m_brake;  ///< sprocket brake
    ChRoadWheelAssemblyList m_suspensions;  ///< road-wheel assemblies
    ChRollerList m_rollers;                 ///< roller subsystems

    friend class ChTrackedVehicle;
};

/// @} vehicle_tracked

}  // end namespace vehicle
}  // end namespace chrono

#endif
