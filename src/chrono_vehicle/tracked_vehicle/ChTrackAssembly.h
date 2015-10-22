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

#include "chrono/core/ChShared.h"
#include "chrono/physics/ChBodyAuxRef.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"

#include "chrono_vehicle/tracked_vehicle/ChSprocket.h"
#include "chrono_vehicle/tracked_vehicle/ChIdler.h"
#include "chrono_vehicle/tracked_vehicle/ChRoadWheelAssembly.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackShoe.h"

namespace chrono {
namespace vehicle {

///
///
///
class CH_VEHICLE_API ChTrackAssembly : public ChShared {
  public:
    ChTrackAssembly(const std::string& name  ///< [in] name of the subsystem
                    )
        : m_name(name) {}

    virtual ~ChTrackAssembly() {}

    /// Get the name identifier for this track assembly subsystem.
    const std::string& GetName() const { return m_name; }

    /// Set the name identifier for this track assembly subsystem.
    void SetName(const std::string& name) { m_name = name; }

    /// Construct a track assembly by mirroring the specified one.
    virtual ChSharedPtr<ChTrackAssembly> Mirror(ChSharedPtr<ChTrackAssembly> source) = 0;

    /// Get a handle to the speicifed track shoe.
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

    /// Assemble track shoes over wheels.
    void Assemble();

    /// Initialize this track assembly subsystem.
    /// The subsystem is initialized by attaching it to the specified chassis body
    /// at the specified location (with respect to and expressed in the reference
    /// frame of the chassis) which represents the location of the sprocket. It is
    /// assumed that the track assembly reference frame is always aligned with the
    /// chassis reference frame.
    virtual void Initialize(ChSharedPtr<ChBodyAuxRef> chassis,  ///< [in] handle to the chassis body
                            const ChVector<>& location          ///< [in] location of sprocket relative to the chassis frame
                            ) = 0;

    /// Update the state of this track assembly at the current time.
    void Update(double time,                        ///< [in] current time
                const TrackShoeForces& shoe_forces  ///< [in] vector of tire force structures
                );

  protected:
    std::string m_name;  ///< name of the subsystem

    ChSharedPtr<ChSprocket> m_sprocket;
    ChSharedPtr<ChIdler> m_idler;
    ChRoadWheelAssemblyList m_suspensions;
    ChTrackShoeList m_shoes;
};

}  // end namespace vehicle
}  // end namespace chrono

#endif
