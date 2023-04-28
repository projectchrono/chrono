// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
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
// Tracked vehicle system co-simulated with track nodes and a terrain node.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#ifndef CH_VEHCOSIM_TRACKED_VEHICLE_NODE_H
#define CH_VEHCOSIM_TRACKED_VEHICLE_NODE_H

#include "chrono_vehicle/ChPowertrainAssembly.h"
#include "chrono_vehicle/ChTerrain.h"
#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicle.h"

#include "chrono_vehicle/cosim/ChVehicleCosimTrackedMBSNode.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_cosim_mbs
/// @{

/// Tracked vehicle co-simulation node.
/// The vehicle system is co-simulated with track nodes and a terrain node.
class CH_VEHICLE_API ChVehicleCosimTrackedVehicleNode : public ChVehicleCosimTrackedMBSNode {
  public:
    /// Construct a tracked vehicle node using the provided vehicle and powertrain JSON specification files.
    ChVehicleCosimTrackedVehicleNode(const std::string& vehicle_json,      ///< vehicle JSON specification file
                                     const std::string& engine_json,       ///< engine JSON specification file
                                     const std::string& transmission_json  ///< transmission JSON specification file
    );

    /// Construct a tracked vehicle node using the provided vehicle and powertrain objects.
    /// Notes:
    /// - the provided vehicle system must be constructed with a null Chrono system.
    /// - the vehicle and powertrain system should not be initialized.
    ChVehicleCosimTrackedVehicleNode(std::shared_ptr<ChTrackedVehicle> vehicle,        ///< vehicle system
                                     std::shared_ptr<ChPowertrainAssembly> powertrain  ///< powertrain system
    );

    ~ChVehicleCosimTrackedVehicleNode();

    /// Get the underlying vehicle system.
    std::shared_ptr<ChTrackedVehicle> GetVehicle() const { return m_vehicle; }

    /// Set the initial vehicle position, relative to the center of the terrain top-surface.
    void SetInitialLocation(const ChVector<>& init_loc) { m_init_loc = init_loc; }

    /// Set the initial vehicle yaw angle (in radians).
    void SetInitialYaw(double init_yaw) { m_init_yaw = init_yaw; }

    /// Attach a vehicle driver system.
    void SetDriver(std::shared_ptr<ChDriver> driver) { m_driver = driver; }

    /// Fix vehicle chassis to ground.
    void SetChassisFixed(bool val) { m_chassis_fixed = val; }

  private:
    /// Initialize the vehicle MBS and any associated subsystems.
    virtual void InitializeMBS(const ChVector2<>& terrain_size,  ///< terrain length x width
                               double terrain_height             ///< initial terrain height
                               ) override;

    /// Return terrain contact geometry and material information for one track shoe.
    virtual ChVehicleGeometry GetTrackShoeContactGeometry() const override;

    /// Return mass of one track shoe.
    virtual double GetTrackShoeMass() const override;

    /// Output vehicle data.
    virtual void OnOutputData(int frame) override;

    /// Perform vehicle system synchronization before advancing the dynamics.
    virtual void PreAdvance() override;

    /// Process the provided track shoe force (received from the corresponding track node).
    virtual void ApplyTrackShoeForce(int track_id, int shoe_id, const TerrainForce& force) override;

    /// Return the number of tracks in the vehicle system.
    virtual int GetNumTracks() const override;

    /// Return the number of track shoes in the specified track subsystem.
    virtual size_t GetNumTrackShoes(int track_id) const override;

    /// Return the total number of track shoes (in all track subsystems).
    virtual size_t GetNumTrackShoes() const override;

    /// Return the specified track shoe.
    virtual std::shared_ptr<ChBody> GetTrackShoeBody(int track_id, int shoe_id) const override;

    /// Get the body state of the specified track shoe body.
    virtual BodyState GetTrackShoeState(int track_id, int shoe_id) const override;

    /// Get the "chassis" body.
    virtual std::shared_ptr<ChBody> GetChassisBody() const override;

    /// Get the sprocket addendum radius (for slip calculation).
    virtual double GetSprocketAddendumRadius() const override;

    /// Impose spindle angular speed as dictated by an attached DBP rig.
    virtual void OnInitializeDBPRig(std::shared_ptr<ChFunction> func) override;

    void WriteBodyInformation(utils::CSV_writer& csv);

  private:
    std::shared_ptr<ChTrackedVehicle> m_vehicle;         ///< vehicle MBS
    std::shared_ptr<ChPowertrainAssembly> m_powertrain;  ///< vehicle powertrain
    std::shared_ptr<ChDriver> m_driver;                  ///< vehicle driver

    ChVector<> m_init_loc;  ///< initial vehicle location (relative to center of terrain top surface)
    double m_init_yaw;      ///< initial vehicle yaw
    bool m_chassis_fixed;   ///< fix chassis to ground

    TerrainForces m_shoe_forces[2];  ///< terrain forces acting on track shoes
};

/// @} vehicle_cosim_mbs

}  // end namespace vehicle
}  // end namespace chrono

#endif
