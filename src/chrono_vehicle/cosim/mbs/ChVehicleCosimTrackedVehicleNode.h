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

#include "chrono_vehicle/ChPowertrain.h"
#include "chrono_vehicle/ChTerrain.h"
#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/ChTire.h"

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
    ChVehicleCosimTrackedVehicleNode(const std::string& vehicle_json,    ///< vehicle JSON specification file
                                     const std::string& powertrain_json  ///< powertrain JSON specification file
    );

    /// Construct a tracked vehicle node using the provided vehicle and powertrain objects.
    /// Notes:
    /// - the provided vehicle system must be constructed with a null Chrono system.
    /// - the vehicle and powertrain system should not be initialized.
    ChVehicleCosimTrackedVehicleNode(std::shared_ptr<ChTrackedVehicle> vehicle,  ///< vehicle system
                                     std::shared_ptr<ChPowertrain> powertrain    ///< powertrain system
    );

    ~ChVehicleCosimTrackedVehicleNode();

    /// Get the underlying vehicle system.
    std::shared_ptr<ChTrackedVehicle> GetVehicle() const { return m_vehicle; }

    /// Get the underlying powertrain system.
    std::shared_ptr<ChPowertrain> GetPowertrain() const { return m_powertrain; }

    /// Set the initial vehicle position, relative to the center of the terrain top-surface.
    void SetInitialLocation(const ChVector<>& init_loc) { m_init_loc = init_loc; }

    /// Set the initial vehicle yaw angle (in radians).
    void SetInitialYaw(double init_yaw) { m_init_yaw = init_yaw; }

    /// Attach a vehicle driver system.
    void SetDriver(std::shared_ptr<ChDriver> driver) { m_driver = driver; }

  private:
    /// Initialize the vehicle MBS and any associated subsystems.
    virtual void InitializeMBS(const ChVector2<>& terrain_size,  ///< terrain length x width
                               double terrain_height             ///< initial terrain height
                               ) override;

    // Output vehicle data.
    virtual void OnOutputData(int frame) override;

    /// Perform vehicle system synchronization before advancing the dynamics.
    virtual void PreAdvance() override;

    /// Process the provided track shoe force (received from the corresponding track node).
    virtual void ApplyTrackShoeForce(int track_id, int shoe_id, const TerrainForce& force) override;

    /// Return the number of tracks in the vehicle system.
    virtual int GetNumTracks() const override;

    /// Return the number of track shoes in the specified track subsystem.
    virtual int GetNumTrackShoes(int track_id) const override;

    /// Return the specified track shoe.
    virtual std::shared_ptr<ChBody> GetTrackShoeBody(int track_id, int shoe_id) const override;

    /// Return the vertical mass load on the i-th spindle.
    virtual double GetSpindleLoad(unsigned int i) const override;

    /// Get the body state of the specified track shoe body.
    virtual BodyState GetTrackShoeState(int track_id, int shoe_id) const override;

    /// Get the "chassis" body.
    virtual std::shared_ptr<ChBody> GetChassisBody() const override;

    /// Impose spindle angular speed as dictated by an attached DBP rig.
    virtual void OnInitializeDBPRig(std::shared_ptr<ChFunction> func) override;

    void WriteBodyInformation(utils::CSV_writer& csv);

  private:
    /// ChTire subsystem, needed to pass the terrain contact forces back to the vehicle wheels.
    class DummyTire : public ChTire {
      public:
        DummyTire(int index, double mass, double radius, double width)
            : ChTire("dummy_tire"), m_index(index), m_mass(mass), m_radius(radius), m_width(width) {}
        virtual std::string GetTemplateName() const override { return "dummy_tire"; }
        virtual double GetRadius() const override { return m_radius; }
        virtual double GetWidth() const override { return m_width; }
        virtual double GetAddedMass() const override { return m_mass; }
        virtual ChVector<> GetAddedInertia() const override { return ChVector<>(0.1, 0.1, 0.1); }
        virtual TerrainForce ReportTireForce(ChTerrain* terrain) const override { return m_force; }
        virtual TerrainForce GetTireForce() const { return m_force; }
        virtual void InitializeInertiaProperties() override {}
        virtual void UpdateInertiaProperties() override {}

        int m_index;
        double m_mass;
        double m_radius;
        double m_width;
        TerrainForce m_force;
    };

    std::shared_ptr<ChTrackedVehicle> m_vehicle;  ///< vehicle MBS
    std::shared_ptr<ChPowertrain> m_powertrain;   ///< vehicle powertrain
    std::shared_ptr<ChDriver> m_driver;           ///< vehicle driver

    ChVector<> m_init_loc;  ///< initial vehicle location (relative to center of terrain top surface)
    double m_init_yaw;      ///< initial vehicle yaw

    TerrainForces m_shoe_forces[2];  ///< terrain forces acting on track shoes

    //// RADU TODO
    std::vector<double> m_spindle_loads;  ///< vertical loads on each spindle
};

/// @} vehicle_cosim_mbs

}  // end namespace vehicle
}  // end namespace chrono

#endif
