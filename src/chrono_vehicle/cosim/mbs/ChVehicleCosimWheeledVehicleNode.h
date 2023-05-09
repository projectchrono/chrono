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
// Wheeled vehicle system co-simulated with tire nodes and a terrain node.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#ifndef CH_VEHCOSIM_WHEELED_VEHICLE_NODE_H
#define CH_VEHCOSIM_WHEELED_VEHICLE_NODE_H

#include "chrono_vehicle/ChPowertrainAssembly.h"
#include "chrono_vehicle/ChTerrain.h"
#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/ChTire.h"

#include "chrono_vehicle/cosim/ChVehicleCosimWheeledMBSNode.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_cosim_mbs
/// @{

/// Wheeled vehicle co-simulation node.
/// The vehicle system is co-simulated with tire nodes and a terrain node.
class CH_VEHICLE_API ChVehicleCosimWheeledVehicleNode : public ChVehicleCosimWheeledMBSNode {
  public:
    /// Construct a wheeled vehicle node using the provided vehicle and powertrain JSON specification files.
    ChVehicleCosimWheeledVehicleNode(const std::string& vehicle_json,      ///< vehicle JSON specification file
                                     const std::string& engine_json,       ///< engine JSON specification file
                                     const std::string& transmission_json  ///< transmission JSON specification file
    );

    /// Construct a wheeled vehicle node using the provided vehicle and powertrain objects.
    /// Notes:
    /// - the provided vehicle system must be constructed with a null Chrono system.
    /// - the vehicle and powertrain system should not be initialized.
    ChVehicleCosimWheeledVehicleNode(std::shared_ptr<ChWheeledVehicle> vehicle,        ///< vehicle system
                                     std::shared_ptr<ChPowertrainAssembly> powertrain  ///< powertrain system
    );

    ~ChVehicleCosimWheeledVehicleNode();

    /// Get the underlying vehicle system.
    std::shared_ptr<ChWheeledVehicle> GetVehicle() const { return m_vehicle; }

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
    virtual void InitializeMBS(const std::vector<ChVector<>>& tire_info,  ///< mass, radius, width for each tire
                               const ChVector2<>& terrain_size,           ///< terrain length x width
                               double terrain_height                      ///< initial terrain height
                               ) override;

    // Output vehicle data.
    virtual void OnOutputData(int frame) override;

    /// Perform vehicle system synchronization before advancing the dynamics.
    virtual void PreAdvance() override;

    /// Process the provided spindle force (received from the corresponding tire node).
    virtual void ApplySpindleForce(unsigned int i, const TerrainForce& spindle_force) override;

    /// Return the number of spindles in the vehicle system.
    virtual int GetNumSpindles() const override;

    /// Return the i-th spindle body in the vehicle system.
    virtual std::shared_ptr<ChBody> GetSpindleBody(unsigned int i) const override;

    /// Return the vertical mass load on the i-th spindle.
    virtual double GetSpindleLoad(unsigned int i) const override;

    /// Get the body state of the spindle body to which the i-th wheel/tire is attached.
    virtual BodyState GetSpindleState(unsigned int i) const override;

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
        virtual double GetTireMass() const override { return m_mass; }
        virtual double GetAddedMass() const override { return m_mass; }
        virtual ChVector<> GetTireInertia() const override { return ChVector<>(0.1, 0.1, 0.1); }
        virtual ChVector<> GetAddedInertia() const override { return ChVector<>(0.1, 0.1, 0.1); }
        virtual TerrainForce ReportTireForce(ChTerrain* terrain) const override { return m_force; }
        virtual TerrainForce ReportTireForce(ChTerrain* terrain, ChCoordsys<>& tire_frame) const override {
            return m_force;
        }
        virtual TerrainForce GetTireForce() const override { return m_force; }
        virtual void InitializeInertiaProperties() override {}
        virtual void UpdateInertiaProperties() override {}

        int m_index;
        double m_mass;
        double m_radius;
        double m_width;
        TerrainForce m_force;
    };

    std::shared_ptr<ChWheeledVehicle> m_vehicle;         ///< vehicle MBS
    std::shared_ptr<ChPowertrainAssembly> m_powertrain;  ///< vehicle powertrain
    std::shared_ptr<ChDriver> m_driver;                  ///< vehicle driver
    std::shared_ptr<ChTerrain> m_terrain;                ///< dummy terrain (for vehicle synchronization)
    std::vector<std::shared_ptr<DummyTire>> m_tires;     ///< dummy tires (for applying spindle forces)

    ChVector<> m_init_loc;  ///< initial vehicle location (relative to center of terrain top surface)
    double m_init_yaw;      ///< initial vehicle yaw
    bool m_chassis_fixed;   ///< fix chassis to ground

    int m_num_spindles;                   ///< number of spindles/wheels of the wheeled vehicle
    std::vector<double> m_spindle_loads;  ///< vertical loads on each spindle
};

/// @} vehicle_cosim_mbs

}  // end namespace vehicle
}  // end namespace chrono

#endif
