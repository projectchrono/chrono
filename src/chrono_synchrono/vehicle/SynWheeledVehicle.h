// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young
// =============================================================================
//
// Wrapper class for ChWheeledVehicles. Additional functions here are related to
// initializing this as a zombie (setting visual representations, # of wheels)
//
// =============================================================================

#ifndef SYN_WHEELED_VEHICLE_H
#define SYN_WHEELED_VEHICLE_H

#include "chrono_synchrono/vehicle/SynVehicle.h"
#include "chrono_synchrono/flatbuffer/message/SynWheeledVehicleMessage.h"

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_vehicle
/// @{

/// Wrapper class for ChWheeledVehicles. Functions here are related to initializing this as a zombie (setting visual
/// representations, # of wheels)
class SYN_API SynWheeledVehicle : public SynVehicle {
  public:
    ///@brief Default constructor for a SynWheeledVehicle
    SynWheeledVehicle();

    ///@brief Constructor for a SynWheeledVehicle where a vehicle pointer is passed
    /// In this case, the system is assumed to have been created by the vehicle,
    /// so the system will subsequently not be advanced or destroyed.
    ///
    ///@param wheeled_vehicle the vehicle to wrap
    SynWheeledVehicle(vehicle::ChWheeledVehicle* wheeled_vehicle);

    ///@brief Constructor for a wheeled vehicle specified through json and a contact method
    /// This constructor will create it's own system
    ///
    ///@param coord_sys the initial position and orientation of the vehicle
    ///@param filename the json specification file to build the vehicle from
    ///@param contact_method the contact method used for the Chrono dynamics
    SynWheeledVehicle(const ChCoordsys<>& coord_sys, const std::string& filename, ChContactMethod contact_method);

    ///@brief Constructor for a wheeled vehicle specified through json and a contact method
    /// This vehicle will use the passed system and not create its own
    ///
    ///@param coord_sys the initial position and orientation of the vehicle
    ///@param filename the json specification file to build the vehicle from
    ///@param system the system to be used for Chrono dynamics and to add bodies to
    SynWheeledVehicle(const ChCoordsys<>& coord_sys, const std::string& filename, ChSystem* system);

    ///@brief Construct a zombie SynWheeledVehicle from a json specification file
    ///
    ///@param filename the json specification file to build the zombie from
    SynWheeledVehicle(const std::string& filename);

    ///@brief Destructor for a SynWheeledVehicle
    virtual ~SynWheeledVehicle();

    ///@brief Initialize the zombie representation of the vehicle.
    /// Will create and add bodies that are visual representation of the vehicle.
    /// The position and orientation is updated by SynChrono and the passed messages
    ///
    ///@param system the system used to add bodies to for consistent visualization
    virtual void InitializeZombie(ChSystem* system) override;

    ///@brief Synchronize the position and orientation of this vehicle with other ranks.
    /// Any message can be passed, so a check should be done to ensure this message was intended for this agent
    ///
    ///@param message the received message that describes the position and orientation
    virtual void SynchronizeZombie(SynMessage* message) override;

    ///@brief Update the current state of this vehicle
    virtual void Update() override;

    // ------------------------------------------------------------------------

    ///@brief Set the zombie visualization files
    ///
    ///@param chassis_vis_file the file used for chassis visualization
    ///@param wheel_vis_file the file used for wheel visualization
    ///@param tire_vis_file the file used for tire visualization
    void SetZombieVisualizationFiles(std::string chassis_vis_file,
                                     std::string wheel_vis_file,
                                     std::string tire_vis_file);

    ///@brief Set the number of wheels of the underlying vehicle
    ///
    ///@param num_wheels number of wheels of the underlying vehicle
    void SetNumWheels(int num_wheels) { m_description->m_num_wheels = num_wheels; }

    // ------------------------------------------------------------------------

    // ------------------------------
    // Helper methods for convenience
    // ------------------------------

    ///@brief Update the state of this vehicle at the current time.
    ///
    ///@param time the time to synchronize to
    ///@param driver_inputs the driver inputs (i.e. throttle, braking, steering)
    ///@param terrain reference to the terrain the vehicle should contact
    void Synchronize(double time, const vehicle::ChDriver::Inputs& driver_inputs, const vehicle::ChTerrain& terrain);

    ///@brief Get the underlying vehicle
    ///
    ///@return ChVehicle&
    virtual vehicle::ChVehicle& GetVehicle() override { return *m_wheeled_vehicle; }

  protected:
    ///@brief Parse a JSON specification file that describes a vehicle
    ///
    ///@param filename the json specification file
    virtual rapidjson::Document ParseVehicleFileJSON(const std::string& filename) override;

    ///@brief Helper method for creating a vehicle from a json specification file
    ///
    ///@param coord_sys the initial position and orientation of the vehicle
    ///@param filename the json specification file
    ///@param system a ChSystem to be used when constructing a vehicle
    virtual void CreateVehicle(const ChCoordsys<>& coord_sys, const std::string& filename, ChSystem* system) override;

  private:
    vehicle::ChWheeledVehicle* m_wheeled_vehicle;  ///< Pointer to the ChWheeledVehicle this class wraps

    std::shared_ptr<SynWheeledVehicleState> m_state;  ///< State of the vehicle (See SynWheeledVehicleMessage)
    std::shared_ptr<SynWheeledVehicleDescription> m_description;  ///< Description for zombie creation on discovery

    std::vector<std::shared_ptr<ChBodyAuxRef>> m_wheel_list;  ///< vector of this agent's zombie wheels

    friend class SynWheeledVehicleAgent;
};

// A class which wraps a vehicle model that contains a ChWheeledVehicle
template <class V>
class SYN_API SynCustomWheeledVehicle : public SynWheeledVehicle {
  public:
    // Constructor for non-zombie vehicle
    SynCustomWheeledVehicle(std::shared_ptr<V> vehicle_model) : SynWheeledVehicle(&vehicle_model->GetVehicle()) {
        m_vehicle_model = vehicle_model;
        m_system = vehicle_model->GetSystem();
    }

  private:
    std::shared_ptr<V> m_vehicle_model;
};

/// @} synchrono_vehicle

}  // namespace synchrono
}  // namespace chrono

#endif