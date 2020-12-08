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
// Wrapper class for ChTrackedVehicles. Additional functions here are related to
// initializing this as a zombie (setting visual representations, treads)
//
// =============================================================================

#ifndef SYN_TRACKED_VEHICLE_H
#define SYN_TRACKED_VEHICLE_H

#include "chrono_synchrono/vehicle/SynVehicle.h"
#include "chrono_synchrono/flatbuffer/message/SynTrackedVehicleMessage.h"

#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicle.h"

// MSVC seems to need these otherwise it compiles out the templated customWheeledVehicle
#include "chrono_models/vehicle/m113/M113.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_vehicle
/// @{

// Wrapper class for ChTrackedVehicles. Additional functions here are related to initializing this as a zombie (setting
// visual representations, treads)
class SYN_API SynTrackedVehicle : public SynVehicle {
  public:
    ///@brief Default constructor for a SynWheeledVehicle
    SynTrackedVehicle();

    ///@brief Constructor for a SynWheeledVehicle where a vehicle pointer is passed
    /// In this case, the system is assumed to have been created by the vehicle,
    /// so the system will subsequently not be advanced or destroyed.
    ///
    ///@param tracked_vehicle the vehicle to wrap
    SynTrackedVehicle(vehicle::ChTrackedVehicle* tracked_vehicle);

    ///@brief Constructor for a tracked vehicle specified through json and a contact method
    /// This constructor will create it's own system
    ///
    ///@param coord_sys the initial position and orientation of the vehicle
    ///@param filename the json specification file to build the vehicle from
    ///@param contact_method the contact method used for the Chrono dynamics
    SynTrackedVehicle(const ChCoordsys<>& coord_sys, const std::string& filename, ChContactMethod contact_method);

    ///@brief Constructor for a tracked vehicle specified through json and a contact method
    /// This constructor will create it's own system
    ///
    ///@param coord_sys the initial position and orientation of the vehicle
    ///@param filename the json specification file to build the vehicle from
    ///@param system the system to be used for Chrono dynamics and to add bodies to
    SynTrackedVehicle(const ChCoordsys<>& coord_sys, const std::string& filename, ChSystem* system);

    ///@brief Construct a zombie SynTrackedVehicle from a json specification file
    ///
    ///@param filename the json specification file to build the zombie from
    SynTrackedVehicle(const std::string& filename);

    ///@brief Destructor for a SynTrackedVehicle
    virtual ~SynTrackedVehicle();

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

    ///@brief Set the zombie visualization files
    ///
    ///@param chassis_vis_file the file used for chassis visualization
    ///@param track_shoe_vis_file the file used for track shoe visualization
    ///@param left_sprocket_vis_file the file used for left sprocket visualization
    ///@param right_sprocket_vis_file the file used for right sprocket visualization
    ///@param left_idler_vis_file the file used for left idler visualization
    ///@param right_idler_vis_file the file used for right idler visualization
    ///@param left_road_wheel_vis_file the file used for left road_wheel visualization
    ///@param right_road_wheel_vis_file the file used for right road_wheel visualization
    void SetZombieVisualizationFiles(std::string chassis_vis_file,
                                     std::string track_shoe_vis_file,
                                     std::string left_sprocket_vis_file,
                                     std::string right_sprocket_vis_file,
                                     std::string left_idler_vis_file,
                                     std::string right_idler_vis_file,
                                     std::string left_road_wheel_vis_file,
                                     std::string right_road_wheel_vis_file);

    ///@brief Set the number of assembly components
    ///
    ///@param num_track_shoes number of track shoes
    ///@param num_sprockets number of sprockets
    ///@param num_idlers number of idlers
    ///@param num_road_wheels number of road_wheels
    void SetNumAssemblyComponents(int num_track_shoes, int num_sprockets, int num_idlers, int num_road_wheels);

    // ------------------------------------------------------------------------

    // ------------------------------
    // Helper methods for convenience
    // ------------------------------

    // Update the state of this vehicle at the current time.

    ///@brief Update the state of this vehicle at the current time.
    ///
    ///@param time the time to synchronize to
    ///@param driver_inputs the driver inputs (i.e. throttle, braking, steering)
    void Synchronize(double time, const vehicle::ChDriver::Inputs& driver_inputs);

    ///@brief Get the underlying vehicle
    ///
    ///@return ChVehicle&
    virtual vehicle::ChVehicle& GetVehicle() override { return *m_tracked_vehicle; }

  private:
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

    ///@brief Helper function for adding multiple trimeshes to a vector
    /// Will essentially instantiate a ChBodyAuxRef and set the trimesh as the asset for each
    ///
    ///@param trimesh the original trimesh
    ///@param ref_list the ref list to add the created bodies to
    ///@param system the system that each body must be added to so it can be visualized
    void AddMeshToVector(std::shared_ptr<ChTriangleMeshShape> trimesh,
                         std::vector<std::shared_ptr<ChBodyAuxRef>>& ref_list,
                         ChSystem* system);

    ///@brief Helper function for adding multiple left and right trimeshes to a vector
    /// Will essentially instantiate a ChBodyAuxRef and set the trimesh as the asset for each
    ///
    ///@param left the original left trimesh
    ///@param right the original right trimesh
    ///@param ref_list the ref list to add the created bodies to
    ///@param system the system that each body must be added to so it can be visualized
    void AddMeshToVector(std::shared_ptr<ChTriangleMeshShape> left,
                         std::shared_ptr<ChTriangleMeshShape> right,
                         std::vector<std::shared_ptr<ChBodyAuxRef>>& ref_list,
                         ChSystem* system);

  protected:
    vehicle::ChTrackedVehicle* m_tracked_vehicle;  ///< Pointer to the ChTrackedVehicle that this class wraps

    std::shared_ptr<SynTrackedVehicleState> m_state;  ///< State of the vehicle (See SynTrackedVehicleMessage)
    std::shared_ptr<SynTrackedVehicleDescription> m_description;  ///< Description for zombie creation on discovery

    vehicle::TerrainForces m_shoe_forces_left;   ///< Terrain forces at the track shoes on the left side
    vehicle::TerrainForces m_shoe_forces_right;  ///< Terrain forces at the track shoes on the right side

    std::vector<std::shared_ptr<ChBodyAuxRef>> m_track_shoe_list;  ///< vector of this agent's zombie track shoes
    std::vector<std::shared_ptr<ChBodyAuxRef>> m_sprocket_list;    ///< vector of this agent's zombie sprockets
    std::vector<std::shared_ptr<ChBodyAuxRef>> m_idler_list;       ///< vector of this agent's zombie idlers
    std::vector<std::shared_ptr<ChBodyAuxRef>> m_road_wheel_list;  ///< vector of this agent's zombie road wheels

    friend class SynTrackedVehicleAgent;
};

// A class which wraps a vehicle model that contains a ChWheeledVehicle
template <class V>
class SYN_API SynCustomTrackedVehicle : public SynTrackedVehicle {
  public:
    // Constructor for non-zombie vehicle
    SynCustomTrackedVehicle(std::shared_ptr<V> vehicle_model) : SynTrackedVehicle(&vehicle_model->GetVehicle()) {
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
