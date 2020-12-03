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
// Authors: Aaron Young, Jay Taves
// =============================================================================
//
// Class that wraps a ChVehicle. The underlying vehicle dynamics remain
// untouched; however, additional functionality is added to allow for SynChrono
// to pass messages and remain synchronized across ranks.
//
// Currently, both wheeled and tracked vehicles are wrapped. See
// SynWheeledVehicle and SynTrackedVehicle for their additional methods.
//
// A SynVehicle will always on the system unless a vehicle is passed in through
// the SynCustom__Vehicle classes. This means that either the system is
// passed on instantiation or is created by this class. If this was not the case
// the underlying ChVehicle may instantiate its own. This can cause issues with
// destruction and nullptrs associated with the systems.
//
// =============================================================================

#ifndef SYN_VEHICLE_H
#define SYN_VEHICLE_H

#include "chrono_synchrono/SynApi.h"
#include "chrono_synchrono/utils/SynUtilsJSON.h"
#include "chrono_synchrono/flatbuffer/message/SynAgentMessage.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/ChTerrain.h"
#include "chrono_vehicle/ChDriver.h"

#include "chrono/assets/ChTriangleMeshShape.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_vehicle
/// @{

/// Base class for all vehicle wrappers. Forces them to provide for initialization and synchronization of their zombie
class SYN_API SynVehicle {
  public:
    ///@brief Default constructor for a SynVehicle
    ///
    ///@param is_zombie whether this agent is a zombie
    SynVehicle(bool is_zombie = true);

    ///@brief Constructor for vehicles specified through json and a contact method
    /// This constructor will create it's own system
    ///
    ///@param filename the json specification file to build the vehicle from
    ///@param contact_method the contact method used for the Chrono dynamics
    SynVehicle(const std::string& filename, ChContactMethod contact_method);

    ///@brief Constructor for vehicles specified through json and a contact method
    /// This vehicle will use the passed system and not create its own
    ///
    ///@param filename the json specification file to build the vehicle from
    ///@param system the system to be used for Chrono dynamics and to add bodies to
    SynVehicle(const std::string& filename, ChSystem* system);

    ///@brief Constructor for zombie vehicles specified through json.
    /// In order to visualize objects in the correct environment, no ChSystem is initialized
    ///
    ///@param filename the json specification file to build the zombie from
    SynVehicle(const std::string& filename);

    ///@brief Destroy the SynVehicle object
    virtual ~SynVehicle();

    ///@brief Initialize the zombie representation of the vehicle.
    /// Will create and add bodies that are visual representation of the vehicle.
    /// The position and orientation is updated by SynChrono and the passed messages
    ///
    ///@param system the system used to add bodies to for consistent visualization
    virtual void InitializeZombie(ChSystem* system) = 0;

    ///@brief Synchronize the position and orientation of this vehicle with other ranks.
    /// Any message can be passed, so a check should be done to ensure this message was intended for this agent
    ///
    ///@param message the received message that describes the position and orientation
    virtual void SynchronizeZombie(SynMessage* message) = 0;

    ///@brief Update the current state of this vehicle
    virtual void Update() = 0;

    // ------------------------------------------------------------------------

    ///@brief Get whether this vehicle is a zombie
    ///
    ///@return true it is a zombie
    ///@return false it is not a zombie
    bool IsZombie() { return m_is_zombie; }

    // ------------------------------------------------------------------------

    // ------------------------------
    // Helper methods for convenience
    // ------------------------------

    ///@brief Advance the state of this vehicle by the time amount specified
    /// If the system is not owned by this vehicle, this method should be overriden
    ///
    ///@param step the amount of time to advance by
    virtual void Advance(double step);

    ///@brief Get the ChVehicle being wrapped by this class
    ///
    ///@return ChVehicle& the reference to the underlying ChVehicle
    virtual vehicle::ChVehicle& GetVehicle() = 0;

    ///@brief Get the ChSystem
    ///
    ///@return ChSystem*
    ChSystem* GetSystem() { return m_system; }

  protected:
    ///@brief Parse a JSON specification file that describes a vehicle
    ///
    ///@param filename the json specification file
    virtual rapidjson::Document ParseVehicleFileJSON(const std::string& filename);

    ///@brief Helper method for creating a vehicle from a json specification file
    ///
    ///@param coord_sys the initial position and orientation of the vehicle
    ///@param filename the json specification file
    ///@param system a ChSystem to be used when constructing a vehicle
    virtual void CreateVehicle(const ChCoordsys<>& coord_sys, const std::string& filename, ChSystem* system) = 0;

    ///@brief Construct a zombie from a json specification file
    ///
    ///@param filename the json specification file
    virtual void CreateZombie(const std::string& filename) { ParseVehicleFileJSON(filename); }

    ///@brief Helper method used to create a ChTriangleMeshShape to be used on as a zombie body
    ///
    ///@param filename the file to generate a ChTriangleMeshShape from
    ///@return std::shared_ptr<ChTriangleMeshShape>
    std::shared_ptr<ChTriangleMeshShape> CreateMeshZombieComponent(const std::string& filename);

    ///@brief Create a zombie chassis body. All ChVehicles have a chassis, so this can be defined here
    ///
    ///@param filename the filename that describes the ChTriangleMeshShape that should represent the chassis
    ///@param system the system to add the body to
    void CreateChassisZombieBody(const std::string& filename, ChSystem* system);

    bool m_is_zombie;     ///< is this vehicle a zombie
    bool m_owns_vehicle;  ///< has the vehicle been created by this object or passed by user

    ChSystem* m_system;  ///< pointer to the chrono system

    std::shared_ptr<ChBodyAuxRef> m_zombie_body;  ///< agent's zombie body reference
};

/// @} synchrono_vehicle

}  // namespace synchrono
}  // namespace chrono
#endif
