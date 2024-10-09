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
// Tracked vehicle model constructed from a JSON specification file
//
// =============================================================================

#ifndef TRACKED_VEHICLE_H
#define TRACKED_VEHICLE_H

#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicle.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked
/// @{

/// Constructors for different vehicle subsystems. Used to construct custom
/// instances of the subsystems from a JSON document.
struct CH_VEHICLE_API CustomConstructorsTV {

    /// A constructor for creating a vehicle subsystem from a JSON document.
    template<typename T>
    using CustomConstructor =
        std::function<std::shared_ptr<T>(
            const std::string&,         /// The name of the template to create an instance of
            const rapidjson::Document&  /// The JSON document
        )>;

    CustomConstructor<ChChassis> chassis_constructor;
    CustomConstructor<ChTrackAssembly> track_assembly_constructor;
    CustomConstructor<ChDrivelineTV> driveline_tv_constructor;
};

/// Tracked vehicle model constructed from a JSON specification file.
class CH_VEHICLE_API TrackedVehicle : public ChTrackedVehicle {
  public:
    TrackedVehicle(const std::string& filename,
                   ChContactMethod contact_method = ChContactMethod::NSC,
                   CustomConstructorsTV custom_constructors = {});

    TrackedVehicle(ChSystem* system,
                   const std::string& filename,
                   CustomConstructorsTV custom_constructors = {});

    ~TrackedVehicle() {}

    virtual void Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel = 0) override;

  private:
    void Create(const std::string& filename, CustomConstructorsTV custom_constructors);

  private:
    double m_track_offset[2];  ///< offsets for the left and right track assemblies
};

/// @} vehicle_tracked

}  // end namespace vehicle
}  // end namespace chrono

#endif
