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
// Classes for monitoring contacts of tracked vehicle subsystems.
//
// =============================================================================

#ifndef CH_TRACK_CONTACT_MANAGER
#define CH_TRACK_CONTACT_MANAGER

#include <list>

#include "chrono/physics/ChContactContainerBase.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/ChChassis.h"
#include "chrono_vehicle/tracked_vehicle/ChSprocket.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackShoe.h"
#include "chrono_vehicle/tracked_vehicle/ChIdler.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked
/// @{

class ChTrackedVehicle;

/// Contact information data structure.
struct ChTrackContactInfo {
    ChVector<> m_point;
    ChMatrix33<> m_csys;
    ChVector<> m_force;
    ChVector<> m_torque;
};

/// Class for monitoring contacts of tracked vehicle subsystems.
class ChTrackContactManager : public chrono::ChReportContactCallback {
  public:
    ChTrackContactManager();

    void MonitorContacts(int flags) { m_flags |= flags; }
    void SetContactCollection(bool val) { m_collect = val; }
    void WriteContacts(const std::string& filename);

    void SetTrackShoeIndexLeft(size_t idx) { m_shoe_index_L = idx; }
    void SetTrackShoeIndexRight(size_t idx) { m_shoe_index_R = idx; }

    void Process(ChTrackedVehicle* vehicle);

    bool InContact(TrackedCollisionFlag::Enum part) const;

  private:
    bool IsFlagSet(TrackedCollisionFlag::Enum val) { return (m_flags & static_cast<int>(val)) != 0; }

    /// Callback, used to report contact points already added to the container.
    /// If it returns false, the contact scanning will be stopped.
    virtual bool ReportContactCallback(const ChVector<>& pA,
                                       const ChVector<>& pB,
                                       const ChMatrix33<>& plane_coord,
                                       const double& distance,
                                       const ChVector<>& react_forces,
                                       const ChVector<>& react_torques,
                                       ChContactable* modA,
                                       ChContactable* modB) override;

    bool m_initialized;  ///< true if the contact manager was initialized
    int m_flags;         ///< contact bit flags
    bool m_collect;      ///< flag indicating whether or not data is collected

    utils::CSV_writer m_csv;

    std::shared_ptr<ChChassis> m_chassis;
    std::shared_ptr<ChSprocket> m_sprocket_L;
    std::shared_ptr<ChSprocket> m_sprocket_R;
    std::shared_ptr<ChIdler> m_idler_L;
    std::shared_ptr<ChIdler> m_idler_R;
    std::shared_ptr<ChTrackShoe> m_shoe_L;
    std::shared_ptr<ChTrackShoe> m_shoe_R;

    size_t m_shoe_index_L;                                ///< index of monitored track shoe on left track
    size_t m_shoe_index_R;                                ///< index of monitored track shoe on right track

    std::list<ChTrackContactInfo> m_chassis_contacts;     ///< list of contacts on chassis
    std::list<ChTrackContactInfo> m_sprocket_L_contacts;  ///< list of contacts on left sprocket gear
    std::list<ChTrackContactInfo> m_sprocket_R_contacts;  ///< list of contacts on right sprocket gear
    std::list<ChTrackContactInfo> m_shoe_L_contacts;      ///< list of contacts on left track shoe
    std::list<ChTrackContactInfo> m_shoe_R_contacts;      ///< list of contacts on right track shoe
    std::list<ChTrackContactInfo> m_idler_L_contacts;     ///< list of contacts on left idler wheel
    std::list<ChTrackContactInfo> m_idler_R_contacts;     ///< list of contacts on right idler wheel

    friend class ChTrackedVehicleIrrApp;
};

/// @} vehicle_tracked

}  // end namespace vehicle
}  // end namespace chrono

#endif
