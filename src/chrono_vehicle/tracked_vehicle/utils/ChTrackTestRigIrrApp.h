// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
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
// Irrlicht-based visualization wrapper for track test rig.
// This class extends ChVehicleIrrApp.
//
// =============================================================================

#ifndef CH_TRACK_TESTRIG_IRRAPP_H
#define CH_TRACK_TESTRIG_IRRAPP_H

#include "chrono_vehicle/utils/ChVehicleIrrApp.h"
#include "chrono_vehicle/tracked_vehicle/test_rig/ChTrackTestRig.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_utils
/// @{

/// Customized Chrono Irrlicht application for track test rig visualization.
class CH_VEHICLE_API ChTrackTestRigIrrApp : public ChVehicleIrrApp {
  public:
    /// Construct a track test rig Irrlicht application.
    ChTrackTestRigIrrApp(ChTrackTestRig* rig,  ///< pointer to the associated vehicle system
                         const std::wstring& title = L"Chrono::Vehicle Track Test Rig",  ///< window title
                         const irr::core::dimension2d<irr::u32>& dims =
                             irr::core::dimension2d<irr::u32>(1000, 800),  ///< window dimensions
                         irr::ELOG_LEVEL log_level = irr::ELL_INFORMATION  ///< Irrlicht logging level
    );

    ~ChTrackTestRigIrrApp() {}

  private:
    virtual void renderOtherGraphics() override;
    void renderContacts(const std::list<ChTrackContactManager::ContactInfo>& lst,
                        const irr::video::SColor& col,
                        bool normals,
                        bool forces,
                        double scale_normals,
                        double scale_forces);

    ChTrackTestRig* m_rig;
};

/// @} vehicle_tracked_utils

}  // end namespace vehicle
}  // end namespace chrono

#endif
