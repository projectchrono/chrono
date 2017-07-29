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
// Base class for a sprocket template with gear profile composed of circular arcs
// and line segments, suitable for interaction with a continuous band track.
//
// =============================================================================

#include <cmath>

#include "chrono_vehicle/tracked_vehicle/sprocket/ChSprocketCB.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeRigidCB.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
class SprocketCBContactCallback : public ChSystem::CustomCollisionCallback {
  public:
    SprocketCBContactCallback(ChTrackAssembly* track,  ///< containing track assembly
                              double envelope,         ///< collision detection envelope
                              int gear_nteeth          ///< number of teeth of the sprocket gear
                              )
        : m_track(track), m_envelope(envelope), m_sprocket(m_track->GetSprocket()), m_gear_nteeth(gear_nteeth) {
        //// TODO
    }

    virtual void OnCustomCollision(ChSystem* system) override;

  private:
    ChTrackAssembly* m_track;                // pointer to containing track assembly
    std::shared_ptr<ChSprocket> m_sprocket;  // handle to the sprocket

    double m_envelope;  // collision detection envelope
    int m_gear_nteeth;  // sprocket gear, number of teeth
};

// Add contacts between the sprocket and track shoes.
void SprocketCBContactCallback::OnCustomCollision(ChSystem* system) {
    // Return now if collision disabled on sproket.
    if (!m_sprocket->GetGearBody()->GetCollide())
        return;

    // Sprocket gear center location (expressed in global frame)
    ChVector<> locS_abs = m_sprocket->GetGearBody()->GetPos();

    // Loop over all track shoes in the associated track
    for (size_t is = 0; is < m_track->GetNumTrackShoes(); ++is) {
        auto shoe = std::static_pointer_cast<ChTrackShoeRigidCB>(m_track->GetTrackShoe(is));

        //// TODO
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChSprocketCB::ChSprocketCB(const std::string& name) : ChSprocket(name) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChSystem::CustomCollisionCallback* ChSprocketCB::GetCollisionCallback(ChTrackAssembly* track) {
    // Check compatibility between this type of sprocket and the track shoes.
    // We expect track shoes of type ChTrackShoeRigidCB.
    auto shoe = std::dynamic_pointer_cast<ChTrackShoeRigidCB>(track->GetTrackShoe(0));
    assert(shoe);

    // Extract parameterization of gear profile
    int gear_nteeth = GetNumTeeth();

    // Create and return the callback object. Note: this pointer will be freed by the base class.
    return new SprocketCBContactCallback(track, 0.005, gear_nteeth);
}

// -----------------------------------------------------------------------------
// Create and return the sprocket gear profile.
// -----------------------------------------------------------------------------
std::shared_ptr<geometry::ChLinePath> ChSprocketCB::GetProfile() {
    auto profile = std::make_shared<geometry::ChLinePath>();

    int num_teeth = GetNumTeeth();

    for (int i = 0; i < num_teeth; ++i) {
        //// TODO
        ////profile->AddSubLine(arc1);
    }

    return profile;
}

}  // end namespace vehicle
}  // end namespace chrono
