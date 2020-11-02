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
// Wheeled trailer model constructed from a JSON specification file
//
// =============================================================================

#ifndef WHEELED_TRAILER_H
#define WHEELED_TRAILER_H

#include "chrono_vehicle/wheeled_vehicle/ChWheeledTrailer.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled
/// @{

/// Wheeled trailer model constructed from a JSON specification file.
class CH_VEHICLE_API WheeledTrailer : public ChWheeledTrailer {
  public:
    WheeledTrailer(ChSystem* system, const std::string& filename);

    ~WheeledTrailer() {}

    virtual int GetNumberAxles() const override { return m_num_axles; }

    virtual void Initialize(std::shared_ptr<ChChassis> frontChassis) override;

  private:
    void Create(const std::string& filename);

  private:
    int m_num_axles;                           // number of axles for this vehicle
    std::vector<ChVector<> > m_suspLocations;  // locations of the suspensions relative to chassis
    std::vector<double> m_wheelSeparations;    // wheel separations for each axle
};

/// @} vehicle_wheeled

}  // end namespace vehicle
}  // end namespace chrono

#endif
