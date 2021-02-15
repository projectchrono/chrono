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
// MTV balancer subsystem (installed on rear chassis)
//
// =============================================================================

#ifndef MTV_BALANCER_H
#define MTV_BALANCER_H

#include "chrono_models/ChApiModels.h"
#include "chrono_vehicle/wheeled_vehicle/subchassis/ChBalancer.h"

namespace chrono {
namespace vehicle {
namespace fmtv {

/// @addtogroup vehicle_models_fmtv
/// @{

/// MTV balancer subsystem (installed on rear chassis).
class CH_MODELS_API MTV_Balancer : public ChBalancer {
  public:
    MTV_Balancer(const std::string& name);

    virtual const ChVector<> GetLocation(PointId which) override;

    virtual double GetBalancerBeamMass() const override { return m_beam_mass; }
    virtual const ChVector<>& GetBalancerBeamInertia() const override { return m_beam_inertia; }
    virtual const double GetBalancerMaxPitch() const override { return m_beam_max_pitch; }
    virtual const ChVector<>& GetBalancerBeamDimensions() const override { return m_beam_dimensions; }

  private:
    static const double m_beam_max_pitch;
    static const double m_beam_mass;
    static const ChVector<> m_beam_inertia;
    static const ChVector<> m_beam_dimensions;
};

/// @} vehicle_models_fmtv

}  // namespace fmtv
}  // end namespace vehicle
}  // end namespace chrono

#endif