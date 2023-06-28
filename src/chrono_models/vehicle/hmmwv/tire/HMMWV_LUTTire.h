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
// HMMWV LUT tire subsystem
//
// =============================================================================

#ifndef HMMWV_LUT_TIRE_H
#define HMMWV_LUT_TIRE_H

#include "chrono_models/ChApiModels.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChLUTTire.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

/// @addtogroup vehicle_models_hmmwv
/// @{

/// Deformable LUT tire model for the HMMWV vehicle.
class CH_MODELS_API HMMWV_LUTTire : public ChLUTTire {
  public:
    HMMWV_LUTTire(const std::string& name);
    ~HMMWV_LUTTire() {}

    /// Get the default tire pressure.
    virtual double GetDefaultPressure() const override { return m_default_pressure; }

  private:
    static const int m_num_divs;
    static const std::vector<double> m_radius;
    static const std::vector<double> m_offset;

    static const double m_tire_mass;
    static const double m_default_pressure;

    static const float m_friction;
    static const float m_restitution;
    static const float m_Young;
    static const float m_Poisson;
    static const float m_kn;
    static const float m_gn;
    static const float m_kt;
    static const float m_gt;
};

/// @} vehicle_models_hmmwv

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono

#endif
