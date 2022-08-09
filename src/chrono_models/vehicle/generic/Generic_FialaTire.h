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
// Authors: Radu Serban, Michael Taylor
// =============================================================================
//
// Generic Fiala tire subsystem
//
// =============================================================================

#ifndef GENERIC_FIALA_TIRE_H
#define GENERIC_FIALA_TIRE_H

#include "chrono_vehicle/wheeled_vehicle/tire/ChFialaTire.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace generic {

/// @addtogroup vehicle_models_generic
/// @{

/// Fiala tire model for a generic vehicle.
class CH_MODELS_API Generic_FialaTire : public ChFialaTire {
  public:
    Generic_FialaTire(const std::string& name);
    ~Generic_FialaTire() {}

    virtual double GetNormalStiffnessForce(double depth) const override;
    virtual double GetNormalDampingForce(double depth, double velocity) const override;

    virtual double GetTireMass() const override { return m_mass; }
    virtual ChVector<> GetTireInertia() const override { return m_inertia; }

    virtual void SetFialaParams() override;

  private:
    static const double m_normalStiffness;
    static const double m_normalDamping;
    static const double m_mass;
    static const ChVector<> m_inertia;
};

/// @} vehicle_models_generic

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono

#endif
