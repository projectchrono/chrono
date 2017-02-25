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

class CH_MODELS_API Generic_FialaTire : public ChFialaTire {
  public:
    Generic_FialaTire(const std::string& name);
    ~Generic_FialaTire() {}

    virtual double GetNormalStiffnessForce(double depth) const override;
    virtual double GetNormalDampingForce(double depth, double velocity) const override {
        return m_normalDamping * velocity;
    }

    virtual void SetFialaParams() override;

  private:
    static const double m_normalStiffness;
    static const double m_normalDamping;
};

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono

#endif
