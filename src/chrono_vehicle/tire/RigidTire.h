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
// Rigid tire constructed with data from file (JSON format).
//
// =============================================================================

#ifndef RIGID_TIRE_H
#define RIGID_TIRE_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/tire/ChRigidTire.h"

#include "thirdparty/rapidjson/document.h"

namespace chrono {

class CH_VEHICLE_API RigidTire : public ChRigidTire {
  public:
    RigidTire(const std::string& filename);
    RigidTire(const rapidjson::Document& d);
    ~RigidTire() {}

    virtual double getRadius() const override { return m_radius; }
    virtual double getWidth() const override { return m_width; }

  private:
    void Create(const rapidjson::Document& d);

    double m_radius;
    double m_width;
};

}  // end namespace chrono

#endif
