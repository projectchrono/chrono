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
// LuGre tire constructed with data from file (JSON format).
//
// =============================================================================

#ifndef LUGRE_TIRE_H
#define LUGRE_TIRE_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/tire/ChLugreTire.h"

#include "thirdparty/rapidjson/document.h"

namespace chrono {

class CH_VEHICLE_API LugreTire : public ChLugreTire {
  public:
    LugreTire(const std::string& filename);
    LugreTire(const rapidjson::Document& d);
    ~LugreTire();

    virtual int getNumDiscs() const override { return m_numDiscs; }
    virtual double getRadius() const override { return m_radius; }
    virtual const double* getDiscLocations() const override { return m_discLocs; }

    virtual double getNormalStiffness() const override { return m_normalStiffness; }
    virtual double getNormalDamping() const override { return m_normalDamping; }

    virtual void SetLugreParams() override {}

  private:
    void Create(const rapidjson::Document& d);

    double m_radius;
    int m_numDiscs;
    double* m_discLocs;

    double m_normalStiffness;
    double m_normalDamping;
};

}  // end namespace chrono

#endif
