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
// Authors: Daniel Melanz
// =============================================================================
//
// MacPherson strut suspension constructed with data from file.
//
// =============================================================================

#ifndef MACPHERSONSTRUT_H
#define MACPHERSONSTRUT_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ChMacPhersonStrut.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_suspension
/// @{

/// MacPherson strut suspension constructed with data from file.
class CH_VEHICLE_API MacPhersonStrut : public ChMacPhersonStrut {
  public:
    MacPhersonStrut(const std::string& filename);
    MacPhersonStrut(const rapidjson::Document& d);
    ~MacPhersonStrut();

    virtual double getSpindleMass() const override { return m_spindleMass; }
    virtual double getStrutMass() const override { return m_strutMass; }
    virtual double getLCAMass() const override { return m_LCAMass; }
    virtual double getUprightMass() const override { return m_uprightMass; }

    virtual double getSpindleRadius() const override { return m_spindleRadius; }
    virtual double getSpindleWidth() const override { return m_spindleWidth; }
    virtual double getStrutRadius() const override { return m_strutRadius; }
    virtual double getLCARadius() const override { return m_LCARadius; }
    virtual double getUprightRadius() const override { return m_uprightRadius; }

    virtual const ChVector<>& getSpindleInertia() const override { return m_spindleInertia; }
    virtual const ChVector<>& getStrutInertia() const override { return m_strutInertia; }
    virtual const ChVector<>& getLCAInertia() const override { return m_LCAInertia; }
    virtual const ChVector<>& getUprightInertia() const override { return m_uprightInertia; }

    virtual double getAxleInertia() const override { return m_axleInertia; }

    virtual double getSpringRestLength() const override { return m_springRestLength; }
    virtual ChLinkSpringCB::ForceFunctor* getSpringForceFunctor() const override { return m_springForceCB; }
    virtual ChLinkSpringCB::ForceFunctor* getShockForceFunctor() const override { return m_shockForceCB; }

  private:
    virtual const ChVector<> getLocation(PointId which) override { return m_points[which]; }

    virtual void Create(const rapidjson::Document& d) override;

    ChLinkSpringCB::ForceFunctor* m_springForceCB;
    ChLinkSpringCB::ForceFunctor* m_shockForceCB;

    ChVector<> m_points[NUM_POINTS];

    double m_spindleMass;
    double m_strutMass;
    double m_LCAMass;
    double m_uprightMass;

    double m_spindleRadius;
    double m_spindleWidth;
    double m_strutRadius;
    double m_LCARadius;
    double m_uprightRadius;

    ChVector<> m_spindleInertia;
    ChVector<> m_strutInertia;
    ChVector<> m_LCAInertia;
    ChVector<> m_uprightInertia;

    double m_axleInertia;

    double m_springRestLength;
};

/// @} vehicle_wheeled_suspension

}  // end namespace vehicle
}  // end namespace chrono

#endif
