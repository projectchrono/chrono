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

    virtual bool UseTierodBodies() const override { return m_use_tierod_bodies; }

    virtual double getCamberAngle() const override { return m_camber_angle; }
    virtual double getToeAngle() const override { return m_toe_angle; }

    virtual double getSpindleMass() const override { return m_spindleMass; }
    virtual double getStrutMass() const override { return m_strutMass; }
    virtual double getLCAMass() const override { return m_LCAMass; }
    virtual double getUprightMass() const override { return m_uprightMass; }
    virtual double getTierodMass() const override { return m_tierodMass; }

    virtual double getSpindleRadius() const override { return m_spindleRadius; }
    virtual double getSpindleWidth() const override { return m_spindleWidth; }
    virtual double getStrutRadius() const override { return m_strutRadius; }
    virtual double getLCARadius() const override { return m_LCARadius; }
    virtual double getUprightRadius() const override { return m_uprightRadius; }
    virtual double getTierodRadius() const override { return m_tierodRadius; }

    virtual const ChVector<>& getSpindleInertia() const override { return m_spindleInertia; }
    virtual const ChVector<>& getStrutInertia() const override { return m_strutInertia; }
    virtual const ChVector<>& getLCAInertia() const override { return m_LCAInertia; }
    virtual const ChVector<>& getUprightInertia() const override { return m_uprightInertia; }
    virtual const ChVector<> getTierodInertia() const override { return m_tierodInertia; }

    virtual double getAxleInertia() const override { return m_axleInertia; }

    virtual double getSpringRestLength() const override { return m_springRestLength; }
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getSpringForceFunctor() const override { return m_springForceCB; }
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getShockForceFunctor() const override { return m_shockForceCB; }

    virtual std::shared_ptr<ChVehicleBushingData> getLCABushingData() const override { return m_LCABushingData; }
    virtual std::shared_ptr<ChVehicleBushingData> getTierodBushingData() const override { return m_tierodBushingData; }

  private:
    virtual const ChVector<> getLocation(PointId which) override { return m_points[which]; }

    virtual void Create(const rapidjson::Document& d) override;

    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_springForceCB;
    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_shockForceCB;

    ChVector<> m_points[NUM_POINTS];

    bool m_use_tierod_bodies;

    double m_camber_angle;
    double m_toe_angle;

    double m_spindleMass;
    double m_strutMass;
    double m_LCAMass;
    double m_uprightMass;
    double m_tierodMass;

    double m_spindleRadius;
    double m_spindleWidth;
    double m_strutRadius;
    double m_LCARadius;
    double m_uprightRadius;
    double m_tierodRadius;

    ChVector<> m_spindleInertia;
    ChVector<> m_strutInertia;
    ChVector<> m_LCAInertia;
    ChVector<> m_uprightInertia;
    ChVector<> m_tierodInertia;

    double m_axleInertia;

    std::shared_ptr<ChVehicleBushingData> m_LCABushingData;
    std::shared_ptr<ChVehicleBushingData> m_tierodBushingData;

    double m_springRestLength;
};

/// @} vehicle_wheeled_suspension

}  // end namespace vehicle
}  // end namespace chrono

#endif
