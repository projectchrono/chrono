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
// Double-A arm suspension constructed with data from file.
//
// =============================================================================

#ifndef DOUBLEWISHBONE_H
#define DOUBLEWISHBONE_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ChDoubleWishbone.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_suspension
/// @{

/// Double-A arm suspension constructed with data from file.
class CH_VEHICLE_API DoubleWishbone : public ChDoubleWishbone {
  public:
    DoubleWishbone(const std::string& filename);
    DoubleWishbone(const rapidjson::Document& d);
    ~DoubleWishbone();

    virtual bool UseTierodBodies() const override { return m_use_tierod_bodies; }

    virtual double getCamberAngle() const override { return m_camber_angle; }
    virtual double getToeAngle() const override { return m_toe_angle; }

    virtual double getSpindleMass() const override { return m_spindleMass; }
    virtual double getUCAMass() const override { return m_UCAMass; }
    virtual double getLCAMass() const override { return m_LCAMass; }
    virtual double getUprightMass() const override { return m_uprightMass; }
    virtual double getTierodMass() const override { return m_tierodMass; }

    virtual double getSpindleRadius() const override { return m_spindleRadius; }
    virtual double getSpindleWidth() const override { return m_spindleWidth; }
    virtual double getUCARadius() const override { return m_UCARadius; }
    virtual double getLCARadius() const override { return m_LCARadius; }
    virtual double getUprightRadius() const override { return m_uprightRadius; }
    virtual double getTierodRadius() const override { return m_tierodRadius; }

    virtual const ChVector<>& getSpindleInertia() const override { return m_spindleInertia; }
    virtual const ChVector<>& getUCAInertiaMoments() const override { return m_UCAInertiaMoments; }
    virtual const ChVector<>& getUCAInertiaProducts() const override { return m_UCAInertiaProducts; }
    virtual const ChVector<>& getLCAInertiaMoments() const override { return m_LCAInertiaMoments; }
    virtual const ChVector<>& getLCAInertiaProducts() const override { return m_LCAInertiaProducts; }
    virtual const ChVector<>& getUprightInertiaMoments() const override { return m_uprightInertiaMoments; }
    virtual const ChVector<>& getUprightInertiaProducts() const override { return m_uprightInertiaProducts; }
    virtual const ChVector<> getTierodInertia() const override { return m_tierodInertia; }

    virtual double getAxleInertia() const override { return m_axleInertia; }

    virtual double getSpringRestLength() const override { return m_springRestLength; }
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getSpringForceFunctor() const override { return m_springForceCB; }
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getShockForceFunctor() const override { return m_shockForceCB; }

    virtual std::shared_ptr<ChVehicleBushingData> getUCABushingData() const override { return m_UCABushingData; }
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
    double m_UCAMass;
    double m_LCAMass;
    double m_uprightMass;
    double m_tierodMass;

    double m_spindleRadius;
    double m_spindleWidth;
    double m_UCARadius;
    double m_LCARadius;
    double m_uprightRadius;
    double m_tierodRadius;

    ChVector<> m_spindleInertia;
    ChVector<> m_UCAInertiaMoments;
    ChVector<> m_UCAInertiaProducts;
    ChVector<> m_LCAInertiaMoments;
    ChVector<> m_LCAInertiaProducts;
    ChVector<> m_uprightInertiaMoments;
    ChVector<> m_uprightInertiaProducts;
    ChVector<> m_tierodInertia;

    double m_axleInertia;

    std::shared_ptr<ChVehicleBushingData> m_UCABushingData;
    std::shared_ptr<ChVehicleBushingData> m_LCABushingData;
    std::shared_ptr<ChVehicleBushingData> m_tierodBushingData;

    double m_springRestLength;
};

/// @} vehicle_wheeled_suspension

}  // end namespace vehicle
}  // end namespace chrono

#endif
