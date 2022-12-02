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
// Authors: Daniel Melanz, Radu Serban
// =============================================================================
//
// Solid axle suspension constructed with data from file.
//
// =============================================================================

#ifndef SOLIDAXLE_H
#define SOLIDAXLE_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ChSolidAxle.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_suspension
/// @{

/// Solid axle suspension constructed with data from file.
class CH_VEHICLE_API SolidAxle : public ChSolidAxle {
  public:
    SolidAxle(const std::string& filename);
    SolidAxle(const rapidjson::Document& d);
    ~SolidAxle();

    virtual double getCamberAngle() const override { return m_camber_angle; }
    virtual double getToeAngle() const override { return m_toe_angle; }

    virtual double getAxleTubeMass() const override { return m_axleTubeMass; }
    virtual double getSpindleMass() const override { return m_spindleMass; }
    virtual double getULMass() const override { return m_ULMass; }
    virtual double getLLMass() const override { return m_LLMass; }
    virtual double getKnuckleMass() const override { return m_knuckleMass; }
    virtual double getTierodMass() const override { return m_tierodMass; }
    virtual double getDraglinkMass() const override { return m_draglinkMass; }
    virtual double getBellCrankMass() const override { return m_bellCrankMass; }

    virtual double getAxleTubeRadius() const override { return m_axleTubeRadius; }
    virtual double getSpindleRadius() const override { return m_spindleRadius; }
    virtual double getSpindleWidth() const override { return m_spindleWidth; }
    virtual double getULRadius() const override { return m_ULRadius; }
    virtual double getLLRadius() const override { return m_LLRadius; }
    virtual double getKnuckleRadius() const override { return m_knuckleRadius; }
    virtual double getTierodRadius() const override { return m_tierodRadius; }
    virtual double getDraglinkRadius() const override { return m_draglinkRadius; }
    virtual double getBellCrankRadius() const override { return m_bellCrankRadius; }

    virtual const ChVector<>& getAxleTubeInertia() const override { return m_axleTubeInertia; }
    virtual const ChVector<>& getSpindleInertia() const override { return m_spindleInertia; }
    virtual const ChVector<>& getULInertia() const override { return m_ULInertia; }
    virtual const ChVector<>& getLLInertia() const override { return m_LLInertia; }
    virtual const ChVector<>& getKnuckleInertia() const override { return m_knuckleInertia; }
    virtual const ChVector<>& getTierodInertia() const override { return m_tierodInertia; }
    virtual const ChVector<>& getDraglinkInertia() const override { return m_draglinkInertia; }
    virtual const ChVector<>& getBellCrankInertia() const override { return m_bellCrankInertia; }

    virtual double getAxleInertia() const override { return m_axleInertia; }

    virtual double getSpringRestLength() const override { return m_springRestLength; }
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getSpringForceFunctor() const override { return m_springForceCB; }
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getShockForceFunctor() const override { return m_shockForceCB; }

    virtual const ChVector<> getAxleTubeCOM() const override { return m_axleTubeCOM; }

  private:
    virtual const ChVector<> getLocation(PointId which) override { return m_points[which]; }

    virtual void Create(const rapidjson::Document& d) override;

    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_springForceCB;
    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_shockForceCB;

    ChVector<> m_points[NUM_POINTS];

    double m_camber_angle;
    double m_toe_angle;

    double m_axleTubeMass;
    double m_spindleMass;
    double m_ULMass;
    double m_LLMass;
    double m_knuckleMass;
    double m_tierodMass;
    double m_draglinkMass;
    double m_bellCrankMass;

    double m_axleTubeRadius;
    double m_spindleRadius;
    double m_spindleWidth;
    double m_ULRadius;
    double m_LLRadius;
    double m_knuckleRadius;
    double m_tierodRadius;
    double m_draglinkRadius;
    double m_bellCrankRadius;

    ChVector<> m_axleTubeCOM;

    ChVector<> m_axleTubeInertia;
    ChVector<> m_spindleInertia;
    ChVector<> m_ULInertia;
    ChVector<> m_LLInertia;
    ChVector<> m_knuckleInertia;
    ChVector<> m_tierodInertia;
    ChVector<> m_draglinkInertia;
    ChVector<> m_bellCrankInertia;

    double m_axleInertia;

    double m_springRestLength;
};

/// @} vehicle_wheeled_suspension

}  // end namespace vehicle
}  // end namespace chrono

#endif
