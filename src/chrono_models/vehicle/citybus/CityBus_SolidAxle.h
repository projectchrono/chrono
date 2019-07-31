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
// Authors: Daniel Melanz, Radu Serban, Evan Hoerl, Shuo He
// =============================================================================
//
// City Bus solid axle suspension subsystem.
//
// This concrete suspension subsystem is defined with respect to a right-handed
// frame with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChSolidAxle) and origin at the midpoint between the wheel
// centers.
//
// All point locations are provided for the left half of the suspension.
//
// =============================================================================

#ifndef CITYBUS_SOLIDAXLE_H
#define CITYBUS_SOLIDAXLE_H

#include "chrono_vehicle/wheeled_vehicle/suspension/ChSolidAxle.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace citybus {

/// @addtogroup vehicle_models_citybus
/// @{

/// Solid-axle suspension subsystem for the city bus vehicle.
class CH_MODELS_API CityBus_SolidAxleFront : public ChSolidAxle {
  public:
    CityBus_SolidAxleFront(const std::string& name);
    ~CityBus_SolidAxleFront();

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
    virtual ChLinkSpringCB::ForceFunctor* getSpringForceFunctor() const override { return m_springForceCB; }
    virtual ChLinkSpringCB::ForceFunctor* getShockForceFunctor() const override { return m_shockForceCB; }

    virtual const ChVector<> getAxleTubeCOM() const override { return m_axleTubeCOM; }

  private:
    virtual const ChVector<> getLocation(PointId which) override;

    ChLinkSpringCB::ForceFunctor* m_springForceCB;
    ChLinkSpringCB::ForceFunctor* m_shockForceCB;

    static const double m_axleTubeMass;
    static const double m_spindleMass;
    static const double m_ULMass;
    static const double m_LLMass;
    static const double m_knuckleMass;
    static const double m_tierodMass;
    static const double m_draglinkMass;
    static const double m_bellCrankMass;

    static const double m_axleTubeRadius;
    static const double m_spindleRadius;
    static const double m_spindleWidth;
    static const double m_ULRadius;
    static const double m_LLRadius;
    static const double m_knuckleRadius;
    static const double m_tierodRadius;
    static const double m_draglinkRadius;
    static const double m_bellCrankRadius;

    static const ChVector<> m_axleTubeCOM;

    static const ChVector<> m_axleTubeInertia;
    static const ChVector<> m_spindleInertia;
    static const ChVector<> m_ULInertia;
    static const ChVector<> m_LLInertia;
    static const ChVector<> m_knuckleInertia;
    static const ChVector<> m_tierodInertia;
    static const ChVector<> m_draglinkInertia;
    static const ChVector<> m_bellCrankInertia;

    static const double m_axleInertia;

    static const double m_springCoefficient;
    static const double m_dampingCoefficient;
    static const double m_springRestLength;
};

/// Solid-axle rear suspension subsystem for the city bus vehicle.
class CH_MODELS_API CityBus_SolidAxleRear : public ChSolidAxle {
  public:
    CityBus_SolidAxleRear(const std::string& name);
    ~CityBus_SolidAxleRear();

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
    virtual ChLinkSpringCB::ForceFunctor* getSpringForceFunctor() const override { return m_springForceCB; }
    virtual ChLinkSpringCB::ForceFunctor* getShockForceFunctor() const override { return m_shockForceCB; }

    virtual const ChVector<> getAxleTubeCOM() const override { return m_axleTubeCOM; }

  private:
    virtual const ChVector<> getLocation(PointId which) override;

    ChLinkSpringCB::ForceFunctor* m_springForceCB;
    ChLinkSpringCB::ForceFunctor* m_shockForceCB;

    static const double m_axleTubeMass;
    static const double m_spindleMass;
    static const double m_ULMass;
    static const double m_LLMass;
    static const double m_knuckleMass;
    static const double m_tierodMass;
    static const double m_draglinkMass;
    static const double m_bellCrankMass;

    static const double m_axleTubeRadius;
    static const double m_spindleRadius;
    static const double m_spindleWidth;
    static const double m_ULRadius;
    static const double m_LLRadius;
    static const double m_knuckleRadius;
    static const double m_tierodRadius;
    static const double m_draglinkRadius;
    static const double m_bellCrankRadius;

    static const ChVector<> m_axleTubeCOM;

    static const ChVector<> m_axleTubeInertia;
    static const ChVector<> m_spindleInertia;
    static const ChVector<> m_ULInertia;
    static const ChVector<> m_LLInertia;
    static const ChVector<> m_knuckleInertia;
    static const ChVector<> m_tierodInertia;
    static const ChVector<> m_draglinkInertia;
    static const ChVector<> m_bellCrankInertia;

    static const double m_axleInertia;

    static const double m_springCoefficient;
    static const double m_dampingCoefficient;
    static const double m_springRestLength;
};

/// @} vehicle_models_citybus

}  // end namespace citybus
}  // end namespace vehicle
}  // end namespace chrono

#endif
