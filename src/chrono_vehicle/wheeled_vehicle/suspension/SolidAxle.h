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

#include "thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

class CH_VEHICLE_API SolidAxle : public ChSolidAxle {
  public:
    SolidAxle(const std::string& filename);
    SolidAxle(const rapidjson::Document& d);
    ~SolidAxle();

    virtual double getAxleTubeMass() const override { return m_axleTubeMass; }
    virtual double getSpindleMass() const override { return m_spindleMass; }
    virtual double getULMass() const override { return m_ULMass; }
    virtual double getLLMass() const override { return m_LLMass; }
    virtual double getKnuckleMass() const override { return m_knuckleMass; }

    virtual double getAxleTubeRadius() const override { return m_axleTubeRadius; }
    virtual double getSpindleRadius() const override { return m_spindleRadius; }
    virtual double getSpindleWidth() const override { return m_spindleWidth; }
    virtual double getULRadius() const override { return m_ULRadius; }
    virtual double getLLRadius() const override { return m_LLRadius; }
    virtual double getKnuckleRadius() const override { return m_knuckleRadius; }

    virtual const ChVector<>& getAxleTubeInertia() const override { return m_axleTubeInertia; }
    virtual const ChVector<>& getSpindleInertia() const override { return m_spindleInertia; }
    virtual const ChVector<>& getULInertia() const override { return m_ULInertia; }
    virtual const ChVector<>& getLLInertia() const override { return m_LLInertia; }
    virtual const ChVector<>& getKnuckleInertia() const override { return m_knuckleInertia; }

    virtual double getAxleInertia() const override { return m_axleInertia; }

    virtual double getSpringRestLength() const override { return m_springRestLength; }
    virtual ChSpringForceCallback* getSpringForceCallback() const override { return m_springForceCB; }
    virtual ChSpringForceCallback* getShockForceCallback() const override { return m_shockForceCB; }

    virtual const ChVector<> getAxleTubeCOM() const override { return m_axleTubeCOM; }

  private:
    virtual const ChVector<> getLocation(PointId which) override { return m_points[which]; }

    void Create(const rapidjson::Document& d);

    ChSpringForceCallback* m_springForceCB;
    ChSpringForceCallback* m_shockForceCB;

    ChVector<> m_points[NUM_POINTS];

    double m_axleTubeMass;
    double m_spindleMass;
    double m_ULMass;
    double m_LLMass;
    double m_knuckleMass;

    double m_axleTubeRadius;
    double m_spindleRadius;
    double m_spindleWidth;
    double m_ULRadius;
    double m_LLRadius;
    double m_knuckleRadius;

    ChVector<> m_axleTubeCOM;

    ChVector<> m_axleTubeInertia;
    ChVector<> m_spindleInertia;
    ChVector<> m_ULInertia;
    ChVector<> m_LLInertia;
    ChVector<> m_knuckleInertia;

    double m_axleInertia;

    double m_springRestLength;
};

}  // end namespace vehicle
}  // end namespace chrono

#endif
