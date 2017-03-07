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
// Semi-trailing arm suspension constructed with data from file.
//
// =============================================================================

#ifndef SEMITRAILINGARM_H
#define SEMITRAILINGARM_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ChSemiTrailingArm.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_suspension
/// @{

/// Semi-trailing arm suspension constructed with data from file.
class CH_VEHICLE_API SemiTrailingArm : public ChSemiTrailingArm {
  public:
    SemiTrailingArm(const std::string& filename);
    SemiTrailingArm(const rapidjson::Document& d);
    ~SemiTrailingArm();

    virtual double getSpindleMass() const override { return m_spindleMass; }
    virtual double getArmMass() const override { return m_armMass; }

    virtual double getSpindleRadius() const override { return m_spindleRadius; }
    virtual double getSpindleWidth() const override { return m_spindleWidth; }
    virtual double getArmRadius() const override { return m_armRadius; }

    virtual const ChVector<>& getSpindleInertia() const override { return m_spindleInertia; }
    virtual const ChVector<>& getArmInertia() const override { return m_armInertia; }

    virtual double getAxleInertia() const override { return m_axleInertia; }

    virtual double getSpringRestLength() const override { return m_springRestLength; }
    virtual ChSpringForceCallback* getSpringForceCallback() const override { return m_springForceCB; }
    virtual ChSpringForceCallback* getShockForceCallback() const override { return m_shockForceCB; }

  private:
    virtual const ChVector<> getLocation(PointId which) override { return m_points[which]; }

    void Create(const rapidjson::Document& d);

    ChSpringForceCallback* m_springForceCB;
    ChSpringForceCallback* m_shockForceCB;

    ChVector<> m_points[NUM_POINTS];

    double m_spindleMass;
    double m_armMass;

    double m_spindleRadius;
    double m_spindleWidth;
    double m_armRadius;

    ChVector<> m_spindleInertia;
    ChVector<> m_armInertia;

    double m_axleInertia;

    double m_springRestLength;
};

/// @} vehicle_wheeled_suspension

}  // end namespace vehicle
}  // end namespace chrono

#endif
