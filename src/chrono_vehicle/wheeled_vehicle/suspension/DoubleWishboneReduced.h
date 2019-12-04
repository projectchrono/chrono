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
// Reduced double-A arm suspension constructed with data from file.
//
// =============================================================================

#ifndef DOUBLEWISHBONEREDUCED_H
#define DOUBLEWISHBONEREDUCED_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ChDoubleWishboneReduced.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_suspension
/// @{

/// Reduced double-A arm suspension constructed with data from file.
class CH_VEHICLE_API DoubleWishboneReduced : public ChDoubleWishboneReduced {
  public:
    DoubleWishboneReduced(const std::string& filename);
    DoubleWishboneReduced(const rapidjson::Document& d);
    ~DoubleWishboneReduced();

    virtual double getSpindleMass() const override { return m_spindleMass; }
    virtual double getUprightMass() const override { return m_uprightMass; }

    virtual double getSpindleRadius() const override { return m_spindleRadius; }
    virtual double getSpindleWidth() const override { return m_spindleWidth; }
    virtual double getUprightRadius() const override { return m_uprightRadius; }

    virtual const ChVector<>& getSpindleInertia() const override { return m_spindleInertia; }
    virtual const ChVector<>& getUprightInertia() const override { return m_uprightInertia; }

    virtual double getAxleInertia() const override { return m_axleInertia; }

    virtual double getSpringRestLength() const override { return m_springRestLength; }
    virtual ChLinkTSDA::ForceFunctor* getShockForceFunctor() const override { return m_shockForceCB; }

  private:
    virtual const ChVector<> getLocation(PointId which) override { return m_points[which]; }

    virtual void Create(const rapidjson::Document& d) override;

    ChLinkTSDA::ForceFunctor* m_shockForceCB;

    ChVector<> m_points[NUM_POINTS];

    double m_spindleMass;
    double m_uprightMass;

    double m_spindleRadius;
    double m_spindleWidth;
    double m_uprightRadius;

    ChVector<> m_spindleInertia;
    ChVector<> m_uprightInertia;

    double m_axleInertia;

    double m_springRestLength;
};

/// @} vehicle_wheeled_suspension

}  // end namespace vehicle
}  // end namespace chrono

#endif
