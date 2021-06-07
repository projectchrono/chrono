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
// Authors: Radu Serban, Holger Haut
// =============================================================================
//
// Hendrickson PRIMAXX suspension constructed with data from file.
//
// =============================================================================

#ifndef HENDRICKSON_PRIMAXX_H
#define HENDRICKSON_PRIMAXX_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ChHendricksonPRIMAXX.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_suspension
/// @{

/// Hendrickson PRIMAXX suspension constructed with data from file.
class CH_VEHICLE_API HendricksonPRIMAXX : public ChHendricksonPRIMAXX {
  public:
    HendricksonPRIMAXX(const std::string& filename);
    HendricksonPRIMAXX(const rapidjson::Document& d);
    ~HendricksonPRIMAXX();

    virtual const ChVector<> getAxlehousingCOM() const override { return m_axlehousingCOM; }
    virtual const ChVector<> getTransversebeamCOM() const override { return m_transversebeamCOM; }

    virtual double getAxlehousingMass() const override { return m_axlehousingMass; }
    virtual double getKnuckleMass() const override { return m_knuckleMass; }
    virtual double getSpindleMass() const override { return m_spindleMass; }
    virtual double getTorquerodMass() const override { return m_torquerodMass; }
    virtual double getLowerbeamMass() const override { return m_lowerbeamMass; }
    virtual double getTransversebeamMass() const override { return m_transversebeamMass; }

    virtual double getAxlehousingRadius() const override { return m_axlehousingRadius; }
    virtual double getKnuckleRadius() const override { return m_knuckleRadius; }
    virtual double getSpindleRadius() const override { return m_spindleRadius; }
    virtual double getSpindleWidth() const override { return m_spindleWidth; }
    virtual double getTorquerodRadius() const override { return m_torquerodRadius; }
    virtual double getLowerbeamRadius() const override { return m_lowerbeamRadius; }
    virtual double getTransversebeamRadius() const override { return m_transversebeamRadius; }

    virtual const ChVector<>& getAxlehousingInertia() const override { return m_axlehousingInertia; }
    virtual const ChVector<>& getKnuckleInertia() const override { return m_knuckleInertia; }
    virtual const ChVector<>& getSpindleInertia() const override { return m_spindleInertia; }
    virtual const ChVector<>& getTorquerodInertia() const override { return m_torquerodInertia; }
    virtual const ChVector<>& getLowerbeamInertia() const override { return m_lowerbeamInertia; }
    virtual const ChVector<>& getTransversebeamInertia() const override { return m_transversebeamInertia; }

    virtual double getAxleInertia() const override { return m_axleInertia; }

    virtual double getShockAHRestLength() const override { return m_shockAH_restLength; }
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getShockAHForceCallback() const override {
        return m_shockAHForceCB;
    }

    virtual double getShockLBRestLength() const override { return m_shockLB_restLength; }
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getShockLBForceCallback() const override {
        return m_shockLBForceCB;
    }

  private:
    virtual const ChVector<> getLocation(PointId which) override { return m_points[which]; }
    virtual const ChVector<> getDirection(DirectionId which) override { return m_dirs[which]; }

    virtual void Create(const rapidjson::Document& d) override;

    ChVector<> m_points[NUM_POINTS];
    ChVector<> m_dirs[NUM_DIRS];

    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_shockAHForceCB;
    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_shockLBForceCB;

    double m_axlehousingMass;
    double m_knuckleMass;
    double m_spindleMass;
    double m_torquerodMass;
    double m_lowerbeamMass;
    double m_transversebeamMass;

    double m_axlehousingRadius;
    double m_knuckleRadius;
    double m_spindleRadius;
    double m_spindleWidth;
    double m_torquerodRadius;
    double m_lowerbeamRadius;
    double m_transversebeamRadius;

    ChVector<> m_axlehousingCOM;
    ChVector<> m_transversebeamCOM;

    ChVector<> m_axlehousingInertia;
    ChVector<> m_knuckleInertia;
    ChVector<> m_spindleInertia;
    ChVector<> m_torquerodInertia;
    ChVector<> m_lowerbeamInertia;
    ChVector<> m_transversebeamInertia;

    double m_axleInertia;

    double m_shockAH_restLength;
    double m_shockLB_restLength;
};

/// @} vehicle_wheeled_suspension

}  // end namespace vehicle
}  // end namespace chrono

#endif
