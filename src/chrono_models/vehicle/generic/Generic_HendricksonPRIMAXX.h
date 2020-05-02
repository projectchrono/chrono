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
// Generic concrete Hendrickson PRIMAXX suspension subsystem.
//
// This concrete suspension subsystem is defined with respect to a right-handed
// frame with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChHendricksonPRIMAXX) and origin in the chassis midplane.
//
// All point locations are provided for the left half of the suspension.
//
// =============================================================================

#ifndef GENERIC_HENDRICKSON_PRIMAXX_H
#define GENERIC_HENDRICKSON_PRIMAXX_H

#include "chrono_vehicle/wheeled_vehicle/suspension/ChHendricksonPRIMAXX.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace generic {

/// @addtogroup vehicle_models_generic
/// @{

/// Hendrickson PRIMAXX suspension for a generic vehicle.
class CH_MODELS_API Generic_HendricksonPRIMAXX : public ChHendricksonPRIMAXX {
  public:
    Generic_HendricksonPRIMAXX(const std::string& name);
    ~Generic_HendricksonPRIMAXX();

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
    virtual const ChVector<> getLocation(PointId which) override;
    virtual const ChVector<> getDirection(DirectionId which) override;

    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_shockAHForceCB;
    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_shockLBForceCB;

    static const double m_axlehousingMass;
    static const double m_knuckleMass;
    static const double m_spindleMass;
    static const double m_torquerodMass;
    static const double m_lowerbeamMass;
    static const double m_transversebeamMass;

    static const double m_axlehousingRadius;
    static const double m_knuckleRadius;
    static const double m_spindleRadius;
    static const double m_spindleWidth;
    static const double m_torquerodRadius;
    static const double m_lowerbeamRadius;
    static const double m_transversebeamRadius;

    static const ChVector<> m_axlehousingCOM;
    static const ChVector<> m_transversebeamCOM;

    static const ChVector<> m_axlehousingInertia;
    static const ChVector<> m_knuckleInertia;
    static const ChVector<> m_spindleInertia;
    static const ChVector<> m_torquerodInertia;
    static const ChVector<> m_lowerbeamInertia;
    static const ChVector<> m_transversebeamInertia;

    static const double m_axleInertia;

    static const double m_shockAH_springCoefficient;
    static const double m_shockAH_dampingCoefficient;
    static const double m_shockAH_restLength;

    static const double m_shockLB_springCoefficient;
    static const double m_shockLB_dampingCoefficient;
    static const double m_shockLB_restLength;
};

/// @} vehicle_models_generic

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono

#endif
