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
// Generic concrete MacPherson strut subsystem.
//
// This concrete suspension subsystem is defined with respect to a right-handed
// frame with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChMacPhersonStrut) and origin in the chassis midplane.
//
// All point locations are provided for the left half of the suspension.
//
// =============================================================================

#ifndef GENERIC_MACPHERSONSTRUT_H
#define GENERIC_MACPHERSONSTRUT_H

#include "chrono_vehicle/wheeled_vehicle/suspension/ChMacPhersonStrut.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace generic {

/// @addtogroup vehicle_models_generic
/// @{

/// MacPherson strut suspension for a generic vehicle.
class CH_MODELS_API Generic_MacPhersonStrut : public ChMacPhersonStrut {
  public:
    // Constructor takes as argument the name of the subsystem instance.
    Generic_MacPhersonStrut(const std::string& name);

    // Destructor
    ~Generic_MacPhersonStrut();

    // Implementation of virtual methods imposed by the base class ChMacPhersonStrut

    virtual const ChVector<> getLocation(PointId which) override;

    virtual double getCamberAngle() const override { return 0; }
    virtual double getToeAngle() const override { return 0; }

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
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getSpringForceFunctor() const override { return m_springForceCB; }
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getShockForceFunctor() const override { return m_shockForceCB; }

  private:
    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_springForceCB;
    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_shockForceCB;

    static const double m_spindleMass;
    static const double m_uprightMass;
    static const double m_strutMass;
    static const double m_LCAMass;

    static const double m_spindleRadius;
    static const double m_spindleWidth;
    static const double m_uprightRadius;
    static const double m_strutRadius;
    static const double m_LCARadius;

    static const ChVector<> m_spindleInertia;
    static const ChVector<> m_strutInertia;
    static const ChVector<> m_LCAInertia;
    static const ChVector<> m_uprightInertia;

    static const double m_axleInertia;

    static const double m_springCoefficient;
    static const double m_dampingCoefficient;
    static const double m_springRestLength;
};

/// @} vehicle_models_generic

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono

#endif
