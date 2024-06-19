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
// Authors: Radu Serban, Mike Taylor, Asher Elmquist, Rainer Gericke
// =============================================================================
//
// BMW E90 (330i 2006) concrete MacPherson Strut suspension subsystem.
// Vehicle Parameters taken from SAE Paper 2007-01-0818
//
// This concrete suspension subsystem is defined with respect to a right-handed
// frame with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChDoubleWishbone) and origin in the chassis midplane.
//
// All point locations are provided for the left half of the suspension.
//
// =============================================================================

#ifndef BMW_E90_MACPHERSON_STRUT_H
#define BMW_E90_MACPHERSON_STRUT_H

#include "chrono_vehicle/wheeled_vehicle/suspension/ChMacPhersonStrut.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace bmw {

/// @addtogroup vehicle_models_bmw
/// @{

/// MacPherson Strut suspension model for a bmw e90 vehicle (can be used in front or rear).
class CH_MODELS_API BMW_E90_MacPhersonStrut : public ChMacPhersonStrut {
  public:
    // Constructor takes as argument the name of the subsystem instance.
    BMW_E90_MacPhersonStrut(const std::string& name);

    // Destructor
    ~BMW_E90_MacPhersonStrut();

    /// Get the wheel track for the suspension subsystem.
    virtual double GetTrack() override { return m_trackWidth; }

  protected:
    // Implementation of virtual methods imposed by the base class ChMacPhersonStrut
    virtual const ChVector3d getLocation(PointId which) override;

    virtual double getCamberAngle() const override { return -2 * CH_DEG_TO_RAD; }
    virtual double getToeAngle() const override { return 0; }

    virtual double getSpindleMass() const override { return m_spindleMass; }
    virtual double getStrutMass() const override { return m_strutMass; }
    virtual double getLCAMass() const override { return m_LCAMass; }
    virtual double getUprightMass() const override { return m_uprightMass; }
    virtual double getTierodMass() const override { return m_tierodMass; }

    virtual double getSpindleRadius() const override { return m_spindleRadius; }
    virtual double getStrutRadius() const override { return m_strutRadius; }
    virtual double getSpindleWidth() const override { return m_spindleWidth; }
    virtual double getLCARadius() const override { return m_LCARadius; }
    virtual double getUprightRadius() const override { return m_uprightRadius; }

    virtual const ChVector3d& getSpindleInertia() const override { return m_spindleInertia; }
    virtual const ChVector3d& getStrutInertia() const override { return m_strutInertia; }
    virtual const ChVector3d& getLCAInertia() const override { return m_LCAInertiaMoments; }
    virtual const ChVector3d& getUprightInertia() const override { return m_uprightInertiaMoments; }
    virtual const ChVector3d getTierodInertia() const override { return m_tierodInertiaMoments; }
    /// Return the inertia of the axle shaft.
    virtual double getAxleInertia() const override { return m_axleInertia; }

    virtual double getSpringRestLength() const override { return m_springRestLength; }
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getSpringForceFunctor() const override { return m_springForceCB; }
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getShockForceFunctor() const override { return m_shockForceCB; }

    virtual bool UseTierodBodies() const override { return true; }

  private:
    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_springForceCB;
    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_shockForceCB;

    static const double m_spindleMass;
    static const double m_uprightMass;
    static const double m_LCAMass;
    static const double m_strutMass;
    static const double m_tierodMass;
    static const double m_axleInertia;

    static const double m_spindleRadius;
    static const double m_strutRadius;
    static const double m_spindleWidth;
    static const double m_uprightRadius;
    static const double m_LCARadius;

    static const double m_trackWidth;

    static const ChVector3d m_spindleInertia;
    static const ChVector3d m_strutInertia;
    static const ChVector3d m_LCAInertiaMoments;
    static const ChVector3d m_uprightInertiaMoments;
    static const ChVector3d m_tierodInertiaMoments;

    static const double m_springCoefficient;
    static const double m_dampingCoefficient;
    static const double m_springRestLength;
    static const double m_springPreload;
    static const double m_kinematicFactor;
};

/// @} vehicle_models_bmw

}  // end namespace bmw
}  // end namespace vehicle
}  // end namespace chrono

#endif
