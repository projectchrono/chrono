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
// Authors: Radu Serban, Justin Madsen, Daniel Melanz, Rainer Gericke
// =============================================================================
//
// Rear BMW E90 suspension subsystems (double A-arm), replacement for multilink
// Vehicle Parameters taken from SAE Paper 2007-01-0818
//
// These concrete suspension subsystems are defined with respect to right-handed
// frames with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChDoubleWishbone) and origins at the midpoint between the
// lower control arms' connection points to the chassis.
//
// All point locations are provided for the left half of the suspension.
//
// =============================================================================

#ifndef BMW_DOUBLEWISHBONE_H
#define BMW_DOUBLEWISHBONE_H

#include "chrono_vehicle/wheeled_vehicle/suspension/ChDoubleWishbone.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace bmw {

/// @addtogroup vehicle_models_bmw
/// @{

// -----------------------------------------------------------------------------

/// Full double wishbone rear suspension for the BMW E90 vehicle.
/// The control arms are modeled using rigid bodies.
class CH_MODELS_API BMW_E90_DoubleWishbone : public ChDoubleWishbone {
  public:
    BMW_E90_DoubleWishbone(const std::string& name, bool use_tierod_bodies);
    ~BMW_E90_DoubleWishbone();

    virtual bool UseTierodBodies() const override { return m_use_tierod_bodies; }

    virtual double getCamberAngle() const override { return 0; }
    virtual double getToeAngle() const override { return 0; }

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

    virtual const ChVector3d& getSpindleInertia() const override { return m_spindleInertia; }
    virtual const ChVector3d& getUCAInertiaMoments() const override { return m_UCAInertiaMoments; }
    virtual const ChVector3d& getUCAInertiaProducts() const override { return m_UCAInertiaProducts; }
    virtual const ChVector3d& getLCAInertiaMoments() const override { return m_LCAInertiaMoments; }
    virtual const ChVector3d& getLCAInertiaProducts() const override { return m_LCAInertiaProducts; }
    virtual const ChVector3d& getUprightInertiaMoments() const override { return m_uprightInertiaMoments; }
    virtual const ChVector3d& getUprightInertiaProducts() const override { return m_uprightInertiaProducts; }
    virtual const ChVector3d getTierodInertia() const override { return m_tierodInertia; }

    virtual double getAxleInertia() const override { return m_axleInertia; }

    virtual double getSpringRestLength() const override { return m_springRestLength; }
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getSpringForceFunctor() const override { return m_springForceCB; }
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getShockForceFunctor() const override { return m_shockForceCB; }

  private:
    virtual const ChVector3d getLocation(PointId which) override;

    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_springForceCB;
    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_shockForceCB;

    bool m_use_tierod_bodies;

    static const double m_spindleMass;
    static const double m_UCAMass;
    static const double m_LCAMass;
    static const double m_uprightMass;
    static const double m_tierodMass;

    static const double m_spindleRadius;
    static const double m_spindleWidth;
    static const double m_UCARadius;
    static const double m_LCARadius;
    static const double m_uprightRadius;
    static const double m_tierodRadius;

    static const ChVector3d m_spindleInertia;
    static const ChVector3d m_UCAInertiaMoments;
    static const ChVector3d m_UCAInertiaProducts;
    static const ChVector3d m_LCAInertiaMoments;
    static const ChVector3d m_LCAInertiaProducts;
    static const ChVector3d m_uprightInertiaMoments;
    static const ChVector3d m_uprightInertiaProducts;
    static const ChVector3d m_tierodInertia;

    static const double m_axleInertia;

    static const double m_springConstant;
    static const double m_springPreload;
    static const double m_damperConstant;
    static const double m_springRestLength;
    static const double m_kinematicFactorSpring;
    static const double m_kinematicFactorDamper;
};

/// @} vehicle_models_bmw

}  // namespace bmw
}  // end namespace vehicle
}  // end namespace chrono

#endif
