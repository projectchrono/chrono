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
// Gator concrete single wishbone suspension subsystem.
//
// This concrete suspension subsystem is defined with respect to a right-handed
// frame with X pointing towards the front, Y to the left, and Z up.
//
// All point locations are provided for the left half of the suspension.
//
// =============================================================================

#ifndef GATOR_SINGLEWISHBONE_H
#define GATOR_SINGLEWISHBONE_H

#include "chrono_vehicle/wheeled_vehicle/suspension/ChSingleWishbone.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace gator {

/// @addtogroup vehicle_models_gator
/// @{

/// Single wishbone suspension model for the Gator vehicle (front).
class CH_MODELS_API Gator_SingleWishbone : public ChSingleWishbone {
  public:
    Gator_SingleWishbone(const std::string& name);

    // Destructor
    ~Gator_SingleWishbone();

    virtual double getSpindleMass() const override { return m_spindleMass; }
    virtual double getCAMass() const override { return m_CAMass; }
    virtual double getUprightMass() const override { return m_uprightMass; }

    virtual double getSpindleRadius() const override { return m_spindleRadius; }
    virtual double getSpindleWidth() const override { return m_spindleWidth; }
    virtual double getCARadius() const override { return m_CARadius; }
    virtual double getUprightRadius() const override { return m_uprightRadius; }

    virtual const ChVector<>& getSpindleInertia() const override { return m_spindleInertia; }
    virtual const ChVector<>& getCAInertiaMoments() const override { return m_CAInertiaMoments; }
    virtual const ChVector<>& getCAInertiaProducts() const override { return m_CAInertiaProducts; }
    virtual const ChVector<>& getUprightInertiaMoments() const override { return m_uprightInertiaMoments; }
    virtual const ChVector<>& getUprightInertiaProducts() const override { return m_uprightInertiaProducts; }

    virtual double getAxleInertia() const override { return m_axleInertia; }

    virtual double getSpringRestLength() const override { return m_springRestLength; }
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getShockForceFunctor() const override { return m_shockForceCB; }

  private:
    virtual const ChVector<> getLocation(PointId which) override;

    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_shockForceCB;

    static const double m_spindleMass;
    static const double m_uprightMass;
    static const double m_CAMass;

    static const double m_spindleRadius;
    static const double m_spindleWidth;
    static const double m_uprightRadius;
    static const double m_CARadius;

    static const ChVector<> m_spindleInertia;
    static const ChVector<> m_CAInertiaMoments;
    static const ChVector<> m_CAInertiaProducts;
    static const ChVector<> m_uprightInertiaMoments;
    static const ChVector<> m_uprightInertiaProducts;

    static const double m_axleInertia;

    static const double m_springCoefficient;
    static const double m_dampingCoefficient;
    static const double m_springRestLength;
};

/// @} vehicle_models_gator

}  // end namespace gator
}  // end namespace vehicle
}  // end namespace chrono

#endif
