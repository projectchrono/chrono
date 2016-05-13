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
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// Front and Rear HMMWV suspension subsystems (reduced double A-arm)
//
// These concrete suspension subsystems are defined with respect to right-handed
// frames with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChDoubleWishboneReduced) and origins at the midpoint
// between the lower control arms' connection points to the chassis.
//
// All point locations are provided for the left half of the supspension.
//
// =============================================================================

#ifndef HMMWV9_DOUBLEWISHBONE_REDUCED_H
#define HMMWV9_DOUBLEWISHBONE_REDUCED_H

#include "chrono_vehicle/wheeled_vehicle/suspension/ChDoubleWishboneReduced.h"

#include "models/ChApiModels.h"

namespace hmmwv {

class CH_MODELS_API HMMWV_DoubleWishboneReducedFront : public chrono::vehicle::ChDoubleWishboneReduced {
  public:
    HMMWV_DoubleWishboneReducedFront(const std::string& name);
    ~HMMWV_DoubleWishboneReducedFront();

    virtual double getSpindleMass() const override { return m_spindleMass; }
    virtual double getUprightMass() const override { return m_uprightMass; }

    virtual double getSpindleRadius() const override { return m_spindleRadius; }
    virtual double getSpindleWidth() const override { return m_spindleWidth; }
    virtual double getUprightRadius() const override { return m_uprightRadius; }

    virtual const chrono::ChVector<>& getSpindleInertia() const override { return m_spindleInertia; }
    virtual const chrono::ChVector<>& getUprightInertia() const override { return m_uprightInertia; }

    virtual double getAxleInertia() const override { return m_axleInertia; }

    virtual double getSpringRestLength() const override { return m_springRestLength; }
    virtual chrono::ChSpringForceCallback* getShockForceCallback() const override { return m_shockForceCB; }

  private:
    virtual const chrono::ChVector<> getLocation(PointId which) override;

    chrono::ChSpringForceCallback* m_shockForceCB;

    static const double m_spindleMass;
    static const double m_uprightMass;

    static const double m_spindleRadius;
    static const double m_spindleWidth;
    static const double m_uprightRadius;

    static const chrono::ChVector<> m_spindleInertia;
    static const chrono::ChVector<> m_uprightInertia;

    static const double m_axleInertia;

    static const double m_springCoefficient;
    static const double m_dampingCoefficient;
    static const double m_springRestLength;
};

// -----------------------------------------------------------------------------

class CH_MODELS_API HMMWV_DoubleWishboneReducedRear : public chrono::vehicle::ChDoubleWishboneReduced {
  public:
    HMMWV_DoubleWishboneReducedRear(const std::string& name);
    ~HMMWV_DoubleWishboneReducedRear();

    virtual double getSpindleMass() const override { return m_spindleMass; }
    virtual double getUprightMass() const override { return m_uprightMass; }

    virtual double getSpindleRadius() const override { return m_spindleRadius; }
    virtual double getSpindleWidth() const override { return m_spindleWidth; }
    virtual double getUprightRadius() const override { return m_uprightRadius; }

    virtual const chrono::ChVector<>& getSpindleInertia() const override { return m_spindleInertia; }
    virtual const chrono::ChVector<>& getUprightInertia() const override { return m_uprightInertia; }

    virtual double getAxleInertia() const override { return m_axleInertia; }

    virtual double getSpringRestLength() const override { return m_springRestLength; }
    virtual chrono::ChSpringForceCallback* getShockForceCallback() const override { return m_shockForceCB; }

  private:
    virtual const chrono::ChVector<> getLocation(PointId which) override;

    chrono::ChSpringForceCallback* m_shockForceCB;

    static const double m_spindleMass;
    static const double m_uprightMass;

    static const double m_spindleRadius;
    static const double m_spindleWidth;
    static const double m_uprightRadius;

    static const chrono::ChVector<> m_spindleInertia;
    static const chrono::ChVector<> m_uprightInertia;

    static const double m_axleInertia;

    static const double m_springCoefficient;
    static const double m_dampingCoefficient;
    static const double m_springRestLength;
};

}  // end namespace hmmwv

#endif
