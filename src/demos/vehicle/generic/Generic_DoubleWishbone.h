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
// Generic concrete double wishbone suspension subsystem.
//
// This concrete suspension subsystem is defined with respect to a right-handed
// frame with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChDoubleWishbone) and origin in the chassis midplane.
//
// All point locations are provided for the left half of the supspension.
//
// =============================================================================

#ifndef GENERIC_DOUBLEWISHBONE_H
#define GENERIC_DOUBLEWISHBONE_H

#include "chrono_vehicle/wheeled_vehicle/suspension/ChDoubleWishbone.h"

class Generic_DoubleWishbone : public chrono::vehicle::ChDoubleWishbone {
  public:
    // Constructor takes as argument the name of the subsystem instance.
    Generic_DoubleWishbone(const std::string& name);

    // Destructor
    ~Generic_DoubleWishbone();

    // Implementation of virtual methods imposed by the base class ChDoubleWishbone

    virtual const chrono::ChVector<> getLocation(PointId which) override;

    virtual double getSpindleMass() const override { return m_spindleMass; }
    virtual double getUCAMass() const override { return m_UCAMass; }
    virtual double getLCAMass() const override { return m_LCAMass; }
    virtual double getUprightMass() const override { return m_uprightMass; }

    virtual double getSpindleRadius() const override { return m_spindleRadius; }
    virtual double getSpindleWidth() const override { return m_spindleWidth; }
    virtual double getUCARadius() const override { return m_UCARadius; }
    virtual double getLCARadius() const override { return m_LCARadius; }
    virtual double getUprightRadius() const override { return m_uprightRadius; }

    virtual const chrono::ChVector<>& getSpindleInertia() const override { return m_spindleInertia; }
    virtual const chrono::ChVector<>& getUCAInertia() const override { return m_UCAInertia; }
    virtual const chrono::ChVector<>& getLCAInertia() const override { return m_LCAInertia; }
    virtual const chrono::ChVector<>& getUprightInertia() const override { return m_uprightInertia; }

    virtual double getAxleInertia() const override { return m_axleInertia; }

    virtual double getSpringRestLength() const override { return m_springRestLength; }
    virtual chrono::ChSpringForceCallback* getSpringForceCallback() const override { return m_springForceCB; }
    virtual chrono::ChSpringForceCallback* getShockForceCallback() const override { return m_shockForceCB; }

  private:
    chrono::ChSpringForceCallback* m_springForceCB;
    chrono::ChSpringForceCallback* m_shockForceCB;

    static const double m_spindleMass;
    static const double m_uprightMass;
    static const double m_UCAMass;
    static const double m_LCAMass;

    static const double m_spindleRadius;
    static const double m_spindleWidth;
    static const double m_uprightRadius;
    static const double m_UCARadius;
    static const double m_LCARadius;

    static const chrono::ChVector<> m_spindleInertia;
    static const chrono::ChVector<> m_UCAInertia;
    static const chrono::ChVector<> m_LCAInertia;
    static const chrono::ChVector<> m_uprightInertia;

    static const double m_axleInertia;

    static const double m_springCoefficient;
    static const double m_dampingCoefficient;
    static const double m_springRestLength;
};

#endif
