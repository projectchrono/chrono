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
// CityBus leafspring axle.
//
// =============================================================================

#ifndef CityBus_LEAFSPRING_AXLE_H
#define CityBus_LEAFSPRING_AXLE_H

#include "chrono_vehicle/wheeled_vehicle/suspension/ChLeafspringAxle.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace citybus {


class CH_MODELS_API CityBus_LeafspringAxle : public ChLeafspringAxle {
  public:
    CityBus_LeafspringAxle(const std::string& name);
    ~CityBus_LeafspringAxle();

  
  protected:
    virtual const ChVector<> getLocation(PointId which) override;

    virtual double getAxleTubeMass() const override { return m_axleTubeMass; }
    virtual double getSpindleMass() const override { return m_spindleMass; }
 
    virtual double getAxleTubeRadius() const override { return m_axleTubeRadius; }
    virtual double getSpindleRadius() const override { return m_spindleRadius; }
    virtual double getSpindleWidth() const override { return m_spindleWidth; }
    
    virtual const ChVector<> getAxleTubeCOM() const override { return ChVector<>(0,0,0); }

    virtual const ChVector<>& getAxleTubeInertia() const override { return m_axleTubeInertia; }
    virtual const ChVector<>& getSpindleInertia() const override { return m_spindleInertia; }

    virtual double getAxleInertia() const override { return m_axleShaftInertia; }
    
    virtual double getSpringRestLength() const override { return m_springRestLength; }
    /// Return the functor object for spring force.
    virtual ChLinkSpringCB::ForceFunctor* getSpringForceFunctor() const override { return m_springForceCB; }
    /// Return the functor object for shock force.
    virtual ChLinkSpringCB::ForceFunctor* getShockForceFunctor() const override { return m_shockForceCB; }

  private:
    ChLinkSpringCB::ForceFunctor* m_springForceCB;
    ChLinkSpringCB::ForceFunctor* m_shockForceCB;

    static const double m_axleShaftInertia;
    
    static const double m_axleTubeMass;
    static const double m_spindleMass;
 
    static const double m_axleTubeRadius;
    static const double m_spindleRadius;
    static const double m_spindleWidth;
 
    static const ChVector<> m_axleTubeInertia;
    static const ChVector<> m_spindleInertia;
    
    static const double m_springCoefficient;
    static const double m_springRestLength;
    static const double m_springDesignLength;
    static const double m_springMinLength;
    static const double m_springMaxLength;

    static const double m_damperCoefficient;
    static const double m_damperDegressivityExpansion;
    static const double m_damperDegressivityCompression;
};

}  // end namespace citybus
}  // end namespace vehicle
}  // end namespace chrono

#endif
