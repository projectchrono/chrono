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
// Mercedes G-class rear axle.
//
// =============================================================================

#ifndef G500_REAR_AXLE_H
#define G500_REAR_AXLE_H

#include "chrono_vehicle/wheeled_vehicle/suspension/ChRigidPanhardAxle.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace gclass {

/// @addtogroup vehicle_models_uaz
/// @{

/// Leafspring axle subsystem for the uaz vehicle.

class CH_MODELS_API G500_RearAxle : public ChRigidPanhardAxle {
  public:
    G500_RearAxle(const std::string& name);
    ~G500_RearAxle();

  
  protected:
    virtual const ChVector<> getLocation(PointId which) override;

    virtual double getCamberAngle() const override { return 0; }
    virtual double getToeAngle() const override { return 0; }

    virtual double getAxleTubeMass() const override { return m_axleTubeMass; }
    virtual double getSpindleMass() const override { return m_spindleMass; }
    virtual double getPanhardRodMass() const override { return m_panhardRodMass; }
    virtual double getARBMass() const override { return m_arbMass; }

    virtual double getARBStiffness() const override { return m_arb_stiffness; }
    virtual double getARBDamping() const override { return m_arb_damping; }

    virtual double getAxleTubeRadius() const override { return m_axleTubeRadius; }
    virtual double getSpindleRadius() const override { return m_spindleRadius; }
    virtual double getSpindleWidth() const override { return m_spindleWidth; }
    virtual double getPanhardRodRadius() const override { return m_panhardRodRadius; }
    virtual double getARBRadius() const override { return m_arbRadius; }

    virtual const ChVector<> getAxleTubeCOM() const override { return ChVector<>(0,0,0); }

    virtual const ChVector<>& getAxleTubeInertia() const override { return m_axleTubeInertia; }
    virtual const ChVector<>& getSpindleInertia() const override { return m_spindleInertia; }
    virtual const ChVector<>& getPanhardRodInertia() const override { return m_panhardRodInertia; }
    virtual const ChVector<>& getARBInertia() const override { return m_arbInertia; }

    virtual double getAxleInertia() const override { return m_axleShaftInertia; }
    
    virtual double getSpringRestLength() const override { return m_springRestLength; }
    /// Return the functor object for spring force.
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getSpringForceFunctor() const override { return m_springForceCB; }
    /// Return the functor object for shock force.
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getShockForceFunctor() const override { return m_shockForceCB; }

  private:
    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_springForceCB;
    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_shockForceCB;

    static const double m_axleShaftInertia;
    
    static const double m_axleTubeMass;
    static const double m_spindleMass;
    static const double m_panhardRodMass;
    static const double m_arbMass;
 
    static const double m_axleTubeRadius;
    static const double m_spindleRadius;
    static const double m_spindleWidth;
    static const double m_panhardRodRadius;
    static const double m_arbRadius;

    static const ChVector<> m_axleTubeInertia;
    static const ChVector<> m_spindleInertia;
    static const ChVector<> m_panhardRodInertia;
    static const ChVector<> m_arbInertia;

    static const double m_springCoefficient;
    static const double m_springRestLength;
    static const double m_springDesignLength;
    static const double m_springMinLength;
    static const double m_springMaxLength;

    static const double m_damperCoefficient;
    static const double m_damperDegressivityExpansion;
    static const double m_damperDegressivityCompression;
    
    static const double m_arb_stiffness;
    static const double m_arb_damping;
};

/// @} vehicle_models_uaz

}  // end namespace uaz
}  // end namespace vehicle
}  // end namespace chrono

#endif
