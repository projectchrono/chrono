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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// UAZBUS leafspring axle with SAE three-link model.
//
// =============================================================================

#ifndef UAZBUS_SAELEAFSPRING_AXLE_H
#define UAZBUS_SAELEAFSPRING_AXLE_H

#include "chrono_vehicle/wheeled_vehicle/suspension/ChSAELeafspringAxle.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace uaz {

/// @addtogroup vehicle_models_uaz
/// @{

/// Leafspring axle subsystem for the uaz vehicle.

class CH_MODELS_API UAZBUS_SAELeafspringAxle : public ChSAELeafspringAxle {
  public:
    UAZBUS_SAELeafspringAxle(const std::string& name);
    ~UAZBUS_SAELeafspringAxle();

  protected:
    virtual const ChVector<> getLocation(PointId which) override;

    virtual double getCamberAngle() const override { return 0; }
    virtual double getToeAngle() const override { return 0; }

    virtual double getAxleTubeMass() const override { return m_axleTubeMass; }
    virtual double getSpindleMass() const override { return m_spindleMass; }
    virtual double getFrontLeafMass() const override { return m_frontleafMass; }
    virtual double getRearLeafMass() const override { return m_rearleafMass; }
    virtual double getClampMass() const override { return m_clampMass; }
    virtual double getShackleMass() const override { return m_clampMass; }

    virtual double getAxleTubeRadius() const override { return m_axleTubeRadius; }
    virtual double getSpindleRadius() const override { return m_spindleRadius; }
    virtual double getSpindleWidth() const override { return m_spindleWidth; }

    virtual const ChVector<> getAxleTubeCOM() const override { return ChVector<>(0, 0, 0); }

    virtual const ChVector<>& getAxleTubeInertia() const override { return m_axleTubeInertia; }
    virtual const ChVector<>& getSpindleInertia() const override { return m_spindleInertia; }
    virtual const ChVector<>& getFrontLeafInertia() const override { return m_frontleafInertia; }
    virtual const ChVector<>& getRearLeafInertia() const override { return m_rearleafInertia; }
    virtual const ChVector<>& getClampInertia() const override { return m_clampInertia; }
    virtual const ChVector<>& getShackleInertia() const override { return m_shackleInertia; }

    virtual double getAxleInertia() const override { return m_axleShaftInertia; }

    virtual double getSpringRestLength() const override { return m_auxSpringRestLength; }
    /// Return the functor object for spring force.
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getSpringForceFunctor() const override {
        return m_auxSpringForceCB;
    }
    /// Return the functor object for shock force.
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getShockForceFunctor() const override { return m_shockForceCB; }

    virtual std::shared_ptr<ChLinkRSDA::TorqueFunctor> getLatTorqueFunctorA() const override {
        return m_latRotSpringCBA;
    }
    virtual std::shared_ptr<ChLinkRSDA::TorqueFunctor> getLatTorqueFunctorB() const override {
        return m_latRotSpringCBB;
    }
 
    virtual std::shared_ptr<ChLinkRSDA::TorqueFunctor> getVertTorqueFunctorA() const override {
        return m_vertRotSpringCBA;
    }
    virtual std::shared_ptr<ChLinkRSDA::TorqueFunctor> getVertTorqueFunctorB() const override {
        return m_vertRotSpringCBB;
    }

  private:
    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_auxSpringForceCB;
    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_shockForceCB;

    std::shared_ptr<ChLinkRSDA::TorqueFunctor> m_latRotSpringCBA;
    std::shared_ptr<ChLinkRSDA::TorqueFunctor> m_latRotSpringCBB;

    std::shared_ptr<ChLinkRSDA::TorqueFunctor> m_vertRotSpringCBA;
    std::shared_ptr<ChLinkRSDA::TorqueFunctor> m_vertRotSpringCBB;

    static const double m_axleShaftInertia;

    static const double m_axleTubeMass;
    static const double m_spindleMass;
    static const double m_frontleafMass;
    static const double m_rearleafMass;
    static const double m_clampMass;
    static const double m_shackleMass;

    static const double m_axleTubeRadius;
    static const double m_spindleRadius;
    static const double m_spindleWidth;

    static const ChVector<> m_axleTubeInertia;
    static const ChVector<> m_spindleInertia;
    static const ChVector<> m_frontleafInertia;
    static const ChVector<> m_rearleafInertia;
    static const ChVector<> m_clampInertia;
    static const ChVector<> m_shackleInertia;

    static const double m_auxSpringCoefficient;
    static const double m_auxSpringRestLength;
    static const double m_auxSpringDesignLength;
    static const double m_auxSpringMinLength;
    static const double m_auxSpringMaxLength;

    static const double m_damperCoefficient;
    static const double m_damperDegressivityExpansion;
    static const double m_damperDegressivityCompression;

    static const double m_leafHeight;
    static const double m_leafWidth;

    static const double m_vert_spring_trans_A;
    static const double m_vert_spring_trans_B;

    static const double m_lat_spring_trans_A;
    static const double m_lat_spring_trans_B;

    static const double m_vert_preload;
};

/// @} vehicle_models_uaz

}  // end namespace uaz
}  // end namespace vehicle
}  // end namespace chrono

#endif
