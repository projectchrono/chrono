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
// 2nd rear Leafspring axle subsystem for the MTV vehicle.
//
// =============================================================================

#ifndef MTV_LEAFSPRING_AXLE2_H
#define MTV_LEAFSPRING_AXLE2_H

#include "chrono_vehicle/wheeled_vehicle/suspension/ChLeafspringAxle.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace fmtv {

/// @addtogroup vehicle_models_fmtv
/// @{

/// 2nd rear Leafspring axle subsystem for the MTV vehicle.
class CH_MODELS_API MTV_LeafspringAxle2 : public ChLeafspringAxle {
  public:
    MTV_LeafspringAxle2(const std::string& name);
    ~MTV_LeafspringAxle2() {}

  protected:
    virtual const ChVector3d getLocation(PointId which) override;

    virtual double getAxleTubeMass() const override { return m_axleTubeMass; }
    virtual double getSpindleMass() const override { return m_spindleMass; }

    virtual double getAxleTubeRadius() const override { return m_axleTubeRadius; }
    virtual double getSpindleRadius() const override { return m_spindleRadius; }
    virtual double getSpindleWidth() const override { return m_spindleWidth; }

    virtual const ChVector3d getAxleTubeCOM() const override { return ChVector3d(0, 0, 0); }

    virtual const ChVector3d& getAxleTubeInertia() const override { return m_axleTubeInertia; }
    virtual const ChVector3d& getSpindleInertia() const override { return m_spindleInertia; }

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

    static const double m_axleTubeRadius;
    static const double m_spindleRadius;
    static const double m_spindleWidth;

    static const ChVector3d m_axleTubeInertia;
    static const ChVector3d m_spindleInertia;

    static const double m_springCoefficient;
    static const double m_springRestLength;
    static const double m_springDesignLength;
    static const double m_springMinLength;
    static const double m_springMaxLength;

    static const double m_damperCoefficient;
    static const double m_damperDegressivityExpansion;
    static const double m_damperDegressivityCompression;
};

/// @} vehicle_models_fmtv

}  // namespace fmtv
}  // end namespace vehicle
}  // end namespace chrono

#endif
