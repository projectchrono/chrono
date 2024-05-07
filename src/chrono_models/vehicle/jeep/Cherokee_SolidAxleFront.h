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
// Authors: Radu Serban, Asher Elmquist, Rainer Gericke
// =============================================================================
//
// Classes for modeling the front suspension of Jeep Cherokee 1997
// Vehicle Parameters taken from SAE Paper 1999-01-0121
// (including the vehicle itself, the powertrain, and the tires).
// The steering mechanism is slightly different from the SAE Paper, since
// the chrono class ChSolidAxle has a bellcrank and the Jeep Cherokee has not.
//
// =============================================================================

#ifndef CHRONO_CHEROKEE_SOLIDAXLEFRONT_H
#define CHRONO_CHEROKEE_SOLIDAXLEFRONT_H

#include "chrono_vehicle/wheeled_vehicle/suspension/ChSolidAxle.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace jeep {

/// @addtogroup vehicle_models_cherokee
/// @{
class CH_MODELS_API Cherokee_SolidAxleFront : public ChSolidAxle {
  public:
    Cherokee_SolidAxleFront(const std::string& name);
    ~Cherokee_SolidAxleFront();

  protected:
    virtual const ChVector3d getLocation(PointId which) override;

    virtual double getAxleTubeMass() const override { return m_axleTubeMass; }
    virtual double getSpindleMass() const override { return m_spindleMass; }
    virtual double getTrackbarMass() const override { return m_trackbarMass; }
    virtual double getULMass() const override { return m_ulMass; }
    virtual double getLLMass() const override { return m_llMass; }
    virtual double getKnuckleMass() const override { return m_knuckleMass; }
    virtual double getTierodMass() const override { return m_tierodMass; }
    virtual double getDraglinkMass() const override { return m_dragLinkMass; }
    virtual double getBellCrankMass() const override { return m_bellCrankMass; }

    virtual double getAxleTubeRadius() const override { return m_axleTubeRadius; }
    virtual double getSpindleRadius() const override { return m_spindleRadius; }
    virtual double getSpindleWidth() const override { return m_spindleWidth; }
    virtual double getTrackbarRadius() const override { return m_trackbarRadius; }
    virtual double getULRadius() const override { return m_ulRadius; }
    virtual double getLLRadius() const override { return m_llRadius; }
    virtual double getKnuckleRadius() const override { return m_knuckleRadius; };
    virtual double getTierodRadius() const override { return m_tierodRadius; }
    virtual double getDraglinkRadius() const override { return m_dragLinkRadius; }
    virtual double getBellCrankRadius() const override { return m_bellCrankRadius; }

    virtual const ChVector3d getAxleTubeCOM() const override { return ChVector3d(0, 0, 0); }

    virtual const ChVector3d& getAxleTubeInertia() const override { return m_axleTubeInertia; }
    virtual const ChVector3d& getSpindleInertia() const override { return m_spindleInertia; }
    virtual const ChVector3d& getTrackbarInertia() const override { return m_trackbarInertia; }
    virtual const ChVector3d& getULInertia() const override { return m_ulInertia; }
    virtual const ChVector3d& getLLInertia() const override { return m_llInertia; }
    virtual const ChVector3d& getKnuckleInertia() const override { return m_knuckleInertia; };
    virtual const ChVector3d& getTierodInertia() const override { return m_tierodInertia; }
    virtual const ChVector3d& getDraglinkInertia() const override { return m_dragLinkInertia; }
    virtual const ChVector3d& getBellCrankInertia() const override { return m_bellCrankInertia; }

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
    static const double m_trackbarMass;
    static const double m_ulMass;
    static const double m_llMass;
    static const double m_knuckleMass;
    static const double m_tierodMass;
    static const double m_dragLinkMass;
    static const double m_bellCrankMass;

    static const double m_axleTubeRadius;
    static const double m_spindleRadius;
    static const double m_spindleWidth;
    static const double m_trackbarRadius;
    static const double m_ulRadius;
    static const double m_llRadius;
    static const double m_knuckleRadius;
    static const double m_tierodRadius;
    static const double m_dragLinkRadius;
    static const double m_bellCrankRadius;

    static const ChVector3d m_axleTubeInertia;
    static const ChVector3d m_spindleInertia;
    static const ChVector3d m_trackbarInertia;
    static const ChVector3d m_ulInertia;
    static const ChVector3d m_llInertia;
    static const ChVector3d m_knuckleInertia;
    static const ChVector3d m_tierodInertia;
    static const ChVector3d m_dragLinkInertia;
    static const ChVector3d m_bellCrankInertia;

    static const double m_springPreload;
    static const double m_springCoefficient;
    static const double m_springRestLength;
    static const double m_springDesignLength;
    static const double m_springMinLength;
    static const double m_springMaxLength;

    static const double m_damperCoefficientExpansion;
    static const double m_damperCoefficientCompression;
    static const double m_damperDegressivityExpansion;
    static const double m_damperDegressivityCompression;
};

/// @} vehicle_models_cherokee

}  // end namespace jeep
}  // end namespace vehicle
}  // end namespace chrono

#endif  // CHRONO_CHEROKEE_SOLIDAXLEFRONT_H
