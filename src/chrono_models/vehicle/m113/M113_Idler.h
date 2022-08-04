// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2022 projectchrono.org
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
// M113 idler subsystem.
//
// =============================================================================

#ifndef M113_IDLER_H
#define M113_IDLER_H

#include <string>

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/tracked_vehicle/idler/ChTranslationalIdler.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace m113 {

/// @addtogroup vehicle_models_m113
/// @{

/// Idler and tensioner model for the M113 vehicle.
class CH_MODELS_API M113_Idler : public ChTranslationalIdler {
  public:
    M113_Idler(const std::string& name, VehicleSide side);
    ~M113_Idler() {}

    /// Return the location of the specified hardpoint.
    /// The returned location must be expressed in the idler subsystem reference frame.
    virtual const ChVector<> GetLocation(PointId which) override;

    /// Return the mass of the carrier body.
    virtual double GetCarrierMass() const override { return m_carrier_mass; }
    /// Return the moments of inertia of the carrier body.
    virtual const ChVector<>& GetCarrierInertia() override { return m_carrier_inertia; }
    /// Return a visualization radius for the carrier body.
    virtual double GetCarrierVisRadius() const override { return m_carrier_radius; }

    /// Return the pitch angle of the prismatic joint.
    virtual double GetPrismaticPitchAngle() const override { return 0; }

    /// Return the functor object for spring force.
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> GetTensionerForceCallback() const override {
        return m_tensionerForceCB;
    }

    /// Return the free length for the tensioner spring.
    virtual double GetTensionerFreeLength() const override { return m_tensioner_l0; }

  private:
    VehicleSide m_side;

    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_tensionerForceCB;

    static const double m_carrier_mass;
    static const ChVector<> m_carrier_inertia;
    static const double m_carrier_radius;

    static const double m_tensioner_l0;
    static const double m_tensioner_k;
    static const double m_tensioner_c;
    static const double m_tensioner_f;
};

/// @} vehicle_models_m113

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono

#endif
