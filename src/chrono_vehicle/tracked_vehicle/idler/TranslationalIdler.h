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
// Translational idler model constructed with data from file (JSON format).
//
// =============================================================================

#ifndef TRANSLATIONAL_IDLER_H
#define TRANSLATIONAL_IDLER_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/tracked_vehicle/idler/ChTranslationalIdler.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_idler
/// @{

/// Translational idler model constructed with data from file (JSON format).
class CH_VEHICLE_API TranslationalIdler : public ChTranslationalIdler {
  public:
    TranslationalIdler(const std::string& filename);
    TranslationalIdler(const rapidjson::Document& d);
    ~TranslationalIdler() {}

    virtual double GetCarrierMass() const override { return m_carrier_mass; }
    virtual const ChVector<>& GetCarrierInertia() override { return m_carrier_inertia; }
    virtual double GetCarrierVisRadius() const override { return m_carrier_vis_radius; }

    virtual double GetPrismaticPitchAngle() const override { return m_pitch_angle; }

    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> GetTensionerForceCallback() const override { return m_tensionerForceCB; }
    virtual double GetTensionerFreeLength() const override { return m_tensioner_l0; }

  private:
    virtual const ChVector<> GetLocation(PointId which) override { return m_points[which]; }

    virtual void Create(const rapidjson::Document& d) override;

    ChVector<> m_points[NUM_POINTS];

    double m_carrier_mass;
    ChVector<> m_carrier_inertia;

    double m_carrier_vis_radius;

    double m_pitch_angle;

    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_tensionerForceCB;
    double m_tensioner_l0;
};

/// @} vehicle_tracked_idler

}  // end namespace vehicle
}  // end namespace chrono

#endif
