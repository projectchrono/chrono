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
// Double idler model constructed with data from file (JSON format).
//
// =============================================================================

#ifndef DOUBLE_IDLER_H
#define DOUBLE_IDLER_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/tracked_vehicle/idler/ChDoubleIdler.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_idler
/// @{

/// Double idler model constructed with data from file (JSON format).
class CH_VEHICLE_API DoubleIdler : public ChDoubleIdler {
  public:
    DoubleIdler(const std::string& filename);
    DoubleIdler(const rapidjson::Document& d);
    ~DoubleIdler() {}

    virtual double GetWheelRadius() const override { return m_wheel_radius; }
    virtual double GetWheelWidth() const override { return m_wheel_width; }
    virtual double GetWheelGap() const override { return m_wheel_gap; }

    virtual double GetWheelMass() const override { return m_wheel_mass; }
    virtual const ChVector<>& GetWheelInertia() override { return m_wheel_inertia; }

    virtual double GetCarrierMass() const override { return m_carrier_mass; }
    virtual const ChVector<>& GetCarrierInertia() override { return m_carrier_inertia; }
    virtual double GetCarrierVisRadius() const override { return m_carrier_vis_radius; }

    virtual double GetPrismaticPitchAngle() const override { return m_pitch_angle; }

    virtual ChSpringForceCallback* GetTensionerForceCallback() const override { return m_tensionerForceCB; }
    virtual double GetTensionerFreeLength() const override { return m_tensioner_l0; }

    virtual void AddVisualizationAssets(VisualizationType vis) override;

  private:
    virtual const ChVector<> GetLocation(PointId which) override { return m_points[which]; }

    void Create(const rapidjson::Document& d);

    ChVector<> m_points[NUM_POINTS];

    double m_wheel_radius;
    double m_wheel_width;
    double m_wheel_gap;

    double m_wheel_mass;
    ChVector<> m_wheel_inertia;

    double m_carrier_mass;
    ChVector<> m_carrier_inertia;

    double m_carrier_vis_radius;

    double m_pitch_angle;

    ChSpringForceCallback* m_tensionerForceCB;
    double m_tensioner_l0;

    bool m_has_mesh;
    std::string m_meshName;
    std::string m_meshFile;
};

/// @} vehicle_tracked_idler

}  // end namespace vehicle
}  // end namespace chrono

#endif
