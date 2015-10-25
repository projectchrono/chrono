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
// M113 road wheel subsystem.
//
// =============================================================================

#ifndef M113_DOUBLE_IDLER_H
#define M113_DOUBLE_IDLER_H

#include <string>

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/tracked_vehicle/road_wheel/ChDoubleRoadWheel.h"

namespace m113 {

///
///
///
class M113_RoadWheel : public chrono::vehicle::ChDoubleRoadWheel {
  public:
    M113_RoadWheel(chrono::vehicle::VisualizationType vis_type);
    ~M113_RoadWheel() {}

    /// Return the mass of the idler wheel body.
    virtual double GetWheelMass() const override { return m_wheel_mass; }
    /// Return the moments of inertia of the idler wheel body.
    virtual const chrono::ChVector<>& GetWheelInertia() override { return m_wheel_inertia; }
    /// Return the radius of the idler wheel.
    virtual double GetWheelRadius() const override { return m_wheel_radius; }
    /// Return the total width of the idler wheel.
    virtual double GetWheelWidth() const override { return m_wheel_width; }
    /// Return the gap width.
    virtual double GetWheelGap() const override { return m_wheel_gap; }

    /// Initialize this road wheel subsystem.
    virtual void Initialize(chrono::ChSharedPtr<chrono::ChBodyAuxRef> chassis,  ///< [in] handle to the chassis body
                            chrono::ChSharedPtr<chrono::ChBody> carrier,        ///< [in] handle to the carrier body
                            const chrono::ChVector<>& location  ///< [in] location relative to the chassis frame
                            ) override;

  private:
    static const double m_wheel_mass;
    static const chrono::ChVector<> m_wheel_inertia;
    static const double m_wheel_radius;
    static const double m_wheel_width;
    static const double m_wheel_gap;

    chrono::vehicle::VisualizationType m_vis_type;
    static const std::string m_meshName;
    static const std::string m_meshFile;
};

}  // end namespace m113

#endif
