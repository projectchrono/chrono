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

#ifndef M113_ROAD_WHEEL_H
#define M113_ROAD_WHEEL_H

#include <string>

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/tracked_vehicle/road_wheel/ChDoubleRoadWheel.h"

#include "models/ChApiModels.h"

namespace m113 {

///
///
///
class CH_MODELS_API M113_RoadWheel : public chrono::vehicle::ChDoubleRoadWheel {
  public:
    virtual ~M113_RoadWheel() {}

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

    /// Add visualization of the road wheel.
    virtual void AddWheelVisualization() override;

    /// Set the road wheel visualization type.
    void SetVisType(chrono::vehicle::VisualizationType vis) { m_vis_type = vis; }

    /// Export the wheel mesh Wavefront OBJ as a POV-Ray mesh macro.
    void ExportMeshPovray(const std::string& out_dir);

  protected:
    M113_RoadWheel(const std::string& name);

    virtual chrono::vehicle::VehicleSide GetVehicleSide() const = 0;

    virtual const std::string& GetMeshName() const = 0;
    virtual const std::string& GetMeshFile() const = 0;

    static const double m_wheel_mass;
    static const chrono::ChVector<> m_wheel_inertia;
    static const double m_wheel_radius;
    static const double m_wheel_width;
    static const double m_wheel_gap;

    chrono::vehicle::VisualizationType m_vis_type;
};

class CH_MODELS_API M113_RoadWheelLeft : public M113_RoadWheel {
  public:
    M113_RoadWheelLeft() : M113_RoadWheel("M113_RoadWheelLeft") {}
    ~M113_RoadWheelLeft() {}

    virtual chrono::vehicle::VehicleSide GetVehicleSide() const override { return chrono::vehicle::LEFT; }

    virtual const std::string& GetMeshName() const override { return m_meshName; }
    virtual const std::string& GetMeshFile() const override { return m_meshFile; }

  private:
    static const std::string m_meshName;
    static const std::string m_meshFile;
};

class CH_MODELS_API M113_RoadWheelRight : public M113_RoadWheel {
  public:
    M113_RoadWheelRight() : M113_RoadWheel("M113_RoadWheelRight") {}
    ~M113_RoadWheelRight() {}

    virtual chrono::vehicle::VehicleSide GetVehicleSide() const override { return chrono::vehicle::RIGHT; }

    virtual const std::string& GetMeshName() const override { return m_meshName; }
    virtual const std::string& GetMeshFile() const override { return m_meshFile; }

  private:
    static const std::string m_meshName;
    static const std::string m_meshFile;
};

}  // end namespace m113

#endif
