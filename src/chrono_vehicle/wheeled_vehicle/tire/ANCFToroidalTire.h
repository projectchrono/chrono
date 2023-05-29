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
// Authors: Radu Serban, Antonio Recuero
// =============================================================================
//
// ANCF toroidal tire.
// This is a customizable ANCF tire class which uses a toroidal tire mesh
// composed of single-layer 4-node ANCF shell elements.
//
// =============================================================================

#ifndef ANCF_TOROIDAL_TIRE_H
#define ANCF_TOROIDAL_TIRE_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChANCFTire.h"
#include "chrono/fea/ChElementShellANCF_3423.h"

namespace chrono {
namespace vehicle {

class CH_VEHICLE_API ANCFToroidalTire : public ChANCFTire {
  public:
    ANCFToroidalTire(const std::string& name);
    ~ANCFToroidalTire() {}

    virtual double GetRadius() const override { return m_rim_radius + m_height; }
    virtual double GetRimRadius() const override { return m_rim_radius; }
    virtual double GetWidth() const override { return 2 * m_height; }
    virtual double GetDefaultPressure() const override { return m_default_pressure; }
    virtual std::vector<std::shared_ptr<fea::ChNodeFEAbase>> GetConnectedNodes() const override;

    void SetRimRadius(double rim_radius) { m_rim_radius = rim_radius; }
    void SetHeight(double height) { m_height = height; }
    void SetThickness(double thickness) { m_thickness = thickness; }

    void SetDivCircumference(int div_circumference) { m_div_circumference = div_circumference; }
    void SetDivWidth(int div_width) { m_div_width = div_width; }

    void SetContactMaterial(std::shared_ptr<ChMaterialSurfaceSMC> mat) { m_mat = mat; }

    void SetDefaultPressure(double pressure) { m_default_pressure = pressure; }
    void SetAlpha(double alpha) { m_alpha = alpha; }
    virtual void CreateMesh(const ChFrameMoving<>& wheel_frame, VehicleSide side) override;

  private:
    virtual void CreateContactMaterial() override;

    double m_rim_radius;
    double m_height;
    double m_thickness;

    int m_div_circumference;
    int m_div_width;

    double m_default_pressure;
    double m_alpha;

    std::shared_ptr<ChMaterialSurfaceSMC> m_mat;
};

}  // end namespace vehicle
}  // end namespace chrono

#endif
