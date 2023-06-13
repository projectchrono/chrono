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
// Authors: Alessandro Tasora, Radu Serban, Antonio Recuero
// =============================================================================
//
// Toroidal tire based on Reissner shell elements
// This is a customizable Reissner tire class which uses a toroidal tire mesh
// composed of single-layer Reissner shell elements.
//
// =============================================================================

#ifndef REISSNER_TOROIDAL_TIRE_H
#define REISSNER_TOROIDAL_TIRE_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChReissnerTire.h"
#include "chrono/fea/ChElementShellReissner4.h"

namespace chrono {
namespace vehicle {

class CH_VEHICLE_API ReissnerToroidalTire : public ChReissnerTire {
  public:
    ReissnerToroidalTire(const std::string& name);
    ~ReissnerToroidalTire() {}

    virtual double GetRadius() const override { return m_rim_radius + m_height; }
    virtual double GetRimRadius() const override { return m_rim_radius; }
    virtual double GetWidth() const override { return 2 * m_height; }
    virtual double GetDefaultPressure() const override { return m_default_pressure; }
    virtual std::vector<std::shared_ptr<fea::ChNodeFEAbase>> GetConnectedNodes() const override;

    void SetRimRadius(double rimRadius_) { m_rim_radius = rimRadius_; }
    void SetHeight(double height_) { m_height = height_; }
    void SetThickness(double thickness_) { m_thickness = thickness_; }

    void SetDivCircumference(int divcirc_) { m_div_circumference = divcirc_; }
    void SetDivWidth(int divwidth_) { m_div_width = divwidth_; }

    void SetContactMaterial(std::shared_ptr<ChMaterialSurfaceSMC> mat) { m_mat = mat; }

    void SetDefaultPressure(double pressure_) { m_default_pressure = pressure_; }
    void SetAlpha(double alpha_) { m_alpha = alpha_; }
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
