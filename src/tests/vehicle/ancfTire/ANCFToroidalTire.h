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
// Sample ANCF toroidal tire.
// This is a concrete ANCF tire class which uses a semi-toroidal tire mesh
// composed of single-layer ANCF shell elements.
//
// =============================================================================

#ifndef ANCF_TOROIDAL_TIRE_H
#define ANCF_TOROIDAL_TIRE_H

#include "chrono_vehicle/wheeled_vehicle/tire/ChANCFTire.h"

class ANCFToroidalTire : public chrono::vehicle::ChANCFTire {
  public:
    ANCFToroidalTire(const std::string& name);
    ~ANCFToroidalTire() {}

    virtual double GetTireRadius() const override { return m_rim_radius + m_height; }
    virtual double GetRimRadius() const override { return m_rim_radius; }
    virtual double GetWidth() const override { return 2 * m_height; }
    virtual double GetDefaultPressure() const override { return m_default_pressure; }
    virtual chrono::vehicle::NodeList GetConnectedNodes(const std::shared_ptr<chrono::fea::ChMesh>& mesh) const override;
    virtual void CreateMesh(std::shared_ptr<chrono::fea::ChMesh> mesh,
                            const chrono::ChFrameMoving<>& wheel_frame,
                            chrono::vehicle::VehicleSide side) override;

  private:
    static const double m_rim_radius;
    static const double m_height;
    static const double m_thickness;

    static const int m_div_circumference;
    static const int m_div_width;

    static const double m_default_pressure;
    static const double m_alpha;
};

#endif
