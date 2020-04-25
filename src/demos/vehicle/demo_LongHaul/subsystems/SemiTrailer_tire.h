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
// Authors: Rainer Gericke
// =============================================================================
//
// Krone ProfiLiner SP5 TMeasy tire subsystem 385/65R22.5 160
//
// =============================================================================

#ifndef SEMITRAILOR_TMEASY_TIRE_H
#define SEMITRAILOR_TMEASY_TIRE_H

#include "chrono_vehicle/wheeled_vehicle/tire/ChTMeasyTire.h"

#include "chrono_models/ChApiModels.h"

class SemiTrailer_tire : public chrono::vehicle::ChTMeasyTire {
  public:
    SemiTrailer_tire(const std::string& name);
    ~SemiTrailer_tire() {}

    virtual double GetVisualizationWidth() const override { return m_width; }

    virtual void SetTMeasyParams() override;
    virtual double GetMass() const override { return m_mass; }
    virtual chrono::ChVector<> GetInertia() const override { return m_inertia; }

    virtual void AddVisualizationAssets(chrono::vehicle::VisualizationType vis) override;
    virtual void RemoveVisualizationAssets() override final;

    void GenerateCharacteristicPlots(const std::string& dirname);

  private:
    static const double m_mass;
    static const chrono::ChVector<> m_inertia;
    chrono::ChFunction_Recorder m_stiffnessMap;

    static const std::string m_meshFile;
    std::shared_ptr<chrono::ChTriangleMeshShape> m_trimesh_shape;
};

#endif
