// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2018 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Radu Serban
// =============================================================================
//
// =============================================================================

#ifndef ROBO_GRANULAR_H
#define ROBO_GRANULAR_H

#include "chrono_multicore/physics/ChSystemMulticore.h"
#include "chrono_vehicle/terrain/GranularTerrain.h"

class GroundGranular {
  public:
    GroundGranular(chrono::ChSystemMulticore* sys);

    void SetParticleProperties(double radius, double density, double friction, double cohesion);
    void SetPatchProperties(double length, double width, unsigned int num_layers);

    double GetBottomHeight() const { return m_center.z(); }
    std::pair<double, double> GetTopHeight(int num_samples = 200) const;

    unsigned int GetNumParticles() const { return m_num_particles; }

  protected:
    virtual void Initialize(double x_min, double z_max, double step_size) = 0;

    std::shared_ptr<chrono::ChMaterialSurface> m_material_c;
    std::shared_ptr<chrono::ChMaterialSurface> m_material_g;

    chrono::ChSystemMulticore* m_sys;  // associated system

    double m_radius;    // particle radius
    double m_density;   // particle material density
    double m_cohesion;  // inter-particle cohesion pressure
    double m_friction;  // inter-particle coefficient of friction

    double m_length;            // patch length
    double m_width;             // path width
    unsigned int m_num_layers;  // number of particle layers

    chrono::ChVector<> m_center;   // center of bottom boundary
    double m_radius1;              // inflated particle radius
    unsigned int m_num_particles;  // number of generated particles

    static const int m_start_id = 1000000;
};

class GroundGranularA : public GroundGranular {
  public:
    GroundGranularA(chrono::ChSystemMulticore* sys);
    virtual void Initialize(double x_min, double z_max, double step_size) override;

  private:
    std::shared_ptr<chrono::vehicle::GranularTerrain> m_terrain;
};

class GroundGranularB : public GroundGranular {
  public:
    GroundGranularB(chrono::ChSystemMulticore* sys);
    virtual void Initialize(double x_min, double z_max, double step_size) override;

  private:
    std::shared_ptr<chrono::ChBody> m_ground;
};

#endif
