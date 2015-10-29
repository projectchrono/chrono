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
// M113 sprocket subsystem.
//
// =============================================================================

#ifndef M113_SPROCKET_H
#define M113_SPROCKET_H

#include <string>

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/tracked_vehicle/ChSprocket.h"

namespace m113 {

///
///
///
class M113_Sprocket : public chrono::vehicle::ChSprocket {
  public:
    virtual ~M113_Sprocket() {}

    /// Get the number of teeth of the gear.
    virtual int GetNumTeeth() const override { return m_num_teeth; }

    /// Get the radius of the gear.
    /// This quantity is used during the automatic track assembly.
    virtual double GetAssemblyRadius() const override { return m_gear_RA; }

    /// Return the mass of the gear body.
    virtual double GetGearMass() const override { return m_gear_mass; }
    /// Return the moments of inertia of the gear body.
    virtual const chrono::ChVector<>& GetGearInertia() override { return m_gear_inertia; }
    /// Return the inertia of the axle shaft.
    virtual double GetAxleInertia() const override { return m_axle_inertia; }
    /// Return the distance between the two gear profiles.
    virtual double GetSeparation() const override { return m_separation; }

    /// Return the 2D gear profile.
    /// The gear profile, a ChLinePath geometric object, is made up of an arbitrary number
    /// of sub-paths of type ChLineArc or ChLineSegment sub-lines. These must be added in
    /// clockwise order, and the end of sub-path i must be coincident with beginning of
    /// sub-path i+1.
    virtual chrono::ChSharedPtr<chrono::geometry::ChLinePath> GetProfile() override;

    /// Add visualization of the road wheel.
    virtual void AddGearVisualization() override;

    /// Export the gear mesh Wavefront OBJ as a POV-Ray mesh macro.
    void ExportMeshPovray(const std::string& out_dir);

  protected:
    M113_Sprocket(const std::string& name, chrono::vehicle::VisualizationType vis_type);

    virtual const std::string& GetMeshName() const = 0;
    virtual const std::string& GetMeshFile() const = 0;

    static const int m_num_teeth;

    static const double m_gear_mass;
    static const chrono::ChVector<> m_gear_inertia;
    static const double m_axle_inertia;
    static const double m_separation;

    // Gear profile data
    static const double m_gear_RT;
    static const double m_gear_RC;
    static const double m_gear_R;
    static const double m_gear_RA;

    chrono::vehicle::VisualizationType m_vis_type;
};

class M113_SprocketLeft : public M113_Sprocket {
  public:
    M113_SprocketLeft(chrono::vehicle::VisualizationType visType) : M113_Sprocket("M113_SprocketLeft", visType) {}
    ~M113_SprocketLeft() {}

    virtual const std::string& GetMeshName() const override { return m_meshName; }
    virtual const std::string& GetMeshFile() const override { return m_meshFile; }

  private:
    static const std::string m_meshName;
    static const std::string m_meshFile;
};

class M113_SprocketRight : public M113_Sprocket {
  public:
    M113_SprocketRight(chrono::vehicle::VisualizationType visType) : M113_Sprocket("M113_SprocketRight", visType) {}
    ~M113_SprocketRight() {}

    virtual const std::string& GetMeshName() const override { return m_meshName; }
    virtual const std::string& GetMeshFile() const override { return m_meshFile; }

  private:
    static const std::string m_meshName;
    static const std::string m_meshFile;
};

}  // end namespace m113

#endif
