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
// M113 sprocket subsystem (single pin).
//
// =============================================================================

#ifndef M113_SPROCKET_SINGLE_PIN_H
#define M113_SPROCKET_SINGLE_PIN_H

#include <string>

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/tracked_vehicle/sprocket/ChSprocketSinglePin.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace m113 {

///
///
///
class CH_MODELS_API M113_SprocketSinglePin : public ChSprocketSinglePin {
  public:
    virtual ~M113_SprocketSinglePin() {}

    /// Get the number of teeth of the gear.
    virtual int GetNumTeeth() const override { return m_num_teeth; }

    /// Get the radius of the gear.
    /// This quantity is used during the automatic track assembly.
    virtual double GetAssemblyRadius() const override { return m_gear_RA; }

    /// Return the mass of the gear body.
    virtual double GetGearMass() const override { return m_gear_mass; }
    /// Return the moments of inertia of the gear body.
    virtual const ChVector<>& GetGearInertia() override { return m_gear_inertia; }
    /// Return the inertia of the axle shaft.
    virtual double GetAxleInertia() const override { return m_axle_inertia; }
    /// Return the distance between the two gear profiles.
    virtual double GetSeparation() const override { return m_separation; }

    /// Return the radius of the addendum circle.
    virtual double GetOuterRadius() const override { return m_gear_RT; }
    /// Return the radius of the (concave) tooth circular arc.
    virtual double GetArcRadius() const override { return m_gear_R; }
    /// Return the radius of the tooth arc centers.
    virtual double GetArcCentersRadius() const override { return m_gear_RC; }

    /// Add visualization of the sprocket.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

  protected:
    M113_SprocketSinglePin(const std::string& name);

    virtual std::string GetMeshName() const = 0;
    virtual std::string GetMeshFile() const = 0;

    static const int m_num_teeth;

    static const double m_gear_mass;
    static const ChVector<> m_gear_inertia;
    static const double m_axle_inertia;
    static const double m_separation;

    // Gear profile data
    static const double m_gear_RT;
    static const double m_gear_RC;
    static const double m_gear_R;
    static const double m_gear_RA;
};

class CH_MODELS_API M113_SprocketSinglePinLeft : public M113_SprocketSinglePin {
  public:
    M113_SprocketSinglePinLeft() : M113_SprocketSinglePin("M113_SprocketLeft") {}
    ~M113_SprocketSinglePinLeft() {}

    virtual std::string GetMeshName() const override { return m_meshName; }
    virtual std::string GetMeshFile() const override { return GetDataFile(m_meshFile); }

  private:
    static const std::string m_meshName;
    static const std::string m_meshFile;
};

class CH_MODELS_API M113_SprocketSinglePinRight : public M113_SprocketSinglePin {
  public:
    M113_SprocketSinglePinRight() : M113_SprocketSinglePin("M113_SprocketRight") {}
    ~M113_SprocketSinglePinRight() {}

    virtual std::string GetMeshName() const override { return m_meshName; }
    virtual std::string GetMeshFile() const override { return GetDataFile(m_meshFile); }

  private:
    static const std::string m_meshName;
    static const std::string m_meshFile;
};

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono

#endif
