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
// Marder idler subsystem.
//
// =============================================================================

#ifndef MARDER_IDLER_H
#define MARDER_IDLER_H

#include <string>

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/tracked_vehicle/idler/ChDoubleIdler.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace marder {

/// @addtogroup vehicle_models_marder
/// @{

/// Idler and tensioner model for the Marder vehicle (base class).
class CH_MODELS_API Marder_Idler : public ChDoubleIdler {
  public:
    virtual ~Marder_Idler() {}

    /// Return the location of the specified hardpoint.
    /// The returned location must be expressed in the idler subsystem reference frame.
    virtual const ChVector<> GetLocation(PointId which) override;

    /// Return the mass of the idler wheel body.
    virtual double GetWheelMass() const override { return m_wheel_mass; }
    /// Return the moments of inertia of the idler wheel body.
    virtual const ChVector<>& GetWheelInertia() override { return m_wheel_inertia; }
    /// Return the radius of the idler wheel.
    virtual double GetWheelRadius() const override { return m_wheel_radius; }
    /// Return the total width of the idler wheel.
    virtual double GetWheelWidth() const override { return m_wheel_width; }
    /// Return the gap width.
    virtual double GetWheelGap() const override { return m_wheel_gap; }

    /// Return the mass of the carrier body.
    virtual double GetCarrierMass() const override { return m_carrier_mass; }
    /// Return the moments of inertia of the carrier body.
    virtual const ChVector<>& GetCarrierInertia() override { return m_carrier_inertia; }
    /// Return a visualization radius for the carrier body.
    virtual double GetCarrierVisRadius() const override { return m_carrier_radius; }

    /// Return the pitch angle of the prismatic joint.
    virtual double GetPrismaticPitchAngle() const override { return 0; }

    /// Return the functor object for spring force.
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> GetTensionerForceCallback() const override {
        return m_tensionerForceCB;
    }

    /// Return the free length for the tensioner spring.
    virtual double GetTensionerFreeLength() const override { return m_tensioner_l0; }

  protected:
    Marder_Idler(const std::string& name);

    /// Create the contact material consistent with the specified contact method.
    virtual void CreateContactMaterial(ChContactMethod contact_method) override;

    /// Add visualization assets for the idler subsystem.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    virtual VehicleSide GetVehicleSide() const = 0;

    virtual std::string GetMeshFile() const = 0;

    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_tensionerForceCB;

    static const double m_wheel_mass;
    static const ChVector<> m_wheel_inertia;
    static const double m_wheel_radius;
    static const double m_wheel_width;
    static const double m_wheel_gap;

    static const double m_carrier_mass;
    static const ChVector<> m_carrier_inertia;
    static const double m_carrier_radius;

    static const double m_tensioner_l0;
    static const double m_tensioner_k;
    static const double m_tensioner_c;
    static const double m_tensioner_f;
};

/// Idler and tensioner model for the M113 vehicle (left side).
class CH_MODELS_API Marder_IdlerLeft : public Marder_Idler {
  public:
    Marder_IdlerLeft() : Marder_Idler("Marder_IdlerLeft") {}
    ~Marder_IdlerLeft() {}

    virtual VehicleSide GetVehicleSide() const override { return LEFT; }

    virtual std::string GetMeshFile() const override { return GetDataFile(m_meshFile); }

  private:
    static const std::string m_meshFile;
};

/// Idler and tensioner model for the M113 vehicle (right side).
class CH_MODELS_API Marder_IdlerRight : public Marder_Idler {
  public:
    Marder_IdlerRight() : Marder_Idler("Marder_IdlerRight") {}
    ~Marder_IdlerRight() {}

    virtual VehicleSide GetVehicleSide() const override { return RIGHT; }

    virtual std::string GetMeshFile() const override { return GetDataFile(m_meshFile); }

  private:
    static const std::string m_meshFile;
};

/// @} vehicle_models_marder

}  // namespace marder
}  // end namespace vehicle
}  // end namespace chrono

#endif
