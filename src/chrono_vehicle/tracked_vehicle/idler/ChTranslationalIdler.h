// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2022 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Base class for an idler subsystem with a tensioner mechanism using a
// translational joint and TSDA.  An idler consists of the idler wheel and
// a connecting body.  The idler wheel is connected through a revolute joint to
// the connecting body which in turn is connected to the chassis through a
// translational joint. A linear actuator acts as a tensioner.
//
// An idler subsystem is defined with respect to a frame centered at the origin
// of the idler wheel, possibly pitched relative to the chassis reference frame.
// The translational joint is aligned with the x axis of this reference frame,
// while the axis of rotation of the revolute joint is aligned with its y axis.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#ifndef CH_TRANSLATIONAL_IDLER_H
#define CH_TRANSLATIONAL_IDLER_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"

#include "chrono_vehicle/tracked_vehicle/ChIdler.h"

namespace chrono {
namespace vehicle {

class ChTrackAssembly;

/// @addtogroup vehicle_tracked_idler
/// @{

/// Base class for an idler subsystem with a translational tensioner.
/// An idler consists of the idler wheel and a connecting body.  The idler wheel is connected
/// through a revolute joint to the connecting body which in turn is connected to the chassis
/// through a translational joint. A linear actuator acts as a tensioner.
class CH_VEHICLE_API ChTranslationalIdler : public ChIdler {
  public:
    ChTranslationalIdler(const std::string& name);
    virtual ~ChTranslationalIdler();

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "TranslationalIdler"; }

    /// Return a handle to the carrier body to which the idler wheel is connected.
    virtual std::shared_ptr<ChBody> GetCarrierBody() const override { return m_carrier; }
        
    /// Get the tensioner force element.
    std::shared_ptr<ChLinkTSDA> GetTensioner() const { return m_tensioner; }

    /// Initialize this idler subsystem.
    /// The idler subsystem is initialized by attaching it to the specified chassis at the specified location (with
    /// respect to and expressed in the reference frame of the chassis). It is assumed that the idler subsystem
    /// reference frame is always aligned with the chassis reference frame. A derived idler subsystem template class
    /// must extend this default implementation and specify contact geometry for the idler wheel.
    virtual void Initialize(std::shared_ptr<ChChassis> chassis,  ///< [in] associated chassis
                            const ChVector<>& location,          ///< [in] location relative to the chassis frame
                            ChTrackAssembly* track               ///< [in] containing track assembly
    ) override;

    /// Add visualization assets for the idler subsystem.
    /// This default implementation adds assets to the carrier body.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove visualization assets for the idler subsystem.
    /// This default implementation removes the assets from the carrier body.
    virtual void RemoveVisualizationAssets() override;

    /// Log current constraint violations.
    virtual void LogConstraintViolations() override;

  protected:
    /// Identifiers for the various hardpoints.
    enum PointId {
        CARRIER,          ///< carrier location
        CARRIER_WHEEL,    ///< carrier, connection to idler wheel
        CARRIER_CHASSIS,  ///< carrier, connection to chassis (translational)
        TSDA_CARRIER,     ///< TSDA connection to carrier
        TSDA_CHASSIS,     ///< TSDA connection to chassis
        NUM_POINTS
    };

    virtual void InitializeInertiaProperties() override;
    virtual void UpdateInertiaProperties() override;

    /// Return the location of the specified hardpoint.
    /// The returned location must be expressed in the idler subsystem reference frame.
    virtual const ChVector<> GetLocation(PointId which) = 0;

    /// Return the mass of the carrier body.
    virtual double GetCarrierMass() const = 0;
    /// Return the moments of inertia of the carrier body.
    virtual const ChVector<>& GetCarrierInertia() = 0;
    /// Return a visualization radius for the carrier body.
    virtual double GetCarrierVisRadius() const = 0;

    /// Return the pitch angle of the prismatic joint.
    virtual double GetPrismaticPitchAngle() const = 0;

    /// Return the functor object for spring force.
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> GetTensionerForceCallback() const = 0;

    /// Return the free length for the tensioner spring.
    virtual double GetTensionerFreeLength() const = 0;

    virtual void ExportComponentList(rapidjson::Document& jsonDocument) const override;

    virtual void Output(ChVehicleOutput& database) const override;

    std::shared_ptr<ChBody> m_carrier;                 ///< carrier body
    std::shared_ptr<ChLinkLockPrismatic> m_prismatic;  ///< carrier-chassis translational joint
    std::shared_ptr<ChLinkTSDA> m_tensioner;           ///< TSDA tensioner element

  private:
    // Points for carrier visualization
    ChVector<> m_pW;
    ChVector<> m_pC;
    ChVector<> m_pT;
};

/// @} vehicle_tracked_idler

}  // end namespace vehicle
}  // end namespace chrono

#endif
