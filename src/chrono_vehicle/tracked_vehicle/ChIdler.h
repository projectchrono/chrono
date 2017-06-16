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
// Authors: Radu Serban
// =============================================================================
//
// Base class for an idler subsystem.  An idler consists of the idler wheel and
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

#ifndef CH_IDLER_H
#define CH_IDLER_H

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChLinkSpringCB.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChPart.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_idler
/// @{

/// Base class for an idler subsystem.
/// An idler consists of the idler wheel and a connecting body.  The idler wheel is connected
/// through a revolute joint to the connecting body which in turn is connected to the chassis
/// through a translational joint. A linear actuator acts as a tensioner.
class CH_VEHICLE_API ChIdler : public ChPart {
  public:
    ChIdler(const std::string& name  ///< [in] name of the subsystem
            );

    virtual ~ChIdler() {}

    /// Return the type of track shoe consistent with this idler.
    virtual GuidePinType GetType() const = 0;

    /// Get a handle to the road wheel body.
    std::shared_ptr<ChBody> GetWheelBody() const { return m_wheel; }

    /// Get a handle to the revolute joint.
    std::shared_ptr<ChLinkLockRevolute> GetRevolute() const { return m_revolute; }

    /// Get the radius of the idler wheel.
    virtual double GetWheelRadius() const = 0;

    /// Turn on/off collision flag for the idler wheel.
    void SetCollide(bool val) { m_wheel->SetCollide(val); }

    /// Get the mass of the idler subsystem.
    virtual double GetMass() const;

    /// Initialize this idler subsystem.
    /// The idler subsystem is initialized by attaching it to the specified
    /// chassis body at the specified location (with respect to and expressed in
    /// the reference frame of the chassis). It is assumed that the idler subsystem
    /// reference frame is always aligned with the chassis reference frame.
    /// A derived idler subsystem template class must extend this default implementation
    /// and specify contact geometry for the idler wheel.
    virtual void Initialize(std::shared_ptr<ChBodyAuxRef> chassis,  ///< [in] handle to the chassis body
                            const ChVector<>& location              ///< [in] location relative to the chassis frame
                            );

    /// Add visualization assets for the idler subsystem.
    /// This default implementation adds assets to the carrier body.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove visualization assets for the idler subsystem.
    /// This default implementation removes the assets from the carrier body.
    virtual void RemoveVisualizationAssets() override;

    /// Log current constraint violations.
    void LogConstraintViolations();

  protected:
    /// Identifiers for the various hardpoints.
    enum PointId {
        WHEEL,            ///< idler wheel location
        CARRIER,          ///< carrier location
        CARRIER_CHASSIS,  ///< carrier, connection to chassis (translational)
        TSDA_CARRIER,     ///< TSDA connection to carrier
        TSDA_CHASSIS,     ///< TSDA connection to chassis
        NUM_POINTS
    };

    /// Return the location of the specified hardpoint.
    /// The returned location must be expressed in the idler subsystem reference frame.
    virtual const ChVector<> GetLocation(PointId which) = 0;

    /// Return the mass of the idler wheel body.
    virtual double GetWheelMass() const = 0;
    /// Return the moments of inertia of the idler wheel body.
    virtual const ChVector<>& GetWheelInertia() = 0;

    /// Return the mass of the carrier body.
    virtual double GetCarrierMass() const = 0;
    /// Return the moments of inertia of the carrier body.
    virtual const ChVector<>& GetCarrierInertia() = 0;
    /// Return a visualization radius for the carrier body.
    virtual double GetCarrierVisRadius() const = 0;

    /// Return the pitch angle of the prismatic joint.
    virtual double GetPrismaticPitchAngle() const = 0;

    /// Return the functor object for spring force.
    virtual ChLinkSpringCB::ForceFunctor* GetTensionerForceCallback() const = 0;

    /// Return the free length for the tensioner spring.
    virtual double GetTensionerFreeLength() const = 0;

    std::shared_ptr<ChBody> m_wheel;                   ///< handle to the idler wheel body
    std::shared_ptr<ChBody> m_carrier;                 ///< handle to the carrier body
    std::shared_ptr<ChLinkLockRevolute> m_revolute;    ///< handle to wheel-carrier revolute joint
    std::shared_ptr<ChLinkLockPrismatic> m_prismatic;  ///< handle to carrier-chassis translational joint
    std::shared_ptr<ChLinkSpringCB> m_tensioner;       ///< handle to the TSDA tensioner element

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
