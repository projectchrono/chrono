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
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// Base class for a double-A arm suspension modeled with distance constraints.
// Derived from ChSuspension, but still an abstract bas class.
//
// The suspension subsystem is modeled with respect to a right-handed frame,
// with X pointing towards the front, Y to the left, and Z up (ISO standard).
// The suspension reference frame is assumed to be always aligned with that of
// the vehicle.  When attached to a chassis, only an offset is provided.
//
// All point locations are assumed to be given for the left half of the
// suspension and will be mirrored (reflecting the y coordinates) to construct
// the right side.
//
// =============================================================================

#ifndef CH_DOUBLEWISHBONEREDUCED_H
#define CH_DOUBLEWISHBONEREDUCED_H

#include <vector>

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/ChSuspension.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_suspension
/// @{

/// Base class for a double-A arm suspension modeled with distance constraints.
/// Derived from ChSuspension, but still an abstract bas class.
///
/// The suspension subsystem is modeled with respect to a right-handed frame,
/// with X pointing towards the front, Y to the left, and Z up (ISO standard).
/// The suspension reference frame is assumed to be always aligned with that of
/// the vehicle.  When attached to a chassis, only an offset is provided.
///
/// All point locations are assumed to be given for the left half of the
/// suspension and will be mirrored (reflecting the y coordinates) to construct
/// the right side.
class CH_VEHICLE_API ChDoubleWishboneReduced : public ChSuspension {
  public:
    ChDoubleWishboneReduced(const std::string& name  ///< [in] name of the subsystem
                            );

    virtual ~ChDoubleWishboneReduced();

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "DoubleWishboneReduced"; }

    /// Specify whether or not this suspension can be steered.
    virtual bool IsSteerable() const final override { return true; }

    /// Specify whether or not this is an independent suspension.
    virtual bool IsIndependent() const final override { return true; }

    /// Initialize this suspension subsystem.
    /// The suspension subsystem is initialized by attaching it to the specified chassis and (if provided) to the
    /// specified subchassis, at the specified location (with respect to and expressed in the reference frame of the
    /// chassis). It is assumed that the suspension reference frame is always aligned with the chassis reference frame.
    /// If a steering subsystem is provided, the suspension tierods are to be attached to the steering's central link
    /// body (steered suspension); otherwise they are to be attached to the chassis (non-steered suspension).
    virtual void Initialize(
        std::shared_ptr<ChChassis> chassis,        ///< [in] associated chassis subsystem
        std::shared_ptr<ChSubchassis> subchassis,  ///< [in] associated subchassis subsystem (may be null)
        std::shared_ptr<ChSteering> steering,      ///< [in] associated steering subsystem (may be null)
        const ChVector<>& location,                ///< [in] location relative to the chassis frame
        double left_ang_vel = 0,                   ///< [in] initial angular velocity of left wheel
        double right_ang_vel = 0                   ///< [in] initial angular velocity of right wheel
        ) override;

    /// Add visualization assets for the suspension subsystem.
    /// This default implementation uses primitives.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove visualization assets for the suspension subsystem.
    virtual void RemoveVisualizationAssets() override;

    /// Get the wheel track for the suspension subsystem.
    virtual double GetTrack() override;

    /// Get a handle to the specified shock (spring-damper) element.
    std::shared_ptr<ChLinkTSDA> GetShock(VehicleSide side) const { return m_shock[side]; }

    /// Return current suspension forces (spring and shock) on the specified side.
    /// Since this suspension type has a combined spring-damper element, the same information is duplicated in the
    /// "spring" and "shock" members of the return struct.
    virtual ChSuspension::Force ReportSuspensionForce(VehicleSide side) const override;

    /// Specify the suspension body on the specified side to attach a possible antirollbar subsystem.
    /// Return the corresponding upright body.
    virtual std::shared_ptr<ChBody> GetAntirollBody(VehicleSide side) const override { return m_upright[side]; }

    /// Log current constraint violations.
    virtual void LogConstraintViolations(VehicleSide side) override;

  protected:
    /// Identifiers for the various hardpoints.
    enum PointId {
        SPINDLE,   ///< spindle location
        UPRIGHT,   ///< upright location
        UCA_F,     ///< upper control arm, chassis front
        UCA_B,     ///< upper control arm, chassis back
        UCA_U,     ///< upper control arm, upright
        LCA_F,     ///< lower control arm, chassis front
        LCA_B,     ///< lower control arm, chassis back
        LCA_U,     ///< lower control arm, upright
        SHOCK_C,   ///< shock, chassis
        SHOCK_U,   ///< shock, upright
        TIEROD_C,  ///< tierod, chassis
        TIEROD_U,  ///< tierod, upright
        NUM_POINTS
    };

    virtual void InitializeInertiaProperties() override;
    virtual void UpdateInertiaProperties() override;

    /// Return the location of the specified hardpoint.
    /// The returned location must be expressed in the suspension reference frame.
    virtual const ChVector<> getLocation(PointId which) = 0;

    /// Return the camber angle, in radians (default: 0).
    virtual double getCamberAngle() const { return 0; }

    /// Return the toe angle, in radians (default: 0).
    /// A positive value indicates toe-in, a negative value indicates toe-out.
    virtual double getToeAngle() const { return 0; }

    /// Return the mass of the spindle body.
    virtual double getSpindleMass() const = 0;
    /// Return the mass of the upright body.
    virtual double getUprightMass() const = 0;

    /// Return the moments of inertia of the spindle body.
    virtual const ChVector<>& getSpindleInertia() const = 0;
    /// Return the moments of inertia of the upright body.
    virtual const ChVector<>& getUprightInertia() const = 0;

    /// Return the inertia of the axle shaft.
    virtual double getAxleInertia() const = 0;

    /// Return the radius of the upright body (visualization only).
    virtual double getUprightRadius() const = 0;

    /// Return the free (rest) length of the spring-damper element.
    virtual double getSpringRestLength() const = 0;
    /// Return the functor object for shock force (spring-damper).
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getShockForceFunctor() const = 0;

    std::shared_ptr<ChBody> m_upright[2];  ///< handles to the upright bodies (left/right)

    std::shared_ptr<ChLinkDistance> m_distUCA_F[2];   ///< handles to the front UCA distance constraints (left/right)
    std::shared_ptr<ChLinkDistance> m_distUCA_B[2];   ///< handles to the back UCA distance constraints (left/right)
    std::shared_ptr<ChLinkDistance> m_distLCA_F[2];   ///< handles to the front LCA distance constraints (left/right)
    std::shared_ptr<ChLinkDistance> m_distLCA_B[2];   ///< handles to the back LCA distance constraints (left/right)
    std::shared_ptr<ChLinkDistance> m_distTierod[2];  ///< handles to the tierod distance constraints (left/right)

    std::shared_ptr<ChLinkTSDA> m_shock[2];  ///< handles to the spring-damper force elements (left/right)

  private:
    // Hardpoint absolute locations
    std::vector<ChVector<>> m_pointsL;
    std::vector<ChVector<>> m_pointsR;

    void InitializeSide(VehicleSide side,
                        std::shared_ptr<ChBodyAuxRef> chassis,
                        std::shared_ptr<ChBody> tierod_body,
                        const std::vector<ChVector<>>& points,
                        double ang_vel);

    static void AddVisualizationUpright(std::shared_ptr<ChBody> upright,
                                        const ChVector<> pt_C,
                                        const ChVector<> pt_U,
                                        const ChVector<> pt_L,
                                        const ChVector<> pt_T,
                                        double radius);

    virtual void ExportComponentList(rapidjson::Document& jsonDocument) const override;

    virtual void Output(ChVehicleOutput& database) const override;
};

/// @} vehicle_wheeled_suspension

}  // end namespace vehicle
}  // end namespace chrono

#endif
