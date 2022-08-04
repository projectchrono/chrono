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
// Base class for a torsion-bar suspension system using linear dampers (template
// definition).
//
// =============================================================================

#ifndef CH_TRANSLATIONAL_DAMPER_SUSPENSION_H
#define CH_TRANSLATIONAL_DAMPER_SUSPENSION_H

#include "chrono/physics/ChLinkRSDA.h"
#include "chrono/physics/ChLinkTSDA.h"
#include "chrono/physics/ChLinkForce.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"

#include "chrono_vehicle/tracked_vehicle/ChTrackSuspension.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_suspension
/// @{

/// Base class for a torsion-bar suspension system using linear dampers (template definition).
class CH_VEHICLE_API ChTranslationalDamperSuspension : public ChTrackSuspension {
  public:
    ChTranslationalDamperSuspension(const std::string& name,  ///< [in] name of the subsystem
                                    bool has_shock = true,  ///< [in] specify whether or not the suspension has a damper
                                    bool lock_arm = false   ///< [in] if true, the suspension arm is locked
    );

    virtual ~ChTranslationalDamperSuspension();

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "TranslationalDamperSuspension"; }

    /// Get a handle to the carrier body.
    virtual std::shared_ptr<ChBody> GetCarrierBody() const override { return m_arm; }

    /// Return the current pitch angle of the carrier body.
    /// This angle is measured in the x-z transversal plane, from the initial configuration,
    /// and follows the right-hand rule.
    virtual double GetCarrierAngle() const override;

    /// Initialize this suspension subsystem.
    /// The suspension subsystem is initialized by attaching it to the specified
    /// chassis body at the specified location (with respect to and expressed in
    /// the reference frame of the chassis). It is assumed that the suspension
    /// reference frame is always centered at the location of the road wheel and
    /// aligned with the chassis reference frame.
    virtual void Initialize(std::shared_ptr<ChChassis> chassis,  ///< [in] associated chassis subsystem
                            const ChVector<>& location,          ///< [in] location relative to the chassis frame
                            ChTrackAssembly* track               ///< [in] containing track assembly
                            ) override;

    /// Return current suspension forces or torques, as appropriate (spring and shock).
    /// A ChTranslationalDamperSuspension returns a torque for the spring and a force for the damper.
    virtual ForceTorque ReportSuspensionForce() const override;

    /// Add visualization assets for the suspension subsystem.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove visualization assets for the suspension subsystem.
    virtual void RemoveVisualizationAssets() override final;

    /// Log current constraint violations.
    void LogConstraintViolations() override;

  protected:
    /// Identifiers for the various hardpoints.
    enum PointId {
        ARM,          ///< arm location
        ARM_WHEEL,    ///< arm, connection point to road wheel
        ARM_CHASSIS,  ///< arm, connection point to chassis
        SHOCK_A,      ///< shock, arm location
        SHOCK_C,      ///< shock, chassis location
        NUM_POINTS
    };

    virtual void InitializeInertiaProperties() override;
    virtual void UpdateInertiaProperties() override;

    /// Return the location of the specified hardpoint.
    /// The returned location must be expressed in the idler subsystem reference frame.
    virtual const ChVector<> GetLocation(PointId which) = 0;

    /// Return the mass of the arm body.
    virtual double GetArmMass() const = 0;
    /// Return the moments of inertia of the arm body.
    virtual const ChVector<>& GetArmInertia() const = 0;
    /// Return a visualization radius for the arm body.
    virtual double GetArmVisRadius() const = 0;

    /// Return the free (rest) angle of the spring element.
    virtual double GetSpringRestAngle() const = 0;
    /// Return the functor object for the torsional spring torque.
    virtual std::shared_ptr<ChLinkRSDA::TorqueFunctor> GetSpringTorqueFunctor() const = 0;
    /// Return the functor object for the (optional) linear rotational damper.
    virtual std::shared_ptr<ChLinkRSDA::TorqueFunctor> GetDamperTorqueFunctor() const { return nullptr; }
    /// Return the functor object for the translational shock force.
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> GetShockForceFunctor() const = 0;

    /// Return stiffness and damping data for the arm bushing.
    /// Returning nullptr (default) results in using a kinematic revolute joint.
    virtual std::shared_ptr<ChVehicleBushingData> getArmBushingData() const { return nullptr; }

    virtual void ExportComponentList(rapidjson::Document& jsonDocument) const override;

    virtual void Output(ChVehicleOutput& database) const override;

    std::shared_ptr<ChBody> m_arm;            ///< trailing arm body
    std::shared_ptr<ChVehicleJoint> m_joint;  ///< joint arm-chassis
    std::shared_ptr<ChLinkRSDA> m_spring;     ///< rotational spring
    std::shared_ptr<ChLinkRSDA> m_damper;     ///< optional rotational damper
    std::shared_ptr<ChLinkTSDA> m_shock;      ///< translational shock element

  private:
    // Points for arm visualization
    ChVector<> m_pO;
    ChVector<> m_pA;
    ChVector<> m_pAC;
    ChVector<> m_pAW;
    ChVector<> m_pAS;
    ChVector<> m_dY;
};

/// @} vehicle_tracked_suspension

}  // end namespace vehicle
}  // end namespace chrono

#endif
