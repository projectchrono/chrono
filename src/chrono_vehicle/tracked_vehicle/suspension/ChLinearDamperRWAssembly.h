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
// Base class for a torsion-bar suspension system using linear dampers (template
// definition).
//
// =============================================================================

#ifndef CH_LINEAR_DAMPER_RWA_H
#define CH_LINEAR_DAMPER_RWA_H

#include "chrono/physics/ChLinkRotSpringCB.h"
#include "chrono/physics/ChLinkSpringCB.h"
#include "chrono/physics/ChLinkForce.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"

#include "chrono_vehicle/tracked_vehicle/ChRoadWheelAssembly.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_suspension
/// @{

/// Base class for a torsion-bar suspension system using linear dampers (template definition).
class CH_VEHICLE_API ChLinearDamperRWAssembly : public ChRoadWheelAssembly {
  public:
    ChLinearDamperRWAssembly(const std::string& name,  ///< [in] name of the subsystem
                             bool has_shock = true     ///< [in] specify whether or not the suspension has a damper
                             );

    virtual ~ChLinearDamperRWAssembly() {}

    /// Get a handle to the carrier body.
    virtual std::shared_ptr<ChBody> GetCarrierBody() const override { return m_arm; }

    /// Return the current pitch angle of the carrir body.
    /// This angle is measured in the x-z transversal plane, from the initial configuration,
    /// and follows the right-hand rule.
    virtual double GetCarrierAngle() const override;

    /// Get a handle to the revolute joint of the arm.
    std::shared_ptr<ChLinkLockRevolute> GetArmRevolute() const { return m_revolute; }

    /// Get the total mass of the roadwheel assembly.
    /// This includes the mass of the roadwheel and of the suspension mechanism.
    virtual double GetMass() const override;

    /// Initialize this suspension subsystem.
    /// The suspension subsystem is initialized by attaching it to the specified
    /// chassis body at the specified location (with respect to and expressed in
    /// the reference frame of the chassis). It is assumed that the suspension
    /// reference frame is always centered at the location of the road wheel and
    /// aligned with the chassis reference frame.
    virtual void Initialize(std::shared_ptr<ChBodyAuxRef> chassis,  ///< [in] handle to the chassis body
                            const ChVector<>& location              ///< [in] location relative to the chassis frame
                            ) override;

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

    /// Return the location of the specified hardpoint.
    /// The returned location must be expressed in the idler subsystem reference frame.
    virtual const ChVector<> GetLocation(PointId which) = 0;

    /// Return the mass of the arm body.
    virtual double GetArmMass() const = 0;
    /// Return the moments of inertia of the arm body.
    virtual const ChVector<>& GetArmInertia() const = 0;
    /// Return a visualization radius for the arm body.
    virtual double GetArmVisRadius() const = 0;

    /// Return the functor object for the torsional spring torque.
    virtual ChLinkRotSpringCB::TorqueFunctor* GetSpringTorqueFunctor() const = 0;

    /// Return the functor object for the translational shock force.
    virtual ChLinkSpringCB::ForceFunctor* GetShockForceFunctor() const = 0;

    bool m_has_shock;                                ///< specifies whether or not the suspension has a damper
    std::shared_ptr<ChBody> m_arm;                   ///< handle to the trailing arm body
    std::shared_ptr<ChLinkLockRevolute> m_revolute;  ///< handle to the revolute joint arm-chassis
    std::shared_ptr<ChLinkRotSpringCB> m_spring;     ///< handle to the rotational spring link
    std::shared_ptr<ChLinkSpringCB> m_shock;         ///< handle to the translational shock link

  private:
    void AddVisualizationArm(const ChVector<>& pt_O,   ///< wheel center (in global frame)
                             const ChVector<>& pt_A,   ///< arm location (in global frame)
                             const ChVector<>& pt_AW,  ///< connection to wheel (in global frame)
                             const ChVector<>& pt_AC,  ///< connection to chassis (in global frame)
                             const ChVector<>& pt_AS   ///< connection to shock absorber (in global frame)
                             );

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
