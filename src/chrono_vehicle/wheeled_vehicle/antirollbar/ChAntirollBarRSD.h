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
// Base class for an anti-roll bar template modeled two arms connected with a
// revolute spring-damper.
// Derived from ChAntirollBar, but still and abstract base class.
//
// The anti-roll bar subsystem is modeled with respect to a right-handed frame,
// with X pointing towards the front, Y to the left, and Z up (ISO standard).
// The subsystem reference frame is assumed to be always aligned with that of
// the vehicle.  When attached to a chassis, only an offset is provided.
//
// =============================================================================

#ifndef CH_ANTIROLLBAR_RSD_H
#define CH_ANTIROLLBAR_RSD_H

#include "chrono/assets/ChColorAsset.h"

#include "chrono_vehicle/wheeled_vehicle/ChAntirollBar.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_antirollbar
/// @{

/// Template for an anti-roll subsystem using an RSD.
/// Base class for an anti-roll bar template modeled two arms connected with a
/// revolute spring-damper.
/// Derived from ChAntirollBar, but still and abstract base class.
///
/// The anti-roll bar subsystem is modeled with respect to a right-handed frame,
/// with X pointing towards the front, Y to the left, and Z up (ISO standard).
/// The subsystem reference frame is assumed to be always aligned with that of
/// the vehicle.  When attached to a chassis, only an offset is provided.
class CH_VEHICLE_API ChAntirollBarRSD : public ChAntirollBar {
  public:
    ChAntirollBarRSD(const std::string& name  ///< [in] name of the subsystem
                     );

    virtual ~ChAntirollBarRSD();

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "AntirollBarRSD"; }

    /// Initialize this anti-roll bar subsystem.
    /// The anti-roll bar subsystem is initialized by attaching it to the specified chassis at the given location (with
    /// respect to and expressed in the reference frame of the chassis) and associating it with the specified suspension
    /// subsystem (assumed to be independent). It is assumed that the anti-roll bar reference frame is always aligned
    /// with the chassis reference frame.
    virtual void Initialize(std::shared_ptr<ChChassis> chassis,        ///< [in] handle to the chassis body
                            std::shared_ptr<ChSuspension> suspension,  ///< [in] associated suspension subsystem
                            const ChVector<>& location                 ///< [in] location relative to the chassis frame
                            ) override;

    /// Log current constraint violations.
    virtual void LogConstraintViolations() override;

  protected:
    virtual void InitializeInertiaProperties() override;
    virtual void UpdateInertiaProperties() override;

    /// Return the mass of the arm body.
    virtual double getArmMass() const = 0;
    /// Return the moments of inertia of the arm body.
    virtual ChVector<> getArmInertia() = 0;

    /// Return the arm length (dimension in y direction).
    virtual double getArmLength() const = 0;
    /// Return the arm width (dimension in X direction).
    virtual double getArmWidth() const = 0;
    /// Return height of droplinks (dimension in Z direction).
    virtual double getDroplinkHeight() const = 0;
    /// Return radius of arm (visualization only)
    virtual double getArmRadius() const = 0;

    /// Return the rotational spring coefficient.
    virtual double getSpringCoefficient() const = 0;
    /// Return the rotational damping coefficient.
    virtual double getDampingCoefficient() const = 0;

    std::shared_ptr<ChBody> m_arm_left;                 ///< handle to the left arm body
    std::shared_ptr<ChBody> m_arm_right;                ///< handle to the right arm body
    std::shared_ptr<ChLinkLockRevolute> m_revolute_ch;  ///< handle to revolute joint to chassis
    std::shared_ptr<ChLinkLockRevolute> m_revolute;     ///< handle to central revolute joint
    std::shared_ptr<ChLinkDistance> m_link_left;        ///< handle to the left droplink distance constraint
    std::shared_ptr<ChLinkDistance> m_link_right;       ///< handle to the right droplink distance constraint

  private:
    void AddVisualizationArm(std::shared_ptr<ChBody> arm,
                             const ChVector<>& pt_1,
                             const ChVector<>& pt_2,
                             const ChVector<>& pt_3,
                             double radius,
                             const ChColor& color);

    virtual void ExportComponentList(rapidjson::Document& jsonDocument) const override;

    virtual void Output(ChVehicleOutput& database) const override;
};

/// @} vehicle_wheeled_antirollbar

}  // end namespace vehicle
}  // end namespace chrono

#endif
