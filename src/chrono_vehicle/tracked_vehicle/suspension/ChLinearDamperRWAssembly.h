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

#include "chrono/physics/ChLinkSpringCB.h"
#include "chrono/physics/ChLinkForce.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"

#include "chrono_vehicle/tracked_vehicle/ChRoadWheelAssembly.h"

namespace chrono {
namespace vehicle {

///
///
///
class CH_VEHICLE_API ChLinearDamperRWAssembly : public ChRoadWheelAssembly {
  public:
    ChLinearDamperRWAssembly(const std::string& name  ///< [in] name of the subsystem
                             );

    virtual ~ChLinearDamperRWAssembly() {}

    /// Return a handle to the carrier body.
    virtual ChSharedPtr<ChBody> GetCarrierBody() const override { return m_arm; }

    /// Initialize this suspension subsystem.
    /// The suspension subsystem is initialized by attaching it to the specified
    /// chassis body at the specified location (with respect to and expressed in
    /// the reference frame of the chassis). It is assumed that the suspension
    /// reference frame is always centered at the location of the road wheel and
    /// aligned with the chassis reference frame.
    virtual void Initialize(ChSharedPtr<ChBodyAuxRef> chassis,  ///< [in] handle to the chassis body
                            const ChVector<>& location          ///< [in] location relative to the chassis frame
                            ) override;

  protected:
    /// Identifiers for the various hardpoints.
    enum PointId {
        ARM,          ///< arm location
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

    /// Return the funtion for torsion force
    virtual ChTorsionForce* GetTorsionForceFunction() const = 0;

    /// Return the callback function for shock force.
    virtual ChSpringForceCallback* GetShockForceCallback() const = 0;

    ChSharedPtr<ChBody> m_arm;                   ///< handle to the trailing arm body
    ChSharedPtr<ChLinkLockRevolute> m_revolute;  ///< handle to the revolute joint arm-chassis
    ChSharedPtr<ChLinkSpringCB> m_shock;         ///< handle to the shock link

  private:
    void AddVisualizationArm(const ChVector<>& pt_A,   ///< arm location (in global frame)
                             const ChVector<>& pt_AW,  ///< connection to wheel (in global frame)
                             const ChVector<>& pt_AC   ///< connection to chassis (in global frame)
                             );
};

}  // end namespace vehicle
}  // end namespace chrono

#endif
