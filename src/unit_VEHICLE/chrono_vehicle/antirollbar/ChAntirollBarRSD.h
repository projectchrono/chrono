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

#include "assets/ChColorAsset.h"

#include "chrono_vehicle/ChAntirollBar.h"

namespace chrono {

class CH_VEHICLE_API ChAntirollBarRSD : public ChAntirollBar {
public:

  ChAntirollBarRSD(
    const std::string& name               ///< [in] name of the subsystem
    );

  virtual ~ChAntirollBarRSD() {}

  /// The anti-roll bar subsystem is initialized by attaching it to the specified
  /// chassis body at the specified location (with respect to and expressed in
  /// the reference frame of the chassis). It is assumed that the suspension
  /// reference frame is always aligned with the chassis reference frame.
  /// Finally, susp_body_left and susp_body_right are handles to the suspension
  /// bodies to which the anti-roll bar's droplinks are to be attached.
  virtual void Initialize(
    ChSharedPtr<ChBodyAuxRef>  chassis,         ///< [in] handle to the chassis body
    const ChVector<>&          location,        ///< [in] location relative to the chassis frame
    ChSharedPtr<ChBody>        susp_body_left,  ///< [in] susp body to which left droplink is connected
    ChSharedPtr<ChBody>        susp_body_right  ///< [in] susp body to which right droplink is connected
    );

  /// Log current constraint violations.
  virtual void LogConstraintViolations();

protected:

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

  ChSharedPtr<ChBody>              m_arm_left;     ///< handle to the left arm body
  ChSharedPtr<ChBody>              m_arm_right;    ///< handle to the right arm body
  ChSharedPtr<ChLinkLockRevolute>  m_revolute_ch;  ///< handle to revolute joint to chassis
  ChSharedPtr<ChLinkLockRevolute>  m_revolute;     ///< handle to central revolute joint
  ChSharedPtr<ChLinkDistance>      m_link_left;    ///< handle to the left droplink distance constraint
  ChSharedPtr<ChLinkDistance>      m_link_right;   ///< handle to the right droplink distance constraint

private:

  void AddVisualizationArm(ChSharedPtr<ChBody>  arm,
                           const ChVector<>&    pt_1,
                           const ChVector<>&    pt_2,
                           const ChVector<>&    pt_3,
                           double               radius,
                           const ChColor&       color);
};


} // end namespace chrono


#endif
