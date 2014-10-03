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
// Base class for a Pitman Arm steering subsystem.
// Derived from ChSteering, but still an abstract base class.
//
// The steering subsystem is modeled with respect to a right-handed frame with
// with X pointing towards the front, Y to the left, and Z up (ISO standard).
//
// When attached to a chassis, both an offset and a rotation (as a quaternion)
// are provided.
//
// =============================================================================

#ifndef CH_PITMANARM_H
#define CH_PITMANARM_H

#include "subsys/ChApiSubsys.h"
#include "subsys/ChSteering.h"

namespace chrono {


class CH_SUBSYS_API ChPitmanArm : public ChSteering
{
public:

  ChPitmanArm(const std::string& name);
  virtual ~ChPitmanArm() {}

  virtual void Initialize(ChSharedPtr<ChBodyAuxRef> chassis,
                          const ChVector<>&         location,
                          const ChQuaternion<>&     rotation);

  virtual void Update(double time, double steering);

  virtual void LogConstraintViolations();

protected:

  enum PointId {
    STEERINGLINK,   ///< steering link location (com)
    PITMANARM,      ///< Pitman arm location (com)
    REV,            ///< location of joint between Pitman arm and chassis
    UNIV,           ///< location of joint between Pitman arm and steering link
    REVSPH_R,       ///< location of revolute joint for the idler arm
    REVSPH_S,       ///< location of spherical joint for the idler arm
    TIEROD_PA,      ///< tierod connection point (Pitman arm side)
    TIEROD_IA,      ///< tierod connection point (idler arm side)
    NUM_POINTS
  };

  enum DirectionId {
    REV_AXIS,       // revolute joint
    UNIV_AXIS_ARM,  // universal joint (Pitman arm side)
    UNIV_AXIS_LINK, // universal joint (steering link side)
    REVSPH_AXIS,    // revolute joint for idler arm
    NUM_DIRS
  };

  virtual const ChVector<> getLocation(PointId which) = 0;
  virtual const ChVector<> getDirection(DirectionId which) = 0;

  virtual double getSteeringLinkMass() const = 0;
  virtual double getPitmanArmMass() const = 0;

  virtual double getSteeringLinkRadius() const = 0;
  virtual double getPitmanArmRadius() const = 0;

  virtual double getMaxAngle() const = 0;

  virtual const ChVector<>& getSteeringLinkInertia() const = 0;
  virtual const ChVector<>& getPitmanArmInertia() const = 0;

  ChSharedPtr<ChBody>                  m_arm;

  ChSharedPtr<ChLinkEngine>            m_revolute;
  ChSharedPtr<ChLinkRevoluteSpherical> m_revsph;
  ChSharedPtr<ChLinkLockUniversal>     m_universal;

private:

  static void AddVisualizationPitmanArm(ChSharedPtr<ChBody> arm,
                                        const ChVector<>&   pt_C,
                                        const ChVector<>&   pt_L,
                                        double              radius);

  static void AddVisualizationSteeringLink(ChSharedPtr<ChBody> link,
                                           const ChVector<>&   pt_P,
                                           const ChVector<>&   pt_I,
                                           const ChVector<>&   pt_TP,
                                           const ChVector<>&   pt_TI,
                                           double              radius);
};


} // end namespace chrono


#endif
