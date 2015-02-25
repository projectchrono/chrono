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
// Authors: Justin Madsen
// =============================================================================
//
// A simple suspension/road wheel system, that uses a torsional spring and rigid arm
//
// =============================================================================

#ifndef TORSION_ARM_SUSPENSION_H
#define TORSION_ARM_SUSPENSION_H

#include "subsys/ChApiSubsys.h"
#include "physics/ChSystem.h"
#include "physics/ChBodyAuxRef.h"
#include "physics/ChShaftsBody.h"
#include "physics/ChShaftsTorsionSpring.h"

#include "ModelDefs.h"

namespace chrono {

/// Functor class for a custom rotaional spring constant modifier
class ChFunction_CustomSpring : public ChFunction
{
public:
  ChFunction* new_Duplicate() {return new ChFunction_CustomSpring;} 

  double Get_y(double x)
  {
    double spring_coef = 50;
    double spring_nonlin_coef = 10;

    return spring_coef + spring_nonlin_coef * fabs(x);
  }
};


/// Consists of two bodies, the torsion arm link and the road-wheel.
/// Arm constrained to chassis via revolute joint, as is the arm to wheel.
/// Rotational spring damper between arm and chassis
class CH_SUBSYS_API TorsionArmSuspension : public ChShared
{
public:

  TorsionArmSuspension(const std::string& name,
    VisualizationType::Enum vis = VisualizationType::Enum::Primitives,
    CollisionType::Enum collide = CollisionType::Enum::Primitives,
    size_t chainSys_idx = 0,  ///< what chain system is the road wheel assoc. with?
    double wheel_mass = 561.1 , ///< [kg]
    const ChVector<>& wheelIxx = ChVector<>(19.82, 19.82, 26.06), // [kg-m2], z-axis of rotation,
    double arm_mass = 75.26,  ///< [kg]
    const ChVector<>& armIxx = ChVector<>(0.77, 0.37, 0.77),  ///< [kg-m2]
    double springK = 25000,	///< torsional spring constant [N-m/rad]
    double springC = 250,	///< torsional damping constant [N-m-s/rad]
    double springPreload = 1500.0,  ///< torque preload [N-m]
    bool use_custom_spring = false  ///< use ChFunction_CustomSpring rather than default: a pre-loaded linear Rotational spring-damper?
    );

  ~TorsionArmSuspension() {delete m_rot_spring;}

  /// init the suspension with the initial pos. and rot., w.r.t. the chassis c-sys
  /// specifies the attachment point of the arm to the hull bodies
  void Initialize(ChSharedPtr<ChBody> chassis,
                  const ChFrame<>& chassis_REF,
                  const ChCoordsys<>& local_Csys);
  
  
  double getSpringCoefficient() const { return m_springK; }
  double getDampingCoefficient() const { return m_springC; }

  ChSharedPtr<ChBody> GetArmBody() const { return m_arm; }
  ChSharedPtr<ChBody> GetWheelBody() const { return m_wheel; }

  /// return the distance to the wheel, in the suspension local coords
  const ChVector<> GetWheelPosRel() const { return m_wheel_PosRel; }

  double GetWheelRadius() const { return m_wheelRadius; }

  // log constraint violations of any bilateral constraints
  void LogConstraintViolations();

  /// write constraint violations to ostream, which will be written to the output file
  void SaveConstraintViolations(std::stringstream& ss);

  /// write headers for the output data file to the input ostream
  const std::string getFileHeader_ConstraintViolations(size_t idx) const;

private:

  // private functions
  void Create(const std::string& name);
  void AddVisualization();
  void AddCollisionGeometry(double mu = 0.4,
                            double mu_sliding = 0.3,
                            double mu_roll = 0,
                            double mu_spin = 0);
  
  // private functions
  const std::string& getMeshName() const { return m_meshName; }
  const std::string& getMeshFile() const { return m_meshFile; }

  // private variables
  ChSharedPtr<ChBody> m_arm;  ///< arm body
  ChSharedPtr<ChLinkLockRevolute> m_armChassis_rev; ///< arm-chassis revolute joint

  // shock absorber is a pre-loaded linear spring-damper
  ChLinkForce* m_rot_spring;  ///< torsional spring, attached to the armChassis rev joint
  double m_springK;	          ///< torsional spring constant
  double m_springC;	          ///< torsional damping constant
  double m_TorquePreload;	    ///< preload torque, on the spring/damper DOF
  // ... OR, use a custom shock absorber
  const bool m_use_custom_spring; ///< use a custom spring or a linear spring-damper?
  ChSharedPtr<ChFunction_CustomSpring> m_custom_spring; ///< custom spring element

  ChSharedPtr<ChBody> m_wheel;  ///< wheel body
  ChSharedPtr<ChLinkLockRevolute> m_armWheel_rev; ///< arm-wheel revolute joint
  ChVector<> m_wheel_PosRel;  ///< position of wheel center w.r.t. arm pin to chassis, corrected for left/right side.

  // body variables
  double m_armMass;
  ChVector<> m_armInertia;
  double m_wheelMass;
  ChVector<> m_wheelInertia;

  // visual and collision geometry types
  VisualizationType::Enum m_vis;
  CollisionType::Enum m_collide;
  const size_t m_chainSys_idx; ///< if there are multiple chain systems 
  // (e.g., on the M113, the subsystem knows which it is a part of for collision family purposes)

  const std::string m_meshName; ///< name of the mesh, if any
  const std::string m_meshFile;  ///< filename of the mesh, if any

  // static variables
  static const double m_armRadius;
  static const double m_wheelWidth;
  static const double m_wheelWidthGap;
  static const double m_wheelRadius;
  static const ChVector<> m_wheel_Pos;
  
  static const double m_shaft_inertia;
};


} // end namespace chrono


#endif
