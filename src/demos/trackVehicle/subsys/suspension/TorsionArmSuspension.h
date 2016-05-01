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
class ChFunction_CustomSpring : public ChFunction {
  public:
    ChFunction* new_Duplicate() override { return new ChFunction_CustomSpring; }

    double Get_y(double x) {
        double spring_coef = 50;
        double spring_nonlin_coef = 10;

        return spring_coef + spring_nonlin_coef * fabs(x);
    }
};

/// Consists of two bodies, the torsion arm link and the road-wheel.
/// Arm constrained to chassis via revolute joint, as is the arm to wheel.
/// Rotational spring damper between arm and chassis
class CH_SUBSYS_API TorsionArmSuspension : public ChShared {
  public:
    TorsionArmSuspension(
        const std::string& name,
        VisualizationType::Enum vis = VisualizationType::Primitives,
        CollisionType::Enum collide = CollisionType::Primitives,
        size_t chainSys_idx = 0,    ///< what chain system is the road wheel assoc. with?
        double wheel_mass = 561.1,  ///< [kg]
        const ChVector<>& wheelIxx = ChVector<>(19.82, 19.82, 26.06),  // [kg-m2], z-axis of rotation,
        double arm_mass = 75.26,                                       ///< [kg]
        const ChVector<>& armIxx = ChVector<>(0.77, 0.37, 0.77),       ///< [kg-m2]
        double springK = 2.5e4,         ///< torsional spring constant [N-m/rad]
        double springC = 5e2,           ///< torsional damping constant [N-m-s/rad]
        double springPreload = 1.5e3,   ///< torque preload [N-m]
        double mu = 0.4,                ///< wheel friction coef                                
        double wheel_width = 0.384,     ///< bogie wheel width [m]
        double wheel_width_gap = .0912, ///< width beween parallel/concentric bogie wheel [m]
        double wheel_radius = 0.305,    ///< bogie wheel radius [m]
        ChVector<> wheel_pos_rel = ChVector<>(-0.2034,
                                              -0.2271,
                                              0.24475),  ///< wheel position to the arm/chassis rev joint, local c-sys
        double arm_radius = 0.05,                        ///< torsion arm radius
        bool use_custom_spring =
            false  ///< use ChFunction_CustomSpring rather than default: a pre-loaded linear Rotational spring-damper?
        );

    ~TorsionArmSuspension() { delete m_rot_spring; }

    /// init the suspension with the initial pos. and rot., w.r.t. the chassis c-sys
    /// specifies the attachment point of the arm to the hull bodies
    void Initialize(ChSharedPtr<ChBody> chassis, const ChFrame<>& chassis_REF, const ChCoordsys<>& local_Csys);

    double getSpringCoefficient() const { return m_springK; }
    double getDampingCoefficient() const { return m_springC; }

    ChSharedPtr<ChBody> GetArmBody() const { return m_arm; }
    ChSharedPtr<ChBody> GetWheelBody() const { return m_wheel; }

    /// return the distance to the wheel, in the suspension local coords
    const ChVector<> GetWheelPosRel() const { return m_wheel_PosRel; }

    double GetWheelRadius() const { return m_wheelRadius; }

    /// write headers for the output data file to the input ostream
    void Write_header(const std::string& filename, DebugType type);

    /// write constraint violation of wheel rev. constraint
    void Write_data(double t, DebugType type);

  private:
    // private functions
    void Create(const std::string& name);
    void AddVisualization();
    void AddCollisionGeometry(VehicleSide side = RIGHTSIDE,
                              double mu = 0.4,
                              double mu_sliding = 0.3,
                              double mu_roll = 0,
                              double mu_spin = 0);

    // private functions
    const std::string& getMeshName() const { return m_meshName; }
    const std::string& getMeshFile() const { return m_meshFile; }

    // private variables
    ChSharedPtr<ChBody> m_arm;                         ///< arm body
    ChSharedPtr<ChLinkLockRevolute> m_armChassis_rev;  ///< arm-chassis revolute joint

    // shock absorber is a pre-loaded linear spring-damper
    ChLinkForce* m_rot_spring;  ///< torsional spring, attached to the armChassis rev joint
    double m_springK;           ///< torsional spring constant
    double m_springC;           ///< torsional damping constant
    double m_TorquePreload;     ///< preload torque, on the spring/damper DOF
    
    // ... OR, use a custom shock absorber
    const bool m_use_custom_spring;                        ///< use a custom spring or a linear spring-damper?
    ChSharedPtr<ChFunction_CustomSpring> m_custom_spring;  ///< custom spring element

    ChSharedPtr<ChBody> m_wheel;                     ///< wheel body
    ChSharedPtr<ChLinkLockRevolute> m_armWheel_rev;  ///< arm-wheel revolute joint

    // body variables
    double m_armMass;
    ChVector<> m_armInertia;
    double m_wheelMass;
    ChVector<> m_wheelInertia;

    // visual and collision geometry types
    VisualizationType::Enum m_vis;
    CollisionType::Enum m_collide;
    const size_t m_chainSys_idx;  ///< if there are multiple chain systems
    double m_mu;    ///< static friction coef
    // (e.g., on the M113, the subsystem knows which it is a part of for collision family purposes)

    // output filenames
    std::string m_filename_DBG_BODY;      // write idler body info
    std::string m_filename_DBG_CV;        // write idler constraint violation
    std::string m_filename_DBG_CONTACTS;  // write idler contact info

    const std::string m_meshName;  ///< name of the mesh, if any
    const std::string m_meshFile;  ///< filename of the mesh, if any

    // constant variables
    const double m_wheelWidth;
    const double m_wheelWidthGap;
    const double m_wheelRadius;
    ChVector<> m_wheel_PosRel;  ///< position of wheel center w.r.t. arm pin to chassis, corrected for left/right side.
    // const ChVector<> m_wheel_Pos;
    const double m_armRadius;

    static const double m_shaft_inertia;
};

}  // end namespace chrono

#endif
