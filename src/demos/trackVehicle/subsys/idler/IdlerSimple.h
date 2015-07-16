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
// A simple Idler system that keeps track chain tension by pre-loading a
//	spring/damper element
//
// =============================================================================

#ifndef IDLERSIMPLE_H
#define IDLERSIMPLE_H

#include "subsys/ChApiSubsys.h"
#include "physics/ChSystem.h"
#include "physics/ChBodyAuxRef.h"
#include "ModelDefs.h"

namespace chrono {

/// An idler system that includes the chain tensioning system.
/// The simplest case of this is a prismatic (translational) constraint
/// between the idler and chassis, and mount a pre-loaded spring-damper
/// along the DOF axis
class CH_SUBSYS_API IdlerSimple : public ChShared {
    friend class TrackSystem;
    friend class TrackSystemM113;

  public:
    /// constructor, where only the body name must be specified.
    //  default mass, inertia, tensioner spring/damping constants for M113 APC
    IdlerSimple(const std::string& name,
                VisualizationType::Enum vis = VisualizationType::Primitives,
                CollisionType::Enum collide = CollisionType::Primitives,
                size_t chainSys_idx = 0,  ///< what chain system is this idler associated with?
                double mass = 429.6,
                const ChVector<>& Ixx = ChVector<>(12.55, 12.55, 14.7),
                double tensionerK = 2e5,    ///< idler tensioner spring coef. [N/m]
                double tensionerC = 1e4,    ///< idler tensioner damper coef. [N-s/m]
                double springFreeLen = 1.0  ///< idler tensioner spring free length [m]
                );

    ~IdlerSimple();

    /// init the idler with the initial pos. and rot., w.r.t. the chassis c-sys
    // x-axis of local_Csys rot should point in translational DOF dir.
    void Initialize(ChSharedPtr<ChBody> chassis,
                    const ChFrame<>& chassis_REF,
                    const ChCoordsys<>& local_Csys,
                    double preLoad = 0);

    // log constraint violations of any bilateral constraints
    void LogConstraintViolations();

    /// write constraint violations to ostream, which will be written to the output file
    void SaveConstraintViolations(std::stringstream& ss);

    /// write headers for the output data file to the input ostream
    const std::string getFileHeader_ConstraintViolations() const;

    // Accessors
    ChSharedPtr<ChLinkSpring> getShock() const { return m_shock; }

    double getSpringCoefficient() const { return m_tensionerK; }
    double getDampingCoefficient() const { return m_tensionerC; }
    double getSpringRestLength() const { return m_springRestLength; }
    ChSharedPtr<ChBody> GetBody() const { return m_idler; }
    double GetRadius() const { return m_radius; }

    double GetSpringForce() const { return m_shock->Get_SpringReact(); }

    // get component reactions for the spring
    double Get_SpringReact_Deform() const;

    double Get_SpringReact_Deform_dt() const;

  private:
    // private functions
    void AddVisualization(size_t chain_idx, bool custom_texture = false, const std::string& tex_name = "none");
    void AddCollisionGeometry(VehicleSide side = RIGHTSIDE,  // right or left side
                              double mu = 0.4,
                              double mu_sliding = 0.3,
                              double mu_roll = 0,
                              double mu_spin = 0);

    // private functions
    const std::string& getMeshName() const { return m_meshName; }
    const std::string& getMeshFile() const { return m_meshFile; }

    // private variables
    ChSharedPtr<ChBody> m_idler;                             ///< handle to idler body
    ChSharedPtr<ChLinkLockRevolutePrismatic> m_idler_joint;  ///< connetion to chassis
    ChSharedPtr<ChLinkSpring> m_shock;                       ///< handle to spring-damper;
    double m_tensionerK;                                     ///< shock linear spring coefficient
    double m_tensionerC;                                     ///< shock linear damping coefficient
    // ChSpringForceCallback* m_shockCB;   ///< shock callback function
    // ChSpringForceCallback* m_springCB;  ///< spring callback function
    double m_springRestLength;  ///< shock rest length

    VisualizationType::Enum m_vis;
    CollisionType::Enum m_collide;
    const size_t m_chainSys_idx;  ///< if there are multiple chain systems
    // (e.g., on the M113, the subsystem knows which it is a part of for collision family purposes)

    double m_mass;
    ChVector<> m_inertia;  ///< z-axis of rotation

    const std::string m_meshName;
    const std::string m_meshFile;

    // static variables
    static const double m_width;
    static const double m_widthGap;  // // inner distance between cydliners
    static const double m_radius;
};

}  // end namespace chrono

#endif
