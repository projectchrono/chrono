// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2015 projectchrono.org
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
// The drive gear propels the tracked vehicle.
//	Body is driven with a specified rotational motion.
//
// =============================================================================

#ifndef DRIVEGEARMOTION_H
#define DRIVEGEARMOTION_H

#include "subsys/ChApiSubsys.h"
#include "subsys/base/ChTrackVehicle.h"
#include "physics/ChSystem.h"
#include "physics/ChBodyAuxRef.h"
#include "ModelDefs.h"
// collision callback function
#include "subsys/collision/TrackCollisionCallback.h"

namespace chrono {

/// Drive gear class, a single rigid body. Attached to the chassis via revolute joint.
/// Torque applied by the driveline.
class CH_SUBSYS_API DriveGearMotion : public ChShared {
  public:
    /// override static values for mass, inertia
    DriveGearMotion(const std::string& name,
                    VisualizationType::Enum vis = VisualizationType::Primitives,
                    CollisionType::Enum collide = CollisionType::Primitives,
                    size_t chainSys_idx = 0,  ///< what chain system is this gear associated with?
                    double gear_mass = 436.7,
                    const ChVector<>& gear_Ixx = ChVector<>(12.22, 12.22, 13.87),
                    double mu = 0.2,    ///< static friction coef.
                    double max_gear_omega = 25.0  ///< max gear rotational velocity, rad/sec
                    );

    ~DriveGearMotion();

    /// init the gear with the initial pos. and rot., w.r.t. the chassis c-sys
    void Initialize(ChSharedPtr<ChBody> chassis,
                    const ChFrame<>& chassis_REF,
                    const ChCoordsys<>& local_Csys,
                    const std::vector<ChSharedPtr<ChBody> >& shoes,
                    ChTrackVehicle* vehicle);

    /// for updating the link engine rot vel, w/ input between -1 and 1
    void Update(double time, double omega_throttle);

    // accessors
    ChSharedPtr<ChBody> GetBody() const { return m_gear; }

    /// applied speed motion applied to gear
    double GetGearMotion() const;

    /// when represented as a cylinder shape
    double GetRadius() const { return m_radius; }

    /// write headers for drive gear for the specified output type
    void Write_header(const std::string& filename,
        DebugType type);

    /// write data this step for the specified output type
    void Write_data(const double t, ChSystem* system,
        DebugType type);

  private:
    // private functions
    double getMaxOmega() { return m_maxOmega; }
    /// returns the value clamped to range [min,max]
    double clamp(double val, double min_val = -1, double max_val = 1);

    const std::string& getMeshName() const { return m_meshName; }
    const std::string& getMeshFile() const { return m_meshFile; }

    void AddVisualization();
    void AddCollisionGeometry(const std::vector<ChSharedPtr<ChBody> >& shoes,
                              ChTrackVehicle* vehicle,
                              VehicleSide side = RIGHTSIDE,
                              double mu = 0.6,
                              double mu_sliding = 0.5,
                              double mu_roll = 0,
                              double mu_spin = 0);

    // private variables
    ChSharedPtr<ChBody> m_gear;
    ChSharedPtr<GearPinGeometry> m_gearPinGeom;  ///< gear and pin geometry info

    ChSharedPtr<ChLinkEngine> m_revolute;  ///< handle to revolute joint

    VisualizationType::Enum m_vis;  // visual asset geometry type
    CollisionType::Enum m_collide;  // collision geometry type
    const size_t m_chainSys_idx;    ///< if there are multiple chain systems
    double m_mu;
    // (e.g., on the M113, the subsystem knows which it is a part of for collision family purposes)

    ChVector<> m_inertia;
    const double m_mass;
    const double m_maxOmega;

    // output filenames
    std::string m_filename_DBG_BODY; // write idler body info
    std::string m_filename_DBG_CV;  // write idler constraint violation
    std::string m_filename_DBG_CONTACTS;   // write idler contact info

    const std::string m_meshName;
    const std::string m_meshFile;

    // static variables
    const double m_radius;
    const double m_width;
    const double m_widthGap;  // inner distance between cydliners

protected:
    /// info set is: (max, avg, stdev = sigma) for x,y,z vector data.
    /// returns total number of contacts with the gear this step.
    int reportGearContact(const ChSharedPtr<ChBody> which_gear,
        ChSystem* system,
        ChVector<>& Fn_info, ChVector<>& Ft_info);
};

}  // end namespace chrono

#endif
