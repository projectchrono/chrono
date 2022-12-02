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
// Classes for monitoring contacts of tracked vehicle subsystems.
//
// =============================================================================

#ifndef CH_TRACK_CONTACT_MANAGER
#define CH_TRACK_CONTACT_MANAGER

#include <list>

#include "chrono/physics/ChContactContainer.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/collision/ChCollisionSystem.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/ChChassis.h"
#include "chrono_vehicle/tracked_vehicle/ChSprocket.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackShoe.h"
#include "chrono_vehicle/tracked_vehicle/ChIdler.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackWheel.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked
/// @{

class ChTrackedVehicle;
class ChTrackTestRig;

// -----------------------------------------------------------------------------

/// Class for monitoring contacts of tracked vehicle subsystems.
class CH_VEHICLE_API ChTrackContactManager : public ChContactContainer::ReportContactCallback {
  public:
    ChTrackContactManager();

    void MonitorContacts(int flags) { m_flags |= flags; }
    void SetContactCollection(bool val) { m_collect = val; }
    void WriteContacts(const std::string& filename);

    void SetTrackShoeIndexLeft(size_t idx) { m_shoe_index_L = idx; }
    void SetTrackShoeIndexRight(size_t idx) { m_shoe_index_R = idx; }

    void SetRenderNormals(bool val) { m_render_normals = val; }
    void SetRenderForces(bool val, double scale) {
        m_render_forces = val;
        m_scale_forces = scale;
    }

    void Process(ChTrackedVehicle* vehicle);
    void Process(ChTrackTestRig* rig);

    bool InContact(TrackedCollisionFlag::Enum part) const;

    ChVector<> GetSprocketResistiveTorque(VehicleSide side) const;

  private:
    /// Contact information data structure.
    struct ContactInfo {
        ChVector<> m_point;
        ChMatrix33<> m_csys;
        ChVector<> m_force;
        ChVector<> m_torque;
    };

    bool IsFlagSet(TrackedCollisionFlag::Enum val) { return (m_flags & static_cast<int>(val)) != 0; }

    /// Callback, used to report contact points already added to the container.
    /// If it returns false, the contact scanning will be stopped.
    virtual bool OnReportContact(const ChVector<>& pA,
                                 const ChVector<>& pB,
                                 const ChMatrix33<>& plane_coord,
                                 const double& distance,
                                 const double& eff_radius,
                                 const ChVector<>& react_forces,
                                 const ChVector<>& react_torques,
                                 ChContactable* modA,
                                 ChContactable* modB) override;

    bool m_initialized;  ///< true if the contact manager was initialized
    int m_flags;         ///< contact bit flags
    bool m_collect;      ///< flag indicating whether or not data is collected

    bool m_render_normals;  ///< render contact normals
    bool m_render_forces;   ///< render contact forces
    double m_scale_forces;  ///< contact force rendering scale

    utils::CSV_writer m_csv;

    std::shared_ptr<ChChassis> m_chassis;
    std::shared_ptr<ChSprocket> m_sprocket_L;
    std::shared_ptr<ChSprocket> m_sprocket_R;
    std::shared_ptr<ChIdler> m_idler_L;
    std::shared_ptr<ChIdler> m_idler_R;
    std::shared_ptr<ChTrackShoe> m_shoe_L;
    std::shared_ptr<ChTrackShoe> m_shoe_R;

    size_t m_shoe_index_L;  ///< index of monitored track shoe on left track
    size_t m_shoe_index_R;  ///< index of monitored track shoe on right track

    std::list<ContactInfo> m_chassis_contacts;     ///< list of contacts on chassis
    std::list<ContactInfo> m_sprocket_L_contacts;  ///< list of contacts on left sprocket gear
    std::list<ContactInfo> m_sprocket_R_contacts;  ///< list of contacts on right sprocket gear
    std::list<ContactInfo> m_shoe_L_contacts;      ///< list of contacts on left track shoe
    std::list<ContactInfo> m_shoe_R_contacts;      ///< list of contacts on right track shoe
    std::list<ContactInfo> m_idler_L_contacts;     ///< list of contacts on left idler wheel
    std::list<ContactInfo> m_idler_R_contacts;     ///< list of contacts on right idler wheel

    friend class ChTrackedVehicleVisualSystemIrrlicht;
    friend class ChTrackTestRigVisualSystemIrrlicht;
};

// -----------------------------------------------------------------------------

// Class for monitoring collisions of tracked vehicle subsystems.
// This private class is only used by ChTrackedVehicle (do not export?)
class CH_VEHICLE_API ChTrackCollisionManager : public collision::ChCollisionSystem::NarrowphaseCallback {
    ChTrackCollisionManager(ChTrackedVehicle* vehicle);

    /// Empty the list of wheel-track shoe collisions
    void Reset();

    /// Callback used to process collision pairs found by the narrow-phase collision step.
    /// Return true to generate a contact for this pair of overlapping bodies.
    virtual bool OnNarrowphase(collision::ChCollisionInfo& contactinfo) override;

    bool m_idler_shoe;                                            ///< process collisions with idler bodies
    bool m_wheel_shoe;                                            ///< process collisions with road-wheel bodies
    bool m_ground_shoe;                                           ///< process collisions with ground bodies
    std::vector<collision::ChCollisionInfo> m_collisions_idler;   ///< current list of idler-track shoe collisions
    std::vector<collision::ChCollisionInfo> m_collisions_wheel;   ///< current list of wheel-track shoe collisions
    std::vector<collision::ChCollisionInfo> m_collisions_ground;  ///< current list of ground-track shoe collisions

    friend class ChTrackedVehicle;
    friend class ChTrackCustomContact;
};

/// Callback interface for user-defined custom contact between road wheels and track shoes.
class CH_VEHICLE_API ChTrackCustomContact : public ChLoadContainer {
  public:
    virtual ~ChTrackCustomContact() {}

    /// Indicate if overriding contact forces with idlers.
    /// If returning true, the derived class must provide an override of ComputeIdlerContactForce.
    virtual bool OverridesIdlerContact() const { return false; }

    /// Indicate if overriding contact forces with road wheels.
    /// If returning true, the derived class must provide an override of ComputeWheelContactForce.
    virtual bool OverridesWheelContact() const { return false; }

    /// Indicate if overriding contact forces with ground.
    /// If returning true, the derived class must provide an override of ComputeGroundContactForce.
    virtual bool OverridesGroundContact() const { return false; }

    /// For the given collision between an idler and a track shoe, compute the contact force on the track shoe at the
    /// contact point. The first contactable in 'cinfo' is the idler body and the second contactable is the track shoe
    /// body. The return force is assumed to be expressed in the absolute reference frame.
    virtual void ComputeIdlerContactForce(
        const collision::ChCollisionInfo& cinfo,  ///< [in] geometric information for the collision pair
        std::shared_ptr<ChBody> idlerBody,        ///< [in] idler body in collision
        std::shared_ptr<ChBody> shoeBody,         ///< [in] track shoe body in collision
        ChVector<>& forceShoe                     ///< [out] force on track shoe at contact point, in abs. frame
    ) {
        std::cout << "ERROR: Idler-shoe custom contact force calculation not implemented!" << std::endl;
        throw(ChException("Idler-shoe custom contact force calculation not implemented."));
    }

    /// For the given collision between a road-wheel and a track shoe, compute the contact force on the track shoe at
    /// the contact point. The first contactable in 'cinfo' is the road-wheel body and the second contactable is the
    /// track shoe body. The return force is assumed to be expressed in the absolute reference frame.
    virtual void ComputeWheelContactForce(
        const collision::ChCollisionInfo& cinfo,  ///< [in] geometric information for the collision pair
        std::shared_ptr<ChBody> wheelBody,        ///< [in] road-wheel body in collision
        std::shared_ptr<ChBody> shoeBody,         ///< [in] track shoe body in collision
        ChVector<>& forceShoe                     ///< [out] force on track shoe at contact point, in abs. frame
    ) {
        std::cout << "ERROR: Wheel-shoe custom contact force calculation not implemented!" << std::endl;
        throw(ChException("Wheel-shoe custom contact force calculation not implemented."));
    }

    /// For the given collision between ground and a track shoe, compute the contact force on the track shoe at
    /// the contact point. The first contactable in 'cinfo' is the ground body and the second contactable is the
    /// track shoe body. The return force is assumed to be expressed in the absolute reference frame.
    virtual void ComputeGroundContactForce(
        const collision::ChCollisionInfo& cinfo,  ///< [in] geometric information for the collision pair
        std::shared_ptr<ChBody> groundBody,       ///< [in] ground body in collision
        std::shared_ptr<ChBody> shoeBody,         ///< [in] track shoe body in collision
        ChVector<>& forceShoe                     ///< [out] force on track shoe at contact point, in abs. frame
    ) {
        std::cout << "ERROR: Ground-shoe custom contact force calculation not implemented!" << std::endl;
        throw(ChException("Ground-shoe custom contact force calculation not implemented."));
    }

  private:
    virtual void Setup() override;
    virtual void Update(double mytime, bool update_assets = true) override;
    void ApplyForces();

    ChTrackCollisionManager* m_collision_manager;

    friend class ChTrackedVehicle;
};

/// @} vehicle_tracked

}  // end namespace vehicle
}  // end namespace chrono

#endif
