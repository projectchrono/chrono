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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHLINKMARKERS_H
#define CHLINKMARKERS_H

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLink.h"
#include "chrono/physics/ChMarker.h"

namespace chrono {

/// Class for links which connect two 'markers'. The markers are two
/// ChMarker objects each belonging to the two linked ChBody parts.
/// Many specialized classes are based on this ChLinkMarkers class, for example
/// the ChLinkSpring and all the family of the ChLinkLock classes - see them-.
/// Also, ChLinkMarkers class allows an optional force vector and torque vector
/// to be set between the two connected markers.

class ChApi ChLinkMarkers : public ChLink {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChLinkMarkers)

  protected:
    ChMarker* marker1;  ///< slave coordsys
    ChMarker* marker2;  ///< master coordsys, =0 if liked to ground

    int markID1;  ///< unique identifier for markers 1 & 2,
    int markID2;  ///< when using plugin dynamic hierarchies

    Coordsys relM;       ///< relative marker position 2-1
    Coordsys relM_dt;    ///< relative marker speed
    Coordsys relM_dtdt;  ///< relative marker acceleration

    double relAngle;        ///< relative angle of rotation
    ChVector<> relAxis;     ///< relative axis of rotation
    ChVector<> relRotaxis;  ///< relative rotaion vector =angle*axis
    ChVector<> relWvel;     ///< relative angular speed
    ChVector<> relWacc;     ///< relative angular acceleration
    double dist;            ///< the distance between the two origins of markers,
    double dist_dt;         ///< the speed between the two  origins of markers

    ChVector<> Scr_force;   ///< internal force  set by script only (just added to C_force)
    ChVector<> Scr_torque;  ///< internal torque set by script only (just added to C_force)
    ChVector<> C_force;     ///< internal force  applied by springs/dampers/actuators
    ChVector<> C_torque;    ///< internal torque applied by springs/dampers/actuators

  public:
    ChLinkMarkers();
    ChLinkMarkers(const ChLinkMarkers& other);
    virtual ~ChLinkMarkers() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkMarkers* Clone() const override { return new ChLinkMarkers(*this); }

  public:
    /// Return the 1st referenced marker (the 'slave' marker, owned by 1st body)
    ChMarker* GetMarker1() { return marker1; }
    /// Return the 2nd referenced marker (the 'master' marker, owned by 2nd body)
    ChMarker* GetMarker2() { return marker2; }
    /// set the two markers associated with this link
    virtual void SetUpMarkers(ChMarker* mark1, ChMarker* mark2);
    void SetMarkID1(int mid) { markID1 = mid; }
    void SetMarkID2(int mid) { markID2 = mid; }
    int GetMarkID1() { return markID1; }
    int GetMarkID2() { return markID2; }

    /// Shortcut: set markers and marker IDs at once.
    bool ReferenceMarkers(ChMarker* mark1, ChMarker* mark2);

    /// Use this function after link creation, to initialize the link from
    /// two markers to join.
    /// Each marker must belong to a rigid body, and both rigid bodies
    /// must belong to the same ChSystem.
    /// The position of mark2 is used as link's position and main reference.
    virtual void Initialize(std::shared_ptr<ChMarker> mark1,  ///< first  marker to join
                            std::shared_ptr<ChMarker> mark2   ///< second marker to join (master)
                            );

    /// Use this function after link creation, to initialize the link from
    /// two joined rigid bodies.
    /// Both rigid bodies must belong to the same ChSystem.
    /// Two markers will be created and added to the rigid bodies (later,
    /// you can use GetMarker1() and GetMarker2() to access them.
    /// To specify the (absolute) position of link and markers, use 'mpos'.
    virtual void Initialize(std::shared_ptr<ChBody> mbody1,  ///< first  body to join
                            std::shared_ptr<ChBody> mbody2,  ///< second body to join
                            const ChCoordsys<>& mpos         ///< the current absolute pos.& alignment.
                            );

    /// Use this function after link creation, to initialize the link from
    /// two joined rigid bodies.
    /// Both rigid bodies must belong to the same ChSystem.
    /// Two markers will be created and added to the rigid bodies (later,
    /// you can use GetMarker1() and GetMarker2() to access them.
    /// To specify the (absolute) position of link and markers, use 'mpos'.
    virtual void Initialize(
        std::shared_ptr<ChBody> mbody1,  ///< first  body to join
        std::shared_ptr<ChBody> mbody2,  ///< second body to join
        bool pos_are_relative,      ///< if =true, following two positions are relative to bodies. If false, are absolute.
        const ChCoordsys<>& mpos1,  ///< the position & alignment of 1st marker (relative to body1 cords, or absolute)
        const ChCoordsys<>& mpos2   ///< the position & alignment of 2nd marker (relative to body2 cords, or absolute)
        );

    /// Get the link coordinate system, expressed relative to Body2 (the 'master'
    /// body). This represents the 'main' reference of the link: reaction forces
    /// and torques are expressed in this coordinate system.
    /// (It is the coordinate system of the 'master' marker2 relative to Body2)
    virtual ChCoordsys<> GetLinkRelativeCoords() override { return marker2->GetCoord(); }

    /// Get the master coordinate system for the assets (this will return the
    /// absolute coordinate system of the 'master' marker2)
    virtual ChFrame<> GetAssetsFrame(unsigned int nclone = 0) override { return marker2->GetAbsFrame(); }

    //
    // UPDATING FUNCTIONS
    //

    /// Updates auxiliary vars relM, relM_dt, relM_dtdt,
    /// dist, dist_dt et similia.
    virtual void UpdateRelMarkerCoords();

    ///  Updates auxiliary forces caused by springs/dampers/etc. which may
    /// be connected between the two bodies of the link.
    /// By default, it adds the forces which might have been added by the
    /// user using Set_Scr_force() and Set_Scr_torque(). Note, these forces
    /// are considered in the reference coordsystem of marker2 (the MAIN marker),
    /// and their application point is the origin of marker1 (the SLAVE marker).
    virtual void UpdateForces(double mytime);

    // -----------COMPLETE UPDATE.
    // sequence:
    //			UpdateTime;
    //          UpdateRelMarkerCoords;
    //			UpdateForces;

    virtual void Update(double mytime, bool update_assets = true) override;

    //
    // STATE FUNCTIONS
    //

    /// Adds force to residual R, as R*= F*c
    /// NOTE: here the off ofset in R is NOT used because add F at the TWO offsets of the two connected bodies,
    /// so it is assumed that offsets for Body1 and Body2 variables have been already set properly!
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override;

    //
    // SOLVER INTERFACE
    //

    /// Overrides the empty behaviour of the parent ChLink implementation, which
    /// does not consider any user-imposed force between the two bodies.
    /// It adds the current link-forces, if any, (caused by springs, etc.) to the 'fb' vectors
    /// of the ChVariables referenced by encapsulated ChConstraints.
    /// In details, it adds the effect caused by C_force and C_torque.
    /// Both C_force and C_torque these forces are considered expressed in the
    /// reference coordsystem of marker2 (the MAIN marker),
    /// and their application point is the origin of marker1 (the SLAVE marker).
    virtual void ConstraintsFbLoadForces(double factor = 1) override;

    //
    // LINK COORDINATES and other functions:
    //

    /// Relative position of marker 1 respect to marker 2.
    const Coordsys& GetRelM() const { return relM; }
    /// Relative speed of marker 1 respect to marker 2.
    const Coordsys& GetRelM_dt() const { return relM_dt; }
    /// Relative acceleration of marker 1 respect to marker 2.
    const Coordsys& GetRelM_dtdt() const { return relM_dtdt; }
    /// Relative rotation angle of marker 1 respect to marker 2 (best with revolute joints..).
    double GetRelAngle() const { return relAngle; }
    /// Relative finite rotation axis of marker 1 respect to marker 2.
    const ChVector<>& GetRelAxis() const { return relAxis; }
    const ChVector<>& GetRelRotaxis() const { return relRotaxis; }
    /// Relative angular speed of marker 1 respect to marker 2.
    const ChVector<>& GetRelWvel() const { return relWvel; }
    /// Relative angular acceleration of marker 1 respect to marker 2.
    const ChVector<>& GetRelWacc() const { return relWacc; }
    /// Relative 'polar' distance of marker 1 respect to marker 2.
    double GetDist() const { return dist; }
    /// Relative speed of marker 1 respect to marker 2, along the polar distance vector.
    double GetDist_dt() const { return dist_dt; }

    /// To get & set the 'script' force buffers(only accessed by
    /// external scripts, so it's up to the script to remember
    /// to set& reset them -link class just add them to C_force etc.
    const ChVector<>& Get_Scr_force() const { return Scr_force; }
    const ChVector<>& Get_Scr_torque() const { return Scr_torque; }
    void Set_Scr_force(const ChVector<>& mf) { Scr_force = mf; }
    void Set_Scr_torque(const ChVector<>& mf) { Scr_torque = mf; }

    /// Get the total applied force accumulators (force, momentum) in link coords.
    /// These forces might be affected by additional springs, dampers, etc. but they do not
    /// include the reaction forces.
    const ChVector<>& GetC_force() const { return C_force; }
    const ChVector<>& GetC_torque() const { return C_torque; }

    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

CH_CLASS_VERSION(ChLinkMarkers,0)


}  // end namespace chrono

#endif
