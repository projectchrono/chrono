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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHLINKMARKERS_H
#define CHLINKMARKERS_H

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLink.h"
#include "chrono/physics/ChMarker.h"

namespace chrono {

/// Class for links which connect two 'markers'.
/// The markers are two ChMarker objects each belonging to the two linked ChBody parts. Many specialized classes are
/// based on this ChLinkMarkers class, for example the family of ChLinkLock classes. ChLinkMarkers class allows an
/// optional force vector and torque vector to be set between the two connected markers.
class ChApi ChLinkMarkers : public ChLink {
  public:
    virtual ~ChLinkMarkers() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkMarkers* Clone() const override { return new ChLinkMarkers(*this); }

    /// Return the 1st referenced marker (the secondary marker, on 1st body).
    ChMarker* GetMarker1() const { return marker1; }

    /// Return the 2nd referenced marker (the main marker, on 2nd body).
    ChMarker* GetMarker2() const { return marker2; }

    /// Get the link frame 1, relative to body 1.
    /// For this class link frame 1 is actually marker 1.
    virtual ChFrame<> GetFrame1Rel() const override { return *marker1; }

    /// Get the link frame 2, relative to body 2.
    /// For this class link frame 2 is actually marker 2.
    virtual ChFrame<> GetFrame2Rel() const override { return *marker2; }

    // LINK COORDINATES and other functions

    /// Relative position of marker 1 respect to marker 2.
    const ChCoordsysd& GetRelCoordsys() const { return relM; }

    /// Relative speed of marker 1 respect to marker 2.
    const ChCoordsysd& GetRelCoordsysDt() const { return relM_dt; }

    /// Relative acceleration of marker 1 respect to marker 2.
    const ChCoordsysd& GetRelCoordsysDt2() const { return relM_dtdt; }

    /// Relative rotation angle of marker 1 respect to marker 2.
    double GetRelAngle() const { return relAngle; }

    /// Relative finite rotation axis of marker 1 respect to marker 2.
    const ChVector3d& GetRelAxis() const { return relAxis; }

    const ChVector3d& GetRelAngleAxis() const { return relRotaxis; }

    /// Relative angular speed of marker 1 respect to marker 2.
    const ChVector3d& GetRelativeAngVel() const { return relWvel; }

    /// Relative angular acceleration of marker 1 respect to marker 2.
    const ChVector3d& GetRelativeAngAcc() const { return relWacc; }

    /// Relative 'polar' distance of marker 1 respect to marker 2.
    double GetDistance() const { return dist; }

    /// Relative speed of marker 1 respect to marker 2, along the polar distance vector.
    double GetDistanceDt() const { return dist_dt; }

    /// Get the total applied force accumulators (force, momentum) in link coords.
    /// These forces might be affected by additional springs, dampers, etc. but they do not
    /// include the reaction forces.
    const ChVector3d& GetAccumulatedForce() const { return C_force; }
    const ChVector3d& GetAccumulatedTorque() const { return C_torque; }

    /// Set the two markers associated with this link.
    virtual void SetupMarkers(ChMarker* mark1, ChMarker* mark2);

    /// Initialize the link to join two markers.
    /// Each marker must belong to a rigid body, and both rigid bodies must belong to the same system.
    /// The position of mark2 is used as link's position and main reference.
    virtual void Initialize(std::shared_ptr<ChMarker> mark1,  ///< first  marker to join (secondary)
                            std::shared_ptr<ChMarker> mark2   ///< second marker to join (main)
    );

    /// Initialize the link to join two rigid bodies.
    /// Both rigid bodies must belong to the same system.
    /// This version creates the link at the specified absolute position and alignment.
    /// Two markers will be created and added to the rigid bodies.
    virtual void Initialize(std::shared_ptr<ChBody> mbody1,  ///< first  body to join
                            std::shared_ptr<ChBody> mbody2,  ///< second body to join
                            const ChFrame<>& frame           ///< initial absolute position & alignment
    );

    /// Initialize the link to join two rigid bodies.
    /// This version uses the local positions and alignments on the two bodies.
    /// Both rigid bodies must belong to the same system.
    /// Two markers will be created and added to the rigid bodies.
    virtual void Initialize(
        std::shared_ptr<ChBody> mbody1,  ///< first  body to join
        std::shared_ptr<ChBody> mbody2,  ///< second body to join
        bool rel_frames,                 ///< marker frames are relative (true) or absolute (false)
        const ChFrame<>& frame1,         ///< position & alignment of 1st marker (absolute or relative to body1)
        const ChFrame<>& frame2          ///< position & alignment of 2nd marker (absolute or relative to body2)
    );

    /// Get the reference frame (expressed in and relative to the absolute frame) of the visual model.
    /// For a ChLinkMarkers, this returns the absolute coordinate system of the main marker2.
    virtual ChFrame<> GetVisualModelFrame(unsigned int nclone = 0) const override { return marker2->GetAbsFrame(); }

    /// Updates auxiliary quantities for all relative degrees of freedom of the two markers.
    virtual void UpdateRelMarkerCoords();

    /// Updates auxiliary forces caused by springs/dampers/etc. which may be connected between the two link bodies.
    /// These forces are considered in the reference coordsystem of marker2 (the main marker), and their application
    /// point is the origin of marker1 (the secondary marker).
    virtual void UpdateForces(double mytime);

    /// Complete link update: UpdateTime -> UpdateRelMarkerCoords -> UpdateForces.
    virtual void Update(double mytime, bool update_assets = true) override;

    /// Adds force to residual R, as R*= F*c
    /// NOTE: here the off offset in R is NOT used because add F at the TWO offsets of the two connected bodies,
    /// so it is assumed that offsets for Body1 and Body2 variables have been already set properly!
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override;

    /// Overrides the empty behaviour of the parent ChLink implementation, which
    /// does not consider any user-imposed force between the two bodies.
    /// It adds the current link-forces, if any, (caused by springs, etc.) to the 'fb' vectors
    /// of the ChVariables referenced by encapsulated ChConstraints.
    /// In details, it adds the effect caused by C_force and C_torque.
    /// Both C_force and C_torque these forces are considered expressed in the
    /// reference coordsystem of marker2 (the MAIN marker),
    /// and their application point is the origin of marker1 (the SLAVE marker).
    virtual void ConstraintsFbLoadForces(double factor = 1) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  protected:
    // Protected constructors.
    ChLinkMarkers();
    ChLinkMarkers(const ChLinkMarkers& other);

    ChMarker* marker1;  ///< secondary coordsys
    ChMarker* marker2;  ///< main coordsys

    ChCoordsysd relM;       ///< relative marker position 2-1
    ChCoordsysd relM_dt;    ///< relative marker speed
    ChCoordsysd relM_dtdt;  ///< relative marker acceleration

    double relAngle;        ///< relative angle of rotation
    ChVector3d relAxis;     ///< relative axis of rotation
    ChVector3d relRotaxis;  ///< relative rotaion vector =angle*axis
    ChVector3d relWvel;     ///< relative angular speed
    ChVector3d relWacc;     ///< relative angular acceleration
    double dist;            ///< the distance between the two origins of markers,
    double dist_dt;         ///< the speed between the two  origins of markers

    ChVector3d C_force;   ///< internal force  applied by springs/dampers/actuators
    ChVector3d C_torque;  ///< internal torque applied by springs/dampers/actuators

    // Cached intermediate variables.
    // These are calculated in UpdateRelMarkerCoords and may be reused in UpdateState.
    ChVector3d PQw;
    ChVector3d PQw_dt;
    ChVector3d PQw_dtdt;
    ChQuaternion<> q_AD;
    ChQuaternion<> q_BC;
    ChQuaternion<> q_8;
    ChVector3d q_4;
};

CH_CLASS_VERSION(ChLinkMarkers, 0)

}  // end namespace chrono

#endif
