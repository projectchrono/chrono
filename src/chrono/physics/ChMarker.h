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

#ifndef CHMARKER_H
#define CHMARKER_H

#include <cstdlib>
#include <iostream>

#include "chrono/core/ChFrameMoving.h"
#include "chrono/core/ChFrame.h"
#include "chrono/functions/ChFunction.h"
#include "chrono/physics/ChObject.h"

namespace chrono {

// Forward reference
class ChBody;

/// Markers are auxiliary reference frames which belong to rigid bodies and move
/// together with them. Most often, markers are used as references to build
/// ChLink() constraints between two rigid bodies.
/// The ChMarker objects allow also to user-define a motion law of marker respect
/// to parent ChBody, if needed to represent imposed trajectories etc.

class ChApi ChMarker : public ChObj, public ChFrameMoving<double> {
  public:
    enum class MotionType {
        FUNCTIONS,  ///< marker uses its own x, y, z functions
        KEYFRAMED,  ///< marker moved via external key frames (derivatives obtained with finite differences)
        EXTERNAL,   ///< marker moved via external functions (derivatives provided)
    };

    ChMarker();
    ChMarker(const std::string& name,
             ChBody* body,
             const ChCoordsysd& rel_pos,
             const ChCoordsysd& rel_pos_dt,
             const ChCoordsysd& rel_pos_dtdt);
    ChMarker(const ChMarker& other);
    ~ChMarker();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChMarker* Clone() const override { return new ChMarker(*this); }

    /// Gets a pointer to the associated rigid body.
    ChBody* GetBody() const { return Body; }

    /// Sets the parent rigid body.
    void SetBody(ChBody* newRB) { Body = newRB; }

    /// Set body-relative marker frame and update auxiliary variables.
    /// The current position becomes the 'resting position' coordinates for the current time.
    void ImposeRelativeTransform(const ChFrame<>& frame);

    /// Set absolute coordinate marker frame and update auxiliary variables.
    /// The current position becomes the 'resting position' coordinates for the current time.
    void ImposeAbsoluteTransform(const ChFrame<>& frame);

    /// Get the 'resting position'.
    /// This is the position which the marker should have when the x,y,z motion laws are at time=0.
    const ChCoordsysd& GetRestCsys() const { return rest_coord; }

    // Body-relative coordinates

    // In order to get/set  body-relative coordinates, you can use the methods of the ChFrameMoving parent class: for
    // example use  my_marker->SetCsys(newpos) to impose a new position&rotation, etc. NOTE!! after each modification of
    // the frame position, speed, acceleration, etc., remember to call UpdateState() if you want to kee updated also the
    // absolute coordinates , ie. the auxiliary structure to get with GetAbsFrame().

    // Absolute coordinates (auxiliary data)

    /// Get reference to the inner 'absolute frame' auxiliary
    /// coordinates. This object (coordinates/speeds/accel. of marker
    /// expressed in absolute coordinates) is useful for performance
    /// reasons. Note! it is updated only after each Update() function.
    const ChFrameMoving<double>& GetAbsFrame() const { return abs_frame; }

    /// Get the translation and rotation (as a ChCoordsysd) with respect to the absolute coordinates.
    const ChCoordsysd& GetAbsCsys() const { return abs_frame.GetCsys(); }

    /// Get the speed of translation and rotation (as a derived ChCoordsysd) with respect to the absolute coordinates.
    const ChCoordsysd& GetAbsCsysDer() const { return abs_frame.GetCsysDer(); }

    /// Get the acceleration of translation and rotation (as a derived ChCoordsysd) with respect to the absolute
    /// coordinates.
    const ChCoordsysd& GetAbsCsysDer2() const { return abs_frame.GetCsysDer2(); }

    /// Set the translation and rotation (as a ChCoordsysd) with respect to the absolute coordinates.
    /// NOTE! internal use only, for the moment. Use  ImposeAbsoluteTransform() if needed.
    void SetAbsCsys(const ChCoordsysd& newpos) { abs_frame.SetCsys(newpos); }

    /// Set the speed of translation and rotation (as a ChCoordsysd) with respect to the absolute coordinates.
    /// NOTE! internal use only, for the moment.
    void SetAbsCsysDer(const ChCoordsysd& newpos_dt) { abs_frame.SetCsysDer(newpos_dt); }

    /// Set the speed of translation and rotation (as a ChCoordsysd) with respect to the absolute coordinates.
    /// NOTE! internal use only, for the moment.
    void SetAbsCsysDer2(const ChCoordsysd& newpos_dtdt) { abs_frame.SetCsysDer2(newpos_dtdt); }

    /// Get the angular velocity with respect to global frame, expressed in absolute coordinates.
    ChVector3d GetAbsAngVel() const { return abs_frame.GetAngVelParent(); }

    /// Get the angular acceleration with respect to global frame, expressed in absolute coordinates.
    ChVector3d GetAbsAngAcc() const { return abs_frame.GetAngAccParent(); }

    // Imposed motion

    /// Set the imposed motion type (default: MotionType::FUNCTIONS).
    void SetMotionType(MotionType m_motion) { motion_type = m_motion; }

    /// Set the imposed motion law, for translation on X body axis.
    void SetMotionAxisX(std::shared_ptr<ChFunction> funct);

    /// Set the imposed motion law, for translation on Y body axis.
    void SetMotionAxisY(std::shared_ptr<ChFunction> funct);

    /// Set the imposed motion law, for translation on Z body axis.
    void SetMotionAxisZ(std::shared_ptr<ChFunction> funct);

    /// Set the imposed motion law, for rotation about an axis.
    void SetMotionAngle(std::shared_ptr<ChFunction> funct);

    /// Set the axis of rotation, if rotation motion law is used.
    void SetMotionAxis(ChVector3d axis);

    /// Get the imposed motion type.
    MotionType GetMotionType() const { return motion_type; }

    /// Get imposed motion law, for translation on X body axis.
    std::shared_ptr<ChFunction> GetMotionAxisX() const { return motion_X; }

    /// Get imposed motion law, for translation on Y body axis.
    std::shared_ptr<ChFunction> GetMotionAxisY() const { return motion_Y; }

    /// Get imposed motion law, for translation on Z body axis.
    std::shared_ptr<ChFunction> GetMotionAxisZ() const { return motion_Z; }

    /// Get imposed motion law, for rotation about an axis.
    std::shared_ptr<ChFunction> GetMotionAngle() const { return motion_ang; }

    /// Get the axis of rotation, if rotation motion law is used.
    ChVector3d GetMotionAxis() const { return motion_axis; }

    // UPDATING

    /// Update the time-dependent variables (e.g., function objects to impose the body-relative motion).
    void UpdateTime(double mytime);

    /// Update auxiliary variables (e.g., the abs_frame data) at current state.
    void UpdateState();

    /// Call UpdateTime() and UpdateState() at once.
    void Update(double mytime);

    /// Update time from external signal.
    void UpdatedExternalTime(double prevtime, double mtime);

    // SERIALIZATION

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    MotionType motion_type;  ///< type of marker motion

    std::shared_ptr<ChFunction> motion_X;    ///< user imposed motion for X coord, body relative
    std::shared_ptr<ChFunction> motion_Y;    ///< user imposed motion for Y coord, body relative
    std::shared_ptr<ChFunction> motion_Z;    ///< user imposed motion for Z coord, body relative
    std::shared_ptr<ChFunction> motion_ang;  ///< user imposed angle rotation about axis
    ChVector3d motion_axis;                  ///< axis for user imposed rotation

    ChBody* Body;  ///< points to parent body

    ChCoordsysd rest_coord;  ///< relative resting position for function=0.

    // These values are set for each marker update, and are used internally if there's some external routine which
    // applies marker motion estimated by finite differencing
    ChCoordsysd last_rel_coord;
    ChCoordsysd last_rel_coord_dt;
    double last_time;

    // Absolute position of frame (expressed in absolute coordinate system)
    ChFrameMoving<double> abs_frame;  ///< absolute frame position
};

CH_CLASS_VERSION(ChMarker, 0)

}  // end namespace chrono

#endif
