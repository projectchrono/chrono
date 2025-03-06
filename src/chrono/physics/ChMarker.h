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

/// Markers are auxiliary reference frames attached to a rigid body and moving with the body.
/// Most often, markers are used as references to build ChLink() constraints between two rigid bodies.
/// ChMarker also allows user-defined motion laws of the marker with respect to the parent body, e.g., to represent
/// imposed trajectories.
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
             const ChCoordsys<>& rel_csys,
             const ChCoordsys<>& rel_csys_dt,
             const ChCoordsys<>& rel_csys_dtdt);
    ChMarker(const ChMarker& other);
    ~ChMarker();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChMarker* Clone() const override { return new ChMarker(*this); }

    /// Gets a pointer to the associated rigid body.
    ChBody* GetBody() const { return m_body; }

    /// Sets the parent rigid body.
    void SetBody(ChBody* body) { m_body = body; }

    /// Set body-relative marker frame and update auxiliary variables.
    /// The current position becomes the 'resting position' coordinates for the current time.
    void ImposeRelativeTransform(const ChFrame<>& frame);

    /// Set absolute coordinate marker frame and update auxiliary variables.
    /// The current position becomes the 'resting position' coordinates for the current time.
    void ImposeAbsoluteTransform(const ChFrame<>& frame);

    /// Get the 'resting position'.
    /// This is the position which the marker should have when the x,y,z motion laws are at time=0.
    const ChCoordsysd& GetRestCoordsys() const { return m_rest_csys; }

    // Body-relative coordinates

    // In order to get/set  body-relative coordinates, one can use the methods of the ChFrameMoving parent class.
    // For example, use my_marker->SetCoordsys(newpos) to impose a new position&rotation.
    // NOTE: after each modification of the frame position, speed, acceleration, etc., remember to call UpdateState()
    // to also update the absolute coordinates, i.e. the auxiliary structure returned by GetAbsFrame().

    // Absolute coordinates (auxiliary data)

    /// Get reference to the inner 'absolute frame' auxiliary
    /// coordinates. This object (coordinates/speeds/accel. of marker
    /// expressed in absolute coordinates) is useful for performance
    /// reasons. Note! it is updated only after each Update() function.
    const ChFrameMoving<double>& GetAbsFrame() const { return m_abs_frame; }

    /// Get the translation and rotation (as a ChCoordsysd) with respect to the absolute coordinates.
    const ChCoordsysd& GetAbsCoordsys() const { return m_abs_frame.GetCoordsys(); }

    /// Get the speed of translation and rotation (as a derived ChCoordsysd) with respect to the absolute coordinates.
    const ChCoordsysd& GetAbsCoordsysDt() const { return m_abs_frame.GetCoordsysDt(); }

    /// Get the acceleration of translation and rotation (as a derived ChCoordsysd) with respect to the absolute
    /// coordinates.
    const ChCoordsysd& GetAbsCoordsysDt2() const { return m_abs_frame.GetCoordsysDt2(); }

    /// Set the translation and rotation (as a ChCoordsysd) with respect to the absolute coordinates.
    /// NOTE! internal use only, for the moment. Use  ImposeAbsoluteTransform() if needed.
    void SetAbsCoordsys(const ChCoordsysd& csys) { m_abs_frame.SetCoordsys(csys); }

    /// Set the speed of translation and rotation (as a ChCoordsysd) with respect to the absolute coordinates.
    /// NOTE! internal use only, for the moment.
    void SetAbsCoordsysDt(const ChCoordsysd& csys_dt) { m_abs_frame.SetCoordsysDt(csys_dt); }

    /// Set the speed of translation and rotation (as a ChCoordsysd) with respect to the absolute coordinates.
    /// NOTE! internal use only, for the moment.
    void SetAbsCoordsysDt2(const ChCoordsysd& csys_dtdt) { m_abs_frame.SetCoordsysDt2(csys_dtdt); }

    /// Get the angular velocity with respect to global frame, expressed in absolute coordinates.
    ChVector3d GetAbsAngVel() const { return m_abs_frame.GetAngVelParent(); }

    /// Get the angular acceleration with respect to global frame, expressed in absolute coordinates.
    ChVector3d GetAbsAngAcc() const { return m_abs_frame.GetAngAccParent(); }

    // Imposed motion

    /// Set the imposed motion type (default: MotionType::FUNCTIONS).
    void SetMotionType(MotionType motion_type) { m_motion_type = motion_type; }

    /// Set the imposed motion law, for translation on X body axis.
    void SetMotionX(std::shared_ptr<ChFunction> funct);

    /// Set the imposed motion law, for translation on Y body axis.
    void SetMotionY(std::shared_ptr<ChFunction> funct);

    /// Set the imposed motion law, for translation on Z body axis.
    void SetMotionZ(std::shared_ptr<ChFunction> funct);

    /// Set the imposed motion law, for rotation about an axis.
    void SetMotionAngle(std::shared_ptr<ChFunction> funct);

    /// Set the axis of rotation, if rotation motion law is used.
    void SetMotionAxis(ChVector3d axis);

    /// Get the imposed motion type.
    MotionType GetMotionType() const { return m_motion_type; }

    /// Get imposed motion law, for translation on X body axis.
    std::shared_ptr<ChFunction> GetMotionX() const { return m_motion_X; }

    /// Get imposed motion law, for translation on Y body axis.
    std::shared_ptr<ChFunction> GetMotionY() const { return m_motion_Y; }

    /// Get imposed motion law, for translation on Z body axis.
    std::shared_ptr<ChFunction> GetMotionZ() const { return m_motion_Z; }

    /// Get imposed motion law, for rotation about an axis.
    std::shared_ptr<ChFunction> GetMotionAngle() const { return m_motion_ang; }

    /// Get the axis of rotation, if rotation motion law is used.
    ChVector3d GetMotionAxis() const { return m_motion_axis; }

    // UPDATING

    /// Update time-dependent quantities (time-varying functions for relative coordinates, if any).
    void UpdateTime(double time);

    /// Update auxiliary variables (e.g., the m_abs_frame data) at current state.
    void UpdateState();

    /// Update the time-dependent variables (e.g., function objects to impose the body-relative motion) and then update
    /// the marker state.
    virtual void Update(double time, bool update_assets) override;

    /// Update time from external signal.
    void UpdatedExternalTime(double prevtime, double mtime);

    // SERIALIZATION

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    MotionType m_motion_type;  ///< type of marker motion

    std::shared_ptr<ChFunction> m_motion_X;    ///< user imposed motion for X coord, body relative
    std::shared_ptr<ChFunction> m_motion_Y;    ///< user imposed motion for Y coord, body relative
    std::shared_ptr<ChFunction> m_motion_Z;    ///< user imposed motion for Z coord, body relative
    std::shared_ptr<ChFunction> m_motion_ang;  ///< user imposed angle rotation about axis
    ChVector3d m_motion_axis;                  ///< axis for user imposed rotation

    ChBody* m_body;  ///< parent body

    ChCoordsysd m_rest_csys;  ///< relative resting position for function=0

    // These values are set for each marker update, and are used internally if there is some external routine which
    // applies marker motion estimated by finite differencing
    ChCoordsysd m_last_rel_csys;
    ChCoordsysd m_last_rel_csys_dt;
    double m_last_time;

    // Absolute position of frame (expressed in absolute coordinate system)
    ChFrameMoving<double> m_abs_frame;  ///< absolute frame position
};

CH_CLASS_VERSION(ChMarker, 0)

}  // end namespace chrono

#endif
