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
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CHLINKMOTIONIMPOSED_H
#define CHLINKMOTIONIMPOSED_H

#include "chrono/physics/ChLinkMate.h"
#include "chrono/motion_functions/ChFunctionPosition.h"
#include "chrono/motion_functions/ChFunctionRotation.h"

namespace chrono {

/// A joint that enforces position and rotation between two frames on two bodies, using six
/// rheonomic constraints.
/// That is, position and rotation of Body 1 are precisely enforced respect to Body 2 by using
/// functions of time, where functions are defined via ChFunctionRotation and ChFunctionPosition objects.
/// A typical usage scenario is where Body 2 is a fixed body, representing the absolute reference,
/// so you move/rotate Body 1 in space according to the provided functions; this is the case for example where
/// you have some data coming from a motion capture hardware following the motion, say, of a human head,
/// and you want to replicate the motion of that head in 3D space.
/// This joint imposes all the 6 DOFs of a body, including rotations, differently from the ChLinkTrajectory
/// that imposes only the position but leaves rotation free.
/// Note that if you are interested in simply imposing a straight motion, just use  ChLinkMotorLinearPosition,
/// and if you just need to impose a rotation on a shaft, just use the ChLinkMotorRotationAngle.
/// Note: no compliance is allowed, so if the imposed motion hits an undeformable obstacle it mets a pathological
/// situation and the solver result can be unstable/unpredictable.
/// Think at it as a 6DOF servo drive with "infinitely stiff" control.
/// By default it is initialized with no motion / no rotation, so by default is just like welding the two bodies,
/// so it is up to the user to provide proper ChFunctionRotation and a proper ChFunctionPosition.
/// Note: differently from most other ChLinkMate -inherited links, that assume the frame2 as link coodrinate system,
/// in this case we use the "moving" auxiliary frame M whose motion is concatenated to frame2 (in absolute coordinates,
/// such moving frame will be cohincident with frame1 if the constraint is well assembled - in fact this makes
/// constraint forces more intuitive, as one will get reaction forces and torques as applied to the frame1 of moving
/// body 1).
class ChApi ChLinkMotionImposed : public ChLinkMateGeneric {
  public:
    ChLinkMotionImposed();
    ChLinkMotionImposed(const ChLinkMotionImposed& other);
    virtual ~ChLinkMotionImposed();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkMotionImposed* Clone() const override { return new ChLinkMotionImposed(*this); }

	/// Set the position function for a generic 3D rotation, as p=p(t) vector function.
	/// Use an object from one of the ChFunctionPosition inherited classes to this end.
    /// This function should be C0 continuous and, to prevent acceleration spikes,
    /// it should ideally be C1 continuous.
	/// The position is imposed to frame1 respect to frame2, in frame2 coordinate system.
    void SetPositionFunction(std::shared_ptr<ChFunctionPosition> mf) { position_function = mf; }

	/// Get the position function q=q(t).
    std::shared_ptr<ChFunctionPosition> GetPositionFunction() const { return position_function; }

    /// Set the rotation function for a generic 3D rotation, as  q=q(t) quaternion rotation function.
	/// Use an object from one of the ChFunctionRotation inherited classes to this end.
    /// This function should be C0 continuous and, to prevent acceleration spikes,
    /// it should ideally be C1 continuous.
	/// The position is imposed to frame1 respect to frame2, in frame2 coordinate system.
    void SetRotationFunction(std::shared_ptr<ChFunctionRotation> mf) { rotation_function = mf; }

    /// Get the rotation function q=q(t).
    std::shared_ptr<ChFunctionRotation> GetRotationFunction() const { return rotation_function; }

	/// For plotting etc: get the last computed value of imposed rotation and position,
	/// expressed as the "moving" ChFrame M respect to frame2.
	ChFrame<>& GetFrameM2() { return frameM2; }

	/// Get the link coordinate system, expressed relative to Body2 (the 'master' body). This represents the 'main'
    /// reference of the link: reaction forces and reaction torques are expressed in this coordinate system. 
	/// Note: differently from most other ChLinkMate -inherited links, that assume the frame2 as link coodrinate system,
	/// in this case we use the "moving" auxiliary frame M whose motion is concatenated to frame2 (in absolute coordinates,
	/// such moving frame will be cohincident with frame1 if the constraint is well assembled - in fact this makes constraint
	/// forces more intuitive, as one will get reaction forces and torques as applied to the frame1 of moving body 1).
    virtual ChCoordsys<> GetLinkRelativeCoords() override { return (frameMb2).GetCoord(); }

    void Update(double mytime, bool update_assets) override;

    //
    // STATE FUNCTIONS
    //
    virtual void IntLoadConstraint_Ct(const unsigned int off, ChVectorDynamic<>& Qc, const double c) override;

    //
    // SOLVER INTERFACE (OLD)
    //
    virtual void ConstraintsBiLoad_Ct(double factor = 1) override;

    /// Add the current stiffness K matrix in encapsulated ChKblock item(s), if any.
    /// The K matrices are load with scaling values Kfactor.
    virtual void KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

  private:

	std::shared_ptr<ChFunctionPosition> position_function;
	std::shared_ptr<ChFunctionRotation> rotation_function;
	ChFrame<> frameM2; // last updated value of the moving frame M respect to frame 2 (the frame connected to body 2)
	ChFrame<> frameMb2; // last updated value of the moving frame M respect to body 2 
}; 

CH_CLASS_VERSION(ChLinkMotionImposed, 0)

}  // end namespace chrono

#endif
