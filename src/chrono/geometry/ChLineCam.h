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

#ifndef CHC_LINECAM_H
#define CHC_LINECAM_H

#include <cmath>

#include "chrono/geometry/ChLine.h"
#include "chrono/functions/ChFunction.h"

namespace chrono {

/// @addtogroup chrono_geometry
/// @{

/// Geometric object describing the profile of a cam.
/// The shape of a cam is specified through a ChFunction which defines the motion law of the follower.
class ChApi ChLineCam : public ChLine {
  public:
    /// Cam types.
    enum class CamType { SLIDEFOLLOWER, ROTATEFOLLOWER, ECCENTRICFOLLOWER, FLAT, FLATOSCILLATE };

    ChLineCam();
    ChLineCam(const ChLineCam& source);
    ~ChLineCam() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLineCam* Clone() const override { return new ChLineCam(*this); }

    /// Get the class type as an enum.
    virtual Type GetType() const override { return Type::LINE_CAM; }

    /// Set the cam type.
    void SetCamType(CamType t) { type = t; }

    /// Get the cam type.
    CamType GetCamType() const { return type; }

    virtual void SetClosed(bool state) override {}

    virtual bool IsClosed() const override { return true; }

    void SetPhase(double f) { phase = f; }

    double GetPhase() const { return phase; }

    /// Base radius of cam
    void SetCamRadius(double r);

    double GetCamRadius() const { return Rb; }

    /// Radius of contact wheel
    void SetWheelRadius(double r) { Rr = r; }

    double GetWheelRadius() const { return Rr; }

    /// The motion law, as a ChFunction.
    void SetMotionLaw(std::shared_ptr<ChFunction> motion_law) { law = motion_law; }

    std::shared_ptr<ChFunction> GetMotionLaw() const { return law; }

    /// Position of center of cam in 3d space.
    void SetCenter(ChVector3d mc) { center = mc; }
    ChVector3d GetCenter() const { return center; }

    /// If true, create a negative cam.
    void SetNegative(bool val) { negative = val; }

    bool IsNegative() const { return negative; }

    /// If true, creates an internal cam.
    void SetInternal(bool val) { internal = val; }

    bool IsInternal() const { return internal; }

    /// Sets the data for the rotating follower (length, distance from cam center, initial phase mb0)
    void SetRotatingFollower(double mp, double md, double mb0);

    double GetVal() const { return p; }
    double GetFollowerDistance() const { return d; }
    double GetFollowerInitPhase() const { return b0; }

    /// Sets the data for the sliding follower (if eccentric, with me eccentricity)
    void SetSlidingEccentrical(double me) { e = me; };

    /// Sets the data for the flat rotating follower (length, distance from cam center, initial phase mb0)
    void SetFlatOscillate(double me, double md, double mb0);

    /// Evaluate at once all important properties of cam, function of rotation 'par'
    /// (par in range 0..1, with 1 corresponding to 360 degrees):
    /// Also returns the pressure angle g and curvature radius q.
    ChVector3d EvaluateCamPoint(double par, double& g, double& q) const;

    /// Return a point on the line, given parametric coordinate U (in [0,1]).
    virtual ChVector3d Evaluate(double U) const override;

    /// Weight evaluation.
    /// Given that the shape is defined by a Ch_function, the
    /// returned weight is the weight of the function (Ch_function_sequence can
    /// have different 'weight' values depending on the function segment)
    double GetWeight(double par) const { return law->GetWeight(par * 2 * CH_PI); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    CamType type;                     ///< type of cam
    std::shared_ptr<ChFunction> law;  ///< follower motion law
    double phase;                     ///< 0..2PI  phase (rotation). Def=0, neutral position

    double Rb;  ///< base radius
    double Rr;  ///< radius of contact wheel

    double p;   ///< length of rotating mover
    double d;   ///< distance center of cam - center of rotation of mover
    double b0;  ///< initial rotation for mover

    double e;  ///< eccentricity of sliding follower
    double s;  ///< distance of sliding follower

    bool negative;  ///< negative cam: for desmodromic stuff, (cam is also Y or X mirrored, depend.on type )
    bool internal;  ///< follower roller is inside the cam

    ChVector3d center;  ///< center of cam in space (def.alignment on xy plane)
};

/// @} chrono_geometry

CH_CLASS_VERSION(ChLineCam, 0)

}  // end namespace chrono

#endif