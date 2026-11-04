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
// Authors: Alessandro Tasora, Radu Serban, Dario Fusai
// =============================================================================

#ifndef CHC_LINECAM_H
#define CHC_LINECAM_H

#include "chrono/geometry/ChLine.h"
#include "chrono/functions/ChFunction.h"

namespace chrono {

/// @addtogroup chrono_geometry
/// @{

/// Geometric object describing the profile of a cam.
/// The shape of a cam is specified through a ChFunction which defines the motion law of the follower.
class ChApi ChLineCam : public ChLine {
  public:
    /// Follower types.
    enum class TypeFollower {
        TRANSLATING_INLINE,  ///< inline (i.e. centered) translating follower, with knife or roller contact surface
        TRANSLATING_OFFSET,  ///< offset (i.e. eccentric) translating follower, with knife or roller contact surface
        TRANSLATING_FLAT,    ///< inline (i.e. centered) translating follower, with flat plate contact surface orthogonal to displacement
        OSCILLATING_ROLLER,  ///< inline (i.e. centered) oscillating follower, with roller contact surface
        OSCILLATING_FLAT     ///< offset (i.e. eccentric) oscillating follower, with flat plate contact surface
    };

    ChLineCam();
    ChLineCam(const ChLineCam& source);
    ~ChLineCam() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLineCam* Clone() const override { return new ChLineCam(*this); }

    /// Get the line class type as an enum.
    virtual Type GetType() const override { return Type::LINE_CAM; }

    /// Assume the cam as closed by definition.
    virtual void SetClosed(bool state) override {}
    virtual bool IsClosed() const override { return true; }

    /// Initialize data for the translating inline roller/knife follower case.
    /// Provide cam base radius and follower roller radius (if zero, assume no roller).
    void InitializeTranslatingInline(double cam_base_rad, double follower_rad);

    /// Initialize data for the translating offset roller/knife follower case.
    /// Provide cam base radius, follower roller radius (if zero, assume no roller), and the offset
    /// between the follower translation axis and cam origin.
    void InitializeTranslatingOffset(double cam_base_rad, double follower_rad, double offset);

    /// Initialize data for the translating flat follower case.
    /// Provide cam base radius.
    /// Assume translation axis inline with cam origin.
    void InitializeTranslatingFlat(double cam_base_rad);

    /// Initialize data for the oscillating roller follower case.
    /// Provide follower roller radius, follower arm length, cam-follower origins distance, and follower initial phase angle.
    /// Autocompute cam base radius.
    void InitializeOscillatingRoller(double follower_rad, double follower_arm_length, double follower_dist, double follower_angle0);

    /// Initialize data for the oscillating flat follower case.
    /// Provide follower arm offset, cam-follower origins distance, and follower initial phase angle.
    /// Autocompute cam base radius.
    void InitializeOscillatingFlat(double follower_arm_offset, double follower_dist, double follower_angle0);

    /// Get follower type.
    TypeFollower GetFollowerType() const { return type; }

    /// Set position of cam origin in XY plane.
    void SetOrigin(const ChVector2d& pt) { origin = pt; }

    /// Get position of cam origin in XY plane.
    const ChVector2d& GetOrigin() const { return origin; }

    /// Get cam base radius.
    /// User-defined for the `TRANSLATING_INLINE`, `TRANSLATING_OFFSET`, and `TRANSLATING_FLAT` cases.
    /// Autocomputed for `OSCILLATING_ROLLER` and `OSCILLATING_FLAT` cases during initialize.
    double GetBaseRadius() const { return Rb; }

    /// Set cam phase in [0, 2PI].
    void SetPhase(double ang) { phase = ang; }

    /// Get cam phase in [0, 2PI].
    double GetPhase() const { return phase; }

    /// Set motion law produced by cam, as function of angle in [0, 2PI].
    void SetMotionLaw(std::shared_ptr<ChFunction> motion_law) { motlaw = motion_law; }

    /// Get motion law produced by cam, as function of angle in [0, 2PI].
    std::shared_ptr<ChFunction> GetMotionLaw() const { return motlaw; }

    /// If true, create a negative cam.
    void SetNegative(bool val) { negative = val; }

    /// Check if the cam is negative (true) or not (false).
    bool IsNegative() const { return negative; }

    /// If true, create an internal cam.
    void SetInternal(bool val) { internal = val; }

    /// Check if cam is internal (true) or not (false).
    bool IsInternal() const { return internal; }

    /// Get follower arm length.
    /// Meaningful for `OSCILLATING_ROLLER`.
    double GetFollowerArmLength() const { return b; }

    /// Get distance between cam and follower origins.
    /// Meaningful for `OSCILLATING_ROLLER` and `OSCILLATING_FLAT` cases.
    double GetFollowerDistance() const { return d; }

    /// Get initial follower rotation.
    /// Meaningful for `OSCILLATING_ROLLER` and `OSCILLATING_FLAT` cases.
    double GetFollowerStartAngle() const { return beta0; }

    /// Get the offset (i.e. the eccentricity) of the follower.
    /// Meaningful for the `TRANSLATING_OFFSET` case (return distance of translation axis wrt cam origin)
    /// and the `OSCILLATING_FLAT` case (return arm lever offset).
    double GetFollowerOffset() const { return e; };

    /// Return a point on cam profile corresponding to parametric coordinate U in [0, 1].
    virtual ChVector3d Evaluate(double U) const override;

    /// Return a point on cam profile corresponding to parametric coordinate U in [0, 1].
    /// Additionally return pressure angle `theta` and curvature radius `rho` in U.
    ChVector3d EvaluateCam(double U, double& theta, double& rho) const;

    /// Weight evaluation at U in [0, 1].
    /// Given that the shape is defined by a ChFunction, the returned weight is
    /// the weight of the function (ChFunctionSequence can have different 'weight' values depending on the function segment).
    double GetWeight(double U) const { return motlaw->GetWeight(U * 2 * CH_PI); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    TypeFollower type;                   ///< type of cam
    ChVector2d origin;                   ///< cam origin in XY plane
    std::shared_ptr<ChFunction> motlaw;  ///< follower motion law as function of angle in [0, 2PI]
    double phase;                        ///< cam phase (rotation) in [0, 2PI]; default=0, neutral position
    double Rb;                           ///< cam base radius
    bool negative;                       ///< negative cam: for desmodromic applications (cam is also Y or X mirrored, depending on type)
    bool internal;                       ///< follower roller is inside the cam
    double Rr;                           ///< follower roller radius, meaningful for the TRANSLATING_INLINE, TRANSLATING_OFFSET, and OSCILLATING_ROLLER cases
    double e;                            ///< eccentricity of translating follower, meaningful for the `TRANSLATING_OFFSET` and `OSCILLATING_FLAT` cases
    double b;                            ///< oscillating follower data: arm length
    double d;                            ///< oscillating follower data: distance between cam origin and follower origin (i.e. arm base endpoint)
    double beta0;                        ///< oscillating follower data: initial follower rotation
};

/// @} chrono_geometry

CH_CLASS_VERSION(ChLineCam, 0)

}  // end namespace chrono

#endif