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

// =============================================================================
// ChLineCam
// =============================================================================
/// Base class to represent the geometric profile of a cam.
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
    virtual ~ChLineCam() {}

    ///// "Virtual" copy constructor (covariant return type).
    // virtual ChLineCam* Clone() const override { return new ChLineCam(*this); }

    /// Get the line class type as an enum.
    virtual Type GetType() const override { return Type::LINE_CAM; }

    /// Assume the cam as closed by definition.
    virtual void SetClosed(bool state) override {}
    virtual bool IsClosed() const override { return true; }

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

    /// Return a point on cam profile corresponding to parametric coordinate U in [0, 1].
    virtual ChVector3d Evaluate(double U) const override;

    /// Return a point on cam profile corresponding to parametric coordinate U in [0, 1].
    /// Additionally return pressure angle `theta` and curvature radius `rho` in U.
    virtual ChVector3d EvaluateCam(double U, double& theta, double& rho) const = 0;

    /// Weight evaluation at U in [0, 1].
    /// Given that the shape is defined by a ChFunction, the returned weight is
    /// the weight of the function (ChFunctionSequence can have different 'weight' values depending on the function segment).
    double GetWeight(double U) const { return motlaw->GetWeight(U * 2 * CH_PI); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  protected:
    ChVector3d CalcProfilePoint(double r, double phi) const;

    TypeFollower type = TypeFollower::TRANSLATING_INLINE;  ///< type of cam
    ChVector2d origin;                                     ///< cam origin in XY plane
    std::shared_ptr<ChFunction> motlaw;                    ///< follower motion law as function of angle in [0, 2PI]
    double phase = 0;                                      ///< cam phase (rotation) in [0, 2PI]; default=0, neutral position
    double Rb = 0;                                         ///< cam base radius
    bool negative = false;                                 ///< negative cam: for desmodromic applications (cam is also Y or X mirrored, depending on type)
    bool internal = false;                                 ///< follower roller is inside the cam
};

// =============================================================================
// ChLineCamTranslatingInline
// =============================================================================
/// Class to represent the profile of a cam with a translating follower (roller or knife), inline with cam origin.
class ChApi ChLineCamTranslatingInline : public ChLineCam {
  public:
    ChLineCamTranslatingInline() {}

    ChLineCamTranslatingInline(double cam_base_rad, double follower_rad);

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLineCamTranslatingInline* Clone() const override { return new ChLineCamTranslatingInline(*this); }

    /// Initialize data for the translating offset roller/knife follower case.
    /// Provide cam base radius, follower roller radius (if zero, assume no roller), and the offset
    /// between the follower translation axis and cam origin.
    void Initialize(double cam_base_rad, double follower_rad);

    /// Return a point on cam profile corresponding to parametric coordinate U in [0, 1].
    /// Additionally return pressure angle `theta` and curvature radius `rho` in U.
    ChVector3d EvaluateCam(double U, double& theta, double& rho) const;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    double Rr;  ///< follower roller radius
};

// =============================================================================
// ChLineCamTranslatingOffset
// =============================================================================
/// Class to represent the profile of a cam with a translating follower (roller or knife), offset from cam origin.
class ChApi ChLineCamTranslatingOffset : public ChLineCam {
  public:
    ChLineCamTranslatingOffset() {}

    ChLineCamTranslatingOffset(double cam_base_rad, double follower_rad, double offset);

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLineCamTranslatingOffset* Clone() const override { return new ChLineCamTranslatingOffset(*this); }

    /// Initialize data for the translating offset roller/knife follower case.
    /// Provide cam base radius, follower roller radius (if zero, assume no roller), and the offset
    /// between the follower translation axis and cam origin.
    void Initialize(double cam_base_rad, double follower_rad, double offset);

    /// Get the offset (i.e. the eccentricity) of the follower.
    double GetFollowerOffset() const { return e; };

    /// Return a point on cam profile corresponding to parametric coordinate U in [0, 1].
    /// Additionally return pressure angle `theta` and curvature radius `rho` in U.
    ChVector3d EvaluateCam(double U, double& theta, double& rho) const;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    double Rr;  ///< follower roller radius
    double e;   ///< eccentricity of translating follower
};

// =============================================================================
// ChLineCamTranslatingFlat
// =============================================================================
/// Class to represent the profile of a cam with a translating flat follower, inline with cam origin.
class ChApi ChLineCamTranslatingFlat : public ChLineCam {
  public:
    ChLineCamTranslatingFlat() {}

    ChLineCamTranslatingFlat(double cam_base_rad);

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLineCamTranslatingFlat* Clone() const override { return new ChLineCamTranslatingFlat(*this); }

    /// Initialize data for the translating flat follower case.
    /// Provide cam base radius.
    /// Assume translation axis inline with cam origin.
    void Initialize(double cam_base_rad);

    /// Return a point on cam profile corresponding to parametric coordinate U in [0, 1].
    /// Additionally return pressure angle `theta` and curvature radius `rho` in U.
    ChVector3d EvaluateCam(double U, double& theta, double& rho) const;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
};

// =============================================================================
// ChLineCamOscillatingRoller
// =============================================================================
/// Class to represent the profile of a cam with an oscillating follower (roller or knife).
class ChApi ChLineCamOscillatingRoller : public ChLineCam {
  public:
    ChLineCamOscillatingRoller() {}

    ChLineCamOscillatingRoller(double follower_rad, double follower_arm_length, double follower_dist, double follower_angle0);

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLineCamOscillatingRoller* Clone() const override { return new ChLineCamOscillatingRoller(*this); }

    /// Initialize data for the oscillating roller follower case.
    /// Provide follower roller radius, follower arm length, cam-follower origins distance, and follower initial phase angle.
    /// Autocompute cam base radius.
    void Initialize(double follower_rad, double follower_arm_length, double follower_dist, double follower_angle0);

    /// Get follower arm length.
    double GetFollowerArmLength() const { return b; }

    /// Get distance between cam and follower origins.
    double GetFollowerDistance() const { return d; }

    /// Get initial follower rotation.
    double GetFollowerStartAngle() const { return beta0; }

    /// Return a point on cam profile corresponding to parametric coordinate U in [0, 1].
    /// Additionally return pressure angle `theta` and curvature radius `rho` in U.
    ChVector3d EvaluateCam(double U, double& theta, double& rho) const;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    double Rr;     ///< follower roller radius
    double b;      ///< arm length
    double d;      ///< distance between cam origin and follower origin (i.e. arm base endpoint)
    double beta0;  ///< initial follower rotation
};

// =============================================================================
// ChLineCamOscillatingFlat
// =============================================================================
/// Class to represent the profile of a cam with an oscillating flat follower.
class ChApi ChLineCamOscillatingFlat : public ChLineCam {
  public:
    ChLineCamOscillatingFlat() {}

    ChLineCamOscillatingFlat(double follower_arm_offset, double follower_dist, double follower_angle0) { Initialize(follower_arm_offset, follower_dist, follower_angle0); }

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLineCamOscillatingFlat* Clone() const override { return new ChLineCamOscillatingFlat(*this); }

    /// Initialize data for the oscillating flat follower case.
    /// Provide follower arm offset, cam-follower origins distance, and follower initial phase angle.
    /// Autocompute cam base radius.
    void Initialize(double follower_arm_offset, double follower_dist, double follower_angle0);

    /// Get distance between cam and follower origins.
    double GetFollowerDistance() const { return d; }

    /// Get initial follower rotation.
    double GetFollowerStartAngle() const { return beta0; }

    /// Return a point on cam profile corresponding to parametric coordinate U in [0, 1].
    /// Additionally return pressure angle `theta` and curvature radius `rho` in U.
    ChVector3d EvaluateCam(double U, double& theta, double& rho) const;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    double e;      ///< eccentricity of translating follower
    double d;      ///< distance between cam origin and follower origin (i.e. arm base endpoint)
    double beta0;  ///< initial follower rotation
};

/// @} chrono_geometry

CH_CLASS_VERSION(ChLineCam, 0)

}  // end namespace chrono

#endif