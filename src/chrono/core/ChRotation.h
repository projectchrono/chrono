// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================

#ifndef CH_ROTATION_H
#define CH_ROTATION_H

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChVector3.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChMatrix33.h"
#include "chrono/utils/ChConstants.h"

namespace chrono {

/// Definitions of various rotation representations for conversions.
enum class RotRepresentation {
    ANGLE_AXIS,         ///< angle-axis
    ROTATION_VECTOR,    ///< rotation vector (vector parallel with axis and length equal to angle)
    EULER_ANGLES_ZXZ,   ///< Euler angle sequence: Z - X' - Z'' (intrinsic rotations)
    CARDAN_ANGLES_ZXY,  ///< Cardan (Tait-Bryan) angle sequence: Z - X' - Y'' (intrinsic rotations)
    CARDAN_ANGLES_ZYX,  ///< Cardan (Tait-Bryan) angle sequence: Z - Y' - X'' (intrinsic rotations)
    CARDAN_ANGLES_XYZ,  ///< Cardan (Tait-Bryan) angle sequence: X - Y' - Z'' (intrinsic rotations)
    RODRIGUES           ///< Rodrigues parameters
};

/// Representation of an Euler/Cardan angle set.
struct AngleSet {
    RotRepresentation seq;  ///< angle sequence
    ChVector3d angles;      ///< Euler/Cardan angles
};

/// Representation of a rotation as angle-axis.
struct AngleAxis {
    double angle;     ///< rotation angle
    ChVector3d axis;  ///< rotation axis
};

// --------------------------

/// Convert from one set of Euler or Cardan angles to another.
ChApi AngleSet AngleSetFromAngleSet(RotRepresentation to_seq, const AngleSet& set);

/// Convert from a set of Euler angles to Rodrigues parameters.
ChApi ChVector3d RodriguesFromAngleSet(const AngleSet& set);

/// Convert from Rodrigues parameters to a set of Euler angles.
ChApi AngleSet AngleSetFromRodrigues(RotRepresentation to_seq, const ChVector3d& params);

// --------------------------

/// Convert from a quaternion to an angle-axis pair.
ChApi AngleAxis AngleAxisFromQuat(const ChQuaterniond& q);

/// Convert from an angle-axis pair to a quaternion.
/// The axis is supposed to be fixed, i.e. it is constant during rotation.
/// The 'axis' vector must be normalized.
ChApi ChQuaterniond QuatFromAngleAxis(const AngleAxis& angle_axis);

/// Convert from an angle and an axis to a quaternion.
/// The axis is supposed to be fixed, i.e. it is constant during rotation.
/// The 'axis' vector must be normalized.
ChApi ChQuaterniond QuatFromAngleAxis(double angle, const ChVector3d& axis);

/// Convert from a speed of rotation and an axis to a quaternion derivative.
/// The rotation axis is assumed to be represented in absolute coordinates.
ChApi ChQuaterniond QuatDtFromAngleAxis(const ChQuaterniond& quat, double angle_dt, const ChVector3d& axis);

/// Convert from a rotation acceleration and an axis to a quaternion second derivative.
/// The rotation axis is assumed to be represented in absolute coordinates.
ChApi ChQuaterniond QuatDt2FromAngleAxis(double angle_dtdt,
                                         const ChVector3d& axis,
                                         const ChQuaterniond& q,
                                         const ChQuaterniond& q_dt);

/// Convert from a rotation about X axis to a quaternion.
ChApi ChQuaterniond QuatFromAngleX(double angle);

/// Convert from a rotation about Y axis to a quaternion.
ChApi ChQuaterniond QuatFromAngleY(double angle);

/// Convert from a rotation about Z axis to a quaternion.
ChApi ChQuaterniond QuatFromAngleZ(double angle);

// --------------------------

/// Convert from a quaternion to a rotation vector.
ChApi ChVector3d RotVecFromQuat(const ChQuaterniond& q);

/// Convert from a rotation vector to a quaternion.
ChApi ChQuaterniond QuatFromRotVec(const ChVector3d& vec);

// --------------------------

/// Convert from a quaternion to Rodrigues parameters.
ChApi ChVector3d RodriguesFromQuat(const ChQuaterniond& q);

/// Convert from Rodrigues parameters to a quaternion.
ChApi ChQuaterniond QuatFromRodrigues(const ChVector3d& params);

/// Convert from a set of Rodrigues parameter derivatives to a quaternion derivative.
ChApi ChQuaterniond QuatDtFromRodrigues(const ChVector3d& params, const ChQuaterniond& q);

/// Convert a set of Rodrigues parameter second derivatives to a quaternion second derivative.
ChApi ChQuaterniond QuatDt2FromRodrigues(const ChVector3d& params, const ChQuaterniond& q);

// --------------------------

/// Convert from a quaternion to a set of Euler angles.
ChApi AngleSet AngleSetFromQuat(RotRepresentation to_seq, const ChQuaterniond& q);

/// Convert from a set of Euler angles to a quaternion.
ChApi ChQuaterniond QuatFromAngleSet(const AngleSet& set);

/// Convert from a set of Euler angle derivatives to a quaternion derivative.
ChApi ChQuaterniond QuatDtFromAngleSet(const AngleSet& set, const ChQuaterniond& q);

/// Convert from a set of Euler angle second derivatives to a quaternion second derivative.
ChApi ChQuaterniond QuatDt2FromAngleSet(const AngleSet& set, const ChQuaterniond& q);

// --------------------------

/// Convert from a vector-to-vector rotation to a quaternion.
/// This quaternion represents the rotation that rotates the source vector to be aligned with the destination vector.
/// The vectors do not need to be normalized.
ChApi ChQuaterniond QuatFromVec2Vec(const ChVector3d& start, const ChVector3d& end);

}  // end namespace chrono

#endif
