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

/// Utilities for conversions between rotation representations.
class ChApi ChRotation {
  public:
    /// Definitions of various rotation representations for conversions.
    enum class Representation {
        ANGLE_AXIS,         ///< angle-axis
        EULER_ANGLES_ZXZ,   ///< Euler angle sequence: Z - X' - Z'' (intrinsic rotations)
        CARDAN_ANGLES_ZXY,  ///< Cardan (Tait-Bryant) angle sequence: Z - X' - Y'' (intrinsic rotations)
        CARDAN_ANGLES_ZYX,  ///< Cardan (Tait-Bryant) angle sequence: Z - Y' - X'' (intrinsic rotations)
        CARDAN_ANGLES_XYZ,  ///< Cardan (Tait-Bryant) angle sequence: X - Y' - Z'' (intrinsic rotations)
        RODRIGUEZ           ///< Rodriguez parameters
    };

    // --------------------------

    /// Convert from one set of Euler or Cardan angles to another.
    static ChVector3d AngleSetToAngleSet(Representation from, Representation to, const ChVector3d& from_angles);

    /// Convert from a set of Euler angles to Rodriguez parameters.
    static ChVector3d AngleSetToRodriguez(Representation from, const ChVector3d& angles);

    /// Convert from Rodriguez parameters to a set of Euler angles.
    static ChVector3d RodriguezToAngleSet(Representation to, const ChVector3d& params);

    // --------------------------

    //// TODO: do we want these?
    //// If yes, they can be implemented using ChQuaternion functions by first separating the unit axis and the angle.
    // static ChVector3d QuaternionToAngAxis(const ChQuaterniond& q);
    // static ChQuaterniond AngAxisToQuaternion(const ChVector3d& params);
    // static ChQuaterniond AngAxisToQuaternionDer(const ChVector3d& params_der, const ChQuaterniond& q);
    // static ChQuaterniond AngAxisToQuaternionDer2(const ChVector3d& params_der2, const ChQuaterniond& q);

    // --------------------------

    /// Convert a quaternion to Rodriguez parameters.
    static ChVector3d QuaternionToRodriguez(const ChQuaterniond& q);

    /// Convert from Rodriguez parameters to a quaternion.
    static ChQuaterniond RodriguezToQuaternion(const ChVector3d& params);
    
    /// Convert a set of Rodriguez parameter derivatives to a quaternion derivative.
    static ChQuaterniond RodriguezToQuaternionDer(const ChVector3d& params_der, const ChQuaterniond& q);

    /// Convert a set of Rodriguez parameter second derivatives to a quaternion second derivative.
    static ChQuaterniond RodriguezToQuaternionDer2(const ChVector3d& params_der2, const ChQuaterniond& q);

    // --------------------------

    /// Convert a quaternion to a set of Euler angles.
    static ChVector3d QuaternionToAngleSet(Representation to, const ChQuaterniond& q);

    /// Convert a set of Euler angles to a quaternion.
    static ChQuaterniond AngleSetToQuaternion(Representation from, const ChVector3d& angles);

    /// Convert a set of Euler angle derivatives to a quaternion derivative.
    static ChQuaterniond AngleSetToQuaternionDer(Representation from,
                                                 const ChVector3d& angles_der,
                                                 const ChQuaterniond& q);

    /// Convert a set of Euler angle second derivatives to a quaternion second derivative.
    static ChQuaterniond AngleSetToQuaternionDer2(Representation from,
                                                 const ChVector3d& angles_der2,
                                                 const ChQuaterniond& q);
};

}  // end namespace chrono

#endif
