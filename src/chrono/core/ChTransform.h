//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 1996, 2005, 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHTRANSFORM_H
#define CHTRANSFORM_H

#include "chrono/core/ChVector.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChMatrix33.h"

namespace chrono {

/// ChTransform: a class for fast coordinate transformations
/// in 3D space.
///
///  A coordinate system (a 'frame') has a translation and
/// a rotation respect to a 'parent' coordinate system,
/// usually the absolute (world) coordinates.
///  This class implements useful static functions
/// to perform the typical local->parent or parent->local
/// transformations of points. Such functions are static,
/// so you don't even need to instantiate a ChTransform object,
/// you can just call functions in this way:
///   ChTransform<>::SomeFunction(..)
///

template <class Real = double>
class ChTransform {
  public:
    //
    // STATIC FUNCTIONS
    //

    // TRANSFORMATIONS, USING POSITION AND ROTATION MATRIX [A]

    /// This function transforms a point from the parent coordinate
    /// system to a local coordinate system, whose relative position
    /// is given by the 'origin' translation and 'alignment' rotation matrix.
    ///  Since the function is static, you do not need a ChTransform object, for example
    /// use it as: mresult=ChTransform<>::TransformParentToLocal(mpar, morig, malign)
    ///  This function is optimised for fast execution.
    /// \return The point in local coordinate, as local=[A]'*(parent-origin)

    static ChVector<Real> TransformParentToLocal(
        const ChVector<Real>& parent,  ///< point to transform, given in parent coordinates;
        const ChVector<Real>&
            origin,  ///< location of local frame with respect to parent, expressed in parent ref frame;
        const ChMatrix33<Real>&
            alignment  ///< rotation of local frame with respect to parent, expressed in parent coords.
        ) {
        Real mx = parent.x() - origin.x();
        Real my = parent.y() - origin.y();
        Real mz = parent.z() - origin.z();
        return ChVector<Real>(((alignment.Get33Element(0, 0)) * mx) + ((alignment.Get33Element(1, 0)) * my) +
                                  ((alignment.Get33Element(2, 0)) * mz),
                              ((alignment.Get33Element(0, 1)) * mx) + ((alignment.Get33Element(1, 1)) * my) +
                                  ((alignment.Get33Element(2, 1)) * mz),
                              ((alignment.Get33Element(0, 2)) * mx) + ((alignment.Get33Element(1, 2)) * my) +
                                  ((alignment.Get33Element(2, 2)) * mz));
    }

    /// This function transforms a point from the local reference
    /// frame to the parent reference frame. The relative attitude of
    /// the local reference frame with respect to the parent reference frame
    /// is given by the 'origin' translation and the 'alignment' rotation matrix.
    /// Since the function is static, you do not need a ChTransform object. For example,
    /// use it as: mresult=ChTransform<>::TransformLocalToParent(mloc, morig, malign).
    /// This function is optimized for fast execution.
    /// \return The point in the parent reference frame, as parent=origin +[A]*(local)

    static ChVector<Real> TransformLocalToParent(
        const ChVector<Real>& local,       ///< point to transform, given in local coordinates
        const ChVector<Real>& origin,      ///< origin of frame respect to parent, in parent coords,
        const ChMatrix33<Real>& alignment  ///< rotation of frame respect to parent, in parent coords.
        ) {
        return ChVector<Real>(((alignment.Get33Element(0, 0)) * local.x()) + ((alignment.Get33Element(0, 1)) * local.y()) +
                                  ((alignment.Get33Element(0, 2)) * local.z()) + origin.x(),
                              ((alignment.Get33Element(1, 0)) * local.x()) + ((alignment.Get33Element(1, 1)) * local.y()) +
                                  ((alignment.Get33Element(1, 2)) * local.z()) + origin.y(),
                              ((alignment.Get33Element(2, 0)) * local.x()) + ((alignment.Get33Element(2, 1)) * local.y()) +
                                  ((alignment.Get33Element(2, 2)) * local.z()) + origin.z());
    }

    // TRANSFORMATIONS, USING POSITION AND ROTATION QUATERNION

    /// This function transforms a point from the parent coordinate
    /// system to a local coordinate system, whose relative position
    /// is given by the 'origin' translation and 'alignment' quaternion q.
    ///  Since the function is static, you do not need a ChTransform object, for example
    /// use it as: mresult=ChTransform<>::TransformParentToLocal(mpar, morig, malign)
    /// \return The point in local coordinate, as local=q*[(parent-origin)]*q

    static ChVector<Real> TransformParentToLocal(
        const ChVector<Real>& parent,        ///< point to transform, given in parent coordinates
        const ChVector<Real>& origin,        ///< origin of frame respect to parent, in parent coords,
        const ChQuaternion<Real>& alignment  ///< rotation of frame respect to parent, in parent coords.
        ) {
        // It could be simply "return alignment.RotateBack(parent-origin);"
        // but for faster execution do this:
        Real e0e0 = alignment.e0() * alignment.e0();
        Real e1e1 = alignment.e1() * alignment.e1();
        Real e2e2 = alignment.e2() * alignment.e2();
        Real e3e3 = alignment.e3() * alignment.e3();
        Real e0e1 = -alignment.e0() * alignment.e1();
        Real e0e2 = -alignment.e0() * alignment.e2();
        Real e0e3 = -alignment.e0() * alignment.e3();
        Real e1e2 = alignment.e1() * alignment.e2();
        Real e1e3 = alignment.e1() * alignment.e3();
        Real e2e3 = alignment.e2() * alignment.e3();
        Real dx = parent.x() - origin.x();
        Real dy = parent.y() - origin.y();
        Real dz = parent.z() - origin.z();
        return ChVector<Real>(((e0e0 + e1e1) * 2. - 1.) * dx + ((e1e2 - e0e3) * 2.) * dy + ((e1e3 + e0e2) * 2.) * dz,
                              ((e1e2 + e0e3) * 2.) * dx + ((e0e0 + e2e2) * 2. - 1.) * dy + ((e2e3 - e0e1) * 2.) * dz,
                              ((e1e3 - e0e2) * 2.) * dx + ((e2e3 + e0e1) * 2.) * dy + ((e0e0 + e3e3) * 2. - 1.) * dz);
    }

    /// This function transforms a point from the local coordinate
    /// system to the parent coordinate system. Relative position of local respect
    /// to parent is given by the 'origin' translation and 'alignment' quaternion rotation.
    ///  Since the function is static, you do not need a ChTransform object, for example
    /// use it as: mresult=ChTransform<>::TransformLocalToParent(mloc, morig, malign)
    /// \return The point in parent coordinate, as parent=origin +q'*(local)*q

    static ChVector<Real> TransformLocalToParent(
        const ChVector<Real>& local,         ///< point to transform, given in local coordinates
        const ChVector<Real>& origin,        ///< origin of frame respect to parent, in parent coords,
        const ChQuaternion<Real>& alignment  ///< rotation of frame respect to parent, in parent coords.
        ) {
        // It could be simply: "return origin+alignment.Rotate(local);"
        // but for faster execution, do this:
        Real e0e0 = alignment.e0() * alignment.e0();
        Real e1e1 = alignment.e1() * alignment.e1();
        Real e2e2 = alignment.e2() * alignment.e2();
        Real e3e3 = alignment.e3() * alignment.e3();
        Real e0e1 = alignment.e0() * alignment.e1();
        Real e0e2 = alignment.e0() * alignment.e2();
        Real e0e3 = alignment.e0() * alignment.e3();
        Real e1e2 = alignment.e1() * alignment.e2();
        Real e1e3 = alignment.e1() * alignment.e3();
        Real e2e3 = alignment.e2() * alignment.e3();
        return ChVector<Real>(origin.x() + ((e0e0 + e1e1) * 2. - 1.) * local.x() + ((e1e2 - e0e3) * 2.) * local.y() +
                                  ((e1e3 + e0e2) * 2.) * local.z(),
                              origin.y() + ((e1e2 + e0e3) * 2.) * local.x() + ((e0e0 + e2e2) * 2. - 1.) * local.y() +
                                  ((e2e3 - e0e1) * 2.) * local.z(),
                              origin.z() + ((e1e3 - e0e2) * 2.) * local.x() + ((e2e3 + e0e1) * 2.) * local.y() +
                                  ((e0e0 + e3e3) * 2. - 1.) * local.z());
    }
};

}  // end namespace chrono

#endif
