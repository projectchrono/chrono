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
// Authors: Radu Serban
// =============================================================================
//
// Tests for Chrono coordinate transformations
//
// =============================================================================

#include <cmath>

#include "chrono/core/ChFrame.h"
#include "chrono/core/ChFrameMoving.h"
#include "chrono/core/ChTimer.h"

#include "gtest/gtest.h"

using std::cout;
using std::endl;
using namespace chrono;

const double ABS_ERR = 1e-10;

void check_vector(const ChVector3d& v1, const ChVector3d& v2, double tol) {
    ASSERT_NEAR(v1.x(), v2.x(), tol);
    ASSERT_NEAR(v1.y(), v2.y(), tol);
    ASSERT_NEAR(v1.z(), v2.z(), tol);
}

TEST(CoordsTest, local_to_global) {
    ChVector3d vtraslA(5, 6, 7);         // origin of local frame w.r.t. global frame
    ChQuaternion<> qrotA(1, 3, 4, 5);    // rotation of local frame w.r.t. global frame
    qrotA.Normalize();                   // quaternion must be normalized to represent a rotation
    ChMatrix33<> mrotA(qrotA);           // corresponding rotation matrix
    ChCoordsys<> csysA(vtraslA, qrotA);  // corresponding coordinate system (translation + rotation)
    ChFrame<> frameA(vtraslA, qrotA);    // corresponding frame

    ChVector3d vG_ref(7.5882352941, 9.0000000000, 10.6470588235);  // reference
    ChVector3d vL(2, 3, 4);                                        // point in local frame
    ChVector3d vG;                                                 // point in global frame (computed)

    vG = vtraslA + mrotA * vL;  // like:  v2 = t + [A]*v1
    cout << vG << " ..using linear algebra \n";
    check_vector(vG, vG_ref, ABS_ERR);

    vG = vtraslA + qrotA.Rotate(vL);
    cout << vG << " ..using quaternion rotation \n";
    check_vector(vG, vG_ref, ABS_ERR);

    vG = csysA.TransformPointLocalToParent(vL);
    cout << vG << " ..using a ChChCoordsys<> object \n";
    check_vector(vG, vG_ref, ABS_ERR);

    vG = frameA.TransformPointLocalToParent(vL);
    cout << vG << " ..using a ChFrame object function \n";
    check_vector(vG, vG_ref, ABS_ERR);

    vG = vL >> frameA;
    cout << vG << " ..using a ChFrame '>>' operator \n";
    check_vector(vG, vG_ref, ABS_ERR);

    vG = frameA * vL;
    cout << vG << " ..using a ChFrame '*' operator \n";
    check_vector(vG, vG_ref, ABS_ERR);
}

TEST(CoordsTest, global_to_local) {
    ChVector3d vtraslA(5, 6, 7);         // origin of local frame w.r.t. global frame
    ChQuaternion<> qrotA(1, 3, 4, 5);    // rotation of local frame w.r.t. global frame
    qrotA.Normalize();                   // quaternion must be normalized to represent a rotation
    ChMatrix33<> mrotA(qrotA);           // corresponding rotation matrix
    ChCoordsys<> csysA(vtraslA, qrotA);  // corresponding coordinate system (translation + rotation)
    ChFrame<> frameA(vtraslA, qrotA);    // corresponding frame

    ChVector3d vL_ref(2, 3, 4);                // point in local frame
    ChVector3d vG = vtraslA + mrotA * vL_ref;  // point in global frame
    ChVector3d vL;                             // point in local frame (computed)

    vL = mrotA.transpose() * (vG - vtraslA);  // like:  v1 = [A]'*(v2-t)
    cout << vL << " ..inv, using linear algebra \n";
    check_vector(vL, vL_ref, ABS_ERR);

    vL = qrotA.RotateBack(vG - vtraslA);
    cout << vL << " ..inv, using quaternion rotation \n";
    check_vector(vL, vL_ref, ABS_ERR);

    vL = csysA.TransformPointParentToLocal(vG);
    cout << vL << " ..inv, using a ChChCoordsys<> object \n";
    check_vector(vL, vL_ref, ABS_ERR);

    vL = frameA.TransformPointParentToLocal(vG);
    cout << vL << " ..inv, using a ChFrame object function \n";
    check_vector(vL, vL_ref, ABS_ERR);

    vL = vG >> frameA.GetInverse();
    cout << vL << " ..inv, using a ChFrame '>>' operator \n";
    check_vector(vL, vL_ref, ABS_ERR);

    vL = frameA.GetInverse() * vG;
    cout << vL << " ..inv, using a ChFrame '*' operator \n";
    check_vector(vL, vL_ref, ABS_ERR);

    vL = frameA / vG;
    cout << vL << " ..inv, using a ChFrame '/' operator \n";
    check_vector(vL, vL_ref, ABS_ERR);

    ChFrame<> mframeAinv(frameA);
    mframeAinv.Invert();
    vL = mframeAinv * vG;
    cout << vL << " ..inv, using an inverted ChFrame \n";
    check_vector(vL, vL_ref, ABS_ERR);
}

TEST(CoordsTest, local_to_global_3frames) {
    ChVector3d v10(5, 6, 7);
    ChQuaternion<> q10(1, 3, 4, 5);
    q10.Normalize();
    ChMatrix33<> m10(q10);

    ChVector3d v21(4, 1, 3);
    ChQuaternion<> q21(3, 2, 1, 5);
    q21.Normalize();
    ChMatrix33<> m21(q21);

    ChVector3d v32(1, 5, 1);
    ChQuaternion<> q32(4, 1, 3, 1);
    q32.Normalize();
    ChMatrix33<> m32(q32);

    ChFrame<> f10(v10, q10);
    ChFrame<> f21(v21, q21);
    ChFrame<> f32(v32, q32);

    ChVector3d mvect1(2, 3, 4);                                         // point in frame 1
    ChVector3d mvect3_ref(14.7344468652, 11.5843621399, 7.5930767369);  // point in frame 3
    ChVector3d mvect3;                                                  // point in frame 3 (computed)

    mvect3 = v10 + m10 * (v21 + m21 * (v32 + m32 * mvect1));
    printf("%16.10f  %16.10f  %16.10f\n", mvect3.x(), mvect3.y(), mvect3.z());
    cout << mvect3 << " ..triple trsf. using linear algebra \n";
    check_vector(mvect3, mvect3_ref, ABS_ERR);

    mvect3 = mvect1 >> f32 >> f21 >> f10;
    cout << mvect3 << " ..triple vector trsf. with ChFrame '>>' operator \n";
    check_vector(mvect3, mvect3_ref, ABS_ERR);

    mvect3 = f10 * f21 * f32 * mvect1;
    cout << mvect3 << " ..triple vector trsf. with ChFrame '*' operator \n";
    check_vector(mvect3, mvect3_ref, ABS_ERR);

    ChFrame<> tempf(f10 * f21 * f32);
    mvect3 = tempf * mvect1;
    cout << mvect3 << " ..triple vector trsf. with ChFrame '*' operator \n";
    check_vector(mvect3, mvect3_ref, ABS_ERR);

    // Concatenated frames

    ChFrame<> f_3(mvect1);

    ChFrame<> f_0;
    f_0 = f_3 >> f32 >> f21 >> f10;
    cout << " ..triple frame trsf. with ChFrame '>>' operator,  \n";
    cout << f_0 << "\n";
    check_vector(f_0.GetPos(), mvect3_ref, ABS_ERR);

    f_0 = f10 * f21 * f32 * f_3;
    cout << " ..triple frame trsf. with ChFrame '*' operator,  \n";
    cout << f_0 << "\n";
    check_vector(f_0.GetPos(), mvect3_ref, ABS_ERR);
}

TEST(CoordsTest, global_to_local_3frames) {
    ChVector3d v10(5, 6, 7);
    ChQuaternion<> q10(1, 3, 4, 5);
    q10.Normalize();
    ChMatrix33<> m10(q10);

    ChVector3d v21(4, 1, 3);
    ChQuaternion<> q21(3, 2, 1, 5);
    q21.Normalize();
    ChMatrix33<> m21(q21);

    ChVector3d v32(1, 5, 1);
    ChQuaternion<> q32(4, 1, 3, 1);
    q32.Normalize();
    ChMatrix33<> m32(q32);

    ChFrame<> f10(v10, q10);
    ChFrame<> f21(v21, q21);
    ChFrame<> f32(v32, q32);

    ChVector3d mvect1_ref(2, 3, 4);                    // point in frame 1
    ChVector3d mvect3 = f10 * f21 * f32 * mvect1_ref;  // point in frame 3
    ChVector3d mvect1;                                 // point in frame 1 (computed)

    mvect1 = (f10 * f21 * f32).GetInverse() * mvect3;  // inverse transf.
    cout << mvect1 << " ..inv three transf \n";
    check_vector(mvect1, mvect1_ref, ABS_ERR);

    mvect1 = f32.GetInverse() * f21.GetInverse() * f10.GetInverse() * mvect3;
    cout << mvect1 << " ..inv three transf (another method) \n";
    check_vector(mvect1, mvect1_ref, ABS_ERR);

    mvect1 = mvect3 >> (f32 >> f21 >> f10).GetInverse();
    cout << mvect1 << " ..inv three transf (another method) \n";
    check_vector(mvect1, mvect1_ref, ABS_ERR);

    mvect1 = mvect3 >> f10.GetInverse() >> f21.GetInverse() >> f32.GetInverse();
    cout << mvect1 << " ..inv three transf (another method) \n";
    check_vector(mvect1, mvect1_ref, ABS_ERR);
}
