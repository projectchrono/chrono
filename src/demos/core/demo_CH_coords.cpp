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
//
// Demo on how to use Chrono coordinate transformations
//
// =============================================================================

#include <cmath>

#include "chrono/core/ChFrame.h"
#include "chrono/core/ChFrameMoving.h"
#include "chrono/core/ChTimer.h"
#include "chrono/core/ChRotation.h"

using namespace chrono;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    std::cout << "CHRONO demo about coordinate transformations:\n" << std::endl;

    // Some methods to achieve coordinate transformations, and some examples of how to manipulate coordinates and
    // frames, using Chrono features.
    // You can use ChChCoordsys or ChFrame functions to transform points from/to local coordinates in 3D.

    ChVector3d mvect2;  // resulting (transformed) vectors will go here
    ChVector3d mvect3;

    // Define a  POINT  to be transformed, expressed in
    // local frame coordinate.
    ChVector3d mvect1(2, 3, 4);

    // Define a vector representing the TRANSLATION of the frame
    // respect to absolute (world) coordinates.
    ChVector3d vtraslA(5, 6, 7);

    // Define a quaternion representing the ROTATION of the frame
    // respect to absolute (world) coordinates. Must be normalized.
    ChQuaternion<> qrotA(1, 3, 4, 5);
    qrotA.Normalize();

    // ..Also create a 3x3 rotation matrix [A] from the quaternion
    // (at any time you can use mrotA.SetFromQuaternion(qrotA) );
    ChMatrix33<> mrotA(qrotA);

    // ..Also create a ChCoordsys<>tem object, representing both
    // translation and rotation.
    ChCoordsys<> csysA(vtraslA, qrotA);

    // OK!!! Now we are ready to perform the transformation, like in
    // linear algebra formula v'=t+[A]*v, so that we will obtain
    // the coordinates of mvect1 in absolute coordinates.
    // This can be achieved in many ways. Let's see them.

    // TRANSFORM USING ROTATION MATRIX AND LINEAR ALGEBRA
    //
    mvect2 = vtraslA + mrotA * mvect1;  // like:  v2 = t + [A]*v1
    std::cout << mvect2 << " ..using linear algebra" << std::endl;

    // TRANSFORM USING QUATERNION ROTATION

    mvect2 = vtraslA + qrotA.Rotate(mvect1);
    std::cout << mvect2 << " ..using quaternion rotation" << std::endl;

    // TRANSFORM USING A ChCoordys OBJECT

    mvect2 = csysA.TransformPointLocalToParent(mvect1);
    std::cout << mvect2 << " ..using a ChChCoordsys<> object" << std::endl;

    mvect2 = mvect1 >> csysA;
    std::cout << mvect2 << " ..using a ChChCoordsys<> '>>' operator" << std::endl;

    mvect2 = csysA * mvect1;
    std::cout << mvect2 << " ..using a ChChCoordsys<> '*' operator" << std::endl;

    // TRANSFORM USING A ChFrame OBJECT

    ChFrame<> mframeA(vtraslA, qrotA);  // or ChFrame<> mframeA(csysA);

    mvect2 = mframeA.TransformPointLocalToParent(mvect1);
    std::cout << mvect2 << " ..using a ChFrame object function" << std::endl;

    mvect2 = mvect1 >> mframeA;
    std::cout << mvect2 << " ..using a ChFrame '>>' operator" << std::endl;

    mvect2 = mframeA * mvect1;
    std::cout << mvect2 << " ..using a ChFrame '*' operator" << std::endl;

    //
    // Now perform transformations in a chain of frames, in
    // sequence.
    //

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

    // ...with linear algebra:

    mvect3 = v10 + m10 * (v21 + m21 * (v32 + m32 * mvect1));
    std::cout << mvect3 << " ..triple trsf. using linear algebra" << std::endl;

    // ...with ChFrame '>>' operator or "*" operator
    // is by far much simplier!

    ChFrame<> f10(v10, q10);
    ChFrame<> f21(v21, q21);
    ChFrame<> f32(v32, q32);

    mvect3 = mvect1 >> f32 >> f21 >> f10;
    std::cout << mvect3 << " ..triple vector trsf. with ChFrame '>>' operator" << std::endl;

    mvect3 = f10 * f21 * f32 * mvect1;
    std::cout << mvect3 << " ..triple vector trsf. with ChFrame '*' operator" << std::endl;

    ChFrame<> tempf(f10 * f21 * f32);
    mvect3 = tempf * mvect1;
    std::cout << mvect3 << " ..triple vector trsf. with ChFrame '*' operator" << std::endl;

    // Not only vectors, but also ChFrame can be transformed
    // with ">>" or "*" operators.

    ChFrame<> f_3(mvect1);
    ChFrame<> f_0;
    f_0 = f_3 >> f32 >> f21 >> f10;
    std::cout << f_0 << " ..triple frame trsf. with ChFrame '>>' operator" << std::endl;

    f_0 = f10 * f21 * f32 * f_3;
    std::cout << f_0 << " ..triple frame trsf. with ChFrame '*' operator" << std::endl;

    // Test the  ">>" or "*" operators also for ChCoordsys:
    ChCoordsys<> c_0;
    c_0 = f_3.GetCoordsys() >> f32.GetCoordsys() >> f21.GetCoordsys() >> f10.GetCoordsys();
    std::cout << f_0 << " ..triple frame trsf. with ChCoordsys '>>' operator" << std::endl;

    c_0 = f10.GetCoordsys() * f21.GetCoordsys() * f32.GetCoordsys() * f_3.GetCoordsys();
    std::cout << f_0 << " ..triple frame trsf. with ChCoordsys '*' operator" << std::endl;

    //
    // Now test inverse transformations too.
    //
    // From the low-level to the higher level methods, here are some
    // ways to accomplish this.
    //

    // TRANSFORM USING ROTATION MATRIX AND LINEAR ALGEBRA
    //

    std::cout << mvect1 << " ..mvect1" << std::endl;
    mvect1 = mrotA.transpose() * (mvect2 - vtraslA);  // like:  v1 = [A]'*(v2-t)
    std::cout << mvect1 << " ..inv, using linear algebra" << std::endl;

    // TRANSFORM USING QUATERNION ROTATION

    mvect1 = qrotA.RotateBack(mvect2 - vtraslA);
    std::cout << mvect1 << " ..inv, using quaternion rotation" << std::endl;

    // TRANSFORM USING A ChCoordys OBJECT

    mvect1 = csysA.TransformPointParentToLocal(mvect2);
    std::cout << mvect1 << " ..inv, using a ChChCoordsys<> object" << std::endl;

    // TRANSFORM USING A ChFrame OBJECT

    mvect1 = mframeA.TransformPointParentToLocal(mvect2);
    std::cout << mvect1 << " ..inv, using a ChFrame object function" << std::endl;

    mvect1 = mvect2 >> mframeA.GetInverse();
    std::cout << mvect1 << " ..inv, using a ChFrame inverse and '>>' operator" << std::endl;

    mvect1 = mframeA.GetInverse() * mvect2;
    std::cout << mvect1 << " ..inv, using a ChFrame inverse and  '*' operator" << std::endl;

    mvect1 = mframeA / mvect2;
    std::cout << mvect1 << " ..inv, using a ChFrame '/' operator" << std::endl;

    ChFrame<> mframeAinv(mframeA);
    mframeAinv.Invert();
    mvect1 = mframeAinv * mvect2;
    std::cout << mvect1 << " ..inv, using an inverted ChFrame" << std::endl;

    // ... also for inverting chain of transformations...

    // mvect3 =  f10 * f21 * f32 * mvect1;				// direct transf..

    mvect1 = (f10 * f21 * f32).GetInverse() * mvect3;  // inverse transf.
    std::cout << mvect1 << " ..inv three transf" << std::endl;

    mvect1 = f32.GetInverse() * f21.GetInverse() * f10.GetInverse() * mvect3;
    std::cout << mvect1 << " ..inv three transf (another method)" << std::endl;

    mvect1 = mvect3 >> (f32 >> f21 >> f10).GetInverse();
    std::cout << mvect1 << " ..inv three transf (another method)" << std::endl;

    mvect1 = mvect3 >> f10.GetInverse() >> f21.GetInverse() >> f32.GetInverse();
    std::cout << mvect1 << " ..inv three transf (another method)" << std::endl;

    //
    // Now test the * and >> operators with some mixed-types operators
    //

    ChFrame<> mframeA1(vtraslA, qrotA);
    ChFrameMoving<> mframemovingB1(vtraslA, qrotA);
    ChFrame<> mresf = mframemovingB1 * mframeA1;
    ChFrameMoving<> mresg = mframeA1 * mframemovingB1;

    //
    // Test some in-place operators, even with mixed-types. Some examples.
    //

    // Transform mframeA1 by rotating & translating by another frame,
    // using the in-place >>= operator, as in  A >>= B,
    // that means:   frameA' = frameA >> frameB  = frameB * frameA
    mframeA1 >>= f10;

    // Transform mframeA1 by translating by a vector:
    mframeA1 >>= ChVector3d(1, 2, 3);

    // Transform mframeA1 by rotating it 30 degrees on axis Y, using a quaternion:
    mframeA1 >>= QuatFromAngleY(30 * CH_DEG_TO_RAD);

    //
    // BENCHMARK FOR EXECUTION SPEED
    //

    std::cout << "-------------------------------------------\n" << std::endl;

    mrotA.SetFromQuaternion(qrotA);
    ChFpMatrix34<> Fp(qrotA);
    ChFmMatrix34<> Fm(qrotA);
    ChGlMatrix34<> Gl(qrotA);
    ChGwMatrix34<> Gw(qrotA);

    ChFrameMoving<> testa(vtraslA, qrotA);
    testa.SetPosDt(ChVector3d(0.5, 0.6, 0.7));
    testa.SetAngVelLocal(ChVector3d(1.1, 2.1, 5.1));
    testa.SetPosDt2(ChVector3d(7, 8, 9));
    testa.SetAngAccLocal(ChVector3d(4.3, 5.3, 2.3));
    std::cout << testa << "a moving frame " << std::endl;

    ChVector3d locpos(0.1, 3.1, 1.1);
    ChVector3d locspeed(3.2, 9.2, 7.2);
    ChVector3d locacc(5.3, 3.3, 2.3);

    ChFrameMoving<> testPl(locpos, QUNIT);
    testPl.SetPosDt(locspeed);
    testPl.SetRotDt(qrotA);
    testPl.SetAngVelLocal(ChVector3d(0.4, 0.5, 0.6));
    testPl.SetPosDt2(locacc);
    testPl.SetAngAccLocal(ChVector3d(0.43, 0.53, 0.63));
    ChFrameMoving<> testPw = testa.TransformLocalToParent(testPl);

    ChFrameMoving<> bres = (testPl >> testa);

    std::cout << bres << " transform loc->abs" << std::endl;

    ChQuaternion<> pollo(3, 5, 6, 7);
    ChVector3d pallo(2, 4, 6);

    ChTimer timer;

    int i;

    timer.start();
    for (i = 0; i < 1000000; i++) {
        testPw = testa.TransformLocalToParent(testPl);
    }
    timer.stop();
    std::cout << "TEST 1e6 calls to ChFrameMoving::TransformLocalToParent. Time = " << timer() << std::endl;

    timer.start();
    for (i = 0; i < 1000000; i++) {
        mvect2 = mvect1 >> mframeA;
    }
    timer.stop();
    std::cout << "TEST 1e6 calls of mvect2 = mvect1 >> mframeA.  Time = " << timer() << std::endl;

    timer.start();
    for (i = 0; i < 1000000; i++) {
        testa.PointAccelerationParentToLocal(vtraslA, vtraslA, vtraslA);
    }
    timer.stop();
    std::cout << "TEST 1e6 calls to PointAccelerationParentToLocal. Time = " << timer() << std::endl;

    return 0;
}
