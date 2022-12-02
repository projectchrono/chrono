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

#include "chrono/core/ChLog.h"
#include "chrono/core/ChTransform.h"
#include "chrono/core/ChFrame.h"
#include "chrono/core/ChFrameMoving.h"
#include "chrono/core/ChTimer.h"

using namespace chrono;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // To write something to the console, use the chrono::GetLog()
    GetLog() << "CHRONO demo about coordinate transformations: \n\n";

    //
    // Some methods to achieve coordinate transformations, and some
    // examples of how to manipulate coordinates and frames, using Chrono features.
    //
    // You can use ChTransform or ChChCoordsys<> or ChFrame functions to transform points
    // from/to local coordinates in 3D, in ascending complexity and capabilities.
    //

    ChVector<> mvect2;  // resulting (transformed) vectors will go here
    ChVector<> mvect3;

    // Define a  POINT  to be transformed, expressed in
    // local frame coordinate.
    ChVector<> mvect1(2, 3, 4);

    // Define a vector representing the TRANSLATION of the frame
    // respect to absolute (world) coordinates.
    ChVector<> vtraslA(5, 6, 7);

    // Define a quaternion representing the ROTATION of the frame
    // respect to absolute (world) coordinates. Must be normalized.
    ChQuaternion<> qrotA(1, 3, 4, 5);
    qrotA.Normalize();

    // ..Also create a 3x3 rotation matrix [A] from the quaternion
    // (at any time you can use mrotA.Set_A_quaternion(qrotA) );
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
    GetLog() << mvect2 << " ..using linear algebra, \n";

    // TRANSFORM USING QUATERNION ROTATION

    mvect2 = vtraslA + qrotA.Rotate(mvect1);
    GetLog() << mvect2 << " ..using quaternion rotation, \n";

    // TRANSFORM USING THE ChTransform STATIC METHODS

    mvect2 = ChTransform<>::TransformLocalToParent(mvect1, vtraslA, mrotA);
    GetLog() << mvect2 << " ..using the ChTransform- vect and rot.matrix, \n";

    mvect2 = ChTransform<>::TransformLocalToParent(mvect1, vtraslA, qrotA);
    GetLog() << mvect2 << " ..using the ChTransform- vect and quat, \n";

    // TRANSFORM USING A ChCoordys OBJECT

    mvect2 = csysA.TransformLocalToParent(mvect1);
    GetLog() << mvect2 << " ..using a ChChCoordsys<> object, \n";

    mvect2 = mvect1 >> csysA;
    GetLog() << mvect2 << " ..using a ChChCoordsys<> '>>' operator, \n";

    mvect2 = csysA * mvect1;
    GetLog() << mvect2 << " ..using a ChChCoordsys<> '*' operator, \n";

    // TRANSFORM USING A ChFrame OBJECT

    ChFrame<> mframeA(vtraslA, qrotA);  // or ChFrame<> mframeA(csysA);

    mvect2 = mframeA.TransformLocalToParent(mvect1);
    GetLog() << mvect2 << " ..using a ChFrame object function, \n";

    mvect2 = mvect1 >> mframeA;
    GetLog() << mvect2 << " ..using a ChFrame '>>' operator, \n";

    mvect2 = mframeA * mvect1;
    GetLog() << mvect2 << " ..using a ChFrame '*' operator, \n";

    //
    // Now perform transformations in a chain of frames, in
    // sequence.
    //

    ChVector<> v10(5, 6, 7);
    ChQuaternion<> q10(1, 3, 4, 5);
    q10.Normalize();
    ChMatrix33<> m10(q10);

    ChVector<> v21(4, 1, 3);
    ChQuaternion<> q21(3, 2, 1, 5);
    q21.Normalize();
    ChMatrix33<> m21(q21);

    ChVector<> v32(1, 5, 1);
    ChQuaternion<> q32(4, 1, 3, 1);
    q32.Normalize();
    ChMatrix33<> m32(q32);

    // ...with linear algebra:

    mvect3 = v10 + m10 * (v21 + m21 * (v32 + m32 * mvect1));
    GetLog() << mvect3 << " ..triple trsf. using linear algebra, \n";

    // ...with ChFrame '>>' operator or "*" operator
    // is by far much simplier!

    ChFrame<> f10(v10, q10);
    ChFrame<> f21(v21, q21);
    ChFrame<> f32(v32, q32);

    mvect3 = mvect1 >> f32 >> f21 >> f10;
    GetLog() << mvect3 << " ..triple vector trsf. with ChFrame '>>' operator, \n";

    mvect3 = f10 * f21 * f32 * mvect1;
    GetLog() << mvect3 << " ..triple vector trsf. with ChFrame '*' operator, \n";

    ChFrame<> tempf(f10 * f21 * f32);
    mvect3 = tempf * mvect1;
    GetLog() << mvect3 << " ..triple vector trsf. with ChFrame '*' operator, \n";

    // Not only vectors, but also ChFrame can be transformed
    // with ">>" or "*" operators.

    ChFrame<> f_3(mvect1);
    ChFrame<> f_0;
    f_0 = f_3 >> f32 >> f21 >> f10;
    GetLog() << f_0 << " ..triple frame trsf. with ChFrame '>>' operator,  \n";

    f_0 = f10 * f21 * f32 * f_3;
    GetLog() << f_0 << " ..triple frame trsf. with ChFrame '*' operator,  \n";

    // Test the  ">>" or "*" operators also for ChCoordsys:
    ChCoordsys<> c_0;
    c_0 = f_3.GetCoord() >> f32.GetCoord() >> f21.GetCoord() >> f10.GetCoord();
    GetLog() << f_0 << " ..triple frame trsf. with ChCoordsys '>>' operator,  \n";

    c_0 = f10.GetCoord() * f21.GetCoord() * f32.GetCoord() * f_3.GetCoord();
    GetLog() << f_0 << " ..triple frame trsf. with ChCoordsys '*' operator,  \n";

    //
    // Now test inverse transformations too.
    //
    // From the low-level to the higher level methods, here are some
    // ways to accomplish this.
    //

    // TRANSFORM USING ROTATION MATRIX AND LINEAR ALGEBRA
    //

    GetLog() << mvect1 << " ..mvect1 \n";
    mvect1 = mrotA.transpose() * (mvect2 - vtraslA);  // like:  v1 = [A]'*(v2-t)
    GetLog() << mvect1 << " ..inv, using linear algebra, \n";

    // TRANSFORM USING QUATERNION ROTATION

    mvect1 = qrotA.RotateBack(mvect2 - vtraslA);
    GetLog() << mvect1 << " ..inv, using quaternion rotation, \n";

    // TRANSFORM USING THE ChTransform STATIC METHODS

    mvect1 = ChTransform<>::TransformParentToLocal(mvect2, vtraslA, mrotA);
    GetLog() << mvect1 << " ..inv, using the ChTransform- vect and rot.matrix, \n";

    mvect1 = ChTransform<>::TransformParentToLocal(mvect2, vtraslA, qrotA);
    GetLog() << mvect1 << " ..inv, using the ChTransform- vect and quat, \n";

    // TRANSFORM USING A ChCoordys OBJECT

    mvect1 = csysA.TransformParentToLocal(mvect2);
    GetLog() << mvect1 << " ..inv, using a ChChCoordsys<> object, \n";

    // TRANSFORM USING A ChFrame OBJECT

    mvect1 = mframeA.TransformParentToLocal(mvect2);
    GetLog() << mvect1 << " ..inv, using a ChFrame object function, \n";

    mvect1 = mvect2 >> mframeA.GetInverse();
    GetLog() << mvect1 << " ..inv, using a ChFrame inverse and '>>' operator, \n";

    mvect1 = mframeA.GetInverse() * mvect2;
    GetLog() << mvect1 << " ..inv, using a ChFrame inverse and  '*' operator, \n";

    mvect1 = mframeA / mvect2;
    GetLog() << mvect1 << " ..inv, using a ChFrame '/' operator, \n";

    ChFrame<> mframeAinv(mframeA);
    mframeAinv.Invert();
    mvect1 = mframeAinv * mvect2;
    GetLog() << mvect1 << " ..inv, using an inverted ChFrame \n";

    // ... also for inverting chain of transformations...

    // mvect3 =  f10 * f21 * f32 * mvect1;				// direct transf..

    mvect1 = (f10 * f21 * f32).GetInverse() * mvect3;  // inverse transf.
    GetLog() << mvect1 << " ..inv three transf \n";

    mvect1 = f32.GetInverse() * f21.GetInverse() * f10.GetInverse() * mvect3;
    GetLog() << mvect1 << " ..inv three transf (another method) \n";

    mvect1 = mvect3 >> (f32 >> f21 >> f10).GetInverse();
    GetLog() << mvect1 << " ..inv three transf (another method) \n";

    mvect1 = mvect3 >> f10.GetInverse() >> f21.GetInverse() >> f32.GetInverse();
    GetLog() << mvect1 << " ..inv three transf (another method) \n";

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
    mframeA1 >>= ChVector<>(1, 2, 3);

    // Transform mframeA1 by rotating it 30 degrees on axis Y, using a quaternion:
    mframeA1 >>= Q_from_AngAxis(30 * CH_C_DEG_TO_RAD, VECT_Y);

    //
    // BENCHMARK FOR EXECUTION SPEED
    //

    GetLog() << " %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% \n\n";

    mrotA.Set_A_quaternion(qrotA);
    ChFpMatrix34<> Fp(qrotA);
    ChFmMatrix34<> Fm(qrotA);
    ChGlMatrix34<> Gl(qrotA);
    ChGwMatrix34<> Gw(qrotA);

    ChFrameMoving<> testa(vtraslA, qrotA);
    testa.SetPos_dt(ChVector<>(0.5, 0.6, 0.7));
    testa.SetWvel_loc(ChVector<>(1.1, 2.1, 5.1));
    testa.SetPos_dtdt(ChVector<>(7, 8, 9));
    testa.SetWacc_loc(ChVector<>(4.3, 5.3, 2.3));
    GetLog() << testa << "a moving frame";

    ChVector<> locpos(0.1, 3.1, 1.1);
    ChVector<> locspeed(3.2, 9.2, 7.2);
    ChVector<> locacc(5.3, 3.3, 2.3);

    ChFrameMoving<> testPl(locpos, QUNIT);
    testPl.SetPos_dt(locspeed);
    testPl.SetRot_dt(qrotA);
    testPl.SetWvel_loc(ChVector<>(0.4, 0.5, 0.6));
    testPl.SetPos_dtdt(locacc);
    testPl.SetWacc_loc(ChVector<>(0.43, 0.53, 0.63));
    ChFrameMoving<> testPw;
    ChFrameMoving<> testX;
    testa.TransformLocalToParent(testPl, testPw);

    ChFrameMoving<> bres = (testPl >> testa);

    GetLog() << bres << " trasf loc->abs \n";

    ChQuaternion<> pollo(3, 5, 6, 7);
    ChVector<> pallo(2, 4, 6);

    ChTimer<double> timer;

    //int numcycles = 100000;
    int i;

    timer.start();
    for (i = 0; i < 1000000; i++) {
        testa.TransformLocalToParent(testPl, testPw);
    }
    timer.stop();
    GetLog() << "TEST 10e6 of ChFrameMoving::TransformLocalToParent (1.38) Time: " << timer() << " \n";
    // VC6   : 1.380
    // VC2003: 0.861
    // VC2005: 0.691
    // GCC   : 0.661

    timer.start();
    for (i = 0; i < 1000000; i++) {
        mvect2 = mvect1 >> mframeA;
    }
    timer.stop();
    GetLog() << "TEST 10e6 of mvect2 = mvect1 >> mframeA; (0.03)" << timer() << " \n";
    // VC6   : 0.03
    // VC2003: 0.03
    // VC2005: 0.03
    // GCC   : 0.03

    timer.start();
    for (i = 0; i < 1000000; i++) {
        testa.PointAccelerationParentToLocal(vtraslA, vtraslA, vtraslA);
    }
    timer.stop();
    GetLog() << "TEST 10e6 of PointAccelerationParentToLocal (0.811)" << timer() << " \n";
    // VC6   : 0.811
    // VC2003: 0.531
    // VC2005: 0.410
    // GCC   : 0.320

    /*
       timer.start();
       for (i= 0; i<numcycles; i++)
       {
           for (int j = 0; j<100; j++)
           {
               mvect2 = mvect1 >> f32 >> f21 >> f10;
               // NOTE: thank to the fact that operators are executed from left to
               // right, the above is MUCH faster (16x) than the equivalent:
               //    mvect2 =  f10 * f21 * f32 * mvect1;
               // because the latter, if no parenthesis are used, would imply
               // three expensive frame*frame operations, and a last frame*vector.
           }
       }
       timer.stop();
       GetLog() << "Test 3 frame transf. with >> ChFrame operator: " <<  timer() << " \n";


       timer.start();
       for (i= 0; i<1000000; i++)
       {
           testa.SetCoord(vtraslA,qrotA);
       }
       timer.stop();
       GetLog() << "Test ChFrame::SetPos() " <<  timer() << " \n";


   //ChQuaternion<> mqdt(1, 2, 3, 4);
       timer.start();
       for (i= 0; i<1000000; i++)
       {
           testa.SetRot_dt(mqdt);
       }
       timer.stop();
       GetLog() << "Test ChFrame::SetRot_dt() " <<  timer() << " \n";

       timer.start();
       for (i= 0; i<1000000; i++)
       {
           testa.SetRot_dtdt(mqdt);
       }
       timer.stop()
       GetLog() << "Test ChFrame::SetRot_dtdt() " <<  timer() << " \n";


   ChVector<> mv(1, 2, 3);
       timer.start();
       for (i= 0; i<1000000; i++)
       {
           testa.SetWvel_loc(mv);
       }
       timer.stop();
       GetLog() << "Test ChFrame::SetWvel_loc() " <<  timer() << " \n";

       timer.start();
       for (i= 0; i<1000000; i++)
       {
           testa.SetWacc_loc(mv);
       }
       timer.stop();
       GetLog() << "Test ChFrame::SetWacc_loc() " <<  timer() << " \n";

       timer.start();
       for (i= 0; i<1000000; i++)
       {
           Vector p= testa.GetWvel_loc();
       }
       timer.stop();
       GetLog() << "Test ChFrame::GetWvel_loc() " <<  timer() << " \n";

       timer.start();
       for (i= 0; i<1000000; i++)
       {
           ChVector<> p= testa.GetWacc_loc();
       }
       timer.stop();
       GetLog() << "Test ChFrame::GetWacc_loc() " <<  timer() << " \n";


   */

    GetLog() << "\n  CHRONO execution terminated.";

    return 0;
}
