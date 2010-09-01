///////////////////////////////////////////////////
//
//   Demo on how to use Chrono coordinate
//   transformations 
//
//	 CHRONO 
//   ------
//   Multibody dinamics engine
// 
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
  
 

#include <math.h>
 
       
#include "core/ChLog.h"
#include "core/ChTrasform.h"
#include "core/ChFrame.h"
#include "core/ChFrameMoving.h"
#include "physics/ChMarker.h"
#include "physics/ChBody.h"
#include "physics/ChApidll.h" 

using namespace chrono;
           
		        
int main(int argc, char* argv[])
{	

	// To write something to the console, use the chrono::GetLog() 

	GetLog() << "CHRONO demo about coordinate transformations: \n\n";   
     
	// The DLL_CreateGlobals() - DLL_DeleteGlobals(); pair is needed if
	// global functions are needed.
	DLL_CreateGlobals();

	//
	// Some methods to achieve coordinate transformations, and some
	// examples of how to manipulate coordinates and frames, using Chrono features.
	//
	// You can use ChTrasform or ChCoordsys or ChFrame functions to transform points 
	// from/to local coordinates in 3D, in ascending complexity and capabilities.
	//
 
 
	chrono::Vector mvect2;					// resulting (transformed) vectors will go here
	chrono::Vector mvect3;
						
						// Define a  POINT  to be transformed, ex�ressed in
						// local frame coordinate.
	chrono::Vector mvect1(2,3,4);			

						// Define a vector representin the TRASLATION of the frame
						// respect to absolute (world) coordinates.
	chrono::Vector	   vtraslA(5,6,7);		

						// Define a quaternion representing the ROTATION of the frame
						// respect to absolute (world) coordinates. Must be normalized.
	chrono::Quaternion qrotA(1, 3, 4, 5);	
	qrotA.Normalize();

						// ..Also create a 3x3 rotation matrix [A] from the quaternion
						// (at any time you can use mrotA.Set_A_quaternion(qrotA) );
	chrono::ChMatrix33<> mrotA(qrotA);
	

						// ..Also create a coordsystem object, representing both 
						// translation and rotation.
	chrono::Coordsys csysA (vtraslA, qrotA);

						// OK!!! Now we are ready to perform the transformation, like in 
						// linear algebra formula v'=t+[A]*v, so that we will obtain
						// the coordinates of mvect1 in absolute coordinates.
						// This can be achieved in many ways. Let's see them.


					// TRASFORM USING ROTATION MATRIX AND LINEAR ALGEBRA
					//   
	mvect2 = vtraslA + mrotA*mvect1;					// like:  v2 = t + [A]*v1  
	GetLog() << mvect2 << " ..using linear algebra, \n";

	
					// TRASFORM USING QUATERNION ROTATION

	mvect2 = vtraslA + qrotA.Rotate(mvect1);
	GetLog() << mvect2 << " ..using quaternion rotation, \n";


					// TRASFORM USING THE ChTrasform STATIC METHODS

	mvect2 = chrono::ChTrasform<>::TrasformLocalToParent(mvect1, vtraslA, mrotA);
	GetLog() << mvect2 << " ..using the ChTrasform- vect and rot.matrix, \n";

	mvect2 = chrono::ChTrasform<>::TrasformLocalToParent(mvect1, vtraslA, qrotA);
	GetLog() << mvect2 << " ..using the ChTrasform- vect and quat, \n";


					// TRASFORM USING A ChCoordys OBJECT

	mvect2 = csysA.TrasformLocalToParent(mvect1); 
	GetLog() << mvect2 << " ..using a ChCoordsys object, \n";

	

					// TRASFORM USING A ChFrame OBJECT

	ChFrame<> mframeA(vtraslA, qrotA);  // or ChFrame<> mframeA(vtraslA, mrotA);

	mvect2 = mframeA.TrasformLocalToParent(mvect1); 
	GetLog() << mvect2 << " ..using a ChFrame object function, \n";

	mvect2 = mvect1 >> mframeA; 
	GetLog() << mvect2 << " ..using a ChFrame '>>' operator, \n";

	mvect2 = mframeA * mvect1; 
	GetLog() << mvect2 << " ..using a ChFrame '*' operator, \n";


	//
	// Now perform transformations in a chain of frames, in 
	// sequence. 
	//


	chrono::Vector	   v10 (5,6,7);		
	chrono::Quaternion q10 (1, 3, 4, 5);	q10.Normalize();
	chrono::ChMatrix33<> m10     (q10);

	chrono::Vector	   v21 (4,1,3);		
	chrono::Quaternion q21 (3, 2, 1, 5);	q21.Normalize();
	chrono::ChMatrix33<> m21     (q21);
	
	chrono::Vector	   v32 (1,5,1);		
	chrono::Quaternion q32 (4, 1, 3, 1);	q32.Normalize();
	chrono::ChMatrix33<> m32     (q32);

					// ...with linear algebra: 

	mvect3 =  v10 + m10 * (v21 + m21 * (v32 + m32 * mvect1)); 
	GetLog() << mvect3 << " ..triple trsf. using linear algebra, \n";
	
					// ...with ChFrame '>>' operator or "*" operator 
					// is by far much simplier!

	chrono::ChFrame<> f10 (v10, q10); 
	chrono::ChFrame<> f21 (v21, q21);
	chrono::ChFrame<> f32 (v32, q32);


	mvect3 = mvect1 >> f32 >> f21 >> f10;
	GetLog() << mvect3 << " ..triple vector trsf. with ChFrame '>>' operator, \n";

	mvect3 =  f10 * f21 * f32 * mvect1;
	GetLog() << mvect3 << " ..triple vector trsf. with ChFrame '*' operator, \n";

	ChFrame<> tempf(f10 * f21 * f32);
	mvect3 = tempf * mvect1;
	GetLog() << mvect3 << " ..triple vector trsf. with ChFrame '*' operator, \n";

					// Not only vectors, but also frames can be transformed
					// with ">>" or "*" operators.

	chrono::ChFrame<> f_3 (mvect1);
	chrono::ChFrame<> f_0;
	f_0 = f_3 >> f32 >> f21 >> f10;
	GetLog() << f_0 << " ..triple frame trsf. with ChFrame '>>' operator,  \n";
	
	f_0 = f10 *  f21 *  f32 *  f_3;
	GetLog() << f_0 << " ..triple frame trsf. with ChFrame '*' operator,  \n";




	//
	// Now test inverse transformations too. 
	//
	// From the low-level to the higher level methods, here are some 
	// ways to accomplish this.
	//

					// TRASFORM USING ROTATION MATRIX AND LINEAR ALGEBRA
					//    

	GetLog() << mvect1 << " ..mvect1 \n";
	mvect1 = mrotA.MatrT_x_Vect(mvect2 - vtraslA);		 // like:  v1 = [A]'*(v2-t)
	GetLog() << mvect1 << " ..inv, using linear algebra, \n";

	
					// TRASFORM USING QUATERNION ROTATION

	mvect1 = qrotA.RotateBack(mvect2 - vtraslA);
	GetLog() << mvect1 << " ..inv, using quaternion rotation, \n";


					// TRASFORM USING THE ChTrasform STATIC METHODS

	mvect1 = chrono::ChTrasform<>::TrasformParentToLocal(mvect2, vtraslA, mrotA);
	GetLog() << mvect1 << " ..inv, using the ChTrasform- vect and rot.matrix, \n";

	mvect1 = chrono::ChTrasform<>::TrasformParentToLocal(mvect2, vtraslA, qrotA);
	GetLog() << mvect1 << " ..inv, using the ChTrasform- vect and quat, \n";


					// TRASFORM USING A ChCoordys OBJECT

	mvect1 = csysA.TrasformParentToLocal(mvect2); 
	GetLog() << mvect1 << " ..inv, using a ChCoordsys object, \n";

	

					// TRASFORM USING A ChFrame OBJECT

	mvect1 = mframeA.TrasformParentToLocal(mvect2); 
	GetLog() << mvect1 << " ..inv, using a ChFrame object function, \n";

	mvect1 = mvect2 >> mframeA.GetInverse(); 
	GetLog() << mvect1 << " ..inv, using a ChFrame '>>' operator, \n";

	mvect1 = mframeA.GetInverse() * mvect2; 
	GetLog() << mvect1 << " ..inv, using a ChFrame '*' operator, \n";

	mvect1 = mframeA / mvect2; 
	GetLog() << mvect1 << " ..inv, using a ChFrame '/' operator, \n";

	ChFrame<> mframeAinv (mframeA);
	mframeAinv.Invert();
	mvect1 = mframeAinv * mvect2; 
	GetLog() << mvect1 << " ..inv, using an inverted ChFrame \n";

 
 

							// ... also for inverting chain of transformations...
	
	//mvect3 =  f10 * f21 * f32 * mvect1;				// direct transf..

	mvect1 =  (f10 * f21 * f32).GetInverse() * mvect3;	// inverse transf.
	GetLog() << mvect1 << " ..inv three transf \n";

	mvect1 =  f32.GetInverse() * f21.GetInverse() * f10.GetInverse() * mvect3; 
	GetLog() << mvect1 << " ..inv three transf (another method) \n";

	mvect1 =  mvect3 >> (f32 >> f21 >> f10).GetInverse();	
	GetLog() << mvect1 << " ..inv three transf (another method) \n";

	mvect1 =  mvect3 >> f10.GetInverse() >> f21.GetInverse() >> f32.GetInverse();
	GetLog() << mvect1 << " ..inv three transf (another method) \n";



	//
	// BENChMARK FOR EXECUTION SPEED
	//
 
	GetLog() << " %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% \n\n";

	mrotA.Set_A_quaternion(qrotA);
	ChMatrixNM<double,3,4> Fp;
	ChFrame<>::SetMatrix_Fp(Fp, qrotA);
	ChMatrixNM<double,3,4> Fm;
	ChFrame<>::SetMatrix_Fm(Fm, qrotA);
	ChMatrixNM<double,3,4> Gl;
	ChFrame<>::SetMatrix_Gl(Gl, qrotA);
	ChMatrixNM<double,3,4> Gw;
	ChFrame<>::SetMatrix_Gw(Gw, qrotA);

	ChFrameMoving<> testa(vtraslA, qrotA);
	testa.SetPos_dt(Vector(0.5,0.6,0.7));
	testa.SetWvel_loc(Vector(1.1,2.1,5.1));
	testa.SetPos_dtdt(Vector(7,8,9));
	testa.SetWacc_loc(Vector(4.3,5.3,2.3));
	GetLog() << testa << "a moving frame";

	Vector locpos(0.1, 3.1, 1.1);
	Vector locspeed(3.2, 9.2, 7.2);
	Vector locacc(5.3, 3.3, 2.3);
	Vector parentpos = locpos >> testa;
 
	ChFrameMoving<> testPl(locpos, QUNIT);
	testPl.SetPos_dt(locspeed);
	testPl.SetRot_dt(qrotA);
	testPl.SetWvel_loc(Vector(0.4, 0.5, 0.6));
	testPl.SetPos_dtdt(locacc);
	testPl.SetWacc_loc(Vector(0.43, 0.53, 0.63));
	ChFrameMoving<> testPw;
	ChFrameMoving<> testX;
	testa.TrasformLocalToParent(testPl,testPw);

	ChFrameMoving<> bres = (testPl >> testa );

	GetLog() << bres << " trasf loc->abs \n"; 

	ChMatrixNM<double,3,4> mGl;
	ChFrame<>::SetMatrix_Gl(mGl, qrotA);
	ChQuaternion<> pollo(3,5,6,7);
	ChVector<>pallo(2,4,6);




	int numcycles =  100000;
	int i;

	ChGLOBALS().Timer_START();
	for (i= 0; i<1000000; i++)
	{ 
		testa.TrasformLocalToParent(testPl,testPw);
	}
	ChGLOBALS().Timer_STOP();
	GetLog() << "TEST 10e6 of ChFrameMoving::TrasformLocalToParent (1.38) Time: " <<  ChGLOBALS().t_duration << " \n";
	// VC6   : 1.380
	// VC2003: 0.861
	// VC2005: 0.691
	// GCC   : 0.661

	ChGLOBALS().Timer_START();
	for (i= 0; i<1000000; i++)
	{ 
		mvect2 = mvect1 >> mframeA; 
	} 
	ChGLOBALS().Timer_STOP();
	GetLog() << "TEST 10e6 of mvect2 = mvect1 >> mframeA; (0.03)" <<  ChGLOBALS().t_duration << " \n";
	// VC6   : 0.03
	// VC2003: 0.03
	// VC2005: 0.03
	// GCC   : 0.03


	ChGLOBALS().Timer_START();
	for (i= 0; i<1000000; i++)
	{ 
		testa.PointAccelerationParentToLocal(vtraslA,vtraslA,vtraslA);
	}
	ChGLOBALS().Timer_STOP();
	GetLog() << "TEST 10e6 of PointAccelerationParentToLocal (0.811)" <<  ChGLOBALS().t_duration << " \n";
	// VC6   : 0.811
	// VC2003: 0.531
	// VC2005: 0.410
	// GCC   : 0.320

 /*
	ChGLOBALS().Timer_START();
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
	ChGLOBALS().Timer_STOP();
	GetLog() << "Test 3 frame transf. with >> ChFrame operator: " <<  ChGLOBALS().t_duration << " \n";


	ChGLOBALS().Timer_START();
	for (i= 0; i<1000000; i++)
	{  
		testa.SetCoord(vtraslA,qrotA);
	}
	ChGLOBALS().Timer_STOP();
	GetLog() << "Test ChFrame::SetPos() " <<  ChGLOBALS().t_duration << " \n";


//Quaternion mqdt(1, 2, 3, 4);
	ChGLOBALS().Timer_START();
	for (i= 0; i<1000000; i++)
	{ 
		testa.SetRot_dt(mqdt);
	}
	ChGLOBALS().Timer_STOP();
	GetLog() << "Test ChFrame::SetRot_dt() " <<  ChGLOBALS().t_duration << " \n";

	ChGLOBALS().Timer_START();
	for (i= 0; i<1000000; i++)
	{ 
		testa.SetRot_dtdt(mqdt);
	}
	ChGLOBALS().Timer_STOP();
	GetLog() << "Test ChFrame::SetRot_dtdt() " <<  ChGLOBALS().t_duration << " \n";


Vector mv(1, 2, 3);
	ChGLOBALS().Timer_START();
	for (i= 0; i<1000000; i++)
	{ 
		testa.SetWvel_loc(mv);
	}
	ChGLOBALS().Timer_STOP();
	GetLog() << "Test ChFrame::SetWvel_loc() " <<  ChGLOBALS().t_duration << " \n";

	ChGLOBALS().Timer_START();
	for (i= 0; i<1000000; i++)
	{ 
		testa.SetWacc_loc(mv);
	}
	ChGLOBALS().Timer_STOP();
	GetLog() << "Test ChFrame::SetWacc_loc() " <<  ChGLOBALS().t_duration << " \n";

	ChGLOBALS().Timer_START();
	for (i= 0; i<1000000; i++)
	{ 
		Vector p= testa.GetWvel_loc();
	} 
	ChGLOBALS().Timer_STOP();
	GetLog() << "Test ChFrame::GetWvel_loc() " <<  ChGLOBALS().t_duration << " \n";
 
	ChGLOBALS().Timer_START();
	for (i= 0; i<1000000; i++)
	{ 
		Vector p= testa.GetWacc_loc();
	}
	ChGLOBALS().Timer_STOP();
	GetLog() << "Test ChFrame::GetWacc_loc() " <<  ChGLOBALS().t_duration << " \n";
 

*/

	GetLog() << "\n  CHRONO execution terminated.";
	
	DLL_DeleteGlobals();

	return 0;
}
 

