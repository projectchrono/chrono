//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHC_LINE_H
#define CHC_LINE_H

//////////////////////////////////////////////////
//  
//   ChCLine.h
//
//   Base class for lines
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <math.h>

#include "ChCGeometry.h"
#include "physics/ChFilePS.h"



namespace chrono
{
namespace geometry 
{





#define CH_GEOCLASS_LINE   4	


///
/// LINE.
///
/// Base class for all geometric objects representing lines 
/// in 3D space.
///

class ChApi ChLine : public ChGeometry 
{
							// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChLine,ChGeometry);

protected:
	int		closed;
	int		complexityU;
public:	

		//
		// CONSTRUCTORS
		//

	ChLine () { closed = FALSE; complexityU = 2;};

	~ChLine () {};

	ChLine(const ChLine & source)
				{
					Copy(&source);
				}

	void Copy (const ChLine* source) 
				{
					closed = source->closed;
					complexityU = source->complexityU;
				};

	ChGeometry* Duplicate () 
				{
					ChGeometry* mgeo = new ChLine(); 
					mgeo->Copy(this); return mgeo;
				};


		//
		// OVERRIDE BASE CLASS FUNCTIONS
		//
				/// Get the class type as unique numerical ID (faster 
				/// than using ChronoRTTI mechanism).
				/// Each inherited class must return an unique ID.
	virtual int GetClassType () {return CH_GEOCLASS_LINE;};


			/// Tell if the curve is closed
	virtual int Get_closed() {return closed;};
	virtual void Set_closed(int mc) {closed = mc;};

			/// Tell the complexity
	virtual int Get_complexity() {return complexityU;};
	virtual void Set_complexity(int mc) {complexityU = mc;};

			/// This is a line
	virtual int GetManifoldDimension() {return 1;}



		//
		// CUSTOM FUNCTIONS
		//

			/// Find the parameter resU for the nearest point on curve to "point".
	int FindNearestLinePoint (Vector& point, double& resU, double approxU, double tol);

			/// Returns curve length. Typical sampling 1..5 (1 already gives correct result with degree1 curves)
	virtual double Lenght (int sampling);

			/// Returns an adimensional information on "how much" this curve is similar to another
			/// in its overall shape (doesnot matter parametrization or start point). Try with 20 samples.
			/// The return value is somewhat the "average distance between the two curves".
			/// Note that the result is affected by "weight" of curves. If it chnges from default 1.0, the 
			/// distance extimation is higher/lower (ex: if a curve defines low 'weight' in its central segment,
			/// its CurveCurveDistance from another segment is not much affected by errors near the central segment).
	double CurveCurveDist (ChLine* compline, int samples);

			/// Same as before, but returns "how near" is complinesegm to 
			/// whatever segment of this line (does not matter the percentual of line).
	        /// Again, this is affected by "weight" of curves. If weight changes along curves ->'weighted' distance  
	double CurveSegmentDist (ChLine* complinesegm, int samples);

			/// Same as above, but instead of making average of the distances,
			/// these functions return the maximum of the distances...
	double CurveCurveDistMax (ChLine* compline, int samples);
	double CurveSegmentDistMax (ChLine* complinesegm, int samples);

			/// Draw into the current graph viewport of a ChFile_ps file
	virtual int DrawPostscript(ChFile_ps* mfle, int markpoints, int bezier_interpolate);

		//
		// STREAMING
		//

	void StreamOUT(ChStreamOutBinary& mstream);

	void StreamIN(ChStreamInBinary& mstream); 


};





} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

#endif  // END of header

 
