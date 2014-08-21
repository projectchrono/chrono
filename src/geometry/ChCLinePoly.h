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

#ifndef CHC_LINEPOLY_H
#define CHC_LINEPOLY_H

//////////////////////////////////////////////////
//  
//   ChCLinePoly.h
//
//   Base class for lines with control points
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <math.h>

#include "ChCLine.h"


namespace chrono
{
namespace geometry 
{



#define CH_GEOCLASS_LINEPOLY   5

///
/// POLY LINE
///
/// Geometric object representing a line in 3D space, which
/// is controlled by control points.
///
	
class ChApi ChLinePoly : public ChLine 
{
							// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChLinePoly,ChLine);

private:

		//
		// DATA
		//

	Vector* points; // control points
	int		numpoints;
	int		degree;

public:	

		//
		// CONSTRUCTORS
		//

	ChLinePoly (int mnumpoints=1);

	~ChLinePoly ();

	ChLinePoly(const ChLinePoly & source)
				{
					points = 0;
					Copy(&source);
				}

	void Copy (const ChLinePoly* source);

	ChGeometry* Duplicate () 
				{
					ChGeometry* mgeo = new ChLinePoly(*this); 
					return mgeo;
				};


		//
		// OVERRIDE BASE CLASS FUNCTIONS
		//

	virtual int GetClassType () {return CH_GEOCLASS_LINEPOLY;};


	virtual int Get_closed();
	virtual void Set_closed(int mc);

	virtual int Get_complexity() {return numpoints;};
	virtual void Set_complexity(int mc) {};

			/// Curve evaluation (only parU is used, in 0..1 range)
	virtual void Evaluate(Vector& pos, 
						const double parU, 
						const double parV = 0., 
						const double parW = 0.);

			/// Returns curve length. sampling does not matter
	double Lenght (int sampling);

			/// Draw into the current graph viewport of a ChFile_ps file
	int DrawPostscript(ChFile_ps* mfle, int markpoints, int bezier_interpolate);



		//
		// CUSTOM FUNCTIONS
		//

			/// Gets the number of control points 
	virtual int Get_numpoints();

			/// Get the degree of the curve (1= linear,
			/// 2= quadric, 3= cubic, etc.)
	virtual int	Get_degree();

			/// Get the n-th control point
	virtual Vector Get_point(int mnum);

			/// Set the n-th control point
	virtual int Set_point (int mnum, Vector mpoint);


		//
		// STREAMING
		//

	void StreamOUT(ChStreamOutBinary& mstream);

	void StreamIN(ChStreamInBinary& mstream); 



};





} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

#endif  // END of header


 
	