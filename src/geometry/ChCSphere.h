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

#ifndef CHC_SPHERE_H
#define CHC_SPHERE_H

//////////////////////////////////////////////////
//  
//   ChCSphere.h
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChCGeometry.h"


namespace chrono 
{
namespace geometry 
{



#define EPS_SHPEREDEGENERATE 1e-20


#define CH_GEOCLASS_SPHERE   2


///
/// A sphere.
/// Geometric object for collisions and such.
///

class ChApi ChSphere : public ChGeometry
{
							// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChSphere,ChGeometry);

public:

		//
		// CONSTRUCTORS
		//

	ChSphere() 
				{
					center= VNULL;
					rad = 0;
				};
	
	ChSphere(Vector& mc, double mrad) 
				{
					center = mc;
					rad = mrad;
				}

	ChSphere(const ChSphere & source)
				{
					Copy(&source);
				}

	void Copy (const ChSphere* source) 
				{
					center = source->center;
					rad = source->rad;
				};

	ChGeometry* Duplicate () 
				{
					ChGeometry* mgeo = new ChSphere(); 
					mgeo->Copy(this); return mgeo;
				};


		//
		// OVERRIDE BASE CLASS FUNCTIONS
		//

	virtual int GetClassType () {return CH_GEOCLASS_SPHERE;};

	virtual void GetBoundingBox(double& xmin, double& xmax, 
					    double& ymin, double& ymax, 
						double& zmin, double& zmax,
						ChMatrix33<>* Rot = NULL)
				{
					Vector trsfCenter = center;
					if (Rot)
					{
						trsfCenter = Rot->MatrT_x_Vect(center);
					}
					xmin=trsfCenter.x-rad;
					xmax=trsfCenter.x+rad;
					ymin=trsfCenter.y-rad;
					ymax=trsfCenter.y+rad;
					zmin=trsfCenter.z-rad;
					zmax=trsfCenter.z+rad;
				}
	
	virtual Vector Baricenter() {return center;};

	virtual void CovarianceMatrix(ChMatrix33<>& C) 
				{
					C.Reset();
					C(0,0)= center.x*center.x;
					C(1,1)= center.y*center.y;
					C(2,2)= center.z*center.z;
				};

				/// This is a solid
	virtual int GetManifoldDimension() {return 3;}


			


		//
		// DATA
		//

	Vector center;

	double rad;


		//
		// STREAMING
		//

	void StreamOUT(ChStreamOutBinary& mstream);

	void StreamIN(ChStreamInBinary& mstream); 

};



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif
