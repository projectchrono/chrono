//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHC_CYLINDER_H
#define CHC_CYLINDER_H

//////////////////////////////////////////////////
//  
//   ChCCylinder.h
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



#define EPS_CYLINDERDEGENERATE 1e-20


#define CH_GEOCLASS_CYLINDER   13


///
/// A cylinder.
/// Geometric object for collisions and such.
///

class ChApi ChCylinder : public ChGeometry
{
							// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChCylinder,ChGeometry);

public:

		//
		// CONSTRUCTORS
		//

	ChCylinder() 
				{
					rad = 0.1;
				};
	
	ChCylinder(ChVector<>& mp1, ChVector<>& mp2, double mrad) 
				{

					rad = mrad;
				}

	ChCylinder(const ChCylinder & source)
				{
					Copy(&source);
				}

	void Copy (const ChCylinder* source) 
				{

					rad = source->rad;
				};

	ChGeometry* Duplicate () 
				{
					ChGeometry* mgeo = new ChCylinder(); 
					mgeo->Copy(this); return mgeo;
				};


		//
		// OVERRIDE BASE CLASS FUNCTIONS
		//

	virtual int GetClassType () {return CH_GEOCLASS_CYLINDER;};

	virtual void GetBoundingBox(double& xmin, double& xmax, 
					    double& ymin, double& ymax, 
						double& zmin, double& zmax,
						ChMatrix33<>* Rot = NULL)
				{
		Vector dims = rad;
		Vector trsfCenter = Baricenter();
							if (Rot)
							{
								trsfCenter = Rot->MatrT_x_Vect(Baricenter());
							}
							xmin=trsfCenter.x-dims.x;
							xmax=trsfCenter.x+dims.x;
							ymin=trsfCenter.y-dims.y;
							ymax=trsfCenter.y+dims.y;
							zmin=trsfCenter.z-dims.z;
							zmax=trsfCenter.z+dims.z;


				}
	
	virtual ChVector<> Baricenter() {return ChVector<>(0);};

			//***TO DO***  obsolete/unused
	virtual void CovarianceMatrix(ChMatrix33<>& C) 
				{
					C.Reset();
					C(0,0)= 1;
					C(1,1)= 1;
					C(2,2)= 1;
				};

				/// This is a solid
	virtual int GetManifoldDimension() {return 3;}


			


		//
		// DATA
		//

	ChVector<> rad;

		//
		// STREAMING
		//

	void StreamOUT(ChStreamOutBinary& mstream);

	void StreamIN(ChStreamInBinary& mstream); 

};



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif
