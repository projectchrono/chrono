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

#ifndef CHC_CAPSULE_H
#define CHC_CAPSULE_H

//////////////////////////////////////////////////
//  
//   ChCCapsule.h
//
//   HEADER file for CHRONO,
//   Multibody dynamics engine
//
// ------------------------------------------------
//             www.projectchrono.org
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChCGeometry.h"


namespace chrono
{
namespace geometry
{


#define CH_GEOCLASS_CAPSULE   14


///
/// A capsule geometric object for collision, visualization, etc.
///

class ChApi ChCapsule : public ChGeometry
{
							// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChCapsule,ChGeometry);

public:

		//
		// CONSTRUCTORS
		//

	ChCapsule() 
				{
					center = ChVector<>(0,0,0);
					rad = 0;
					hlen = 0;
				};
	
	ChCapsule(ChVector<>& mcenter, double mrad, double mhlen)
				{
					center = mcenter;
					rad = mrad;
					hlen = mhlen;
				}

	ChCapsule(const ChCapsule & source)
				{
					Copy(&source);
				}

	void Copy(const ChCapsule* source)
				{
					center = source->center;
					rad = source->rad;
					hlen = source->hlen;
				}

	ChGeometry* Duplicate()
				{
					ChGeometry* mgeo = new ChCapsule();
					mgeo->Copy(this);
					return mgeo;
				}


		//
		// OVERRIDE BASE CLASS FUNCTIONS
		//

	virtual int GetClassType () {return CH_GEOCLASS_CAPSULE;}

	virtual void GetBoundingBox(double& xmin, double& xmax,
	                            double& ymin, double& ymax,
	                            double& zmin, double& zmax,
	                            ChMatrix33<>* Rot = NULL)
				{
					Vector trsfCenter = Rot ? Rot->MatrT_x_Vect(center) : center;

					xmin = trsfCenter.x - rad;
					xmax = trsfCenter.x + rad;
					ymin = trsfCenter.y - (rad + hlen);
					ymax = trsfCenter.y + (rad + hlen);
					zmin = trsfCenter.z - rad;
					zmax = trsfCenter.z + rad;
				}

	virtual ChVector<> Baricenter() {return center;}

			//***TO DO***  obsolete/unused
	virtual void CovarianceMatrix(ChMatrix33<>& C)
				{
					C.Reset();
					C(0,0)= center.x*center.x;
					C(1,1)= center.y*center.y;
					C(2,2)= center.z*center.z;
				}

				/// This is a solid
	virtual int GetManifoldDimension() {return 3;}


		//
		// DATA
		//


	ChVector<> center;
	double     rad;
	double     hlen;


		//
		// STREAMING
		//

	void StreamOUT(ChStreamOutBinary& mstream);

	void StreamIN(ChStreamInBinary& mstream); 

};



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif
