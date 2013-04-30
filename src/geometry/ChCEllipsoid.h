#ifndef CHC_ELLIPSOID_H
#define CHC_ELLIPSOID_H

//////////////////////////////////////////////////
//  
//   ChCEllipsoid.h
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
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

class ChApi ChEllipsoid : public ChGeometry
{
							// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChEllipsoid,ChGeometry);

public:

		//
		// CONSTRUCTORS
		//

	ChEllipsoid()
				{
					center= VNULL;
					rad = 0;
				};
	
	ChEllipsoid(Vector& mc, Vector mrad)
				{
					center = mc;
					rad = mrad;
				}

	ChEllipsoid(const ChEllipsoid & source)
				{
					Copy(&source);
				}

	void Copy (const ChEllipsoid* source)
				{
					center = source->center;
					rad = source->rad;
				};

	ChGeometry* Duplicate () 
				{
					ChGeometry* mgeo = new ChEllipsoid();
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
					xmin=trsfCenter.x-rad.x;
					xmax=trsfCenter.x+rad.x;
					ymin=trsfCenter.y-rad.y;
					ymax=trsfCenter.y+rad.y;
					zmin=trsfCenter.z-rad.z;
					zmax=trsfCenter.z+rad.z;
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

	Vector rad;


		//
		// STREAMING
		//

	void StreamOUT(ChStreamOutBinary& mstream);

	void StreamIN(ChStreamInBinary& mstream); 

};



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif
