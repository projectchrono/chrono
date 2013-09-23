//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHC_TRI_H
#define CHC_TRI_H

//////////////////////////////////////////////////
//  
//   ChCTriangle.h
//
//   Basic triangle geometry in 3d.
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



namespace chrono
{
namespace geometry 
{




#define EPS_TRIDEGENERATE 1e-20


#define CH_GEOCLASS_TRIANGLE   1

///
/// A triangle.
/// Geometric object for collisions and such.
///

class ChApi ChTriangle : public ChGeometry
{
							// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChTriangle,ChGeometry);

public:
	ChTriangle();
	
	ChTriangle(const ChVector<>& mp1, const ChVector<>& mp2, const ChVector<>& mp3); 
	
	ChTriangle(const ChTriangle& source);

	virtual ~ChTriangle();

					/// Assignment operator: copy from another triangle
	ChTriangle& operator=(const ChTriangle& source)	
				{
					if (&source == this) return *this;  
					this->Copy(&source);
					return *this; 
				}

	

	void Copy (const ChTriangle* source) 
				{
					p1= source->p1;
					p2= source->p2;
					p3= source->p3;
				};

	ChGeometry* Duplicate () 
				{
					ChGeometry* mgeo = new ChTriangle(); 
					mgeo->Copy(this); return mgeo;
				};


		//
		// OVERRIDE BASE CLASS FUNCTIONS
		//

	virtual int GetClassType () {return CH_GEOCLASS_TRIANGLE;};

	virtual void GetBoundingBox(double& xmin, double& xmax, 
					    double& ymin, double& ymax, 
						double& zmin, double& zmax, 
						ChMatrix33<>* Rot = NULL);
	
	virtual ChVector<> Baricenter();
	virtual void CovarianceMatrix(ChMatrix33<>& C);

				/// This is a surface
	virtual int GetManifoldDimension() {return 2;}


		//
		// CUSTOM FUNCTIONS
		//
			
			// return false if triangle has almost zero area
	bool IsDegenerated();

			// compute triangle normal
	bool   Normal(Vector& N);

	ChVector<> GetNormal() 
				{
					ChVector<> mn; this->Normal(mn); return mn;
				};



			/// Given point B and a generic triangle, computes the distance from the triangle plane,
			/// returning also the projection of point on the plane and other infos
			///			\return the signed distance
	static double PointTriangleDistance(ChVector<> B,	///< point to be measured
							   ChVector<>& A1,			///< point of triangle 
							   ChVector<>& A2,			///< point of triangle
							   ChVector<>& A3,			///< point of triangle
							   double& mu,				///< returns U parametric coord of projection
							   double& mv,				///< returns V parametric coord of projection
							   bool& is_into,			///< returns true if projection falls on the triangle
							   ChVector<>& Bprojected	///< returns the position of the projected point
							   );

			/// Given point B, computes the distance from this triangle plane,
			/// returning also the projection of point on the plane and other infos
			///			\return the signed distance
	double PointTriangleDistance(ChVector<> B,			///< point to be measured
							   double& mu,				///< returns U parametric coord of projection
							   double& mv,				///< returns V parametric coord of projection
							   bool& is_into,			///< returns true if projection falls on the triangle
							   ChVector<>& Bprojected	///< returns the position of the projected point
							   )
				{ 
					return PointTriangleDistance(B, this->p1, this->p2, this->p3, mu, mv, is_into, Bprojected); 
				}

			/// Calculate distance between a point p and a line identified
			/// with segment dA,dB. Returns distance. Also, the mu value reference
			/// tells if the nearest projection of point on line falls into segment (for mu 0...1)
			///			\return the distance
	static double PointLineDistance(ChVector<>& p,	///< point to be measured 
								ChVector<>& dA,		///< a point on the line 
								ChVector<>& dB,		///< another point on the line
								double& mu,			///< parametric coord: if in 0..1 interval, projection is between dA and dB 
								bool& is_insegment	///< returns true if projected point is between dA and dB
								);

		//
		// DATA
		//

	ChVector<> p1;
	ChVector<> p2;
	ChVector<> p3;


		//
		// STREAMING
		//

	void StreamOUT(ChStreamOutBinary& mstream);

	void StreamIN(ChStreamInBinary& mstream); 

};



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif
