#ifndef CHCOLLIDE_H
#define CHCOLLIDE_H

//////////////////////////////////////////////////
//  
//   ChCollide.h
//
//   Class for collision utilities
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "physics/ChBody.h"


namespace chrono 
{

//using namespace collision;



//
// COLLISION UTILITIES
//



		/// Calculate the line segment PaPb that is the shortest route between
		/// two lines P1P2 and P3P4. Calculate also the values of mua and mub where
		///    Pa = P1 + mua (P2 - P1)
		///    Pb = P3 + mub (P4 - P3)
		/// Return FALSE if no solution exists.

int ChLineLineIntersect(
   Vector p1, Vector p2,Vector p3,Vector p4,Vector *pa,Vector *pb,
   double *mua, double *mub);



		/// Calculate distance between a point p and a line identified
		/// with segment dA,dB. Returns distance. Also, the mu value reference
		/// tells if the nearest projection of point on line falls into segment (for mu 0...1)

double ChPointLineDistance(Vector p, Vector dA, Vector dB, double& mu, int& is_insegment);



		/// Calculate distance of a point from a triangle surface. 
		/// Also computes if projection is inside the triangle.

double ChPointTriangleDistance(Vector B, Vector A1, Vector A2, Vector A3, 
							   double& mu, double& mv, int& is_into,
							   Vector& Bprojected);





} // END_OF_NAMESPACE____


#endif  // END of ChCollide.h 
