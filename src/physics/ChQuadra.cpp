///////////////////////////////////////////////////
//
//   ChQuadra.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <float.h>
#include <memory.h>

#include "core/ChMath.h"
#include "physics/ChObject.h"
#include "physics/ChQuadra.h"



namespace chrono 
{

//////////////////////////////////////
//////////////////////////////////////

// CLASS FOR GAUSS QUADRATURE

// Performs the Gauss quadrature (numerical integration) of a function,
// passed as parameter. The function must have three xyz parameters, to 
// allow at least 3d integrals, and must return a pointer to a matrix, ie.
// the numerically evaluated value/vector/matrix of the function at the
// xyz point. Note; the xyz parameters must range into the cubic space
// of limit -1 .. +1  , so the integrated function must be a _normalized_
// function!. 
 

void ChQuadrature::GaussIntegrate (ChFelem* element, 
					ChMatrix<>* result, int orderX, int orderY, int orderZ, int dimensions)
{
	ChMatrixDynamic<>* temp= new ChMatrixDynamic<> (result->GetRows(), result->GetColumns()) ; 
	ChMatrix<>* mpoint;

			// weights  [order][npoint 0..n][0=weight/1=point]
			// {{weight, samplepoint..}, {weight, sample}, etc.}
	double weights  [3][3][2] = { 
		{{2,0}, {0,0}, {0,0}},	
		{{1,0.57735026919},{1,-0.57735026919}, {0,0}},
		{{0.555555555555555, sqrt(0.6)}, {0.8888888888888, 0}, {0.555555555555, -(sqrt(0.6))}}  };

	result->Reset();

	Vector weight;
	Vector position;
	double totweight;

	int ix; int iy; int iz;

	for (iz = 0; iz < orderZ; iz ++)
	{
		for (iy = 0; iy < orderY; iy ++)
		{
			for (ix = 0; ix < orderX; ix ++)
			{
				weight.x= weights[(orderX -1)][ix][0];
				weight.y= weights[(orderY -1)][iy][0];
				weight.z= weights[(orderZ -1)][iz][0];

				position.x= weights[(orderX -1)][ix][1];
				position.y= weights[(orderY -1)][iy][1];
				position.z= weights[(orderZ -1)][iz][1];

				if (dimensions == 2) { 
					weight.z=1; position.z=0;}; 
				if (dimensions == 1) { 
					weight.z=1; position.z=0; 
					weight.y=1; position.y=0;};

				totweight = weight.x * weight.y * weight.z;

					// %%%  Evaluate integral argument here!  %%% //
				

				mpoint = element->Compute_BEBj(position.x, position.y, position.z);


				// *parameter = position;
				// mpoint = (function)(); 
				

				// **** should be better (function)(parameter), but does not work!
				// There are problem when passing parameters to the function. Why? ***

					// update the sum of the numerical integral  
				temp->CopyFromMatrix ( *mpoint);
				temp->MatrScale(totweight);
				result->MatrInc(*temp);		
			}
		}
	}

	delete temp;
}

} // END_OF_NAMESPACE____

