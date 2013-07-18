#ifndef CHELEMENT3D_H
#define CHELEMENT3D_H

#include "fem/ChElementGeneric.h"

namespace chrono{
	namespace fem{

/// Class for all 3-Dimensional elements 


class ChApi ChElement3D : public ChElementGeneric
{
protected:
	double Volume;

public:
	/// Computes the volume of the element (and stores the value in this->volume)
	virtual double ComputeVolume() = 0;
	double GetVolume() {return Volume;}


};



//struct MatrixAndDet						// use this structure to keep in memory booth 'MatrB'  
//	{										// and Jacobian determinant (necessary for integration)
//		ChMatrixDynamic<> Matrix;			// Not necessary because we store all MatrB matrices
//		double JacobianDet;
//	};


	//////// calss for tetrahedral elements
class ChApi ChTetrahedron : public ChElement3D					//		  /|\						//
{																//		 / |  \						//
protected:														//		/  |	\					//
																//	   /.  |	  \					//
public:															//	   \   |.		\				//
	int ID;														//		\  |	.	  \				//
																//		 \ |		.	\			//
};																//		  \|__ __ __ __'__\			//


															
class ChApi ChHexahedron : public ChElement3D					//		    __ __ __ __				//
{																//		  /			  /|			//				
protected:														//		 /_|__ __ __ / |			//
																//		|			|  |			//
public:															//		|  |		|  |			//
	int ID;														//		|	 __	 __	|  |			//
																//		| /			| /				//		
};																//		|__ __ __ __|/				//			  




	}//___end of namespace fem___
}//___end of namespace chrono___

#endif