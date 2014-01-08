//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//
// File authors: Andrea Favali, Alessandro Tasora

#ifndef CHELEMENT3D_H
#define CHELEMENT3D_H

#include "ChElementGeneric.h"
#include "ChGaussIntegrationRule.h"
#include "ChPolarDecomposition.h"
#include "ChMatrixCorotation.h"

namespace chrono{
	namespace fem{


		/// Class for all 3-Dimensional elements. 

class ChApiFem ChElement3D : public ChElementGeneric
{
protected:
	double Volume;

public:

	double GetVolume() {return Volume;}
};




		/// Class for corotational elements (elements with rotation 
		/// matrices that follow the global motion of the element)
class ChApiFem ChElementCorotational //: public ChElement3D
{
protected:
	ChMatrix33<> A;	// rotation matrix

public:
	ChElementCorotational () { A(0,0)=1;  A(1,1)=1;  A(2,2)=1; }
	
					/// Access the rotation matrix of the element.
	ChMatrix33<>& Rotation() {return A;}

					/// Given the actual position of the nodes, recompute
					/// the rotation matrix A. 
					/// CHLDREN CLASSES MUST IMPLEMENT THIS!!!
	virtual void UpdateRotation() = 0; 
};



		///
		/// Class for tetrahedral elements.
		///
class ChApiFem ChTetrahedron : public ChElement3D,
							   public ChElementCorotational																
																//		  /|\						//
{																//		 / |  \						//
protected:														//		/  |	\					//
																//	   /.  |	  \					//
public:															//	   \   |.		\				//
	int ID;														//		\  |	.	  \				//
																//		 \ |		.	\			//
																//		  \|__ __ __ __'__\			//
	  
	virtual void Update() 
				{
					// parent class update:
					ChElement3D::Update();
					// always keep updated the rotation matrix A:
					this->UpdateRotation();
				};
															
};																



		///
		/// Class for hexahedral elements.
		///
class ChApiFem ChHexahedron :  public ChElement3D,
							   public ChElementCorotational	  	
																//		    __ __ __ __				//
{																//		  /			  /|			//				
protected:														//		 /_|__ __ __ / |			//
	ChGaussIntegrationRule* ir;									//		|			|  |			//
	std::vector<ChGaussPoint*> GpVector;						//		|  |		|  |			//
																//		|	 __	 __	|  |			//	
																//		| /			| /				//
																//		|__ __ __ __|/				//
public:															
	int ID;		


	virtual void Update() 
				{
					// parent class update:
					ChElement3D::Update();
					// always keep updated the rotation matrix A:
					this->UpdateRotation();
				};
};																			  




	}//___end of namespace fem___
}//___end of namespace chrono___

#endif