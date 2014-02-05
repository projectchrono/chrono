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

#ifndef CHELEMENTHEXAHEDRON_H
#define CHELEMENTHEXAHEDRON_H

#include "ChElement3D.h"
#include "ChElementCorotational.h"


namespace chrono{
	namespace fem{


		///
		/// Class for hexahedral elements.
		///
class ChApiFem ChElementHexahedron :  
								public ChElement3D,
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