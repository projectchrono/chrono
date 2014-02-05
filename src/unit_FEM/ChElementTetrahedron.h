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

#ifndef CHELEMENTTETRAHEDRON_H
#define CHELEMENTTETRAHEDRON_H

#include "ChElement3D.h"
#include "ChElementCorotational.h"


namespace chrono{
	namespace fem{



		///
		/// Class for tetrahedral elements.
		///
class ChApiFem ChElementTetrahedron : 
								public ChElement3D,
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



	}//___end of namespace fem___
}//___end of namespace chrono___

#endif