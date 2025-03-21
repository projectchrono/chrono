// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Erol Lale, Jibril B. Coulibaly
// =============================================================================
// Section class for LDPM and CSL elements 
//
//  i)   Material
//  ii)  Projected area of facets
//  iii) Center of facets  
//  iv)  System of reference of facets
//
// A description of the material parameter can be found in: https://doi.org/10.1016/j.cemconcomp.2011.02.011
// =============================================================================

#include "chrono_wood/ChBeamSectionCBLCON.h"
#include "chrono/core/ChMatrix.h"

namespace chrono {
namespace wood {

// Construct CSL or LDPM section.

ChBeamSectionCBLCON::ChBeamSectionCBLCON( std::shared_ptr<ChWoodMaterialVECT> material,  // material 
                                    double area,    // Projected total area of the facet
                                    ChVector3d center,    // Center point of the facet area      
                                    ChMatrix33<double> facetFrame    /// local system of frame of facet 
                                       )
    : m_material{material}, m_area{area}, m_center{center}, m_facetFrame{facetFrame} {
        m_state.setZero();
}

ChBeamSectionCBLCON::ChBeamSectionCBLCON() {
	m_state.setZero();
};

ChBeamSectionCBLCON::~ChBeamSectionCBLCON() {};


void ChBeamSectionCBLCON::ComputeProjectionMatrix() {	
	mprojection_matrix.setZero();
	//
	ChMatrix33<double> nmL=this->Get_facetFrame();
	//	
	//
	//
	for (int i=0; i<3; i++){
		mprojection_matrix(i,0)=nmL(0,0)*nmL(i,0);  			// e_xx
		mprojection_matrix(i,1)=nmL(0,1)*nmL(i,1);			// e_yy
		mprojection_matrix(i,2)=nmL(0,2)*nmL(i,2);			// e_zz
		mprojection_matrix(i,3)=(nmL(0,0)*nmL(i,1));			// e_xy
		mprojection_matrix(i,4)=(nmL(0,0)*nmL(i,2));			// e_xz
		mprojection_matrix(i,5)=(nmL(0,1)*nmL(i,2));			// e_yz	
		mprojection_matrix(i,6)=(nmL(0,1)*nmL(i,0));			// e_yx
		mprojection_matrix(i,7)=(nmL(0,2)*nmL(i,0));			// e_zx
		mprojection_matrix(i,8)=(nmL(0,2)*nmL(i,1));			// e_zy	
		
	}	
		
	
		
};

void ChBeamSectionCBLCON::ComputeEigenStrain(std::shared_ptr<ChMatrixNM<double,1,9>> macro_strain){
	ChVectorDynamic<> eigen_strain;					
	auto pp=this->GetProjectionMatrix();
	eigen_strain = -pp * macro_strain->transpose();	
	this->Set_nonMechanicStrain(eigen_strain);				
			
}



}  // end of namespace wood
}  // end of namespace chrono
