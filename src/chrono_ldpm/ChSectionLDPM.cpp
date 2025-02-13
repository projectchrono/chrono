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
// Authors: Erol Lale
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

#include "chrono_ldpm/ChSectionLDPM.h"

namespace chrono {
namespace ldpm {

// Construct CSL or LDPM section.

ChSectionLDPM::ChSectionLDPM( std::shared_ptr<ChMaterialVECT> material,  // material 
                                    double area,    // Projected total area of the facet
                                    ChVector3d center,    // Center point of the facet area      
                                    ChMatrix33<double> facetFrame    /// local system of frame of facet 
                                       )
    : m_material{material}, m_area{area}, m_center{center}, m_facetFrame{facetFrame} {
        
}

ChSectionLDPM::ChSectionLDPM() {};

ChSectionLDPM::~ChSectionLDPM() {};

void ChSectionLDPM::ComputeProjectionMatrix() {	
	mprojection_matrix.setZero();
	//
	ChMatrix33<double> nmL=this->Get_facetFrame();
	//
	double one_sqrt2=1.0/sqrt(2.0);
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

void ChSectionLDPM::ComputeEigenStrain(std::shared_ptr<ChMatrixNM<double,1,9>> macro_strain){
	ChVectorDynamic<> eigen_strain;					
	auto pp=this->GetProjectionMatrix();
	eigen_strain = -pp * macro_strain->transpose();	
	this->Set_nonMechanicStrain(eigen_strain);				
			
}

//pSM =[n11*n21, n12*n22, n13*n23, n11*n22 + n12*n21, n11*n23 + n13*n21, n12*n23 + n13*n22]
//pSL =[n11*n31, n12*n32, n13*n33, n11*n32 + n12*n31, n11*n33 + n13*n31, n12*n33 + n13*n32]

}  // end of namespace ldpm
}  // end of namespace chrono
