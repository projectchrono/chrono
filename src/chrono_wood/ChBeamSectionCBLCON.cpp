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

#include "chrono_wood/ChBeamSectionCBLCON.h"

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


    

ChMatrixDynamic<> ChBeamSectionCBLCON::calculate_MI(double L, ChVector3d PN, ChVector3d PC){	
    ChMatrixDynamic<> MI(6, 6);
    MI.setZero();        
    //ChVector3d cm=(PN-PC);  
    //std::cout<<"------------------------------------------------------"<<std::endl;    
    //
    double w=this->GetWidth(); 
    double h=this->GetHeight(); 
    double Area=this->Get_area();
    double Ve=Area*abs(L);
    double rho=this->Get_material()->Get_density();
    double mass=Ve*rho;   
    
    double MJxx; // eccentricity in x direction equals to L/2  ----->  Jxx=(mass*L*L/12+mass*(L/2)*(L/2)
    double MJyy;
    double MJzz;
    double MJyz;
    //
    MI.setZero();
    MI(0, 0) = mass;
    MI(1, 1) = mass;
    MI(2, 2) = mass;
    //
    switch(this->GetSectionType()) {
        case longitudinal:
            // code block
            //std::cout << "This is section for longitudinal con" << std::endl;
            MJxx=mass*L*L/3.; // eccentricity in x direction equals to L/2  ----->  Jxx=(mass*L*L/12+mass*(L/2)*(L/2)
	    MJzz=mass*h*h/3.;
	    MJyy=mass*w*w/12.;
	
	    //std::cout<<"mass: "<<mass<<std::endl;
	    //std::cout<<"MJxx: "<<MJxx<<"\t";
	    //std::cout<<"MJyy: "<<MJyy<<"\t";
	    //std::cout<<"MJzz: "<<MJzz<<"\n";
	    //
	    MI(3, 3) = MJyy + MJzz;
	    MI(4, 4) = MJyy + MJxx;
	    MI(5, 5) = MJzz + MJxx;
	   
	    
	    //double cm_x=L/2.;    
	    MI(1, 5) = mass * L/2;
	    MI(2, 4) = -mass * L/2;  
	    MI(5, 1) = MI(1, 5);  
	    MI(4, 2) = MI(2, 4);
            //
            MI(3, 4) = -mass * h*L/4;  
    	    MI(4, 3) = MI(3, 4);    	   
    	    //
            //MI(5, 5) += mass * cm.y()*cm.y();
            MI(3, 2) = mass * h/2;    
    	    MI(5, 0) = -mass * h/2;    
    	    MI(2, 3) = MI(3, 2);    
    	    MI(0, 5) = MI(5, 0);
    	    //std::cout<<"Long MJxx, MJyy, MJzz: "<<MJxx<<"\t"<<MJyy<<"\t"<<MJzz<<std::endl;
    	    //std::cout<<"MI:\n"<<MI<<std::endl;
            break;
        case transverse_top:
            // code block
            //std::cout << "This is section for transverse_top con" << std::endl;
            MJxx=mass*L*L/3.; // eccentricity in x direction equals to L/2  ----->  Jxx=(mass*L*L/12+mass*(L/2)*(L/2)
	    MJyy=mass*w*w/12.;
	    MJzz=mass*h*h/3.;
	    
	    //std::cout<<"mass: "<<mass<<std::endl;
	    //std::cout<<"MJxx: "<<MJxx<<"\t";
	    //std::cout<<"MJyy: "<<MJyy<<"\t";
	    //std::cout<<"MJzz: "<<MJzz<<"\n";
            //
	    MI(3, 3) = MJyy + MJzz;
	    MI(4, 4) = MJyy + MJxx;
	    MI(5, 5) = MJzz + MJxx;
	    MI(4, 5) = -MJyz;
	    MI(5, 4) = -MJyz;
	    
	    //double cm_x=L/2.;    
	    MI(1, 5) = mass * L/2;
	    MI(2, 4) = -mass * L/2;  
	    MI(5, 1) = MI(1, 5);  
	    MI(4, 2) = MI(2, 4);
	    MI(3, 4) = mass * h*L/4;  
    	    MI(4, 3) = MI(3, 4);  
            //            
            MI(3, 2) = -mass * h/2;    
    	    MI(5, 0) = mass * h/2;    
    	    MI(2, 3) = MI(3, 2);    
    	    MI(0, 5) = MI(5, 0);
    	    //std::cout<<"Top MJxx, MJyy, MJzz: "<<MJxx<<"\t"<<MJyy<<"\t"<<MJzz<<std::endl;
    	    //std::cout<<"MI:\n"<<MI<<std::endl;
            break;
        case transverse_bot:
            // code block
            //std::cout << "This is section for transverse_bot con" << std::endl;
            MJxx=mass*L*L/3.; // eccentricity in x direction equals to L/2  ----->  Jxx=(mass*L*L/12+mass*(L/2)*(L/2)
	    MJyy=mass*w*w/12.;
	    MJzz=mass*h*h/3.;
	
	    //std::cout<<"mass: "<<mass<<std::endl;
	    //std::cout<<"MJxx: "<<MJxx<<"\t";
	    //std::cout<<"MJyy: "<<MJyy<<"\t";
	    //std::cout<<"MJzz: "<<MJzz<<"\n";
            //
	    MI(3, 3) = MJyy + MJzz;
	    MI(4, 4) = MJyy + MJxx;
	    MI(5, 5) = MJzz + MJxx;
	    //MI(4, 5) = -MJyz;
	    //MI(5, 4) = -MJyz;
	    
	    //double cm_x=L/2.;    
	    MI(1, 5) = mass * L/2;
	    MI(2, 4) = -mass * L/2;  
	    MI(5, 1) = MI(1, 5);  
	    MI(4, 2) = MI(2, 4);
	    MI(3, 4) = -mass * h*L/4;  
    	    MI(4, 3) = MI(3, 4);  
            //            
            MI(3, 2) = mass * h/2;    
    	    MI(5, 0) = -mass * h/2;    
    	    MI(2, 3) = MI(3, 2);    
    	    MI(0, 5) = MI(5, 0);
    	    //std::cout<<"Bot MJxx, MJyy, MJzz: "<<MJxx<<"\t"<<MJyy<<"\t"<<MJzz<<std::endl;
    	    //std::cout<<"MI:\n"<<MI<<std::endl;    	    
            break;  
        default:
            MJxx=mass*L*L/3.; // eccentricity in x direction equals to L/2  ----->  Jxx=(mass*L*L/12+mass*(L/2)*(L/2)
	    MJyy=mass*w*w/12.;
	    MJzz=mass*h*h/12.;
	    
	    //std::cout<<"mass: "<<mass<<std::endl;
	    //std::cout<<"MJxx: "<<MJxx<<"\t";
	    //std::cout<<"MJyy: "<<MJyy<<"\t";
	    //std::cout<<"MJzz: "<<MJzz<<"\n";
            //
	    MI(3, 3) = MJyy + MJzz;
	    MI(4, 4) = MJyy + MJxx;
	    MI(5, 5) = MJzz + MJxx; 
	        
	    //double cm_x=L/2.;    
	    MI(1, 5) = mass * L/2;
	    MI(2, 4) = -mass * L/2;  
	    MI(5, 1) = MI(1, 5);  
	    MI(4, 2) = MI(2, 4);   
    	    //
            //std::cout<<"Gen MJxx, MJyy, MJzz: "<<MJxx<<"\t"<<MJyy<<"\t"<<MJzz<<std::endl;
    	    //std::cout<<"MI:\n"<<MI<<std::endl;  
    	       	    
                
    }
	//std::cout<<"MI\n:"<<MI<<std::endl;
	//MI.block(3, 3, 3, 3)*=1.0E+9;
    return MI;
}


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
