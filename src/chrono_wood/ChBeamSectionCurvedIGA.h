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
//
#ifndef CH_BEAM_SECTION_CURVED_H
#define CH_BEAM_SECTION_CURVED_H

#include "chrono_wood/ChWoodApi.h"
#include "chrono/fea/ChBeamSection.h"
#include "chrono/fea/ChBeamSectionCosserat.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChMatrix33.h"
#include "chrono/core/ChQuadrature.h"

using namespace chrono;
using namespace chrono::fea;

namespace chrono {
namespace wood {

/// @addtogroup wood_utils
/// @{

struct rectangle_wing {    
    double height;
    double width;
    double angle;   
};

/// Internal variables for basic lumped plasticity in Cosserat beams.
class ChWoodApi ChInternalDataPlastic : public ChBeamMaterialInternalData {
  public:
    ChInternalDataPlastic() {}

    virtual ~ChInternalDataPlastic() {}

    virtual void Copy(const ChBeamMaterialInternalData& other) override {
        ChBeamMaterialInternalData::Copy(other);

        if (auto mother = dynamic_cast<const ChInternalDataPlastic*>(&other)) {
            p_strain_e = mother->p_strain_e;
            p_strain_k = mother->p_strain_k;
            //p_strain_acc_e = mother->p_strain_acc_e;
            //p_strain_acc_k = mother->p_strain_acc_k;
        }
    }

    //ChVector3d p_strain_acc_e;  // separate strain accumulator for xyz
    //ChVector3d p_strain_acc_k;  // separate strain accumulator for xyz
    ChVector3d p_strain_e;
    ChVector3d p_strain_k;
};


class CurvedBeamSection : public ChBeamSectionCosserat {
protected:
    CurvedBeamSection(){};    
    std::vector<rectangle_wing> props;
    int nwings=1;
    double width_y=1.;  // width of section in y direction
    double width_z=1.;  // width of section in z direction
	double area=1.; // area of the section
	double Cy=0.5;  // eccentricity of section in normal direction 
	double Cz=0.0;  // eccentricity of section in binormal direction
public:
    CurvedBeamSection( 
        std::shared_ptr<ChInertiaCosserat> minertia,
        std::shared_ptr<ChElasticityCosserat> melasticity,
        std::shared_ptr<ChPlasticityCosserat> mplasticity = nullptr,
        std::shared_ptr<ChDampingCosserat> mdamping = nullptr,
        double mwidth_y = 1.0,  // Default width
        double mwidth_z = 1.0  // Default height
        ) : ChBeamSectionCosserat(minertia, melasticity, mplasticity, mdamping),
           width_y(mwidth_y), 
           width_z(mwidth_z) {}

    virtual ~CurvedBeamSection() {}
    
    // Setters and getters for width and height
    void SetNwings(double mnwings) { nwings = mnwings; }
    double GetNwings() const { return nwings; }
    
    void SetWidth_y(double mwidth_y) { width_y = mwidth_y; }
    double GetWidth_y() const { return width_y; }

    void SetWidth_z(double mwidth_z) { width_z = mwidth_z; }
    double GetWidth_z() const { return width_z; }
	
	void SetCy(double mCy) { Cy = mCy; }
    double GetCy() const { return Cy; }
	
	void SetCz(double mCz) { Cz = mCz; }
    double GetCz() const { return Cz; }
    
	void SetArea(double marea) { area = marea; }
    double GetArea() const { return area; }
	
    void SetProps(rectangle_wing mwing) { props.push_back(mwing); }
    void SetProps(std::vector<rectangle_wing> mprops) { props = mprops; }
    rectangle_wing GetPropsN(int i) const { return props[i]; }
    std::vector<rectangle_wing> GetProps() const { return props; }
    
    std::vector<double> EasyMultiWingsSection(const std::vector<rectangle_wing>& mwings) {
    
    	double w;
	double A=0; double Sn=0; double Sb=0; 
    	double Inn=0; double Inb=0; double Ibb=0;  
    	//
    	double multiplier; 
    	ChMatrix33<> minertia, newinertia, rot; 	   	
    	//
    	for (unsigned int iwing = 0; iwing < nwings; iwing++){
    		/*double D = this->GetWidth_y();
    		double Bw = this->GetWidth_z();    	
    		double alpha = 0; //this->section->alpha; */
    		
    		double D = this->props[iwing].height;
    		double Bw = this->props[iwing].width;    
    		double alpha = props[iwing].angle; 
    		//printf("wing: %d \t D: %f \t Bw: %f\t alpha: %f\n", iwing, D, Bw, alpha);
    		//alpha-=atan2(nrml[1],nrml[0]);
    		double w_A = D * Bw;
    		double w_Ibb = (1.0 / 12.0) * Bw * pow(D, 3);
    		double w_Inn = (1.0 / 12.0) * D * pow(Bw, 3);   
    		//
    		double cy=D*Cy; //eccentricity in -n direction    		
    		w_Ibb+=w_A*cy*cy;   		   		
    		double S=w_A*cy; 
    		
    		double c=cos(alpha);
    		double s=sin(alpha);
    		double c2=c*c;
    		double s2=s*s;
    		double cs=c*s;
    		
    		double w_Sn = S* s; 
    		double w_Sb = S* c;
    		
		//
		
		A   +=  w_A;
		Sn  +=  w_Sn;
		Sb  +=  w_Sb;
		Inn +=  w_Inn*c2 + w_Ibb*s2;
		Inb +=  (w_Ibb - w_Inn)*cs;
		Ibb +=  w_Ibb*c2 + w_Inn*s2;
		
		//   
        }
                
        //printf("Easy A: %22.15f, Sn: %22.15f, Sb: %22.15f, Inn: %22.15f, Inb: %22.15f, Ibb: %22.15f\n", A, Sn, Sb, Inn, Inb, Ibb);
		this->SetArea(A);
        std::vector<double> props{A, Sn, Sb, Inn, Inb, Ibb};
        return props;
        
    }

    // Override functions if needed
    virtual void ComputeStress(
        ChVector3d& stress_n,
        ChVector3d& stress_m,
        const ChVector3d& strain_n,
        const ChVector3d& strain_m,
        ChBeamMaterialInternalData* mdata_new = nullptr,
        const ChBeamMaterialInternalData* mdata = nullptr ) override {
        // Implement your custom stress computation here
        ChVectorN<double, 6> mstrain;
    	ChVectorN<double, 6> mstress;
    	ChMatrixNM<double, 6, 6> Dmat;
    	//
		if (mdata_new){
			auto mydata = dynamic_cast<const ChInternalDataPlastic*>(mdata);
			auto mydata_new = dynamic_cast<ChInternalDataPlastic*>(mdata_new);
			Eigen::Vector3d p_strain_e( mydata->p_strain_e.x(), mydata->p_strain_e.y(), mydata->p_strain_e.z() );
			mstrain.segment(0, 3) = strain_n.eigen() - p_strain_e;
			//mstrain.segment(3, 3) = strain_m.eigen() - mdata->p_curvature_acc;
		}else{
			mstrain.segment(0, 3) = strain_n.eigen();
			mstrain.segment(3, 3) = strain_m.eigen();
		}
    	//
        this->GetElasticity()->ComputeStiffnessMatrix(Dmat, strain_n, strain_m);
        //   
        mstress = Dmat * mstrain;       
    	stress_n = mstress.segment(0, 3);
    	stress_m = mstress.segment(3, 3);
    };
    
    virtual void ComputeStress(
        ChVector3d& stress_n,
        ChVector3d& stress_m,
        const ChVector3d& strain_n,
        const ChVector3d& strain_m,
        ChMatrixNM<double, 6, 6>& Dmat,
        ChBeamMaterialInternalData* mdata_new = nullptr,
        const ChBeamMaterialInternalData* mdata = nullptr ) {
        // Implement your custom stress computation here
        ChVectorN<double, 6> mstrain;
    	ChVectorN<double, 6> mstress;    	
    	//
    	if (mdata_new){
			auto mydata = dynamic_cast<const ChInternalDataPlastic*>(mdata);						
			auto mydata_new = dynamic_cast<ChInternalDataPlastic*>(mdata_new);
			Eigen::Vector3d p_strain_e( mydata->p_strain_e.x(), mydata->p_strain_e.y(), mydata->p_strain_e.z() );
			mstrain.segment(0, 3) = strain_n.eigen() - p_strain_e;
			//mstrain.segment(3, 3) = strain_m.eigen() - mdata->p_curvature_acc;
		}else{
			mstrain.segment(0, 3) = strain_n.eigen();
			mstrain.segment(3, 3) = strain_m.eigen();
		}
    	//        
        //   
        mstress = Dmat * mstrain;       
    	stress_n = mstress.segment(0, 3);
    	stress_m = mstress.segment(3, 3);
    };
    
    
    std::vector<double> ComputeSectionCharacteristic( const double kappa, ///< curvature
                            const int order)               ///< order of integration
    {
        ChQuadratureTables* mtables = 0;
        std::vector<double>* lroots;
        std::vector<double>* weight;
        bool static_tables;

        if ((unsigned int)order <= ChQuadrature::GetStaticTables()->Lroots.size()) {
            mtables = ChQuadrature::GetStaticTables();
            lroots = &mtables->Lroots[order - 1];
            weight = &mtables->Weight[order - 1];
            static_tables = true;
        } else {
            mtables = new ChQuadratureTables(order, order);
            mtables->PrintTables();
            lroots = &mtables->Lroots[0];
            weight = &mtables->Weight[0];
            static_tables = false;
        }
       
        //
	double w;
	double A=0; double Sn=0; double Sb=0; 
    	double Inn=0; double Inb=0; double Ibb=0;  
    	//
    	double multiplier;  	   	
    	//
    	for (unsigned int iwing = 0; iwing < nwings; iwing++){
    		/*double D = this->GetWidth_y();
    		double Bw = this->GetWidth_z();    	
    		double alpha = 0; //this->section->alpha; */
    		
    		double D = this->props[iwing].height;
    		double Bw = this->props[iwing].width;    
    		double alpha = props[iwing].angle;     		
    		//alpha-=atan2(nrml[1],nrml[0]);
    		double jacob=D* Bw / 4.;
			double cy=D*Cy;
			double cz=Bw*Cz;
    		//
		for (unsigned int ix = 0; ix < lroots->size(); ix++){  		        
		    for (unsigned int iy = 0; iy < lroots->size(); iy++) {  		                                            
		        w = (weight->at(ix) * weight->at(iy));                     
		        double pnprime = D/2. * lroots->at(ix) + cy ;
				double pbprime = Bw/2. * lroots->at(iy) + cz ;  
				//
				double pn = std::cos(alpha)*pnprime - std::sin(alpha)*pbprime;
				double pb = std::sin(alpha)*pnprime + std::cos(alpha)*pbprime; 			
				//
				multiplier=  jacob * w;    
					//
				A   +=  1.0/(1.0-kappa*pn)*multiplier;
				Sn  +=  pb/(1.0-kappa*pn)*multiplier;
				Sb  +=  pn/(1.0-kappa*pn)*multiplier;
				Inn +=  pb*pb/(1.0-kappa*pn)*multiplier;
				Inb +=  pn*pb/(1.0-kappa*pn)*multiplier;
				Ibb +=  pn*pn/(1.0-kappa*pn)*multiplier;
		    }
		}  
		//   
        } 
		this->SetArea(A);
	std::vector<double> props{A, Sn, Sb, Inn, Inb, Ibb};
	//std::cout<<"/nA: "<<A<<"  Sn: "<<Sn<<"  Sb: "<<Sb<<"  Inn: "<<Inn<<"  Ibb: "<<Ibb<<"  Inb: "<<Inb<<std::endl;
	
        if (!static_tables)
            delete mtables;
        
        return props;
    }
        
};


/// A simple specialization of CurvedBeamSection if you do not need to define
/// its separate models for elasticity, plasticity, damping and inertia.
/// Good if you just need the simplest model for a rectangular centered beam. This section automatically
/// creates, initializes and embeds, at construction, these models:
/// - elasticity: ChElasticityCosseratSimple
/// - inertia:    ChInertiaCosseratSimple
/// - damping:    none   - you can add it later
/// - plasticity: none
class ChWoodApi ChBeamSectionCurvedEasyRectangular : public CurvedBeamSection {
  public:
    ChBeamSectionCurvedEasyRectangular(double width_y,  ///< width of section in y direction
                                         double width_z,  ///< width of section in z direction
                                         double E,        ///< Young modulus
                                         double G,        ///< shear modulus
                                         double density   ///< volumetric density (ex. in SI units: [kg/m^3])
    ) {
	    auto melasticity = chrono_types::make_shared<ChElasticityCosseratSimple>();
	    melasticity->SetYoungModulus(E);
	    melasticity->SetShearModulus(G);
	    melasticity->SetAsRectangularSection(width_y, width_z);
		this->SetArea(width_y*width_z);
	    this->SetElasticity(melasticity);

	    auto minertia = chrono_types::make_shared<ChInertiaCosseratSimple>();
	    minertia->SetAsRectangularSection(width_y, width_z, density);
	    this->SetInertia(minertia);

	    auto mdrawshape = chrono_types::make_shared<ChBeamSectionShapeRectangular>(width_y, width_z);
	    this->SetDrawShape(mdrawshape);
	}
};

/// A simple specialization of CurvedBeamSection if you do not need to define
/// its separate models for elasticity, plasticity, damping and inertia.
/// Good if you just need the simplest model for a circular centered beam. This section automatically
/// creates, initializes and embeds, at construction, these models:
/// - elasticity: ChElasticityCosseratSimple
/// - inertia:    ChInertiaCosseratSimple
/// - damping:    none   - you can add it later
/// - plasticity: none
class ChWoodApi ChBeamSectionCurvedEasyCircular : public CurvedBeamSection {
  public:
    ChBeamSectionCurvedEasyCircular(double diameter,  ///< diameter of section
                                      double E,         ///< Young modulus
                                      double G,         ///< shear modulus
                                      double density    ///< volumetric density (ex. in SI units: [kg/m^3])
    );
};



/// A simple specialization of CurvedBeamSection if you do not need to define
/// its separate models for elasticity, plasticity, damping and inertia.
/// Good if you just need the simplest model for a circular centered beam. This section automatically
/// creates, initializes and embeds, at construction, these models:
/// - elasticity: ChElasticityCosseratSimple
/// - inertia:    ChInertiaCosseratSimple
/// - damping:    none   - you can add it later
/// - plasticity: none
class ChWoodApi ChBeamSectionCurvedEasyMultiWings : public CurvedBeamSection {
  public:
    ChBeamSectionCurvedEasyMultiWings( const std::vector<rectangle_wing>& mwings,                                      
                                         double E,        ///< Young modulus
                                         double G,        ///< shear modulus
                                         double density   ///< volumetric density (ex. in SI units: [kg/m^3])
    ){
    	    
    	    this->SetNwings(mwings.size());
    	    this->SetProps(mwings);
    	    auto melasticity = chrono_types::make_shared<ChElasticityCosseratAdvancedGenericFPM>();
	    //melasticity->SetYoungModulus(E);
	    //melasticity->SetShearModulus(G);
	    //melasticity->SetAsRectangularSection(width_y, width_z);
	    //
	    // Compute sectional properties
	    //	   
	    std::vector<double> sec_props=this->EasyMultiWingsSection(mwings);
	    //std::vector<double> sec_props = ComputeSectionCharacteristic(0, 2);
		this->SetArea(sec_props[0]);
	    double A=sec_props[0]; 
	    double Sn=sec_props[1]; 
	    double Sb=sec_props[2]; 
	    double Inn=sec_props[3]; 
	    double Inb=sec_props[4]; 
	    double Ibb=sec_props[5];
	    double J= Inn + Ibb;
		
	    //std::cout<<"A: "<<A*density<<"  Sn: "<<Sn*density<<"  Sb: "<<Sb*density<<"  Inn: "<<Inn*density<<"  Ibb: "<<Ibb*density<<"  Inb: "<<Inb*density<<std::endl;
	    //
	    // Set sectional stiffness
	    //
	    ChMatrixNM<double, 6, 6> Dmat;
	    Dmat.setZero();	
	    //
	    Dmat(0,0)=A*E; Dmat(0,4)=Sn*E; Dmat(0,5)=-Sb*E; 
	    Dmat(1,1)=0.85*A*G; Dmat(1,3)=-G*Sn;
	    Dmat(2,2)=0.85*A*G; Dmat(2,3)= G*Sb;
	    Dmat(3,1)=-Sn*G; Dmat(3,2)= G*Sb; Dmat(3,3)=G*J;
	    Dmat(4,0)=Sn*E; Dmat(4,4)=Inn*E; Dmat(4,5)=-Inb*E;
	    Dmat(5,0)=-Sb*E; Dmat(5,4)=-Inb*E; Dmat(5,5)=Ibb*E;    
	    //
	    melasticity->SetStiffnessMatrix(Dmat);
	    this->SetElasticity(melasticity);
	    //
	    // Set Inertia properties
	    //
	    auto minertia = chrono_types::make_shared<ChInertiaCosseratAdvanced>(A*density, Sb/A, Sn/A, Inn*density, Ibb*density, Inb*density); // mu_density=A*rho,  c_y,  c_z,  Jyy_moment,  Jzz_moment,  Jyz_moment 	     
	    this->SetInertia(minertia);
	    
	    //ChMatrixNM<double, 6, 6> M;
	    //minertia->ComputeInertiaMatrix(M);
	    //std::cout<<"M:\n"<<M<<std::endl;
	    
	    
	    //
	    // Set sectional shape
	    //	 
	    
	    //std::vector<std::vector<ChVector3d > > polyline_points;
    	    //for (unsigned int iwing = 0; iwing < size(mwings); iwing++){
    		/*double D = this->GetWidth_y();
    		double Bw = this->GetWidth_z();    	
    		double alpha = 0; //this->section->alpha; */
    		
    		////auto props=newsection->GetProps();
    		//double hw = mwings[iwing].height;
    		//double Bw = mwings[iwing].width;    
    		//double alpha = mwings[iwing].angle;     		
    		
    		//double c=cos(alpha);
    		//double s=sin(alpha);
    		
    		//std::vector<ChVector3d >  rectangul={{0, -s*Bw/2, c*Bw/2}, {0, hw*c-s*Bw/2, -hw*s-c*Bw/2}, {0, hw*c+s*Bw/2, -hw*s+c*Bw/2}, {0, s*Bw/2, c*Bw/2}};
    		//polyline_points.push_back(rectangul);   
    	    //}
    	
    	
    	    //auto msection_drawshape = chrono_types::make_shared<ChBeamSectionShapePolyline>(polyline_points);
    	    //this->SetDrawShape(msection_drawshape);
	    
	    
	      
	    //auto mdrawshape = chrono_types::make_shared<ChBeamSectionShapeRectangular>(width_y, width_z);
	    //auto msection_drawshape = chrono_types::make_shared<ChBeamSectionShapePolyline>(polyline_points);
	    //this->SetDrawShape(mdrawshape);
    
    
    
    };
    
    
    
    
     ChBeamSectionCurvedEasyMultiWings(double kappa,  ///< curvature
                                         double order,  ///< order of the curve  
                                         const std::vector<rectangle_wing>& mwings,                                      
                                         double E,        ///< Young modulus
                                         double G,        ///< shear modulus
                                         double density   ///< volumetric density (ex. in SI units: [kg/m^3])
    ){
    	    
    	    this->SetNwings(mwings.size());
    	    this->SetProps(mwings);
    	    auto melasticity = chrono_types::make_shared<ChElasticityCosseratAdvancedGenericFPM>();
	    //melasticity->SetYoungModulus(E);
	    //melasticity->SetShearModulus(G);
	    //melasticity->SetAsRectangularSection(width_y, width_z);
	    //
	    // Compute sectional properties
	    //	   
	    //std::vector<double> sec_props=EasyMultiWingsSection(mwings);
	    std::vector<double> sec_props = this->ComputeSectionCharacteristic(kappa, order);
		this->SetArea(sec_props[0]);
	    double A=sec_props[0]; 
	    double Sn=sec_props[1]; 
	    double Sb=sec_props[2]; 
	    double Inn=sec_props[3]; 
	    double Inb=sec_props[4]; 
	    double Ibb=sec_props[5];
	    double J= Inn + Ibb;
	    //std::cout<<"A: "<<A*density<<"  Sn: "<<Sn*density<<"  Sb: "<<Sb*density<<"  Inn: "<<Inn*density<<"  Ibb: "<<Ibb*density<<"  Inb: "<<Inb*density<<std::endl;
	    //
	    // Set sectional stiffness
	    //
	    ChMatrixNM<double, 6, 6> Dmat;
	    Dmat.setZero();	
	    //
	    Dmat(0,0)=A*E; Dmat(0,4)=Sn*E; Dmat(0,5)=-Sb*E; 
	    Dmat(1,1)=0.85*A*G; Dmat(1,3)=-G*Sn;
	    Dmat(2,2)=0.85*A*G; Dmat(2,3)= G*Sb;
	    Dmat(3,1)=-Sn*G; Dmat(3,2)= G*Sb; Dmat(3,3)=G*J;
	    Dmat(4,0)=Sn*E; Dmat(4,4)=Inn*E; Dmat(4,5)=-Inb*E;
	    Dmat(5,0)=-Sb*E; Dmat(5,4)=-Inb*E; Dmat(5,5)=Ibb*E;  
	    //
	    melasticity->SetStiffnessMatrix(Dmat);
	    this->SetElasticity(melasticity);
	    //
	    // Set Inertia properties
	    //
	    auto minertia = chrono_types::make_shared<ChInertiaCosseratAdvanced>(A*density, Sb/A, Sn/A, Inn*density, Ibb*density, Inb*density); // mu_density=A*rho,  c_y,  c_z,  Jyy_moment,  Jzz_moment,  Jyz_moment 	     
	    this->SetInertia(minertia);
	    
	    //ChMatrixNM<double, 6, 6> M;
	    //minertia->ComputeInertiaMatrix(M);
	    //std::cout<<"M:\n"<<M<<std::endl;
    
    };
    
    
};




/// @} wood_utils

}  // end namespace wood
}  // end namespace chrono

#endif

