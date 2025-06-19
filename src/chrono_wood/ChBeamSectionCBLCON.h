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

#ifndef ChBeamSection_CBLCON_H
#define ChBeamSection_CBLCON_H


#include <vector>
#include <memory>
#include <string>

#include "chrono/core/ChMatrix.h"
#include "chrono_wood/ChWoodApi.h"
#include "chrono_wood/ChWoodMaterialVECT.h"
#include "chrono/core/ChMatrix33.h"
#include "chrono/fea/ChBeamSection.h"

using namespace chrono::fea;

namespace chrono {
namespace wood {


/// 
/// 
///  
/// 

//class ChWoodApi ChBeamSectionCBLCON : public ChBeamSection {
class ChWoodApi ChBeamSectionCBLCON  : public ChBeamSection { 
  public:

    using StateVarVector = ChVectorN<double, 18>;

    ChBeamSectionCBLCON(  std::shared_ptr<ChWoodMaterialVECT> material,  // material 
                                    double area,    // Projected total area of the facet
                                    ChVector3d center,    // Center point of the facet area      
                                    ChMatrix33<double> facetFrame   /// local system of frame of facet
                                     );

    ChBeamSectionCBLCON();
        
    ~ChBeamSectionCBLCON();

    // STIFFNESS INTERFACE

    /// Gets the axial rigidity, usually A*E, but might be ad hoc
    //virtual double GetAxialRigidity() const = 0;
    // Setter & Getter for projected facet area
    double Get_area() const { return m_area; }
    void Set_area(double area) { m_area=area; }
    //
    double GetWidth() const { return mcon_width; }
    void SetWidth(double con_width) { mcon_width=con_width; }
    //
    double GetHeight() const { return mcon_height; }
    void SetHeight(double con_height) { mcon_height=con_height; }
	//
	double GetRandomField() const { return m_randomField; }
    void SetRandomField(double randomField) { m_randomField=randomField; }
	//
    // Setter & Getter for Material
    std::shared_ptr<ChWoodMaterialVECT> Get_material() const { return m_material; };
    void Set_material(std::shared_ptr<ChWoodMaterialVECT> material) { m_material=material; }
    // Setter & Getter for centroid of facet
    ChVector3d Get_center() const { return m_center; }
    void Set_center( ChVector3d center) { m_center=center; }
    //
    ChMatrix33<double> Get_facetFrame() const { return m_facetFrame; }
    void Set_facetFrame( ChMatrix33<double> facetFrame) { m_facetFrame=facetFrame; }
  
    StateVarVector  Get_StateVar() const { return m_state; };
    void Set_StateVar(StateVarVector  state) { m_state=state; }
    
    enum ConSectionType { transverse_generic, transverse_top, transverse_bot, longitudinal };
    
    ConSectionType GetSectionType() const { return mysectype; }
    void SetSectionType(ConSectionType sectype) { mysectype=sectype; }
    
    ChMatrix33<> CalcBoxInertia(double xdim, double ydim, double zdim) {
    ChMatrix33<> J;
    J.setZero();
    J(0, 0) = (1.0 / 12.0) * (ydim * ydim + zdim * zdim);
    J(1, 1) = (1.0 / 12.0) * (zdim * zdim + xdim * xdim);
    J(2, 2) = (1.0 / 12.0) * (xdim * xdim + ydim * ydim);    

    return J;
    }
	
	void ComputeProjectionMatrix();
    
    ChMatrixNM<double,3,9> GetProjectionMatrix() const { return mprojection_matrix; }
    void SetProjectionMatrix( ChMatrixNM<double,3,9> projection_matrix) { mprojection_matrix=projection_matrix; }
	
	void ComputeEigenStrain(std::shared_ptr<ChMatrixNM<double,1,9>> macro_strain);
	
	ChVectorDynamic<>  Get_nonMechanicStrain() const { return m_nonMechanicStrain; };
    void Set_nonMechanicStrain(ChVectorDynamic<>  nonMechanicStrain) { m_nonMechanicStrain=nonMechanicStrain; } 
    
    
  protected:
    std::shared_ptr<ChWoodMaterialVECT> m_material;
    ChVector3d m_center;
    ChMatrix33<double> m_facetFrame;
    StateVarVector m_state;
	ChVectorDynamic<>  m_nonMechanicStrain;
    //std::shared_ptr<ChInternalDataCSL> m_state;  
    double mcon_width=1.0;  
    double mcon_height=1.0; 
    double m_area=1.0; 
	double m_randomField=0.0;
    ConSectionType mysectype;
	
    // 
    double rdamping_beta;
    double rdamping_alpha;
    double JzzJyy_factor;
	
	ChMatrixNM<double,3,9> mprojection_matrix;
};








}  // end namespace wood
}  // end namespace chrono

#endif
