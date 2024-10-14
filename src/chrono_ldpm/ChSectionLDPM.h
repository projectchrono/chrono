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

#ifndef CH_SECTION_LDPM_H
#define CH_SECTION_LDPM_H

#include "chrono/core/ChMatrix33.h"
#include <vector>
#include <memory>
#include <string>
#include <cmath>

#include "chrono_ldpm/ChMaterialVECT.h"
#include "chrono/fea/ChBeamSection.h"


#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChMatrix33.h"

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChMatrix33.h"




namespace chrono {
namespace ldpm {


/// 
/// 
///  
/// 

//class ChApi ChSectionLDPM : public ChBeamSection {
class ChApi ChSectionLDPM  : public chrono::fea::ChBeamSection { 
  public:
    ChSectionLDPM(  std::shared_ptr<ChMaterialVECT> material,  // material 
                                    double area,    // Projected total area of the facet
                                    ChVector3d center,    // Center point of the facet area      
                                    ChMatrix33<double> facetFrame   /// local system of frame of facet
                                     );

    ChSectionLDPM();
        
    ~ChSectionLDPM();

    // STIFFNESS INTERFACE

    /// Gets the axial rigidity, usually A*E, but might be ad hoc
    //virtual double GetAxialRigidity() const = 0;
    // Setter & Getter for projected facet area
    double Get_area() const { return m_area; }
    void Set_area(double area) { m_area=area; }
    // Setter & Getter for Material
    std::shared_ptr<ChMaterialVECT> Get_material() const { return m_material; };
    void Set_material(std::shared_ptr<ChMaterialVECT> material) { m_material=material; }
    // Setter & Getter for centroid of facet
    ChVector3d Get_center() const { return m_center; }
    void Set_center( ChVector3d center) { m_center=center; }
    //
    ChMatrix33<double> Get_facetFrame() const { return m_facetFrame; }
    void Set_facetFrame( ChMatrix33<double> facetFrame) { m_facetFrame=facetFrame; }
    // Setter & Getter for facet state variables
    ChVectorDynamic<>  Get_StateVar() const { return m_state; };
    void Set_StateVar(ChVectorDynamic<>  state) { m_state=state; }  
    // Setter & Getter for rotation of the lattice
    ChQuaternion<> Get_ref_rot() const { return mq_lattice_ref_rot; }
    void Set_ref_rot(ChQuaternion<> q_element_ref_rot) { mq_lattice_ref_rot=q_element_ref_rot; }
    
    ChQuaternion<> Get_abs_rot() const { return mq_lattice_abs_rot; }
    void Set_abs_rot(ChQuaternion<> q_element_abs_rot) { mq_lattice_abs_rot=q_element_abs_rot; }
    
    void ComputeProjectionMatrix();
    
    ChMatrixNM<double,3,9> GetProjectionMatrix() const { return mprojection_matrix; }
    void SetProjectionMatrix( ChMatrixNM<double,3,9> projection_matrix) { mprojection_matrix=projection_matrix; }
  protected:
    std::shared_ptr<ChMaterialVECT> m_material;
    ChVector3d m_center;
    ChMatrix33<double> m_facetFrame;
    ChVectorDynamic<>  m_state;
    
    ChQuaternion<> mq_lattice_abs_rot;
    ChQuaternion<> mq_lattice_ref_rot; 
    
    //std::shared_ptr<ChInternalDataCSL> m_state;    
    double m_area=1.0; 
    //
    ChMatrixNM<double,3,9> mprojection_matrix;
    // 
    //double rdamping_beta;
    //double rdamping_alpha;
    //double JzzJyy_factor;
};








}  // end namespace ldpm
}  // end namespace chrono

#endif
