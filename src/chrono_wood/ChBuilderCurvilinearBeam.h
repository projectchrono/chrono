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
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CH_BUILDER_CURVILINEARBEAM_H
#define CH_BUILDER_CURVILINEARBEAM_H

#include "chrono_wood/ChWoodApi.h"
#include "chrono_wood/ChElementCurvilinearBeamIGA.h"
#include "chrono_wood/ChElementCurvilinearBeamBezier.h"
#include "chrono_wood/ChBeamSectionCurvedIGA.h"
#include "chrono_wood/ChLineBezier.h"

#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChElementBeamIGA.h"
#include "chrono/fea/ChElementCableANCF.h"
#include "chrono/fea/ChElementBeamANCF_3333.h"
#include "chrono/fea/ChElementBeamTaperedTimoshenko.h"
#include "chrono/fea/ChElementBeamTaperedTimoshenkoFPM.h"
#include "chrono/fea/ChContactSurfaceNodeCloud.h"

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"
#include "chrono/physics/ChContactMaterialSMC.h"

#include "chrono/geometry/ChLineBSpline.h"
#include "chrono/geometry/ChLineNurbs.h"
//#include "chrono/geometry/ChLineBezier.h"

using namespace chrono::fea;
using namespace chrono;

namespace chrono {
namespace wood {

/// @addtogroup wood_utils
/// @{


/// Utility class for creating complex beams using ChElementBeamIGA elements, for example subdivides a segment in
/// multiple finite elements.
class ChWoodApi ChBuilderBeamIGA {
  protected:
    std::vector<std::shared_ptr<ChElementBeamIGA>> beam_elems;
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> beam_nodes;

  public:
    /// Add beam FEM elements to the mesh to create a segment beam from point A to point B, using ChElementBeamIGA type
    /// elements. Before running, each time resets lists of beam_elems and beam_nodes.
    void BuildBeam(std::shared_ptr<ChMesh> mesh,                 ///< mesh to store the resulting elements
                   std::shared_ptr<ChBeamSectionCosserat> sect,  ///< section property for beam elements
                   const int N,                                  ///< number of elements in the segment
                   const ChVector3d A,                           ///< starting point
                   const ChVector3d B,                           ///< ending point
                   const ChVector3d Ydir,                        ///< the 'up' Y direction of the beam
                   const int order = 3                           ///< the order of spline (default=3,cubic)
    );
	
	

    /// Add beam FEM elements to the mesh to create a spline beam using ChElementBeamIGA type elements, given a B-spline
    /// line in 3D space. Before running, each time resets lists of beam_elems and beam_nodes.
    void BuildBeam(std::shared_ptr<ChMesh> mesh,                 ///< mesh to store the resulting elements
                   std::shared_ptr<ChBeamSectionCosserat> sect,  ///< section material for beam elements
                   ChLineBSpline& spline,              ///< the B-spline to be used as the centerline
                   const ChVector3d Ydirn                        ///< the 'up' Y direction of the beam
    );
	
	/// Add beam FEM elements to the mesh to create a Non-Uniform Rational B-spline beam using ChElementBeamIGA type elements, given a B-spline
    /// line in 3D space. Before running, each time resets lists of beam_elems and beam_nodes.
    void BuildBeam(std::shared_ptr<ChMesh> mesh,                 ///< mesh to store the resulting elements
                   std::shared_ptr<ChBeamSectionCosserat> sect,  ///< section material for beam elements
                   ChLineNurbs& Nurbs,              ///< the Non-Uniform Rational B-spline curve to be used as the centerline
                   const ChVector3d Ydirn                        ///< the 'up' Y direction of the beam
    );
	
	
	/// Add beam FEM elements to the mesh to create a Bezier beam using ChElementBeamIGA type elements, given a B-spline
    /// line in 3D space. Before running, each time resets lists of beam_elems and beam_nodes.
    void BuildBeam(std::shared_ptr<ChMesh> mesh,                 ///< mesh to store the resulting elements
                   std::shared_ptr<ChBeamSectionCosserat> sect,  ///< section material for beam elements
                   ChLineBezier& Bezier,              ///< the Bezier curve to be used as the centerline
                   const ChVector3d Ydirn                        ///< the 'up' Y direction of the beam
    );

    /// Access the list of elements used by the last built beam.
    /// It can be useful for changing properties afterwards.
    /// This list is reset all times a BuildBeam function is called.
    std::vector<std::shared_ptr<ChElementBeamIGA>>& GetLastBeamElements() { return beam_elems; }

    /// Access the list of nodes used by the last built beam.
    /// It can be useful for adding constraints or changing properties afterwards.
    /// This list is reset all times a BuildBeam function is called.
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>>& GetLastBeamNodes() { return beam_nodes; }
	
	enum spline_type { Bspline, Nurbs, Bezier };
};



/// Utility class for creating complex beams using ChElementCurvilinearBeamIGA elements, for example subdivides a segment in
/// multiple finite elements.
class ChWoodApi ChBuilderCurvilinearBeamIGA {
  protected:
    std::vector<std::shared_ptr<ChElementCurvilinearBeamIGA>> beam_elems;
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> beam_nodes;

  public:
	/// Type of the curve
	enum spline_type { Bspline, Nurbs, Bezier };
    /// Add beam FEM elements to the mesh to create a segment beam from point A to point B, using ChElementBeamIGA type
    /// elements. Before running, each time resets lists of beam_elems and beam_nodes.
    void BuildBeam(std::shared_ptr<ChMesh> mesh,                 ///< mesh to store the resulting elements
                   std::shared_ptr<CurvedBeamSection> sect,  ///< section property for beam elements
                   const int N,                                  ///< number of elements in the segment
                   const ChVector3d A,                           ///< starting point
                   const ChVector3d B,                           ///< ending point
                   const ChVector3d Ydir,                        ///< the 'up' Y direction of the beam
                   const int order = 3,                           ///< the order of spline (default=3,cubic)
		   spline_type curvetype=Bspline
    );
    ///
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Curve based build functions
    ///
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    ///
    /// Add beam FEM elements to the mesh to create a spline beam using ChElementBeamIGA type elements, given a B-spline
    /// line in 3D space. Before running, each time resets lists of beam_elems and beam_nodes.
    void BuildBeam(std::shared_ptr<ChMesh> mesh,                 ///< mesh to store the resulting elements
                   std::shared_ptr<CurvedBeamSection> sect,  ///< section material for beam elements
                   ChLineBSpline& spline,              ///< the B-spline to be used as the centerline
                   const ChVector3d Ydirn                        ///< the 'up' Y direction of the beam
    );
	
	/// Add beam FEM elements to the mesh to create a Non-Uniform Rational B-spline beam using ChElementBeamIGA type elements, given a B-spline
    /// line in 3D space. Before running, each time resets lists of beam_elems and beam_nodes.
    void BuildBeam(std::shared_ptr<ChMesh> mesh,                 ///< mesh to store the resulting elements
                   std::shared_ptr<CurvedBeamSection> sect,  ///< section material for beam elements
                   ChLineNurbs& Nspline,              ///< the Non-Uniform Rational B-spline curve to be used as the centerline
                   const ChVector3d Ydirn                        ///< the 'up' Y direction of the beam
    );
	
	
	/// Add beam FEM elements to the mesh to create a Bezier beam using ChElementBeamIGA type elements, given a B-spline
    /// line in 3D space. Before running, each time resets lists of beam_elems and beam_nodes.
    void BuildBeam(std::shared_ptr<ChMesh> mesh,                 ///< mesh to store the resulting elements
                   std::shared_ptr<CurvedBeamSection> sect,  ///< section material for beam elements
                   ChLineBezier& Bezier,              ///< the Bezier curve to be used as the centerline
                   const ChVector3d Ydirn                        ///< the 'up' Y direction of the beam
    );
    
    ///
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Element based build functions
    ///
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    ///
    void BuildBeam(std::shared_ptr<ChMesh> mesh,                 // mesh to store the resulting elements
                   std::shared_ptr<CurvedBeamSection> sect,  // section material for beam elements
                   const int p, 				   // order of spline curve
                   std::vector<std::shared_ptr<ChNodeFEAxyzrot>> mynodes,  // Control points list belongs to this patch
                   std::vector<double> knots,  // the knot vector of a patch of B-spline or Nurbs curve
                   std::vector<double> weights,  // the weight vector of a patch of a Nurbs curve
                   const ChVector3d Ydir             // the 'up' Y direction of the beam
    );
    
   /* void BuildBeam(std::shared_ptr<ChMesh> mesh,                 // mesh to store the resulting elements
                   std::shared_ptr<CurvedBeamSection> sect,  // section material for beam elements
                   const int p, 				   // order of spline curve
                   std::vector<std::shared_ptr<ChNodeFEAxyzrot>> mynodes,  // Control points list belongs to this patch
                   std::vector<double> knots,  // the knot vector of a patch of B-spline or Nurbs curve                  
                   const ChVector3d Ydir             // the 'up' Y direction of the beam
    ){};*/

    /// Access the list of elements used by the last built beam.
    /// It can be useful for changing properties afterwards.
    /// This list is reset all times a BuildBeam function is called.
    std::vector<std::shared_ptr<ChElementCurvilinearBeamIGA>>& GetLastBeamElements() { return beam_elems; }

    /// Access the list of nodes used by the last built beam.
    /// It can be useful for adding constraints or changing properties afterwards.
    /// This list is reset all times a BuildBeam function is called.
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>>& GetLastBeamNodes() { return beam_nodes; }
    
    /// 
    /// Read nodes and element data from Freecad preprocessor.
    ///     
    void read_CBL_info(std::shared_ptr<ChMesh> my_mesh,   std::shared_ptr<CurvedBeamSection> section, std::string& CBL_data_path, 
				std::string& CBL_GeoName);
    
    void read_CBL_info_ElBased(std::shared_ptr<ChMesh> my_mesh,   std::shared_ptr<CurvedBeamSection> msection, std::string& CBL_data_path, 
				std::string& CBL_GeoName);
	
	
};



class ChWoodApi ChBuilderCurvilinearBeamBezier {
  protected:
    std::vector<std::shared_ptr<ChElementCurvilinearBeamBezier>> beam_elems;
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> beam_nodes;

  public:
	/// Type of the curve
    enum spline_type { Bspline, Nurbs, Bezier };
    /// Add beam FEM elements to the mesh to create a segment beam from point A to point B, using ChElementBeamIGA type
    /// elements. Before running, each time resets lists of beam_elems and beam_nodes.
    /*void BuildBeam(std::shared_ptr<ChMesh> mesh,                 ///< mesh to store the resulting elements
                   std::shared_ptr<CurvedBeamSection> sect,  ///< section property for beam elements
                   const int N,                                  ///< number of elements in the segment
                   const ChVector3d A,                           ///< starting point
                   const ChVector3d B,                           ///< ending point
                   const ChVector3d Ydir,                        ///< the 'up' Y direction of the beam
                   const int order = 3,                           ///< the order of spline (default=3,cubic)
		   spline_type curvetype=Bezier
    );*/
    ///
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Curve based build functions
    ///
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///  
    /// Add beam FEM elements to the mesh to create a Bezier beam using ChElementBeamIGA type elements, given a B-spline
    /// line in 3D space. Before running, each time resets lists of beam_elems and beam_nodes.
    void BuildBeam(std::shared_ptr<ChMesh> mesh,                 ///< mesh to store the resulting elements
                   std::shared_ptr<CurvedBeamSection> sect,  ///< section material for beam elements
                   ChLineBezier& Bezier,              ///< the Bezier curve to be used as the centerline
                   const ChVector3d Ydirn                        ///< the 'up' Y direction of the beam
    );
    
    ///
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Element based build functions
    ///
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    ///
    ///
    /// Position of control points are known but ChNodeFEAxyzrot type nodes will be created in this function
    ///
    void BuildBeam(std::shared_ptr<ChMesh> mesh,                 // mesh to store the resulting elements
                   std::shared_ptr<CurvedBeamSection> sect,  // section material for beam elements
                   const int p, 				   // order of spline curve
                   std::vector<ChVector3d> my_points,  // Control points list belongs to this patch
                   ChVectorDynamic<> knots,  // the knot vector of a patch of B-spline or Nurbs curve
                   const ChVector3d Ydir             // the 'up' Y direction of the beam
    );
    
    ///
    /// ChNodeFEAxyzrot type nodes are already defined
    ///
    
    void BuildBeam(std::shared_ptr<ChMesh> mesh,                 // mesh to store the resulting elements
                   std::shared_ptr<CurvedBeamSection> sect,  // section material for beam elements
                   const int p, 				   // order of spline curve
                   std::vector<std::shared_ptr<ChNodeFEAxyzrot>> mynodes,  // Control points list belongs to this patch
                   std::vector<double> knots,  // the knot vector of a patch of B-spline or Nurbs curve
                   const ChVector3d Ydir             // the 'up' Y direction of the beam
    );
    /// Access the list of elements used by the last built beam.
    /// It can be useful for changing properties afterwards.
    /// This list is reset all times a BuildBeam function is called.
    std::vector<std::shared_ptr<ChElementCurvilinearBeamBezier>>& GetLastBeamElements() { return beam_elems; }

    /// Access the list of nodes used by the last built beam.
    /// It can be useful for adding constraints or changing properties afterwards.
    /// This list is reset all times a BuildBeam function is called.
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>>& GetLastBeamNodes() { return beam_nodes; }
    
    /// 
    /// Read nodes and element data from Freecad preprocessor.
    ///     
    void read_CBL_info(std::shared_ptr<ChMesh> my_mesh,  std::string& CBL_data_path, 
				std::string& CBL_GeoName, double YoungModulus,	double ShearModulus, double density);
    
    void read_CBL_info_ElBased(std::shared_ptr<ChMesh> my_mesh,   std::string& CBL_data_path, 
				std::string& CBL_GeoName, double YoungModulus,	double ShearModulus, double density);
	
	
};


/// @} wood_utils

}  // end namespace wood
}  // end namespace chrono

#endif
