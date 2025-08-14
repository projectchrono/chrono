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

#ifndef CHBUILDER_CBLCON_H
#define CHBUILDER_CBLCON_H

#include "chrono_wood/ChWoodApi.h"
#include "chrono_wood/ChElementCBLCON.h"
#include "chrono/fea/ChElementTetraCorot_4.h"


#include "chrono/fea/ChMesh.h"
//#include "chrono/fea/ChElementBeamEuler.h"
//#include "chrono/fea/ChElementBeamIGA.h"
//#include "chrono/fea/ChElementCableANCF.h"
//#include "chrono/fea/ChElementBeamANCF_3333.h"
//#include "chrono/fea/ChElementBeamTaperedTimoshenko.h"
//#include "chrono/fea/ChElementBeamTaperedTimoshenkoFPM.h"
#include "chrono/fea/ChContactSurfaceNodeCloud.h"

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"
#include "chrono/physics/ChContactMaterialSMC.h"

#include "chrono/geometry/ChLineBSpline.h"

using namespace chrono::fea;
using namespace chrono;


namespace chrono {
namespace wood {

/// @addtogroup fea_utils
/// @{

/// Utility class for creating complex beams using ChElementBeamEuler elements, for example subdivides a segment in
/// multiple finite elements.
class ChWoodApi ChBuilderCBLCON {
  protected:
    std::vector<std::shared_ptr<ChElementCBLCON>> beam_elems;
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> beam_nodes;

  public:
    /// Add beam FEM elements to the mesh to create a segment beam from point A to point B, using ChElementBeamEuler
    /// type elements. Before running, each time resets lists of beam_elems and beam_nodes.
    void BuildBeam(std::shared_ptr<ChMesh> mesh,                 ///< mesh to store the resulting elements
                   std::shared_ptr<ChBeamSectionCBLCON> sect,     ///< section material for beam elements
                   const int N,                                  ///< number of elements in the segment
                   const ChVector3d A,                           ///< starting point
                   const ChVector3d B,                           ///< ending point
                   const ChVector3d Ydir                         ///< the 'up' Y direction of the beam
    );

    /// Add beam FEM elements to the mesh to create a segment beam from one existing node to another existing node,
    /// using ChElementBeamEuler type elements. Before running, each time resets lists of beam_elems and beam_nodes.
    void BuildBeam(std::shared_ptr<ChMesh> mesh,                 ///< mesh to store the resulting elements
                   std::shared_ptr<ChBeamSectionCBLCON> sect,     ///< section material for beam elements
                   const int N,                                  ///< number of elements in the segment
                   std::shared_ptr<ChNodeFEAxyzrot> nodeA,       ///< starting point
                   std::shared_ptr<ChNodeFEAxyzrot> nodeB,       ///< ending point
                   const ChVector3d Ydir                         ///< the 'up' Y direction of the beam
    );

    /// Add beam FEM elements to the mesh to create a segment beam from one existing node to a point B, using
    /// ChElementBeamEuler type elements. Before running, each time resets lists of beam_elems and beam_nodes.
    void BuildBeam(std::shared_ptr<ChMesh> mesh,                 ///< mesh to store the resulting elements
                   std::shared_ptr<ChBeamSectionCBLCON> sect,     ///< section material for beam elements
                   const int N,                                  ///< number of elements in the segment
                   std::shared_ptr<ChNodeFEAxyzrot> nodeA,       ///< starting point
                   const ChVector3d B,                           ///< ending point
                   const ChVector3d Ydir                         ///< the 'up' Y direction of the beam
    );
    
    void read_CBLCON_info(std::shared_ptr<ChMesh> my_mesh,  std::shared_ptr<ChWoodMaterialVECT> vect_mat, std::string& CBLCON_data_path, 
				std::string& CBLCON_GeoName);
	
    void read_CBLCON_info(std::shared_ptr<ChMesh> my_mesh,  std::vector<std::shared_ptr<ChWoodMaterialVECT>> vect_mat, std::string& CBLCON_data_path, 
				std::string& CBLCON_GeoName);

    /// Access the list of elements used by the last built beam.
    /// It can be useful for changing properties afterwards.
    /// This list is reset all times a BuildBeam function is called.
    std::vector<std::shared_ptr<ChElementCBLCON>>& GetLastBeamElements() { return beam_elems; }

    /// Access the list of nodes used by the last built beam.
    /// It can be useful for adding constraints or changing properties afterwards.
    /// This list is reset all times a BuildBeam function is called.
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>>& GetLastBeamNodes() { return beam_nodes; }
};


/// @} fea_utils

}  // end namespace wood
}  // end namespace chrono

#endif
