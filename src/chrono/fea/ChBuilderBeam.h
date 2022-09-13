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

#ifndef CHBUILDERBEAM_H
#define CHBUILDERBEAM_H

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
#include "chrono/physics/ChMaterialSurfaceSMC.h"

#include "chrono/geometry/ChLineBspline.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_utils
/// @{

/// Utility class for creating complex beams using ChElementBeamEuler elements, for example subdivides a segment in
/// multiple finite elements.
class ChApi ChBuilderBeamEuler {
  protected:
    std::vector<std::shared_ptr<ChElementBeamEuler>> beam_elems;
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> beam_nodes;

  public:
    /// Add beam FEM elements to the mesh to create a segment beam from point A to point B, using ChElementBeamEuler
    /// type elements. Before running, each time resets lists of beam_elems and beam_nodes.
    void BuildBeam(std::shared_ptr<ChMesh> mesh,                 ///< mesh to store the resulting elements
                   std::shared_ptr<ChBeamSectionEuler> sect,     ///< section material for beam elements
                   const int N,                                  ///< number of elements in the segment
                   const ChVector<> A,                           ///< starting point
                   const ChVector<> B,                           ///< ending point
                   const ChVector<> Ydir                         ///< the 'up' Y direction of the beam
    );

    /// Add beam FEM elements to the mesh to create a segment beam from one existing node to another existing node,
    /// using ChElementBeamEuler type elements. Before running, each time resets lists of beam_elems and beam_nodes.
    void BuildBeam(std::shared_ptr<ChMesh> mesh,                 ///< mesh to store the resulting elements
                   std::shared_ptr<ChBeamSectionEuler> sect,     ///< section material for beam elements
                   const int N,                                  ///< number of elements in the segment
                   std::shared_ptr<ChNodeFEAxyzrot> nodeA,       ///< starting point
                   std::shared_ptr<ChNodeFEAxyzrot> nodeB,       ///< ending point
                   const ChVector<> Ydir                         ///< the 'up' Y direction of the beam
    );

    /// Add beam FEM elements to the mesh to create a segment beam from one existing node to a point B, using
    /// ChElementBeamEuler type elements. Before running, each time resets lists of beam_elems and beam_nodes.
    void BuildBeam(std::shared_ptr<ChMesh> mesh,                 ///< mesh to store the resulting elements
                   std::shared_ptr<ChBeamSectionEuler> sect,     ///< section material for beam elements
                   const int N,                                  ///< number of elements in the segment
                   std::shared_ptr<ChNodeFEAxyzrot> nodeA,       ///< starting point
                   const ChVector<> B,                           ///< ending point
                   const ChVector<> Ydir                         ///< the 'up' Y direction of the beam
    );

    /// Access the list of elements used by the last built beam.
    /// It can be useful for changing properties afterwards.
    /// This list is reset all times a BuildBeam function is called.
    std::vector<std::shared_ptr<ChElementBeamEuler>>& GetLastBeamElements() { return beam_elems; }

    /// Access the list of nodes used by the last built beam.
    /// It can be useful for adding constraints or changing properties afterwards.
    /// This list is reset all times a BuildBeam function is called.
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>>& GetLastBeamNodes() { return beam_nodes; }
};

/// Utility class for creating complex beams using ChElementCableANCF elements, for example subdivides a segment in
/// multiple finite elements.
class ChApi ChBuilderCableANCF {
  protected:
    std::vector<std::shared_ptr<ChElementCableANCF>> beam_elems;
    std::vector<std::shared_ptr<ChNodeFEAxyzD>> beam_nodes;

  public:
    /// Adds cable FEM elements to the mesh to create a segment beam from point A to point B, using ChElementCableANCF
    /// type elements. Before running, each time resets lists of beam_elems and beam_nodes.
    void BuildBeam(std::shared_ptr<ChMesh> mesh,              ///< mesh to store the resulting elements
                   std::shared_ptr<ChBeamSectionCable> sect,  ///< section material for cable elements
                   const int N,                               ///< number of elements in the segment
                   const ChVector<> A,                        ///< starting point
                   const ChVector<> B                         ///< ending point
    );

    /// Adds cable FEM elements to the mesh to create a segment beam from point A to point B, using ChElementCableANCF
    /// type elements. Before running, each time resets lists of beam_elems and beam_nodes. This version also returns
    /// connectivity information.
    void BuildBeam(std::shared_ptr<ChMesh> mesh,              ///< mesh to store the resulting elements
                   std::shared_ptr<ChBeamSectionCable> sect,  ///< section material for cable elements
                   const int N,                               ///< number of elements in the segment
                   const ChVector<> A,                        ///< starting point
                   const ChVector<> B,                        ///< ending point
                   std::vector<std::vector<int>>& nodes,      ///< node indices for all elements
                   std::vector<std::vector<int>>& node_nbrs   ///< neighbor node indices (connectivity)
    );

    /// Access the list of elements used by the last built beam.
    /// It can be useful for changing properties afterwards.
    /// This list is reset all times a BuildCable function is called.
    std::vector<std::shared_ptr<ChElementCableANCF>>& GetLastBeamElements() { return beam_elems; }

    /// Access the list of nodes used by the last built beam.
    /// It can be useful for adding constraints or changing properties afterwards.
    /// This list is reset all times a BuildCable function is called.
    std::vector<std::shared_ptr<ChNodeFEAxyzD>>& GetLastBeamNodes() { return beam_nodes; }
};

/// Utility class for creating complex beams using ChElementBeamANCF_3333 elements, for example subdivides a segment in
/// multiple finite elements.
class ChApi ChBuilderBeamANCF {
  protected:
    std::vector<std::shared_ptr<ChElementBeamANCF_3333>> beam_elems;
    std::vector<std::shared_ptr<ChNodeFEAxyzDD>> beam_nodes;

  public:
    /// Add beam FEM elements to the mesh to create a segment beam from point A to point B, using ChElementCableANCF
    /// type elements. Before running, each time resets lists of beam_elems and beam_nodes.
    void BuildBeam(std::shared_ptr<ChMesh> mesh,             ///< mesh to store the resulting elements
                   std::shared_ptr<ChMaterialBeamANCF> mat,  ///<  material for beam elements
                   const int N,                              ///< number of elements in the segment
                   const ChVector<> A,                       ///< starting point
                   const ChVector<> B,                       ///< ending point
                   const double h,                           ///< height
                   const double w,                           ///< width
                   const ChVector<> DIR,                     ///< initial nodal direction
                   const ChVector<> CUR,                     ///< initial nodal curvature
                   const bool grav = false,                  ///< set true to apply gravity force
                   const double damp = 0                     ///< damping
    );

    /// Access the list of elements used by the last built beam.
    /// It can be useful for changing properties afterwards.
    /// This list is reset all times a BuildBeam function is called.
    std::vector<std::shared_ptr<ChElementBeamANCF_3333>>& GetLastBeamElements() { return beam_elems; }

    /// Access the list of nodes used by the last built beam.
    /// It can be useful for adding constraints or changing properties afterwards.
    /// This list is reset all times a BuildBeam function is called.
    std::vector<std::shared_ptr<ChNodeFEAxyzDD>>& GetLastBeamNodes() { return beam_nodes; }
};

/// Utility class for creating complex beams using ChElementBeamIGA elements, for example subdivides a segment in
/// multiple finite elements.
class ChApi ChBuilderBeamIGA {
  protected:
    std::vector<std::shared_ptr<ChElementBeamIGA>> beam_elems;
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> beam_nodes;

  public:
    /// Add beam FEM elements to the mesh to create a segment beam from point A to point B, using ChElementBeamIGA type
    /// elements. Before running, each time resets lists of beam_elems and beam_nodes.
    void BuildBeam(std::shared_ptr<ChMesh> mesh,                 ///< mesh to store the resulting elements
                   std::shared_ptr<ChBeamSectionCosserat> sect,  ///< section property for beam elements
                   const int N,                                  ///< number of elements in the segment
                   const ChVector<> A,                           ///< starting point
                   const ChVector<> B,                           ///< ending point
                   const ChVector<> Ydir,                        ///< the 'up' Y direction of the beam
                   const int order = 3                           ///< the order of spline (default=3,cubic)
    );

    /// Add beam FEM elements to the mesh to create a spline beam using ChElementBeamIGA type elements, given a B-spline
    /// line in 3D space. Before running, each time resets lists of beam_elems and beam_nodes.
    void BuildBeam(std::shared_ptr<ChMesh> mesh,                 ///< mesh to store the resulting elements
                   std::shared_ptr<ChBeamSectionCosserat> sect,  ///< section material for beam elements
                   geometry::ChLineBspline& spline,              ///< the B-spline to be used as the centerline
                   const ChVector<> Ydirn                        ///< the 'up' Y direction of the beam
    );

    /// Access the list of elements used by the last built beam.
    /// It can be useful for changing properties afterwards.
    /// This list is reset all times a BuildBeam function is called.
    std::vector<std::shared_ptr<ChElementBeamIGA>>& GetLastBeamElements() { return beam_elems; }

    /// Access the list of nodes used by the last built beam.
    /// It can be useful for adding constraints or changing properties afterwards.
    /// This list is reset all times a BuildBeam function is called.
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>>& GetLastBeamNodes() { return beam_nodes; }
};

/// Utility class for creating complex beams using ChElementBeamTaperedTimoshenko elements, for example subdivides a segment in
/// multiple finite elements.
class ChApi ChBuilderBeamTaperedTimoshenko {
  protected:
    std::vector<std::shared_ptr<ChElementBeamTaperedTimoshenko>> beam_elems;
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> beam_nodes;

  public:
    /// Add beam FEM elements to the mesh to create a segment beam from point A to point B, using ChElementBeamTaperedTimoshenko
    /// type elements. Before running, each time resets lists of beam_elems and beam_nodes.
    void BuildBeam(std::shared_ptr<ChMesh> mesh,              ///< mesh to store the resulting elements
                   std::shared_ptr<ChBeamSectionTaperedTimoshenkoAdvancedGeneric> sect,  ///< section material for beam elements
                   const int N,                               ///< number of elements in the segment
                   const ChVector<> A,                        ///< starting point
                   const ChVector<> B,                        ///< ending point
                   const ChVector<> Ydir                      ///< the 'up' Y direction of the beam
    );

    /// Add beam FEM elements to the mesh to create a segment beam from one existing node to another existing node,
    /// using ChElementBeamTaperedTimoshenko type elements. Before running, each time resets lists of beam_elems and beam_nodes.
    void BuildBeam(std::shared_ptr<ChMesh> mesh,              ///< mesh to store the resulting elements
        std::shared_ptr<ChBeamSectionTaperedTimoshenkoAdvancedGeneric> sect,  ///< section material for beam elements
                   const int N,                               ///< number of elements in the segment
                   std::shared_ptr<ChNodeFEAxyzrot> nodeA,    ///< starting point
                   std::shared_ptr<ChNodeFEAxyzrot> nodeB,    ///< ending point
                   const ChVector<> Ydir                      ///< the 'up' Y direction of the beam
    );

    /// Add beam FEM elements to the mesh to create a segment beam from one existing node to a point B, using
    /// ChElementBeamTaperedTimoshenko type elements. Before running, each time resets lists of beam_elems and beam_nodes.
    void BuildBeam(std::shared_ptr<ChMesh> mesh,              ///< mesh to store the resulting elements
        std::shared_ptr<ChBeamSectionTaperedTimoshenkoAdvancedGeneric> sect,  ///< section material for beam elements
                   const int N,                               ///< number of elements in the segment
                   std::shared_ptr<ChNodeFEAxyzrot> nodeA,    ///< starting point
                   const ChVector<> B,                        ///< ending point
                   const ChVector<> Ydir                      ///< the 'up' Y direction of the beam
    );

    /// Access the list of elements used by the last built beam.
    /// It can be useful for changing properties afterwards.
    /// This list is reset all times a BuildBeam function is called.
    std::vector<std::shared_ptr<ChElementBeamTaperedTimoshenko>>& GetLastBeamElements() { return beam_elems; }

    /// Access the list of nodes used by the last built beam.
    /// It can be useful for adding constraints or changing properties afterwards.
    /// This list is reset all times a BuildBeam function is called.
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>>& GetLastBeamNodes() { return beam_nodes; }
};

/// Utility class for creating complex beams using ChElementBeamTaperedTimoshenkoFPM elements, for example subdivides a
/// segment in multiple finite elements.
class ChApi ChBuilderBeamTaperedTimoshenkoFPM {
  protected:
    std::vector<std::shared_ptr<ChElementBeamTaperedTimoshenkoFPM>> beam_elems;
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> beam_nodes;

  public:
    /// Add beam FEM elements to the mesh to create a segment beam from point A to point B, using
    /// ChElementBeamTaperedTimoshenkoFPM type elements. Before running, each time resets lists of beam_elems and
    /// beam_nodes.
    void BuildBeam(
        std::shared_ptr<ChMesh> mesh,                                         ///< mesh to store the resulting elements
        std::shared_ptr<ChBeamSectionTaperedTimoshenkoAdvancedGenericFPM> sect,  ///< section material for beam elements
        const int N,                                                          ///< number of elements in the segment
        const ChVector<> A,                                                   ///< starting point
        const ChVector<> B,                                                   ///< ending point
        const ChVector<> Ydir                                                 ///< the 'up' Y direction of the beam
    );
    
    /// Add beam FEM elements to the mesh to create a segment beam from one existing node to another existing node,
    /// using ChElementBeamTaperedTimoshenkoFPM type elements. Before running, each time resets lists of beam_elems and
    /// beam_nodes.
    void BuildBeam(
        std::shared_ptr<ChMesh> mesh,                                         ///< mesh to store the resulting elements
        std::shared_ptr<ChBeamSectionTaperedTimoshenkoAdvancedGenericFPM> sect,  ///< section material for beam elements
        const int N,                                                          ///< number of elements in the segment
        std::shared_ptr<ChNodeFEAxyzrot> nodeA,                               ///< starting point
        std::shared_ptr<ChNodeFEAxyzrot> nodeB,                               ///< ending point
        const ChVector<> Ydir                                                 ///< the 'up' Y direction of the beam
    );

    /// Add beam FEM elements to the mesh to create a segment beam from one existing node to a point B, using
    /// ChElementBeamTaperedTimoshenkoFPM type elements. Before running, each time resets lists of beam_elems and beam_nodes.
    void BuildBeam(
        std::shared_ptr<ChMesh> mesh,                                         ///< mesh to store the resulting elements
        std::shared_ptr<ChBeamSectionTaperedTimoshenkoAdvancedGenericFPM> sect,  ///< section material for beam elements
        const int N,                                                          ///< number of elements in the segment
        std::shared_ptr<ChNodeFEAxyzrot> nodeA,                               ///< starting point
        const ChVector<> B,                                                   ///< ending point
        const ChVector<> Ydir                                                 ///< the 'up' Y direction of the beam
    );

    /// Access the list of elements used by the last built beam.
    /// It can be useful for changing properties afterwards.
    /// This list is reset all times a BuildBeam function is called.
    std::vector<std::shared_ptr<ChElementBeamTaperedTimoshenkoFPM>>& GetLastBeamElements() { return beam_elems; }

    /// Access the list of nodes used by the last built beam.
    /// It can be useful for adding constraints or changing properties afterwards.
    /// This list is reset all times a BuildBeam function is called.
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>>& GetLastBeamNodes() { return beam_nodes; }
};


/// Class for an object that continuously extrude a beam (composed of ChElementBeamEuler elements) with prescribed
/// velocity.
class ChApi ChExtruderBeamEuler {
  protected:
    std::vector<std::shared_ptr<ChElementBeamEuler>> beam_elems;
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> beam_nodes;

    std::shared_ptr<ChBody> ground;
    std::shared_ptr<ChLinkMotorLinearSpeed> actuator;
    std::shared_ptr<ChLinkMateGeneric> guide;

    ChSystem* mysystem;
    std::shared_ptr<ChMesh> mesh;

    std::shared_ptr<ChBeamSectionEuler> beam_section;
    double h;
    ChCoordsys<> outlet;
    double mytime;
    double speed;

    std::shared_ptr<ChMaterialSurfaceSMC> contact_material;

    std::shared_ptr<ChContactSurfaceNodeCloud> contactcloud;
    double contact_radius;

  public:
    /// Initialize and add required constraints to system
    ChExtruderBeamEuler(ChSystem* msystem,                            ///< system to store the constraints
                        std::shared_ptr<ChMesh> mmesh,                ///< mesh to store the resulting elements
                        std::shared_ptr<ChBeamSectionEuler> sect,     ///< section material for beam elements
                        double mh,                                    ///< element length
                        const ChCoordsys<> moutlet,  ///< outlet pos & orientation (x is extrusion direction)
                        double mspeed                ///< speed
    );

    ~ChExtruderBeamEuler();

    /// Set the material for the beam, and enables collision detection for the beam nodes.
    /// By default, collision not enabled.
    void SetContact(
        std::shared_ptr<ChMaterialSurfaceSMC> mcontact_material,  ///< material to use for surface
        double mcontact_radius  ///< radius of colliding spheres at each node (usually = to avg.beam thickness)
    );

    /// Create beam elements, if needed, and update the constraint that imposes the extrusion speed.
    void Update();

    /// Access the list of created elements.
    std::vector<std::shared_ptr<ChElementBeamEuler>>& GetLastBeamElements() { return beam_elems; }

    /// Access the list of created nodes.
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>>& GetLastBeamNodes() { return beam_nodes; }
};

/// Class for an object that continuously extrude a beam (composed of ChElementBeamIGA elements) with prescribed
/// velocity.
class ChApi ChExtruderBeamIGA {
  protected:
    std::vector<std::shared_ptr<ChElementBeamIGA>> beam_elems;
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> beam_nodes;
    std::vector<double> beam_knots;
    int beam_order;

    std::shared_ptr<ChBody> ground;
    std::shared_ptr<ChLinkMotorLinearSpeed> actuator;
    std::shared_ptr<ChLinkMateGeneric> guide;

    ChSystem* mysystem;
    std::shared_ptr<ChMesh> mesh;

    std::shared_ptr<ChBeamSectionCosserat> beam_section;
    double h;
    ChCoordsys<> outlet;
    double mytime;
    double speed;

    std::shared_ptr<ChMaterialSurfaceSMC> contact_material;

    std::shared_ptr<ChContactSurfaceNodeCloud> contactcloud;
    double contact_radius;

  public:
    /// Initialize and add required constraints to system
    ChExtruderBeamIGA(ChSystem* msystem,                            ///< system to store the constraints
                      std::shared_ptr<ChMesh> mmesh,                ///< mesh to store the resulting elements
                      std::shared_ptr<ChBeamSectionCosserat> sect,  ///< section material for beam elements
                      double mh,                                    ///< element length
                      const ChCoordsys<> moutlet,  ///< outlet pos & orientation (x is extrusion direction)
                      double mspeed,               ///< speed
                      int morder                   ///< element order, default =3 (cubic)
    );

    ~ChExtruderBeamIGA();

    /// Set the material for the beam, and enables collision detection for the beam nodes.
    /// By default, collision not enabled.
    void SetContact(
        std::shared_ptr<ChMaterialSurfaceSMC> mcontact_material,  ///< material to use for surface
        double mcontact_radius  ///< radius of colliding spheres at each node (usually = to avg.beam thickness)
    );

    /// Create beam elements, as needed, and update the constraint that  imposes the extrusion speed.
    /// Return 'true' if any new nodes/elements were created (hence a system change) and 'false' otherwise.
    bool Update();

    /// Access the list of created elements.
    std::vector<std::shared_ptr<ChElementBeamIGA>>& GetLastBeamElements() { return beam_elems; }

    /// Access the list of created nodes.
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>>& GetLastBeamNodes() { return beam_nodes; }
};

/// @} fea_utils

}  // end namespace fea
}  // end namespace chrono

#endif
