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

#ifndef CHBODYEASYCASCADE_H
#define CHBODYEASYCASCADE_H

#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono_cascade/ChCascadeMeshTools.h"
#include "chrono_cascade/ChCascadeShapeAsset.h"
#include "chrono_cascade/ChCascadeDoc.h"


namespace chrono {

namespace cascade {

/// @addtogroup cascade_module
/// @{

// This header includes some easy-to-use bodies, that can be used
// to create ChBody objects that already include a visualization shape
// without the need of adding it afterward with AddAsset().
// Also, a collision shape is created and added automatically too, if
// collision is needed.

/// Easy-to-use class for quick creation of rigid bodies with a
/// OpenCASCADE shape.
/// Compared to the base ChBodyAuxRef class, this class also does
/// automatically, at object creation, the following tasks that
/// you would normally do by hand if using ChBodyAuxRef:
/// - mass and moment of inertia is automatically set, according to the geometry in the
///   OpenCASCADE shape.
/// - the COG (center of mass) of the body is automatically moved where the Cascade shape 
///   has the barycenter, the REF (reference) is automatically moved where the Cascade shape has the reference
/// - a visualization shape is created and added, if visualization asset is desired
/// - a collision shape is created and added, if collision is desired (note: best performance 
///   when not using this triangle collision mesh and rather adding simplified coll.shapes by hand)


class ChBodyEasyCascade : public ChBodyAuxRef {
  public:
    /// Creates a body plus adds a visualization shape and, optionally,
    /// a collision shape. Mass and inertia are set automatically depending
    /// on density. COG is automatically displaced, and REF position is initialized as shape location.
    /// Sphere is assumed with center at body reference coordsystem.
    ChBodyEasyCascade(TopoDS_Shape& mshape,         ///< pass the OpenCASCADE shape
                        double mdensity,            ///< density  
                        bool collide = false,       ///< if true, add a collision shape that uses the triangulation of shape
                        bool visual_asset = true    ///< if true, uses a triangulated shape for visualization
                        ) {
        
        chrono::ChFrame<>* user_ref_to_abs = 0; // as parameter?
        chrono::ChFrame<> frame_ref_to_abs;

        if (!user_ref_to_abs) {
            TopLoc_Location loc_shape_to_abs = mshape.Location();
            chrono::cascade::ChCascadeDoc::FromCascadeToChrono(loc_shape_to_abs, frame_ref_to_abs);
        } else {
            frame_ref_to_abs = *user_ref_to_abs;
        }

        // Reset shape location to local ref csys (identity).
        TopoDS_Shape objshape = mshape;
        objshape.Location(TopLoc_Location());  

        // compute mass properties and COG reference
        chrono::ChVector<> mcog;
        chrono::ChVector<> minertiaXX;
        chrono::ChVector<> minertiaXY;
        double mvol;
        double mmass;
        chrono::cascade::ChCascadeDoc::GetVolumeProperties(objshape, mdensity, mcog, minertiaXX, minertiaXY, mvol, mmass);

        // Set mass and COG and REF references
        this->SetDensity((float)mdensity);
        this->SetMass(mmass);
        this->SetInertiaXX(minertiaXX);
        this->SetInertiaXY(minertiaXY);
        this->SetFrame_REF_to_abs(frame_ref_to_abs);

        chrono::ChFrame<> frame_cog_to_ref;
        frame_cog_to_ref.SetPos(mcog);
        frame_cog_to_ref.SetRot(chrono::QUNIT);
        this->SetFrame_COG_to_REF(frame_cog_to_ref);

        // Add a visualization asset if needed
        auto mtrimesh = std::make_shared<ChTriangleMeshShape>();
        if (visual_asset) {
            ChCascadeMeshTools::fillTriangleMeshFromCascade(mtrimesh->GetMesh(), objshape);
            this->AddAsset(mtrimesh);
        }

        // Add a collision shape if needed
        if (collide && visual_asset) {
            GetCollisionModel()->ClearModel();
            GetCollisionModel()->AddTriangleMesh(mtrimesh->GetMesh(),false,false);
            GetCollisionModel()->BuildModel();
            SetCollide(true);
        }
    }
};



/// @} cascade_module

}  // END_OF_NAMESPACE____
}  // END_OF_NAMESPACE____

#endif
