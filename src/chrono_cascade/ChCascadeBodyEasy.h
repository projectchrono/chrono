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

#ifndef CH_CASCADE_BODYEASY_H
#define CH_CASCADE_BODYEASY_H

#include "chrono_cascade/ChApiCASCADE.h"
#include "chrono_cascade/ChCascadeDoc.h"
#include "chrono_cascade/ChCascadeMeshTools.h"
#include "chrono_cascade/ChVisualShapeCascade.h"
#include "chrono_cascade/ChCascadeTriangulate.h"

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/geometry/ChLinePath.h"
#include "chrono/geometry/ChLineSegment.h"
#include "chrono/geometry/ChLineArc.h"
#include "chrono/utils/ChCompositeInertia.h"

#include <TopExp_Explorer.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Builder.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <GC_MakeSegment.hxx>
#include <gp_Circ.hxx>

namespace chrono {
namespace cascade {

/// @addtogroup cascade_module
/// @{

/// Easy-to-use class for quick creation of rigid bodies with an OpenCASCADE shape.
/// Compared to the base ChBodyAuxRef class, this class also does automatically, at object creation, the following tasks
/// that you would normally do by hand if using ChBodyAuxRef:
/// - mass and moment of inertia is automatically set, according to the geometry in the
///   OpenCASCADE shape.
/// - the COG (center of mass) of the body is automatically moved where the Cascade shape
///   has the barycenter, the REF (reference) is automatically moved where the Cascade shape has the reference
/// - a visualization shape is created and added, if visualization asset is desired
/// - a collision shape is created and added, if collision is desired (note: best performance
///   when not using this triangle collision mesh and rather adding simplified coll.shapes by hand)
class ChApiCASCADE ChCascadeBodyEasy : public ChBodyAuxRef {
  public:
    /// Creates a body plus adds a visualization shape and, optionally, a collision shape.
    /// Mass and inertia are set automatically depending on density. The COG is automatically displaced, and the
    /// reference position is initialized as shape location. Parameters for mesh triangulation can be set via
    /// ChCascadeTriangulate.
    ChCascadeBodyEasy(TopoDS_Shape& shape,                               ///< OpenCASCADE shape
                      double density,                                    ///< density
                      std::shared_ptr<ChCascadeTriangulate> vis_params,  ///< tesselation parameters
                      bool collide = false,  ///< if true, add a collision shape that uses the triangulation of shape
                      std::shared_ptr<ChMaterialSurface> mat = nullptr  ///< surface contact material
    );

    /// Creates a body plus adds a visualization shape and, optionally, a collision shape.
    /// Mass and inertia are set automatically depending on density. The COG is automatically displaced, and the
    /// reference position is initialized as shape location. Kept here for backward compatibility.
    ChCascadeBodyEasy(TopoDS_Shape& shape,    ///< OpenCASCADE shape
                      double density,         ///< density
                      bool visualize = true,  ///< if true, uses a triangulated shape for visualization
                      bool collide = false,   ///< if true, add a collision shape that uses the triangulation of shape
                      std::shared_ptr<ChMaterialSurface> mat = nullptr  ///< surface contact material
    );

  private:
    void Init(TopoDS_Shape& shape,                               ///< OpenCASCADE shape
              double density,                                    ///< density
              std::shared_ptr<ChCascadeTriangulate> vis_params,  ///< tesselation parameters
              bool collide = false,  ///< if true, add a collision shape that uses the triangulation of shape
              std::shared_ptr<ChMaterialSurface> mat = nullptr  ///< surface contact material
    );

  public:
    // Store the Cascade shape here, with null transformation relative to the REF
    TopoDS_Shape topods_shape;
};

/// Easy-to-use class for quick creation of flat "2D" rigid bodies given a 2D 'wire' profile and a thickness.
/// These bodies are meant for 2D problems with optimized contact between 2D profiles. Profiles must be defined in the
/// REF local reference of the body and must be parallel to XY plane, then the thickness is extruded in the positive Y
/// direction. Compared to the base ChBodyAuxRef class, this class also does automatically, at object creation, the
/// following tasks that you would normally do by hand if using ChBodyAuxRef:
/// - mass and moment of inertia is automatically set, according to the wire profile and
///   thickness of the shape. The profile can contain holes.
/// - the COG (center of mass) of the body is automatically moved at the barycenter,
///   the REF (reference) is automatically moved where the Cascade TopoDS_Face has the reference (if passing a face,
///   otherwise in 0,0,0)
/// - a visualization shape is created and added, if visualization asset is desired
/// - a collision shape is created and added, if collision is desired, and such collision shape is
///   made with 2D arcs and 2D lines in the profile, optimized for the 2D-2D contact case.
///   Note that the collision shape is a 2D profile placed at half thickness of the extruded profile,
///   here is the slice where contacts will happen.
class ChApiCASCADE ChCascadeBodyEasyProfile : public ChBodyAuxRef {
  public:
    /// Creates a body plus adds a visualization shape and, optionally,
    /// a collision shape optimized for the 2D vs 2D contact. Mass and inertia are set automatically depending
    /// on density. COG is automatically displaced, and REF position is initialized as 0,0,0 xyz.
    /// Parameters for mesh triangulation can be set via ChCascadeTriangulate.
    ChCascadeBodyEasyProfile(
        std::vector<std::shared_ptr<::chrono::geometry::ChLinePath>> wires,  ///< profile of face, in XY plane
        std::vector<std::shared_ptr<::chrono::geometry::ChLinePath>> holes,  ///< profiles of holes, in XY plane
        double thickness,                                                    ///< thickness in Z direction
        double density,                                                      ///< density
        std::shared_ptr<ChCascadeTriangulate> vis_params,                    ///< tesselation parameters
        bool collide = false,  ///< if true, add a 2D collision shape that uses the outer profile of the face
        std::shared_ptr<ChMaterialSurface> mat = nullptr  ///< surface contact material if colliding
    );

    /// If multiple profiles on different Z dephts are needed, after the ChCascadeBodyEasyProfile constructor is
    /// executed with the first profile, you can use this function to add further profiles. Note that the additional
    /// profiles should be at different Z depths, and not intersecting along Z distance, because no boolean 'join'
    /// operation is done and in case they overlap by some amount, the computation of inertia and mass would be
    /// overestimated (i.e. each extruded profile is considered separately).
    void AddProfile(
        std::vector<std::shared_ptr<::chrono::geometry::ChLinePath>> wires,  ///< profile of face, in XY plane
        std::vector<std::shared_ptr<::chrono::geometry::ChLinePath>> holes,  ///< profiles of holes, in XY plane
        double mhickness,                                                    ///< thickness in Z direction
        double density,                                                      ///< density
        std::shared_ptr<ChCascadeTriangulate> vis_params,                    ///< tesselation parameters
        bool collide = false,  ///< if true, add a 2D collision shape that uses the outer profile of the face
        std::shared_ptr<ChMaterialSurface> mat = nullptr  ///< surface contact material if colliding
    );

    void ClearProfiles();

    /// This function 1) generates the visualizer asset shapes for the part and 2) adds the proper collision shapes
    /// as 2D profiles optimized for 2D-2D contact.
    /// This is already called automatically when construction the ChCascadeBodyEasyProfile and each time you
    /// call AddProfile(), but you might want to call this by hand if you ever change the profile coordinates, arc
    /// radii etc. after you created the shape and you want to rebuild visualization and collision shapes.
    void UpdateCollisionAndVisualizationShapes();

    TopoDS_Shape topods_shape;  ///< Cascade shapes, with null transform relative to reference frame

  private:
    class ChCascadeExtrusionFace {
      public:
        ChCascadeExtrusionFace() {}
        ChCascadeExtrusionFace(const ChCascadeExtrusionFace& other)  // needed because later stored in a std::vector
            : wires(other.wires),
              holes(other.holes),
              thickness(other.thickness),
              density(other.density),
              collide(other.collide),
              material(other.material) {
            visualization = other.visualization;
        }

        std::vector<std::shared_ptr<::chrono::geometry::ChLinePath>> wires;
        std::vector<std::shared_ptr<::chrono::geometry::ChLinePath>> holes;
        double thickness;
        double density;
        bool collide;
        std::shared_ptr<ChCascadeTriangulate> visualization;
        std::shared_ptr<ChMaterialSurface> material;
    };

    std::vector<ChCascadeExtrusionFace> faces;

    const TopoDS_Wire FromChronoPathToCascadeWire(std::shared_ptr<::chrono::geometry::ChLinePath> profile);
};

/// @} cascade_module

}  // end namespace cascade
}  // end namespace chrono

#endif
