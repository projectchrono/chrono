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

#include "chrono_cascade/ChCascadeDoc.h"
#include "chrono_cascade/ChCascadeMeshTools.h"
#include "chrono_cascade/ChCascadeShapeAsset.h"
#include "chrono_cascade/ChCascadeTriangulate.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/geometry/ChLinePath.h"
#include "chrono/geometry/ChLineSegment.h"
#include "chrono/geometry/ChLineArc.h"

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
	/// Parameters for mesh triangulation can be set via ChCascadeTriangulateTolerances.
    ChBodyEasyCascade(TopoDS_Shape& mshape,      ///< OpenCASCADE shape
                      double mdensity,           ///< density
					  const ChCascadeTriangulate& visualization, ///< pass ChCascadeTriangulateTolerances if need visualization, pass a ChCascadeTriangulateNone if no visualization needed.
                      bool collide = false,  ///< if true, add a collision shape that uses the triangulation of shape
                      std::shared_ptr<ChMaterialSurface> mat = nullptr  ///< surface contact material
    ) {
        this->Init(mshape, mdensity, visualization, collide, mat);
    }



	/// Creates a body plus adds a visualization shape and, optionally,
    /// a collision shape. Mass and inertia are set automatically depending
    /// on density. COG is automatically displaced, and REF position is initialized as shape location.
	/// Kept here for backward compatibility. Better use the constructor that takes ChCascadeTriangulate as parameter.
	ChBodyEasyCascade(TopoDS_Shape& mshape,      ///< OpenCASCADE shape
		double mdensity,           ///< density
		bool visual_asset = true,  ///< if true, uses a triangulated shape for visualization, with default tesselation tolerances
		bool collide = false,  ///< if true, add a collision shape that uses the triangulation of shape
		std::shared_ptr<ChMaterialSurface> mat = nullptr  ///< surface contact material
	) {
		if (visual_asset == true)
			this->Init(mshape, mdensity, ChCascadeTriangulateTolerances(), collide, mat);
		else
			this->Init(mshape, mdensity, ChCascadeTriangulateNone(), collide, mat);
	}
		

  private:
	 void Init(TopoDS_Shape& mshape,			 ///< OpenCASCADE shape
                      double mdensity,           ///< density
					  const ChCascadeTriangulate& visualization, ///< pass ChCascadeTriangulateTolerances if need visualization, pass a ChCascadeTriangulateNone if no visualization needed.
                      bool collide = false,  ///< if true, add a collision shape that uses the triangulation of shape
                      std::shared_ptr<ChMaterialSurface> mat = nullptr  ///< surface contact material
    ) {
        chrono::ChFrame<>* user_ref_to_abs = 0;  // as parameter?
        chrono::ChFrame<> frame_ref_to_abs;

        if (!user_ref_to_abs) {
            TopLoc_Location loc_shape_to_abs = mshape.Location();
            chrono::cascade::ChCascadeDoc::FromCascadeToChrono(loc_shape_to_abs, frame_ref_to_abs);
        } else {
            frame_ref_to_abs = *user_ref_to_abs;
        }

        // Reset shape location to local ref csys (identity).
        this->topods_shape = mshape;
        this->topods_shape.Location(TopLoc_Location());

        // compute mass properties and COG reference
        chrono::ChVector<> mcog;
        chrono::ChVector<> minertiaXX;
        chrono::ChVector<> minertiaXY;
        double mvol;
        double mmass;
        chrono::cascade::ChCascadeDoc::GetVolumeProperties(topods_shape, mdensity, mcog, minertiaXX, minertiaXY, mvol,
                                                           mmass);

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
		if (const ChCascadeTriangulateTolerances* tolerances = dynamic_cast<const ChCascadeTriangulateTolerances*>(&visualization))
		{
            auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
            ChCascadeMeshTools::fillTriangleMeshFromCascade(*trimesh, topods_shape, *tolerances);

            auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
            trimesh_shape->SetMesh(trimesh);
            this->AddAsset(trimesh_shape);

            // Add a collision shape if needed
            if (collide) {
                assert(mat);
                GetCollisionModel()->ClearModel();
                GetCollisionModel()->AddTriangleMesh(mat, trimesh, false, false);
                GetCollisionModel()->BuildModel();
                SetCollide(true);
            }
		}
    }

  public:
    // Store the Cascade shape here, with null transformation relative to the REF
    TopoDS_Shape topods_shape;

};




/// Easy-to-use class for quick creation of flat "2D" rigid bodies given a 2D 'wire' profile and
/// a thickness. These bodies are meant for 2D problems with optimized contact between 2D profiles.
/// Profiles must be defined in the REF local reference of the body and must be parallel to XY plane, 
/// then the thickness is extruded in the positive Y direction.
/// Compared to the base ChBodyAuxRef class, this class also does
/// automatically, at object creation, the following tasks that
/// you would normally do by hand if using ChBodyAuxRef:
/// - mass and moment of inertia is automatically set, according to the wire profile and 
///   thickness of the shape. The profile can contain holes.
/// - the COG (center of mass) of the body is automatically moved at the barycenter, 
///   the REF (reference) is automatically moved where the Cascade TopoDS_Face has the reference (if passing a face, otherwise in 0,0,0)
/// - a visualization shape is created and added, if visualization asset is desired
/// - a collision shape is created and added, if collision is desired, and such collision shape is
///   made with 2D arcs and 2D lines in the profile, optimized for the 2D-2D contact case. 
///   Note that the collision shape is a 2D profile placed at half thickness of the extruded profile, 
///   here is the slice where contacts will happen. 

class ChBodyEasyCascadeProfile : public ChBodyAuxRef {
public:
	/// Creates a body plus adds a visualization shape and, optionally,
	/// a collision shape optimized for the 2D vs 2D contact. Mass and inertia are set automatically depending
	/// on density. COG is automatically displaced, and REF position is initialized as 0,0,0 xyz.
	/// Parameters for mesh triangulation can be set via ChCascadeTriangulateTolerances.
	ChBodyEasyCascadeProfile(
		std::vector<std::shared_ptr<::chrono::geometry::ChLinePath>> mwires, ///< profile of face, in XY plane. Usually just one, but might be N if disconnected like leopard dots. Each must be closed.
		std::vector<std::shared_ptr<::chrono::geometry::ChLinePath>> mholes, ///< profiles of holes, in XY plane, if any. Each must be closed paths. Must not intersecate.
		double mthickness,				///< thickness in Z direction
		double mdensity,				///< density
		const ChCascadeTriangulate& mvisualization, ///< pass ChCascadeTriangulateTolerances if need visualization, pass a ChCascadeTriangulateNone if no visualization needed.
		bool mcollide = false,			///< if true, add a 2D collision shape that uses the outer profile of the face - only arcs and linear edges are considered. 
		std::shared_ptr<ChMaterialSurface> mat = nullptr  ///< surface contact material if colliding
	)
	{
		this->AddProfile(mwires, mholes, mthickness, mdensity, mvisualization, mcollide, mat);
	}

	/// If multiple profiles on different Z dephts are needed, after the ChBodyEasyCascadeProfile constructor is executed with the first profile,
	/// you can use this function to add further profiles.
	/// Note that the additional profiles should be at different Z depths, and not intersecting along Z distance, because no boolean 'join' operation is done and
	/// in case they overlap by some amount, the computation of inertia and mass would be overestimated (i.e. each extruded profile is considered separately).
	void AddProfile(
		std::vector<std::shared_ptr<::chrono::geometry::ChLinePath>> mwires, ///< profile of face, in XY plane. Usually just one, but might be N if disconnected like leopard dots. Each must be closed.
		std::vector<std::shared_ptr<::chrono::geometry::ChLinePath>> mholes, ///< profiles of holes, in XY plane, if any. Each must be closed paths. Must not intersecate.
		double mthickness,				///< thickness in Z direction
		double mdensity,				///< density
		const ChCascadeTriangulate& visualization, ///< pass ChCascadeTriangulateTolerances if need visualization, pass a ChCascadeTriangulateNone if no visualization needed.
		bool mcollide = false,			///< if true, add a 2D collision shape that uses the outer profile of the face - only arcs and linear edges are considered. 
		std::shared_ptr<ChMaterialSurface> mat = nullptr  ///< surface contact material if colliding
	) {
		ChCascadeExtrusionFace mface;
		mface.wires = mwires;
		mface.holes = mholes;
		mface.thickness = mthickness;
		mface.density = mdensity;
		mface.collide = mcollide;
		mface.material = mat;
		this->faces.push_back(mface);
		this->UpdateCollisionAndVisualizationShapes(visualization);
	}

	void ClearProfiles() {
		this->faces.clear();
		this->UpdateCollisionAndVisualizationShapes(ChCascadeTriangulateNone());
	}

	  /// This function 1) generates the visualizer asset shapes for the part and 2) adds the proper collision shapes 
	  /// as 2D profiles optimized for 2D-2D contact. 
	  /// This is already called automatically when construction the ChBodyEasyCascadeProfile and each time you 
	  /// call AddProfile(), but you might want to call this by hand if you ever change the profile coordinates, arc 
	  /// radii etc. after you created the shape and you want to rebuild visualization and collision shapes.
	  void UpdateCollisionAndVisualizationShapes(const ChCascadeTriangulate& visualization ///< pass ChCascadeTriangulateTolerances if need visualization, pass a ChCascadeTriangulateNone if no visualization needed.
	  ) {

		  TopoDS_Compound mcompound;
		  TopoDS_Builder builder;
		  builder.MakeCompound(mcompound);


		  for (auto& chface : this->faces) {
			  BRepBuilderAPI_MakeFace facebuilder(FromChronoPathToCascadeWire(chface.wires[0]));

			  bool first = true;
			  for (auto profile : chface.wires) {
				  if (first) {
					  first = false;
					  continue;  // first already added in BRepBuilderAPI_MakeFace constructor
				  }
				  auto cascade_wire = FromChronoPathToCascadeWire(profile);
				  facebuilder.Add(cascade_wire);
			  }
			  for (auto profilehole : chface.holes) {
				  auto cascade_wire_hole = FromChronoPathToCascadeWire(profilehole);
				  cascade_wire_hole.Reverse();
				  facebuilder.Add(cascade_wire_hole);
			  }

			  gp_Pnt starting_point(0., 0., 0.);
			  gp_Pnt end_point(0., 0., chface.thickness);
			  gp_Vec vec(starting_point, end_point);

			  auto prism = BRepPrimAPI_MakePrism(facebuilder.Face(), vec).Shape();

			  // Add a visualization asset if needed
			  if (const ChCascadeTriangulateTolerances * tolerances =
				  dynamic_cast<const ChCascadeTriangulateTolerances*>(&visualization)) {
				  auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
				  ChCascadeMeshTools::fillTriangleMeshFromCascade(*trimesh, prism, *tolerances);

				  auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
				  trimesh_shape->SetMesh(trimesh);
				  this->AddAsset(trimesh_shape);
			  }

			  builder.Add(mcompound, prism);
		  }

		  chrono::ChFrame<>* user_ref_to_abs = 0;  // as parameter?
		  chrono::ChFrame<> frame_ref_to_abs;

		  if (!user_ref_to_abs) {
			  TopLoc_Location loc_shape_to_abs = mcompound.Location();
			  chrono::cascade::ChCascadeDoc::FromCascadeToChrono(loc_shape_to_abs, frame_ref_to_abs);
		  }
		  else {
			  frame_ref_to_abs = *user_ref_to_abs;
		  }

		  // Reset shape location to local ref csys (identity).
		  this->topods_shape = mcompound;
		  this->topods_shape.Location(TopLoc_Location());

		  // compute mass properties and COG reference
		  chrono::ChVector<> mcog;
		  chrono::ChVector<> minertiaXX;
		  chrono::ChVector<> minertiaXY;
		  double mvol;
		  double mmass;
		  chrono::cascade::ChCascadeDoc::GetVolumeProperties(topods_shape, density, mcog, minertiaXX, minertiaXY, mvol,
			  mmass);

		  // Set mass and COG and REF references
		  this->SetDensity((float)density);
		  this->SetMass(mmass);
		  this->SetInertiaXX(minertiaXX);
		  this->SetInertiaXY(minertiaXY);
		  //this->SetFrame_REF_to_abs(frame_ref_to_abs);

		  chrono::ChFrame<> frame_cog_to_ref;
		  frame_cog_to_ref.SetPos(mcog);
		  frame_cog_to_ref.SetRot(chrono::QUNIT);
		  this->SetFrame_COG_to_REF(frame_cog_to_ref);



		  // Add a collision shape if needed

		  GetCollisionModel()->ClearModel();
		  bool somefacecollide = false;

		  for (auto& chface : this->faces) {
			  if (chface.collide) {
				  assert(chface.material);
				  for (auto mpath : chface.wires) {
					  ChVector<> pathposz; mpath->Evaluate(pathposz, 0.0); // for offset along Z
					  GetCollisionModel()->Add2Dpath(chface.material, mpath, ChVector<>(0,0,pathposz.z() + chface.thickness*0.5), ChMatrix33<>(1), chface.thickness*0.99);
				  }
				  for (auto mhole : chface.holes) {
					  ChVector<> pathposz; mhole->Evaluate(pathposz, 0.0); // for offset along Z
					  GetCollisionModel()->Add2Dpath(chface.material, mhole, ChVector<>(0,0,pathposz.z() + chface.thickness*0.5), ChMatrix33<>(1), chface.thickness*0.99);
				  }
				  somefacecollide = true;
			  }
		  }
		  GetCollisionModel()->BuildModel();
		  if (somefacecollide)
			  SetCollide(true);
		  else
			  SetCollide(false);
	  
      }

  private:

	  class ChCascadeExtrusionFace {
	  public:
		  std::vector<std::shared_ptr<::chrono::geometry::ChLinePath>> wires;
		  std::vector<std::shared_ptr<::chrono::geometry::ChLinePath>> holes;
		  double thickness;
		  double density;
		  bool collide;
		  std::shared_ptr<ChMaterialSurface> material;
	  };
	  std::vector<ChCascadeExtrusionFace> faces;

	  const TopoDS_Wire FromChronoPathToCascadeWire(std::shared_ptr<::chrono::geometry::ChLinePath> profile) {
		  BRepBuilderAPI_MakeWire mwirebuilder;
		  for (size_t i = 0; i < profile->GetSubLinesCount(); ++i) {
			  if (auto msegment = std::dynamic_pointer_cast<::chrono::geometry::ChLineSegment>(profile->GetSubLineN(i))) {
				  if (msegment->pA.z() != msegment->pB.z())
				    throw ChException("Error! ChBodyEasyCascadeProfile: sub segment of ChLinePath not parallel to XY plane!");

				  gp_Pnt aPntA(msegment->pA.x(), msegment->pA.y(), msegment->pA.z());
				  gp_Pnt aPntB(msegment->pB.x(), msegment->pB.y(), msegment->pB.z());
				  Handle(Geom_TrimmedCurve) aSegment1 = GC_MakeSegment(aPntA, aPntB);

				  TopoDS_Edge aEdge1 = BRepBuilderAPI_MakeEdge(aSegment1);

				  mwirebuilder.Add(aEdge1);

			  }
			  else if (auto marc = std::dynamic_pointer_cast<::chrono::geometry::ChLineArc>(profile->GetSubLineN(i))) {
				  if ((marc->origin.rot.e1() != 0) ||  (marc->origin.rot.e2() != 0))
				    throw ChException("Error! ChBodyEasyCascadeProfile: a sub arc of ChLinePath not parallel to XY plane!");

				  gp_Circ aCirc(gp_Ax2(gp_Pnt(marc->origin.pos.x(), marc->origin.pos.y(), marc->origin.pos.z()), gp_Dir(0., 0., 1.)), marc->radius);

				  // note the reversal of angle2 angle1 respect to chrono ChLineArc: in OCC proceeds as gpCirc Y axis in twist direction.
				  // In OCC sense=true means starts from 1st angle parameter, 
				  Handle(Geom_TrimmedCurve) aArcOfCircle;
				  if (!marc->counterclockwise) {
					  aArcOfCircle = GC_MakeArcOfCircle(aCirc, marc->angle2, marc->angle1, false); 
				  }
				  else {
					  aArcOfCircle = GC_MakeArcOfCircle(aCirc, marc->angle1, marc->angle2, true); 
				  }

				  TopoDS_Edge aEdge = BRepBuilderAPI_MakeEdge(aArcOfCircle);

				  mwirebuilder.Add(aEdge);

			  }
			  else {
				  throw ChException("Error! ChBodyEasyCascadeProfile: ChLinePath must contain only ChLineArc and/or ChLineSegment.");
			  }
		  }

		  if (!mwirebuilder.IsDone()) {
				  throw ChException("Error! ChBodyEasyCascadeProfile: profile is not closed.");
			}

		  return mwirebuilder.Wire();
	  }

	 

  public:
      // Store the Cascade shape here, with null transformation relative to the REF
      TopoDS_Shape topods_shape;

};


/// @} cascade_module

}  // end namespace cascade
}  // end namespace chrono

#endif
