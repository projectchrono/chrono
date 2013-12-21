//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//
// File author: A. Tasora

#ifndef CHVISUALIZATIONFEMMESH_H
#define CHVISUALIZATIONFEMMESH_H


#include "assets/ChAssetLevel.h"
#include "assets/ChColor.h"
#include "geometry/ChCTriangleMeshConnected.h"
#include "unit_FEM/ChMesh.h"
#include "unit_FEM/ChNodeFEMxyz.h"


namespace chrono 
{
namespace fem
{

/// Class for a FEM mesh visualization.
/// It converts tetahedrons, etc. into a coloured triangle mesh asset
/// of class ChTriangleMeshShape that is contained in its sublevel, 
/// so that it can be rendered or postprocessed.

class ChApiFem ChVisualizationFEMmesh: public ChAssetLevel 
{
	public:
	 enum eChFemDataType {
				E_PLOT_NONE,
				E_PLOT_NODE_DISP_NORM,
				E_PLOT_NODE_DISP_X,
				E_PLOT_NODE_DISP_Y,
				E_PLOT_NODE_DISP_Z,
				E_PLOT_NODE_SPEED_NORM,
				E_PLOT_NODE_SPEED_X,
				E_PLOT_NODE_SPEED_Y,
				E_PLOT_NODE_SPEED_Z,
				E_PLOT_NODE_ACCEL_NORM,
				E_PLOT_NODE_ACCEL_X,
				E_PLOT_NODE_ACCEL_Y,
				E_PLOT_NODE_ACCEL_Z,
				E_PLOT_ELEM_STRAIN_VONMISES,
				E_PLOT_ELEM_STRESS_VONMISES,
		};

	protected:
		//
		// DATA
		//

		ChMesh* FEMmesh;

		eChFemDataType fem_data_type;

		double colorscale_min;
		double colorscale_max;

		bool shrink_elements;
		double shrink_factor;

		bool wireframe; 

		bool smooth_faces;

		bool undeformed_reference;

		ChColor meshcolor;

		std::vector<int> normal_accumulators;

	public:

		


		//
		// CONSTRUCTORS
		//

		ChVisualizationFEMmesh(ChMesh& mymesh);

		virtual ~ChVisualizationFEMmesh() 
			{
			}


		//
		// FUNCTIONS
		//

			// Access the referenced FEM mesh
		virtual ChMesh& GetMesh() { return *FEMmesh; }


			// Returns the current data type to be plotted (speeds, forces, etc.)
		eChFemDataType GetFEMdataType() {return fem_data_type;}
			
			// Set the current data type to be plotted (speeds, forces, etc.)
		void SetFEMdataType(eChFemDataType mdata) {fem_data_type = mdata;}
		
			// Set upper and lower values of the plotted variable for the colorscale plots.
		void SetColorscaleMinMax(double mmin, double mmax) {colorscale_min = mmin; colorscale_max = mmax;}
			
			// Set shrinkage of elements during drawing
		void SetShrinkElements(bool mshrink, double mfact) {this->shrink_elements = mshrink; shrink_factor = mfact;}

			// Set as wireframe visualization
		void SetWireframe(bool mwireframe) {this->wireframe = mwireframe;}

			// Set color for E_PLOT_NONE mode or for wireframe lines
		void SetMeshColor(ChColor mcolor) {this->meshcolor = mcolor;}

			// Activate Gourad or Phong smoothing for faces of non-straight elements 
			// (with a small performance overhead) -NOTE: experimental
		void SetSmoothFaces(bool msmooth) {this->smooth_faces = msmooth;}

			// If this flag is turned on, the mesh is drawn as it is 
			// undeformed (the reference position).
		void SetDrawInUndeformedReference(bool mdu) {this->undeformed_reference = mdu;}

			// Updates the triangle visualization mesh so that it matches with the
			// FEM mesh (ex. tetrahedrons are converted in 4 surfaces, etc.
		virtual void Update ();

private:
		double	ComputeScalarOutput( ChSharedPtr<ChNodeFEMxyz> mnode, ChSharedPtr<ChElementBase> melement);
		ChVector<float> ComputeFalseColor(double in);

};



}// END_OF_NAMESPACE____
}// END_OF_NAMESPACE____


#endif
