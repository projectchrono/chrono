//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHCASCADEDOC_H
#define CHCASCADEDOC_H

//////////////////////////////////////////////////
//
//   ChCascadeDoc.h
//
//   Wraps the OCAF document (the hierarchy of
//   shapes in the OpenCascade framework)
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "unit_CASCADE/ChApiCASCADE.h"
#include "core/ChStream.h"
#include "core/ChFrameMoving.h"


class TopoDS_Face;
class TopoDS_Shape;
class Handle_TDocStd_Document;
class TopLoc_Location;
class TDF_Label;

namespace chrono
{

/// The "cascade" namespace contains tools for interoperation with CAD 
/// files. The OpenCASCADE open-source library is used to this end:
/// it can load STEP files saved from most 3D CADs.

namespace cascade
{


/// Class that contains an OCAF document (a tree hierarchy of
/// shapes in the OpenCascade framework). Most often this is
/// populated by loading a STEP file from disk.

class ChApiCASCADE ChCascadeDoc
{
public:
	ChCascadeDoc();
	virtual ~ChCascadeDoc();

		/// Populate the document with all shapes that are contained in
		/// the STEP file, saved from some CAD. If load was ok, return true.
	bool Load_STEP(const char* filename);

		/// Show shape hierarchy, writing on mstream (mstream could be GetLog() 
		/// to print in default console log)
	void Dump(ChStreamOutAscii& mstream);

		/// Get the root shape. Note that there could be more than one root, 
		/// if so, use 'num' to select the one that you need.
	bool GetRootShape(TopoDS_Shape& mshape, const int num=1);

		/// Get a sub-shape with a given name, returned in 'mshape'.
		/// Since the document can contain assembles, subassemblies etc, the name
		/// can use a 'directory type' syntax, using the / slash such as in
		///   "assembly/subassebmly/subsubassembly/mypart"
		/// It is possible to use # and ? wildcards as in Unix.
		/// If there are multiple parts (or assemblies) with the same name, only the first
		/// instance is returned in 'mshape'; otherwise, one can use the # wildcard
		/// to get the #n-th object, for example "MyAssembly/bolt#3", "Car/Wheel#2/hub", etc.
		/// If the 'set_location_to_root' parameter is true (default), the location of the 
		/// shape is changed so that it represents its position respect to the root, that is
		/// the shape .Location() function will give the absolute position, otherwise if false
		/// it will give its position relative to the assembly where it is a sub-shape.
		/// If the 'get_multiple' = true, if there are multiple parts satisfying the search string,
		/// they are all returned in a single shape of compound type (with null location).
	bool GetNamedShape(TopoDS_Shape& mshape, char* name, bool set_location_to_root = true, bool get_multiple=false);

		/// Get the volume properties (center of mass, inertia moments, volume)
		/// of a given shape. 
	bool static GetVolumeProperties(const TopoDS_Shape& mshape,	///< pass the shape here
						const double density,				///< pass the density here 
						ChVector<>& center_position,		///< get the COG position center, respect to shape pos.
						ChVector<>& inertiaXX,				///< get the inertia diagonal terms
						ChVector<>& inertiaXY,				///< get the inertia extradiagonal terms
						double& volume,						///< get the volume
						double& mass						///< get the mass
						);						


		class callback_CascadeDoc{
		  public:
			virtual bool ForShape(TopoDS_Shape& mshape, TopLoc_Location& mloc, char* mname, int mlevel, TDF_Label& mlabel)=0;
		 };
		/// Execute a callback on all contained shapes, with user-defined callback inherited
		/// from callback_CascadeDoc. Btw. If the callback_CascadeDoc::ForShape callback returns false, subshapes are not processed.
	void ScanCascadeShapes(callback_CascadeDoc& mcallback);

		/// Convert OpenCascade coordinates into Chrono coordinates
	static void FromCascadeToChrono(const TopLoc_Location& from_coord, ChFrame<>& to_coord);
	
		/// Convert Chrono coordinates into OpenCascade coordinates
	static void FromChronoToCascade(const ChFrame<>& from_coord, TopLoc_Location& to_coord);

private:
		// pointer to cascade OCAF doc handle; 
	Handle_TDocStd_Document* doc;

};









} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif // END of header

