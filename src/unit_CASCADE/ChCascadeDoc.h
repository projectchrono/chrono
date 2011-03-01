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
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
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

		/// Get the volume properties (center of mass, inertia moments, volume)
		/// of a given shape.
	bool GetVolumeProperties(const TopoDS_Shape& mshape,	///< pass the shape here
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
		/// Execute a callback on all contained shapes
	void ScanCascadeShapes(callback_CascadeDoc& mcallback);

	
private:
		// pointer to cascade OCAF doc handle; 
	Handle_TDocStd_Document* doc;

};









} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif // END of header

