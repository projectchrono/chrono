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

#ifndef CH_CASCADE_DOC_H
#define CH_CASCADE_DOC_H

#include "chrono_cascade/ChApiCASCADE.h"

#include "chrono/core/ChFrameMoving.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChContactMaterial.h"

#include <TDocStd_Document.hxx>

class TopoDS_Shape;
class TopLoc_Location;
class TDF_Label;

namespace chrono {
namespace cascade {

/// @addtogroup cascade_module
/// @{

/// Class that contains an OCAF document (a tree hierarchy of shapes in the OpenCascade framework).
/// Most often this is populated by loading a STEP file from disk.
class ChApiCASCADE ChCascadeDoc {
  public:
    ChCascadeDoc();
    virtual ~ChCascadeDoc();

    /// Populate the document with all shapes that are contained in the STEP file.
    /// Return true if load is successful.
    bool LoadSTEP(const std::string& filename);

    /// Dump the shapes hierarchy on given stream.
    void Dump(std::ostream& stream) const;

    /// Get the root shape.
    /// Note that there could be more than one root: if so, use 'num' to select the one that you need.
    bool GetRootShape(TopoDS_Shape& shape, const int num = 1) const;

    /// Get a sub-shape with a given name, returned in 'shape'.
    /// Since the document can contain assemblies, subassemblies, etc.,
    /// the input name can use a 'directory type' syntax using the '/' slash such as: "assembly/subassebmly/subsubassembly/mypart".
    /// It is possible to use '#' and '?' wildcards as in Unix.
    /// If there are multiple parts (or assemblies) with the same name, only the first instance is returned in 'shape'.
    /// Otherwise, one can use the '#' wildcard to get the n-th object, such as: "MyAssembly/bolt#3" or "Car/Wheel#2/hub".
    /// If the 'set_location_to_root' parameter is true (default), the location of the
    /// shape is changed so that it represents its position with respect to the root, that is
    /// the shape .Location() function will give the absolute position, otherwise if false
    /// it will give its position relative to the assembly where it is a sub-shape.
    /// If the 'get_multiple' = true, if there are multiple parts satisfying the search string,
    /// they are all returned in a single shape of compound type (with null location).
    bool GetNamedShape(TopoDS_Shape& shape,
                       const std::string& name,
                       bool set_location_to_root = true,
                       bool get_multiple = false) const;

    /// Class to be used as a callback interface for post-processing Cascade shapes.
    class ChApiCASCADE ScanShapesCallback {
      public:
        /// Callback function to be executed for each scanned Cascade shape.
        /// If this function returns 'false', processing of children shapes is skipped.
        virtual bool ForShape(TopoDS_Shape& shape,
                              TopLoc_Location& loc,
                              const std::string& name,
                              int level,
                              TDF_Label& label) = 0;
    };

    /// Scan all Cascade shapes and execute the provided callback for each one.
    void ScanCascadeShapes(ScanShapesCallback& callback) const;

    /// Get the volume properties (center of mass, inertia moments, volume) of a given shape.
    static bool GetVolumeProperties(
        const TopoDS_Shape& shape,    ///< pass the shape here
        const double density,         ///< pass the density here
        ChVector3d& center_position,  ///< get the COG position center, with respect to shape pos
        ChVector3d& inertiaXX,        ///< get the inertia diagonal terms
        ChVector3d& inertiaXY,        ///< get the inertia extradiagonal terms
        double& volume,               ///< get the volume
        double& mass                  ///< get the mass
    );

    /// Convert OpenCascade coordinates into Chrono coordinates.
    static void ConvertFrameCascadeToChrono(const TopLoc_Location& from_coord, ChFramed& to_coord);

    /// Convert Chrono coordinates into OpenCascade coordinates.
    static void ConvertFrameChronoToCascade(const ChFramed& from_coord, TopLoc_Location& to_coord);

    /// Get internal OCAF document.
    opencascade::handle<TDocStd_Document>* GetDocument() const { return doc; }

  private:
    /// Cascade OCAF doc handle.
    /// NOTE: if simply using 'Handle(TDocStd_Document) doc' (ie. with no pointer to handle), it crashes.
    opencascade::handle<TDocStd_Document>* doc;
};

/// @} cascade_module

}  // end namespace cascade
}  // end namespace chrono

#endif
