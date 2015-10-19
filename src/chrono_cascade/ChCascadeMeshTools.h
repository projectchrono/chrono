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

#ifndef CHCASCADEMESHTOOLS_H
#define CHCASCADEMESHTOOLS_H

//////////////////////////////////////////////////
//
//   ChCascadeMeshTools.h
//
//   Tools to convert an OpenCASCADE shape into
//   triangle meshes.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "chrono_cascade/ChApiCASCADE.h"

#include "core/ChStream.h"
#include "geometry/ChCTriangleMesh.h"

class TopoDS_Face;
class TopoDS_Shape;
class Poly_Connect;
class TColgp_Array1OfDir;
class Handle_TDocStd_Document;
class TopLoc_Location;
class TDF_Label;

namespace chrono {
namespace cascade {

/// Tools to convert an OpenCASCADE shapes into
/// triangle meshes.

class ChApiCASCADE ChCascadeMeshTools {
  public:
    //---------------------------------------------------------------------------------
    // CONVERSION TO CHRONO TRIANGLE MESHES

    /// This function can be used to convert a OpenCASCADE face into a triangle mesh.
    /// The face must be already mshed (ex because you called fillTriangleMeshFromCascade before).
    static void fillTriangleMeshFromCascadeFace(
        geometry::ChTriangleMesh& chmesh,  ///< Mesh that will be filled with triangles
        const TopoDS_Face& F               ///< OpenCASCADE face to be meshed
        );

    /// This function can be used to convert a OpenCASCADE shape into a
    /// Chrono::Engine ChTriangleMesh triangle mesh.
    static void fillTriangleMeshFromCascade(
        geometry::ChTriangleMesh& chmesh,  ///< Mesh that will be filled with triangles
        const TopoDS_Shape& mshape,        ///< OpenCASCADE face to be meshed
        double deflection = 1,             ///< Tolerance on meshing (the lower, the finer the mesh)
        bool   relative_deflection= false, ///< If true, deflection is relative to face size
        double angulardeflection = 0.5);

    //---------------------------------------------------------------------------------
    // CONVERSION TO 'OBJ' WAVEFRONT FILE FORMAT

    /// This function can be used to convert a OpenCASCADE shape into a
    /// 'obj' file format. The file 'objfile' must be already opened, and empty.
    /// Also normals are saved.
    static void fillObjFileFromCascade(
        ChStreamOutAscii& objfile,   ///< the .obj file will be written here
        const TopoDS_Shape& mshape,  ///<
        double deflection = 1,       ///< Tolerance on meshing (the lower, the finer the mesh)
        bool   relative_deflection= false, ///< If true, deflection is relative to face size
        double angulardeflection = 0.5);


};

}  // END_OF_NAMESPACE____
}  // END_OF_NAMESPACE____

#endif  // END of header