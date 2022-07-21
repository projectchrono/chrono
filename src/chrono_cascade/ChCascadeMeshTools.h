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

#ifndef CHCASCADEMESHTOOLS_H
#define CHCASCADEMESHTOOLS_H

#include "chrono_cascade/ChApiCASCADE.h"
#include "chrono_cascade/ChCascadeTriangulate.h"

#include "chrono/core/ChStream.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

class TopoDS_Face;
class TopoDS_Shape;
class Poly_Connect;
class TopLoc_Location;
class TDF_Label;

namespace chrono {
namespace cascade {

/// @addtogroup cascade_module
/// @{

/// Tools to convert an OpenCASCADE shapes into triangle meshes.
class ChApiCASCADE ChCascadeMeshTools {
  public:
    /// This function can be used to convert a OpenCASCADE face into a triangle mesh.
    /// The face must be already mshed (ex because you called fillTriangleMeshFromCascade before).
    static void fillTriangleMeshFromCascadeFace(
        geometry::ChTriangleMeshConnected& mesh,  ///< Mesh that will be filled with triangles
        const TopoDS_Face& F                      ///< OpenCASCADE face to be meshed
    );

    /// This function can be used to convert a OpenCASCADE shape into a
    /// Chrono ChTriangleMesh triangle mesh.
    static void fillTriangleMeshFromCascade(
        geometry::ChTriangleMeshConnected& mesh,  ///< Mesh that will be filled with triangles
        const TopoDS_Shape& shape,                ///< OpenCASCADE face to be meshed
        const ChCascadeTriangulate& tolerances    ///< tesselation tolerances
    );

    /// This function can be used to convert a OpenCASCADE shape into a
    /// 'obj' file format. The file 'objfile' must be already opened, and empty.
    /// Also normals are saved.
    static void fillObjFileFromCascade(ChStreamOutAscii& objfile,  ///< the .obj file will be written here
                                       const TopoDS_Shape& shape,  ///< OpenCASCADE face to be output as 'obj' file
                                       const ChCascadeTriangulate& tolerances  ///< tesselation tolerances
    );
};

/// @} cascade_module

}  // end namespace cascade
}  // end namespace chrono

#endif  // END of header