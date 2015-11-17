//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010, 2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHC_TRIANGLEMESH_H
#define CHC_TRIANGLEMESH_H

//////////////////////////////////////////////////
//
//   ChCTriangleMesh.h
//
//   Basic interface for triangle meshes in 3d.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include <math.h>

#include "ChCTriangle.h"

namespace chrono {
namespace geometry {

#define CH_GEOCLASS_TRIANGLEMESH 9

///
/// A basic triangle mesh: just a list of triangles (no edge connectivity info).
///

class ChApi ChTriangleMesh : public ChGeometry {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChTriangleMesh, ChGeometry);

    //
    // DATA
    //

  public:
    ChTriangleMesh(){};

    //
    // MESH INTERFACE FUNCTIONS
    //

    /// Add a triangle to this triangle mesh, by specifying the three coordinates
    virtual void addTriangle(const ChVector<>& vertex0, const ChVector<>& vertex1, const ChVector<>& vertex2) = 0;

    /// Add a triangle to this triangle mesh, by specifying a ChTriangle
    virtual void addTriangle(const ChTriangle& atriangle) = 0;

    /// Get the number of triangles already added to this mesh
    virtual int getNumTriangles() const = 0;

    /// Get the n-th triangle in mesh
    virtual ChTriangle getTriangle(int index) const = 0;

    /// Clear all data
    virtual void Clear() = 0;

    /// Transform all vertexes, by displacing and rotating (rotation  via matrix, so also scaling if needed)
    virtual void Transform(const ChVector<> displ, const ChMatrix33<> rotscale) = 0;

    /// Transform all vertexes, by displacing and rotating (rotation  via matrix, so also scaling if needed)
    virtual void Transform(const ChVector<> displ, const ChQuaternion<> mquat = ChQuaternion<>(1,0,0,0)) {
        this->Transform(displ, ChMatrix33<>(mquat));
    }

    //
    // OVERRIDE BASE CLASS FUNCTIONS
    //

    virtual int GetClassType() { return CH_GEOCLASS_TRIANGLEMESH; };

    /*
    virtual void GetBoundingBox(double& xmin, double& xmax,
                        double& ymin, double& ymax,
                        double& zmin, double& zmax,
                        ChMatrix33<>* Rot = NULL) { }; //TODO

    virtual Vector Baricenter();//TODO
    virtual void CovarianceMatrix(ChMatrix33<>& C);//TODO
    */
    /// This is a surface
    virtual int GetManifoldDimension() { return 2; }

    //
    // SERIALIZATION
    //

    virtual void ArchiveOUT(ChArchiveOut& marchive)
    {
        // version number
        marchive.VersionWrite(1);
        // serialize parent class
        ChGeometry::ArchiveOUT(marchive);
        // serialize all member data:
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) 
    {
        // version number
        int version = marchive.VersionRead();
        // deserialize parent class
        ChGeometry::ArchiveIN(marchive);
        // stream in all member data:
    }
};

}  // END_OF_NAMESPACE____
}  // END_OF_NAMESPACE____

#endif
