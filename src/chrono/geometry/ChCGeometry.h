//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHC_GEOMETRY
#define CHC_GEOMETRY

//////////////////////////////////////////////////
//
//   ChCGeometry.h
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include <memory>

#include "core/ChMath.h"
#include "core/ChApiCE.h"

namespace chrono {

/// Namespace for classes which represent basic geometric objects
namespace geometry {

/// @addtogroup chrono_geometry
/// @{

#define CH_GEOCLASS_GEOMETRY 0

/// Base class for geometric objects.
/// Geometric object are used for collisions and such.
class ChApi ChGeometry {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI_ROOT(ChGeometry);

  public:
    //
    // CONSTRUCTORS
    //

    ChGeometry();

    virtual ~ChGeometry();

    virtual void Copy(ChGeometry* source){};

    virtual ChGeometry* Duplicate() {
        ChGeometry* mgeo = new ChGeometry();
        mgeo->Copy(this);
        return mgeo;
    };

    /// Get the class type as unique numerical ID (faster
    /// than using ChronoRTTI mechanism).
    /// Each inherited class must return an unique ID.
    virtual int GetClassType() { return CH_GEOCLASS_GEOMETRY; };

    //
    // GEOMETRIC FUNCTIONS
    //

    /// Compute bounding box.
    /// If a matrix Rot is not null, it should compute bounding box along
    /// the rotated directions represented by that transformation matrix Rot.
    /// It must be overridden by inherited classes.
    virtual void GetBoundingBox(double& xmin,
                                double& xmax,
                                double& ymin,
                                double& ymax,
                                double& zmin,
                                double& zmax,
                                ChMatrix33<>* Rot = NULL) {
        xmin = xmax = ymin = ymax = zmin = zmax = 0.0;
    }

    /// Enlarge a previous existing bounding box.
    /// Usually it does not need to be overriden: base function uses GetBoundingBox()
    /// If Rot is not null, the bounding box axes are considered rotated.
    virtual void InflateBoundingBox(double& xmin,
                                    double& xmax,
                                    double& ymin,
                                    double& ymax,
                                    double& zmin,
                                    double& zmax,
                                    ChMatrix33<>* Rot = NULL) {
        double bxmin, bxmax, bymin, bymax, bzmin, bzmax;
        this->GetBoundingBox(bxmin, bxmax, bymin, bymax, bzmin, bzmax, Rot);
        if (xmin > bxmin)
            xmin = bxmin;
        if (ymin > bymin)
            ymin = bymin;
        if (zmin > bzmin)
            zmin = bzmin;
        if (xmax < bxmax)
            xmax = bxmax;
        if (ymax < bymax)
            ymax = bymax;
        if (zmax < bzmax)
            zmax = bzmax;
    };

    /// Returns the radius of the sphere which can enclose the geometry
    virtual double Size() {
        double bxmin, bxmax, bymin, bymax, bzmin, bzmax;
        this->GetBoundingBox(bxmin, bxmax, bymin, bymax, bzmin, bzmax);
        return sqrt(pow((0.5 * (bxmax - bxmin)), 2) + pow((0.5 * (bymax - bymin)), 2) +
                    pow((0.5 * (bzmax - bzmin)), 2));
    };

    /// Evaluates a point on a geometry, given parametric coordinates,
    /// if possible. Parameters  U,V,W should be usually in 0..1 range.
    /// For a line, only U parameter is needed, for a surface also V.
    /// Computed value goes into the 'pos' reference.
    /// It should be overriden by inherited classes.
    virtual void Evaluate(Vector& pos, const double parU, const double parV = 0., const double parW = 0.) {
        pos = VNULL;
    };

    /// Evaluates a tangent versor on a geometry, given parametric coordinates,
    /// if possible. Parameters  U,V,W should be usually in 0..1 range.
    /// For a line, only U parameter is needed, for a surface also V.
    /// Computed value goes into the 'pos' reference.
    /// It could be overriden by inherited classes if a precise solution is
    /// known (otherwise it defaults to numerical BDF using the Evaluate()
    /// function.
    virtual void Derive(Vector& dir, const double parU, const double parV = 0., const double parW = 0.);

    /// Compute center of mass
    /// It should be overriden by inherited classes
    virtual Vector Baricenter() { return VNULL; };

    /// Compute the 3x3 covariance matrix (only the diagonal and upper part)
    /// It should be overriden by inherited classes
    virtual void CovarianceMatrix(ChMatrix33<>& C) { C.Reset(); };

    /// Tells the dimension of the geometry
    /// (0=point, 1=line, 2=surface, 3=solid)
    virtual int GetManifoldDimension() { return 0; }

    /// Generic update of internal data. Default, does nothing.
    /// Most often it is not needed to implement this method, but
    /// some inherited classes may implement it (ex. to update references to
    /// external data. etc.).
    virtual void Update(){};

    //
    // SERIALIZATION
    //

    virtual void ArchiveOUT(ChArchiveOut& marchive)
    {
        // version number
        marchive.VersionWrite(1);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) 
    {
        // version number
        int version = marchive.VersionRead();
    }

};

/// @} chrono_geometry

}  // END_OF_NAMESPACE____
}  // END_OF_NAMESPACE____

#endif
