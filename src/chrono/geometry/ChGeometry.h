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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHC_GEOMETRY
#define CHC_GEOMETRY

#include <memory>

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChMath.h"

namespace chrono {

/// Namespace for classes which represent basic geometric objects
namespace geometry {

/// @addtogroup chrono_geometry
/// @{

/// Base class for geometric objects used for collisions and visualization.
class ChApi ChGeometry {
  public:
    /// Enumeration of geometric objects
    enum GeometryType {
        NONE,
        SPHERE,
        BOX,
        CYLINDER,
        TRIANGLE,
        CAPSULE,
        CONE,
        LINE,
        LINE_ARC,
        LINE_BEZIER,
        LINE_CAM,
        LINE_PATH,
        LINE_POLY,
        LINE_SEGMENT,
        ROUNDED_BOX,
        ROUNDED_CYLINDER,
        ROUNDED_CONE,
        TRIANGLEMESH,
        TRIANGLEMESH_CONNECTED,
        TRIANGLEMESH_SOUP
    };

  public:
    ChGeometry() {}
    ChGeometry(const ChGeometry& source) {}
    virtual ~ChGeometry() {}

    /// "Virtual" copy constructor.
    virtual ChGeometry* Clone() const = 0;

    /// Get the class type as unique numerical ID (faster
    /// than using ChronoRTTI mechanism).
    /// Each inherited class must return an unique ID.
    virtual GeometryType GetClassType() const { return NONE; }

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
                                ChMatrix33<>* Rot = nullptr) const {
        xmin = xmax = ymin = ymax = zmin = zmax = 0.0;
    }

    /// Enlarge a previous existing bounding box.
    /// Usually it does not need to be overridden: base function uses GetBoundingBox()
    /// If Rot is not null, the bounding box axes are considered rotated.
    virtual void InflateBoundingBox(double& xmin,
                                    double& xmax,
                                    double& ymin,
                                    double& ymax,
                                    double& zmin,
                                    double& zmax,
                                    ChMatrix33<>* Rot = NULL) const;

    /// Returns the radius of the sphere which can enclose the geometry
    virtual double Size() const;

    /// Compute center of mass
    /// It should be overridden by inherited classes
    virtual ChVector<> Baricenter() const { return VNULL; }

    /// Compute the 3x3 covariance matrix (only the diagonal and upper part)
    /// It should be overridden by inherited classes
    // TODO: obsolete (unused)
    virtual void CovarianceMatrix(ChMatrix33<>& C) const { C.setZero(); }

    /// Tells the dimension of the geometry
    /// (0=point, 1=line, 2=surface, 3=solid)
    virtual int GetManifoldDimension() const { return 0; }

    /// Generic update of internal data. Default, does nothing.
    /// Most often it is not needed to implement this method, but
    /// some inherited classes may implement it (ex. to update references to
    /// external data. etc.).
    virtual void Update() {}

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive);

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive);
};

/// @} chrono_geometry

}  // end namespace geometry

CH_CLASS_VERSION(geometry::ChGeometry, 0)

}  // end namespace chrono

#endif
