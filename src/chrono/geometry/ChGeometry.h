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

#ifndef CH_GEOMETRY_H
#define CH_GEOMETRY_H

#include <memory>
#include <limits>

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChFrame.h"

namespace chrono {

/// @addtogroup chrono_geometry
/// @{

/// Enumeration for the 3 frame directions.
enum class ChAxis {
    X,  ///< x direction of a reference frame
    Y,  ///< y direction of a reference frame
    Z   ///< z direction of a reference frame
};

/// Axis-aligned bounding box.
struct ChApi ChAABB {
    /// Default is an inverted bounding box.
    ChAABB();

    /// Construct an AABB with provided corners.
    ChAABB(const ChVector3d& aabb_min, const ChVector3d& aabb_max);

    /// Get AABB center.
    ChVector3d Center() const;

    /// Get AABB dimensions.
    ChVector3d Size() const;

    /// Return true for an inverted bounding box.
    bool IsInverted() const;

    /// Return the union of this AABB and the specified AABB.
    ChAABB operator+(const ChAABB& aabb);

    /// Include the specified AABB in this AABB.
    ChAABB& operator+=(const ChAABB& aabb);

    /// Include the specified point in this AABB.
    ChAABB& operator+=(const ChVector3d p);

    /// Transform by the given frame.
    ChAABB Transform(const ChFrame<>& frame) const;

    ChVector3d min;  ///< low AABB corner
    ChVector3d max;  ///< high AABB corner
};

/// Base class for geometric objects used for collisions and visualization.
class ChApi ChGeometry {
  public:
    /// Enumeration of geometric object types.
    enum class Type {
        NONE,
        SPHERE,
        ELLIPSOID,
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
        TRIANGLEMESH,
        TRIANGLEMESH_CONNECTED,
        TRIANGLEMESH_SOUP
    };

  public:
    ChGeometry() {}
    ChGeometry(const ChGeometry& other) {}
    virtual ~ChGeometry() {}

    /// "Virtual" copy constructor.
    virtual ChGeometry* Clone() const = 0;

    /// Get the class type as an enum.
    virtual Type GetType() const { return Type::NONE; }

    /// Compute bounding box along the directions of the shape definition frame.
    /// The default implementation returns a bounding box with zero dimensions.
    virtual ChAABB GetBoundingBox() const;

    /// Enlarge the given existing bounding box with the bounding box of this object.
    void InflateBoundingBox(ChAABB& bbox) const;

    /// Returns the radius of a bounding sphere for this geometry.
    /// The default implementation returns the radius of a sphere bounding the geometry bounding box, which is not
    /// always the tightest possible.
    virtual double GetBoundingSphereRadius() const;

    /// Compute center of mass.
    virtual ChVector3d Baricenter() const { return VNULL; }

    /// Returns the dimension of the geometry
    /// (0=point, 1=line, 2=surface, 3=solid)
    virtual int GetManifoldDimension() const { return 0; }

    /// Generic update of internal data.
    virtual void Update() {}

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out);

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in);
};

/// @} chrono_geometry

CH_CLASS_VERSION(ChGeometry, 0)

}  // end namespace chrono

#endif
