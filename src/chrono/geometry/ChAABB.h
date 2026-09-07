// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#ifndef CH_AABB_H
#define CH_AABB_H

#include <memory>
#include <limits>

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChFrame.h"

#include "chrono/geometry/ChAABB.h"

#include "chrono/serialization/ChArchive.h"

namespace chrono {

/// Axis-aligned bounding box in integer grid coordinates.
struct ChApi ChIntAABB {
    /// Default is an inverted bounding box.
    ChIntAABB();

    /// Construct an AABB with provided corners.
    ChIntAABB(const ChVector3i& aabb_min, const ChVector3i& aabb_max);

    /// Get AABB dimensions.
    ChVector3i Size() const;

    /// Return true for an inverted bounding box.
    bool IsInverted() const;

    /// Return the union of this AABB and the specified AABB.
    ChIntAABB operator+(const ChIntAABB& aabb);

    /// Include the specified AABB in this AABB.
    ChIntAABB& operator+=(const ChIntAABB& aabb);

    /// Include the specified point in this AABB.
    ChIntAABB& operator+=(const ChVector3i p);

    /// Method to allow serialization of transient data to archives.
    void ArchiveOut(ChArchiveOut& archive_out);

    /// Method to allow de serialization of transient data from archives.
    void ArchiveIn(ChArchiveIn& archive_in);

    ChVector3i min;  ///< low AABB corner
    ChVector3i max;  ///< high AABB corner
};

// -----------------------------------------------------------------------------

/// Axis-aligned bounding box.
struct ChApi ChAABB {
    /// Default is an inverted bounding box.
    ChAABB();

    /// Construct an AABB with provided corners.
    ChAABB(const ChVector3d& aabb_min, const ChVector3d& aabb_max);

    /// Construct an AABB from the given integer AABB and grid spacing.
    ChAABB(const ChIntAABB& aabb, double spacing);

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

    /// Inflate this AABB by the given vector.
    ChAABB Inflate(const ChVector3d& v) const;

    /// Transform by the given frame.
    ChAABB Transform(const ChFramed& frame) const;

    /// Method to allow serialization of transient data to archives.
    void ArchiveOut(ChArchiveOut& archive_out);

    /// Method to allow de serialization of transient data from archives.
    void ArchiveIn(ChArchiveIn& archive_in);

    ChVector3d min;  ///< low AABB corner
    ChVector3d max;  ///< high AABB corner
};

/// @} chrono_geometry

}  // end namespace chrono

#endif
