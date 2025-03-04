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

#ifndef CH_VISUAL_SHAPE_POINTPOINT_H
#define CH_VISUAL_SHAPE_POINTPOINT_H

#include "chrono/assets/ChVisualShapeLine.h"

namespace chrono {

/// @addtogroup chrono_assets
/// @{

/// Base class for visualization of some deformable line shape
/// between two moving points related to the parent ChPhysicsItem.
/// (at this point, only ChLink and its derivatives are supported.)
/// NOTE: An instance of this class should not be shared among multiple ChPhysicsItem instances.
/// Otherwise drawing may broken since each physics item will try to update
/// geometry of the line and causes race conditions.

class ChApi ChVisualShapePointPoint : public ChVisualShapeLine {
  public:
    ChVisualShapePointPoint() = default;

    // Update the underlying line geometry and set current locations of the end points.
    virtual void Update(ChObj* updater, const ChFrame<>& coords) override;

    const ChVector3d& GetPoint1Abs() const { return point1; }
    const ChVector3d& GetPoint2Abs() const { return point2; }

  private:
    // Update underlying line geometry from given two endpoints.
    // This method will be called on Update() call and should be implemented by derived classes.
    virtual void UpdateLineGeometry(const ChVector3d& endpoint1, const ChVector3d& endpoint2) = 0;

    ChVector3d point1;  ///< location of 1st end point (in global frame)
    ChVector3d point2;  ///< location of 2nd end point (in global frame)
};

/// Shape for visualizing a line segment between two moving points related to the parent ChPhysicsItem.
/// An instance of this class should not be shared among multiple ChPhysicsItem instances. Otherwise drawing may broken
/// since each physics item will try to update geometry of the line and causes race conditions.
class ChApi ChVisualShapeSegment : public ChVisualShapePointPoint {
  private:
    /// Set line geometry as segment between two end points (assumed in local frame).
    virtual void UpdateLineGeometry(const ChVector3d& endpoint1, const ChVector3d& endpoint2) override;
};

/// Shape for visualizing a coil spring between two moving points related to the parent ChPhysicsItem.
/// An instance of this class should not be shared among multiple ChPhysicsItem instances. Otherwise drawing may broken
/// since each physics item will try to update geometry of the line and causes race conditions.
class ChApi ChVisualShapeSpring : public ChVisualShapePointPoint {
  public:
    ChVisualShapeSpring(double mradius = 0.05, int mresolution = 65, double mturns = 5.)
        : radius(mradius), turns(mturns), resolution(mresolution) {}
    double GetRadius() { return radius; }
    size_t GetResolution() { return resolution; }
    double GetTurns() { return turns; }

  private:
    /// Set line geometry as coil between two end points (assumed in local frame).
    virtual void UpdateLineGeometry(const ChVector3d& endpoint1, const ChVector3d& endpoint2) override;

  private:
    double radius;
    double turns;
    size_t resolution;
};

/// Shape representing a rotational spring.
class ChApi ChVisualShapeRotSpring : public ChVisualShapeLine {
  public:
    ChVisualShapeRotSpring(double radius, int resolution = 65) : m_radius(radius), m_resolution(resolution) {}

    virtual void Update(ChObj* updater, const ChFrame<>& coords) override;

  private:
    double m_radius;
    size_t m_resolution;
};

/// @} chrono_assets

}  // end namespace chrono

#endif
