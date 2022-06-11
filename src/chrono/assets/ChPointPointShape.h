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

#ifndef CH_POINTPOINT_SHAPE_H
#define CH_POINTPOINT_SHAPE_H

#include "chrono/assets/ChLineShape.h"

namespace chrono {

/// Base class for visualization of some deformable line shape
/// between two moving points related to the parent ChPhysicsItem.
/// (at this point, only ChLink and its derivatives are supported.)
/// NOTE: An instance of this class should not be shared among multiple ChPhysicsItem instances.
/// Otherwise drawing may broken since each physics item will try to update
/// geometry of the line and causes race conditions.

class ChApi ChPointPointShape : public ChLineShape {
  public:
    ChPointPointShape() = default;

    // Call UpdateLineGeometry() if updater has any pair of points.
    // (at this point, only ChLink and its derivatives are supported.)
    virtual void Update(ChPhysicsItem* updater, const ChFrame<>& coords) override;

  private:
    // Update underlying line geometry from given two endpoints.
    // This method will be called on Update() call and should be implemented by derived classes.
    virtual void UpdateLineGeometry(const ChVector<>& endpoint1, const ChVector<>& endpoint2) = 0;
};

/// Shape for visualizing a line segment between two moving points related to the parent ChPhysicsItem.
/// - Currently, only ChLink and its derivatives are supported.
/// - An instance of this class should not be shared among multiple ChPhysicsItem instances. Otherwise drawing may
///   broken since each physics item will try to update geometry of the line and causes race conditions.
class ChApi ChSegmentShape : public ChPointPointShape {
  private:
    // Set line geometry as segment between two end point
    virtual void UpdateLineGeometry(const ChVector<>& endpoint1, const ChVector<>& endpoint2) override;
};

/// Shape for visualizing a coil spring between two moving points related to the parent ChPhysicsItem.
/// Notes:
/// - Currently, only ChLink and its derivatives are supported.
/// - An instance of this class should not be shared among multiple ChPhysicsItem instances. Otherwise drawing may
///   broken since each physics item will try to update geometry of the line and causes race conditions.
class ChApi ChSpringShape : public ChPointPointShape {
  public:
    ChSpringShape(double mradius = 0.05, int mresolution = 65, double mturns = 5.)
        : radius(mradius), turns(mturns), resolution(mresolution) {}

  private:
    // Set line geometry as coil between two end point
    virtual void UpdateLineGeometry(const ChVector<>& endpoint1, const ChVector<>& endpoint2) override;

  private:
    double radius;
    double turns;
    size_t resolution;
};

/// Shape representing a rotational spring.
class ChApi ChRotSpringShape : public ChLineShape {
  public:
    ChRotSpringShape(double radius, int resolution = 65) : m_radius(radius), m_resolution(resolution) {}

    virtual void Update(ChPhysicsItem* updater, const ChFrame<>& coords) override;

  private:
    double m_radius;
    size_t m_resolution;
};

}  // end namespace chrono

#endif
