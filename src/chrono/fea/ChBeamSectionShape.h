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

#ifndef CHBEAMSECTIONSHAPE_H
#define CHBEAMSECTIONSHAPE_H

#include "chrono/core/ChMathematics.h"
#include "chrono/core/ChVector.h"
#include <vector>

namespace chrono {
namespace fea {

/// @addtogroup fea_utils
/// @{

/// Base class for drawing tesselated profiles of beams in 3D views, if needed.
/// This cross section visualization shape is independent from physical properties
/// (area, inertia, etc.) that you can define with other components of the ChBeamSection,
/// such as for example ChBeamSectionCosseratElasticity, etc.
/// Used as a component of ChBeamSection.

class ChApi ChBeamSectionShape {
  public:
    virtual ~ChBeamSectionShape() {}

    //
    // Functions for drawing the shape via triangulation:
    //

    /// Get the n. of lines making the profile of the section, for meshing purposes.
    /// C0 continuity is required between lines, C1 also required within each line.
    /// Ex. a circle has 1 line, a cube 4 lines, etc. Sharp corners can be done mith multiple lines.
    virtual int GetNofLines() const = 0;

    /// Get the n. of points to be allocated per each section, for the i-th line in the section.
    /// We assume one also allocates a n. of 3d normals equal to n of points.
    virtual int GetNofPoints(const int i_line) const = 0;

    /// Compute the points (in the reference of the section), for the i-th line in the section.
    /// Note: mpoints must already have the proper size.
    virtual void GetPoints(const int i_line, std::vector<ChVector<>>& mpoints) const = 0;

    /// Compute the normals (in the reference of the section) at each point, for the i-th line in the section.
    /// Note: mnormals must already have the proper size.
    virtual void GetNormals(const int i_line, std::vector<ChVector<>>& mnormals) const = 0;

    /// Returns the axis-aligned bounding box (assuming axes of local reference of the section)
    /// This functions has many uses, ex.for drawing, optimizations, collisions.
    /// We provide a fallback default implementation that iterates over all points thanks to GetPoints(),
    /// but one could override this if a more efficient implementaiton is possible (ex for circular beams, etc.)
    virtual void GetAABB(double& ymin, double& ymax, double& zmin, double& zmax) const;
};

/// A ready-to-use class for drawing properties of circular beams.
/// Used as a component of ChBeamSection

class ChApi ChBeamSectionShapeCircular : public ChBeamSectionShape {
  public:
    ChBeamSectionShapeCircular(double mradius, int mresolution = 10) {
        radius = mradius;
        resolution = mresolution;
        this->UpdateProfile();
    }

    //
    // Functions for drawing the shape via triangulation:
    //

    virtual int GetNofLines() const override { return 1; };

    virtual int GetNofPoints(const int i_line) const override { return resolution + 1; };

    /// Compute the points (in the reference of the section).
    /// Note: mpoints must already have the proper size.
    virtual void GetPoints(const int i_line, std::vector<ChVector<>>& mpoints) const override { mpoints = points; };

    /// Compute the normals (in the reference of the section) at each point.
    /// Note: mnormals must already have the proper size.
    virtual void GetNormals(const int i_line, std::vector<ChVector<>>& mnormals) const override { mnormals = normals; }

    //
    // Functions for drawing, optimizations, collisions
    //

    /// Returns the axis-aligned bounding box (assuming axes of local reference of the section)
    /// This functions has many uses, ex.for drawing, optimizations, collisions.
    virtual void GetAABB(double& ymin, double& ymax, double& zmin, double& zmax) const override;

  private:
    // internal: update internal precomputed vertex arrays
    void UpdateProfile();

    int resolution;
    double radius;
    std::vector<ChVector<>> points;
    std::vector<ChVector<>> normals;
};

/// A ready-to-use class for drawing properties of rectangular beams.
/// Used as a component of ChBeamSection.

class ChApi ChBeamSectionShapeRectangular : public ChBeamSectionShape {
  public:
    ChBeamSectionShapeRectangular(double y_width, double z_width) {
        z_thick = z_width;
        y_thick = y_width;
        this->UpdateProfile();
    }

    //
    // Functions for drawing the shape via triangulation:
    //

    virtual int GetNofLines() const override { return 4; };

    virtual int GetNofPoints(const int i_line) const override { return 2; };

    /// Compute the points (in the reference of the section).
    /// Note: mpoints must already have the proper size.
    virtual void GetPoints(const int i_line, std::vector<ChVector<>>& mpoints) const override {
        mpoints = ml_points[i_line];
    };

    /// Compute the normals (in the reference of the section) at each point.
    /// Note: mnormals must already have the proper size.
    virtual void GetNormals(const int i_line, std::vector<ChVector<>>& mnormals) const override {
        mnormals = ml_normals[i_line];
    }

    /// Returns the axis-aligned bounding box (assuming axes of local reference of the section)
    virtual void GetAABB(double& ymin, double& ymax, double& zmin, double& zmax) const override;

  private:
    // internal: update internal precomputed vertex arrays
    void UpdateProfile();

    double y_thick;
    double z_thick;
    std::vector<std::vector<ChVector<>>> ml_points;
    std::vector<std::vector<ChVector<>>> ml_normals;
};

/// A class for drawing properties of beams whose section is a set of M polylines, each with N points.
/// The shading will show the longitudinal edges extruded at each Nth point as a smooth edge,
/// so if you need sharp edges, just create multiple polylines (ex a quadrialteral=4 polylines).
/// Used as a component of ChBeamSection.

class ChApi ChBeamSectionShapePolyline : public ChBeamSectionShape {
  public:
    ChBeamSectionShapePolyline(const std::vector<std::vector<ChVector<>>>& polyline_points) {
        this->ml_points = polyline_points;
        this->UpdateProfile();
    }

    //
    // Functions for drawing the shape via triangulation:
    //

    virtual int GetNofLines() const override { return (int)this->ml_points.size(); };

    virtual int GetNofPoints(const int i_line) const override { return (int)this->ml_points[i_line].size(); };

    /// Compute the points (in the reference of the section).
    /// Note: mpoints must already have the proper size.
    virtual void GetPoints(const int i_line, std::vector<ChVector<>>& mpoints) const override {
        mpoints = ml_points[i_line];
    };

    /// Compute the normals (in the reference of the section) at each point.
    /// Note: mnormals must already have the proper size.
    virtual void GetNormals(const int i_line, std::vector<ChVector<>>& mnormals) const override {
        mnormals = ml_normals[i_line];
    }

  private:
    // internal: update internal precomputed vertex arrays, computing normals by smoothing segments
    void UpdateProfile();

    std::vector<std::vector<ChVector<>>> ml_points;
    std::vector<std::vector<ChVector<>>> ml_normals;
};

/// @} fea_utils

}  // end namespace fea
}  // end namespace chrono

#endif
