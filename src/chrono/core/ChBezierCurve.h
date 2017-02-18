// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Definitions of the classes that implement a Bezier 3D path.
//
// ChBezierCurve
//    This class encapsulates a piece-wise cubic Bezier approximation of a
//    3D curve, represented as a set of three arrays of locations. For each
//    point on the curve, we also define a vector 'inCV' which represents the
//    vertex of the control polygon prior to the point and a vector 'outCV'
//    which represents the vertex of the control polygon following the point.
//    This class provides methods for evaluating the value, as well as the
//    first and second derivatives of a point on a specified interval of the
//    piece-wise 3D curve (using the Bernstein polynomial representation of
//    Bezier curves). In addition, it provides a method for calculating the
//    closest point on a specified interval of the curve to a specified
//    location.
//
// ChBezierCurveTracker
//    This utility class implements a tracker for a given path. It uses time
//    coherence in order to provide an appropriate initial guess for the
//    iterative (Newton) root finder.
//
// =============================================================================

#ifndef CH_BEZIER_CURVE_H
#define CH_BEZIER_CURVE_H

#include <vector>
#include <string>

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChVector.h"
#include "chrono/serialization/ChArchive.h"

namespace chrono {

// -----------------------------------------------------------------------------
/// Definition of a piece-wise cubic Bezier approximation of a 3D curve.
///
/// This class encapsulates a piece-wise cubic Bezier approximation of a
/// 3D curve, represented as a set of three arrays of locations. For each
/// point on the curve, we also define a vector 'inCV' which represents the
/// vertex of the control polygon prior to the point and a vector 'outCV'
/// which represents the vertex of the control polygon following the point.
/// This class provides methods for evaluating the value, as well as the
/// first and second derivatives of a point on a specified interval of the
/// piece-wise 3D curve (using the Bernstein polynomial representation of
/// Bezier curves). In addition, it provides a method for calculating the
/// closest point on a specified interval of the curve to a specified
/// location.
// -----------------------------------------------------------------------------
class ChApi ChBezierCurve {
  public:
    /// Constructor from specified nodes and control points.
    ChBezierCurve(const std::vector<ChVector<> >& points,
                  const std::vector<ChVector<> >& inCV,
                  const std::vector<ChVector<> >& outCV);

    /// Constructor from specified nodes.
    /// In this case, we evaluate the control polygon vertices inCV and outCV
    /// so that we obtain a piecewise cubic spline interpolant of the given knots.
    ChBezierCurve(const std::vector<ChVector<> >& points);

    /// Default constructor (required by serialization)
    ChBezierCurve() {}

    /// Destructor for ChBezierCurve.
    ~ChBezierCurve() {}

    /// Set the nodes and control points
    void setPoints(const std::vector<ChVector<> >& points,
                   const std::vector<ChVector<> >& inCV,
                   const std::vector<ChVector<> >& outCV);

    /// Return the number of knot points.
    size_t getNumPoints() const { return m_points.size(); }

    /// Return the knot point with specified index.
    const ChVector<>& getPoint(size_t i) const { return m_points[i]; }

    /// Evaluate the value of the Bezier curve.
    /// This function calculates and returns the point on the curve in the
    /// specified interval between two knot points and at the given curve
    /// parameter (assumed to be in [0,1]). It uses the Bernstein polynomial
    /// representation of a Bezier curve.
    ChVector<> eval(size_t i, double t) const;

    /// Evaluate the tangent vector to the Bezier curve.
    /// This function calculates and returns the first derivative (tangent vector)
    /// to the curve in the specified interval between two knot points and at the
    /// given curve parameter (assumed to be in [0,1]). It uses the Bernstein
    /// polynomial representation of a Bezier curve.
    ChVector<> evalD(size_t i, double t) const;

    /// Evaluate the second derivative vector to the Bezier curve.
    /// This function calculates and returns the second derivative vector to the
    /// curve in the specified interval between two knot points and at the given
    /// curve parameter (assumed to be in [0,1]). It uses the Bernstein polynomial
    /// representation of a Bezier curve.
    ChVector<> evalDD(size_t i, double t) const;

    /// Calculate the closest point on the curve to the given location.
    /// This function calculates and returns the point on the curve in the specified
    /// interval that is closest to the specified location. On input, the value 't' is
    /// an initial guess. On return, it contains the curve parameter corresponding
    /// to the closest point.
    ChVector<> calcClosestPoint(const ChVector<>& loc, size_t i, double& t) const;

    /// Write the knots and control points to the specified file.
    void write(const std::string& filename);

    /// Create a ChBezierCurve using data in the specified file.
    /// The input file is assumed to contain on the first line the number of data
    /// points and the number of data columns.  The latter can be one of 3 or 9.
    /// In the first case, subsequent lines should contain the coordinates of the
    /// curve knots (one point per line). The returned Bezier curve is a piecewise
    /// cubic spline through the specified points.
    /// In the second case, subsequent lines should contain the coordinates of the
    /// curve knot, the coordinates of the "incoming" control point, and the
    /// coordinates of the "outgoing" control point (i.e. 9 values per line). The
    /// returned curve is a general Bezier curve using the specified knots and
    /// control polygons.
    static ChBezierCurve* read(const std::string& filename);


    //
    // SERIALIZATION
    //

    void ArchiveOUT(ChArchiveOut& marchive)
    {
        // version number
        marchive.VersionWrite<ChBezierCurve>();

        // serialize all member data:
        marchive << CHNVP(m_points);
        marchive << CHNVP(m_inCV);
        marchive << CHNVP(m_outCV);
        marchive << CHNVP(m_maxNumIters);
        marchive << CHNVP(m_sqrDistTol);
        marchive << CHNVP(m_cosAngleTol);
        marchive << CHNVP(m_paramTol);
    }

    /// Method to allow de serialization of transient data from archives.
    void ArchiveIN(ChArchiveIn& marchive) 
    {
        // version number
        int version = marchive.VersionRead<ChBezierCurve>();

        // stream in all member data:
        marchive >> CHNVP(m_points);
        marchive >> CHNVP(m_inCV);
        marchive >> CHNVP(m_outCV);
        marchive >> CHNVP(m_maxNumIters);
        marchive >> CHNVP(m_sqrDistTol);
        marchive >> CHNVP(m_cosAngleTol);
        marchive >> CHNVP(m_paramTol);
    }

  private:
    /// Utility function to solve for the outCV control points.
    /// This function solves the resulting tridiagonal system for one of the
    /// coordinates (x, y, or z) of the outCV control points, to impose that the
    /// resulting Bezier curve is a spline interpolant of the knots.
    static void solveTriDiag(size_t n, double* rhs, double* x);

    std::vector<ChVector<> > m_points;  ///< set of knot points
    std::vector<ChVector<> > m_inCV;    ///< set on "incident" control points
    std::vector<ChVector<> > m_outCV;   ///< set of "outgoing" control points

    static const size_t m_maxNumIters;  ///< maximum number of Newton iterations
    static const double m_sqrDistTol;   ///< tolerance on squared distance
    static const double m_cosAngleTol;  ///< tolerance for orthogonality test
    static const double m_paramTol;     ///< tolerance for change in parameter value

    friend class ChBezierCurveTracker;
};

// -----------------------------------------------------------------------------
/// Definition of a tracker on a ChBezierCurve path.
///
/// This utility class implements a tracker for a given path. It uses time
/// coherence in order to provide an appropriate initial guess for the
/// iterative (Newton) root finder.
// -----------------------------------------------------------------------------
class ChApi ChBezierCurveTracker {
  public:
    /// Create a tracker associated with the specified Bezier curve.
      ChBezierCurveTracker(ChBezierCurve* path, bool isClosedPath = false) 
          : m_path(path), m_curInterval(0), m_curParam(0), m_isClosedPath(isClosedPath) {}

    /// Destructor for ChBezierCurveTracker.
    ~ChBezierCurveTracker() {}

    /// Reset the tracker at the specified location.
    /// This function reinitializes the pathTracker at the specified location. It
    /// calculates an appropriate initial guess for the curve segment and sets the
    /// curve parameter to 0.5.
    void reset(const ChVector<>& loc);

    /// Calculate the closest point on the underlying curve to the specified location.
    /// This function returns the closest point on the underlying path to the
    /// specified location. The return value is -1 if this point coincides with the
    /// first point of the path, +1 if it coincides with the last point of the path,
    /// and 0 otherwise. Note that, in order to provide a reasonable initial guess
    /// for the Newton iteration, we use time coherence (by keeping track of the path
    /// interval and curve parameter within that interval from the last query). As
    /// such, this function should be called with a continuous sequence of locations.
    int calcClosestPoint(const ChVector<>& loc, ChVector<>& point);

    /// Set if the path is treated as an open loop or a closed loop for tracking
    void setIsClosedPath(bool isClosedPath);

  private:
    ChBezierCurve* m_path;  ///< associated Bezier curve
    size_t m_curInterval;   ///< current search interval
    double m_curParam;      ///< parameter for current closest point
    bool m_isClosedPath;    ///< treat the path as a closed loop curve
};

CH_CLASS_VERSION(ChBezierCurve,0)

}  // end of namespace chrono

#endif
