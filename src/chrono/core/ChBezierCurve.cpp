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
// Implementation of the classes that implement a Bezier 3D path.
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

#include <algorithm>
#include <iostream>
#include <sstream>
#include <fstream>

#include "chrono/core/ChBezierCurve.h"
#include "chrono/core/ChMathematics.h"

namespace chrono {

// -----------------------------------------------------------------------------
// Initialize static members
// -----------------------------------------------------------------------------
const size_t ChBezierCurve::m_maxNumIters = 50;
const double ChBezierCurve::m_sqrDistTol = 1e-4;
const double ChBezierCurve::m_cosAngleTol = 1e-4;
const double ChBezierCurve::m_paramTol = 1e-4;

// -----------------------------------------------------------------------------
// ChBezierCurve::ChBezierCurve()
//
// Constructors for a piecewise cubic Bezier curve.  The first version uses the
// specified nodes and control polygon vertices.  The second version evaluates
// the control polygon vertices so that the resulting curve is a piecewise cubic
// spline.
// -----------------------------------------------------------------------------
ChBezierCurve::ChBezierCurve(const std::vector<ChVector<> >& points,
                             const std::vector<ChVector<> >& inCV,
                             const std::vector<ChVector<> >& outCV)
    : m_points(points), m_inCV(inCV), m_outCV(outCV) {
    assert(points.size() > 1);
    assert(points.size() == inCV.size());
    assert(points.size() == outCV.size());
}

ChBezierCurve::ChBezierCurve(const std::vector<ChVector<> >& points) : m_points(points) {
    size_t numPoints = points.size();
    assert(numPoints > 1);

    m_inCV.resize(numPoints);
    m_outCV.resize(numPoints);

    m_inCV[0] = points[0];
    m_outCV[numPoints - 1] = points[numPoints - 1];

    // Special case for two points only.  In this case, the curve should be a
    // straight line.
    if (numPoints == 2) {
        m_outCV[0] = (2.0 * points[0] + points[1]) / 3.0;
        m_inCV[1] = (points[0] + 2.0 * points[1]) / 3.0;
        return;
    }

    // Calculate coordinates of the outCV control points.
    size_t n = numPoints - 1;
    double* rhs = new double[n];
    double* x = new double[n];
    double* y = new double[n];
    double* z = new double[n];

    // X coordinates.
    for (size_t i = 1; i < n - 1; ++i)
        rhs[i] = 4 * points[i].x() + 2 * points[i + 1].x();
    rhs[0] = points[0].x() + 2 * points[1].x();
    rhs[n - 1] = (8 * points[n - 1].x() + points[n].x()) / 2;
    solveTriDiag(n, rhs, x);

    // Y coordinates.
    for (size_t i = 1; i < n - 1; ++i)
        rhs[i] = 4 * points[i].y() + 2 * points[i + 1].y();
    rhs[0] = points[0].y() + 2 * points[1].y();
    rhs[n - 1] = (8 * points[n - 1].y() + points[n].y()) / 2;
    solveTriDiag(n, rhs, y);

    // Z coordinates.
    for (size_t i = 1; i < n - 1; ++i)
        rhs[i] = 4 * points[i].z() + 2 * points[i + 1].z();
    rhs[0] = points[0].z() + 2 * points[1].z();
    rhs[n - 1] = (8 * points[n - 1].z() + points[n].z()) / 2;
    solveTriDiag(n, rhs, z);

    // Set control points outCV and inCV.
    for (size_t i = 0; i < n - 1; i++) {
        m_outCV[i] = ChVector<>(x[i], y[i], z[i]);
        m_inCV[i + 1] =
            ChVector<>(2 * points[i + 1].x() - x[i + 1], 2 * points[i + 1].y() - y[i + 1], 2 * points[i + 1].z() - z[i + 1]);
    }
    m_outCV[n - 1] = ChVector<>(x[n - 1], y[n - 1], z[n - 1]);
    m_inCV[n] = ChVector<>((points[n].x() + x[n - 1]) / 2, (points[n].y() + y[n - 1]) / 2, (points[n].z() + z[n - 1]) / 2);

    // Cleanup.
    delete[] rhs;
    delete[] x;
    delete[] y;
    delete[] z;
}

void ChBezierCurve::setPoints(const std::vector<ChVector<> >& points,
                              const std::vector<ChVector<> >& inCV,
                              const std::vector<ChVector<> >& outCV) {
    assert(points.size() > 1);
    assert(points.size() == inCV.size());
    assert(points.size() == outCV.size());
    m_points = points;
    m_inCV = inCV;
    m_outCV = outCV;
}

// Utility function for solving the tridiagonal system for one of the
// coordinates (x, y, or z) of the outCV control points.
void ChBezierCurve::solveTriDiag(size_t n, double* rhs, double* x) {
    double* tmp = new double[n];

    double b = 2.0;
    x[0] = rhs[0] / b;

    // Decomposition and forward substitution.
    for (size_t i = 1; i < n; i++) {
        tmp[i] = 1 / b;
        b = (i < n - 1 ? 4.0 : 3.5) - tmp[i];
        x[i] = (rhs[i] - x[i - 1]) / b;
    }

    // Backsubstitution.
    for (size_t i = 1; i < n; i++)
        x[n - i - 1] -= tmp[n - i] * x[n - i];

    delete[] tmp;
}

// -----------------------------------------------------------------------------
// ChBezierCurve::read()
//
// This function creates and returns a pointer to a ChBezierCurve using data in
// the file with specified name. The input file is assumed to contain on the
// first line the number of data points and the number of data columns.  The
// latter can be one of 3 or 9.
// In the first case, subsequent lines should contain the coordinates of the
// curve knots (one point per line). The returned Bezier curve is a piecewise
// cubic spline through the specified points.
// In the second case, subsequent lines should contain the coordinates of the
// curve knot, the coordinates of the "incoming" control point, and the
// coordinates of the "outgoing" control point (i.e. 9 values per line). The
// returned curve is a general Bezier curve using the specified knots and
// control polygons.
// -----------------------------------------------------------------------------
ChBezierCurve* ChBezierCurve::read(const std::string& filename) {
    // Open input file stream
    std::ifstream ifile;
    std::string line;
    try {
        ifile.exceptions(std::ios::failbit | std::ios::badbit | std::ios::eofbit);
        ifile.open(filename.c_str());
    } catch (std::exception) {
        throw ChException("Cannot open input file");
    }

    // Read number of knots and type of curve
    size_t numPoints;
    size_t numCols;

    std::getline(ifile, line);
    std::istringstream iss(line);
    iss >> numPoints >> numCols;

    if (numCols == 3) {
        // Read knots from the following numPoints lines
        std::vector<ChVector<> > points;

        for (size_t i = 0; i < numPoints; i++) {
            double x, y, z;

            std::getline(ifile, line);
            std::istringstream iss(line);
            iss >> x >> y >> z;

            points.push_back(ChVector<>(x, y, z));
        }

        ifile.close();
        return new ChBezierCurve(points);
    }

    if (numCols == 9) {
        // Read knots and control points from the following numPoints lines
        std::vector<ChVector<> > points;
        std::vector<ChVector<> > inCV;
        std::vector<ChVector<> > outCV;

        for (size_t i = 0; i < numPoints; i++) {
            double x, y, z;
            double inX, inY, inZ;
            double outX, outY, outZ;

            std::getline(ifile, line);
            std::istringstream iss(line);
            iss >> x >> y >> z >> inX >> inY >> inZ >> outX >> outY >> outZ;

            points.push_back(ChVector<>(x, y, z));
            inCV.push_back(ChVector<>(inX, inY, inZ));
            outCV.push_back(ChVector<>(outX, outY, outZ));
        }

        ifile.close();
        return new ChBezierCurve(points, inCV, outCV);
    }

    // Not the expected number of columns.  Close the file and throw an exception.
    ifile.close();
    throw ChException("Invalid input file");
}

// -----------------------------------------------------------------------------
// ChBezierCurve::write()
//
// Utility function for writing this Bezier curve to a file with the specified
// name.  The format of the output file corresponds to that expected by the
// function ChBezierCurve::read().
// -----------------------------------------------------------------------------
void ChBezierCurve::write(const std::string& filename) {
    // Open output file stream
    std::ofstream ofile(filename.c_str());

    // Write number of points. Note that we always write the control points.
    size_t numPoints = m_points.size();
    ofile << numPoints << "  9\n";

    // Write points and control polygone vertices
    for (size_t i = 0; i < numPoints; i++) {
        ofile << m_points[i].x() << "  " << m_points[i].y() << "  " << m_points[i].z() << "     ";
        ofile << m_inCV[i].x() << "  " << m_inCV[i].y() << "  " << m_inCV[i].z() << "     ";
        ofile << m_outCV[i].x() << "  " << m_outCV[i].y() << "  " << m_outCV[i].z() << "\n";
    }

    ofile.close();
}

// -----------------------------------------------------------------------------
// ChBezierCurve::eval()
// ChBezierCurve::evalD()
// ChBezierCurve::evalDD()
//
// These functions evaluate the value and derivatives, respectively, of this
// Bezier curve at the specified value in the specified interval. We use the
// Bernstein polynomial representation of a Bezier curve. The first function
// returns the point on the curve; the second function returns the tangent
// vector.
// -----------------------------------------------------------------------------
ChVector<> ChBezierCurve::eval(size_t i, double t) const {
    assert(i >= 0 && i < getNumPoints() - 1);

    double omt = 1 - t;
    double t2 = t * t;
    double omt2 = omt * omt;

    double B0 = omt * omt2;
    double B1 = 3 * t * omt2;
    double B2 = 3 * t2 * omt;
    double B3 = t * t2;

    return B0 * m_points[i] + B1 * m_outCV[i] + B2 * m_inCV[i + 1] + B3 * m_points[i + 1];
}

ChVector<> ChBezierCurve::evalD(size_t i, double t) const {
    assert(i >= 0 && i < getNumPoints() - 1);

    double omt = 1 - t;
    double t2 = t * t;
    double omt2 = omt * omt;

    double B0 = -3 * omt2;
    double B1 = 3 * omt2 - 6 * t * omt;
    double B2 = 6 * t * omt - 3 * t2;
    double B3 = 3 * t2;

    return B0 * m_points[i] + B1 * m_outCV[i] + B2 * m_inCV[i + 1] + B3 * m_points[i + 1];
}

ChVector<> ChBezierCurve::evalDD(size_t i, double t) const {
    assert(i >= 0 && i < getNumPoints() - 1);

    double omt = 1 - t;

    double B0 = 6 * omt;
    double B1 = -12 * omt + 6 * t;
    double B2 = 6 * omt - 12 * t;
    double B3 = 6 * t;

    return B0 * m_points[i] + B1 * m_outCV[i] + B2 * m_inCV[i + 1] + B3 * m_points[i + 1];
}

// -----------------------------------------------------------------------------
// ChBezierCurve::calcClosestPoint()
//
// This function calculates and returns the closest point in the specified
// interval of this curve to the specified location. On input, the value 't' is
// an initial guess. On return, it contains the curve parameter corresponding
// to the closest point.
//
// The algorithm uses Newton iterations to find the point Q on the curve such
// that (Q-P) is perpendicular to Q', the tangent to the curve at Q. It uses the
// following stopping criteria:
//  - maximum number of iterations
//  - point coincidence: Q == P;
//  - orthogonality: angle between (Q - P) and Q' close to 90;
//  - curve parameter out of the (0,1) range;
//  - no significant change in the curve parameter (along the Q' direction).
// -----------------------------------------------------------------------------
ChVector<> ChBezierCurve::calcClosestPoint(const ChVector<>& loc, size_t i, double& t) const {
    ChVector<> Q = eval(i, t);
    ChVector<> Qd;
    ChVector<> Qdd;

    for (size_t j = 0; j < m_maxNumIters; j++) {
        ChVector<> vec = Q - loc;
        double d2 = vec.Length2();

        if (d2 < m_sqrDistTol)
            break;

        Qd = evalD(i, t);

        double dot = Vdot(vec, Qd);
        double cosAngle = dot / (vec.Length() * Qd.Length());

        if (fabs(cosAngle) < m_cosAngleTol)
            break;

        Qdd = evalDD(i, t);

        double dt = dot / (Vdot(vec, Qdd) + Qd.Length2());

        t -= dt;

        if (t < m_paramTol || t > 1 - m_paramTol) {
            ChClampValue(t, 0.0, 1.0);
            Q = eval(i, t);
            break;
        }

        Q = eval(i, t);

        if ((dt * Qd).Length2() < m_sqrDistTol)
            break;
    };

    return Q;
}

// -----------------------------------------------------------------------------
// ChBezierCurveTracker::reset()
//
// This function reinitializes the pathTracker at the specified location. It
// calculates an appropriate initial guess for the curve segment and sets the
// curve parameter to 0.5.
// -----------------------------------------------------------------------------
class PointSpec {
  public:
    PointSpec() {}
    PointSpec(size_t index, double dist2) : m_index(index), m_dist2(dist2) {}

    size_t m_index;
    double m_dist2;
};

static bool comparePoints(const PointSpec& p1, const PointSpec& p2) {
    return p1.m_dist2 < p2.m_dist2;
}

void ChBezierCurveTracker::reset(const ChVector<>& loc) {
    // Walk all curve points and calculate the distance to the specified reset
    // location, then sort them in increasing order.
    std::vector<PointSpec> points;

    for (size_t i = 0; i < m_path->getNumPoints(); i++)
        points.push_back(PointSpec(i, (loc - m_path->m_points[i]).Length2()));

    std::sort(points.begin(), points.end(), comparePoints);

    // Set the initial guess to be at t=0.5 in either the interval starting at
    // the point with minimum distance or in the previous interval.
    m_curParam = 0.5f;
    m_curInterval = points[0].m_index;

    if (m_curInterval == 0)
        return;

    if (m_curInterval == m_path->getNumPoints() - 1) {
        m_curInterval--;
        return;
    }

    ChVector<> loc2cur = m_path->m_points[m_curInterval] - loc;
    ChVector<> loc2prev = m_path->m_points[m_curInterval - 1] - loc;

    if (Vdot(loc2cur, loc2prev) < 0)
        m_curInterval--;
}

// -----------------------------------------------------------------------------
// ChBezierCurveTracker::calcClosestPoint()
//
// This function returns the closest point on the underlying path to the
// specified location. The return value is -1 if this point coincides with the
// first point of the path, +1 if it coincides with the last point of the path,
// and 0 otherwise. Note that, in order to provide a reasonable initial guess
// for the Newton iteration, we use time coherence (by keeping track of the path
// interval and curve parameter within that interval from the last query). As
// such, this function should be called with a continuous sequence of locations.
//
// The algorithm is as follows:
//  - find the closest point in the current interval of the Bezier curve to the
//    specified location;
//  - stop if the curve parameter is in (0, 1);
//  - if the curve parameter is close to 0, check the previous interval, unless
//    at the previous iteration the parameter was close to 1;
//  - if the curve parameter is close to 1, check the next interval, unless at
//    the previous iteration the parameter was close to 0.
// -----------------------------------------------------------------------------
int ChBezierCurveTracker::calcClosestPoint(const ChVector<>& loc, ChVector<>& point) {
    bool lastAtMin = false;
    bool lastAtMax = false;

    while (true) {
        point = m_path->calcClosestPoint(loc, m_curInterval, m_curParam);

        if (m_curParam < ChBezierCurve::m_paramTol) {
            if ((m_curInterval == 0) && (!m_isClosedPath))
                return -1;

            if (lastAtMax)
                return 0;

            // If the search region is at the beginning of the interval check the 
            // previous interval.  Loop to the last interval if the path is a 
            // closed loop and is it is currently in the first interval
            if ((m_curInterval == 0) && (m_isClosedPath))
                m_curInterval = m_path->getNumPoints() - 2; 
            else
                m_curInterval--;

            lastAtMin = true;
            m_curParam = 1;
        } else if (m_curParam > 1 - ChBezierCurve::m_paramTol) {
            if ((m_curInterval == m_path->getNumPoints() - 2) && (!m_isClosedPath))
                return +1;

            if (lastAtMin)
                return 0;

            // If the search region is at the end of the interval check the 
            // next interval.  Loop to the first interval if the path is a 
            // closed loop and is it is currently in the last interval
            if ((m_curInterval == m_path->getNumPoints() - 2) && (m_isClosedPath))
                m_curInterval = 0;
            else
                m_curInterval++;

            lastAtMax = true;
            m_curParam = 0;
        } else
            return 0;
    }
}


// -----------------------------------------------------------------------------
// ChBezierCurveTracker::setIsClosedPath()
//
// This function sets how the end points of the curve are treated by the
// tracker.  With an open path, the tracker will not loop back to check the 
// start of the curve.  With a closed loop path, it will loop back
// -----------------------------------------------------------------------------

void ChBezierCurveTracker::setIsClosedPath(bool isClosedPath){
    m_isClosedPath = isClosedPath;
}

}  // end of namespace chrono
