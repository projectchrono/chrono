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
#include <fstream>
#include <iostream>
#include <sstream>

#include "chrono/core/ChBezierCurve.h"

#include "chrono/core/ChMathematics.h"
#include "chrono/core/ChMatrix.h"

namespace chrono {

// -----------------------------------------------------------------------------
// Initialize static members
// -----------------------------------------------------------------------------
const size_t ChBezierCurve::m_maxNumIters = 50;
const double ChBezierCurve::m_sqrDistTol = 1e-6;
const double ChBezierCurve::m_cosAngleTol = 1e-4;
const double ChBezierCurve::m_paramTol = 1e-5;

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
                             const std::vector<ChVector<> >& outCV,
                             bool closed)
    : m_points(points), m_inCV(inCV), m_outCV(outCV), m_closed(closed) {
    assert(m_points.size() > 1);
    assert(inCV.size() == m_points.size());
    assert(outCV.size() == m_points.size());

    if (m_closed) {
        auto d2 = (m_points.back() - m_points.front()).Length2();
        if (d2 < m_sqrDistTol) {
            assert((m_inCV.back() - m_inCV.front()).Length2() < m_sqrDistTol);
            assert((m_outCV.back() - m_outCV.front()).Length2() < m_sqrDistTol);
        } else {
            m_points.push_back(m_points.front());
            m_inCV.push_back(m_inCV.front());
            m_outCV.push_back(m_outCV.front());
        }
    }
}

ChBezierCurve::ChBezierCurve(const std::vector<ChVector<> >& points, bool closed) : m_points(points), m_closed(closed) {
    int np = (int)m_points.size();  // number of points
    assert(np > 1);

    if (m_closed) {
        assert(np > 2);
        auto d2 = (m_points.back() - m_points.front()).Length2();
        ////std::cout << "Closed path last segment length: " << d2 << std::endl;
        if (d2 > m_sqrDistTol) {
            ////std::cout << "Add new point for closed loop" << std::endl;
            m_points.push_back(m_points.front());
            np++;
        }
    }

    int n = np - 1;  // number of intervals

    m_inCV.resize(np);
    m_outCV.resize(np);

    m_inCV[0] = m_points[0];
    m_outCV[n] = m_points[n];

    // Special case for two points only.  In this case, the curve should be a straight line.
    if (np == 2) {
        m_outCV[0] = (2.0 * m_points[0] + m_points[1]) / 3;
        m_inCV[1] = (2.0 * m_points[1] + m_points[0]) / 3;
        m_closed = false;
        return;
    }

    // Calculate coordinates of the outCV control points.

    if (m_closed) {
        // For a closed curve, impose that the slope and curvature matches.
        // This results in a linear system with an "almost" triangular matrix (solve with Eigen).

        typedef Eigen::Triplet<double> T;
        typedef Eigen::SparseMatrix<double> Mat;
        typedef Eigen::SparseLU<Mat> Slv;

        ChVectorDynamic<> rhs_x(n);
        ChVectorDynamic<> rhs_y(n);
        ChVectorDynamic<> rhs_z(n);        
        ChVectorDynamic<> x(n);
        ChVectorDynamic<> y(n);
        ChVectorDynamic<> z(n);

        std::vector<T> triplets;
        for (int i = 1; i < n - 1; i++) {
            triplets.push_back(T(i, i-1, 1.0));
            triplets.push_back(T(i, i, 4.0));
            triplets.push_back(T(i, i+1, 1.0));
            rhs_x[i] = 4 * m_points[i].x() + 2 * m_points[i + 1].x();
            rhs_y[i] = 4 * m_points[i].y() + 2 * m_points[i + 1].y();
            rhs_z[i] = 4 * m_points[i].z() + 2 * m_points[i + 1].z();
        }

        triplets.push_back(T(0, 0, 1.0));
        triplets.push_back(T(0, n - 2, 1.0));
        triplets.push_back(T(0, n - 1, 4.0));
        rhs_x[0] = m_points[0].x() + 4 * m_points[n - 1].x() + m_points[n].x();
        rhs_y[0] = m_points[0].y() + 4 * m_points[n - 1].y() + m_points[n].y();
        rhs_z[0] = m_points[0].z() + 4 * m_points[n - 1].z() + m_points[n].z();

        triplets.push_back(T(n - 1, 0, 4.0));
        triplets.push_back(T(n - 1, 1, 1.0));
        triplets.push_back(T(n - 1, n - 1, 1.0));
        rhs_x[n - 1] = 3 * m_points[0].x() + 2 * m_points[1].x() + m_points[n].x();
        rhs_y[n - 1] = 3 * m_points[0].y() + 2 * m_points[1].y() + m_points[n].y();
        rhs_z[n - 1] = 3 * m_points[0].z() + 2 * m_points[1].z() + m_points[n].z();

        Mat A(n, n);
        A.setFromTriplets(triplets.begin(), triplets.end());

        ////std::cout << ChMatrixDynamic<double>(A) << std::endl;

        Slv solver;
        solver.analyzePattern(A);
        solver.factorize(A);
        x = solver.solve(rhs_x);
        y = solver.solve(rhs_y);
        z = solver.solve(rhs_z);

        ////std::cout << (A * x - rhs_x).norm() << std::endl;
        ////std::cout << (A * y - rhs_y).norm() << std::endl;
        ////std::cout << (A * z - rhs_z).norm() << std::endl;

        // Set control points outCV and inCV.
        for (size_t i = 0; i < n; i++)
            m_outCV[i] = ChVector<>(x[i], y[i], z[i]);

        for (size_t i = 1; i < n; i++)
            m_inCV[i] = 2.0 * m_points[i] - m_outCV[i];
        m_inCV[n] = m_points[n] + m_points[0] - m_outCV[0];

        ////std::cout << "slope diff: " << (m_outCV[0] - m_points[0]) - (m_points[n] - m_inCV[n]) << std::endl;
        ////std::cout << "curv diff: " << (m_inCV[1] - 2.0 * m_outCV[0] + m_points[0]) - (m_points[n] - 2.0 * m_inCV[n] + m_outCV[n-1]) << std::endl;

    } else {
        double* rhs_x = new double[n];
        double* rhs_y = new double[n];
        double* rhs_z = new double[n];
        double* x = new double[n];
        double* y = new double[n];
        double* z = new double[n];

        // For an open curve, impose natural BC (zero curvature at ends).
        // This results in a linear system with a tri-diagonal matrix (solve with custom function).
        for (size_t i = 1; i < n - 1; ++i) {
            rhs_x[i] = 4 * m_points[i].x() + 2 * m_points[i + 1].x();
            rhs_y[i] = 4 * m_points[i].y() + 2 * m_points[i + 1].y();
            rhs_z[i] = 4 * m_points[i].z() + 2 * m_points[i + 1].z();
        }

        rhs_x[0] = m_points[0].x() + 2 * m_points[1].x();
        rhs_y[0] = m_points[0].y() + 2 * m_points[1].y();
        rhs_z[0] = m_points[0].z() + 2 * m_points[1].z();
        
        rhs_x[n - 1] = (8 * m_points[n - 1].x() + m_points[n].x()) / 2;
        rhs_y[n - 1] = (8 * m_points[n - 1].y() + m_points[n].y()) / 2;
        rhs_z[n - 1] = (8 * m_points[n - 1].z() + m_points[n].z()) / 2;
        
        solveTriDiag(n, rhs_x, x);
        solveTriDiag(n, rhs_y, y);
        solveTriDiag(n, rhs_z, z);

        // Set control points outCV and inCV.
        for (size_t i = 0; i < n; i++)
            m_outCV[i] = ChVector<>(x[i], y[i], z[i]);

        for (size_t i = 1; i < n; i++)
            m_inCV[i] = 2.0 * m_points[i] - m_outCV[i];
        m_inCV[n] = (m_outCV[n - 1] + m_points[n]) / 2;

        // Cleanup.
        delete[] rhs_x;
        delete[] rhs_y;
        delete[] rhs_z;
        delete[] x;
        delete[] y;
        delete[] z;
    }
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
std::shared_ptr<ChBezierCurve> ChBezierCurve::read(const std::string& filename, bool closed) {
    // Open input file stream
    std::ifstream ifile;
    std::string line;
    try {
        ifile.exceptions(std::ios::failbit | std::ios::badbit | std::ios::eofbit);
        ifile.open(filename.c_str());
    } catch (const std::exception &) {
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
            std::istringstream jss(line);
            jss >> x >> y >> z;

            points.push_back(ChVector<>(x, y, z));
        }

        ifile.close();
        return std::shared_ptr<ChBezierCurve>(new ChBezierCurve(points, closed));
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
            std::istringstream jss(line);
            jss >> x >> y >> z >> inX >> inY >> inZ >> outX >> outY >> outZ;

            points.push_back(ChVector<>(x, y, z));
            inCV.push_back(ChVector<>(inX, inY, inZ));
            outCV.push_back(ChVector<>(outX, outY, outZ));
        }

        ifile.close();
        return std::shared_ptr<ChBezierCurve>(new ChBezierCurve(points, inCV, outCV, closed));
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

    // Write points and control polygon vertices
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
// ChBezierCurve::eval()
//
// This function evaluates the value of this Bezier curve at the specified value.
// A value t=0 returns the first point of the Bezier curve.
// A value t=1 returns the last point of the Bezier curve.
// -----------------------------------------------------------------------------
ChVector<> ChBezierCurve::eval(double t) const {
    double par = ChClamp(t, 0.0, 1.0);
    size_t numIntervals = getNumPoints() - 1;
    double epar = par * numIntervals;
    size_t i = static_cast<size_t>(std::floor(par * numIntervals));
    ChClampValue(i, size_t(0), numIntervals - 1);

    return eval(i, epar - (double)i);
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
    // Bracket location of projection
    int m_numEvals = 20;
    double dt = 1.0 / m_numEvals;
    int min_idx = -1;
    double d2_min = std::numeric_limits<double>::max();
    for (int j = 0; j <= m_numEvals; j++) {
        double d2 = (eval(i, j * dt) - loc).Length2();
        if (d2 < d2_min) {
            min_idx = j;
            d2_min = d2;
        }
    }

    // Bisection
    int count = m_numEvals + 1;
    double t0 = std::max((min_idx - 1) * dt, 0.0);
    double t1 = std::min((min_idx + 1) * dt, 1.0);
    while (t1 - t0 > m_paramTol) {
        t = (t0 + t1) / 2;
        double d2_0 = (eval(i, t - m_paramTol) - loc).Length2();
        double d2_1 = (eval(i, t + m_paramTol) - loc).Length2();
        if (d2_0 < d2_1)
            t1 = t;
        else
            t0 = t;
        count += 2;
    }

    ////std::cout << "num. evaluations: " << count << std::endl;

    return eval(i, t);

    /*
    // Newton method
    ChVector<> Q = eval(i, t);
    ChVector<> Qd;
    ChVector<> Qdd;

    size_t j = 0;
    for (j = 0; j < m_maxNumIters; j++) {
        ChVector<> vec = Q - loc;
        double d2 = vec.Length2();

        if (d2 < m_sqrDistTol)
            break;

        Qd = evalD(i, t);

        double dot = Vdot(vec, Qd);
        double cosAngle = dot / (std::sqrt(d2) * Qd.Length());

        if (fabs(cosAngle) < m_cosAngleTol)
            break;

        Qdd = evalDD(i, t);

        double dt = dot / (Vdot(vec, Qdd) + Qd.Length2());

        t -= dt;

        Q = eval(i, t);

        if ((dt * Qd).Length2() < m_sqrDistTol)
            break;
    }

    ////std::cout << "num iterations: " << j << "   max: " << m_maxNumIters << std::endl;

    if (t < m_paramTol || t > 1 - m_paramTol) {
        ChClampValue(t, 0.0, 1.0);
        Q = eval(i, t);
    } else {
        ChVector<> Q_0 = eval(i, 0.0);
        ChVector<> Q_1 = eval(i, 1.0);

        double d2 = (Q - loc).Length2();
        double d2_0 = (Q_0 - loc).Length2();
        double d2_1 = (Q_1 - loc).Length2();

        if (d2_0 < d2) {
            t = 0;
            Q = Q_0;
            d2 = d2_0;
        }
        if (d2_1 < d2) {
            t = 1;
            Q = Q_1;
        }
    }

    return Q;
    */
}

// -----------------------------------------------------------------------------

void ChBezierCurve::ArchiveOut(ChArchiveOut& marchive)
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

void ChBezierCurve::ArchiveIn(ChArchiveIn& marchive)
{
    // version number
    /*int version =*/ marchive.VersionRead<ChBezierCurve>();

    // stream in all member data:
    marchive >> CHNVP(m_points);
    marchive >> CHNVP(m_inCV);
    marchive >> CHNVP(m_outCV);
    marchive >> CHNVP(m_maxNumIters);
    marchive >> CHNVP(m_sqrDistTol);
    marchive >> CHNVP(m_cosAngleTol);
    marchive >> CHNVP(m_paramTol);
}

// -----------------------------------------------------------------------------

ChBezierCurveTracker::ChBezierCurveTracker(std::shared_ptr<ChBezierCurve> path)
    : m_path(path), m_curInterval(0), m_curParam(0) {}

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
    // Evaluate in current interval
    point = m_path->calcClosestPoint(loc, m_curInterval, m_curParam);

    if (m_curParam < ChBezierCurve::m_paramTol) {
        // Close to lower limit. Consider previous interval
        size_t prevInterval = m_curInterval - 1;
        if (m_curInterval == 0) {
            if (m_path->IsClosed())
                prevInterval = m_path->getNumPoints() - 2;
            else
                return -1;
        }

        // Check previous interval
        double p_m;
        auto pt_m = m_path->calcClosestPoint(loc, prevInterval, p_m);

        if ((pt_m - loc).Length2() < (point - loc).Length2()) {
            ////std::cout << "loc = " << loc << "  DECREASE to " << m_curInterval - 1 << "  p = " << p_m
            ////          << "    point: " << point << "  point minus: " << pt_m << std::endl;
            m_curInterval = prevInterval;
            m_curParam = p_m;
            point = pt_m;
        }

        return 0;
    } else if (m_curParam > 1 - ChBezierCurve::m_paramTol) {
        // Close to upper limit. Consider next interval
        size_t nextInterval = m_curInterval + 1;
        if (m_curInterval == m_path->getNumPoints() - 2) {
            if (m_path->IsClosed())
                nextInterval = 0;
            else
                return +1;
        }

        // Check next interval
        double p_p;
        auto pt_p = m_path->calcClosestPoint(loc, m_curInterval + 1, p_p);

        if ((pt_p - loc).Length2() < (point - loc).Length2()) {
            ////std::cout << "loc = " << loc << "  INCREASE to " << m_curInterval + 1 << "  p = " << p_p
            ////          << "    point: " << point << "  point plus: " << pt_p << std::endl;
            m_curInterval = nextInterval;
            m_curParam = p_p;
            point = pt_p;
        }

        return 0;
    } else {
        // Not close to interval bounds. Done
        return 0;
    }

    /*
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
    */
}

int ChBezierCurveTracker::calcClosestPoint(const ChVector<>& loc, ChFrame<>& tnb, double& curvature) {
    // Find closest point to specified location
    ChVector<> r;
    int flag = calcClosestPoint(loc, r);

    // Find 1st and 2nd order derivative vectors at the closest point
    ChVector<> rp = m_path->evalD(m_curInterval, m_curParam);
    ChVector<> rpp = m_path->evalDD(m_curInterval, m_curParam);

    // Calculate TNB frame
    ChVector<> rp_rpp = Vcross(rp, rpp);
    double rp_norm = rp.Length();
    double rp_rpp_norm = rp_rpp.Length();

    ChVector<> T = rp / rp_norm;
    ChVector<> N;
    ChVector<> B;
    if (std::abs(rp_rpp_norm) > 1e-6) {
        N = Vcross(rp_rpp, rp) / (rp_norm * rp_rpp_norm);
        B = rp_rpp / rp_rpp_norm;
    } else {  // Zero curvature
        B = ChVector<>(0, 0, 1);
        N = Vcross(B, T);
        B = Vcross(T, N);
    }

    ChMatrix33<> A(T, N, B);

    tnb.SetRot(A);
    tnb.SetPos(r);


    // Calculate curvature
    curvature = rp_rpp_norm / (rp_norm * rp_norm * rp_norm);

    return flag;
}

}  // end of namespace chrono
