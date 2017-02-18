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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHC_TRI_H
#define CHC_TRI_H

#include <cmath>

#include "chrono/geometry/ChGeometry.h"

namespace chrono {
namespace geometry {

/// A triangle geometric shape for collisions and visualization.
class ChApi ChTriangle : public ChGeometry {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChTriangle)

  public:
    ChVector<> p1;  ///< first triangle vertex
    ChVector<> p2;  ///< second triangle vertex
    ChVector<> p3;  ///< third triangle vertex

  public:
    ChTriangle() : p1(VNULL), p2(VNULL), p3(VNULL) {}
    ChTriangle(const ChVector<>& mp1, const ChVector<>& mp2, const ChVector<>& mp3) : p1(mp1), p2(mp2), p3(mp3) {}
    ChTriangle(const ChTriangle& source);
    ~ChTriangle() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChTriangle* Clone() const override { return new ChTriangle(*this); }

    /// Assignment operator: copy from another triangle
    ChTriangle& operator=(const ChTriangle& source);

    virtual GeometryType GetClassType() const override { return TRIANGLE; }

    virtual void GetBoundingBox(double& xmin,
                                double& xmax,
                                double& ymin,
                                double& ymax,
                                double& zmin,
                                double& zmax,
                                ChMatrix33<>* Rot = NULL) const override;

    virtual ChVector<> Baricenter() const override;

    virtual void CovarianceMatrix(ChMatrix33<>& C) const override;

    /// This is a surface
    virtual int GetManifoldDimension() const override { return 2; }

    // return false if triangle has almost zero area
    bool IsDegenerated() const;

    // compute triangle normal
    bool Normal(ChVector<>& N) const;
    ChVector<> GetNormal() const;

    /// Given point B and a generic triangle, computes the distance from the triangle plane,
    /// returning also the projection of point on the plane and other infos
    ///			\return the signed distance
    static double PointTriangleDistance(ChVector<> B,           ///< point to be measured
                                        ChVector<>& A1,         ///< point of triangle
                                        ChVector<>& A2,         ///< point of triangle
                                        ChVector<>& A3,         ///< point of triangle
                                        double& mu,             ///< returns U parametric coord of projection
                                        double& mv,             ///< returns V parametric coord of projection
                                        bool& is_into,          ///< returns true if projection falls on the triangle
                                        ChVector<>& Bprojected  ///< returns the position of the projected point
                                        );

    /// Given point B, computes the distance from this triangle plane,
    /// returning also the projection of point on the plane and other infos
    ///			\return the signed distance
    double PointTriangleDistance(ChVector<> B,           ///< point to be measured
                                 double& mu,             ///< returns U parametric coord of projection
                                 double& mv,             ///< returns V parametric coord of projection
                                 bool& is_into,          ///< returns true if projection falls on the triangle
                                 ChVector<>& Bprojected  ///< returns the position of the projected point
                                 ) {
        return PointTriangleDistance(B, this->p1, this->p2, this->p3, mu, mv, is_into, Bprojected);
    }

    /// Calculate distance between a point p and a line identified
    /// with segment dA,dB. Returns distance. Also, the mu value reference
    /// tells if the nearest projection of point on line falls into segment (for mu 0...1)
    ///			\return the distance
    static double PointLineDistance(
        ChVector<>& p,      ///< point to be measured
        ChVector<>& dA,     ///< a point on the line
        ChVector<>& dB,     ///< another point on the line
        double& mu,         ///< parametric coord: if in 0..1 interval, projection is between dA and dB
        bool& is_insegment  ///< returns true if projected point is between dA and dB
        );

    virtual void ArchiveOUT(ChArchiveOut& marchive) override {
        // version number
        marchive.VersionWrite<ChTriangle>();
        // serialize parent class
        ChGeometry::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(p1);
        marchive << CHNVP(p2);
        marchive << CHNVP(p3);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override {
        // version number
        int version = marchive.VersionRead<ChTriangle>();
        // deserialize parent class
        ChGeometry::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(p1);
        marchive >> CHNVP(p2);
        marchive >> CHNVP(p3);
    }
};

}  // end namespace geometry

CH_CLASS_VERSION(geometry::ChTriangle,0)

}  // end namespace chrono

#endif
