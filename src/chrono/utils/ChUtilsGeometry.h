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
// Authors: Radu Serban, Hammad Mazhar, Arman Pazouki
// =============================================================================
//
// Utility functions for various geometrical calculations.
//
// =============================================================================

#ifndef CH_UTILS_GEOMETRY_H
#define CH_UTILS_GEOMETRY_H

#include <cmath>

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChVector.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChMathematics.h"

#include "chrono/collision/ChCCollisionModel.h"

namespace chrono {
namespace utils {

// -----------------------------------------------------------------------------
// These utility functions return the bounding radius of the corresponding
// shape. A sphere with this radius and centered at the origin of the frame
// defining the shape is a bounding sphere for that shape.
// -----------------------------------------------------------------------------
inline double CalcSphereBradius(double radius) {
    return radius;
}

inline double CalcEllipsoidBradius(const ChVector<>& hdims) {
    return hdims.LengthInf();
}

inline double CalcBoxBradius(const ChVector<>& hdims) {
    return hdims.Length();
}

inline double CalcCapsuleBradius(double radius, double hlen) {
    return hlen + radius;
}

inline double CalcCylinderBradius(double radius, double hlen) {
    return sqrt(hlen * hlen + radius * radius);
}

inline double CalcConeBradius(double radius, double hlen) {
    return sqrt(hlen * hlen + radius * radius);
}

inline double CalcRoundedCylinderBradius(double radius, double hlen, double srad) {
    return sqrt(hlen * hlen + radius * radius) + srad;
}

inline double CalcRoundedBoxBradius(const ChVector<>& hdims, double srad) {
    return hdims.Length() + srad;
}

inline double CalcTorusBradius(double radius, double thickness) {
    return radius + thickness;
}

// -----------------------------------------------------------------------------
// These utility functions calculate the volume of the corresponding shape.
// -----------------------------------------------------------------------------
inline double CalcSphereVolume(double radius) {
    return (4.0 / 3.0) * CH_C_PI * radius * radius * radius;
}

inline double CalcEllipsoidVolume(const ChVector<>& hdims) {
    return (4.0 / 3.0) * CH_C_PI * hdims.x() * hdims.y() * hdims.z();
}

inline double CalcBoxVolume(const ChVector<>& hdims) {
    return 8.0 * hdims.x() * hdims.y() * hdims.z();
}

inline double CalcBiSphereVolume(double radius, double cDist) {
	double delta = 2 * radius - cDist;
	double cos_theta = (radius - 0.5 * delta) / radius;
	return (4.0 / 3.0) * CH_C_PI * radius * radius * radius * (1 + cos_theta);
}

inline double CalcCapsuleVolume(double radius, double hlen) {
    double tmp = radius * radius * hlen + (2.0 / 3.0) * radius * radius * radius;
    return 2.0 * CH_C_PI * tmp;
}

inline double CalcCylinderVolume(double radius, double hlen) {
    return 2.0 * CH_C_PI * radius * radius * hlen;
}

inline double CalcConeVolume(double radius, double len) {
    return CH_C_PI * radius * radius * len / 3.0;
}

inline double CalcRoundedCylinderVolume(double radius, double hlen, double srad) {
    double tmp = (radius + srad) * (radius + srad) * hlen + srad * (radius * radius + (2.0 / 3.0) * srad * srad) +
                 (CH_C_PI / 2.0 - 1.0) * radius * srad * srad;
    return 2.0 * CH_C_PI * tmp;
}

inline double CalcRoundedBoxVolume(const ChVector<>& hdims, double srad) {
    return 8 * hdims.x() * hdims.y() * hdims.z() + 2 * srad * (hdims.x() * hdims.y() + hdims.y() * hdims.z() + hdims.z() * hdims.x()) +
           (4.0 * CH_C_PI / 3.0) * srad * srad * srad;
}

inline double CalcTorusVolume(double radius, double thickness) {
    return 2 * CH_C_PI * CH_C_PI * thickness * thickness * radius;
}

// -----------------------------------------------------------------------------
// This utility function transforms the specified gyration tensor, assumed to
// be specified in a centroidal reference frame, to the 'parent' frame defined
// by the translation vector 'pos' and orientation 'rot' of the local frame.
// -----------------------------------------------------------------------------
inline void TransformGyration(ChMatrix33<>& J, const ChVector<>& pos, const ChQuaternion<>& rot) {
    ////
    ////  TODO
    ////
}

// -----------------------------------------------------------------------------
// These utility functions calculate the gyration tensor of the corresponding
// shape, given its position and orientation.
// -----------------------------------------------------------------------------
inline ChMatrix33<> CalcSphereGyration(double radius,
                                       const ChVector<>& pos = ChVector<>(0, 0, 0),
                                       const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0)) {
    ChMatrix33<> J;
    double Jxx = (2.0 / 5.0) * radius * radius;

    J.SetElement(0, 0, Jxx);
    J.SetElement(1, 1, Jxx);
    J.SetElement(2, 2, Jxx);

    TransformGyration(J, pos, rot);

    return J;
}

inline ChMatrix33<> CalcEllipsoidGyration(const ChVector<>& hdims,
                                          const ChVector<>& pos = ChVector<>(0, 0, 0),
                                          const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0)) {
    ChMatrix33<> J;

    J.SetElement(0, 0, (1.0 / 5.0) * (hdims.y() * hdims.y() + hdims.z() * hdims.z()));
    J.SetElement(1, 1, (1.0 / 5.0) * (hdims.z() * hdims.z() + hdims.x() * hdims.x()));
    J.SetElement(2, 2, (1.0 / 5.0) * (hdims.x() * hdims.x() + hdims.y() * hdims.y()));

    TransformGyration(J, pos, rot);

    return J;
}

// Calculate the gyration tensor of a box. hdims is a vector of half dimensions of the box.
inline ChMatrix33<> CalcBoxGyration(const ChVector<>& hdims,
                                    const ChVector<>& pos = ChVector<>(0, 0, 0),
                                    const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0)) {
    ChMatrix33<> J;

    J.SetElement(0, 0, (1.0 / 3.0) * (hdims.y() * hdims.y() + hdims.z() * hdims.z()));
    J.SetElement(1, 1, (1.0 / 3.0) * (hdims.z() * hdims.z() + hdims.x() * hdims.x()));
    J.SetElement(2, 2, (1.0 / 3.0) * (hdims.x() * hdims.x() + hdims.y() * hdims.y()));

    TransformGyration(J, pos, rot);

    return J;
}

// Calculate the gyration tensor of a bisphere, which is two identical spheres attached to
// each other. delta is the overlap length and radius is the radius of each sphere
inline ChMatrix33<> CalcBiSphereGyration(double radius,
                                                double cDist,
                                                const ChVector<>& pos = ChVector<>(0, 0, 0),
                                                const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0)) {
	// TODO: simple implementation for now

	double delta = 2 * radius - cDist;
	double cos_theta = (radius - 0.5 * delta) / radius;
	double z_prim = radius - 0.5 * delta;

    ChMatrix33<> J;
	double comp1 =  0.4 * radius * radius * ( 1 + cos_theta );
	double comp2 = - 0.2 * radius * radius * ( 1./3. * ( - cos_theta * cos_theta * cos_theta - 1 ) + (1 + cos_theta));
	double comp3 = 2. / 3. * z_prim * z_prim * (1 + cos_theta);
	double comp4 = 0.5 * radius * z_prim * sqrt(1 - cos_theta * cos_theta);
	double numerator = 2 * (comp1 + comp2 + comp3 + comp4);
	double denominator = 4. / 3. * (1 + cos_theta);
	double Jxx = numerator / denominator;
	double Jyy = 0.6 * radius * radius * ( 1./3. * ( - cos_theta * cos_theta * cos_theta - 1 ) + ( 1 + cos_theta ) ) / (1 + cos_theta);

    J.SetElement(0, 0, Jxx);
    J.SetElement(1, 1, Jyy);
    J.SetElement(2, 2, Jxx);

    TransformGyration(J, pos, rot);

    return J;
}

// Calculate the gyration tensor of a capsule. hlen is the half length of the cylindrical part and radius
// is the radius of the spherical part
inline ChMatrix33<> CalcCapsuleGyration(double radius,
                                        double hlen,
                                        const ChVector<>& pos = ChVector<>(0, 0, 0),
                                        const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0)) {
    ChMatrix33<> J;

    double massRatio = 1.5 * hlen / radius;
    double cmDist = hlen + 3.0 / 8 * radius;
    double Ixx = massRatio / (1 + massRatio) * (1.0 / 12.0) * (3 * radius * radius + 4 * hlen * hlen) +
                 1 / (1 + massRatio) * (0.259 * radius * radius + cmDist * cmDist);
    double Iyy = massRatio / (1 + massRatio) * (1.0 / 2.0) * (radius * radius) +
                 1 / (1 + massRatio) * (2.0 / 5.0) * (radius * radius);

    J.SetElement(0, 0, Ixx);
    J.SetElement(1, 1, Iyy);
    J.SetElement(2, 2, Ixx);

    TransformGyration(J, pos, rot);

    return J;
}

// Calculate the gyration tensor of a cylinder. hlen is the half length of the cylindrical part and radius
// is the base radius
inline ChMatrix33<> CalcCylinderGyration(double radius,
                                         double hlen,
                                         const ChVector<>& pos = ChVector<>(0, 0, 0),
                                         const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0)) {
    ChMatrix33<> J;

    J.SetElement(0, 0, (1.0 / 12.0) * (3 * radius * radius + 4 * hlen * hlen));
    J.SetElement(1, 1, (1.0 / 2.0) * (radius * radius));
    J.SetElement(2, 2, (1.0 / 12.0) * (3 * radius * radius + 4 * hlen * hlen));

    TransformGyration(J, pos, rot);

    return J;
}

// Calculate the gyration tensor of a cone. len is the length of the cone axis and radius
// is the base radius
inline ChMatrix33<> CalcConeGyration(double radius,
                                     double len,
                                     const ChVector<>& pos = ChVector<>(0, 0, 0),
                                     const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0)) {
    ChMatrix33<> J;

    double Ixx = (3.0 / 80.0) * (len * len) + (3.0 / 20.0) * (radius * radius);
    J.SetElement(0, 0, Ixx);
    J.SetElement(1, 1, (3.0 / 10.0) * (radius * radius));
    J.SetElement(2, 2, Ixx);

    TransformGyration(J, pos, rot);

    return J;
}

inline ChMatrix33<> CalcRoundedCylinderGyration(double radius,
                                                double hlen,
                                                double srad,
                                                const ChVector<>& pos = ChVector<>(0, 0, 0),
                                                const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0)) {
    ChMatrix33<> J;

    double modifiedRadius = radius + srad;
    double modifiedHlen = hlen + srad;
    //// TODO: for now, use the gyration of the offset cylinder
    J.SetElement(0, 0, (1.0 / 12.0) * (3 * modifiedRadius * modifiedRadius + 4 * modifiedHlen * modifiedHlen));
    J.SetElement(1, 1, (1.0 / 2.0) * (modifiedRadius * modifiedRadius));
    J.SetElement(2, 2, (1.0 / 12.0) * (3 * modifiedRadius * modifiedRadius + 4 * modifiedHlen * modifiedHlen));

    TransformGyration(J, pos, rot);

    return J;
}

inline ChMatrix33<> CalcRoundedBoxGyration(const ChVector<>& hdims,
                                           double srad,
                                           const ChVector<>& pos = ChVector<>(0, 0, 0),
                                           const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0)) {
    ChMatrix33<> J;

    ChVector<> modifiedHdims = hdims + ChVector<>(srad, srad, srad);
    //// TODO: for now, use the gyration of the offset box
    J.SetElement(0, 0, (1.0 / 3.0) * (modifiedHdims.y() * modifiedHdims.y() + modifiedHdims.z() * modifiedHdims.z()));
    J.SetElement(1, 1, (1.0 / 3.0) * (modifiedHdims.z() * modifiedHdims.z() + modifiedHdims.x() * modifiedHdims.x()));
    J.SetElement(2, 2, (1.0 / 3.0) * (modifiedHdims.x() * modifiedHdims.x() + modifiedHdims.y() * modifiedHdims.y()));

    TransformGyration(J, pos, rot);

    return J;
}

inline ChMatrix33<> CalcTorusGyration(double radius,
                                      double thickness,
                                      const ChVector<>& pos = ChVector<>(0, 0, 0),
                                      const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0)) {
    ChMatrix33<> J;

    J.SetElement(0, 0, (5.0 / 8.0) * (thickness * thickness) + (1.0 / 2.0) * (radius * radius));
    J.SetElement(1, 1, (3.0 / 4.0) * (thickness * thickness) + (radius * radius));
    J.SetElement(2, 2, (5.0 / 8.0) * (thickness * thickness) + (1.0 / 2.0) * (radius * radius));

    TransformGyration(J, pos, rot);

    return J;
}

}  // end namespace utils
}  // end namespace chrono

#endif
