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
// Utility functions for various geometrical calculations.
//
// =============================================================================

#ifndef CH_UTILS_GEOMETRY_H
#define CH_UTILS_GEOMETRY_H

#include <cmath>

#include "core/ChSmartpointers.h"
#include "core/ChVector.h"
#include "core/ChQuaternion.h"
#include "core/ChMathematics.h"

#include "collision/ChCCollisionModel.h"

#include "utils/ChApiUtils.h"


namespace chrono {
namespace utils {


// -----------------------------------------------------------------------------
// These utility functions return the bounding radius of the corresponding
// shape. A sphere with this radius and centered at the origin of the frame
// defining the shape is a bounding sphere for that shape.
// -----------------------------------------------------------------------------
inline
double CalcSphereBradius(double radius)
{
  return radius;
}

inline
double CalcEllipsoidBradius(const ChVector<>& hdims)
{
  return hdims.LengthInf();
}

inline
double CalcBoxBradius(const ChVector<>& hdims)
{
  return hdims.Length();
}

inline
double CalcCapsuleBradius(double radius, double hlen)
{
  return hlen + radius;
}

inline
double CalcCylinderBradius(double radius, double hlen)
{
  return sqrt(hlen * hlen + radius * radius);
}

inline
double CalcRoundedCylinderBradius(double radius, double hlen, double srad)
{
  return sqrt(hlen * hlen + radius * radius) + srad;
}

inline
double CalcRoundedBoxBradius(const ChVector<>& hdims, double srad)
{
  return hdims.Length() + srad;
}


// -----------------------------------------------------------------------------
// These utility functions calculate the volume of the corresponding shape.
// -----------------------------------------------------------------------------
inline
double CalcSphereVolume(double radius)
{
  return (4.0/3.0) * CH_C_PI * radius * radius * radius;
}

inline
double CalcEllipsoidVolume(const ChVector<>& hdims)
{
  return (4.0/3.0) * CH_C_PI * hdims.x * hdims.y * hdims.z;
}

inline
double CalcBoxVolume(const ChVector<>& hdims)
{
  return 8.0 * hdims.x * hdims.y * hdims.z;
}

inline
double CalcCapsuleVolume(double radius, double hlen)
{
  double tmp = radius * radius * hlen + (2.0/3.0) * radius * radius * radius;
  return 2.0 * CH_C_PI * tmp;
}

inline
double CalcCylinderVolume(double radius, double hlen)
{
  return 2.0 * CH_C_PI * radius * radius * hlen;
}

inline
double CalcRoundedCylinderVolume(double radius, double hlen, double srad)
{
  double tmp = (radius + srad) * (radius + srad) * hlen +
         srad * (radius * radius + (2.0/3.0) * srad * srad) +
         (CH_C_PI/2.0 - 1.0) * radius * srad * srad;
  return 2.0 * CH_C_PI * tmp;
}

inline
double CalcRoundedBoxVolume(const ChVector<>& hdims, double srad)
{
  return  8 * hdims.x * hdims.y * hdims.z +
          2 * srad * (hdims.x * hdims.y + hdims.y * hdims.z + hdims.z * hdims.x) +
          (4.0 * CH_C_PI / 3.0) * srad * srad * srad;
}

// -----------------------------------------------------------------------------
// This utility function transforms the specified gyration tensor, assumed to
// be specified in a centroidal reference frame, to the 'parent' frame defined
// by the translation vector 'pos' and orientation 'rot' of the local frame.
// -----------------------------------------------------------------------------
inline
void TransformGyration(ChMatrix33<>&         J,
                       const ChVector<>&     pos,
                       const ChQuaternion<>& rot)
{
  ////
  ////  TODO
  ////
}

// -----------------------------------------------------------------------------
// These utility functions calculate the gyration tensor of the corresponding
// shape, given its position and orientation.
// -----------------------------------------------------------------------------
inline
ChMatrix33<> CalcSphereGyration(
                 double                radius,
                 const ChVector<>&     pos = ChVector<>(0,0,0),
                 const ChQuaternion<>& rot = ChQuaternion<>(1,0,0,0))
{
  ChMatrix33<> J;
  double       Jxx = (2.0/5.0) * radius * radius;

  J.SetElement(0, 0, Jxx);
  J.SetElement(1, 1, Jxx);
  J.SetElement(2, 2, Jxx);

  TransformGyration(J, pos, rot);

  return J;
}

inline
ChMatrix33<> CalcEllipsoidGyration(
                 const ChVector<>&     hdims,
                 const ChVector<>&     pos = ChVector<>(0,0,0),
                 const ChQuaternion<>& rot = ChQuaternion<>(1,0,0,0))
{
  ChMatrix33<> J;

  J.SetElement(0, 0, (1.0/5.0) * (hdims.y * hdims.y + hdims.z * hdims.z));
  J.SetElement(1, 1, (1.0/5.0) * (hdims.z * hdims.z + hdims.x * hdims.x));
  J.SetElement(2, 2, (1.0/5.0) * (hdims.x * hdims.x + hdims.y * hdims.y));

  TransformGyration(J, pos, rot);

  return J;
}

inline
ChMatrix33<> CalcBoxGyration(
                 const ChVector<>&     hdims,
                 const ChVector<>&     pos = ChVector<>(0,0,0),
                 const ChQuaternion<>& rot = ChQuaternion<>(1,0,0,0))
{
  ChMatrix33<> J;

  J.SetElement(0, 0, (1.0/12.0) * (hdims.y * hdims.y + hdims.z * hdims.z));
  J.SetElement(1, 1, (1.0/12.0) * (hdims.z * hdims.z + hdims.x * hdims.x));
  J.SetElement(2, 2, (1.0/12.0) * (hdims.x * hdims.x + hdims.y * hdims.y));

  TransformGyration(J, pos, rot);

  return J;
}

inline
ChMatrix33<> CalcCapsuleGyration(
                 double                radius,
                 double                hlen,
                 const ChVector<>&     pos = ChVector<>(0,0,0),
                 const ChQuaternion<>& rot = ChQuaternion<>(1,0,0,0))
{
  ChMatrix33<> J;

  //// TODO: for now, use the gyration of the circumscibed cylinder
  double hlen1 = hlen + radius;

  J.SetElement(0, 0, (1.0/12.0) * (3 * radius * radius + hlen1 * hlen1));
  J.SetElement(1, 1, (1.0/12.0) * (radius * radius));
  J.SetElement(2, 2, (1.0/12.0) * (3 * radius * radius + hlen1 * hlen1));

  TransformGyration(J, pos, rot);

  return J;
}

inline
ChMatrix33<> CalcCylinderGyration(
                 double                radius,
                 double                hlen,
                 const ChVector<>&     pos = ChVector<>(0,0,0),
                 const ChQuaternion<>& rot = ChQuaternion<>(1,0,0,0))
{
  ChMatrix33<> J;

  J.SetElement(0, 0, (1.0/12.0) * (3 * radius * radius + hlen * hlen));
  J.SetElement(1, 1, (1.0/12.0) * (radius * radius));
  J.SetElement(2, 2, (1.0/12.0) * (3 * radius * radius + hlen * hlen));

  TransformGyration(J, pos, rot);

  return J;
}

inline
ChMatrix33<> CalcRoundedCylinderGyration(
                 double                radius,
                 double                hlen,
                 double                srad,
                 const ChVector<>&     pos = ChVector<>(0,0,0),
                 const ChQuaternion<>& rot = ChQuaternion<>(1,0,0,0))
{
  ChMatrix33<> J;

  //// TODO: for now, use the gyration of the skeleton cylinder
  J.SetElement(0, 0, (1.0/12.0) * (3 * radius * radius + hlen * hlen));
  J.SetElement(1, 1, (1.0/12.0) * (radius * radius));
  J.SetElement(2, 2, (1.0/12.0) * (3 * radius * radius + hlen * hlen));

  TransformGyration(J, pos, rot);

  return J;
}

inline
ChMatrix33<> CalcRoundedBoxGyration(
                 const ChVector<>&     hdims,
                 double                srad,
                 const ChVector<>&     pos = ChVector<>(0,0,0),
                 const ChQuaternion<>& rot = ChQuaternion<>(1,0,0,0))
{
  ChMatrix33<> J;

  //// TODO: for now, use the gyration of the skeleton box
  J.SetElement(0, 0, (1.0/12.0) * (hdims.y * hdims.y + hdims.z * hdims.z));
  J.SetElement(1, 1, (1.0/12.0) * (hdims.z * hdims.z + hdims.x * hdims.x));
  J.SetElement(2, 2, (1.0/12.0) * (hdims.x * hdims.x + hdims.y * hdims.y));

  TransformGyration(J, pos, rot);

  return J;
}


} // end namespace utils
} // end namespace chrono


#endif
