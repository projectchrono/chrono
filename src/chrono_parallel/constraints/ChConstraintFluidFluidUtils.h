#ifndef CHCONSTRAINT_FLUIDFLUIDUTILS_H
#define CHCONSTRAINT_FLUIDFLUIDUTILS_H

#include "chrono_parallel/ChDataManager.h"
#include "chrono_parallel/math/ChParallelMath.h"

#define F_PI 3.141592653589793238462643383279
#define INVPI 1 / F_PI

#define KERNEL poly6
#define GRAD_KERNEL grad_spiky
#define GRAD2_KERNEL grad2_viscosity

namespace chrono {

// Cubic spline kernel
// d is positive. h is the sph particle  radius (i.e. h in the document) d is the distance of 2 particles
real cubic_spline(const real& dist, const real& h) {
  real q = std::abs(dist) / h;
  if (q < 1) {
    return (0.25f / (F_PI * h * h * h) * (pow(2 - q, 3) - 4 * pow(1 - q, 3)));
  }
  if (q < 2) {
    return (0.25f / (F_PI * h * h * h) * pow(2 - q, 3));
  }
  return 0;
}
// d is positive. r is the sph particles
real3 grad_cubic_spline(const real3& dist, const real d, const real& h) {
  real q = d / h;

  if (q < 1) {
    return (3 * q - 4) * .75f * (INVPI)*powf(h, -5) * dist;
  }
  if (q < 2) {
    return (-q + 4.0f - 4.0f / q) * .75f * (INVPI)*powf(h, -5) * dist;
  }
  return 0;
}
real poly6(const real& dist, const real& h) {
  return (dist <= h) * 315.0 / (64.0 * F_PI * pow(h, 9)) * pow((h * h - dist * dist), 3);
}

real3 grad_poly6(const real3& dist, const real d, const real& h) {
  return (d <= h) * -945.0 / (32.0 * F_PI * pow(h, 9)) * pow((h * h - d * d), 2) * dist;
}

real spiky(const real& dist, const real& h) {
  return (dist <= h) * 15.0 / (F_PI * pow(h, 6)) * pow(h - dist, 3);
}
real3 grad_spiky(const real3& dist, const real d, const real& h) {
  return (d <= h) * -45.0 / (F_PI * pow(h, 6)) * pow(h - d, 2) * dist;
}

real3 viscosity(const real3& dist, const real d, const real& h) {
  return (d <= h) * 15.0 / (2 * F_PI * pow(h, 3)) *
         (-(d * d * d) / (2 * h * h * h) + (d * d) / (h * h) + (h) / (2 * d) - 1) * dist;
}
real3 grad2_viscosity(const real3& dist, const real d, const real& h) {
  return (d <= h) * 45.0 / (F_PI * pow(h, 6)) * (h - d);
}

////-----------------------------------------------------------------------------------------------------
// kernel from constraint fluid approximation paper/code
real kernel(const real& dist, const real& h) {
  if (dist > h) {
    return 0;
  }

  return pow(1 - pow(dist / h, 2), 3);
}

// laplacian operator for poly6
real grad2_poly6(const real& dist, const real& h) {
  if (dist > h) {
    return 0;
  }
  return 945.0 / (32.0 * F_PI * pow(h, 9)) * (h * h - dist * dist) * (7 * dist * dist - 3 * h * h);
}

#define SS(alpha) mrho* vij.alpha
#define TT(beta) grad.beta

M33 ComputeShearTensor(const real& mrho, const real3& grad, const real3& vij) {
  real3 U = -.5 * R3(2 * SS(x) * TT(x), (SS(y) * TT(x) + SS(x) * TT(y)), (SS(z) * TT(x) + SS(x) * TT(z)));
  real3 V = -.5 * R3((SS(x) * TT(y) + SS(y) * TT(x)), 2 * SS(y) * TT(y), (SS(z) * TT(y) + SS(y) * TT(z)));
  real3 W = -.5 * R3((SS(x) * TT(z) + SS(z) * TT(x)), (SS(y) * TT(z) + SS(z) * TT(y)), 2 * SS(z) * TT(z));
  return M33(U, V, W);

  //  return (VectorxVector(mrho * vij, grad) + VectorxVector(grad, mrho * vij)) * -.5;
}


//// Compute ||T||  = sqrt((1/2*Trace((shear*Transpose(shear)))))
// real ComputeShearTensorNorm(const real& mrho, const real3& grad, const real3& vij) {
//  real t1 = SS(x) * SS(x);
//  real t2 = TT(x) * TT(x);
//  real t5 = TT(y) * TT(y);
//  real t11 = SS(y) * SS(y);
//  real t13 = TT(z) * TT(z);
//  real t19 = SS(z) * SS(z);
//  real t31 = 2 * t2 * t1 + t5 * t1 + 2 * SS(x) * TT(y) * SS(y) * TT(x) + t2 * t11 + t13 * t1 +
//             2 * SS(x) * TT(z) * SS(z) * TT(x) + t2 * t19 + 2 * t5 * t11 + t13 * t11 +
//             2 * SS(y) * TT(z) * SS(z) * TT(y) + t5 * t19 + 2 * t13 * t19;
//  return sqrt(t31) * 0.5;
//}

}

#endif
