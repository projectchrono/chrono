#ifndef CHCONSTRAINT_FLUIDFLUIDUTILS_H
#define CHCONSTRAINT_FLUIDFLUIDUTILS_H

#include "chrono_parallel/ChDataManager.h"
#include "chrono_parallel/math/ChParallelMath.h"
#include "chrono_parallel/math/matrix.h"  // for quaternion, real4
#define F_PI 3.141592653589793238462643383279
#define INVPI 1 / F_PI

#define KERNEL poly6
#define GRAD_KERNEL unormalized_grad_spiky
#define GRAD2_KERNEL grad2_viscosity

namespace chrono {
#define H2 h* h
#define H3 h* h* h
#define H6 H3* H3
#define H9 H3* H3* H3
///
#define CPOLY6 315.0 / (64.0 * F_PI * H9)
#define KPOLY6 CPOLY6* Pow((H2 - dist * dist), 3)

#define CGPOLY6 -945.0 / (32.0 * F_PI * H9)
#define KGPOLY6 CGPOLY6* Pow((H2 - dist * dist), 2)

#define CLPOLY6 945.0 / (32.0 * F_PI * H9)
#define KLPOLY6 CLPOLY6*(H2 - dist * dist) * (7 * dist * dist - 3 * H2)

///
#define CGSPIKY -45.0 / (F_PI * H6)
#define KGSPIKY CGSPIKY* Pow(h - dist, 2)

#define CLVISC 45.0 / (F_PI * H6)
#define KLVISC CLVISC*(h - dist)

inline real N(const real& dist, const real& h) {
    real x = Abs(dist) / h;
    if (Abs(x) < real(1.0)) {
        return real(0.5) * Cube(Abs(x)) - Sqr(x) + 2.0 / 3.0;
    } else if (Abs(x) < real(2.0)) {
        return -1.0 / 6.0 * Cube(Abs(x)) + Sqr(x) - real(2.0) * Abs(x) + 4.0 / 3.0;
    }
    return real(0.0);
}

// Cubic spline kernel
// d is positive. h is the sph particle  radius (i.e. h in the document) d is the distance of 2 particles
inline real cubic_spline(const real& dist, const real& h) {
    real q = Abs(dist) / h;
    if (q < 1) {
        return (0.25f / (F_PI * h * h * h) * (Pow(2 - q, 3) - 4 * Pow(1 - q, 3)));
    }
    if (q < 2) {
        return (0.25f / (F_PI * h * h * h) * Pow(2 - q, 3));
    }
    return 0;
}
// d is positive. r is the sph particles
inline real3 grad_cubic_spline(const real3& dist, const real d, const real& h) {
    real q = d / h;

    if (q < 1) {
        return (3 * q - 4) * .75 * (INVPI)*Pow(h, -5) * dist;
    }
    if (q < 2) {
        return (-q + 4.0 - 4.0 / q) * .75 * (INVPI)*Pow(h, -5) * dist;
    }
    return real3(0);
}
inline real poly6(const real& dist, const real& h) {
    return /* (dist <= h)* */ KPOLY6;
}

inline real3 grad_poly6(const real3& xij, const real d, const real& h) {
    return (d <= h) * -945.0 / (32.0 * F_PI * Pow(h, 9)) * Pow((h * h - d * d), 2) * xij;
}

inline real spiky(const real& dist, const real& h) {
    return (dist <= h) * 15.0 / (F_PI * Pow(h, 6)) * Pow(h - dist, 3);
}
inline real3 grad_spiky(const real3& xij, const real dist, const real& h) {
    return (dist <= h) * KGSPIKY * xij;
}

inline real unormalized_spiky(const real& dist, const real& h) {
    const real k = 15.0 / (F_PI * h * h * h);
    return k * Sqr(1.0 - dist / h);
}

inline real3 unormalized_grad_spiky(const real3& xij, const real d, const real& h) {
    const real k = 15.0 / (F_PI * h * h * h);
    return -k * (1.0 - d / h) / h * xij / d;
}

inline real3 viscosity(const real3& xij, const real d, const real& h) {
    return (d <= h) * 15.0 / (2 * F_PI * Pow(h, 3)) *
           (-(d * d * d) / (2 * h * h * h) + (d * d) / (h * h) + (h) / (2 * d) - 1) * xij;
}
inline real3 grad2_viscosity(const real3& xij, const real d, const real& h) {
    return real3((d <= h) * 45.0 / (F_PI * Pow(h, 6)) * (h - d));
}

////-----------------------------------------------------------------------------------------------------
// kernel from constraint fluid approximation paper/code
inline real kernel(const real& dist, const real& h) {
    if (dist > h) {
        return 0;
    }

    return Pow(1 - Pow(dist / h, 2), 3);
}

// laplacian operator for poly6
inline real grad2_poly6(const real& dist, const real& h) {
    if (dist > h) {
        return 0;
    }
    return 945.0 / (32.0 * F_PI * Pow(h, 9)) * (h * h - dist * dist) * (7 * dist * dist - 3 * h * h);
}

#define SS(alpha) mrho* vij.alpha
#define TT(beta) grad.beta

inline Mat33 ComputeShearTensor(const real& mrho, const real3& grad, const real3& vij) {
    return Mat33(-.5 * 2 * SS(x) * TT(x), -.5 * (SS(y) * TT(x) + SS(x) * TT(y)), -.5 * (SS(z) * TT(x) + SS(x) * TT(z)),
                 -.5 * (SS(x) * TT(y) + SS(y) * TT(x)), -.5 * 2 * SS(y) * TT(y), -.5 * (SS(z) * TT(y) + SS(y) * TT(z)),
                 -.5 * (SS(x) * TT(z) + SS(z) * TT(x)), -.5 * (SS(y) * TT(z) + SS(z) * TT(y)), -.5 * 2 * SS(z) * TT(z));

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
