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
// Authors: Hammad Mazhar
// =============================================================================
//
// Description: simple linear algebra functions using thrust
// =============================================================================

#ifndef CHTHRUSTLINEARALGEBRA_H
#define CHTHRUSTLINEARALGEBRA_H

#include <algorithm>

#include "chrono_parallel/ChConfigParallel.h"
#include "chrono_parallel/math/ChParallelMath.h"

static void SEAXPY(const real &a,
                   const custom_vector<real> &x,
                   const custom_vector<real> &y,
                   custom_vector<real> &output) {
#ifdef CHRONO_PARALLEL_OMP_40
#pragma omp parallel for simd safelen(4)
#else
#pragma omp parallel for
#endif
   for (int i = 0; i < output.size(); i++) {
      output[i] = a * x[i] + y[i];
   }

}

static void SEAXPY(int SIZE,
                   const real &a,
                   real* __restrict__ x,
                   real* __restrict__ y,
                   real* __restrict__ output) {
#ifdef CHRONO_PARALLEL_OMP_40
#pragma omp parallel for simd safelen(4)
#else
#pragma omp parallel for
#endif
   for (int i = 0; i < SIZE; i++) {
      output[i] = a * x[i] + y[i];
   }

}

static void SEAXMY(const real &a,
                   const custom_vector<real> &x,
                   const custom_vector<real> &y,
                   custom_vector<real> &output) {
#ifdef CHRONO_PARALLEL_OMP_40
#pragma omp parallel for simd safelen(4)
#else
#pragma omp parallel for
#endif
   for (int i = 0; i < output.size(); i++) {
      output[i] = a * x[i] - y[i];
   }
}
template<typename T>
static custom_vector<T> operator +(const custom_vector<T> &x,
                                   const custom_vector<T> &y) {
   custom_vector<T> temp(x.size());

#ifdef CHRONO_PARALLEL_OMP_40
#pragma omp parallel for simd safelen(4)
#else
#pragma omp parallel for
#endif
   for (int i = 0; i < temp.size(); i++) {
      temp[i] = x[i] + y[i];
   }

   return temp;
}
template<typename T>
static void operator +=(custom_vector<T> &x,
                        const custom_vector<T> &y) {
#ifdef CHRONO_PARALLEL_OMP_40
#pragma omp parallel for simd safelen(4)
#else
#pragma omp parallel for
#endif
   for (int i = 0; i < x.size(); i++) {
      x[i] = x[i] + y[i];
   }
}
template<typename T>
static custom_vector<T> operator -(const custom_vector<T> &x,
                                   const custom_vector<T> &y) {
   custom_vector<T> temp(x.size());

#ifdef CHRONO_PARALLEL_OMP_40
#pragma omp parallel for simd safelen(4)
#else
#pragma omp parallel for
#endif
   for (int i = 0; i < x.size(); i++) {
      temp[i] = x[i] - y[i];
   }

   return temp;

}

static void Sub(custom_vector<real> &ans,
                const custom_vector<real> &x,
                const custom_vector<real> &y) {
#ifdef CHRONO_PARALLEL_OMP_40
#pragma omp parallel for simd safelen(4)
#else
#pragma omp parallel for
#endif
   for (int i = 0; i < x.size(); i++) {
      ans[i] = x[i] - y[i];
   }

}
template<typename T, typename U>
static custom_vector<T> operator *(const custom_vector<T> &y,
                                   const U &x) {
   custom_vector<T> temp(y.size());

#ifdef CHRONO_PARALLEL_OMP_40
#pragma omp parallel for simd safelen(4)
#else
#pragma omp parallel for
#endif
   for (int i = 0; i < y.size(); i++) {
      temp[i] = x * y[i];
   }

   return temp;
}
template<typename T, typename U>
static custom_vector<T> operator *(const custom_vector<T> &x,
                                   const custom_vector<U> &y) {
   custom_vector<T> temp(x.size());

#ifdef CHRONO_PARALLEL_OMP_40
#pragma omp parallel for simd safelen(4)
#else
#pragma omp parallel for
#endif
   for (int i = 0; i < x.size(); i++) {
      temp[i] = x[i] * y[i];
   }

   return temp;
}
template<typename T, typename U>
static custom_vector<U> operator *(const T &x,
                                   const custom_vector<U> &y) {
   custom_vector<U> temp(y.size());

#ifdef CHRONO_PARALLEL_OMP_40
#pragma omp parallel for simd safelen(4)
#else
#pragma omp parallel for
#endif
   for (int i = 0; i < y.size(); i++) {
      temp[i] = x * y[i];
   }

   return temp;
}

template<typename T, typename U>
static void operator *=(custom_vector<T> &x,
                        const custom_vector<U> &y) {
#ifdef CHRONO_PARALLEL_OMP_40
#pragma omp parallel for simd safelen(4)
#else
#pragma omp parallel for
#endif
   for (int i = 0; i < x.size(); i++) {
      x[i] = x[i] * y[i];
   }
}
template<typename T>
static custom_vector<T> operator /(const custom_vector<T> &x,
                                   const custom_vector<T> &y) {
   custom_vector<T> temp(y.size());

#ifdef CHRONO_PARALLEL_OMP_40
#pragma omp parallel for simd safelen(4)
#else
#pragma omp parallel for
#endif
   for (int i = 0; i < y.size(); i++) {
      temp[i] = x[i] / y[i];
   }

   return temp;
}
template<typename T>
static real Dot(const T &x,
                const T &y) {
   double sum = 0;
#pragma omp parallel for reduction(+:sum)
   for (int i = 0; i < x.size(); i++) {
      sum += x[i] * y[i];
   }
   return sum;

}

struct abs_functor : public thrust::unary_function<real, real> {

   __host__ __device__
   real operator()(const real &x) const {
      return fabs(x);
   }
};

static custom_vector<real> Abs(const custom_vector<real> &x) {
   custom_vector<real> temp(x.size());

#ifdef CHRONO_PARALLEL_OMP_40
#pragma omp parallel for simd safelen(4)
#else
#pragma omp parallel for
#endif
   for (int i = 0; i < x.size(); i++) {
      temp[i] = fabs(x[i]);
   }
   return temp;
}

static custom_vector<real> max(const real a,
                               const custom_vector<real> &x) {
   custom_vector<real> temp(x.size());

#ifdef CHRONO_PARALLEL_OMP_40
#pragma omp parallel for simd safelen(4)
#else
#pragma omp parallel for
#endif
   for (int i = 0; i < x.size(); i++) {
      temp[i] = std::max(a, x[i]);
   }
   return temp;
}

template<typename T>
struct square {
   __host__ __device__
   T operator()(const T& x) const {
      return x * x;
   }
};
template<typename T>
static real Norm(const T &x) {
   real sum = 0;
#pragma omp parallel for reduction(+:sum)
   for (int i = 0; i < x.size(); i++) {
      real _x = x[i];
      sum += _x * _x;
   }
   return sqrt(sum);

}
static real NormInf(const custom_vector<real> &x) {
   custom_vector<real> res = Abs(x);
   return res[thrust::max_element(thrust::omp::par, res.begin(), res.end()) - res.begin()];
}

static real CompRes(const custom_vector<real> &res,
                    const uint n_o_c) {
   //real minval = *thrust::min_element(res.begin(), res.end());
   //real minval = *thrust::min_element(res.begin(), res.begin()+n_o_c);
   real minval = res[thrust::min_element(thrust::omp::par, res.begin(), res.begin() + n_o_c) - res.begin()];
   return std::fmax(real(0.0), -minval);
}

template<class T>
static inline std::ostream &operator<<(std::ostream &out,
                                       const thrust::device_vector<T> &x) {
   for (uint i = 0; i < x.size(); i++) {
      out << x[i] << std::endl;
   }
   return out;
}
template<class T>
static inline std::ostream &operator<<(std::ostream &out,
                                       const thrust::host_vector<T> &x) {
   for (uint i = 0; i < x.size(); i++) {
      out << x[i] << std::endl;
   }
   return out;
}

// Binary operation for adding two-object tuples
struct sum_tuples {
   template<typename Tuple>
   __host__ __device__
   Tuple operator()(const Tuple& a,
                    const Tuple& b) const {
      return Tuple(thrust::get<0>(a) + thrust::get<0>(b), thrust::get<1>(a) + thrust::get<1>(b));
   }
};

#endif
