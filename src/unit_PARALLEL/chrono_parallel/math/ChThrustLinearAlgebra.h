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

#include "chrono_parallel/ChConfigParallel.h"
#include "chrono_parallel/math/ChParallelMath.h"

static void SEAXPY(const real &a, const custom_vector<real> &x,const custom_vector<real> &y, custom_vector<real> &output)
{
#ifdef CHRONO_PARALLEL_OMP_40
#pragma omp parallel for simd safelen(4)
#else
#pragma omp parallel for
#endif
	for(int i=0; i<output.size(); i++) {
		output[i] = a*x[i]+y[i];
	}

}

static void SEAXPY(int SIZE, const real &a, real* __restrict__  x,real* __restrict__ y, real* __restrict__ output)
{
#ifdef CHRONO_PARALLEL_OMP_40
#pragma omp parallel for simd safelen(4)
#else
#pragma omp parallel for
#endif
	for(int i=0; i<SIZE; i++) {
		output[i] = a*x[i]+y[i];
	}

}

static void SEAXMY(const real &a, const custom_vector<real> &x,const custom_vector<real> &y, custom_vector<real> &output)
{
#ifdef CHRONO_PARALLEL_OMP_40
#pragma omp parallel for simd safelen(4)
#else
#pragma omp parallel for
#endif
	for(int i=0; i<output.size(); i++) {
		output[i] = a*x[i]-y[i];
	}
}
template<typename T>
static custom_vector<T> operator +(const custom_vector<T> &x, const custom_vector<T> &y)
{
	custom_vector<T> temp(x.size());

#ifdef CHRONO_PARALLEL_OMP_40
#pragma omp parallel for simd safelen(4)
#else
#pragma omp parallel for
#endif
	for(int i=0; i<temp.size(); i++) {
		temp[i] = x[i]+y[i];
	}

	return temp;
}
template<typename T>
static void operator +=(custom_vector<T> &x, const custom_vector<T> &y)
{
#ifdef CHRONO_PARALLEL_OMP_40
#pragma omp parallel for simd safelen(4)
#else
#pragma omp parallel for
#endif
	for(int i=0; i<x.size(); i++) {
		x[i] = x[i]+y[i];
	}
}
template<typename T>
static custom_vector<T> operator -(const custom_vector<T> &x, const custom_vector<T> &y)
{
	custom_vector<T> temp(x.size());

#ifdef CHRONO_PARALLEL_OMP_40
#pragma omp parallel for simd safelen(4)
#else
#pragma omp parallel for
#endif
	for(int i=0; i<x.size(); i++) {
		temp[i] = x[i]-y[i];
	}

	return temp;

}

static void Sub(custom_vector<real> &ans, const custom_vector<real> &x, const custom_vector<real> &y)
{
#ifdef CHRONO_PARALLEL_OMP_40
#pragma omp parallel for simd safelen(4)
#else
#pragma omp parallel for
#endif
	for(int i=0; i<x.size(); i++) {
		ans[i] = x[i]-y[i];
	}

}
template<typename T, typename U>
static custom_vector<T> operator *(const custom_vector<T> &y, const U &x)
{
	custom_vector<T> temp(y.size());

#ifdef CHRONO_PARALLEL_OMP_40
#pragma omp parallel for simd safelen(4)
#else
#pragma omp parallel for
#endif
	for(int i=0; i<y.size(); i++) {
		temp[i] = x*y[i];
	}

	return temp;
}
template<typename T, typename U>
static custom_vector<T> operator *(const custom_vector<T> &x, const custom_vector<U> &y)
{
	custom_vector<T> temp(x.size());

#ifdef CHRONO_PARALLEL_OMP_40
#pragma omp parallel for simd safelen(4)
#else
#pragma omp parallel for
#endif
	for(int i=0; i<x.size(); i++) {
		temp[i] = x[i]*y[i];
	}

	return temp;
}
template<typename T, typename U>
static custom_vector<U> operator *(const T &x, const custom_vector<U> &y)
{
	custom_vector<U> temp(y.size());

#ifdef CHRONO_PARALLEL_OMP_40
#pragma omp parallel for simd safelen(4)
#else
#pragma omp parallel for
#endif
	for(int i=0; i<y.size(); i++) {
		temp[i] = x*y[i];
	}

	return temp;
}

template<typename T, typename U>
static void operator *=(custom_vector<T> &x, const custom_vector<U> &y)
{
#ifdef CHRONO_PARALLEL_OMP_40
#pragma omp parallel for simd safelen(4)
#else
#pragma omp parallel for
#endif
	for(int i=0; i<x.size(); i++) {
		x[i] = x[i]*y[i];
	}
}
template<typename T>
static custom_vector<T> operator /(const custom_vector<T> &x, const custom_vector<T> &y)
{
	custom_vector<T> temp(y.size());

#ifdef CHRONO_PARALLEL_OMP_40
#pragma omp parallel for simd safelen(4)
#else
#pragma omp parallel for
#endif
	for(int i=0; i<y.size(); i++) {
		temp[i] = x[i]/y[i];
	}

	return temp;
}

static real Dot(const custom_vector<real> &x, const custom_vector<real> &y)
{
	real sum=0;
#pragma omp parallel for reduction(+:sum)
	for(int i=0; i<x.size(); i++) {
		sum+=x[i]*y[i];
	}
	return sum;

}

struct abs_functor: public thrust::unary_function<real, real> {

		__host__ __device__
		real operator()(const real &x) const {
			return fabs(x);
		}
};

static custom_vector<real> Abs(const custom_vector<real> &x)
{
	custom_vector<real> temp(x.size());

#ifdef CHRONO_PARALLEL_OMP_40
#pragma omp parallel for simd safelen(4)
#else
#pragma omp parallel for
#endif
	for(int i=0; i<x.size(); i++) {
		temp[i] = fabs(x[i]);
	}
	return temp;
}

static custom_vector<real> max(const real a, const custom_vector<real> &x){
	custom_vector<real> temp(x.size());

#ifdef CHRONO_PARALLEL_OMP_40
#pragma omp parallel for simd safelen(4)
#else
#pragma omp parallel for
#endif
	for(int i=0; i<x.size(); i++) {
		temp[i] = std::max(a,x[i]);
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

static real Norm(const custom_vector<real> &x)
{
	real sum=0;
#pragma omp parallel for reduction(+:sum)
	for(int i=0; i<x.size(); i++) {
		real _x = x[i];
		sum+=_x*_x;
	}
	return sqrt(sum);

}
static real NormInf(const custom_vector<real> &x)
{

	custom_vector<real> res = Abs(x);

	return res[thrust::max_element(thrust::omp::par,res.begin(),res.end())-res.begin()];
}

static real CompRes(const custom_vector<real> &res, const uint n_o_c)
{
	//real minval = *thrust::min_element(res.begin(), res.end());
	//real minval = *thrust::min_element(res.begin(), res.begin()+n_o_c);
	real minval = res[thrust::min_element(thrust::omp::par,res.begin(),res.begin()+n_o_c)-res.begin()];
	return fmax(real(0.0),-minval);
}

template<class T>
static inline std::ostream &operator<<(std::ostream &out, const thrust::device_vector<T> &x) {
	for (uint i = 0; i < x.size(); i++) {
		out << x[i] << std::endl;
	}
	return out;
}
template<class T>
static inline std::ostream &operator<<(std::ostream &out, const thrust::host_vector<T> &x) {
	for (uint i = 0; i < x.size(); i++) {
		out << x[i] << std::endl;
	}
	return out;
}


// This example computes several statistical properties of a data
// series in a single reduction. The algorithm is described in detail here:
// http://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Parallel_algorithm
//
// Thanks to Joseph Rhoads for contributing this example

// structure used to accumulate the moments and other
// statistical properties encountered so far.
template<typename T>
struct summary_stats_data {
		T n;
		T min;
		T max;
		T mean;
		T M2;
		T M3;
		T M4;

		// initialize to the identity element
		void initialize() {
			n = mean = M2 = M3 = M4 = 0;
			min = std::numeric_limits<T>::max();
			max = std::numeric_limits<T>::min();
		}

		T variance() {
			return M2 / (n - 1);
		}
		T variance_n() {
			return M2 / n;
		}
		T skewness() {
			return std::sqrt(n) * M3 / std::pow(M2, (T) 1.5);
		}
		T kurtosis() {
			return n * M4 / (M2 * M2);
		}
};

// stats_unary_op is a functor that takes in a value x and
// returns a variace_data whose mean value is initialized to x.
template<typename T>
struct summary_stats_unary_op {
		__host__ __device__
		summary_stats_data<T> operator()(const T& x) const {
			summary_stats_data<T> result;
			result.n = 1;
			result.min = x;
			result.max = x;
			result.mean = x;
			result.M2 = 0;
			result.M3 = 0;
			result.M4 = 0;

			return result;
		}
};

// summary_stats_binary_op is a functor that accepts two summary_stats_data
// structs and returns a new summary_stats_data which are an
// approximation to the summary_stats for
// all values that have been agregated so far
template<typename T>
struct summary_stats_binary_op: public thrust::binary_function<const summary_stats_data<T>&,
		const summary_stats_data<T>&, summary_stats_data<T> > {
		__host__ __device__
		summary_stats_data<T> operator()(const summary_stats_data<T>& x, const summary_stats_data<T>& y) const {
			summary_stats_data<T> result;

			// precompute some common subexpressions
			T n = x.n + y.n;
			T n2 = n * n;
			T n3 = n2 * n;

			T delta = y.mean - x.mean;
			T delta2 = delta * delta;
			T delta3 = delta2 * delta;
			T delta4 = delta3 * delta;

			//Basic number of samples (n), min, and max
			result.n = n;
			result.min = fmin(x.min, y.min);
			result.max = fmax(x.max, y.max);

			result.mean = x.mean + delta * y.n / n;

			result.M2 = x.M2 + y.M2;
			result.M2 += delta2 * x.n * y.n / n;

			result.M3 = x.M3 + y.M3;
			result.M3 += delta3 * x.n * y.n * (x.n - y.n) / n2;
			result.M3 += (T) 3.0 * delta * (x.n * y.M2 - y.n * x.M2) / n;

			result.M4 = x.M4 + y.M4;
			result.M4 += delta4 * x.n * y.n * (x.n * x.n - x.n * y.n + y.n * y.n) / n3;
			result.M4 += (T) 6.0 * delta2 * (x.n * x.n * y.M2 + y.n * y.n * x.M2) / n2;
			result.M4 += (T) 4.0 * delta * (x.n * y.M3 - y.n * x.M3) / n;

			return result;
		}
};

template<typename Iterator>
void print_range(const std::string& name, Iterator first, Iterator last) {
	typedef typename std::iterator_traits<Iterator>::value_type T;

	std::cout << name << ": ";
	thrust::copy(first, last, std::ostream_iterator<T>(std::cout, " "));
	std::cout << "\n";
}

static summary_stats_data<real> Statistics(const custom_vector<real> &x)
{
	summary_stats_unary_op<real> unary_op;
	summary_stats_binary_op<real> binary_op;
	summary_stats_data<real> init;
	summary_stats_data<real> result = thrust::transform_reduce(x.begin(), x.end(), unary_op, init, binary_op);
	return result;
}

// Binary operation for adding two-object tuples
struct sum_tuples {
		template<typename Tuple>
		__host__ __device__
		Tuple operator()(const Tuple& a, const Tuple& b) const {
			return Tuple(thrust::get<0>(a) + thrust::get<0>(b), thrust::get<1>(a) + thrust::get<1>(b));
		}
};

#endif
