#ifndef CHTHRUSTLINEARALGEBRA_H
#define CHTHRUSTLINEARALGEBRA_H

#include "math/ChParallelMath.h"

struct saxpy_functor: public thrust::binary_function<real, real, real> {
		const real a;

		saxpy_functor(real _a) :
				a(_a) {
		}

		__host__ __device__
		real operator()(const real &x, const real &y) const {
			return a * x + y;
		}
};

struct saxmy_functor: public thrust::binary_function<real, real, real> {
		const real a;

		saxmy_functor(real _a) :
				a(_a) {
		}

		__host__ __device__
		real operator()(const real &x, const real &y) const {
			return a * x - y;
		}
};
struct scale_real3_functor: public thrust::binary_function<real3, real, real3> {
		scale_real3_functor() {
		}
		__host__ __device__
		real3 operator()(const real3 & x, const real &y) const {
			return y * x;
		}
};
struct real3_real3_functor: public thrust::binary_function<real3, real3, real3> {
		real3_real3_functor() {
		}
		__host__ __device__
		real3 operator()(const real3 & x, const real3 &y) const {
			return y * x;
		}
};
static void SEAXPY(const real &a, const custom_vector<real> &x,const custom_vector<real> &y, custom_vector<real> &output)
{
#ifdef SIM_ENABLE_GPU_MODE
	thrust::transform(x.begin(), x.end(), y.begin(), output.begin(), saxpy_functor(a));
#else
#pragma omp parallel for
	for(int i=0; i<output.size(); i++) {
		output[i] = a*x[i]+y[i];
	}

#endif
}
static void SEAXMY(const real &a, const custom_vector<real> &x,const custom_vector<real> &y, custom_vector<real> &output)
{
#ifdef SIM_ENABLE_GPU_MODE
	thrust::transform(x.begin(), x.end(), y.begin(), output.begin(), saxmy_functor(a));
#else
#pragma omp parallel for
	for(int i=0; i<output.size(); i++) {
		output[i] = a*x[i]-y[i];
	}

#endif
}

static custom_vector<real> operator +(const custom_vector<real> &x, const custom_vector<real> &y)
{
	custom_vector<real> temp(x.size());
	thrust::plus<real> op;
	thrust::transform(thrust::omp::par,x.begin(), x.end(), y.begin(), temp.begin(), op);
	return temp;
}
static custom_vector<real3> operator +(const custom_vector<real3> &x, const custom_vector<real3> &y)
{
	custom_vector<real3> temp(x.size());
	thrust::plus<real3> op;
	thrust::transform(thrust::omp::par,x.begin(), x.end(), y.begin(), temp.begin(), op);
	return temp;
}

static void operator +=(custom_vector<real3> &x, const custom_vector<real3> &y)
{
	thrust::plus<real3> op;
	thrust::transform(thrust::omp::par,x.begin(), x.end(), y.begin(), x.begin(), op);

}

static void PLUS_EQ(custom_vector<real> &x, const custom_vector<real> &y)
{
#ifdef SIM_ENABLE_GPU_MODE
	thrust::plus<real3> op;
	thrust::transform(x.begin(), x.end(), y.begin(), x.begin(), op);
#else
#pragma omp parallel for
	for(int i=0; i<x.size(); i++) {
		x[i] = x[i]+y[i];
	}
#endif
}

static custom_vector<real> operator -(const custom_vector<real> &x, const custom_vector<real> &y)
{

	custom_vector<real> temp(x.size());
#ifdef SIM_ENABLE_GPU_MODE

	thrust::minus<real> op;
	thrust::transform(x.begin(), x.end(), y.begin(), temp.begin(), op);

#else

#pragma omp parallel for
	for(int i=0; i<x.size(); i++) {
		temp[i] = x[i]-y[i];
	}
#endif
	return temp;

}

static void Sub(custom_vector<real> &ans, const custom_vector<real> &x, const custom_vector<real> &y)
{
#ifdef SIM_ENABLE_GPU_MODE

	thrust::minus<real> op;
	thrust::transform(x.begin(), x.end(), y.begin(), ans.begin(), op);

#else

#pragma omp parallel for
		for(int i=0; i<x.size(); i++) {
			ans[i] = x[i]-y[i];
		}
#endif

}

static custom_vector<real> operator *(const real &x, const custom_vector<real> &y)
{
	custom_vector<real> temp(y.size());
#ifdef SIM_ENABLE_GPU_MODE
	thrust::multiplies<real> op;
	thrust::transform(y.begin(), y.end(), thrust::make_constant_iterator(x), temp.begin(), op);
#else

#pragma omp parallel for
	for(int i=0; i<y.size(); i++) {
		temp[i] = x*y[i];
	}

#endif
	return temp;
}
static custom_vector<real> operator *(const custom_vector<real> &y, const real &x)
{
	custom_vector<real> temp(y.size());
#ifdef SIM_ENABLE_GPU_MODE
	thrust::multiplies<real> op;
	thrust::transform(y.begin(), y.end(), thrust::make_constant_iterator(x), temp.begin(), op);
#else

#pragma omp parallel for
	for(int i=0; i<y.size(); i++) {
		temp[i] = x*y[i];
	}

#endif
	return temp;
}

static custom_vector<real> operator *(const custom_vector<real> &x, const custom_vector<real> &y)
{
	custom_vector<real> temp(x.size());
#ifdef SIM_ENABLE_GPU_MODE
	thrust::multiplies<real> op;
	thrust::transform(x.begin(), x.end(), y.begin(), temp.begin(), op);
#else

#pragma omp parallel for
	for(int i=0; i<x.size(); i++) {
		temp[i] = x[i]*y[i];
	}

#endif
	return temp;
}

static custom_vector<real3> operator *(const real &x, const custom_vector<real3> &y)
{
	custom_vector<real3> temp(y.size());
#pragma omp parallel for
	for(int i=0; i<y.size(); i++) {
		temp[i] = x*y[i];
	}

	return temp;
}

static custom_vector<real3> operator *(const custom_vector<real3> &x, const custom_vector<real> &y)
{
	custom_vector<real3> temp(x.size());
	thrust::transform(thrust::omp::par,x.begin(), x.end(), y.begin(), temp.begin(), scale_real3_functor());
	return temp;
}
static custom_vector<real3> operator *(const custom_vector<real3> &x, const custom_vector<real3> &y)
{
	custom_vector<real3> temp(x.size());
	thrust::transform(thrust::omp::par,x.begin(), x.end(), y.begin(), temp.begin(), real3_real3_functor());
	return temp;
}

static void operator *=(custom_vector<real3> &x, const custom_vector<real> &y)
{
	thrust::transform(thrust::omp::par,x.begin(), x.end(), y.begin(), x.begin(), scale_real3_functor());
}
static void operator *=(custom_vector<real3> &x, const custom_vector<real3> &y)
{
	thrust::transform(x.begin(), x.end(), y.begin(), x.begin(), real3_real3_functor());
}
static custom_vector<real> operator /(const custom_vector<real> &x, const custom_vector<real> &y)
{
	custom_vector<real> temp(x.size());
	thrust::divides<real> op;
	thrust::transform(thrust::omp::par,x.begin(), x.end(), y.begin(), temp.begin(), op);
	return temp;
}

static real Dot(const custom_vector<real> &x, const custom_vector<real> &y)
{
#ifdef SIM_ENABLE_GPU_MODE
	thrust::plus<real> binary_op1;
	thrust::multiplies<real> binary_op2;
	real answer = thrust::inner_product(x.begin(), x.end(), y.begin(), real(0.0), binary_op1, binary_op2);
	return answer;
#else
	real sum=0;

#pragma omp parallel for reduction(+:sum)
	for(int i=0; i<x.size(); i++) {
		sum+=x[i]*y[i];
	}
	return sum;
#endif

}

struct abs_functor: public thrust::unary_function<real, real> {

		__host__ __device__
		float operator()(const real &x) const {
			return fabs(x);
		}
};

static custom_vector<real> Abs(const custom_vector<real> &x)
{
	custom_vector<real> temp(x.size());
	thrust::transform(thrust::omp::par,x.begin(), x.end(),temp.begin() , abs_functor());
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
#ifdef SIM_ENABLE_GPU_MODE
	square<real> unary_op;
	thrust::plus<real> binary_op;
	real init = 0;
	return sqrt( thrust::transform_reduce(x.begin(), x.end(), unary_op, init, binary_op) );
	//return sqrt(Dot(x, x));
#else
	real sum=0;

#pragma omp parallel for reduction(+:sum)
	for(int i=0; i<x.size(); i++) {
		real _x = x[i];
		sum+=_x*_x;
	}
	return sqrt(sum);
#endif
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
struct summary_stats_binary_op: public thrust::binary_function<const summary_stats_data<T>&, const summary_stats_data<T>&, summary_stats_data<T> > {
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
#endif
