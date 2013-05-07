#ifndef CHTHRUSTLINEARALGEBRA_H
#define CHTHRUSTLINEARALGEBRA_H

#include "ChCudaMath.h"

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
	thrust::transform(x.begin(), x.end(), y.begin(), output.begin(), saxpy_functor(a));
}
static void SEAXMY(const real &a, const custom_vector<real> &x,const custom_vector<real> &y, custom_vector<real> &output)
{
	thrust::transform(x.begin(), x.end(), y.begin(), output.begin(), saxmy_functor(a));
}

static custom_vector<real> operator +(const custom_vector<real> &x, const custom_vector<real> &y)
{
	custom_vector<real> temp(x.size());
	thrust::plus<real> op;
	thrust::transform(x.begin(), x.end(), y.begin(), temp.begin(), op);
	return temp;
}
static custom_vector<real3> operator +(const custom_vector<real3> &x, const custom_vector<real3> &y)
{
	custom_vector<real3> temp(x.size());
	thrust::plus<real3> op;
	thrust::transform(x.begin(), x.end(), y.begin(), temp.begin(), op);
	return temp;
}

static void operator +=(custom_vector<real3> &x, const custom_vector<real3> &y)
{
	thrust::plus<real3> op;
	thrust::transform(x.begin(), x.end(), y.begin(), x.begin(), op);

}

static custom_vector<real> operator -(const custom_vector<real> &x, const custom_vector<real> &y)
{
	custom_vector<real> temp(x.size());
	thrust::minus<real> op;
	thrust::transform(x.begin(), x.end(), y.begin(), temp.begin(), op);
	return temp;
}

static custom_vector<real> operator *(const real &x, const custom_vector<real> &y)
{
	custom_vector<real> temp(y.size());
	thrust::multiplies<real> op;
	thrust::transform(y.begin(), y.end(), thrust::make_constant_iterator(x), temp.begin(), op);
	return temp;
}
static custom_vector<real> operator *(const custom_vector<real> &y, const real &x)
{
	custom_vector<real> temp(y.size());
	thrust::multiplies<real> op;
	thrust::transform(y.begin(), y.end(), thrust::make_constant_iterator(x), temp.begin(), op);
	return temp;
}

static custom_vector<real> operator *(const custom_vector<real> &x, const custom_vector<real> &y)
{
	custom_vector<real> temp(x.size());
	thrust::multiplies<real> op;
	thrust::transform(x.begin(), x.end(), y.begin(), temp.begin(), op);
	return temp;
}
static custom_vector<real3> operator *(const custom_vector<real3> &x, const custom_vector<real> &y)
{
	custom_vector<real3> temp(x.size());
	thrust::transform(x.begin(), x.end(), y.begin(), temp.begin(), scale_real3_functor());
	return temp;
}
static custom_vector<real3> operator *(const custom_vector<real3> &x, const custom_vector<real3> &y)
{
	custom_vector<real3> temp(x.size());
	thrust::transform(x.begin(), x.end(), y.begin(), temp.begin(), real3_real3_functor());
	return temp;
}

static void operator *=(custom_vector<real3> &x, const custom_vector<real> &y)
{
	thrust::transform(x.begin(), x.end(), y.begin(), x.begin(), scale_real3_functor());
}
static void operator *=(custom_vector<real3> &x, const custom_vector<real3> &y)
{
	thrust::transform(x.begin(), x.end(), y.begin(), x.begin(), real3_real3_functor());
}
static custom_vector<real> operator /(const custom_vector<real> &x, const custom_vector<real> &y)
{
	custom_vector<real> temp(x.size());
	thrust::divides<real> op;
	thrust::transform(x.begin(), x.end(), y.begin(), temp.begin(), op);
	return temp;
}

static real Dot(const custom_vector<real> &x, const custom_vector<real> &y)
{
	thrust::plus<real> binary_op1;
	thrust::multiplies<real> binary_op2;
	real answer = thrust::inner_product(x.begin(), x.end(), y.begin(), real(0.0), binary_op1, binary_op2);
	return answer;
}

template<typename T>
struct square
{
	__host__ __device__
	T operator()(const T& x) const {
		return x * x;
	}
};

struct abs_functor: public thrust::unary_function<real, real> {

	__host__ __device__
	float operator()(const real &x) const {
		return fabs(x);
	}
};


static custom_vector<real> Abs(const custom_vector<real> &x)
{
	custom_vector<real> temp(x.size());
	thrust::transform(x.begin(), x.end(),temp.begin() , abs_functor());
	return temp;

}

static real Norm(const custom_vector<real> &x)
{
	square<real> unary_op;
	thrust::plus<real> binary_op;
	real init = 0;
	return sqrt( thrust::transform_reduce(x.begin(), x.end(), unary_op, init, binary_op) );
	//return sqrt(Dot(x, x));

	}
static real NormInf(const custom_vector<real> &x)
{

	custom_vector<real> res = Abs(x);

	return res[thrust::max_element(res.begin(),res.end())-res.begin()];
}




static real CompRes(const custom_vector<real> &res, const uint n_o_c)
{
	//real minval = *thrust::min_element(res.begin(), res.end());
	//real minval = *thrust::min_element(res.begin(), res.begin()+n_o_c);
		real minval = res[thrust::min_element(res.begin(),res.begin()+n_o_c)-res.begin()];
		return fmax(real(0.0),-minval);
	}

#endif
