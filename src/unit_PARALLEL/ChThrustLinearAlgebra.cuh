#ifndef CHTHRUSTLINEARALGEBRA_H
#define CHTHRUSTLINEARALGEBRA_H

#include "ChCudaMath.h"

struct saxpy_functor : public thrust::binary_function<real, real, real> {
        const real a;

        saxpy_functor(real _a) : a(_a) {}

        __host__ __device__
        float operator()(const real &x, const real &y) const {
                return a * x + y;
        }
};

struct saxmy_functor : public thrust::binary_function<real, real, real> {
        const real a;

        saxmy_functor(real _a) : a(_a) {}

        __host__ __device__
        float operator()(const real &x, const real &y) const {
                return y - a * x;
        }
};

static void SEAXPY(const real &a, custom_vector<real> &x, custom_vector<real> &y, custom_vector<real> &output)
{
        thrust::transform(x.begin(), x.end(), y.begin(), output.begin(), saxpy_functor(a));
}


static custom_vector<real> operator +(const custom_vector<real> &x, const custom_vector<real> &y)
{
        custom_vector<real> temp(x.size());
        thrust::plus<real> op;
        thrust::transform(x.begin(), x.end(), y.begin(), temp.begin(), op);
        return temp;
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
static custom_vector<real> operator /(const custom_vector<real> &x, const custom_vector<real> &y)
{
        custom_vector<real> temp(x.size());
        thrust::divides<real> op;
        thrust::transform(x.begin(), x.end(), y.begin(), temp.begin(), op);
        return temp;
}

static real Dot(const custom_vector<real> &x, const custom_vector<real> &y)
{
        thrust::plus<real>       binary_op1;
        thrust::multiplies<real> binary_op2;
        real answer = thrust::inner_product(x.begin(), x.end(), y.begin(), real(0.0), binary_op1, binary_op2);
        return answer;
}


#endif
