#ifndef UTILS_CUH
#define UTILS_CUH

/* C/C++ Standard Library*/
#include <iostream>

/* Thrust Library */
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>

/* Chrono::FSI Library */
#include "../custom_math.h"

namespace chrono {
namespace fsi {
const void printStruct(struct Real2 &s)
{
    std::cout << "x = " << s.x << ", ";
    std::cout << "y = " << s.y << ", " << std::endl;
}
const void printStruct(struct int2 &s)
{
    std::cout << "x = " << s.x << ", ";
    std::cout << "y = " << s.y << ", " << std::endl;
}
const void printStruct(struct Real3 &s)
{
    std::cout << "x = " << s.x << ", ";
    std::cout << "y = " << s.y << ", ";
    std::cout << "z = " << s.z << ", " << std::endl;
}
const void printStruct(struct int3 &s)
{
    std::cout << "x = " << s.x << ", ";
    std::cout << "y = " << s.y << ", ";
    std::cout << "z = " << s.z << ", " << std::endl;
}
const void printStruct(struct Real4 &s)
{
    std::cout << "x = " << s.x << ", ";
    std::cout << "y = " << s.y << ", ";
    std::cout << "z = " << s.z << ", ";
    std::cout << "w = " << s.w << ", " << std::endl;
}
const void printStruct(struct int4 &s)
{
    std::cout << "x = " << s.x << ", ";
    std::cout << "y = " << s.y << ", ";
    std::cout << "z = " << s.z << ", ";
    std::cout << "w = " << s.w << ", " << std::endl;
}
/** Host Vecttor functions */
const void printThrustVector(thrust::host_vector<int3> &v)
{
    std::cout << "Thrust host vector: " << std::endl;
    for(int i = 0; i < v.size(); i++) {
        std::cout << "[" << i << "]" << std::endl;
        printStruct(v[i]);
    }
}
const void printThrustVector(thrust::host_vector<Real3> &v)
{
    std::cout << "Thrust host vector: " << std::endl;
    for(int i = 0; i < v.size(); i++) {
        std::cout << "[" << i << "]" << std::endl;
        printStruct(v[i]);
    }
}
const void printThrustVector(thrust::host_vector<int4> &v)
{
    std::cout << "Thrust host vector: " << std::endl;
    for(int i = 0; i < v.size(); i++) {
        std::cout << "[" << i << "]" << std::endl;
        printStruct(v[i]);
    }
}
const void printThrustVector(thrust::host_vector<Real4> &v)
{
    std::cout << "Thrust host vector: " << std::endl;
    for(int i = 0; i < v.size(); i++) {
        std::cout << "[" << i << "]" << std::endl;
        printStruct(v[i]);
    }
}

} // end namespace fsi
} // end namespace chrono


#endif

