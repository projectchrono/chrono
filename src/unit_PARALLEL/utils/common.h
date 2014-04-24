#ifndef CH_UTILS_COMMON_H
#define CH_UTILS_COMMON_H

#include <random>

#include "core/ChPlatform.h"
#include "physics/ChSystem.h"
#include "physics/ChSystemDEM.h"

#include "ChSystemParallel.h"


// -------------------------------------------------------------------------------
// DLL define
// -------------------------------------------------------------------------------
#if defined(CH_API_COMPILE_UTILS)
#define CH_UTILS_API ChApiEXPORT
#else
#define CH_UTILS_API ChApiINPORT
#endif


namespace chrono {
namespace utils {


enum SystemType {
	SEQUENTIAL_DVI,
	SEQUENTIAL_DEM,
	PARALLEL_DVI,
	PARALLEL_DEM
};

// -------------------------------------------------------------------------------
// Construct a single random engine (on first use)
//
// Note that this object is never destructed (but this is OK)
// -------------------------------------------------------------------------------
inline
std::default_random_engine& rengine()
{
	static std::default_random_engine* re = new std::default_random_engine(std::random_device());
	return *re;
}

// -------------------------------------------------------------------------------
// sampleTruncatedDist
//
// Utility function for generating samples from a truncated normal distribution.
// -------------------------------------------------------------------------------
template <typename T>
inline
T sampleTruncatedDist(std::normal_distribution<T>& distribution,
                      T                            minVal,
                      T                            maxVal)
{
	T val;

	do {
		val = distribution(rengine());
	} while (val < minVal || val > maxVal);

	return val;
}

// -------------------------------------------------------------------------------
// GetSystemType()
//
// This utility function infers the type of the specified ChSystem.
// -------------------------------------------------------------------------------
inline
SystemType GetSystemType(ChSystem* system)
{
	if (dynamic_cast<ChSystemParallelDVI*>(system))
		return PARALLEL_DVI;

	if (dynamic_cast<ChSystemParallelDEM*>(system))
		return PARALLEL_DEM;
	
	if (dynamic_cast<ChSystemDEM*>(system))
		return SEQUENTIAL_DEM;

	return SEQUENTIAL_DVI;
}


} // end namespace utils
} // end namespace chrono


#endif