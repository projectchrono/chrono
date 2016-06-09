/*
 * printToFile.cuh
 *
 *  Created on: Mar 2, 2015
 *      Author: Arman Pazouki
 */
#ifndef PRINTTOFILE_CUH
#define PRINTTOFILE_CUH
#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/custom_math.h"
#include "chrono_fsi/UtilsFsi/ChUtilsPrintSph.h"
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>

struct SimParams;


namespace chrono {
namespace fsi {
namespace utils {

//--------------------------------------------------------------------------------------------------------------------------------
CH_FSI_API void PrintToFile(const thrust::device_vector<Real3>& posRadD,
		const thrust::device_vector<Real3>& velMasD,
		const thrust::device_vector<Real4>& rhoPresMuD,
		const thrust::host_vector<int4>& referenceArray,
		const std::string& out_dir);
}  // end namespace utils
}  // end namespace fsi
}  // end namespace chrono

#endif
