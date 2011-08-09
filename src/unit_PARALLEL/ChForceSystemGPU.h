#ifndef CH_FORCESYSTEMGPU_H
#define CH_FORCESYSTEMGPU_H

#include "ChCuda.h"
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <math.h>
#include <float.h>
#include <algorithm>
#include <cutil_math.h>
#include "lcp/ChLcpSystemDescriptor.h"
#include "ChForceSolverGPU.h"
#include <stdio.h>
#include <cutil.h>
#include "ChBodyGPU.h"
#include "physics/ChBody.h"

namespace chrono{
class ChApiGPU ChForceSystemGPU
	{
	public:

	ChForceSystemGPU();
	void Init();
	void Solve(ChLcpSystemDescriptor& sysd);

	ChForceSolverGPU mGPUSolver;

	private:


	};
}

#endif
