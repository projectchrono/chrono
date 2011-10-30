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
#include "ChForceSolverGPU.h"
#include "ChGPUDataManager.h"
#include <stdio.h>
#include <cutil.h>
#include "ChBodyGPU.h"

namespace chrono {
	class ChApiGPU ChForceSystemGPU {
		public:

			ChForceSystemGPU();
			void Init();
			void Solve();

			ChForceSolverGPU * solver;
		private:
			int param;
	};
}

#endif
