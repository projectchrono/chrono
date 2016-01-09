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
// Author: Arman Pazouki
// =============================================================================
//
// Base class for processin collision in fsi system.//
// =============================================================================

#ifndef CH_COLLISIONSYSTEM_FSI_H_
#define CH_COLLISIONSYSTEM_FSI_H_

namespace fsi {

class CH_FSI_API ChCollisionSystemFsi {
	public:
		void setParameters();
		void calcHash();
		void reorderDataAndFindCellStart();
		void collide();

	private:
		friend class chrono::fsi::ChSystemFsi;


};
}

#endif /* CH_COLLISIONSYSTEM_FSI_H_ */
