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
// Class for fsi properties and functions.//
// =============================================================================

#include "ChFsiGeneral.cuh"
namespace chrono {
namespace fsi {

ChFsiGeneral::ChFsiGeneral() : paramsH(NULL), numObjectsH(NULL) {}

ChFsiGeneral::ChFsiGeneral(SimParams* other_paramsH, NumberOfObjects* other_numObjects) :
 paramsH(other_paramsH), numObjectsH(other_numObjects) {}

ChFsiGeneral::~ChFsiGeneral() {}

uint ChFsiGeneral::iDivUp(uint a, uint b) {
	return (a % b != 0) ? (a / b + 1) : (a / b);
}

void ChFsiGeneral::computeGridSize(uint n, uint blockSize, uint& numBlocks,
		uint& numThreads) {
	uint n2 = (n == 0) ? 1 : n;
	numThreads = min(blockSize, n2);
	numBlocks = iDivUp(n2, numThreads);
}

/**
 * @brief calcGridHash
 * @details  See SDKCollisionSystem.cuh
 */
CUDA_CALLABLE_MEMBER int3 ChFsiGeneral::calcGridPos(Real3 p) {
	int3 gridPos;
	gridPos.x = floor((p.x - paramsD.worldOrigin.x) / paramsD.cellSize.x);
	gridPos.y = floor((p.y - paramsD.worldOrigin.y) / paramsD.cellSize.y);
	gridPos.z = floor((p.z - paramsD.worldOrigin.z) / paramsD.cellSize.z);
	return gridPos;
}

/**
 * @brief calcGridHash
 * @details  See SDKCollisionSystem.cuh
 */
CUDA_CALLABLE_MEMBER uint ChFsiGeneral::calcGridHash(int3 gridPos) {
	gridPos.x -= ((gridPos.x >= paramsD.gridSize.x) ? paramsD.gridSize.x : 0);
	gridPos.y -= ((gridPos.y >= paramsD.gridSize.y) ? paramsD.gridSize.y : 0);
	gridPos.z -= ((gridPos.z >= paramsD.gridSize.z) ? paramsD.gridSize.z : 0);

	gridPos.x += ((gridPos.x < 0) ? paramsD.gridSize.x : 0);
	gridPos.y += ((gridPos.y < 0) ? paramsD.gridSize.y : 0);
	gridPos.z += ((gridPos.z < 0) ? paramsD.gridSize.z : 0);

	return __umul24(__umul24(gridPos.z, paramsD.gridSize.y), paramsD.gridSize.x)
			+ __umul24(gridPos.y, paramsD.gridSize.x) + gridPos.x;
}

} // end namespace fsi
} // end namespace chrono