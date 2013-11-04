//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//
// File authors: Andrea Favali

#ifndef CHGAUSSINTEGRATIONRULE
#define CHGAUSSINTEGRATIONRULE


#include "ChGaussPoint.h"

namespace chrono
{
namespace fem
{

	
/// Class for the management of the Gauss Quadrature in 1D, 2D or 3D space.
/// Integration is done over the canonical interval (-1,...,+1), so the position of
/// each gauss point is expressed in terms of natural coordinates.
/// Input: number of gauss points, pointer to the vector 'GpVector'. 
/// The vector will be resized according to the number of points required.

class  ChGaussIntegrationRule
{
private:
	

public:
	ChGaussIntegrationRule();
	~ChGaussIntegrationRule();
	
	virtual void SetIntOnLine(int nPoints, std::vector<ChGaussPoint*>* GpVector){};
	virtual void SetIntOnTriangle(int nPoints, std::vector<ChGaussPoint*>* GpVector);
	virtual void SetIntOnSquare(int nPoints, std::vector<ChGaussPoint*>* GpVector);
	virtual void SetIntOnCube(int nPoints, std::vector<ChGaussPoint*>* GpVector);

};



}//__end mamespace fem
}//__end namespace chrono

#endif
