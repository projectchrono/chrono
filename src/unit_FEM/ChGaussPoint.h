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

#ifndef CHGAUSSPOINT
#define CHGAUSSPOINT


#include "physics/ChTensors.h"
#include "core/ChMath.h"
#include "lcp/ChLcpSystemDescriptor.h"


namespace chrono
{
namespace fem
{

/// Class for a gauss point, that has a position (1D-3D) and a weight.
/// It also contains the strain and the stress tensor.

class ChGaussPoint
{
private:
		/// Number.
    int number;
		/// Reference to parent integration rule
    //ChIntegrationRule *irule;
	    /// Local (natural) coordinates of the point
    ChVector<> LocalCoordinates;
	    /// Absolute point coordinates
    ChVector<> *coordinates;
	    /// Integration weight
    double weight;

public:		
	
		/// Matrix of partial derivatives: to obtain strain & stress
	ChMatrixDynamic<> *MatrB;
	    /// Strain tensor
    ChStrainTensor<> Strain;
	    /// Stress tensor
    ChStressTensor<> Stress;


			/// Constructor; n number of the gauss point, coordinates of the point, w weight
    ChGaussPoint(/*ChGaussIntegrationRule *ir,*/ int n, ChVector<> *coord, double w);
			/// Destructor
	virtual ~ChGaussPoint();

			/// Returns local coordinates
	ChVector<> GetLocalCoordinates() { return LocalCoordinates; }
	void setLocalCoordinates(ChVector<> &c) { LocalCoordinates = c; }

			/// Returns absolute coordinates
    ChVector<> GetCoordinates() { if ( coordinates ) { return * coordinates; } else { return LocalCoordinates; } }
    void SetCoordinates(ChVector<> &c)
		{ 
			if ( coordinates ) { * coordinates = c; } else { coordinates = new ChVector<>(c); } 
		}

		    /// Returns  integration weight of receiver
    virtual double GetWeight() { return weight; }
    void setWeight(double w) { weight = w; }

		    /// Returns number of the point
    int GetNumber() { return number; }
    
			/// Returns corresponding integration rule to receiver
//    ChIntegrationRule *giveIntegrationRule() { return irule; }
   

};



}//__end mamespace fem
}//__end namespace chrono

#endif