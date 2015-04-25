/*
 * MyStruncts.cuh
 *
 *  Created on: Mar 4, 2015
 *      Author: arman
 */

#ifndef MYSTRUCTS_CUH_
#define MYSTRUCTS_CUH_
#include "custom_cutil_math.h"

enum BceVersion {ADAMI, mORIGINAL};

struct SimParams {
		int3 gridSize;
		Real3 worldOrigin;
		Real3 cellSize;

		uint numBodies;
		Real3 boxDims;

		Real sizeScale;
		Real HSML;
		Real MULT_INITSPACE;
		int NUM_BOUNDARY_LAYERS;
		Real toleranceZone;
		int NUM_BCE_LAYERS;
		Real solidSurfaceAdjust;
		Real BASEPRES;
		Real LARGE_PRES;
		Real3 deltaPress;
		int nPeriod;
		Real3 gravity;
		Real3 bodyForce3;
		Real rho0;
		Real mu0;
		Real v_Max;
		Real EPS_XSPH;
		Real multViscosity_FSI;
		Real dT;
		Real tFinal;
		Real timePause; 			//run the fluid only during this time, with dTm = 0.1 * dT
		Real timePauseRigidFlex; 	//keep the rigid and flex stationary during this time (timePause + timePauseRigidFlex) until the fluid is fully developed
		Real kdT;
		Real gammaBB;
		Real3 cMin;
		Real3 cMax;
		Real3 cMinInit; // Arman : note, this need to be added to check point
		Real3 cMaxInit; // Arman : note, this need to be added to check point
		Real3 straightChannelBoundaryMin;
		Real3 straightChannelBoundaryMax;
		Real binSize0;

		Real3 rigidRadius;
		int densityReinit; //0: no; 1: yes
		int contactBoundary; //0: straight channel, 1: serpentine

};
struct NumberOfObjects {
		int numRigidBodies;
		int numFlexBodies;
		int numFlBcRigid;

		int numFluidMarkers;
		int numBoundaryMarkers;
		int startRigidMarkers;
		int startFlexMarkers;
		int numRigid_SphMarkers;
		int numFlex_SphMarkers;
		int numAllMarkers;
};


#endif /* MYSTRUCTS_CUH_ */
