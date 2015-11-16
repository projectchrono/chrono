/*
 * SetupFsiParams.h
 *
 *  Created on: Mar 2, 2015
 *      Author: Arman Pazouki
 */

#ifndef BALLDROPPARAMS_H_
#define BALLDROPPARAMS_H_
#include "MyStructs.cuh"  //just for SimParams

// Duration of the "hold time" (vehicle chassis fixed and no driver inputs).
// This can be used to allow the granular material to settle.
double time_hold = 0;               // 2;
float contact_recovery_speed = 10;  // 0.1;
double time_step = .5e-3;           // 1e-3;

int fluidCollisionFamily = 1;

BceVersion bceType = mORIGINAL;

void SetupParamsH(SimParams& paramsH) {
	//**********************************************
	paramsH.sizeScale = 1;  // don't change it.
	paramsH.HSML = 0.2;
	paramsH.MULT_INITSPACE = 1.0;
	paramsH.NUM_BOUNDARY_LAYERS = 3;
	paramsH.toleranceZone = paramsH.NUM_BOUNDARY_LAYERS
			* (paramsH.HSML * paramsH.MULT_INITSPACE);
	paramsH.BASEPRES = 0;        // 10;
	paramsH.LARGE_PRES = 10000;  // paramsH.BASEPRES;//10000;
	paramsH.deltaPress;          //** modified below
	paramsH.multViscosity_FSI = 5.0;
	paramsH.gravity = mR3(0, -9.81, 0);  // mR3(0);//mR3(0, -9.81, 0);
	paramsH.bodyForce3 = mR3(0, 0, 0); // mR4(3.2e-3,0,0,0);// mR4(0);;// /*Re = 100 */ //mR4(3.2e-4, 0, 0, 0);/*Re = 100 */
	paramsH.rho0 = 1000;
	paramsH.mu0 = .001;
	paramsH.v_Max = contact_recovery_speed; // Arman, I changed it to 0.1 for vehicle. Check this
											// later;//10;//50e-3;//18e-3;//1.5;//2e-1; /*0.2 for Re = 100 */ //2e-3;
	paramsH.EPS_XSPH = .5f;
	paramsH.dT = time_step;  // 0.0005;//0.1;//.001; //sph alone: .01 for Re 10;
	paramsH.tFinal = 7;             // 20 * paramsH.dT; //400
	paramsH.timePause = time_hold; //.0001 * paramsH.tFinal;//.0001 * paramsH.tFinal; 	// time before applying any
	// bodyforce. Particles move only due to initialization. keep it as small as possible.
	// the time step will be 1/10 * dT.
	paramsH.kdT = 5;  // I don't know what is kdT
	paramsH.gammaBB = 0.5;
	// ************
	paramsH.binSize0;           // will be changed
	paramsH.rigidRadius;        // will be changed
	paramsH.densityReinit = 0; // 0: no re-initialization, 1: with initialization
	//****************************************************************************************
	//*** initialize straight channel
	paramsH.straightChannelBoundaryMin = mR3(0, 0, 0);  // 3D channel
	paramsH.straightChannelBoundaryMax = mR3(3, 2, 3) * paramsH.sizeScale;
	//********************************************************************************************************
	//**  reminiscent of the past******************************************************************************
	//	paramsH.cMin = mR3(-paramsH.toleranceZone, -paramsH.toleranceZone, -paramsH.toleranceZone);
	//// 3D channel
	//	paramsH.cMax = mR3( 3  + paramsH.toleranceZone, 2 + paramsH.toleranceZone,  3 + paramsH.toleranceZone);
	paramsH.cMin = mR3(0, 0, 0);  // 3D channel
	paramsH.cMax = mR3(3, 2, 3);

	//****************************************************************************************
	paramsH.cMinInit = mR3(0, 0, 0);
	paramsH.cMaxInit = mR3(3, 2, 3);
	//****************************************************************************************
	// printf("a1  paramsH.cMax.x, y, z %f %f %f,  binSize %f\n", paramsH.cMax.x, paramsH.cMax.y, paramsH.cMax.z, 2 *
	// paramsH.HSML);
	int3 side0 = mI3(
			floor((paramsH.cMax.x - paramsH.cMin.x) / (2 * paramsH.HSML)),
			floor((paramsH.cMax.y - paramsH.cMin.y) / (2 * paramsH.HSML)),
			floor((paramsH.cMax.z - paramsH.cMin.z) / (2 * paramsH.HSML)));
	Real3 binSize3 = mR3((paramsH.cMax.x - paramsH.cMin.x) / side0.x,
			(paramsH.cMax.y - paramsH.cMin.y) / side0.y,
			(paramsH.cMax.z - paramsH.cMin.z) / side0.z);
	paramsH.binSize0 = (binSize3.x > binSize3.y) ? binSize3.x : binSize3.y;
	//	paramsH.binSize0 = (paramsH.binSize0 > binSize3.z) ? paramsH.binSize0 : binSize3.z;
	paramsH.binSize0 = binSize3.x; // for effect of distance. Periodic BC in x direction. we do not care about paramsH.cMax y and z.
	paramsH.cMax = paramsH.cMin + paramsH.binSize0 * mR3(side0);
	paramsH.boxDims = paramsH.cMax - paramsH.cMin;
	//************************** modify pressure ***************************
	//		paramsH.deltaPress = paramsH.rho0 * paramsH.boxDims * paramsH.bodyForce3;  //did not work as I expected
	paramsH.deltaPress = 0.9 * paramsH.boxDims * paramsH.bodyForce3;

	// modify bin size stuff
	//****************************** bin size adjustement and contact detection stuff *****************************
	int3 SIDE = mI3(
			int((paramsH.cMax.x - paramsH.cMin.x) / paramsH.binSize0 + .1),
			int((paramsH.cMax.y - paramsH.cMin.y) / paramsH.binSize0 + .1),
			int((paramsH.cMax.z - paramsH.cMin.z) / paramsH.binSize0 + .1));
	Real mBinSize = paramsH.binSize0; // Best solution in that case may be to change cMax or cMin such that periodic
									  // sides be a multiple of binSize
	//**********************************************************************************************************
	paramsH.gridSize = SIDE;
	// paramsH.numCells = SIDE.x * SIDE.y * SIDE.z;
	paramsH.worldOrigin = paramsH.cMin;
	paramsH.cellSize = mR3(mBinSize, mBinSize, mBinSize);

	//***** print numbers
	printf(
			"********************\n paramsH.sizeScale: %f\n paramsH.HSML: %f\n paramsH.bodyForce3: %f %f %f\n "
					"paramsH.gravity: %f %f %f\n paramsH.rho0: %e\n paramsH.mu0: %f\n paramsH.v_Max: %f\n paramsH.dT: %e\n "
					"paramsH.tFinal: %f\n  paramsH.timePause: %f\n  paramsH.timePauseRigidFlex: %f\n paramsH.densityReinit: %d\n",
			paramsH.sizeScale, paramsH.HSML, paramsH.bodyForce3.x,
			paramsH.bodyForce3.y, paramsH.bodyForce3.z, paramsH.gravity.x,
			paramsH.gravity.y, paramsH.gravity.z, paramsH.rho0, paramsH.mu0,
			paramsH.v_Max, paramsH.dT, paramsH.tFinal, paramsH.timePause,
			paramsH.timePauseRigidFlex, paramsH.densityReinit);
	printf(" paramsH.cMin: %f %f %f, paramsH.cMax: %f %f %f\n binSize: %f\n",
			paramsH.cMin.x, paramsH.cMin.y, paramsH.cMin.z, paramsH.cMax.x,
			paramsH.cMax.y, paramsH.cMax.z, paramsH.binSize0);
	printf(" paramsH.MULT_INITSPACE: %f\n", paramsH.MULT_INITSPACE);
	printf(
			" paramsH.NUM_BOUNDARY_LAYERS: %d\n paramsH.toleranceZone: %f\n paramsH.NUM_BCE_LAYERS: %d\n "
					"paramsH.solidSurfaceAdjust: %f\n",
			paramsH.NUM_BOUNDARY_LAYERS, paramsH.toleranceZone,
			paramsH.NUM_BCE_LAYERS, paramsH.solidSurfaceAdjust);
	printf(
			" paramsH.BASEPRES: %f\n paramsH.LARGE_PRES: %f\n paramsH.deltaPress: %f %f %f\n",
			paramsH.BASEPRES, paramsH.LARGE_PRES, paramsH.deltaPress.x,
			paramsH.deltaPress.y, paramsH.deltaPress.z);
	printf(
			" paramsH.nPeriod: %d\n paramsH.EPS_XSPH: %f\n paramsH.multViscosity_FSI: %f\n paramsH.rigidRadius: %f\n",
			paramsH.nPeriod, paramsH.EPS_XSPH, paramsH.multViscosity_FSI,
			paramsH.rigidRadius);
	printf("boxDims: %f, %f, %f\n", paramsH.boxDims.x, paramsH.boxDims.y,
			paramsH.boxDims.z);
	printf("SIDE: %d, %d, %d\n", paramsH.gridSize.x, paramsH.gridSize.y,
			paramsH.gridSize.z);
}

#endif
