/*
 * InitializeSphMarkers.cpp
 *
 *  Created on: Mar 2, 2015
 *      Author: Arman Pazouki
 */

#include <string>
#include "chrono_fsi/InitializeSphMarkers.h"
#include "CreateBCE.h"
#include "chrono_fsi/SphInterface.h" //for convert stuff such as ConvertRealToChVector

//#include "chrono_utils/ChUtilsVehicle.h"
#include "utils/ChUtilsGeometry.h"
#include "utils/ChUtilsCreators.h"
#include "utils/ChUtilsGenerators.h"

//**********************************************

void SetNumObjects(NumberOfObjects& numObjects,
		const thrust::host_vector<int4>& referenceArray, int numAllMarkers) {
	numObjects.numFluidMarkers = (referenceArray[0]).y - (referenceArray[0]).x;
	numObjects.numBoundaryMarkers = (referenceArray[1]).y
			- (referenceArray[1]).x;
	numObjects.numAllMarkers = numAllMarkers;

	numObjects.numRigidBodies = 0;
	numObjects.numRigid_SphMarkers = 0;
	numObjects.numFlex_SphMarkers = 0;
	std::cout << "********************" << std::endl;
	std::cout << "numFlexBodies: " << numObjects.numFlexBodies << std::endl;
	std::cout << "numRigidBodies: " << numObjects.numRigidBodies << std::endl;
	std::cout << "numFluidMarkers: " << numObjects.numFluidMarkers << std::endl;
	std::cout << "numBoundaryMarkers: " << numObjects.numBoundaryMarkers
			<< std::endl;
	std::cout << "numRigid_SphMarkers: " << numObjects.numRigid_SphMarkers
			<< std::endl;
	std::cout << "numFlex_SphMarkers: " << numObjects.numFlex_SphMarkers
			<< std::endl;
	std::cout << "numAllMarkers: " << numObjects.numAllMarkers << std::endl;
	std::cout << "********************" << std::endl;
}

//**********************************************
//bool is_number(const std::string& s) {
//  std::string::const_iterator it = s.begin();
//  while (it != s.end() && std::isdigit(*it))
//    ++it;
//  return !s.empty() && it == s.end();
//}
//**********************************************
int2 CreateFluidMarkers(thrust::host_vector<Real3>& posRadH,
		thrust::host_vector<Real4>& velMasH,
		thrust::host_vector<Real4>& rhoPresMuH,
		thrust::host_vector<uint>& bodyIndex, const SimParams& paramsH,
		Real& sphMarkerMass) {
	/* Number of fluid particles */
	int num_FluidMarkers = 0;
	/* Number of boundary particles */
	int num_BoundaryMarkers = 0;
	srand(964);
	/* Initial separation of both fluid and boundary particles */
	Real initSpace0 = paramsH.MULT_INITSPACE * paramsH.HSML;
	/* Number of particles per x dimension */
	int nFX = ceil((paramsH.cMaxInit.x - paramsH.cMinInit.x) / (initSpace0));
	/* Spacing between center of particles in x dimension*/
	Real initSpaceX = (paramsH.cMaxInit.x - paramsH.cMinInit.x) / nFX;
	/* Number of particles per y dimension */
	int nFY = ceil((paramsH.cMaxInit.y - paramsH.cMinInit.y) / (initSpace0));
	/* Spacing between center of particles in y dimension*/
	Real initSpaceY = (paramsH.cMaxInit.y - paramsH.cMinInit.y) / nFY;
	/* Number of particles per z dimension */
	int nFZ = ceil((paramsH.cMaxInit.z - paramsH.cMinInit.z) / (initSpace0));
	/* Spacing between center of particles in z dimension*/
	Real initSpaceZ = (paramsH.cMaxInit.z - paramsH.cMinInit.z) / nFZ;
	printf("nFX Y Z %d %d %d, max distY %f, initSpaceY %f\n", nFX, nFY, nFZ,
			(nFY - 1) * initSpaceY, initSpaceY);
	/* Mass of a small cube in the fluid = (dx*dy*dz) * density */
	sphMarkerMass = (initSpaceX * initSpaceY * initSpaceZ) * paramsH.rho0;

	for (int i = 0; i < nFX; i++) {
		for (int j = 0; j < nFY; j++) {
			for (int k = 0; k < nFZ; k++) {
				Real3 posRad;
				//					printf("initSpace X, Y, Z %f %f %f \n", initSpaceX, initSpaceY,
				// initSpaceZ);
				posRad =
						paramsH.cMinInit
								+ mR3(i * initSpaceX, j * initSpaceY,
										k * initSpaceZ)
								+
								mR3(
										.5
												* initSpace0) /* + mR3(sphR) + initSpace * .05 * (Real(rand()) / RAND_MAX)*/;
				if ((posRad.x > paramsH.straightChannelBoundaryMin.x
						&& posRad.x < paramsH.straightChannelBoundaryMax.x)
						&& (posRad.y > paramsH.straightChannelBoundaryMin.y
								&& posRad.y
										< paramsH.straightChannelBoundaryMax.y)
						&& (posRad.z > paramsH.straightChannelBoundaryMin.z
								&& posRad.z
										< paramsH.straightChannelBoundaryMax.z)) {
					if (i < nFX) {
						num_FluidMarkers++;
						posRadH.push_back(posRad);
						Real3 v3 = mR3(0);
						velMasH.push_back(mR4(v3, sphMarkerMass));
						rhoPresMuH.push_back(
						mR4(paramsH.rho0, paramsH.BASEPRES, paramsH.mu0, -1));
					}
				} else {
					//					num_BoundaryMarkers++;
					//					mPosRadBoundary.push_back(posRad);
					//					mVelMasBoundary.push_back(mR4(0, 0, 0, sphMarkerMass));
					//					mRhoPresMuBoundary.push_back(mR4(paramsH.rho0, paramsH.LARGE_PRES,
					// paramsH.mu0,
					// 0));
				}
			}
		}
	}
	int2 num_fluidOrBoundaryMarkers = mI2(num_FluidMarkers,
			num_BoundaryMarkers);
	// *** copy boundary markers to the end of the markers arrays
	posRadH.resize(num_fluidOrBoundaryMarkers.x + num_fluidOrBoundaryMarkers.y);
	velMasH.resize(num_fluidOrBoundaryMarkers.x + num_fluidOrBoundaryMarkers.y);
	rhoPresMuH.resize(
			num_fluidOrBoundaryMarkers.x + num_fluidOrBoundaryMarkers.y);

	int numAllMarkers = num_fluidOrBoundaryMarkers.x
			+ num_fluidOrBoundaryMarkers.y;
	bodyIndex.resize(numAllMarkers);
	thrust::fill(bodyIndex.begin(), bodyIndex.end(), 1);
	thrust::exclusive_scan(bodyIndex.begin(), bodyIndex.end(),
			bodyIndex.begin());

	return num_fluidOrBoundaryMarkers;
}
// =============================================================================
// left off here
// TransformToCOG
// This utility function converts a given position and orientation, specified
// with respect to a body's reference frame, into a frame defined with respect
// to the body's centroidal frame.  Note that by default, a body's reference
// frame is the centroidal frame. This is not true for a ChBodyAuxRef.
void TransformBceFrameToCOG(chrono::ChBody* body, const chrono::ChVector<>& pos,
		const chrono::ChMatrix33<>& rot, chrono::ChFrame<>& frame) {
	frame = chrono::ChFrame<>(pos, rot);
	if (chrono::ChBodyAuxRef* body_ar =
			dynamic_cast<chrono::ChBodyAuxRef*>(body)) {
		frame = frame >> body_ar->GetFrame_REF_to_COG();
	}
}

chrono::ChVector<> TransformBCEToCOG(chrono::ChBody* body, const Real3 & pos3) {
	chrono::ChVector<> pos = ConvertRealToChVector(pos3);
	chrono::ChFrame<> frame;
	TransformBceFrameToCOG(body, pos, chrono::QUNIT, frame);
	return frame.GetPos();
}
// =============================================================================
void CreateBceGlobalMarkersFromBceLocalPos(thrust::host_vector<Real3>& posRadH,
		thrust::host_vector<Real4>& velMasH,
		thrust::host_vector<Real4>& rhoPresMuH,
		thrust::host_vector<::int4>& referenceArray,
		NumberOfObjects& numObjects,
		const thrust::host_vector<Real3>& posRadBCE, Real sphMarkerMass,
		const SimParams& paramsH, chrono::ChSharedPtr<chrono::ChBody> body,
		bool isSolid) {
	if (referenceArray.size() < 1) {
		printf(
				"\n\n\n\n Error! fluid need to be initialized before boundary. Reference array should have two "
						"components \n\n\n\n");
		std::cin.get();
	}
	::int4 refSize4 = referenceArray[referenceArray.size() - 1];
	int type = 0;
	if (isSolid) {
		type = refSize4.w + 1;
	}
	if (type < 0) {
		printf(
				"\n\n\n\n Error! reference array type is not correct. It does not denote boundary or rigid \n\n\n\n");
		std::cin.get();
	} else if (type > 0 && (referenceArray.size() - 1 != type)) {
		printf(
				"\n\n\n\n Error! reference array size does not match type \n\n\n\n");
		std::cin.get();
	}

	//#pragma omp parallel for  // it is very wrong to do it in parallel. race condition will occur
	for (int i = 0; i < posRadBCE.size(); i++) {
		chrono::ChVector<> position = TransformBCEToCOG(body.get(),
				posRadBCE[i]);

		chrono::ChVector<> posParent =
				chrono::ChTransform<>::TransformLocalToParent(position,
						body->GetPos(), body->GetRot());

		printf("pos %f %f %f , rot %f %f %f %f \n", body->GetPos().x, body->GetPos().y, body->GetPos().z,
				body->GetRot().e0, body->GetRot().e1, body->GetRot().e2, body->GetRot().e3);
		std::cin.get();
		posRadH.push_back(ConvertChVectorToR3(posParent));

		chrono::ChVector<> pointPar = ConvertRealToChVector(posRadBCE[i]);
		chrono::ChVector<> posLoc =
				chrono::ChTransform<>::TransformParentToLocal(pointPar,
						body->GetPos(), body->GetRot());
		chrono::ChVector<> vAbs = body->PointSpeedLocalToParent(posLoc);
		Real3 v3 = ConvertChVectorToR3(vAbs);
		velMasH.push_back(mR4(v3, sphMarkerMass));

		rhoPresMuH.push_back(
		mR4(paramsH.rho0, paramsH.BASEPRES, paramsH.mu0, type));
	}

	// ------------------------
	// Modify number of objects
	// ------------------------

	int numBce = posRadBCE.size();
	numObjects.numAllMarkers += numBce;
	if (type == 0) {
		numObjects.numBoundaryMarkers += numBce;
		if (referenceArray.size() == 1) {
			referenceArray.push_back(
			mI4(refSize4.y, refSize4.y + numBce, 0, 0));
		} else if (referenceArray.size() == 2) {
			refSize4.y = refSize4.y + numBce;
			referenceArray[1] = refSize4;
		} else {
			printf(
					"Error! reference array size is greater than 2 while marker type is 0 \n\n");
			std::cin.get();
		}
	} else {
		if (referenceArray.size() < 2) {
			printf(
					"Error! Boundary markers are not initialized while trying to initialize rigid marker!\n\n");
			std::cin.get();
		}
		numObjects.numRigid_SphMarkers += numBce;
		numObjects.numRigidBodies += 1;
		numObjects.startRigidMarkers = referenceArray[1].y;
		referenceArray.push_back(
		mI4(refSize4.y, refSize4.y + numBce, 1, type)); // 1: for rigid
		if (numObjects.numRigidBodies != referenceArray.size() - 2) {
			printf(
					"Error! num rigid bodies does not match reference array size!\n\n");
			std::cin.get();
		}
	}

	//	SetNumObjects(numObjects, referenceArray, numAllMarkers);
}
// =============================================================================
void CreateBceGlobalMarkersFromBceLocalPosBoundary(
		thrust::host_vector<Real3>& posRadH,
		thrust::host_vector<Real4>& velMasH,
		thrust::host_vector<Real4>& rhoPresMuH,
		thrust::host_vector<::int4>& referenceArray,
		NumberOfObjects& numObjects,
		const thrust::host_vector<Real3>& posRadBCE, Real sphMarkerMass,
		const SimParams& paramsH, chrono::ChSharedPtr<chrono::ChBody> body) {

	CreateBceGlobalMarkersFromBceLocalPos(posRadH, velMasH, rhoPresMuH,
			referenceArray, numObjects, posRadBCE, sphMarkerMass, paramsH, body,
			false);
}
// =============================================================================
void AddSphereBceToChSystemAndSPH(
		thrust::host_vector<Real3>& posRadH, // do not set the size here since you are using push back later
		thrust::host_vector<Real4>& velMasH,
		thrust::host_vector<Real4>& rhoPresMuH,
		thrust::host_vector<::int4>& referenceArray,
		NumberOfObjects& numObjects, Real sphMarkerMass,
		const SimParams& paramsH, chrono::ChSharedPtr<chrono::ChBody> body,
		Real radius, chrono::ChVector<> relPos, chrono::ChQuaternion<> relRot) {

	chrono::utils::AddSphereGeometry(body.get_ptr(), radius);
	thrust::host_vector<Real3> posRadBCE;
	CreateBCE_On_Sphere(posRadBCE, radius, paramsH);

	if (posRadH.size() != numObjects.numAllMarkers) {
		printf("Error! numMarkers, %d, does not match posRadH.size(), %d\n",
				numObjects.numAllMarkers, posRadH.size());
		std::cin.get();
	}

	CreateBceGlobalMarkersFromBceLocalPos(posRadH, velMasH, rhoPresMuH,
			referenceArray, numObjects, posRadBCE, sphMarkerMass, paramsH,
			body);
	posRadBCE.clear();

}
// =============================================================================

void AddCylinderBceToChSystemAndSPH(
		thrust::host_vector<Real3>& posRadH, // do not set the size here since you are using push back later
		thrust::host_vector<Real4>& velMasH,
		thrust::host_vector<Real4>& rhoPresMuH,
		thrust::host_vector<::int4>& referenceArray,
		NumberOfObjects& numObjects, Real sphMarkerMass,
		const SimParams& paramsH, chrono::ChSharedPtr<chrono::ChBody> body,
		Real radius, Real height, chrono::ChVector<> relPos,
		chrono::ChQuaternion<> relRot) {
	//

	// Arman move to model file
//				chrono::ChSharedPtr<chrono::ChBody> body = chrono::ChSharedPtr<
//						chrono::ChBody>(
//						new chrono::ChBody(
//								new chrono::collision::ChCollisionModelParallel));
//				// body->SetIdentifier(-1);
//				body->SetBodyFixed(false);
//				body->SetCollide(true);
//
//				//Arman, move out specific chrono stuff
//				Real mu_g = .1;
//				body->GetMaterialSurface()->SetFriction(mu_g);
//				body->SetPos(pos);
//				body->SetRot(rot);
//				//    body->SetWvel_par(ChVector<>(0, 10, 0));
//				double volume = chrono::utils::CalcCylinderVolume(radius, 0.5 * height);
//				chrono::ChVector<> gyration = chrono::utils::CalcCylinderGyration(radius,
//						0.5 * height).Get_Diag();
//				double density = paramsH.rho0;
//				double mass = density * volume;
//				body->SetMass(mass);
//				body->SetInertiaXX(mass * gyration);
//				//
//				body->GetCollisionModel()->ClearModel();
//				chrono::utils::AddCylinderGeometry(body.get_ptr(), radius, 0.5 * height);
//				body->GetCollisionModel()->BuildModel();
//				mphysicalSystem.AddBody(body);
	//

	chrono::utils::AddCylinderGeometry(body.get_ptr(), radius, 0.5 * height);
	thrust::host_vector<Real3> posRadBCE;
	CreateBCE_On_Cylinder(posRadBCE, radius, height, paramsH);

	if (posRadH.size() != numObjects.numAllMarkers) {
		printf("Error! numMarkers, %d, does not match posRadH.size(), %d\n",
				numObjects.numAllMarkers, posRadH.size());
		std::cin.get();
	}

	CreateBceGlobalMarkersFromBceLocalPos(posRadH, velMasH, rhoPresMuH,
			referenceArray, numObjects, posRadBCE, sphMarkerMass, paramsH,
			body);
	posRadBCE.clear();

//	referenceArray.push_back(mI4(numMarkers, numMarkers + numBce, 1, type)); // 1: for rigid
//	numObjects.numRigidBodies += 1;
//	numObjects.startRigidMarkers = numMarkers; // Arman : not sure if you need to set startFlexMarkers
//	numObjects.numRigid_SphMarkers += numBce;
//	numObjects.numAllMarkers = posRadH.size();
//	FSI_Bodies.push_back(body);
}

// =============================================================================
// Arman note, the function in the current implementation creates boundary bce (accesses only referenceArray[1])

// Arman thrust::host_vector<uint>& bodyIndex,

void AddBoxBceToChSystemAndSPH(
		thrust::host_vector<Real3>& posRadH, // do not set the size here since you are using push back later
		thrust::host_vector<Real4>& velMasH,
		thrust::host_vector<Real4>& rhoPresMuH,
		thrust::host_vector<::int4>& referenceArray,
		NumberOfObjects& numObjects, Real sphMarkerMass,
		const SimParams& paramsH, chrono::ChSharedPtr<chrono::ChBody> body,
		const chrono::ChVector<>& size, chrono::ChVector<> relPos,
		chrono::ChQuaternion<> relRot) {

	chrono::utils::AddBoxGeometry(body.get_ptr(), size, relPos, relRot, true);
	thrust::host_vector<Real3> posRadBCE;

	CreateBCE_On_Box(posRadBCE, ConvertChVectorToR3(size), 12, paramsH);

	if (posRadH.size() != numObjects.numAllMarkers) {
		printf("Error! numMarkers, %d, does not match posRadH.size(), %d\n",
				numObjects.numAllMarkers, posRadH.size());
		std::cin.get();
	}

	CreateBceGlobalMarkersFromBceLocalPosBoundary(posRadH, velMasH, rhoPresMuH,
			referenceArray, numObjects, posRadBCE, sphMarkerMass, paramsH,
			body);
	posRadBCE.clear();

//	int type = 0; //boundary
//	CreateBceGlobalMarkersFromBceLocalPos(posRadH, velMasH, rhoPresMuH, posRadBCE, type,
//			sphMarkerMass, paramsH, body);
//	int numBce = posRadBCE.size();
//	int numSaved = posRadH.size();
//
//	::int4 ref4 = referenceArray[1];
//	ref4.y = ref4.y + numBCE;
//	referenceArray[1] = ref4;
//
//	int numAllMarkers = numBCE + numSaved;
//	SetNumObjects(numObjects, referenceArray, numAllMarkers);

}

// =============================================================================
void AddBCE2FluidSystem_FromFile(
		thrust::host_vector<Real3>& posRadH, // do not set the size here since you are using push back later
		thrust::host_vector<Real4>& velMasH,
		thrust::host_vector<Real4>& rhoPresMuH,
		thrust::host_vector<::int4>& referenceArray,
		NumberOfObjects& numObjects, Real sphMarkerMass,
		const SimParams& paramsH, chrono::ChSharedPtr<chrono::ChBody> body,
		std::string dataPath) {
	//----------------------------
	//  chassis
	//----------------------------
	thrust::host_vector<Real3> posRadBCE;

	LoadBCE_fromFile(posRadBCE, dataPath);
	if (posRadH.size() != numObjects.numAllMarkers) {
		printf("Error! numMarkers, %d, does not match posRadH.size(), %d\n",
				numObjects.numAllMarkers, posRadH.size());
		std::cin.get();
	}

	CreateBceGlobalMarkersFromBceLocalPos(posRadH, velMasH, rhoPresMuH,
			referenceArray, numObjects, posRadBCE, sphMarkerMass, paramsH,
			body);
	posRadBCE.clear();
}
