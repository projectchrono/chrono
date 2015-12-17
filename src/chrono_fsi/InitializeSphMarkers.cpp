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
	Real initSpace0 = paramsH.MULT_INITSPACE * paramsH.HSML; 	/* Initial separation of both fluid and boundary particles */
	int nFX = ceil((paramsH.cMaxInit.x - paramsH.cMinInit.x) / (initSpace0)); 	/* Number of particles per x dimension */
	Real initSpaceX = (paramsH.cMaxInit.x - paramsH.cMinInit.x) / nFX; 	/* Spacing between center of particles in x dimension*/

	int nFY = ceil((paramsH.cMaxInit.y - paramsH.cMinInit.y) / (initSpace0)); 	/* Number of particles per y dimension */
	Real initSpaceY = (paramsH.cMaxInit.y - paramsH.cMinInit.y) / nFY; 	/* Spacing between center of particles in y dimension*/

	int nFZ = ceil((paramsH.cMaxInit.z - paramsH.cMinInit.z) / (initSpace0)); 	/* Number of particles per z dimension */
	Real initSpaceZ = (paramsH.cMaxInit.z - paramsH.cMinInit.z) / nFZ; 	/* Spacing between center of particles in z dimension*/

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

chrono::ChVector<> TransformBCEToCOG(chrono::ChBody* body,
		const chrono::ChVector<> & pos) {
	chrono::ChFrame<> frame;
	TransformBceFrameToCOG(body, pos, chrono::QUNIT, frame);
	return frame.GetPos();
}

chrono::ChVector<> TransformBCEToCOG(chrono::ChBody* body, const Real3 & pos3) {
	chrono::ChVector<> pos = ConvertRealToChVector(pos3);
	return TransformBCEToCOG(body, pos);
}
// =============================================================================
void CreateBceGlobalMarkersFromBceLocalPos(thrust::host_vector<Real3>& posRadH,
		thrust::host_vector<Real4>& velMasH,
		thrust::host_vector<Real4>& rhoPresMuH,
		thrust::host_vector<::int4>& referenceArray,
		NumberOfObjects& numObjects,
		const thrust::host_vector<Real3>& posRadBCE, Real sphMarkerMass,
		const SimParams& paramsH, chrono::ChSharedPtr<chrono::ChBody> body,
		chrono::ChVector<> collisionShapeRelativePos,
		chrono::ChQuaternion<> collisionShapeRelativeRot, bool isSolid) {
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

		chrono::ChVector<> posLoc_collisionShape = ConvertRealToChVector(
				posRadBCE[i]);
		chrono::ChVector<> posLoc_body =
				chrono::ChTransform<>::TransformLocalToParent(
						posLoc_collisionShape, collisionShapeRelativePos,
						collisionShapeRelativeRot);
		chrono::ChVector<> posLoc_COG = TransformBCEToCOG(body.get(),
				posLoc_body);
		chrono::ChVector<> posGlob =
				chrono::ChTransform<>::TransformLocalToParent(posLoc_COG,
						body->GetPos(), body->GetRot());
		posRadH.push_back(ConvertChVectorToR3(posGlob));

		chrono::ChVector<> vAbs = body->PointSpeedLocalToParent(posLoc_COG);
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
		const SimParams& paramsH, chrono::ChSharedPtr<chrono::ChBody> body,
		chrono::ChVector<> collisionShapeRelativePos,
		chrono::ChQuaternion<> collisionShapeRelativeRot) {

	CreateBceGlobalMarkersFromBceLocalPos(posRadH, velMasH, rhoPresMuH,
			referenceArray, numObjects, posRadBCE, sphMarkerMass, paramsH, body,
			collisionShapeRelativePos, collisionShapeRelativeRot, false);
}
// =============================================================================
void AddSphereBce(
		thrust::host_vector<Real3>& posRadH, // do not set the size here since you are using push back later
		thrust::host_vector<Real4>& velMasH,
		thrust::host_vector<Real4>& rhoPresMuH,
		thrust::host_vector<::int4>& referenceArray,
		NumberOfObjects& numObjects, Real sphMarkerMass,
		const SimParams& paramsH, chrono::ChSharedPtr<chrono::ChBody> body,
		Real radius, chrono::ChVector<> relPos, chrono::ChQuaternion<> relRot) {

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

void AddCylinderBce(
		thrust::host_vector<Real3>& posRadH, // do not set the size here since you are using push back later
		thrust::host_vector<Real4>& velMasH,
		thrust::host_vector<Real4>& rhoPresMuH,
		thrust::host_vector<::int4>& referenceArray,
		NumberOfObjects& numObjects, Real sphMarkerMass,
		const SimParams& paramsH, chrono::ChSharedPtr<chrono::ChBody> body,
		Real radius, Real height, chrono::ChVector<> relPos,
		chrono::ChQuaternion<> relRot) {

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
}

// =============================================================================
// Arman note, the function in the current implementation creates boundary bce (accesses only referenceArray[1])

// Arman thrust::host_vector<uint>& bodyIndex,

void AddBoxBce(
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
			referenceArray, numObjects, posRadBCE, sphMarkerMass, paramsH, body,
			relPos, relRot);
	posRadBCE.clear();
}

// =============================================================================
void AddBCE_FromFile(
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

















// =============================================================================
void CreateSphereFSI(
		thrust::host_vector<Real3>& posRadH, // do not set the size here since you are using push back later
		thrust::host_vector<Real4>& velMasH,
		thrust::host_vector<Real4>& rhoPresMuH,
		thrust::host_vector<::int4>& referenceArray,
		chrono::ChSystem& mphysicalSystem,
		std::vector<chrono::ChSharedPtr<chrono::ChBody> >& FSI_Bodies,
		NumberOfObjects& numObjects, Real sphMarkerMass,
		const SimParams& paramsH,
		Real radius,
		chrono::ChSharedPtr<chrono::ChMaterialSurface> mat_prop,
		Real density,
		chrono::ChVector<> pos) {

//	ChVector<> pos = ChVector<>(-9.5, .20, 3);
//	Real radius = 0.3;


	chrono::ChSharedPtr<chrono::ChBody> body = chrono::ChSharedPtr<
			chrono::ChBody>(
			new chrono::ChBody(
					new chrono::collision::ChCollisionModelParallel));
	body->SetBodyFixed(false);
	body->SetCollide(true);
	body->SetMaterialSurface(mat_prop);
	body->SetPos(pos);
	double volume = chrono::utils::CalcSphereVolume(radius);
	chrono::ChVector<> gyration =
			chrono::utils::CalcSphereGyration(radius).Get_Diag();
	double mass = density * volume;
	body->SetMass(mass);
	body->SetInertiaXX(mass * gyration);
	//
	body->GetCollisionModel()->ClearModel();
	chrono::utils::AddSphereGeometry(body.get_ptr(), radius);
	body->GetCollisionModel()->BuildModel();
	mphysicalSystem.AddBody(body);
	FSI_Bodies.push_back(body);

	AddSphereBce(posRadH, velMasH, rhoPresMuH, referenceArray,
			numObjects, sphMarkerMass, paramsH, body, radius);
}
// =============================================================================
void CreateCylinderFSI(
		thrust::host_vector<Real3>& posRadH, // do not set the size here since you are using push back later
		thrust::host_vector<Real4>& velMasH,
		thrust::host_vector<Real4>& rhoPresMuH,
		thrust::host_vector<::int4>& referenceArray,
		chrono::ChSystem& mphysicalSystem,
		std::vector<chrono::ChSharedPtr<chrono::ChBody> >& FSI_Bodies,
		NumberOfObjects& numObjects, Real sphMarkerMass,
		const SimParams& paramsH,
		Real radius,
		Real length,
		chrono::ChSharedPtr<chrono::ChMaterialSurface> mat_prop,
		Real density,
		chrono::ChVector<> pos,
		chrono::ChQuaternion<> rot) {
	chrono::ChSharedPtr<chrono::ChBody> body = chrono::ChSharedPtr<
			chrono::ChBody>(
			new chrono::ChBody(
					new chrono::collision::ChCollisionModelParallel));
	body->SetBodyFixed(false);
	body->SetCollide(true);
	body->SetMaterialSurface(mat_prop);
	body->SetPos(pos);
	body->SetRot(rot);
	double volume = chrono::utils::CalcCylinderVolume(radius, 0.5 * length);
	chrono::ChVector<> gyration = chrono::utils::CalcCylinderGyration(radius,
			0.5 * length).Get_Diag();
	double mass = density * volume;
	body->SetMass(mass);
	body->SetInertiaXX(mass * gyration);
	//
	body->GetCollisionModel()->ClearModel();
	chrono::utils::AddCylinderGeometry(body.get_ptr(), radius, 0.5 * length);
	body->GetCollisionModel()->BuildModel();
	mphysicalSystem.AddBody(body);

	FSI_Bodies.push_back(body);
	AddCylinderBce(posRadH, velMasH, rhoPresMuH, referenceArray,
			numObjects, sphMarkerMass, paramsH, body, radius, length);
}
// =============================================================================
void CreateBoxFSI(
		thrust::host_vector<Real3>& posRadH, // do not set the size here since you are using push back later
		thrust::host_vector<Real4>& velMasH,
		thrust::host_vector<Real4>& rhoPresMuH,
		thrust::host_vector<::int4>& referenceArray,
		chrono::ChSystem& mphysicalSystem,
		std::vector<chrono::ChSharedPtr<chrono::ChBody> >& FSI_Bodies,
		NumberOfObjects& numObjects, Real sphMarkerMass,
		const SimParams& paramsH,
		const chrono::ChVector<>& hsize,
		chrono::ChSharedPtr<chrono::ChMaterialSurface> mat_prop,
		Real density,
		chrono::ChVector<> pos,
		chrono::ChQuaternion<> rot) {

	chrono::ChSharedPtr<chrono::ChBody> body = chrono::ChSharedPtr<
			chrono::ChBody>(
			new chrono::ChBody(
					new chrono::collision::ChCollisionModelParallel));
	body->SetBodyFixed(false);
	body->SetCollide(true);
	body->SetMaterialSurface(mat_prop);
	body->SetPos(pos);
	body->SetRot(rot);
	double volume = chrono::utils::CalcBoxVolume(hsize);
	chrono::ChVector<> gyration = chrono::utils::CalcBoxGyration(hsize).Get_Diag();
	double mass = density * volume;
	body->SetMass(mass);
	body->SetInertiaXX(mass * gyration);
	//
	body->GetCollisionModel()->ClearModel();
	chrono::utils::AddBoxGeometry(body.get_ptr(), hsize);
	body->GetCollisionModel()->BuildModel();
	mphysicalSystem.AddBody(body);


	FSI_Bodies.push_back(body);
	AddBoxBce(posRadH, velMasH, rhoPresMuH, referenceArray,
			numObjects, sphMarkerMass, paramsH, body, hsize);
}
