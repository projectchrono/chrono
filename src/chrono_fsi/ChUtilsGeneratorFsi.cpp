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
// Utility class for generating BCE markers.//
// =============================================================================

namespace chrono{
namespace fsi {
namespace utils {

// =============================================================================
// left off here
// TransformToCOG
// This utility function converts a given position and orientation, specified
// with respect to a body's reference frame, into a frame defined with respect
// to the body's centroidal frame.  Note that by default, a body's reference
// frame is the centroidal frame. This is not true for a ChBodyAuxRef.
void TransformBceFrameToCOG(
	chrono::ChBody* body, 
	const chrono::ChVector<>& pos,
	const chrono::ChMatrix33<>& rot, 
	chrono::ChFrame<>& frame) {
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
void CreateBceGlobalMarkersFromBceLocalPos(
	SphMarkerDataH& sphMarkersH,
	FsiGeneralData& fsiGeneralData,
	NumberOfObjects& numObjects,
	const SimParams& paramsH,
	const thrust::host_vector<Real3>& posRadBCE,
	chrono::ChSharedPtr<chrono::ChBody> body,
	chrono::ChVector<> collisionShapeRelativePos,
	chrono::ChQuaternion<> collisionShapeRelativeRot,
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
		sphMarkersH.posRadH.push_back(ConvertChVectorToR3(posGlob));

		chrono::ChVector<> vAbs = body->PointSpeedLocalToParent(posLoc_COG);
		Real3 v3 = ConvertChVectorToR3(vAbs);
		sphMarkersH.velMasH.push_back(v3);

		sphMarkersH.rhoPresMuH.push_back(
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
	SphMarkerDataH& sphMarkersH,
	FsiGeneralData& fsiGeneralData,
	NumberOfObjects& numObjects,
	const SimParams& paramsH,
	const thrust::host_vector<Real3>& posRadBCE,
	chrono::ChSharedPtr<chrono::ChBody> body,
	chrono::ChVector<> collisionShapeRelativePos,
	chrono::ChQuaternion<> collisionShapeRelativeRot) {

	CreateBceGlobalMarkersFromBceLocalPos(sphMarkersH, fsiGeneralData,
			numObjects, paramsH, posRadBCE, body,
			collisionShapeRelativePos, collisionShapeRelativeRot, false);
}
// =============================================================================
void AddSphereBce(
	SphMarkerDataH& sphMarkersH,
	FsiGeneralData& fsiGeneralData,
	NumberOfObjects& numObjects,
	const SimParams& paramsH, 
	chrono::ChSharedPtr<chrono::ChBody> body, 
	chrono::ChVector<> relPos,
	chrono::ChQuaternion<> relRot,
	Real radius) {

	thrust::host_vector<Real3> posRadBCE;
	CreateBCE_On_Sphere(posRadBCE, radius, paramsH);

	if (sphMarkersH.posRadH.size() != numObjects.numAllMarkers) {
		printf("Error! numMarkers, %d, does not match posRadH.size(), %d\n",
				numObjects.numAllMarkers, sphMarkersH.posRadH.size());
		std::cin.get();
	}

	CreateBceGlobalMarkersFromBceLocalPos(sphMarkersH, fsiGeneralData,
			numObjects, paramsH, posRadBCE, body);

	posRadBCE.clear();

}
// =============================================================================

void AddCylinderBce(
	SphMarkerDataH& sphMarkersH,
	FsiGeneralData& fsiGeneralData,
	NumberOfObjects& numObjects,
	const SimParams& paramsH, 
	chrono::ChSharedPtr<chrono::ChBody> body, 
	chrono::ChVector<> relPos,
	chrono::ChQuaternion<> relRot,
	Real radius, 
	Real height) {

	thrust::host_vector<Real3> posRadBCE;
	CreateBCE_On_Cylinder(posRadBCE, radius, height, paramsH);

	if (sphMarkersH.posRadH.size() != numObjects.numAllMarkers) {
		printf("Error! numMarkers, %d, does not match posRadH.size(), %d\n",
				numObjects.numAllMarkers, sphMarkersH.posRadH.size());
		std::cin.get();
	}
	CreateBceGlobalMarkersFromBceLocalPos(sphMarkersH, fsiGeneralData,
			numObjects, paramsH, posRadBCE, body);

	posRadBCE.clear();
}

// =============================================================================
// Arman note, the function in the current implementation creates boundary bce (accesses only referenceArray[1])

// Arman thrust::host_vector<uint>& bodyIndex,

void AddBoxBce(
	SphMarkerDataH& sphMarkersH,
	FsiGeneralData& fsiGeneralData,
	NumberOfObjects& numObjects,
	const SimParams& paramsH, 
	chrono::ChSharedPtr<chrono::ChBody> body, 
	chrono::ChVector<> relPos,
	chrono::ChQuaternion<> relRot,
	const chrono::ChVector<>& size) {

	chrono::utils::AddBoxGeometry(body.get_ptr(), size, relPos, relRot, true);
	thrust::host_vector<Real3> posRadBCE;

	CreateBCE_On_Box(posRadBCE, ConvertChVectorToR3(size), 12, paramsH);

	if (sphMarkersH.posRadH.size() != numObjects.numAllMarkers) {
		printf("Error! numMarkers, %d, does not match posRadH.size(), %d\n",
				numObjects.numAllMarkers, sphMarkersH.posRadH.size());
		std::cin.get();
	}

	CreateBceGlobalMarkersFromBceLocalPosBoundary(sphMarkersH,
			fsiGeneralData, numObjects, paramsH, posRadBCE, body,
			relPos, relRot);
	posRadBCE.clear();
}

// =============================================================================
void AddBCE_FromFile(
	SphMarkerDataH& sphMarkersH,
	FsiGeneralData& fsiGeneralData,
	NumberOfObjects& numObjects,
	const SimParams& paramsH, 
	chrono::ChSharedPtr<chrono::ChBody> body,
	std::string dataPath) {
	//----------------------------
	//  chassis
	//----------------------------
	thrust::host_vector<Real3> posRadBCE;

	LoadBCE_fromFile(posRadBCE, dataPath);
	if (sphMarkersH.posRadH.size() != numObjects.numAllMarkers) {
		printf("Error! numMarkers, %d, does not match posRadH.size(), %d\n",
				numObjects.numAllMarkers, sphMarkersH.posRadH.size());
		std::cin.get();
	}

	CreateBceGlobalMarkersFromBceLocalPos(sphMarkersH, fsiGeneralData,
			numObjects, paramsH, posRadBCE, body);
	posRadBCE.clear();
}















// =============================================================================
void CreateSphereFSI(
	SphMarkerDataH& sphMarkersH,
	FsiGeneralData& fsiGeneralData,
	std::vector<chrono::ChSharedPtr<chrono::ChBody> > * fsiBodeisPtr,
	chrono::ChSystem& mphysicalSystem,
	NumberOfObjects& numObjects,
	const SimParams& paramsH,
	chrono::ChSharedPtr<chrono::ChMaterialSurface> mat_prop,
	Real density,
	chrono::ChVector<> pos,
	Real radius) {

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
	fsiBodeisPtr->push_back(body);

	AddSphereBce(sphMarkersH, fsiGeneralData,
			numObjects, paramsH, body, 
			chrono::ChVector<>(0,0,0), chrono::ChQuaternion<>(1, 0, 0, 0), 
			radius);
}
// =============================================================================
void CreateCylinderFSI(
	SphMarkerDataH& sphMarkersH,
	FsiGeneralData& fsiGeneralData,
	std::vector<chrono::ChSharedPtr<chrono::ChBody> > * fsiBodeisPtr,
	chrono::ChSystem& mphysicalSystem,
	NumberOfObjects& numObjects,
	const SimParams& paramsH,
	chrono::ChSharedPtr<chrono::ChMaterialSurface> mat_prop,
	Real density,
	chrono::ChVector<> pos,
	chrono::ChQuaternion<> rot,
	Real radius,
	Real length) {
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

	fsiBodeisPtr->push_back(body);
	AddCylinderBce(sphMarkersH, fsiGeneralData,
			numObjects, paramsH, body, chrono::ChVector<>(0,0,0), chrono::ChQuaternion<>(1, 0, 0, 0), 
			radius, length);
}
// =============================================================================
void CreateBoxFSI(
	SphMarkerDataH& sphMarkersH,
	FsiGeneralData& fsiGeneralData,
	std::vector<chrono::ChSharedPtr<chrono::ChBody> > * fsiBodeisPtr,
	chrono::ChSystem& mphysicalSystem,
	NumberOfObjects& numObjects,
	const SimParams& paramsH,
	chrono::ChSharedPtr<chrono::ChMaterialSurface> mat_prop,
	Real density,
	chrono::ChVector<> pos,
	chrono::ChQuaternion<> rot,
	const chrono::ChVector<>& hsize) {

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


	fsiBodeisPtr->push_back(body);
	AddBoxBce(sphMarkersH, fsiGeneralData, 
		numObjects, paramsH, body, chrono::ChVector<>(0,0,0), chrono::ChQuaternion<>(1, 0, 0, 0), 
		hsize);
}

} // end namespace utils
} // end namespace fsi
} // end namespace chrono