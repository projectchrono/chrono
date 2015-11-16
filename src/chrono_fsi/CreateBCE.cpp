/*
 * CreateBCE.cpp
 *
 *  Created on: Nov 12, 2015
 *      Author: Arman Pazouki
 */

#include "chrono_fsi/CreateBCE.h"
#include "chrono_fsi/SphInterface.h"

//**********************************************

Real CreateOne3DRigidCylinder(thrust::host_vector<Real3>& posRadH,
		thrust::host_vector<Real4>& velMasH,
		thrust::host_vector<Real4>& rhoPresMuH, chrono::ChBody* body,
		Real cyl_rad, Real cyl_h, Real rigidMass, Real sphMarkerMass, int type,
		const SimParams& paramsH) {
	// Arman : take care of velocity and w stuff for BCE
	int num_BCEMarkers = 0;
	Real spacing = paramsH.MULT_INITSPACE * paramsH.HSML;
	for (Real s = -0.5 * cyl_h; s <= 0.5 * cyl_h; s += spacing) {
		Real3 centerPointLF = mR3(0, s, 0);
		posRadH.push_back(
				R3_LocalToGlobal(centerPointLF, body->GetPos(),
						body->GetRot()));
		//        ChVector<> vel = body->PointSpeedLocalToParent(centerPointLF);

		velMasH.push_back(mR4(0, 0, 0, sphMarkerMass));
		rhoPresMuH.push_back(
				mR4(paramsH.rho0, paramsH.BASEPRES, paramsH.mu0, type)); // take care of type			 ///
																		 // type needs to be unique, to
																		 // differentiate flex from other
																		 // flex as well as other rigids
		num_BCEMarkers++;
		for (Real r = spacing; r < cyl_rad - paramsH.solidSurfaceAdjust; r +=
				spacing) {
			Real deltaTeta = spacing / r;
			for (Real teta = .1 * deltaTeta;
					teta < 2 * chrono::CH_C_PI - .1 * deltaTeta; teta +=
							deltaTeta) {
				Real3 BCE_Pos_local = mR3(r * cos(teta), 0, r * sin(teta))
						+ centerPointLF;
				//                Real3 BCE_Pos_Global = Rotate_By_Quaternion(q4, BCE_Pos_local) + centerPoint;

				posRadH.push_back(
						R3_LocalToGlobal(BCE_Pos_local, body->GetPos(),
								body->GetRot()));
				velMasH.push_back(mR4(0, 0, 0, sphMarkerMass));
				rhoPresMuH.push_back(
						mR4(paramsH.rho0, paramsH.BASEPRES, paramsH.mu0, type)); // take care of type
				num_BCEMarkers++;
			}
		}
	}

	return num_BCEMarkers;
}

// =============================================================================
Real CreateOne3DRigidSphere(thrust::host_vector<Real3>& posRadH,
		thrust::host_vector<Real4>& velMasH,
		thrust::host_vector<Real4>& rhoPresMuH, chrono::ChBody* body,
		Real cyl_rad, Real rigidMass, Real sphMarkerMass, int type,
		const SimParams& paramsH) {
	// Arman : take care of velocity and w stuff for BCE
	int num_BCEMarkers = 0;
	Real spacing = paramsH.MULT_INITSPACE * paramsH.HSML;

	for (Real r = spacing; r < cyl_rad - paramsH.solidSurfaceAdjust; r +=
			spacing) {
		Real deltaTeta = spacing / r;
		Real deltaPhi = deltaTeta;

		for (Real phi = .1 * deltaPhi; phi < chrono::CH_C_PI - .1 * deltaPhi;
				phi += deltaPhi) {
			for (Real teta = .1 * deltaTeta;
					teta < 2 * chrono::CH_C_PI - .1 * deltaTeta; teta +=
							deltaTeta) {
				Real3 BCE_Pos_local = mR3(r * sin(phi) * cos(teta),
						r * sin(phi) * sin(teta), r * cos(phi));

				posRadH.push_back(
						R3_LocalToGlobal(BCE_Pos_local, body->GetPos(),
								body->GetRot()));
				velMasH.push_back(mR4(0, 0, 0, sphMarkerMass));
				rhoPresMuH.push_back(
				mR4(paramsH.rho0, paramsH.BASEPRES, paramsH.mu0, type)); // take care of type
				num_BCEMarkers++;
			}
		}
	}

	return num_BCEMarkers;
}

//**********************************************
// note, the function in the current implementation creates boundary bce (zero velocity)
void CreateBCE_On_Box(thrust::host_vector<Real3>& posRadBCE,
		thrust::host_vector<Real4>& velMasBCE,
		thrust::host_vector<Real4>& rhoPresMuBCE, const SimParams& paramsH,
		const Real& sphMarkerMass, const chrono::ChVector<>& size,
		const chrono::ChVector<>& pos, const chrono::ChQuaternion<>& rot,
		int face) { // x=1, y=2, z =3; therefore 12 means creating markers on the top surface parallel to xy plane,
					// similarly -12 means bottom face paralel to xy. similarly 13, -13, 23, -23

	Real initSpace0 = paramsH.MULT_INITSPACE * paramsH.HSML;
	int nFX = ceil(size.x / (initSpace0));
	int nFY = ceil(size.y / (initSpace0));
	int nFZ = ceil(size.z / (initSpace0));

	Real initSpaceX = size.x / nFX;
	Real initSpaceY = size.y / nFY;
	Real initSpaceZ = size.z / nFZ;

	int2 iBound = mI2(-nFX, nFX);
	int2 jBound = mI2(-nFY, nFY);
	int2 kBound = mI2(-nFZ, nFZ);

	switch (face) {
	case 12:
		kBound = mI2(nFZ - paramsH.NUM_BOUNDARY_LAYERS + 1, nFZ);
		break;
	case -12:
		kBound = mI2(-nFZ, -nFZ + paramsH.NUM_BOUNDARY_LAYERS - 1);
		break;
	case 13:
		jBound = mI2(nFY - paramsH.NUM_BOUNDARY_LAYERS + 1, nFY);
		break;
	case -13:
		jBound = mI2(-nFY, -nFY + paramsH.NUM_BOUNDARY_LAYERS - 1);
		break;
	case 23:
		iBound = mI2(nFX - paramsH.NUM_BOUNDARY_LAYERS + 1, nFX);
		break;
	case -23:
		iBound = mI2(-nFX, -nFX + paramsH.NUM_BOUNDARY_LAYERS - 1);
		break;
	default:
		printf("wrong argument box bce initialization\n");
		break;
	}

	for (int i = iBound.x; i <= iBound.y; i++) {
		for (int j = jBound.x; j <= jBound.y; j++) {
			for (int k = kBound.x; k <= kBound.y; k++) {
				chrono::ChVector<> relMarkerPos = chrono::ChVector<>(
						i * initSpaceX, j * initSpaceY, k * initSpaceZ);
				chrono::ChVector<> markerPos = rot.Rotate(relMarkerPos) + pos;

				if ((markerPos.x < paramsH.cMin.x
						|| markerPos.x > paramsH.cMax.x)
						|| (markerPos.y < paramsH.cMin.y
								|| markerPos.y > paramsH.cMax.y)
						|| (markerPos.z < paramsH.cMin.z
								|| markerPos.z > paramsH.cMax.z)) {
					continue;
				}

				posRadBCE.push_back(mR3(markerPos.x, markerPos.y, markerPos.z));
				velMasBCE.push_back(mR4(0, 0, 0, sphMarkerMass));
				rhoPresMuBCE.push_back(
						mR4(paramsH.rho0, paramsH.LARGE_PRES, paramsH.mu0, 0));
			}
		}
	}
}

//**********************************************

void LoadBCE_fromFile(thrust::host_vector<Real3>& posRadBCE, // do not set the size here since you are using push back later
		std::string fileName) {
	std::string ddSt;
	char buff[256];
	int numBce = 0;
	const int cols = 3;
	//*******************************************************************
	std::cout << "  reading BCE data from: " << fileName << " ...\n";
	std::ifstream inMarker;
	inMarker.open(fileName);
	if (!inMarker) {
		std::cout << "   Error! Unable to open file: " << fileName << std::endl;
	}
	getline(inMarker, ddSt);
	Real q[cols];
	while (getline(inMarker, ddSt)) {
		std::stringstream linestream(ddSt);
		for (int i = 0; i < cols; i++) {
			linestream.getline(buff, 50, ',');
			q[i] = atof(buff);
		}
		posRadBCE.push_back(mR3(q[0], q[1], q[2]));
		numBce++;
	}

	std::cout << "  Loaded BCE data from: " << fileName << std::endl;
}

