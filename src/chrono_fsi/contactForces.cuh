#ifndef CONTACTFORCES_CUH
#define CONTACTFORCES_CUH

#include "custom_cutil_math.h"
#include "SPHCudaUtils.h"
#include "SDKCollisionSystem.cuh"

////************ note: paramsD is zero here. These expressions are wrong
struct SerpentineParams {
	real_ mm 	;
	real2 r1_2 	;
	real2 r2_2 	;
	real2 r3_2 	;
	real2 r4_2 	;

	real2 r5_2 	;
	real2 r6_2 	;
	real_ x_FirstChannel;
	real_ sPeriod; //serpentine period
	real_ x_SecondChannel;
};
////*************
__constant__ SerpentineParams serpGeomD;
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
__device__ inline real_ IsInEllipse(real2 pos, real2 radii) {
//	printf(" pos %f %f  r2 %f %f\n", pos.x, pos.y, radii.x, radii.y);
//	real2 kk  = pos / radii;
//	printf("kk.x kk.y   %f %f \n", kk.x, kk.y);
//	printf("lengthkk %f and the other length %f \n", length(kk), length(pos / radii));
	return length(pos / radii);
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
__device__ inline real_ IsOutBoundaryEllipsoid(real2 & n2, real2 coord, real2 cent2, real2 r2) {
	real2 relDist2 = coord - cent2;
//	printf("**** relDist %f %f  r2 %f %f\n", relDist2.x, relDist2.y, r2.x, r2.y);
	real_ criteria = IsInEllipse(relDist2, r2);
	if (criteria < 1) {
//		printf("yeap my friend\n");
		real_ x = relDist2.x / criteria;
		real2 penet2 = relDist2 - R2(x, relDist2.y / relDist2.x * x); 	// a vector from boundary ellipsoid to the objects contact point.
																		// but the contact point is inside the boundary ellipse
		n2 = -1 * penet2 / length(penet2);
		return -1 * length(penet2);
	} else {
//		printf("creiteria %f\n", criteria);
		return 1; //a positive number implying it is outside
	}
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
__device__ inline real_ IsInBoundaryEllipsoid(real2 & n2, real2 coord, real2 cent2, real2 r2) {
	real2 relDist2 = coord - cent2;
//	printf("**** relDist %f %f  r2 %f %f\n", relDist2.x, relDist2.y, r2.x, r2.y);
	real_ criteria = IsInEllipse(relDist2, r2);
	if (criteria > 1) {
//		printf("neap my friend\n");
		real_ x = relDist2.x / criteria;
		real2 penet2 = relDist2 - R2(x, relDist2.y / relDist2.x * x); 	// a vector from boundary ellipsoid to the objects contact point.
																		// but the contact point is inside the boundary ellipse
		n2 = -1 * penet2 / length(penet2);
		return -1 * length(penet2);
	} else {
//		printf("creiteria %f\n", criteria);
		return 1; //a positive number implying it is outside
	}
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
__device__ inline real_ ContactWith_YPlanes(real3 & n3, real3 posRad, real_ rRigidDEM) {
	if (posRad.y < 0 + rRigidDEM) {
		n3 = R3(0, 1, 0);
		if (posRad.y - rRigidDEM < 0) printf("a22 %f\n", posRad.y - rRigidDEM);
		return (posRad.y - rRigidDEM);
	}
	if (posRad.y > 1.0 * serpGeomD.mm - rRigidDEM) {
		n3 = R3(0, -1, 0);
		if (1.0 * serpGeomD.mm - rRigidDEM - posRad.y < 0) printf("a22 %f\n", 1.0 * serpGeomD.mm - rRigidDEM - posRad.y);
		return (1.0 * serpGeomD.mm - rRigidDEM - posRad.y);
	}
	n3 = R3(1, 0, 0); // just because. no penetration
	return 1;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//return +: no interpenetration. -: there is interpenetration and the value is the amount of interpenetration
__device__ inline real_ ContactWithSerpentineCurveBeta(real2 & n2, real3 posRad, real_ rRigidDEM) {
	real_ x, y;
	real_ penDist = 0;

	//serpentine
	//real_ r1 = 1.3 * serpGeomD.mm, r2 = 1.0 * serpGeomD.mm, r3=2.0 * serpGeomD.mm, r4 = 0.3 * serpGeomD.mm;

	x = fmod(posRad.x, serpGeomD.sPeriod); //posRad.x - int(posRad.x / serpGeomD.sPeriod) * serpGeomD.sPeriod; //fmod
	y = posRad.z;
	if (y > 0) {
		if (x >= 0 && x < serpGeomD.r3_2.x + serpGeomD.r4_2.x) {
			penDist = IsOutBoundaryEllipsoid(n2, R2(x, y), R2(0, 0), serpGeomD.r2_2 + R2(rRigidDEM));
			if (penDist < 0) {
				printf("a16 %f\n", penDist);
				return penDist;
			}
			penDist = IsInBoundaryEllipsoid(n2, R2(x, y), R2(0, 0), serpGeomD.r3_2 - R2(rRigidDEM));
			if (penDist < 0) {
				printf("a17 %f\n", penDist);
				return penDist;
			}
		}
		if (x >= serpGeomD.r3_2.x + serpGeomD.r4_2.x && x < 2 * serpGeomD.r3_2.x + 2 * serpGeomD.r4_2.x) {
			penDist = IsOutBoundaryEllipsoid(n2, R2(x, y), R2(2 * serpGeomD.r3_2.x + 2 * serpGeomD.r4_2.x, 0), serpGeomD.r2_2 + R2(rRigidDEM));
			if (penDist < 0) {
				printf("a18 %f\n", penDist);
				return penDist;
			}
			penDist = IsInBoundaryEllipsoid(n2, R2(x, y), R2(2 * serpGeomD.r3_2.x + 2 * serpGeomD.r4_2.x, 0), serpGeomD.r3_2 - R2(rRigidDEM));
			if (penDist < 0) {
				printf("a19 %f\n", penDist);
				return penDist;
			}
		}
	} else {
		penDist = IsOutBoundaryEllipsoid(n2, R2(x, y), R2(serpGeomD.r3_2.x + serpGeomD.r4_2.x, 0), serpGeomD.r4_2 + R2(rRigidDEM));
		if (penDist < 0) {
			printf("a20 %f\n", penDist);
			return penDist;
		}
		penDist = IsInBoundaryEllipsoid(n2, R2(x, y), R2(serpGeomD.r3_2.x + serpGeomD.r4_2.x, 0), serpGeomD.r1_2 - R2(rRigidDEM));
		if (penDist < 0) {
			printf("a21 %f\n", penDist);
			return penDist;
		}
	}
	return 1;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
// for DEM, rRigidDEM is sphere radius
// for markers initialization , rRigidDEM = 0
__device__ inline real_ ContactWithSerpentineCurve(real3 & n3, real3 posRad, real_ rRigidDEM) {
	real_ x, y;
	real_ penDist = 0;
	real2 n2;

	//serpentine
	if (posRad.x < paramsD.nPeriod * serpGeomD.sPeriod) {
		penDist = ContactWithSerpentineCurveBeta(n2, posRad, rRigidDEM);
		n3 = R3(n2.x, 0, n2.y);
		if (penDist < 0) printf("a1 %f\n", penDist);
		return penDist;
	} else {
		//straight channel
		x = posRad.x - paramsD.nPeriod * serpGeomD.sPeriod;
		y = posRad.z;

		if (y < 0) {
			penDist = IsOutBoundaryEllipsoid(n2, R2(x, y), R2(serpGeomD.r3_2.x + serpGeomD.r4_2.x, 0), serpGeomD.r4_2 + R2(rRigidDEM));
			if (penDist < 0) {
				n3 = R3(n2.x, 0, n2.y);
				printf("a2 %f\n", penDist);
				return penDist;
			}
			penDist = IsInBoundaryEllipsoid(n2, R2(x, y), R2(serpGeomD.r3_2.x + serpGeomD.r4_2.x, 0), serpGeomD.r1_2 - R2(rRigidDEM));
			if (penDist < 0) {
				n3 = R3(n2.x, 0, n2.y);
				printf("a3 %f\n", penDist);
				return penDist;
			}
		} else {
			if (x < serpGeomD.r3_2.x + 2 * serpGeomD.r4_2.x + serpGeomD.r6_2.x) {
				if (x < serpGeomD.r3_2.x + serpGeomD.r4_2.x) {
					penDist = IsOutBoundaryEllipsoid(n2, R2(x, y), R2(0, 0), serpGeomD.r2_2 + R2(rRigidDEM));
					if (penDist < 0) {
						n3 = R3(n2.x, 0, n2.y);
						printf("a4 %f\n", penDist);
						return penDist;
					}
					penDist = IsInBoundaryEllipsoid(n2, R2(x, y), R2(0, 0), serpGeomD.r3_2 - R2(rRigidDEM));
					if (penDist < 0) {
						n3 = R3(n2.x, 0, n2.y);
						printf("a5 %f\n", penDist);
						return penDist;
					}
				}
				if (x >= serpGeomD.r3_2.x + serpGeomD.r4_2.x && x < serpGeomD.r3_2.x + 2 * serpGeomD.r4_2.x + serpGeomD.r6_2.x) {
					penDist = IsOutBoundaryEllipsoid(n2, R2(x, y), R2(serpGeomD.r2_2.x + 2 * serpGeomD.r1_2.x + serpGeomD.r5_2.x, 0), serpGeomD.r5_2 + R2(rRigidDEM));
					if (penDist < 0) {
						n3 = R3(n2.x, 0, n2.y);
						printf("a6 %f\n", penDist);
						return penDist;
					}
					penDist = IsInBoundaryEllipsoid(n2, R2(x, y), R2(serpGeomD.r3_2.x + 2 * serpGeomD.r4_2.x + serpGeomD.r6_2.x, 0), serpGeomD.r6_2 - R2(rRigidDEM));
					if (penDist < 0) {
						n3 = R3(n2.x, 0, n2.y);
						printf("a7 %f\n", penDist);
						return penDist;
					}
				}
			} else {
				//horizontal walls
				x = x - (serpGeomD.r3_2.x + 2 * serpGeomD.r4_2.x + serpGeomD.r6_2.x);
				real2 y2_slimHor = R2(2.314, 3.314) * serpGeomD.mm;
				real2 y2_endHor = R2(serpGeomD.r2_2.y, serpGeomD.r3_2.y);
				if (x < serpGeomD.x_FirstChannel) {
					penDist = y - (serpGeomD.r5_2.y + rRigidDEM);
					if (penDist < 0) {
						n2 = R2(0, 1);
						n3 = R3(n2.x, 0, n2.y);
						printf("a8 %f\n", penDist);
						return penDist; //note that y is negative, fabs(y) = -y
					}
					penDist = -y + serpGeomD.r6_2.y - rRigidDEM;
					if (penDist < 0) {
						n2 = R2(0, -1);
						n3 = R3(n2.x, 0, n2.y);
						printf("a9 %f\n", penDist);
						return penDist;
					}
				}
				if (x >= serpGeomD.x_FirstChannel
						&& x < serpGeomD.x_FirstChannel + serpGeomD.x_SecondChannel) {
					penDist = y - (y2_slimHor.x + rRigidDEM);
					if (penDist < 0) {
						n2 = R2(0, 1);
						n3 = R3(n2.x, 0, n2.y);
						printf("a10 %f\n", penDist);
						return penDist;
					}
					penDist = -y + y2_slimHor.y - rRigidDEM;
					if (penDist < 0) {
						n2 = R2(0, -1);
						n3 = R3(n2.x, 0, n2.y);
						printf("a11 %f\n", penDist);
						return penDist;
					}
				}
				if (x >= serpGeomD.x_FirstChannel + serpGeomD.x_SecondChannel) {
					penDist = y - (y2_endHor.x + rRigidDEM);
					if (penDist < 0) {
						n2 = R2(0, 1);
						n3 = R3(n2.x, 0, n2.y);
						printf("a12 %f\n", penDist);
						return penDist;
					}
					penDist = -y + y2_endHor.y - rRigidDEM;
					if (penDist < 0) {
						n2 = R2(0, -1);
						n3 = R3(n2.x, 0, n2.y);
						printf("a13 %f\n", penDist);
						return penDist;
					}
				}
				//****** vertical walls
				if (x > 0 && x < serpGeomD.x_FirstChannel + .5 * serpGeomD.x_SecondChannel) {
					if (y < y2_slimHor.x || y > y2_slimHor.y) {
						penDist = serpGeomD.x_FirstChannel - rRigidDEM - x;
						if (penDist < 0) {
							n2 = R2(-1, 0);
							n3 = R3(n2.x, 0, n2.y);
							printf("a14 %f\n", penDist);
							return penDist;
						}
					}
				}
				if (x > serpGeomD.x_FirstChannel + .5 * serpGeomD.x_SecondChannel){
					if (y < y2_slimHor.x || y > y2_slimHor.y) {
						penDist = x - (serpGeomD.x_FirstChannel + serpGeomD.x_SecondChannel + rRigidDEM);
						if (penDist < 0) {
							n2 = R2(1, 0);
							n3 = R3(n2.x, 0, n2.y);
							printf("a15 %f\n", penDist);
							return penDist;
						}
					}
				}
			}
		}
	}
	return 1;
}
//--------------------------------------------------------------------------------------------------------------------------------
void setParameters2(SimParams *hostParams, NumberOfObjects *numObjects);
//--------------------------------------------------------------------------------------------------------------------------------
void Add_ContactForces(
		real3* totalAccRigid3,
		real3* posRigidD,
		real4* velMassRigidD);

#endif
