#include "ChCCollisionGPU.h"
#include "ChCCollisionGPU.cuh"

__device__ inline void AddContact(contactGPU* CData,uint offset,  int A,int B, float3 onA, float3 onB,float3 n,float dist){
	CData[offset].I=F3(dist,A,B);
	CData[offset].N=n;
	CData[offset].Pa=onA;
	CData[offset].Pb=onB;
	CData[offset].G=F3(0,0,0);
}

__device__ bool pointInTriangle(const float3 &p1,const float3 &p2,const float3 &p3, const float3 &normal, const float3 &S ){
	float3 edge1=( p2 - p1 );
	float3 edge2=( p3 - p2 );
	float3 edge3=( p1 - p3 );

	float3 p1_to_p=( S - p1 );
	float3 p2_to_p=( S - p2 );
	float3 p3_to_p=( S - p3 );

	float3 edge1_normal=cross(edge1,normal);
	float3 edge2_normal=cross(edge2,normal);
	float3 edge3_normal=cross(edge3,normal);

	float r1 = dot(edge1_normal, p1_to_p );
	float r2 = dot(edge2_normal, p2_to_p );
	float r3 = dot(edge3_normal, p3_to_p );
	if ( ( r1 > 0 && r2 > 0 && r3 > 0 ) ||( r1 <= 0 && r2 <= 0 && r3 <= 0 ) ){return true;}
	return false;
}

__device__ float SegmentSqrDistance(const float3& from, const float3& to,const float3 &p, float3 &nearest) {
	float3  diff = p - from; 
	float3	v = to - from;
	float t = dot(v,diff);
	if (t > 0) {
		float dotVV = dot(v,v);
		if (t < dotVV) {
			t /= dotVV;
			diff -= t*v;
		}else {
			t = 1;
			diff -= v;
		}
	}else{t = 0;}
	nearest = from + t*v;
	return dot(diff,diff);	
}

__global__ void Sphere_Sphere(object * object_data,int3 * Pair , uint* Contact_Number,contactGPU* CData,uint totalPossibleConts){
	uint Index = blockIdx.x* blockDim.x + threadIdx.x;
	if(Index>=totalPossibleConts){return;}
	int3 pair=Pair[Index];
	if(pair.z==0){
		object A=object_data[pair.x];
		object B=object_data[pair.y];
		float3 N=F3(B.A)-F3(A.A);
		float centerDist =dot(N,N);
		float rAB =B.A.w + A.A.w;
		if (centerDist <= (rAB)*(rAB)){
			float dist=sqrtf(centerDist);
			N=N/dist;
			AddContact(CData,Index,A.B.x,B.B.x,F3(A.A)+A.A.w*N,F3(B.A)-B.A.w*N,N,-dist);
			Contact_Number[Index]=Index;
		}
	}
}

__device__ void CalcMatrices(float3 &center, float4 &q,const  float3 & r,float3 & M_1,float3 &M_2) {
	float3 Q0, Q1, Q2, A0, A1, A2,r2;
	r2.x = r.x*r.x;
	r2.y = r.y*r.y;
	r2.z = r.z*r.z;

	A0.x = 2.0f * (q.x * q.x + q.y * q.y) - 1.0f;
	A0.y = 2.0f * (q.y * q.z - q.x * q.w);
	A0.z = 2.0f * (q.x * q.z + q.y * q.w);

	A1.x = 2.0f * (q.y * q.z + q.x * q.w); 
	A1.y = 2.0f * (q.x * q.x + q.z * q.z) - 1.0f;
	A1.z = 2.0f * (q.z * q.w - q.x * q.y);

	A2.x = 2.0f * (q.y * q.w - q.x * q.z);
	A2.y = 2.0f * (q.x * q.y + q.z * q.w);
	A2.z = 2.0f * (q.x * q.x + q.w * q.w) - 1.0f;

	M_1 =  F3(
		r2.x * A0.x * A0.x + r2.y * A0.y * A0.y + r2.z * A0.z * A0.z,
		r2.x * A0.x * A1.x + r2.y * A0.y * A1.y + r2.z * A0.z * A1.z,
		r2.x * A0.x * A2.x + r2.y * A0.y * A2.y + r2.z * A0.z * A2.z);
	M_2 =  F3(
		r2.x * A1.x * A1.x + r2.y * A1.y * A1.y + r2.z * A1.z * A1.z,
		r2.x * A1.x * A2.x + r2.y * A1.y * A2.y + r2.z * A1.z * A2.z,
		r2.x * A2.x * A2.x + r2.y * A2.y * A2.y + r2.z * A2.z * A2.z);
}
__device__ inline float distanceFunc(
									 float  al1, float  al2, 
									 const float3 &centerA,const float3 &centerB, 
									 const float3 &M11, const float3 &M12,
									 const float3 &M21, const float3 &M22,
									 float3& distVector, float3& c, float3& p1, float3& p2) {

										 c.x =  cos(al1) * cos(al2);
										 c.y =  sin(al1) * cos(al2);
										 c.z =  sin(al2);

										 float t1=  1.0f / sqrt((M11.x * c.x + M11.y * c.y + M11.z * c.z) * c.x + 
											 (M11.y * c.x + M12.x * c.y + M12.y * c.z) * c.y + 
											 (M11.z * c.x + M12.y * c.y + M12.z * c.z) * c.z ); 
										 p1 = F3(M11.x * c.x + M11.y * c.y + M11.z * c.z, M11.y * c.x + M12.x * c.y + M12.y * c.z, M11.z * c.x + M12.y * c.y + M12.z * c.z) * t1 + centerA;
										 float t2=- 1.0f / sqrt((M21.x * c.x + M21.y * c.y + M21.z * c.z) * c.x + 
											 (M21.y * c.x + M22.x * c.y + M22.y * c.z) * c.y + 
											 (M21.z * c.x + M22.y * c.y + M22.z * c.z) * c.z ); 
										 p2 = F3(M21.x * c.x + M21.y * c.y + M21.z * c.z, M21.y * c.x + M22.x * c.y + M22.y * c.z, M21.z * c.x + M22.y * c.y + M22.z * c.z) * t2 + centerB;

										 distVector = p1 - p2;
										 return dot(distVector, distVector);
}
__device__ inline float distanceFunc(
									 float  al1, float  al2, 
									 const float3 &centerA,const float3 &centerB, 
									 const float3 &M11, const float3 &M12,
									 const float3 &M21, const float3 &M22,
									 float3& distVector, float3& c, float& lam1, float& lam2) {
										 float3 p1,p2;
										 float cal2=__cosf(al2);
										 c.x =  __cosf(al1) * cal2;
										 c.y =  __sinf(al1) * cal2;
										 c.z =  __sinf(al2);
										 lam1 = .5f * sqrt(	(M11.x * c.x + M11.y * c.y + M11.z * c.z) * c.x + 
											 (M11.y * c.x + M12.x * c.y + M12.y * c.z) * c.y + 
											 (M11.z * c.x + M12.y * c.y + M12.z * c.z) * c.z ); 
										 lam2 = .5f * sqrt(	(M21.x * c.x + M21.y * c.y + M21.z * c.z) * c.x + 
											 (M21.y * c.x + M22.x * c.y + M22.y * c.z) * c.y + 
											 (M21.z * c.x + M22.y * c.y + M22.z * c.z) * c.z );

										 float t1= 0.5f / lam1;
										 p1 = F3(M11.x * c.x + M11.y * c.y + M11.z * c.z, M11.y * c.x + M12.x * c.y + M12.y * c.z, M11.z * c.x + M12.y * c.y + M12.z * c.z) * t1 + centerA;
										 float t2=-0.5f / lam2;
										 p2 = F3(M21.x * c.x + M21.y * c.y + M21.z * c.z, M21.y * c.x + M22.x * c.y + M22.y * c.z, M21.z * c.x + M22.y * c.y + M22.z * c.z) * t2 + centerB;

										 distVector = p1 - p2;
										 return dot(distVector, distVector);
}
__device__ inline float distanceFunc(
									 float  al1, float  al2, 
									 const float3 &centerA,const float3 &centerB, 
									 const float3 &M11, const float3 &M12,
									 const float3 &M21, const float3 &M22,
									 float3& c) {
										 float cal2=cos(al2);
										 c.x =  cos(al1) * cal2;
										 c.y =  sin(al1) * cal2;
										 c.z =  sin(al2);
										 float t1=  1.0f / sqrt((M11.x * c.x + M11.y * c.y + M11.z * c.z) * c.x + 
											 (M11.y * c.x + M12.x * c.y + M12.y * c.z) * c.y + 
											 (M11.z * c.x + M12.y * c.y + M12.z * c.z) * c.z ); 
										 float3 p1 = F3(M11.x * c.x + M11.y * c.y + M11.z * c.z, M11.y * c.x + M12.x * c.y + M12.y * c.z, M11.z * c.x + M12.y * c.y + M12.z * c.z) * t1 + centerA;
										 float t2=- 1.0f / sqrt((M21.x * c.x + M21.y * c.y + M21.z * c.z) * c.x + 
											 (M21.y * c.x + M22.x * c.y + M22.y * c.z) * c.y + 
											 (M21.z * c.x + M22.y * c.y + M22.z * c.z) * c.z );
										 float3 p2 = F3(M21.x * c.x + M21.y * c.y + M21.z * c.z, M21.y * c.x + M22.x * c.y + M22.y * c.z, M21.z * c.x + M22.y * c.y + M22.z * c.z) * t2 + centerB;
										 p1 -= p2;		// recycle p1 for the distance
										 return dot(p1, p1);
}
__device__ inline float distanceFuncFast(
	float  al1, float  al2, 
	const float3 &centerA,const float3 &centerB, 
	const float3 &M11, const float3 &M12,
	const float3 &M21, const float3 &M22,
	float3& c) {
		float cal2=__cosf(al2);
		c.x =  __cosf(al1) * cal2;
		c.y =  __sinf(al1) * cal2;
		c.z =  __sinf(al2);
		float t1=  rsqrtf(	(M11.x * c.x + M11.y * c.y + M11.z * c.z) * c.x + 
			(M11.y * c.x + M12.x * c.y + M12.y * c.z) * c.y + 
			(M11.z * c.x + M12.y * c.y + M12.z * c.z) * c.z );
		float3 p1 = F3(M11.x * c.x + M11.y * c.y + M11.z * c.z, M11.y * c.x + M12.x * c.y + M12.y * c.z, M11.z * c.x + M12.y * c.y + M12.z * c.z) * t1 + centerA;
		float t2=- rsqrtf(	(M21.x * c.x + M21.y * c.y + M21.z * c.z) * c.x + 
			(M21.y * c.x + M22.x * c.y + M22.y * c.z) * c.y + 
			(M21.z * c.x + M22.y * c.y + M22.z * c.z) * c.z);
		float3 p2 = F3(M21.x * c.x + M21.y * c.y + M21.z * c.z, M21.y * c.x + M22.x * c.y + M22.y * c.z, M21.z * c.x + M22.y * c.y + M22.z * c.z) * t2 + centerB;
		p1 -= p2;		// recycle p1 for the distance
		return dot(p1, p1);
}

#define LARGE_NUMBER 10000000
#define PI_128 0.02454369261

__device__ bool ContactCalculation(object &A, object &B, float3& pA, float3& pB, float3& c, float& distance) {
	float3 M11, M12;
	float3 M21, M22;

	float3 centerA = F3(A.A);	//*** body A
	float3 centerB = F3(B.A);	//*** body B
	float al1 = 0;							// alpha1 and alpha2 are the angles of contact normal vector
	float al2 = 0;

	float3 d;
	float lam1, lam2;
	float distance2 = 10000000;
	float distance3;						//its name has two parts because it is used in two different locations.
	int noOfPoints = 5;					//to double up the efficiency, use odd numbers.
	float count=0;	

	float3 dP11;							//dP11 is not used after pointA, so after pointB dP11 is recycled for c2
	float3 dP12;							//dP12 is not used after pointA, so after pointC dP12 is recycled for dist0, dist1, dist2:	{dist0, dist1, dist2} = {dP12[0], dP12[1], dP12[2]}
	float3 dP21;							//dP21 is not used after pointA, so after pointD dP21 is recycled for lowerBound, midPoint, upperBound:	{lowerBound, midPoint, upperBound} = {dP21[0], dP21[1], dP21[2]}
	float3 dP22;
	float3 MC1;
	float4 Quat1=F4(A.C.w,A.C.x,A.C.y,A.C.z);
	float4 Quat2=F4(B.C.w,B.C.x,B.C.y,B.C.z);

	CalcMatrices(centerA, Quat1, F3(A.B),M11,M12);
	CalcMatrices(centerB, Quat2, F3(B.B),M21,M22);

	//*****************************************
	// FINDING THE BEST INITIAL POINT
	//*****************************************
	float invv=1.0f / float(noOfPoints);
	float t1,t2;
	for (int i = 0; i < noOfPoints*noOfPoints; i++){
		t1=float(.5f + (i - (noOfPoints * int(i*invv))))*invv*2.0*PI;
		t2=float(.5f + (i*invv))*invv*2.0*PI;
		distance3 = distanceFuncFast( t1, t2, centerA, centerB, M11,M12,M21,M22, c);
		if (distance3 < distance2) { distance2 = distance3; al1 = t1; al2 = t2;}
	}
	//*****************************************
	// END OF FINDING THE BEST INITIAL POINT
	//*****************************************

	//*****************************************
	// Iterative Loop
	//*****************************************

	float bet1, bet2;
	float lowerBound = -PI_128,upperBound = PI_128,midPoint = 0 ;
	float mid1stInterval;
	float mid2ndInterval;

	while (count < 16) {
		count++;
		distance2 = distanceFunc(al1, al2, centerA, centerB, M11,M12,M21,M22, d, c, lam1, lam2);
		//Note:	MC1[i] = M1[i][k] * c[k], i=0,1,2. k=0,1,2 summation index
		// and:	MC1[i] = M2[i][k] * c[k]
		MC1.x = M11.x * c.x + M11.y * c.y + M11.z * c.z; 
		MC1.y = M11.y * c.x + M12.x * c.y + M12.y * c.z; 
		MC1.z = M11.z * c.x + M12.y * c.y + M12.z * c.z; 

		//<------------------------------
		float lampow=.125f/(lam1*lam1*lam1);
		float t1=.5f / lam1;
		float3 T1,T2;
		// T = [T1; T2; T2] is symmetric. Only T1 and T2 can be used to store the 6 components
		T1.x = t1 * M11.x -  lampow *  MC1.x * MC1.x;			
		T1.y = t1 * M11.y -  lampow *  MC1.x * MC1.y;
		T1.z = t1 * M11.z -  lampow *  MC1.x * MC1.z;

		T1.y = t1 * M11.y  - lampow * MC1.y * MC1.x;			
		T2.x = t1 * M12.x  - lampow * MC1.y * MC1.y;
		T2.y = t1 * M12.y  - lampow * MC1.y * MC1.z;

		T1.z = t1 * M11.z  - lampow * MC1.z * MC1.x;			
		T2.y = t1 * M12.y  - lampow * MC1.z * MC1.y;
		T2.z = t1 * M12.z  - lampow * MC1.z * MC1.z;

		float sal1=sin(al1), sal2=sin(al2);
		float cal1=cos(al1), cal2=cos(al2);

		dP11.x = -T1.x * sal1 * cal2 + T1.y * cal1 * cal2;
		dP11.y = -T1.y * sal1 * cal2 + T2.x * cal1 * cal2;
		dP11.z = -T1.z * sal1 * cal2 + T2.y * cal1 * cal2;

		dP12.x = -T1.x * cal1 * sal2 - T1.y * sal1 * sal2 + T1.z * cal2;
		dP12.y = -T1.y * cal1 * sal2 - T2.x * sal1 * sal2 + T2.y * cal2;
		dP12.z = -T1.z * cal1 * sal2 - T2.y * sal1 * sal2 + T2.z * cal2;
		//MC1 recycled. MC2-->MC1. MC1=MC2=M2*C
		MC1.x = M21.x * c.x + M21.y * c.y + M21.z * c.z; 
		MC1.y = M21.y * c.x + M22.x * c.y + M22.y * c.z; 
		MC1.z = M21.z * c.x + M22.y * c.y + M22.z * c.z; 

		lampow=.125f/(lam2*lam2*lam2);
		t1=.5f / lam2;
		T1.x = t1 * M21.x - lampow * MC1.x * MC1.x;	
		T1.y = t1 * M21.y - lampow * MC1.x * MC1.y;	
		T1.z = t1 * M21.z - lampow * MC1.x * MC1.z;

		T1.y = t1 * M21.y - lampow * MC1.y * MC1.x;	
		T2.x = t1 * M22.x - lampow * MC1.y * MC1.y;	
		T2.y = t1 * M22.y - lampow * MC1.y * MC1.z;

		T1.z = t1 * M21.z - lampow * MC1.z * MC1.x;
		T2.y = t1 * M22.y - lampow * MC1.z * MC1.y;	
		T2.z = t1 * M22.z - lampow * MC1.z * MC1.z;

		dP21.x = T1.x * sal1 * cal2 - T1.y * cal1*cal2;
		dP21.y = T1.y * sal1 * cal2 - T2.x * cal1*cal2;
		dP21.z = T1.z * sal1 * cal2 - T2.y * cal1*cal2;

		dP22.x = T1.x * cal1 * sal2 + T1.y * sal1 * sal2 - T1.z * cal2;
		dP22.y = T1.y * cal1 * sal2 + T2.x * sal1 * sal2 - T2.y * cal2;
		dP22.z = T1.z * cal1 * sal2 + T2.y * sal1 * sal2 - T2.z * cal2;


		//Note:	f1 = 2 * d[i] * dD1[i]; i=0,1,2
		//		f2 = 2 * d[i] * dD2[i];
		///////////*******************
		float2 gradDescent;
		gradDescent.x = dot(d, dP11 - dP21);
		gradDescent.y = dot(d, dP12 - dP22);
		distance3 = 1.0/sqrt( gradDescent.x*gradDescent.x + gradDescent.y*gradDescent.y );
		gradDescent.x = -gradDescent.x * distance3;
		gradDescent.y = -gradDescent.y * distance3;
		//pointA

		lowerBound = -PI_128;
		upperBound = PI_128;
		midPoint = 0;
		// ***** line search
		bet1 = al1 + midPoint * gradDescent.x;
		bet2 = al2 + midPoint * gradDescent.y;
		dP12.y  = distanceFuncFast(bet1, bet2, centerA, centerB, M11,M12,M21,M22,c);

		for (int i = 0; i < 5; i++)	//lineCounter --> i
		{
			mid1stInterval = .5f * (midPoint + lowerBound);
			bet1 = al1 + mid1stInterval * gradDescent.x;
			bet2 = al2 + mid1stInterval * gradDescent.y;
			dP12.x  = distanceFuncFast(bet1, bet2, centerA, centerB, M11,M12,M21,M22,c);

			mid2ndInterval = .5f * (midPoint + upperBound);
			bet1 = al1 + mid2ndInterval * gradDescent.x;
			bet2 = al2 + mid2ndInterval * gradDescent.y;
			dP12.z  = distanceFuncFast(bet1, bet2, centerA, centerB, M11,M12,M21,M22,c);
			if (dP12.x < dP12.y) {
				if (dP12.x  < dP12.z ) {
					upperBound = midPoint;
					midPoint = mid1stInterval;
					dP12.y  = dP12.x;
				} else {
					lowerBound = midPoint;
					midPoint = mid2ndInterval;
					dP12.y  = dP12.z;
				}
			} else {
				if (dP12.y  < dP12.z) {
					lowerBound = mid1stInterval;
					upperBound = mid2ndInterval;
				} else {
					lowerBound = midPoint;
					midPoint = mid2ndInterval;
					dP12.y  = dP12.z;
				}
			}
		}

		al1 += midPoint * gradDescent.x;
		al2 += midPoint * gradDescent.y;
	}
	//*****************************************
	// End of Iterative Loop
	//*****************************************
	distance = distanceFunc(al1, al2, centerA, centerB, M11,M12,M21,M22, d, c, pA, pB);
	if(dot(d, c) < 0){return false;}

	return true;
}

__device__ __host__ uint getID(const object &A){
	if(A.B.w==0)	{return A.B.x;}
	else if(A.B.w==1){return A.A.w;}
	else if(A.B.w==2){return A.A.w;}
	else if(A.B.w==3){return A.A.w;}
	else return 100;
}
__global__ void Ellipsoid_Ellipsoid(object * object_data,
									int3 * Pair ,
									uint* Contact_Number,
									contactGPU* CData,
									uint totalPossibleConts)
{
	uint Index = blockIdx.x* blockDim.x + threadIdx.x;
	if(Index>=totalPossibleConts){return;}
	if(Contact_Number[Index]!=0xFFFFFFFF){return;}
	int3 pair=Pair[Index];
	if(pair.z!=3){return;}

	object A=object_data[pair.x];
	object B=object_data[pair.y];
	float3 N,p1,p2;
	float depth;
	if(!ContactCalculation(A,B,p1,p2,N,depth)){return;}
	AddContact(CData,Index,  getID(A),getID(B), p1, p2,N,-depth);
	Contact_Number[Index]=Index;
}




__global__ void Sphere_Triangle(object * object_data,int3 * Pair ,uint* Contact_Number,contactGPU* CData,uint totalPossibleConts){
	uint Index = blockIdx.x* blockDim.x + threadIdx.x;
	if(Index>=totalPossibleConts){return;}
	if(Contact_Number[Index]!=0xFFFFFFFF){return;}
	int3 pair=Pair[Index];
	if(pair.z==1||pair.z==2){
		bool hasContact = false;
		if(pair.z==2){
			uint temp=pair.x;
			pair.x=pair.y;
			pair.y=temp;
		}
		object sphere=object_data[pair.x];
		object triangle=object_data[pair.y];
		float3 
			A=F3(triangle.A),
			B=F3(triangle.B),
			C=F3(triangle.C),
			S=F3(sphere.A),
			contactPoint;

		float3 N=cross((B-A),(C-A));
		N/=sqrtf(dot(N,N));
		float distanceFromPlane = dot(S-A,N);

		if (distanceFromPlane < 0.0f){
			distanceFromPlane *= -1.0f;	
			N *= -1.0f;
		}
		if(distanceFromPlane > sphere.A.w){return;}
		else if(distanceFromPlane < sphere.A.w+collision_envelope_const) {
			if (pointInTriangle(A,B,C,N,S)) {
				hasContact = true;
				contactPoint = S - N*distanceFromPlane;
			}else{
				float RadSqr = (sphere.A.w+collision_envelope_const)*(sphere.A.w+collision_envelope_const);
				if (SegmentSqrDistance(A,B,S, contactPoint) < RadSqr) {hasContact = true;}
				if (SegmentSqrDistance(B,C,S, contactPoint) < RadSqr) {hasContact = true;}
				if (SegmentSqrDistance(C,A,S, contactPoint) < RadSqr) {hasContact = true;}
			}
		}
		if (hasContact) {
			N = S - contactPoint;
			float distance = dot(N,N);
			if (distance < (sphere.A.w - 0.)*(sphere.A.w - 0.)) {
				distance = sqrtf(distance);
				N = N/distance;
				CData[Index].I=F3(-(sphere.A.w-distance),triangle.A.w,sphere.B.x);
				CData[Index].N=N;
				CData[Index].Pa=contactPoint;
				CData[Index].Pb=S-N*sphere.A.w;
				//CData[Index].G=F4(0,0,0,0);
				Contact_Number[Index]=Index;
				return;
			}
		}
	}
}



__device__ __host__ inline float3 GetSupportPoint_Sphere	(const object& p,const  float3 &n){		
	return (p.A.w) * normalize(n);
}
__device__ __host__ inline float3 GetSupportPoint_Triangle	(const object& p,const float3 &n){
	float3 Pa=make_float3(p.A);
	float3 Pb=make_float3(p.B);
	float3 Pc=make_float3(p.C);
	float dist = dot(Pa,n);
	float3 point=Pa;
	if(dot(Pb,n)>dist){dist=dot(Pb,n); point=Pb;}
	if(dot(Pc,n)>dist){dist=dot(Pc,n); point=Pc;}
	return point;
}
__device__ __host__ inline float3 GetSupportPoint_Box		(const object& p,const float3 &n){
	float3 result;
	result.x= n.x>=0? p.B.x:-p.B.x;
	result.y= n.y>=0? p.B.y:-p.B.y;
	result.z= n.z>=0? p.B.z:-p.B.z;
	
	return result;
}
__device__ __host__ inline float3 GetSupportPoint_Ellipsoid	(const object& p,const float3 &n){
	//return normalize(n* F3(p.B))* F3(p.B);
	return F3(p.B)*F3(p.B)*n/length(n*F3(p.B));
}
__device__ __host__ inline float3 GetSupportPoint_Cylinder	(const object& p,const float3 &n){
	return make_float3(0,0,0);
}
__device__ __host__ inline float3 GetSupportPoint_Plane		(const object& p,const float3 &n){
	float3 result = make_float3(p.B);
	if (n.x < 0) result.x = -result.x;
	if (n.y < 0) result.y = -result.y;
	return result;
}
__device__ __host__ inline float3 GetSupportPoint_Cone		(const object& p,const float3 &n){
	return make_float3(0,0,0);
}
__device__ __host__ inline float3 GetCenter_Sphere			(const object& p){
	return make_float3(0,0,0);
}
__device__ __host__ inline float3 GetCenter_Triangle		(const object& p){
	return make_float3((p.A.x+p.B.x+p.C.x)/3.0f,(p.A.y+p.B.y+p.C.y)/3.0f,(p.A.z+p.B.z+p.C.z)/3.0f);
}
__device__ __host__ inline float3 GetCenter_Box				(const object& p){return Zero_Vector;}
__device__ __host__ inline float3 GetCenter_Ellipsoid		(const object& p){return Zero_Vector;}
__device__ __host__ inline float3 GetCenter_Cylinder		(const object& p){return Zero_Vector;}
__device__ __host__ inline float3 GetCenter_Plane			(const object& p){return Zero_Vector;}
__device__ __host__ inline float3 GetCenter_Cone			(const object& p){return Zero_Vector;}
__device__ __host__ bool IsZero3(const float3 &v){
	return (	v.x < Vector_ZERO_EPSILON && v.x > -Vector_ZERO_EPSILON &&
		v.y < Vector_ZERO_EPSILON && v.y > -Vector_ZERO_EPSILON &&
		v.z < Vector_ZERO_EPSILON && v.z > -Vector_ZERO_EPSILON );
}
__device__ __host__ float3 GetCenter(const object& p){
	if(p.B.w==1){return GetCenter_Triangle(p);}	//triangle
	else{return make_float3(0,0,0)+ F3(p.A);}	//All other shapes assumed to be locally centered
}
__device__ __host__ float3 TransformSupportVert(const object& p,const float3& b){
	float3 localSupport;
	float3 n=normalize(b);
	if(p.B.w==1){//triangle
		return GetSupportPoint_Triangle(p,n);	
	}
	else if(p.B.w==0){//sphere
		localSupport = GetSupportPoint_Sphere(p,quatRotate(n,(~p.C)));	
	}
	else if(p.B.w==2){//box
		localSupport = GetSupportPoint_Box(p,quatRotate(n,(~p.C)));
	}
	else if(p.B.w==3){//ellipsoid
		localSupport = GetSupportPoint_Ellipsoid(p,quatRotate(n,(~p.C)));
	}
	else if(p.B.w==4){//cylinder
		localSupport = GetSupportPoint_Cylinder(p,quatRotate(n,(~p.C)));
	}
	else if(p.B.w==5){//plane
		localSupport = GetSupportPoint_Plane(p,quatRotate(n,(~p.C)));	
	}
	else if(p.B.w==6){//cone
		localSupport = GetSupportPoint_Cone(p,quatRotate(n,(~p.C)));
	}
	return quatRotate(localSupport,p.C) + F3(p.A); //globalSupport
}



//Code for Convex-Convex Collision detection, adopted from xeno-collide
__device__ __host__ bool CollideAndFindPoint(const object& p1, const object& p2, float3& returnNormal, float3& point1, float3& point2)
{
	float3 v01, v02, v0, n,v11, v12, v1, v21, v22, v2;
	// v0 = center of Minkowski sum
	v01 = GetCenter(p1);
	v02 = GetCenter(p2);
	v0 = v02 - v01;

	// Avoid case where centers overlap -- any direction is fine in this case
	if (IsZero3(v0)) v0 = make_float3(1, 0, 0);

	// v1 = support in direction of origin
	n = normalize(-v0);
	v11 = TransformSupportVert(p1, -n);
	v12 = TransformSupportVert(p2, n);
	v1 = v12 - v11;
	if (dot(v1 , n) <= 0){return false;}

	// v2 - support perpendicular to v1,v0
	n = cross(v1 , v0);
	if (IsZero3(n)){
		n = v1 - v0;
		n=normalize(n);
		returnNormal = n;
		point1=v11;
		point2=v12;
		return true;
	}
	v21 = TransformSupportVert(p1, -n);
	v22 = TransformSupportVert(p2, n);
	v2 = v22 - v21;
	if (dot(v2 , n) <= 0){return false;}

	// Determine whether origin is on + or - side of plane (v1,v0,v2)
	n = cross((v1 - v0) , (v2 - v0));
	// If the origin is on the - side of the plane, reverse the direction of the plane
	if (dot(n , v0) > 0){
		Swap(v1, v2);
		Swap(v11, v21);
		Swap(v12, v22);
		n = -n;
	}
	// Phase One: Identify a portal
	float3 v31, v32, v3;
	while (1){
		// Obtain the support point in a direction perpendicular to the existing plane
		// Note: This point is guaranteed to lie off the plane
		v31 = TransformSupportVert(p1, -n);
		v32 = TransformSupportVert(p2, n); 
		v3  = v32 - v31;
		if (dot(v3 , n) <= 0){return false;}
		// If origin is outside (v1,v0,v3), then eliminate v2 and loop
		else if (dot(cross(v1 , v3) , v0) < 0){
			v2 = v3;
			v21 = v31;
			v22 = v32;
			n = cross((v1 - v0) , (v3 - v0));
			continue;
		}
		// If origin is outside (v3,v0,v2), then eliminate v1 and loop
		else if (dot(cross(v3 , v2) , v0) < 0){
			v1 = v3;
			v11 = v31;
			v12 = v32;
			n = cross((v3 - v0) , (v2 - v0));
			continue;
		}
		break;
	}
	bool hit = false;

	// Phase Two: Refine the portal
	// We are now inside of a wedge...
	int phase2=0;
	while(1){
		phase2++;
		// Compute normal of the wedge face
		n = cross((v2 - v1) , (v3 - v1));
		n=n/sqrtf(dot(n,n));
		// Compute distance from origin to wedge face
		// If the origin is inside the wedge, we have a hit
		if (dot(n , v1) >= 0. && !hit){
			// Compute the barycentric coordinates of the origin
			float b0 = dot(cross(v1 , v2) , v3);
			float b1 = dot(cross(v3 , v2) , v0);
			float b2 = dot(cross(v0 , v1) , v3);
			float b3 = dot(cross(v2 , v1) , v0);

			float sum = b0 + b1 + b2 + b3;

			if (sum <= 0.){
				b0 = 0;
				b1 = dot(cross(v2 , v3) , n);
				b2 = dot(cross(v3 , v1) , n);
				b3 = dot(cross(v1 , v2) , n);

				sum = b1 + b2 + b3;
			}
			float inv = 1.0f / sum;
			point1 = (b0 * v01 + b1 * v11 + b2 * v21 + b3 * v31) * inv;
			point2 = (b0 * v02 + b1 * v12 + b2 * v22 + b3 * v32) * inv;
			hit = true;// HIT!!!
		}
		// Find the support point in the direction of the wedge face
		float3 v41 = TransformSupportVert(p1, -n);
		float3 v42 = TransformSupportVert(p2, n); 
		float3 v4 = v42 - v41;

		float delta = dot((v4 - v3) , n);
		float separation = -dot(v4 , n);

		// If the boundary is thin enough or the origin is outside the support plane for the newly discovered vertex, then we can terminate
		if ( delta <= kCollideEpsilon || separation >= 0. || phase2 > 200 ){
			
			returnNormal = normalize(n);
				//if(abs(returnNormal.x)<Vector_ZERO_EPSILON){returnNormal.x=0;}
				//if(abs(returnNormal.y)<Vector_ZERO_EPSILON){returnNormal.y=0;}
				//if(abs(returnNormal.z)<Vector_ZERO_EPSILON){returnNormal.z=0;}
			return hit;
		}
		if (dot(cross(v4 , v1) , v0) < 0.){			// Compute the tetrahedron dividing face (v4,v0,v1)
			if (dot(cross(v4 , v2) , v0) < 0){		// Compute the tetrahedron dividing face (v4,v0,v2)
				v1 = v4;	v11 = v41;	v12 = v42;	// Inside d1 & inside d2 ==> eliminate v1
			}else{
				v3 = v4;	v31 = v41;	v32 = v42;	// Inside d1 & outside d2 ==> eliminate v3
			}
		}else{
			if (dot(cross(v4 , v3) , v0) < 0.){		// Compute the tetrahedron dividing face (v4,v0,v3)
				v2 = v4;	v21 = v41;	v22 = v42;	// Outside d1 & inside d3 ==> eliminate v2
			}else{
				v1 = v4;	v11 = v41;	v12 = v42;	// Outside d1 & outside d3 ==> eliminate v1
			}
		}
	}
}
__global__ void MPR_GPU_Store(
							  object * object_data,
							  int3 * Pair ,
							  uint* Contact_Number,
							  contactGPU* CData,
							  uint totalPossibleConts)
{
	uint Index = blockIdx.x* blockDim.x + threadIdx.x;
	if(Index>=totalPossibleConts){return;}
	if(Contact_Number[Index]!=0xFFFFFFFF){return;}
	int3 pair=Pair[Index];
	//if(pair.z<4){return;}
	object A=object_data[pair.x];
	object B=object_data[pair.y];
	float3 N,p1,p2;
	if(!CollideAndFindPoint(A,B,N,p1, p2)){return;};
	p1=(TransformSupportVert(A,-N)-p1)*N*N+p1;
	p2=(TransformSupportVert(B,N)-p2)*N*N+p2;
	float depth=sqrtf(dot((p2-p1),(p2-p1)));
	AddContact(CData,Index,  getID(A),getID(B), p1, p2,-N,-depth);
	Contact_Number[Index]=Index;
}

void ChCCollisionGPU::Narrowphase(){       						//NarrowPhase Contact CD 
	generic_counter.resize(number_of_contacts);
	Thrust_Fill(generic_counter,0xFFFFFFFF);
	contact_data_gpu->resize(number_of_contacts);
	//Sphere_Sphere<<<BLOCKS(number_of_contacts),THREADS>>>(		//Compute Sphere-Sphere Contacts
	//	OBJCAST(object_data),
	//	CASTI3(contact_pair),									//Indices of bodies that make up AABB contact
	//	CASTU1(generic_counter),								//Contact Index, store the thread index
	//	CONTCAST((*contact_data_gpu)),							//Contact Data GPU
	//	number_of_contacts);									//Number of potential contacts

	//Sphere_Triangle<<<BLOCKS(number_of_contacts),THREADS>>>(	//Compute Sphere-Sphere Contacts
	//	OBJCAST(object_data),                         			//Object Data
	//	CASTI3(contact_pair),                     				//Indices of bodies that make up AABB contact
	//	CASTU1(generic_counter),								//Contact Index, store the thread index
	//	CONTCAST((*contact_data_gpu)),                          //Contact Data GPU
	//	number_of_contacts);									//Number of potential contacts
	//Ellipsoid_Ellipsoid<<<BLOCKS(number_of_contacts),THREADS>>>(		//Compute convex-covnex Contacts
	//	OBJCAST(object_data),                         			//Object Data
	//	CASTI3(contact_pair),                     				//Indices of bodies that make up AABB contact
	//	CASTU1(generic_counter),								//Contact Index, store the thread index
	//	CONTCAST((*contact_data_gpu)),                          //Contact Data GPU
	//	number_of_contacts);									//Number of potential contacts

	MPR_GPU_Store<<<BLOCKS(number_of_contacts),THREADS>>>(		//Compute convex-covnex Contacts
		OBJCAST(object_data),                         			//Object Data
		CASTI3(contact_pair),                     				//Indices of bodies that make up AABB contact
		CASTU1(generic_counter),								//Contact Index, store the thread index
		CONTCAST((*contact_data_gpu)),                          //Contact Data GPU
		number_of_contacts);									//Number of potential contacts

	Thrust_Sort_By_Key(generic_counter,(*contact_data_gpu));
	number_of_contacts=number_of_contacts-Thrust_Count(generic_counter,0xFFFFFFFF);
	contact_data_gpu->resize(number_of_contacts);
	CopyCDToCPU();
}

void ChCCollisionGPU::CopyCDToCPU(){ 
//thrust::copy(contact_data_gpu->begin(),contact_data_gpu->end(),contact_data_host.begin());


contact_data_host=*contact_data_gpu;




}  

