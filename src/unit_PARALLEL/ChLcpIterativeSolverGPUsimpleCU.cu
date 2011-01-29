///////////////////////////////////////////////////
//
//   ChLcpIterativeSolverGPUsimpleCU.cu
//
//
//    file for CHRONO HYPEROCTANT LCP solver 
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

// includes, system

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "ChCuda.h"
//#include "ChLcpIterativeSolverGPUsimple.h"
#include <cutil_math.h>

// dot product of the first three elements of two float4 values
inline __host__ __device__ float dot_three(const float4 & a, const float4 & b)
{ 
	return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline __host__ __device__ float dot_three(const float4 & a, const float3 & b)
{ 
	return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline __host__ __device__ void vect_mult_inc(float4& a, const float4 &b, const float &c)
{ 
	a.x += (b.x * c);
	a.y += (b.y * c);
	a.z += (b.z * c);
}
inline __host__ __device__ void vect_mult_inc(float3& a, const float4 &b, const float &c)
{ 
	a.x += (b.x * c);
	a.y += (b.y * c);
	a.z += (b.z * c);
}
inline __host__ __device__ void vect_mult(float4& a, const float4 &b, const float &c)
{ 
	a.x = b.x * c;
	a.y = b.y * c;
	a.z = b.z * c;
}
inline __host__ __device__ void vect_mult(float4& a, const float3 &b, const float &c)
{ 
	a.x = b.x * c;
	a.y = b.y * c;
	a.z = b.z * c;
}
inline __host__ __device__ void vect_mult(float3& a, const float4 &b, const float &c)
{ 
	a.x = b.x * c;
	a.y = b.y * c;
	a.z = b.z * c;
}
inline __host__ __device__ void vect_inertia_multiply(float4& a, const float4 &b)
{ 
	a.x *= b.x;
	a.y *= b.y;
	a.z *= b.z;
}
inline __host__ __device__ void vect_inertia_multiply(float3& a, const float4 &b)
{ 
	a.x *= b.x;
	a.y *= b.y;
	a.z *= b.z;
}
inline __host__ __device__ void vect_inertia_multiply(float4& a, const float &b)
{ 
	a.x *= b;
	a.y *= b;
	a.z *= b;
}
inline __host__ __device__ void vect_inertia_multiply(float3& a, const float &b)
{ 
	a.x *= b;
	a.y *= b;
	a.z *= b;
}


__constant__ CH_REALNUMBER deviceLcpOmega;
__constant__ unsigned int  deviceBodyPitch;
__constant__ unsigned int  deviceContactPitch;
__constant__ unsigned int  deviceBilateralPitch;
__constant__ unsigned int  deviceReductionPitch;
__constant__ unsigned int  contactsGPU;
__constant__ unsigned int  bilateralsGPU;
__constant__ unsigned int  bodiesGPU;
__constant__ unsigned int  updatesGPU;
__constant__ CH_REALNUMBER forceFactor;  // usually, the step size
__constant__ CH_REALNUMBER maxRecoverySpeedNeg;
__constant__ CH_REALNUMBER Cfactor;				// usually 1/dt
__constant__ CH_REALNUMBER stepSize;
///////////////////////////////////////////////////////////////////////////////////

texture<float4> texContacts;
texture<float4> texBody;

// Kernel for a single iteration of the LCP over all contacts
//  
//   Version 2.0 - Tasora
//
__global__ void ChKernelLCPiteration( CH_REALNUMBER4* contacts, CH_REALNUMBER4* bodies, CH_REALNUMBER3* velocity, CH_REALNUMBER3* omega, uint* offset, CH_REALNUMBER* velocityOLD) 
{
	// Compute the i,deviceContactPitch values used to access data inside the large 'contact'
	// array using pointer arithmetic.
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
	if(i>=contactsGPU){return;}
	// In all phases of the following computations, try to use only those following
	// vars, hoping this will help the compiler to minimize register requirements..
	CH_REALNUMBER3 vB; 
	CH_REALNUMBER3 a; 
	CH_REALNUMBER3 vA;
	CH_REALNUMBER eta; 

	float4 T0 = contacts[i + 0*deviceContactPitch];
	float4 T1 = contacts[i + 1*deviceContactPitch];
	float4 T2 = contacts[i + 2*deviceContactPitch];
	float4 T3 = contacts[i + 3*deviceContactPitch];
	float4 T4 = contacts[i + 4*deviceContactPitch];
	float4 T5 = contacts[i + 5*deviceContactPitch];
	int B1_index = T3.w ; //__float_as_int(vA.w);
	float4 B1=tex1Dfetch(texBody,B1_index);//bodies[B1_index];
	int B1_active = B1.w;

	// ---- perform   a = ([J1 J2] {v1 | v2}^ + b) 
	a.x = a.y = a.z = 0.0;

	if (!B1_active){
		// J1w x w1  
		vB  = F3(tex1Dfetch(texBody,B1_index+deviceBodyPitch));//bodies[B1_index+deviceBodyPitch]); // w1
		a.x = dot_three (T3 , vB);
		a.y = dot_three (T4 , vB);
		a.z = dot_three (T5 , vB);
		// -J12x x v1     ... + b 
		vB = F3(B1); // v1
		a.x -= dot_three (T0 , vB);
		a.x += T0.w; // +bx
		a.y -= dot_three ( T1 , vB);
		// a.y +=vA.w; // +by
		a.z -= dot_three ( T2 , vB);
		// a.z +=vA.w; // +bz
	}else{
		a.x += T0.w;   // +bx
		// a.y +=vA.w; // +by
		// a.z +=vA.w; // +bz
	}
	float4 T6 = contacts[i + 6*deviceContactPitch];
	float4 T7 = contacts[i + 7*deviceContactPitch];
	float4 T8 = contacts[i + 8*deviceContactPitch];
	int B2_index = T6.w ; //__float_as_int(vA.w);
	float4 B2=tex1Dfetch(texBody,B2_index);//bodies[B2_index];
	int B2_active = B2.w;
	if (!B2_active){
		// J2w x w2  
		vB = F3(tex1Dfetch(texBody,B2_index+deviceBodyPitch));//bodies[B2_index+deviceBodyPitch]); // w2
		a.x += dot_three ( T6 , vB);
		a.y += dot_three ( T7 , vB);
		a.z += dot_three ( T8 , vB);
		eta = T8.w;

		// J12x x v2  
		vB = F3(B2); // v1
		a.x += dot_three ( T0 , vB);
		a.y += dot_three ( T1 , vB);
		a.z += dot_three ( T2 , vB);
	}else{
		eta = T8.w;//contacts[i+ deviceContactPitch *8].w;
	}
	/// ---- perform a *= omega*eta

	a *= deviceLcpOmega*eta;	// deviceLcpOmega is in constant memory 
	/// ---- perform a = gammas - a ; in place.
	float4 T=contacts[deviceContactPitch *9  + i];

	vB = F3(T);
	eta=T.w;
	a = (vB) - a;

	/// ---- perform projection of 'a' onto friction cone  -----------

	//CH_REALNUMBER mu = eta; // save this register: use eta instead

	// reuse vA as registers   
	vA.y = sqrt (a.y*a.y + a.z*a.z );									// vA.y = f_tang
	if (vA.y > (eta * a.x)){											// inside upper cone? keep untouched! ( eta = friction coeff. mu )
		if ((eta * vA.y) < -a.x){										// inside lower cone? reset  normal,u,v to zero!
			a = F3(0.f,0.f,0.f);
		} else{															// remaining case: project orthogonally to generator segment of upper cone
			a.x =  ( vA.y * eta + a.x ) / (eta*eta + 1.f) ;
			vA.x = a.x * eta / vA.y;      //  vA.x = tproj_div_t
			a.y *= vA.x ;
			a.z *= vA.x ; 
		}
	}

	// ----- store gamma_new
	contacts[i + 9*deviceContactPitch ] = F4(a,eta);

	/// ---- compute delta in multipliers: a = gamma_new - gamma_old   = delta_gamma    , in place.
	a -= vB;// assuming no one touched vB={gamma_old ,mu}  during previos projection phase!

	if (!B1_active)
	{
		uint off=offset[i];
		B1=tex1Dfetch(texBody,B1_index+ 4*deviceBodyPitch);//bodies[B1_index+ 4*deviceBodyPitch];
		/// ---- compute dv1 
		vect_mult    (vB, T0, -a.x); 
		vect_mult_inc(vB, T1, -a.y); 
		vect_mult_inc(vB, T2, -a.z); 
		vect_inertia_multiply(vB,B1.w);
		vA=velocity[off];
		velocityOLD[off]=sqrtf(dot(vA-vB,vA-vB));
		velocity[off] = (vB);										//  ---> store  dv1  

		/// ---- compute dw1 =  Inert.1' * J1w^ * deltagamma
		vect_mult    (vB, T3, a.x); 
		vect_mult_inc(vB, T4, a.y); 
		vect_mult_inc(vB, T5, a.z); 
		vect_inertia_multiply(vB,B1);
		omega[off] = (vB);											//  ---> store  dw1
	}

	if (!B2_active)
	{
		uint off=offset[i+contactsGPU];
		B2=tex1Dfetch(texBody,B2_index+ 4*deviceBodyPitch);//bodies[B2_index+ 4*deviceBodyPitch];
		/// ---- compute dv2 
		vect_mult    (vB, T0, a.x); 
		vect_mult_inc(vB, T1, a.y); 
		vect_mult_inc(vB, T2, a.z); 
		vect_inertia_multiply(vB,B2.w);
		vA=velocity[off];
		velocityOLD[off]=sqrtf(dot(vA-vB,vA-vB));
		velocity[off] = (vB);							//  ---> store  dv2  

		/// ---- compute dw2 
		vect_mult    (vB, T6, a.x); 
		vect_mult_inc(vB, T7, a.y); 
		vect_mult_inc(vB, T8, a.z); 
		vect_inertia_multiply(vB,B2);
		omega[off] = (vB);								//  ---> store  dw2
	}
} 

///////////////////////////////////////////////////////////////////////////////////

// Kernel for a single iteration of the LCP over all scalar bilateral contacts
// (a bit similar to the ChKernelLCPiteration above, but without projection etc.)
//   Version 2.0 - Tasora
//



__global__ void ChKernelLCPiterationBilaterals( CH_REALNUMBER4* bilaterals, CH_REALNUMBER4* bodies, CH_REALNUMBER3* velocity, CH_REALNUMBER3* omega,uint* offset) 
{ 
	// Compute the i,bilateralContactPitch values used to access data inside the large 'bilaterals'
	// array using pointer arithmetic.
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;

	// In all phases of the following computations, try to use only those following
	// vars, hoping this will help the compiler to minimize register requirements..
	CH_REALNUMBER4 vA; 
	CH_REALNUMBER4 vB; 
	//CH_REALNUMBER4 vR; 
	CH_REALNUMBER a; 
	int B1_index=0, B1_active;
	int B2_index=0, B2_active;
	CH_REALNUMBER gamma;

	B1_index = bilaterals[i].w ; //__float_as_int(vA.w);
	B2_index = bilaterals[i+ deviceBilateralPitch].w ; //__float_as_int(vA.w);
	B1_active = bodies[B1_index].w;
	B2_active = bodies[B2_index].w;
	// ---- perform   a = ([J1 J2] {v1 | v2}^ + b) 

	a=0;

	if (B1_active==0)
	{				
		vA = bilaterals[i];						// line 0
		vB = bodies[B1_index];	// v1
		a += dot_three ( vA , vB);

		vA = bilaterals[i+2*deviceBilateralPitch]; // line 2
		vB = bodies[B1_index + deviceBodyPitch]; // w1
		a += dot_three ( vA , vB);
	}

	if (B2_active==0)
	{				
		vA = bilaterals[i+deviceBilateralPitch];		// line 1
		vB = bodies[B2_index];	// v2
		a += dot_three ( vA , vB);

		vA = bilaterals[i+3*deviceBilateralPitch];		// line 3
		vB = bodies[B2_index + deviceBodyPitch]; // w2
		a += dot_three ( vA , vB);
	}

	vA = bilaterals[i + 4*deviceBilateralPitch]; // line 4   (eta, b, gamma, 0)

	// add known term     + b
	a += vA.y;   // b

	gamma = vA.z; // old gamma


	/// ---- perform a *= omega/g_i

	a *= deviceLcpOmega;	// deviceLcpOmega is in constant memory 

	a *= vA.x;   // eta = 1/g_i;  


	/// ---- perform a = gamma - a ; in place.

	a = gamma - a;


	/// ---- perform projection of 'a' (only if simple unilateral behavior C>0 is requested)

	if (vA.w)
	{
		if (a < 0.) 
			a=0.;
	}

	// ----- store gamma_new

	vA.z = a;
	bilaterals[i + 4*deviceBilateralPitch] = vA;

	/// ---- compute delta in multipliers: a = gamma_new - gamma_old   = delta_gamma    , in place.

	a -= gamma; 

	/// --- fetch indexes to reduction buffer
	//vR= bilaterals[i + 5*deviceBilateralPitch];				// R1  R2  n1  n2

	/// ---- compute dv1 =  invInert.1 * J1^ * deltagamma  
	if (B1_active==0)
	{	
		vB = bodies[B1_index + 4*deviceBodyPitch];	// iJ iJ iJ im

		vA = bilaterals[i]; 						// line 0: J1(x)
		vA.x *= vB.w;
		vA.y *= vB.w;
		vA.z *= vB.w;
		vA *= a;
		vA.w  = 0;
		velocity[offset[2*contactsGPU+i]] = F3(vA);							//  ---> store  v1 vel. in reduction buffer	

		vA = bilaterals[i+2*deviceBilateralPitch];	// line 2:  J1(w)		  
		vA.x *= vB.x;
		vA.y *= vB.y;
		vA.z *= vB.z;
		vA *= a;
		omega[offset[2*contactsGPU+i]] = F3(vA);	// ---> store  w1 vel. in reduction buffer
		//bodyNum[2*contactsGPU+i]=B1_index;
	}
	if (B2_active==0)
	{	
		vB = bodies[B2_index + 4*deviceBodyPitch];	// iJ iJ iJ im

		vA = bilaterals[i+deviceBilateralPitch]; 	// line 1: J2(x)
		vA.x *= vB.w;
		vA.y *= vB.w;
		vA.z *= vB.w;
		vA *= a;
		vA.w  = 0;
		velocity[offset[2*contactsGPU+i+bilateralsGPU]] = F3(vA);							//  ---> store  v2 vel. in reduction buffer	

		vA = bilaterals[i+3*deviceBilateralPitch];	// line 3:  J2(w)		  
		vA.x *= vB.x;
		vA.y *= vB.y;
		vA.z *= vB.z;
		vA *= a;
		omega[offset[2*contactsGPU+i+bilateralsGPU]] = F3(vA);	// ---> store  w2 vel. in reduction buffer
		//bodyNum[2*contactsGPU+i+bilateralsGPU]=B2_index;
	}
} 


////////////////////////////////////////////////////////////////////////////////////////////////

//
// Kernel for adding invmass*force*stepsize to body speed vector.
// This kernel must be applied to the stream of the body buffer.
//



__global__ void ChKernelLCPaddForces(CH_REALNUMBER4* bodies) 
{ 
	// Compute the i values used to access data inside the large 
	// array using pointer arithmetic.
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
	if(i<bodiesGPU){
		CH_REALNUMBER4 mF; // temporary vector
		CH_REALNUMBER4 minvMasses;
		minvMasses = bodies[i+ 4*deviceBodyPitch];

		// v += m_inv * h * f
		mF =  bodies[i+ 5*deviceBodyPitch]; // vector with f (force)
		mF *= forceFactor;
		mF *= minvMasses.w; 
		bodies[i] += mF;

		// w += J_inv * h * c
		mF =  bodies[i+ 6*deviceBodyPitch]; // vector with f (torque)
		mF *= forceFactor;
		mF.x *= minvMasses.x;
		mF.y *= minvMasses.y;
		mF.z *= minvMasses.z;
		bodies[i+deviceBodyPitch] += mF;
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////


//variables already declared in device constant memory  



// Kernel for preprocessing the contact information for the CCP. 
//   Version 1.1 - Negrut
//
//  This kernel expects to find the data arranged as float4 in a horizontal
//  buffer called 'deviceContacts'.  The computation is done in place, that is, the
//  'deviceContacts' buffer is populated with extra entries while some of the old 
//  ones get overwritten.  Take a look at the MS-Word doc "dataFormatGPU.doc" in the 
//  folder "docs" to see the content of the deviceContacts buffer upon entry
//  and exit from this kernel.
__global__ void ChKernelContactsPreprocess(CH_REALNUMBER4* deviceContacts, CH_REALNUMBER4* deviceBodies) 
{
	//shared memory allocated dynamically; the way the computation is handled, there is
	//one block per multiprocessor.  Since the number of threads per block right
	//now is 512 I can statically allocate the amount of shared memory at the onset of 
	//computation
	__shared__  float shMemData[CH_SH_MEM_SIZE*CH_PREPROCESSING_TPB];

	CH_REALNUMBER4 reg03;
	CH_REALNUMBER  reg4;
	CH_REALNUMBER  reg5;
	CH_REALNUMBER4 reg69;
	CH_REALNUMBER3 regS;
	CH_REALNUMBER  eta = 0.;

	unsigned int memAddress = blockDim.x*blockIdx.x + threadIdx.x;
	int reg_uInt;

	//fetch in data from contact, regarding body 1:
	reg03 = deviceContacts[memAddress];   //reg03 contains the components of the contact normal, pointing towards the exterior of what is now considered "body 1"

	//normalize the vector
	reg4  = reg03.x;
	reg4 *= reg4;

	reg5  = reg03.y;
	reg5 *= reg5;
	reg4 += reg5;

	reg5  = reg03.z;
	reg5 *= reg5;
	reg4 += reg5;

	reg4 = sqrt(reg4); // <--- this is the magnitude of the normal

	//avoid division by zero; if normal is undefined, make it be the world X direction
	if( reg4==0. ) {
		reg03.x = 1.;
		reg03.y = 0.;
		reg03.z = 0.;
		reg4    = 1.;
	}

	//now do the scaling to get a healthy X axis
	reg03.x /= reg4; shMemData[CH_SH_MEM_SIZE*threadIdx.x  ] = reg03.x;
	reg03.y /= reg4; shMemData[CH_SH_MEM_SIZE*threadIdx.x+1] = reg03.y;
	reg03.z /= reg4; shMemData[CH_SH_MEM_SIZE*threadIdx.x+2] = reg03.z;

	//some type of Gramm Schmidt; work with the global axis that is "most perpendicular" to the contact 
	//normal vector; effectively this means looking for the smallest component (in abs) and using the 
	//corresponding direction to carry out cross product.  Some thread divergence here...
	reg4 = fabs(reg03.x);
	reg_uInt = 0;
	reg5 = fabs(reg03.y);
	if( reg4>reg5 ) {
		reg4 = reg5;
		reg_uInt = 1;
	}
	reg5 = fabs(reg03.z);
	if( reg4>reg5 ) {
		//it turns out that Z axis is closest to being perpendicular to contact vector;
		//drop stuff directly into the shared memory
		reg_uInt = 2;
		shMemData[CH_SH_MEM_SIZE*threadIdx.x+3] =  reg03.y;
		shMemData[CH_SH_MEM_SIZE*threadIdx.x+4] = -reg03.x;
		shMemData[CH_SH_MEM_SIZE*threadIdx.x+5] =      0.0;
	}

	//store stuff in shared memory
	if( reg_uInt==0 ){
		//it turns out that X axis is closest to being perpendicular to contact vector;
		//store values temporarily into the shared memory
		shMemData[CH_SH_MEM_SIZE*threadIdx.x+3] =      0.0;
		shMemData[CH_SH_MEM_SIZE*threadIdx.x+4] =  reg03.z;
		shMemData[CH_SH_MEM_SIZE*threadIdx.x+5] = -reg03.y;
	}
	else if( reg_uInt==1 ){
		//it turns out that Y axis is closest to being perpendicular to contact vector;
		//store values temporarily into the shared memory
		shMemData[CH_SH_MEM_SIZE*threadIdx.x+3] = -reg03.z;
		shMemData[CH_SH_MEM_SIZE*threadIdx.x+4] =      0.0;
		shMemData[CH_SH_MEM_SIZE*threadIdx.x+5] =  reg03.x;
	}

	//normalized the local contact Y axis (therefore automatically the 
	//local contact Z axis will be normalized and you need do nothing)
	reg4  = shMemData[CH_SH_MEM_SIZE*threadIdx.x+3];
	reg4 *= reg4;
	reg5  = shMemData[CH_SH_MEM_SIZE*threadIdx.x+4];
	reg5 *= reg5;
	reg4 += reg5;
	reg5  = shMemData[CH_SH_MEM_SIZE*threadIdx.x+5];
	reg5 *= reg5;
	reg4 += reg5;
	reg4 = sqrt(reg4);
	shMemData[CH_SH_MEM_SIZE*threadIdx.x+3] /= reg4;
	shMemData[CH_SH_MEM_SIZE*threadIdx.x+4] /= reg4;
	shMemData[CH_SH_MEM_SIZE*threadIdx.x+5] /= reg4;

	//now carry out the last cross product to find out the contact local Z axis;
	//to this end multiply the contact normal by the local Y component
	reg4 = reg03.y*shMemData[CH_SH_MEM_SIZE*threadIdx.x+5];
	reg5 = reg03.z*shMemData[CH_SH_MEM_SIZE*threadIdx.x+4];
	shMemData[CH_SH_MEM_SIZE*threadIdx.x+6] = reg4 - reg5;

	reg4 = reg03.z*shMemData[CH_SH_MEM_SIZE*threadIdx.x+3];
	reg5 = reg03.x*shMemData[CH_SH_MEM_SIZE*threadIdx.x+5];
	shMemData[CH_SH_MEM_SIZE*threadIdx.x+7] = reg4 - reg5;

	reg4 = reg03.x*shMemData[CH_SH_MEM_SIZE*threadIdx.x+4];
	reg5 = reg03.y*shMemData[CH_SH_MEM_SIZE*threadIdx.x+3];
	shMemData[CH_SH_MEM_SIZE*threadIdx.x+8] = reg4 - reg5;

	// The gap distance is the dot product <(s2-s1), contact_normal>, since contact_normal is normalized
	reg03 = deviceContacts[memAddress+6*deviceContactPitch]; //fetches s_2,w
	reg69 = deviceContacts[memAddress+3*deviceContactPitch]; //fetches s_1,w
	reg03 -= reg69;
	reg03.w  = reg03.x * shMemData[CH_SH_MEM_SIZE*threadIdx.x  ]; 
	reg03.w += reg03.y * shMemData[CH_SH_MEM_SIZE*threadIdx.x+1]; 
	reg03.w += reg03.z * shMemData[CH_SH_MEM_SIZE*threadIdx.x+2]; 
	reg03.w *= Cfactor;	

	// Clamp Anitescu stabilization coefficient: 

	if (reg03.w < maxRecoverySpeedNeg)		
		reg03.w = maxRecoverySpeedNeg;	


	//ok, a first batch of things is now computed, copy stuff back to global memory;
	//what gets passed back is --> A_c^T <--  
	//copy first column of A_c in reg03.x, reg03.y, reg03.z; note that reg03.w hols on to the gap
	reg03.x = shMemData[CH_SH_MEM_SIZE*threadIdx.x  ]; 
	reg03.y = shMemData[CH_SH_MEM_SIZE*threadIdx.x+1]; 
	reg03.z = shMemData[CH_SH_MEM_SIZE*threadIdx.x+2]; 
	deviceContacts[memAddress] = reg03;   //reg03 contains the components of the contact normal, pointing towards the exterior of what is now considered "body 1"
	memAddress += deviceContactPitch; 

	//second column of A_c
	reg03.x = shMemData[CH_SH_MEM_SIZE*threadIdx.x+3];
	reg03.y = shMemData[CH_SH_MEM_SIZE*threadIdx.x+4];
	reg03.z = shMemData[CH_SH_MEM_SIZE*threadIdx.x+5];
	reg03.w = 0.;  
	deviceContacts[memAddress] = reg03;   //reg03 contains the components of the contact Y axis
	memAddress += deviceContactPitch; 

	//third column of A_c
	reg03.x = shMemData[CH_SH_MEM_SIZE*threadIdx.x+6];
	reg03.y = shMemData[CH_SH_MEM_SIZE*threadIdx.x+7];
	reg03.z = shMemData[CH_SH_MEM_SIZE*threadIdx.x+8];
	reg03.w = 0.;
	deviceContacts[memAddress] = reg03;   //reg03 contains the components of the contact Z axis
	memAddress += deviceContactPitch; 


	//////////////////////////////////////////////////////////////////////////
	// START WORKING ON BODY ONE; ROWS 4 THROUGH 6 OF THE CONTACT STRUCTURE //
	//////////////////////////////////////////////////////////////////////////
	//bring in the location of the contact point P on body 1, as well as indexB1.
	//Store them in reg03.  Then bring the location of the body center of mass
	//and store in reg69.  The vector s_1,w is finally stored in the regS register
	memAddress = blockDim.x*blockIdx.x + threadIdx.x;
	memAddress  += (3*deviceContactPitch);
	reg03 = deviceContacts[memAddress];
	regS.x = reg03.x;
	regS.y = reg03.y;
	regS.z = reg03.z;
	reg_uInt = int (reg03.w);

	if (reg_uInt >= 0)
	{
		reg69 = deviceBodies[2*deviceBodyPitch+reg_uInt]; 
		regS.x -= reg69.x;                                
		regS.y -= reg69.y;                                
		regS.z -= reg69.z;                                

		//bring in the inertia attributes; store in shared memory; to be used to compute \eta
		reg69 = deviceBodies[4*deviceBodyPitch+reg_uInt];
		shMemData[CH_SH_MEM_SIZE*threadIdx.x+9 ] = reg69.x;  // this is the inverse of I_x  
		shMemData[CH_SH_MEM_SIZE*threadIdx.x+10] = reg69.y;  // this is the inverse of I_y
		shMemData[CH_SH_MEM_SIZE*threadIdx.x+11] = reg69.z;  // this is the inverse of I_z
		shMemData[CH_SH_MEM_SIZE*threadIdx.x+12] = reg69.w;  // this is the inverse of mass

		//bring in the Euler parameters associated with body 1; note that after this, reg03.w is not needed anymore, hang on to reguInt though
		reg69 = deviceBodies[3*deviceBodyPitch+reg_uInt];

		//start computing A_c^T \tilde s E G^T.  Start by getting the first row of A_c^T \tilde s:
		reg4  = shMemData[CH_SH_MEM_SIZE*threadIdx.x+1]*regS.z;
		reg4 -= shMemData[CH_SH_MEM_SIZE*threadIdx.x+2]*regS.y;

		reg5  = shMemData[CH_SH_MEM_SIZE*threadIdx.x+2]*regS.x;
		reg5 -= shMemData[CH_SH_MEM_SIZE*threadIdx.x  ]*regS.z;

		reg03.w  = shMemData[CH_SH_MEM_SIZE*threadIdx.x  ]*regS.y;  //<-- NOTE: reg03.w used to hold on to index for body 1, not needed anymore...
		reg03.w -= shMemData[CH_SH_MEM_SIZE*threadIdx.x+1]*regS.x;

		//next, compute the first row of A_c^T \tilde s E G^T; overwrite reg03.x, reg03.y, reg03.z
		reg4 *= 2.;
		reg5 *= 2.;
		reg03.w *= 2.;

		reg03.x  = reg4*(reg69.x*reg69.x + reg69.y*reg69.y - 0.5f);
		reg03.x += reg5*(reg69.y*reg69.z + reg69.x*reg69.w);
		reg03.x += reg03.w*(reg69.y*reg69.w - reg69.x*reg69.z);

		reg03.y  = reg4*(reg69.y*reg69.z - reg69.x*reg69.w);
		reg03.y += reg5*(reg69.x*reg69.x + reg69.z*reg69.z - 0.5f);
		reg03.y += reg03.w*(reg69.z*reg69.w + reg69.x*reg69.y);

		reg03.z  = reg4*(reg69.y*reg69.w + reg69.x*reg69.z);
		reg03.z += reg5*(reg69.z*reg69.w - reg69.x*reg69.y);
		reg03.z += reg03.w*(reg69.x*reg69.x + reg69.w*reg69.w - 0.5f);

		reg03.w = reg_uInt; //<-- index of body 1, put it back; after this, i'm ready to copy back the row in the contact structure

		deviceContacts[memAddress] = reg03;
		memAddress += deviceContactPitch;

		//before spoiling reg03, update expression of eta
		reg03.x *= reg03.x;
		reg03.x *= shMemData[CH_SH_MEM_SIZE*threadIdx.x+ 9];
		eta     += reg03.x;

		reg03.y *= reg03.y;
		reg03.y *= shMemData[CH_SH_MEM_SIZE*threadIdx.x+10];
		eta     += reg03.y;

		reg03.z *= reg03.z;
		reg03.z *= shMemData[CH_SH_MEM_SIZE*threadIdx.x+11];
		eta     += reg03.z;

		//work now on the second row of the product A_c^T \tilde s E G^T; 
		reg4  = shMemData[CH_SH_MEM_SIZE*threadIdx.x+4]*regS.z;
		reg4 -= shMemData[CH_SH_MEM_SIZE*threadIdx.x+5]*regS.y;

		reg5  = shMemData[CH_SH_MEM_SIZE*threadIdx.x+5]*regS.x;
		reg5 -= shMemData[CH_SH_MEM_SIZE*threadIdx.x+3]*regS.z;

		reg03.w  = shMemData[CH_SH_MEM_SIZE*threadIdx.x+3]*regS.y;  //<-- NOTE: reg03.w used to hold on to index for body 1, not needed anymore...
		reg03.w -= shMemData[CH_SH_MEM_SIZE*threadIdx.x+4]*regS.x;

		//next, compute the second row of A_c^T \tilde s E G^T; overwrite reg03.x, reg03.y, reg03.z
		reg4 *= 2.;
		reg5 *= 2.;
		reg03.w *= 2.;

		reg03.x  = reg4*(reg69.x*reg69.x + reg69.y*reg69.y - 0.5f);
		reg03.x += reg5*(reg69.y*reg69.z + reg69.x*reg69.w);
		reg03.x += reg03.w*(reg69.y*reg69.w - reg69.x*reg69.z);

		reg03.y  = reg4*(reg69.y*reg69.z - reg69.x*reg69.w);
		reg03.y += reg5*(reg69.x*reg69.x + reg69.z*reg69.z - 0.5f);
		reg03.y += reg03.w*(reg69.z*reg69.w + reg69.x*reg69.y);

		reg03.z  = reg4*(reg69.y*reg69.w + reg69.x*reg69.z);
		reg03.z += reg5*(reg69.z*reg69.w - reg69.x*reg69.y);
		reg03.z += reg03.w*(reg69.x*reg69.x + reg69.w*reg69.w - 0.5f);

		deviceContacts[memAddress] = reg03;
		memAddress += deviceContactPitch;

		//before spoiling reg03, update expression of eta
		reg03.x *= reg03.x;
		reg03.x *= shMemData[CH_SH_MEM_SIZE*threadIdx.x+ 9];
		eta     += reg03.x;

		reg03.y *= reg03.y;
		reg03.y *= shMemData[CH_SH_MEM_SIZE*threadIdx.x+10];
		eta     += reg03.y;

		reg03.z *= reg03.z;
		reg03.z *= shMemData[CH_SH_MEM_SIZE*threadIdx.x+11];
		eta     += reg03.z;


		//work now on the third row of the product A_c^T \tilde s E G^T; 
		reg4  = shMemData[CH_SH_MEM_SIZE*threadIdx.x+7]*regS.z;
		reg4 -= shMemData[CH_SH_MEM_SIZE*threadIdx.x+8]*regS.y;

		reg5  = shMemData[CH_SH_MEM_SIZE*threadIdx.x+8]*regS.x;
		reg5 -= shMemData[CH_SH_MEM_SIZE*threadIdx.x+6]*regS.z;

		reg03.w  = shMemData[CH_SH_MEM_SIZE*threadIdx.x+6]*regS.y;  //<-- NOTE: reg03.w used to hold on to index for body 1, not needed anymore...
		reg03.w -= shMemData[CH_SH_MEM_SIZE*threadIdx.x+7]*regS.x;

		//next, compute the third row of A_c^T \tilde s E G^T; overwrite reg03.x, reg03.y, reg03.z
		reg4 *= 2.;
		reg5 *= 2.;
		reg03.w *= 2.;

		reg03.x  = reg4*(reg69.x*reg69.x + reg69.y*reg69.y - 0.5f);
		reg03.x += reg5*(reg69.y*reg69.z + reg69.x*reg69.w);
		reg03.x += reg03.w*(reg69.y*reg69.w - reg69.x*reg69.z);

		reg03.y  = reg4*(reg69.y*reg69.z - reg69.x*reg69.w);
		reg03.y += reg5*(reg69.x*reg69.x + reg69.z*reg69.z - 0.5f);
		reg03.y += reg03.w*(reg69.z*reg69.w + reg69.x*reg69.y);

		reg03.z  = reg4*(reg69.y*reg69.w + reg69.x*reg69.z);
		reg03.z += reg5*(reg69.z*reg69.w - reg69.x*reg69.y);
		reg03.z += reg03.w*(reg69.x*reg69.x + reg69.w*reg69.w - 0.5f);

		deviceContacts[memAddress] = reg03;
		memAddress += deviceContactPitch;

		//before spoiling reg03, update expression of eta
		reg03.x *= reg03.x;
		reg03.x *= shMemData[CH_SH_MEM_SIZE*threadIdx.x+ 9];
		eta     += reg03.x;

		reg03.y *= reg03.y;
		reg03.y *= shMemData[CH_SH_MEM_SIZE*threadIdx.x+10];
		eta     += reg03.y;

		reg03.z *= reg03.z;
		reg03.z *= shMemData[CH_SH_MEM_SIZE*threadIdx.x+11];
		eta     += reg03.z;

		//add to eta the contribution that comes out of the mass and matrix A_c.
		reg4  = shMemData[CH_SH_MEM_SIZE*threadIdx.x  ];
		reg4 *= reg4;
		reg5  = reg4;

		reg4  = shMemData[CH_SH_MEM_SIZE*threadIdx.x+1];
		reg4 *= reg4;
		reg5 += reg4;

		reg4  = shMemData[CH_SH_MEM_SIZE*threadIdx.x+2];
		reg4 *= reg4;
		reg5 += reg4;

		reg4  = shMemData[CH_SH_MEM_SIZE*threadIdx.x+3];
		reg4 *= reg4;
		reg5 += reg4;

		reg4  = shMemData[CH_SH_MEM_SIZE*threadIdx.x+4];
		reg4 *= reg4;
		reg5 += reg4;

		reg4  = shMemData[CH_SH_MEM_SIZE*threadIdx.x+5];
		reg4 *= reg4;
		reg5 += reg4;

		reg4  = shMemData[CH_SH_MEM_SIZE*threadIdx.x+6];
		reg4 *= reg4;
		reg5 += reg4;

		reg4  = shMemData[CH_SH_MEM_SIZE*threadIdx.x+7];
		reg4 *= reg4;
		reg5 += reg4;

		reg4  = shMemData[CH_SH_MEM_SIZE*threadIdx.x+8];
		reg4 *= reg4;
		reg5 += reg4;

		//take care of what needs to be done for eta
		reg5 *= shMemData[CH_SH_MEM_SIZE*threadIdx.x+12];  //multiply by inverse of mass matrix of B1
		eta += reg5;
	}

	//////////////////////////////////////////////////////////////////////////
	// START WORKING ON BODY TWO; ROWS 7 THROUGH 9 OF THE CONTACT STRUCTURE //
	//////////////////////////////////////////////////////////////////////////
	//bring in the location of the contact point P on body 2, as well as indexB2.
	//Store them in reg03.  Then bring the location of the body center of mass
	//and store in reg69.  The vector s_2,w is finally stored in the regS register
	memAddress = blockIdx.x* blockDim.x + threadIdx.x;
	memAddress += (6*deviceContactPitch);
	reg03 = deviceContacts[memAddress];
	regS.x = reg03.x;
	regS.y = reg03.y;
	regS.z = reg03.z;
	reg_uInt = int (reg03.w);

	if (reg_uInt >= 0)
	{
		reg69 = deviceBodies[2*deviceBodyPitch+reg_uInt]; 
		regS.x -= reg69.x;                                
		regS.y -= reg69.y;                                
		regS.z -= reg69.z;                                

		//bring in the inertia attributes; store in shared memory; to be used to compute \eta
		reg69 = deviceBodies[4*deviceBodyPitch+reg_uInt];
		shMemData[CH_SH_MEM_SIZE*threadIdx.x+9 ] = reg69.x;  // this is the inverse of I_x  
		shMemData[CH_SH_MEM_SIZE*threadIdx.x+10] = reg69.y;  // this is the inverse of I_y
		shMemData[CH_SH_MEM_SIZE*threadIdx.x+11] = reg69.z;  // this is the inverse of I_z
		shMemData[CH_SH_MEM_SIZE*threadIdx.x+12] = reg69.w;  // this is the inverse of mass

		//bring in the Euler parameters associated with body 2; note that after this, reg03.w is not needed anymore, hang on to reguInt though
		reg69 = deviceBodies[3*deviceBodyPitch+reg_uInt];

		//start computing A_c^T \tilde s E G^T.  Start by getting the first row of A_c^T \tilde s:
		reg4  = shMemData[CH_SH_MEM_SIZE*threadIdx.x+1]*regS.z;
		reg4 -= shMemData[CH_SH_MEM_SIZE*threadIdx.x+2]*regS.y;

		reg5  = shMemData[CH_SH_MEM_SIZE*threadIdx.x+2]*regS.x;
		reg5 -= shMemData[CH_SH_MEM_SIZE*threadIdx.x  ]*regS.z;

		reg03.w  = shMemData[CH_SH_MEM_SIZE*threadIdx.x  ]*regS.y;  //<-- NOTE: reg03.w used to hold on to index for body 1, not needed anymore...
		reg03.w -= shMemData[CH_SH_MEM_SIZE*threadIdx.x+1]*regS.x;

		//next, compute the first row of A_c^T \tilde s E G^T; overwrite reg03.x, reg03.y, reg03.z
		reg4 *= 2.;
		reg5 *= 2.;
		reg03.w *= 2.;

		reg03.x  =  reg4*(reg69.x*reg69.x + reg69.y*reg69.y - 0.5f);
		reg03.x +=  reg5*(reg69.y*reg69.z + reg69.x*reg69.w);
		reg03.x +=  reg03.w*(reg69.y*reg69.w - reg69.x*reg69.z);
		reg03.x  = -reg03.x;  // <--- note the "-" sign on this quantity

		reg03.y  =  reg4*(reg69.y*reg69.z - reg69.x*reg69.w);
		reg03.y +=  reg5*(reg69.x*reg69.x + reg69.z*reg69.z - 0.5f);
		reg03.y +=  reg03.w*(reg69.z*reg69.w + reg69.x*reg69.y);
		reg03.y  = -reg03.y;  // <--- note the "-" sign on this quantity

		reg03.z  =  reg4*(reg69.y*reg69.w + reg69.x*reg69.z);
		reg03.z +=  reg5*(reg69.z*reg69.w - reg69.x*reg69.y);
		reg03.z +=  reg03.w*(reg69.x*reg69.x + reg69.w*reg69.w - 0.5f);
		reg03.z  = -reg03.z;  // <--- note the "-" sign on this quantity

		reg03.w = reg_uInt; //<-- index of body 2, put it back; after this, i'm ready to copy back the row in the contact structure

		deviceContacts[memAddress] = reg03;
		memAddress += deviceContactPitch;

		//before spoiling reg03, update expression of eta
		reg03.x *= reg03.x;
		reg03.x *= shMemData[CH_SH_MEM_SIZE*threadIdx.x+ 9];
		eta     += reg03.x;

		reg03.y *= reg03.y;
		reg03.y *= shMemData[CH_SH_MEM_SIZE*threadIdx.x+10];
		eta     += reg03.y;

		reg03.z *= reg03.z;
		reg03.z *= shMemData[CH_SH_MEM_SIZE*threadIdx.x+11];
		eta     += reg03.z;

		//work now on the second row of the product A_c^T \tilde s E G^T; 
		reg4  = shMemData[CH_SH_MEM_SIZE*threadIdx.x+4]*regS.z;
		reg4 -= shMemData[CH_SH_MEM_SIZE*threadIdx.x+5]*regS.y;

		reg5  = shMemData[CH_SH_MEM_SIZE*threadIdx.x+5]*regS.x;
		reg5 -= shMemData[CH_SH_MEM_SIZE*threadIdx.x+3]*regS.z;

		reg03.w  = shMemData[CH_SH_MEM_SIZE*threadIdx.x+3]*regS.y;  //<-- NOTE: reg03.w used to hold on to index for body 1, not needed anymore...
		reg03.w -= shMemData[CH_SH_MEM_SIZE*threadIdx.x+4]*regS.x;

		//next, compute the second row of A_c^T \tilde s E G^T; overwrite reg03.x, reg03.y, reg03.z
		reg4 *= 2.;
		reg5 *= 2.;
		reg03.w *= 2.; 

		reg03.x  =  reg4*(reg69.x*reg69.x + reg69.y*reg69.y - 0.5f);
		reg03.x +=  reg5*(reg69.y*reg69.z + reg69.x*reg69.w);
		reg03.x +=  reg03.w*(reg69.y*reg69.w - reg69.x*reg69.z);
		reg03.x  = -reg03.x;  // <--- note the "-" sign on this quantity

		reg03.y  =  reg4*(reg69.y*reg69.z - reg69.x*reg69.w);
		reg03.y +=  reg5*(reg69.x*reg69.x + reg69.z*reg69.z - 0.5f);
		reg03.y +=  reg03.w*(reg69.z*reg69.w + reg69.x*reg69.y);
		reg03.y  = -reg03.y;  // <--- note the "-" sign on this quantity

		reg03.z  =  reg4*(reg69.y*reg69.w + reg69.x*reg69.z);
		reg03.z +=  reg5*(reg69.z*reg69.w - reg69.x*reg69.y);
		reg03.z +=  reg03.w*(reg69.x*reg69.x + reg69.w*reg69.w - 0.5f);
		reg03.z  = -reg03.z;  // <--- note the "-" sign on this quantity

		deviceContacts[memAddress] = reg03;
		memAddress += deviceContactPitch;

		//before spoiling reg03, update expression of eta
		reg03.x *= reg03.x;
		reg03.x *= shMemData[CH_SH_MEM_SIZE*threadIdx.x+ 9];
		eta     += reg03.x;

		reg03.y *= reg03.y;
		reg03.y *= shMemData[CH_SH_MEM_SIZE*threadIdx.x+10];
		eta     += reg03.y;

		reg03.z *= reg03.z;
		reg03.z *= shMemData[CH_SH_MEM_SIZE*threadIdx.x+11];
		eta     += reg03.z;

		//work now on the third row of the product A_c^T \tilde s E G^T; 
		reg4  = shMemData[CH_SH_MEM_SIZE*threadIdx.x+7]*regS.z;
		reg4 -= shMemData[CH_SH_MEM_SIZE*threadIdx.x+8]*regS.y;

		reg5  = shMemData[CH_SH_MEM_SIZE*threadIdx.x+8]*regS.x;
		reg5 -= shMemData[CH_SH_MEM_SIZE*threadIdx.x+6]*regS.z;

		reg03.w  = shMemData[CH_SH_MEM_SIZE*threadIdx.x+6]*regS.y;  //<-- NOTE: reg03.w used to hold on to index for body 1, not needed anymore...
		reg03.w -= shMemData[CH_SH_MEM_SIZE*threadIdx.x+7]*regS.x;

		//next, compute the third row of A_c^T \tilde s E G^T; overwrite reg03.x, reg03.y, reg03.z
		reg4 *= 2.;
		reg5 *= 2.;
		reg03.w *= 2.;

		reg03.x  =  reg4*(reg69.x*reg69.x + reg69.y*reg69.y - 0.5f);
		reg03.x +=  reg5*(reg69.y*reg69.z + reg69.x*reg69.w);
		reg03.x +=  reg03.w*(reg69.y*reg69.w - reg69.x*reg69.z);
		reg03.x  = -reg03.x;  // <--- note the "-" sign on this quantity

		reg03.y  =  reg4*(reg69.y*reg69.z - reg69.x*reg69.w);
		reg03.y +=  reg5*(reg69.x*reg69.x + reg69.z*reg69.z - 0.5f);
		reg03.y +=  reg03.w*(reg69.z*reg69.w + reg69.x*reg69.y);
		reg03.y  = -reg03.y;  // <--- note the "-" sign on this quantity

		reg03.z  =  reg4*(reg69.y*reg69.w + reg69.x*reg69.z);
		reg03.z +=  reg5*(reg69.z*reg69.w - reg69.x*reg69.y);
		reg03.z +=  reg03.w*(reg69.x*reg69.x + reg69.w*reg69.w - 0.5f);
		reg03.z  = -reg03.z;  // <--- note the "-" sign on this quantity

		//contribution of last row to value of eta; this is slightly different 
		//since i cannot step on the values in reg03 yet.
		reg4  = reg03.x;
		reg4 *= reg4;
		reg4 *= shMemData[CH_SH_MEM_SIZE*threadIdx.x+ 9];
		eta     += reg4;

		reg4  = reg03.y;
		reg4 *= reg4;
		reg4 *= shMemData[CH_SH_MEM_SIZE*threadIdx.x+10];
		eta     += reg4;

		reg4  = reg03.z;
		reg4 *= reg4;
		reg4 *= shMemData[CH_SH_MEM_SIZE*threadIdx.x+11];
		eta     += reg4; 

		//add to eta the contribution that comes out of the mass and matrix A_c.
		reg4  = shMemData[CH_SH_MEM_SIZE*threadIdx.x  ];
		reg4 *= reg4;
		reg5  = reg4;

		reg4  = shMemData[CH_SH_MEM_SIZE*threadIdx.x+1];
		reg4 *= reg4;
		reg5 += reg4;

		reg4  = shMemData[CH_SH_MEM_SIZE*threadIdx.x+2];
		reg4 *= reg4;
		reg5 += reg4;

		reg4  = shMemData[CH_SH_MEM_SIZE*threadIdx.x+3];
		reg4 *= reg4;
		reg5 += reg4;

		reg4  = shMemData[CH_SH_MEM_SIZE*threadIdx.x+4];
		reg4 *= reg4;
		reg5 += reg4;

		reg4  = shMemData[CH_SH_MEM_SIZE*threadIdx.x+5];
		reg4 *= reg4;
		reg5 += reg4;

		reg4  = shMemData[CH_SH_MEM_SIZE*threadIdx.x+6];
		reg4 *= reg4;
		reg5 += reg4;

		reg4  = shMemData[CH_SH_MEM_SIZE*threadIdx.x+7];
		reg4 *= reg4;
		reg5 += reg4;

		reg4  = shMemData[CH_SH_MEM_SIZE*threadIdx.x+8];
		reg4 *= reg4;
		reg5 += reg4;

		//take care of what needs to be done for eta
		reg5 *= shMemData[CH_SH_MEM_SIZE*threadIdx.x+12];  //multiply by inverse of mass matrix of B1
		eta += reg5;
	}

	eta = 3./eta;  // <-- final value of eta
	reg03.w = eta; // <-- value of eta is passed back to global
	deviceContacts[( blockIdx.x* blockDim.x + threadIdx.x) + (8*deviceContactPitch)] = reg03;
} 

////////////////////////////////////////////////////////////////////////////////////////////////////

// Updates the speeds in the body buffer with values accumulated in the 
// reduction buffer:   V_new = V_old + delta_speeds

__global__ void ChKernelLCPspeedupdate(CH_REALNUMBER4* bodies , CH_REALNUMBER3* d_vel_update,CH_REALNUMBER3* d_omega_update, uint* d_body_num, uint* counter){
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
	if(i>=updatesGPU){return;}
	int start=(i>0)*counter[i-1], end=counter[i];
	int id=d_body_num[start];
	if(!bodies[id].w){
		float4 mUpdateV=F4(0);
		float4 mUpdateO=F4(0);
		for(int j=start; j<end; j++){
			mUpdateV+=make_float4((d_vel_update[j]),0);
			mUpdateO+=make_float4((d_omega_update[j]),0);
		}
		bodies[id]+=mUpdateV;
		bodies[id+deviceBodyPitch]+=mUpdateO;
	}
}


////////////////////////////////////////////////////////////////////////////////////////////////////

// Creates a quaternion as a function of a vector of rotation and an angle (the vector is assumed already 
// normalized, and angle is in radians).

inline __host__ __device__ float4 fQuaternion_from_AngAxis(const float angle, const float v_x,  const float v_y, const float v_z)
{
	float sinhalf= sinf (angle * 0.5);
	float4 quat; 
	quat.x = cosf (angle * 0.5);
	quat.y = v_x * sinhalf;
	quat.z = v_y * sinhalf;
	quat.w = v_z * sinhalf;
	return quat;
}


/// The quaternion becomes the quaternion product of the two quaternions A and B: 
/// following the classic Hamilton rule:  this=AxB
/// This is the true, typical quaternion product. It is NOT commutative.

inline __host__ __device__ float4 fQuaternion_product  (const float4 qa, const float4 qb) 
{
	float4 quat;
	quat.x = qa.x * qb.x - qa.y * qb.y - qa.z * qb.z - qa.w * qb.w;
	quat.y = qa.x * qb.y + qa.y * qb.x - qa.w * qb.z + qa.z * qb.w;
	quat.z = qa.x * qb.z + qa.z * qb.x + qa.w * qb.y - qa.y * qb.w;
	quat.w = qa.x * qb.w + qa.w * qb.x - qa.z * qb.y + qa.y * qb.z;
	return quat;
}




//  Kernel for performing the time step integration (with 1st order Eulero)
//  on the body data stream.
//
//                 -- number of registers: 12 (suggested 320 threads/block)



__global__ void  ChKernelIntegrateTimeStep(CH_REALNUMBER4* bodies, bool normalize_quaternion) 
{ 
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
	if(i>=bodiesGPU){return;}

	bodies[deviceBodyPitch *2 + i] = bodies[deviceBodyPitch *2 + i]+bodies[deviceBodyPitch *0 + i]*stepSize;  // Do 1st order integration of linear speeds

	// Do 1st order integration of quaternion position as q[t+dt] = qw_abs^(dt) * q[dt] = q[dt] * qw_local^(dt)
	// where qw^(dt) is the quaternion { cos(0.5|w|), wx/|w| sin(0.5|w|), wy/|w| sin(0.5|w|), wz/|w| sin(0.5|w|)}^dt
	// that is argument of sine and cosines are multiplied by dt.
	
	CH_REALNUMBER4 Rw = bodies[deviceBodyPitch *1 + i];

	CH_REALNUMBER wlen = sqrtf(Rw.x*Rw.x + Rw.y*Rw.y + Rw.z*Rw.z);
	if (fabs(wlen)>10e-10){
		Rw = fQuaternion_from_AngAxis ((stepSize*wlen) , Rw.x/wlen, Rw.y/wlen, Rw.z/wlen);
	}else{
		Rw.x = 1.f ; Rw.y=Rw.z=Rw.w=0.f;			// to avoid singularity for near zero angular speed
	}
	CH_REALNUMBER4 mq = fQuaternion_product (bodies[deviceBodyPitch *3 + i], Rw); 

	mq /= sqrtf(mq.x*mq.x + mq.y*mq.y + mq.z*mq.z + mq.w*mq.w);

	bodies[deviceBodyPitch *3 + i] = mq;
} 

__global__ void ChKernelLCPcomputeOffsets(CH_REALNUMBER4* contacts ,CH_REALNUMBER4* bilaterals,  uint* Body){
	uint i = blockIdx.x * blockDim.x + threadIdx.x;
	if(i<contactsGPU){
		Body[i]								=contacts[i+ deviceContactPitch *3].w;
		Body[i+contactsGPU]					=contacts[i+ deviceContactPitch *6].w;
	}
	if(i<bilateralsGPU){
		Body[2*contactsGPU+i]				=bilaterals[i].w;
		Body[2*contactsGPU+i+bilateralsGPU]	=bilaterals[i+deviceBilateralPitch].w;
	}
}

namespace chrono{
	void ChRunSolverTimestep(
		CH_REALNUMBER max_recovery_speed,
		CH_REALNUMBER mCfactor,
		CH_REALNUMBER mstepSize,
		CH_REALNUMBER tolerance,
		uint bodies_data_pitch,
		uint contacts_data_pitch,
		uint bilaterals_data_pitch,
		//uint reduction_data_pitch,
		float mLcpOmega,
		uint max_iterations,
		uint n_bilaterals_GPU,
		uint n_contacts_GPU,
		uint n_bodies_GPU,
		float4* d_buffer_contacts,
		thrust::host_vector<float4> &h_bodies,
		thrust::host_vector<float4> &h_bilaterals){
			CUT_CHECK_ERROR("START Solver");	
			/*cout<<max_recovery_speed<<" "
			<<mCfactor<<" "
			<<mstepSize<<" "
			<<bodies_data_pitch<<" "
			<<contacts_data_pitch<<" "
			<<bilaterals_data_pitch<<" "
			<<mLcpOmega<<" "
			<<max_iterations<<" "
			<<n_bilaterals_GPU<<" "
			<<n_contacts_GPU<<" "
			<<n_bodies_GPU;*/
			unsigned int n_total_constraints=n_contacts_GPU+n_bilaterals_GPU;

			thrust::device_vector<float4>	d_buffer_bodies		=h_bodies;
			thrust::device_vector<float4>	d_buffer_bilaterals	=h_bilaterals;
			thrust::device_vector<float3>	d_buffer_vel		((n_total_constraints)*2,F3(0));
			thrust::device_vector<float3>	d_buffer_omega		((n_total_constraints)*2,F3(0));
			thrust::device_vector<uint>		d_bodyNum			((n_total_constraints)*2,0);
			thrust::device_vector<uint>		d_updateNum			((n_total_constraints)*2,0);
			thrust::device_vector<uint>		d_offset_counter	((n_total_constraints)*2,0);
			thrust::device_vector<uint>		d_update_offset		((n_total_constraints)*2,0);
			thrust::device_vector<float>	d_delta_correction	((n_total_constraints)*2,0);

			//cudaBindTexture( NULL, texContacts,   CASTF4(d_buffer_contacts),   contacts_data_pitch*CH_CONTACT_VSIZE*CH_CONTACT_HSIZE );
			cudaBindTexture( NULL, texBody,   CASTF4(d_buffer_bodies),   h_bodies.size()*sizeof(float4)*CH_BODY_VSIZE );
			CUT_CHECK_ERROR("Allocate Memory");	
			CH_REALNUMBER mforceFactor=1.0;
			CH_REALNUMBER negated_recspeed = -max_recovery_speed;

			CUDA_SAFE_CALL(cudaMemcpyToSymbolAsync(deviceBodyPitch,		&bodies_data_pitch,		sizeof(bodies_data_pitch)));		//body_pitch should end up in constant memory
			CUDA_SAFE_CALL(cudaMemcpyToSymbolAsync(deviceContactPitch,	&contacts_data_pitch,	sizeof(contacts_data_pitch)));		//contact pitch should end up in constant memory
			CUDA_SAFE_CALL(cudaMemcpyToSymbolAsync(Cfactor,				&mCfactor,				sizeof(mCfactor)));					//stepsize should end up in constant memory
			CUDA_SAFE_CALL(cudaMemcpyToSymbolAsync(maxRecoverySpeedNeg,	&negated_recspeed,		sizeof(negated_recspeed)));			//clamp of recovery speed should end up in constant memory
			CUDA_SAFE_CALL(cudaMemcpyToSymbolAsync(deviceBilateralPitch,&bilaterals_data_pitch,	sizeof(bilaterals_data_pitch)));	//bilaterals pitch should end up in constant memory
			CUDA_SAFE_CALL(cudaMemcpyToSymbolAsync(deviceLcpOmega,		&mLcpOmega,				sizeof(mLcpOmega)));				//omega should end up in constant memory
			CUDA_SAFE_CALL(cudaMemcpyToSymbolAsync(forceFactor,			&mforceFactor,			sizeof(mforceFactor)));				//stepsize should end up in constant memory
			CUDA_SAFE_CALL(cudaMemcpyToSymbolAsync(stepSize,			&mstepSize,				sizeof(mstepSize)));				//stepsize should end up in constant memory
			CUDA_SAFE_CALL(cudaMemcpyToSymbolAsync(contactsGPU,			&n_contacts_GPU,		sizeof(n_contacts_GPU)));			//number of contacts should end up in constant memory
			CUDA_SAFE_CALL(cudaMemcpyToSymbolAsync(bilateralsGPU,		&n_bilaterals_GPU,		sizeof(n_bilaterals_GPU)));			//number of bilaterals should end up in constant memory
			CUDA_SAFE_CALL(cudaMemcpyToSymbolAsync(bodiesGPU,			&n_bodies_GPU,			sizeof(n_bodies_GPU)));				//number of bodies should end up in constant memory

			CUT_CHECK_ERROR("COPY CONSTANTS");	

			ChKernelContactsPreprocess	<<< max(ceil(n_contacts_GPU		/double(CH_PREPROCESSING_TPB)),1.0)	, CH_PREPROCESSING_TPB	>>>(d_buffer_contacts, CASTF4(d_buffer_bodies));						CUT_CHECK_ERROR("ChKernelContactsPreprocess");
			ChKernelLCPaddForces		<<< max(ceil(n_bodies_GPU		/double(CH_LCPADDFORCES_TPB)),1.0)	, CH_LCPADDFORCES_TPB	>>>(CASTF4(d_buffer_bodies) );											CUT_CHECK_ERROR("ChKernelLCPaddForces");	
			ChKernelLCPcomputeOffsets	<<< max(ceil(n_total_constraints/double(CH_PREPROCESSING_TPB)),1.0)	, CH_PREPROCESSING_TPB	>>>(d_buffer_contacts,CASTF4(d_buffer_bilaterals),CASTU1(d_bodyNum));	CUT_CHECK_ERROR("ChKernelLCPcomputeOffsets");

			thrust::sequence(d_updateNum.begin(),d_updateNum.end());
			thrust::sequence(d_update_offset.begin(),d_update_offset.end());

			thrust::fill(d_offset_counter.begin(),d_offset_counter.end(),0);

			thrust::sort_by_key(d_bodyNum.begin(),d_bodyNum.end(),d_updateNum.begin());
			thrust::sort_by_key(d_updateNum.begin(),d_updateNum.end(),d_update_offset.begin());

			int updates=thrust::reduce_by_key(d_bodyNum.begin(), d_bodyNum.end(), thrust::constant_iterator<uint>(1), d_updateNum.begin(), d_offset_counter.begin()).first-d_updateNum.begin();
			thrust::inclusive_scan(d_offset_counter.begin(), d_offset_counter.end(), d_offset_counter.begin());
			
			CUDA_SAFE_CALL(cudaMemcpyToSymbolAsync(updatesGPU,			&updates,			sizeof(updates)));
				int iter=0;
			if(n_contacts_GPU>0||n_bilaterals_GPU>0){
			
				for (iter = 0; iter < max_iterations; iter++){
					if(n_contacts_GPU){
						ChKernelLCPiteration<<< max(ceil((n_contacts_GPU)/((double)CH_LCPITERATION_TPB) ),1.0), CH_LCPITERATION_TPB >>>(
							d_buffer_contacts, 
							CASTF4(d_buffer_bodies), 
							CASTF3(d_buffer_vel), 
							CASTF3(d_buffer_omega) ,
							CASTU1(d_update_offset),
							CASTF1(d_delta_correction)
							);
						CUT_CHECK_ERROR("ChKernelLCPiteration");
					}
					/*if(n_bilaterals_GPU){
						ChKernelLCPiterationBilaterals<<< ceil((n_bilaterals_GPU)/((double)CH_LCPITERATIONBILATERALS_TPB)), CH_LCPITERATIONBILATERALS_TPB >>>(
							CASTF4(d_buffer_bilaterals), 
							CASTF4(d_buffer_bodies), 
							CASTF3(d_buffer_vel), 
							CASTF3(d_buffer_omega), 
							CASTU1(d_update_offset));
						CUT_CHECK_ERROR("ChKernelLCPiterationBilaterals");
					}*/
					ChKernelLCPspeedupdate<<< max(ceil((n_contacts_GPU+n_bilaterals_GPU)/((double)CH_PREPROCESSING_TPB)),1.0), CH_PREPROCESSING_TPB >>>(
						CASTF4(d_buffer_bodies), 
						CASTF3(d_buffer_vel),
						CASTF3(d_buffer_omega),  
						CASTU1(d_bodyNum),
						CASTU1(d_offset_counter));
					if(iter%10==0){
						int position = thrust::max_element(d_delta_correction.begin(), d_delta_correction.end()) - d_delta_correction.begin();
						float value = d_delta_correction[position]; 
						if(value<tolerance){break;}
					}
				}
			}
			cout<<" "<<iter<<" ";
			ChKernelIntegrateTimeStep<<< max(ceil(n_bodies_GPU/double(CH_LCPINTEGRATE_TPB)),1.0), CH_LCPINTEGRATE_TPB >>>(CASTF4(d_buffer_bodies), true);
			CUT_CHECK_ERROR("ChKernelIntegrateTimeStep");

			h_bodies		=	d_buffer_bodies;
			h_bilaterals	=	d_buffer_bilaterals;
			cudaUnbindTexture( texBody );
	}

}