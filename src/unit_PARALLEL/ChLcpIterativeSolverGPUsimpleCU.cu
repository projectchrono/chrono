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
#include "ChLcpIterativeSolverGPUsimpleCU.h"
#include <cutil_math.h>



// Kernels for solving the CCP complementarity problem in GPU. 
//
//  These kernels expects to find the data arranged as blocks of
//  float4 data (columns of N 'four4' structures) in horizontal
//  buffers called 'contacts', 'bilaterals, 'bodies' etc.
//
//  The data model is represented below, using some ascii-art.
//  In the following schemes, we use these symbols:
//   B1 and B2 are indexes pointing to bodies in body buffer. For inactive body, B1=-1 or B2=-1
//   R1 and R2 are indexes pointing to reduction buffer cell. Senseless for inactive bodies.
//   n1 and n2 tell the repetition index of the body (0,1,2,..) in reduction buffer.
// 
//   
//  'contacts' buffer is made with an horizontal array of:
//		[             , bx ]      0
//		[ matr.J12(x) ,  0 ]      1
//		[_____________,  0 ]      2
//		[             , B1 ]	  3
//		[ matr.J1(w)  ,    ]	  4
//		[_____________,    ]	  5
//		[             , B2 ]	  6
//		[ matr.J2(w)  ,    ]	  7
//		[_____________, et ]	  8
//		[ gx,  gy,  gz, mu ]      9
//      [ R1,  R2,  n1, n2 ]     10     index to fill the reduction buffer, and n repetition
//   
//  'bilaterals' buffer is made with an horizontal array of:
//		[ matr.J1(x)  , B1 ]      0
//		[ matr.J2(x)  , B2 ]      1
//		[ matr.J1(w)  ,    ]      2
//		[ matr.J2(w)  ,    ]      3
//      [ eta, b, g   ,  u ]      4      u=1 if unilateral, 0 if bilateral
//      [ R1 , R2, n1 , n2 ]      5      index to fill the reduction buffer, and n repetition
//
//  'bodies' buffer is made with an horizontal array of:  
//		[ vx, vy, vz  , R  ]       0     R= body index in reduction buffer
//		[ wx, wy, wz  ,    ]       1
//		[ xx, xy, xz  , -  ]       2
//		[ q0, q1, q2 , q3  ]       3
//		[iJ1,iJ2,iJ3  ,im  ]       4
//		[ fx, fy, fz  , -  ]       5
//		[ cx, cy, cz  , -  ]       6
//
//  'reduction' buffer is made with an horizontal array of:
//		[ vx, vy, vz  , - ]       0
//		[ wx, wy, wz  , n ]       1   n=repetition (0=no repetition, also 0=final accumulators)
//
//  'variables' buffer is made with an horizontal array of: (optional to bodies for future development)
//		[ vx, vy, vz  , R  ]       0     R= body index in reduction buffer
//		[ wx, wy, wz  ,    ]       1
//			([ xx, xy, xz  , -  ]       2
//			([ q0, q1, q2 , q3  ]       3
//		[iJ1,iJ2,iJ3  ,im  ]       4
//		[ fx, fy, fz  , -  ]       5
//		[ cx, cy, cz  , -  ]       6
//
//  Note that the contacts are uploaded to the GPU in a compressed format, 
//  that is later transformed in the above 'contact' buffer thank to the ChKernelContactsPreprocess
//  kernel. Such kernel prepares the 'contacts' buffer in-place, starting from the state below:
//
//  'contacts' buffer  before preprocessing:
//		[   Normal    , 0  ]      0
//		[  -   -   -  ,  - ]      1
//		[  -   -   -  ,  - ]      2
//		[     P1      , B1 ]	  3
//		[  -   -   -  ,  - ]	  4
//		[  -   -   -  ,  - ]	  5
//		[     P2      , B2 ]	  6
//		[  -   -   -  ,  - ]	  7
//		[  -   -   -  ,  - ]	  8
//		[ gx,  gy,  gz, mu ]      9
//      [  -   -   -  ,  - ]     10  




// dot product of the first three elements of two float4 values
inline __host__ __device__ float dot_three(float4 a, float4 b)
{ 
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline __host__ __device__ void vect_mult_inc(float4& a, float4 b, float c)
{ 
    a.x += (b.x * c);
	a.y += (b.y * c);
	a.z += (b.z * c);
}
inline __host__ __device__ void vect_mult(float4& a, float4 b, float c)
{ 
    a.x = b.x * c;
	a.y = b.y * c;
	a.z = b.z * c;
}
inline __host__ __device__ void vect_inertia_multiply(float4& a, float4 b)
{ 
    a.x *= b.x;
	a.y *= b.y;
	a.z *= b.z;
}


__constant__ CH_REALNUMBER deviceLcpOmega;
__constant__ unsigned int  deviceBodyPitch;
__constant__ unsigned int  deviceContactPitch;
__constant__ unsigned int  deviceBilateralPitch;
__constant__ unsigned int  deviceReductionPitch;





///////////////////////////////////////////////////////////////////////////////////


// Kernel for a single iteration of the LCP over all contacts
//  
//   Version 2.0 - Tasora
//


__global__ void  
ChKernelLCPiteration__OLD_( CH_REALNUMBER4* contacts, CH_REALNUMBER4* bodies, CH_REALNUMBER4* reduction) 
{
	// Compute the i,deviceContactPitch values used to access data inside the large 'contact'
	// array using pointer arithmetic.
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;

	// In all phases of the following computations, try to use only those following
	// vars, hoping this will help the compiler to minimize register requirements..
	CH_REALNUMBER4 vA; 
	CH_REALNUMBER4 vB; 
	CH_REALNUMBER3 a; 
	unsigned int B1_index;
	unsigned int B2_index;
	CH_REALNUMBER eta; 
	unsigned int address;

	// ---- perform   a = ([J1 J2] {v1 | v2}^ + b) 

	// J1w x w1  
	address = deviceContactPitch *3 + i; // line 3
	vA = contacts[address];
	B1_index = vA.w ; //__float_as_int(vA.w); 
	B1_index += deviceBodyPitch;
	vB = bodies[B1_index]; // w1
	a.x = dot_three ( vA , vB);

	address += deviceContactPitch; // line 4
	vA = contacts[address];
	a.y = dot_three ( vA , vB);
 
	address += deviceContactPitch; // line 5
	vA = contacts[address];
	a.z = dot_three ( vA , vB);

	// J2w x w2  
	address += deviceContactPitch; // line 6
	vA = contacts[address];
	B2_index = vA.w; // __float_as_int(vA.w);
	B2_index += deviceBodyPitch;
	vB = bodies[B2_index]; // w2
	a.x += dot_three ( vA , vB);

	address += deviceContactPitch; // line 7
	vA = contacts[address];
	a.y += dot_three ( vA , vB);
 
	address += deviceContactPitch; // line 8
	vA = contacts[address];
	a.z += dot_three ( vA , vB);
	eta = vA.w;


	// -J12x x v1     ... + b 
	address = i;			// line 0
	vA = contacts[address];
	B1_index -= deviceBodyPitch;
	vB = bodies[B1_index]; // v1
	a.x -= dot_three ( vA , vB);  // note: -= because same J12x, but negative sign
	a.x += vA.w; // +bx

	address += deviceContactPitch; // line 1
	vA = contacts[address];
	a.y -= dot_three ( vA , vB);
	a.y += vA.w; // +by

	address += deviceContactPitch; // line 2
	vA = contacts[address];
	a.z -= dot_three ( vA , vB);
	a.z += vA.w; // +bz

	// J12x x v2   
	//vA = contacts[address]; // don't fetch - reuse the last
	B2_index -= deviceBodyPitch;
	vB = bodies[B2_index]; // v2
	a.z += dot_three ( vA , vB);  // note: += because same J12x, but positive sign

	address -= deviceContactPitch; // line 1
	vA = contacts[address];
	a.y += dot_three ( vA , vB);

	address -= deviceContactPitch; // line 0
	vA = contacts[address];
	a.x += dot_three ( vA , vB);

	/// ---- perform a *= omega*eta

	a *= deviceLcpOmega;	// deviceLcpOmega is in constant memory 

	a *= eta;  

	/// ---- perform a = gammas - a ; in place.

	vB = contacts[deviceContactPitch *9  + i];
	a.x = vB.x - a.x;
	a.y = vB.y - a.y;
	a.z = vB.z - a.z; 
	

	/// ---- perform projection of 'a' onto friction cone  -----------

	//CH_REALNUMBER mu = vB.w; // save this register: use vB.w instead

		// reuse vA as registers   
	vA.w = sqrt (a.y*a.y + a.z*a.z ); // vA.w = f_tang

		// inside upper cone? keep untouched! ( vB.w = friction coeff. mu )
	if (vA.w <= (vB.w * a.x))
	{}
	else
	{
		// inside lower cone? reset  normal,u,v to zero!
		if ((vB.w * vA.w) < -a.x)
		{
			a.x = 0.f;
			a.y = 0.f;
			a.z = 0.f;
		} 
		else
		{
			// remaining case: project orthogonally to generator segment of upper cone
			a.x =  ( vA.w * vB.w + a.x ) / (vB.w*vB.w + 1.f) ;
			vA.z = a.x * vB.w / vA.w;      //  vA.z = tproj_div_t
			a.y *= vA.z ;
			a.z *= vA.z ; 
		}
	}


	// ----- store gamma_new

	vA.x = a.x;
	vA.y = a.y;
	vA.z = a.z;
	vA.w = vB.w;
	contacts[i + 9*deviceContactPitch ] = vA;

	/// ---- compute delta in multipliers: a = gamma_new - gamma_old   = delta_gamma    , in place.

	a.x -= vB.x;
	a.y -= vB.y;
	a.z -= vB.z; // assuming no one touched vB={gamma_old ,mu}  during previos projection phase!


	/// ---- compute dw1 =  Inert.1 * J1w^ * deltagamma
	address = i + 5*deviceContactPitch;   // line 5
	vA = contacts[address];
	vect_mult(vB, vA, a.z);  
	address -= deviceContactPitch;		  // line 4
	vA = contacts[address];
	vect_mult_inc(vB, vA, a.y); 
	address -= deviceContactPitch;		  // line 3
	vA = contacts[address]; 
	vect_mult_inc(vB, vA, a.x);
	vB.w = 0;

	B1_index += 4 * deviceBodyPitch;
	vA = bodies[B1_index];				// J1 J1 J1 m1
	eta=vA.w; // temp.storage of m1
	vect_inertia_multiply(vB,vA);
	
	vA= contacts[address + 7*deviceContactPitch];			// R1  R2  n1  n2
	vB.w = vA.z;											// the n1 repetition counter 

	reduction[((unsigned int)vA.x) + deviceReductionPitch] = vB;			// ---> store  w1 vel. in reduction buffer

	address -= deviceContactPitch;			  // line 2
	vA = contacts[address];
	vect_mult(vB, vA, a.z); 
	address -= deviceContactPitch;			  // line 1
	vA = contacts[address]; 
	vect_mult_inc(vB, vA, a.y); 
	address -= deviceContactPitch;			  // line 0
	vA = contacts[address]; 
	vect_mult_inc(vB, vA, a.x); 
	vB.w = 0;

	vA.x = eta; 
	vA.y = eta; 
	vA.z = eta;  // vA is:  m1 m1 m1; 
	vect_inertia_multiply(vB,vA);

	vA= contacts[address + 10*deviceContactPitch];			// R1  R2  n1  n2
	vB *= -1; // ***TO OPTIMIZE*** (change sign because jacob. J12 is the same of v1, but with changed sign)

	reduction[(unsigned int)vA.x] = vB;						//  ---> store  v1 vel. 

	address = i + 6*deviceContactPitch;	       // line 6
	vA = contacts[address]; 
	vect_mult(vB, vA, a.x);  
	address += deviceContactPitch;			   // line 7 
	vA = contacts[address]; 
	vect_mult_inc(vB, vA, a.y); 
	address += deviceContactPitch;			   // line 8
	vA = contacts[address]; 
	vect_mult_inc(vB, vA, a.z); 
	vB.w = 0;	
	
	B2_index += 4 * deviceBodyPitch;
	vA = bodies[B2_index];					 // J2 J2 J2 m2
	eta=vA.w; // temp.storage of m2
	vect_inertia_multiply(vB,vA);

	vA= contacts[address + 2*deviceContactPitch];			// R1  R2  n1  n2
	vB.w = vA.w;											// the n2 repetition counter 

	reduction[((unsigned int)vA.y) + deviceReductionPitch] = vB;			// ---> store  w2 vel. in reduction buffer

	address = i;							   // line 0
	vA = contacts[i]; 
	vect_mult(vB, vA, a.x);  
	address += deviceContactPitch;			   // line 1
	vA = contacts[address]; 
	vect_mult_inc(vB, vA, a.y); 
	address += deviceContactPitch;			   // line 2
	vA = contacts[address]; 
	vect_mult_inc(vB, vA, a.z); 
	vB.w = 0;

	vA.x = eta;  
	vA.y = eta; 
	vA.z = eta;  // vA is:  m m m; 
	vect_inertia_multiply(vB,vA);

	vA= contacts[address + 8*deviceContactPitch];			// R1  R2  n1  n2

	reduction[(unsigned int)vA.y] = vB;									//  ---> store  v2 vel. in reduction buffer

} 


// A less optimized but more readable version of the previous, for debugging purposes..

__global__ void  
ChKernelLCPiteration( CH_REALNUMBER4* contacts, CH_REALNUMBER4* bodies, CH_REALNUMBER4* reduction) 
{
	// Compute the i,deviceContactPitch values used to access data inside the large 'contact'
	// array using pointer arithmetic.
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;

	// In all phases of the following computations, try to use only those following
	// vars, hoping this will help the compiler to minimize register requirements..
	CH_REALNUMBER4 vA; 
	CH_REALNUMBER4 vB; 
	CH_REALNUMBER3 a; 
	int B1_index;
	int B2_index;
	CH_REALNUMBER eta; 

	B1_index = contacts[i+ deviceContactPitch *3].w ; //__float_as_int(vA.w);
	B2_index = contacts[i+ deviceContactPitch *6].w ; //__float_as_int(vA.w);

	// ---- perform   a = ([J1 J2] {v1 | v2}^ + b) 

	a.x = a.y = a.z = 0.0;

	if (B1_index >= 0)
	{
		// J1w x w1  
		vB = bodies[B1_index+deviceBodyPitch]; // w1
		vA = contacts[i+ deviceContactPitch *3]; 
		a.x = dot_three (vA , vB);

		vA = contacts[i+ deviceContactPitch *4];
		a.y = dot_three ( vA , vB);
	 
		vA = contacts[i+ deviceContactPitch *5];
		a.z = dot_three ( vA , vB);

		// -J12x x v1     ... + b 
		vB = bodies[B1_index]; // v1
		vA = contacts[i+ deviceContactPitch *0]; 
		a.x -= dot_three (vA , vB);
		a.x += vA.w; // +bx

		vA = contacts[i+ deviceContactPitch *1];
		a.y -= dot_three ( vA , vB);
		// a.y +=vA.w; // +by

		vA = contacts[i+ deviceContactPitch *2];
		a.z -= dot_three ( vA , vB);
		// a.z +=vA.w; // +bz
	}
	else
	{
		vA = contacts[i+ deviceContactPitch *0]; 
		a.x += vA.w;   // +bx
		// a.y +=vA.w; // +by
		// a.z +=vA.w; // +bz
	}

	if (B2_index >= 0)
	{
		// J2w x w2  
		vB = bodies[B2_index+deviceBodyPitch]; // w2
		vA = contacts[i+ deviceContactPitch *6]; 
		a.x += dot_three (vA , vB);

		vA = contacts[i+ deviceContactPitch *7];
		a.y += dot_three ( vA , vB);
	 
		vA = contacts[i+ deviceContactPitch *8];
		a.z += dot_three ( vA , vB);

		eta = vA.w;

		// J12x x v2  
		vB = bodies[B2_index]; // v1
		vA = contacts[i+ deviceContactPitch *0]; 
		a.x += dot_three (vA , vB);

		vA = contacts[i+ deviceContactPitch *1];
		a.y += dot_three ( vA , vB);

		vA = contacts[i+ deviceContactPitch *2];
		a.z += dot_three ( vA , vB);
	}
	else
	{
		eta = contacts[i+ deviceContactPitch *8].w;
	}

	/// ---- perform a *= omega*eta

	a *= deviceLcpOmega;	// deviceLcpOmega is in constant memory 

	a *= eta;  

	/// ---- perform a = gammas - a ; in place.

	vB = contacts[deviceContactPitch *9  + i];
	a.x = vB.x - a.x;
	a.y = vB.y - a.y;
	a.z = vB.z - a.z; 
	

	/// ---- perform projection of 'a' onto friction cone  -----------

	//CH_REALNUMBER mu = vB.w; // save this register: use vB.w instead

		// reuse vA as registers   
	vA.w = sqrt (a.y*a.y + a.z*a.z ); // vA.w = f_tang

		// inside upper cone? keep untouched! ( vB.w = friction coeff. mu )
	if (vA.w <= (vB.w * a.x))
	{}
	else
	{
		// inside lower cone? reset  normal,u,v to zero!
		if ((vB.w * vA.w) < -a.x)
		{
			a.x = 0.f;
			a.y = 0.f;
			a.z = 0.f;
		} 
		else
		{
			// remaining case: project orthogonally to generator segment of upper cone
			a.x =  ( vA.w * vB.w + a.x ) / (vB.w*vB.w + 1.f) ;
			vA.z = a.x * vB.w / vA.w;      //  vA.z = tproj_div_t
			a.y *= vA.z ;
			a.z *= vA.z ; 
		}
	}

	// ----- store gamma_new

	vA.x = a.x;
	vA.y = a.y;
	vA.z = a.z;
	vA.w = vB.w;
	contacts[i + 9*deviceContactPitch ] = vA;

	/// ---- compute delta in multipliers: a = gamma_new - gamma_old   = delta_gamma    , in place.

	a.x -= vB.x;
	a.y -= vB.y;
	a.z -= vB.z; // assuming no one touched vB={gamma_old ,mu}  during previos projection phase!

	if (B1_index >= 0)
	{
		/// ---- compute dv1 
		vA = contacts[i + 0*deviceContactPitch];
		vect_mult    (vB, vA, -a.x); 
		vA = contacts[i + 1*deviceContactPitch];
		vect_mult_inc(vB, vA, -a.y); 
		vA = contacts[i + 2*deviceContactPitch];
		vect_mult_inc(vB, vA, -a.z); 
		vB.w = 0;
		vA = bodies[B1_index+ 4*deviceBodyPitch];
		vA.x = vA.w;
		vA.y = vA.w;
		vA.z = vA.w;
		vA.w=0;
		vect_inertia_multiply(vB,vA);
		vA = contacts[i + 10*deviceContactPitch];		// R1  R2  n1  n2
		reduction[(int)vA.x] = vB;										//  ---> store  dv1  

		/// ---- compute dw1 =  Inert.1' * J1w^ * deltagamma
		vA = contacts[i + 3*deviceContactPitch];
		vect_mult    (vB, vA, a.x); 
		vA = contacts[i + 4*deviceContactPitch];
		vect_mult_inc(vB, vA, a.y); 
		vA = contacts[i + 5*deviceContactPitch];
		vect_mult_inc(vB, vA, a.z); 
		vB.w = 0;
		vA = bodies[B1_index+ 4*deviceBodyPitch]; // J J J m
		vect_inertia_multiply(vB,vA);
		vA = contacts[i + 10*deviceContactPitch];		// R1  R2  n1  n2
		vB.w = vA.z; // the n1 repetition counter 
		reduction[(int)vA.x  + deviceReductionPitch] = vB;				//  ---> store  dw1  
	}

	if (B2_index >= 0)
	{
		/// ---- compute dv2 
		vA = contacts[i + 0*deviceContactPitch];
		vect_mult    (vB, vA, a.x); 
		vA = contacts[i + 1*deviceContactPitch];
		vect_mult_inc(vB, vA, a.y); 
		vA = contacts[i + 2*deviceContactPitch];
		vect_mult_inc(vB, vA, a.z); 
		vB.w = 0;
		vA = bodies[B2_index+ 4*deviceBodyPitch];
		vA.x = vA.w;
		vA.y = vA.w;
		vA.z = vA.w;
		vA.w=0;
		vect_inertia_multiply(vB,vA);
		vA = contacts[i + 10*deviceContactPitch];		// R1  R2  n1  n2
		reduction[(int)vA.y] = vB;										//  ---> store  dv2  

		/// ---- compute dw2 
		vA = contacts[i + 6*deviceContactPitch];
		vect_mult    (vB, vA, a.x); 
		vA = contacts[i + 7*deviceContactPitch];
		vect_mult_inc(vB, vA, a.y); 
		vA = contacts[i + 8*deviceContactPitch];
		vect_mult_inc(vB, vA, a.z); 
		vB.w = 0;
		vA = bodies[B2_index+ 4*deviceBodyPitch];
		vA.w=0;
		vect_inertia_multiply(vB,vA);
		vA = contacts[i + 10*deviceContactPitch];		// R1  R2  n1  n2
		vB.w = vA.w; // the n2 repetition counter 
		reduction[(int)vA.y  + deviceReductionPitch] = vB;				//  ---> store  dw2
	}
} 


extern "C"
void ChRunKernelLCPiteration( dim3 dim_grid, 
							  dim3 dim_threads, 
							  CH_REALNUMBER4* d_buffer_contacts, 
							  CH_REALNUMBER4* d_buffer_bodies,
							  CH_REALNUMBER4* d_buffer_reduction,
							  unsigned int contacts_data_pitch,
							  unsigned int bodies_data_pitch,
							  unsigned int reduction_data_pitch,
							  CH_REALNUMBER mLcpOmega) 
{
	//body_pitch should end up in constant memory
	CUDA_SAFE_CALL(cudaMemcpyToSymbol(deviceBodyPitch,&bodies_data_pitch,sizeof(bodies_data_pitch)));
	//contact pitch should end up in constant memory
	CUDA_SAFE_CALL(cudaMemcpyToSymbol(deviceContactPitch,&contacts_data_pitch,sizeof(contacts_data_pitch)));
	//reduction pitch should end up in constant memory
	CUDA_SAFE_CALL(cudaMemcpyToSymbol(deviceReductionPitch, &reduction_data_pitch,sizeof(reduction_data_pitch)));
	//omega should end up in constant memory
	CUDA_SAFE_CALL(cudaMemcpyToSymbol(deviceLcpOmega,&mLcpOmega,sizeof(mLcpOmega)));

    // execute the kernel
	ChKernelLCPiteration<<< dim_grid, dim_threads >>>(d_buffer_contacts, d_buffer_bodies, d_buffer_reduction);
}








///////////////////////////////////////////////////////////////////////////////////



// Kernel for a single iteration of the LCP over all scalar bilateral contacts
// (a bit similar to the ChKernelLCPiteration above, but without projection etc.)
//   Version 2.0 - Tasora
//



__global__ void  
ChKernelLCPiterationBilaterals( CH_REALNUMBER4* bilaterals, CH_REALNUMBER4* bodies, CH_REALNUMBER4* reduction) 
{ 
	// Compute the i,bilateralContactPitch values used to access data inside the large 'bilaterals'
	// array using pointer arithmetic.
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
 
	// In all phases of the following computations, try to use only those following
	// vars, hoping this will help the compiler to minimize register requirements..
	CH_REALNUMBER4 vA; 
	CH_REALNUMBER4 vB; 
	CH_REALNUMBER4 vR; 
	CH_REALNUMBER a; 
	int B1_index;
	int B2_index;
	CH_REALNUMBER gamma;

	B1_index = bilaterals[i].w ; //__float_as_int(vA.w);
	B2_index = bilaterals[i+ deviceBilateralPitch].w ; //__float_as_int(vA.w);

	// ---- perform   a = ([J1 J2] {v1 | v2}^ + b) 
 
	a=0;

	if (B1_index >=0)
	{				
		vA = bilaterals[i];						// line 0
		vB = bodies[B1_index];	// v1
		a += dot_three ( vA , vB);

		vA = bilaterals[i+2*deviceBilateralPitch]; // line 2
		vB = bodies[B1_index + deviceBodyPitch]; // w1
		a += dot_three ( vA , vB);
	}

	if (B2_index >=0)
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
	vR= bilaterals[i + 5*deviceBilateralPitch];				// R1  R2  n1  n2

	/// ---- compute dv1 =  invInert.1 * J1^ * deltagamma  
	if (B1_index >=0)
	{	
		vB = bodies[B1_index + 4*deviceBodyPitch];	// iJ iJ iJ im

		vA = bilaterals[i]; 						// line 0: J1(x)
		vA.x *= vB.w;
		vA.y *= vB.w;
		vA.z *= vB.w;
		vA *= a;
		vA.w  = 0;
		reduction[((int)vR.x)] = vA;							//  ---> store  v1 vel. in reduction buffer	

		vA = bilaterals[i+2*deviceBilateralPitch];	// line 2:  J1(w)		  
		vA.x *= vB.x;
		vA.y *= vB.y;
		vA.z *= vB.z;
		vA *= a;
		vA.w = vR.z;	// set the n1 repetition counter 
		reduction[((int)vR.x) + deviceReductionPitch] = vA;	// ---> store  w1 vel. in reduction buffer
	}
	if (B2_index >=0)
	{	
		vB = bodies[B2_index + 4*deviceBodyPitch];	// iJ iJ iJ im

		vA = bilaterals[i+deviceBilateralPitch]; 	// line 1: J2(x)
		vA.x *= vB.w;
		vA.y *= vB.w;
		vA.z *= vB.w;
		vA *= a;
		vA.w  = 0;
		reduction[((int)vR.y)] = vA;							//  ---> store  v2 vel. in reduction buffer	

		vA = bilaterals[i+3*deviceBilateralPitch];	// line 3:  J2(w)		  
		vA.x *= vB.x;
		vA.y *= vB.y;
		vA.z *= vB.z;
		vA *= a;
		vA.w = vR.w;	// set the n2 repetition counter 
		reduction[((int)vR.y) + deviceReductionPitch] = vA;	// ---> store  w2 vel. in reduction buffer
	}
} 




extern "C"
void ChRunKernelLCPiterationBilateral( dim3 dim_grid, 
							  dim3 dim_threads, 
							  CH_REALNUMBER4* d_buffer_bilaterals, 
							  CH_REALNUMBER4* d_buffer_bodies,
							  CH_REALNUMBER4* d_buffer_reduction,
							  unsigned int bilaterals_data_pitch,
							  unsigned int bodies_data_pitch,
							  unsigned int reduction_data_pitch,
							  CH_REALNUMBER mLcpOmega) 
{
	//body_pitch should end up in constant memory
	CUDA_SAFE_CALL(cudaMemcpyToSymbol(deviceBodyPitch,&bodies_data_pitch,sizeof(bodies_data_pitch)));
	//bilaterals pitch should end up in constant memory
	CUDA_SAFE_CALL(cudaMemcpyToSymbol(deviceBilateralPitch,&bilaterals_data_pitch,sizeof(bilaterals_data_pitch)));
	//reduction pitch should end up in constant memory
	CUDA_SAFE_CALL(cudaMemcpyToSymbol(deviceReductionPitch, &reduction_data_pitch,sizeof(reduction_data_pitch)));
	//omega should end up in constant memory
	CUDA_SAFE_CALL(cudaMemcpyToSymbol(deviceLcpOmega,&mLcpOmega,sizeof(mLcpOmega)));

    // execute the kernel
	ChKernelLCPiterationBilaterals<<< dim_grid, dim_threads >>>(d_buffer_bilaterals, d_buffer_bodies, d_buffer_reduction);
}






////////////////////////////////////////////////////////////////////////////////////////////////

//
// Kernel for adding invmass*force*stepsize to body speed vector.
// This kernel must be applied to the stream of the body buffer.
//

__constant__ CH_REALNUMBER forceFactor;  // usually, the step size
 
__global__ void  
ChKernelLCPaddForces(CH_REALNUMBER4* bodies) 
{ 
	// Compute the i values used to access data inside the large 
	// array using pointer arithmetic.
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
 
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

extern "C"
void ChRunKernelLCPaddForces( dim3 dim_grid, 
							  dim3 dim_threads, 
							  CH_REALNUMBER4* d_buffer_bodies,
							  unsigned int bodies_data_pitch,
							  CH_REALNUMBER mforceFactor) 
{
	//body_pitch should end up in constant memory
	CUDA_SAFE_CALL(cudaMemcpyToSymbol(deviceBodyPitch,&bodies_data_pitch,sizeof(bodies_data_pitch)));
	//stepsize should end up in constant memory
	CUDA_SAFE_CALL(cudaMemcpyToSymbol(forceFactor,&mforceFactor,sizeof(mforceFactor)));

    // execute the kernel
	ChKernelLCPaddForces<<< dim_grid, dim_threads >>>(d_buffer_bodies);
}


 


////////////////////////////////////////////////////////////////////////////////////////////////


//variables already declared in device constant memory  
        
__constant__ CH_REALNUMBER maxRecoverySpeedNeg;
__constant__ CH_REALNUMBER Cfactor;				// usually 1/dt

// Kernel for preprocessing the contact information for the CCP. 
//   Version 1.1 - Negrut
//
//  This kernel expects to find the data arranged as float4 in a horizontal
//  buffer called 'deviceContacts'.  The computation is done in place, that is, the
//  'deviceContacts' buffer is populated with extra entries while some of the old 
//  ones get overwritten.  Take a look at the MS-Word doc "dataFormatGPU.doc" in the 
//  folder "docs" to see the content of the deviceContacts buffer upon entry
//  and exit from this kernel.


__global__ void ChKernelContactsPreprocess__OLD__(CH_REALNUMBER4* deviceContacts, CH_REALNUMBER4* deviceBodies) 
{
   //shared memory allocated dynamically; the way the computation is handled, there is
   //one block per multiprocessor.  Since the number of threads per block right
   //now is 512 I can statically allocate the amount of shared memory at the onset of 
   //computation
   __shared__  float shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*CH_PREPROCESSING_THREADS_PER_BLOCK];

   register CH_REALNUMBER4 reg03;
   register CH_REALNUMBER  reg4;
   register CH_REALNUMBER  reg5;
   register CH_REALNUMBER4 reg69;
   register CH_REALNUMBER3 regS;
   register CH_REALNUMBER  eta = 0.;

   register unsigned int memAddress = blockDim.x*blockIdx.x + threadIdx.x;
   register unsigned int reg_uInt;

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
   reg03.x /= reg4; shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x  ] = reg03.x;
   reg03.y /= reg4; shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+1] = reg03.y;
   reg03.z /= reg4; shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+2] = reg03.z;

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
      shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+3] =  reg03.y;
      shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+4] = -reg03.x;
      shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+5] =      0.0;
   }

   //store stuff in shared memory
   if( reg_uInt==0 ){
      //it turns out that X axis is closest to being perpendicular to contact vector;
      //store values temporarily into the shared memory
      shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+3] =      0.0;
      shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+4] =  reg03.z;
      shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+5] = -reg03.y;
   }
   else if( reg_uInt==1 ){
      //it turns out that Y axis is closest to being perpendicular to contact vector;
      //store values temporarily into the shared memory
      shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+3] = -reg03.z;
      shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+4] =      0.0;
      shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+5] =  reg03.x;
   }

   //normalized the local contact Y axis (therefore automatically the 
   //local contact Z axis will be normalized and you need do nothing)
   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+3];
   reg4 *= reg4;
   reg5  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+4];
   reg5 *= reg5;
   reg4 += reg5;
   reg5  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+5];
   reg5 *= reg5;
   reg4 += reg5;
   reg4 = sqrt(reg4);
   shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+3] /= reg4;
   shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+4] /= reg4;
   shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+5] /= reg4;

   //now carry out the last cross product to find out the contact local Z axis;
   //to this end multiply the contact normal by the local Y component
   reg4 = reg03.y*shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+5];
   reg5 = reg03.z*shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+4];
   shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+6] = reg4 - reg5;

   reg4 = reg03.z*shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+3];
   reg5 = reg03.x*shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+5];
   shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+7] = reg4 - reg5;

   reg4 = reg03.x*shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+4];
   reg5 = reg03.y*shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+3];
   shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+8] = reg4 - reg5;

   // The gap distance is the dot product <(s2-s1), contact_normal>, since contact_normal is normalized
   reg03 = deviceContacts[memAddress+6*deviceContactPitch]; //fetches s_2,w
   reg69 = deviceContacts[memAddress+3*deviceContactPitch]; //fetches s_1,w
   reg03 -= reg69;
   reg03.w  = reg03.x * shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x  ]; 
   reg03.w += reg03.y * shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+1]; 
   reg03.w += reg03.z * shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+2]; 
   reg03.w *= Cfactor;	

   // Clamp Anitescu stabilization coefficient: 
   if (reg03.w < maxRecoverySpeedNeg)		// ***ALEX*** should be this...
   		reg03.w = maxRecoverySpeedNeg;	// ***ALEX*** should be this...
 
   //ok, a first batch of things is now computed, copy stuff back to global memory;
   //what gets passed back is --> A_c^T <--  
   //copy first column of A_c in reg03.x, reg03.y, reg03.z; note that reg03.w hols on to the gap
   reg03.x = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x  ]; 
   reg03.y = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+1]; 
   reg03.z = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+2]; 
   deviceContacts[memAddress] = reg03;   //reg03 contains the components of the contact normal, pointing towards the exterior of what is now considered "body 1"
   memAddress += deviceContactPitch; 

   //second column of A_c
   reg03.x = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+3];
   reg03.y = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+4];
   reg03.z = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+5];
   reg03.w = 0.;  
   deviceContacts[memAddress] = reg03;   //reg03 contains the components of the contact Y axis
   memAddress += deviceContactPitch; 
   
   //third column of A_c
   reg03.x = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+6];
   reg03.y = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+7];
   reg03.z = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+8];
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
   reg_uInt = (unsigned int) reg03.w;
   reg69 = deviceBodies[2*deviceBodyPitch+reg_uInt]; 
   regS.x -= reg69.x;                                
   regS.y -= reg69.y;                                
   regS.z -= reg69.z;                                

   //bring in the inertia attributes; store in shared memory; to be used to compute \eta
   reg69 = deviceBodies[4*deviceBodyPitch+reg_uInt];
   shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+9 ] = reg69.x;  // this is the inverse of I_x  
   shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+10] = reg69.y;  // this is the inverse of I_y
   shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+11] = reg69.z;  // this is the inverse of I_z
   shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+12] = reg69.w;  // this is the inverse of mass

   //bring in the Euler parameters associated with body 1; note that after this, reg03.w is not needed anymore, hang on to reguInt though
   reg69 = deviceBodies[3*deviceBodyPitch+reg_uInt];

   //start computing A_c^T \tilde s E G^T.  Start by getting the first row of A_c^T \tilde s:
   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+1]*regS.z;
   reg4 -= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+2]*regS.y;

   reg5  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+2]*regS.x;
   reg5 -= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x  ]*regS.z;

   reg03.w  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x  ]*regS.y;  //<-- NOTE: reg03.w used to hold on to index for body 1, not needed anymore...
   reg03.w -= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+1]*regS.x;

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
   reg03.x *= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+ 9];
   eta     += reg03.x;

   reg03.y *= reg03.y;
   reg03.y *= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+10];
   eta     += reg03.y;

   reg03.z *= reg03.z;
   reg03.z *= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+11];
   eta     += reg03.z;

   //work now on the second row of the product A_c^T \tilde s E G^T; 
   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+4]*regS.z;
   reg4 -= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+5]*regS.y;

   reg5  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+5]*regS.x;
   reg5 -= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+3]*regS.z;

   reg03.w  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+3]*regS.y;  //<-- NOTE: reg03.w used to hold on to index for body 1, not needed anymore...
   reg03.w -= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+4]*regS.x;

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
   reg03.x *= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+ 9];
   eta     += reg03.x;

   reg03.y *= reg03.y;
   reg03.y *= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+10];
   eta     += reg03.y;

   reg03.z *= reg03.z;
   reg03.z *= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+11];
   eta     += reg03.z;


   //work now on the third row of the product A_c^T \tilde s E G^T; 
   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+7]*regS.z;
   reg4 -= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+8]*regS.y;

   reg5  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+8]*regS.x;
   reg5 -= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+6]*regS.z;

   reg03.w  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+6]*regS.y;  //<-- NOTE: reg03.w used to hold on to index for body 1, not needed anymore...
   reg03.w -= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+7]*regS.x;

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
   reg03.x *= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+ 9];
   eta     += reg03.x;

   reg03.y *= reg03.y;
   reg03.y *= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+10];
   eta     += reg03.y;

   reg03.z *= reg03.z;
   reg03.z *= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+11];
   eta     += reg03.z;

   //add to eta the contribution that comes out of the mass and matrix A_c.
   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x  ];
   reg4 *= reg4;
   reg5  = reg4;

   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+1];
   reg4 *= reg4;
   reg5 += reg4;

   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+2];
   reg4 *= reg4;
   reg5 += reg4;

   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+3];
   reg4 *= reg4;
   reg5 += reg4;

   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+4];
   reg4 *= reg4;
   reg5 += reg4;

   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+5];
   reg4 *= reg4;
   reg5 += reg4;

   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+6];
   reg4 *= reg4;
   reg5 += reg4;

   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+7];
   reg4 *= reg4;
   reg5 += reg4;

   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+8];
   reg4 *= reg4;
   reg5 += reg4;
   
   //take care of what needs to be done for eta
   reg5 *= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+12];  //multiply by inverse of mass matrix of B1
   eta += reg5;


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
   reg_uInt = (unsigned int) reg03.w;
   reg69 = deviceBodies[2*deviceBodyPitch+reg_uInt]; 
   regS.x -= reg69.x;                                
   regS.y -= reg69.y;                                
   regS.z -= reg69.z;                                

   //bring in the inertia attributes; store in shared memory; to be used to compute \eta
   reg69 = deviceBodies[4*deviceBodyPitch+reg_uInt];
   shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+9 ] = reg69.x;  // this is the inverse of I_x  
   shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+10] = reg69.y;  // this is the inverse of I_y
   shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+11] = reg69.z;  // this is the inverse of I_z
   shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+12] = reg69.w;  // this is the inverse of mass

   //bring in the Euler parameters associated with body 2; note that after this, reg03.w is not needed anymore, hang on to reguInt though
   reg69 = deviceBodies[3*deviceBodyPitch+reg_uInt];

   //start computing A_c^T \tilde s E G^T.  Start by getting the first row of A_c^T \tilde s:
   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+1]*regS.z;
   reg4 -= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+2]*regS.y;

   reg5  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+2]*regS.x;
   reg5 -= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x  ]*regS.z;

   reg03.w  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x  ]*regS.y;  //<-- NOTE: reg03.w used to hold on to index for body 1, not needed anymore...
   reg03.w -= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+1]*regS.x;

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
   reg03.x *= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+ 9];
   eta     += reg03.x;

   reg03.y *= reg03.y;
   reg03.y *= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+10];
   eta     += reg03.y;

   reg03.z *= reg03.z;
   reg03.z *= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+11];
   eta     += reg03.z;

   //work now on the second row of the product A_c^T \tilde s E G^T; 
   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+4]*regS.z;
   reg4 -= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+5]*regS.y;

   reg5  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+5]*regS.x;
   reg5 -= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+3]*regS.z;

   reg03.w  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+3]*regS.y;  //<-- NOTE: reg03.w used to hold on to index for body 1, not needed anymore...
   reg03.w -= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+4]*regS.x;

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
   reg03.x *= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+ 9];
   eta     += reg03.x;

   reg03.y *= reg03.y;
   reg03.y *= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+10];
   eta     += reg03.y;

   reg03.z *= reg03.z;
   reg03.z *= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+11];
   eta     += reg03.z;

   //work now on the third row of the product A_c^T \tilde s E G^T; 
   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+7]*regS.z;
   reg4 -= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+8]*regS.y;

   reg5  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+8]*regS.x;
   reg5 -= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+6]*regS.z;

   reg03.w  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+6]*regS.y;  //<-- NOTE: reg03.w used to hold on to index for body 1, not needed anymore...
   reg03.w -= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+7]*regS.x;

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
   reg4 *= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+ 9];
   eta     += reg4;

   reg4  = reg03.y;
   reg4 *= reg4;
   reg4 *= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+10];
   eta     += reg4;

   reg4  = reg03.z;
   reg4 *= reg4;
   reg4 *= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+11];
   eta     += reg4;

   //add to eta the contribution that comes out of the mass and matrix A_c.
   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x  ];
   reg4 *= reg4;
   reg5  = reg4;

   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+1];
   reg4 *= reg4;
   reg5 += reg4;

   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+2];
   reg4 *= reg4;
   reg5 += reg4;

   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+3];
   reg4 *= reg4;
   reg5 += reg4;

   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+4];
   reg4 *= reg4;
   reg5 += reg4;

   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+5];
   reg4 *= reg4;
   reg5 += reg4;

   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+6];
   reg4 *= reg4;
   reg5 += reg4;

   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+7];
   reg4 *= reg4;
   reg5 += reg4;

   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+8];
   reg4 *= reg4;
   reg5 += reg4;
   
   //take care of what needs to be done for eta
   reg5 *= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+12];  //multiply by inverse of mass matrix of B1
   eta += reg5;

   eta = 3./eta;  // <-- final value of eta
   reg03.w = eta; // <-- value of eta is passed back to global
   deviceContacts[memAddress] = reg03;
} 


__global__ void ChKernelContactsPreprocess(CH_REALNUMBER4* deviceContacts, CH_REALNUMBER4* deviceBodies) 
{
   //shared memory allocated dynamically; the way the computation is handled, there is
   //one block per multiprocessor.  Since the number of threads per block right
   //now is 512 I can statically allocate the amount of shared memory at the onset of 
   //computation
   __shared__  float shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*CH_PREPROCESSING_THREADS_PER_BLOCK];

   register CH_REALNUMBER4 reg03;
   register CH_REALNUMBER  reg4;
   register CH_REALNUMBER  reg5;
   register CH_REALNUMBER4 reg69;
   register CH_REALNUMBER3 regS;
   register CH_REALNUMBER  eta = 0.;

   register unsigned int memAddress = blockDim.x*blockIdx.x + threadIdx.x;
   register  int reg_uInt;

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
   reg03.x /= reg4; shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x  ] = reg03.x;
   reg03.y /= reg4; shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+1] = reg03.y;
   reg03.z /= reg4; shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+2] = reg03.z;

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
      shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+3] =  reg03.y;
      shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+4] = -reg03.x;
      shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+5] =      0.0;
   }

   //store stuff in shared memory
   if( reg_uInt==0 ){
      //it turns out that X axis is closest to being perpendicular to contact vector;
      //store values temporarily into the shared memory
      shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+3] =      0.0;
      shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+4] =  reg03.z;
      shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+5] = -reg03.y;
   }
   else if( reg_uInt==1 ){
      //it turns out that Y axis is closest to being perpendicular to contact vector;
      //store values temporarily into the shared memory
      shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+3] = -reg03.z;
      shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+4] =      0.0;
      shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+5] =  reg03.x;
   }

   //normalized the local contact Y axis (therefore automatically the 
   //local contact Z axis will be normalized and you need do nothing)
   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+3];
   reg4 *= reg4;
   reg5  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+4];
   reg5 *= reg5;
   reg4 += reg5;
   reg5  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+5];
   reg5 *= reg5;
   reg4 += reg5;
   reg4 = sqrt(reg4);
   shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+3] /= reg4;
   shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+4] /= reg4;
   shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+5] /= reg4;

   //now carry out the last cross product to find out the contact local Z axis;
   //to this end multiply the contact normal by the local Y component
   reg4 = reg03.y*shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+5];
   reg5 = reg03.z*shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+4];
   shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+6] = reg4 - reg5;

   reg4 = reg03.z*shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+3];
   reg5 = reg03.x*shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+5];
   shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+7] = reg4 - reg5;

   reg4 = reg03.x*shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+4];
   reg5 = reg03.y*shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+3];
   shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+8] = reg4 - reg5;

   // The gap distance is the dot product <(s2-s1), contact_normal>, since contact_normal is normalized
   reg03 = deviceContacts[memAddress+6*deviceContactPitch]; //fetches s_2,w
   reg69 = deviceContacts[memAddress+3*deviceContactPitch]; //fetches s_1,w
   reg03 -= reg69;
   reg03.w  = reg03.x * shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x  ]; 
   reg03.w += reg03.y * shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+1]; 
   reg03.w += reg03.z * shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+2]; 
   reg03.w *= Cfactor;	

   // Clamp Anitescu stabilization coefficient: 

   if (reg03.w < maxRecoverySpeedNeg)		
   		reg03.w = maxRecoverySpeedNeg;	


   //ok, a first batch of things is now computed, copy stuff back to global memory;
   //what gets passed back is --> A_c^T <--  
   //copy first column of A_c in reg03.x, reg03.y, reg03.z; note that reg03.w hols on to the gap
   reg03.x = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x  ]; 
   reg03.y = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+1]; 
   reg03.z = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+2]; 
   deviceContacts[memAddress] = reg03;   //reg03 contains the components of the contact normal, pointing towards the exterior of what is now considered "body 1"
   memAddress += deviceContactPitch; 

   //second column of A_c
   reg03.x = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+3];
   reg03.y = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+4];
   reg03.z = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+5];
   reg03.w = 0.;  
   deviceContacts[memAddress] = reg03;   //reg03 contains the components of the contact Y axis
   memAddress += deviceContactPitch; 
   
   //third column of A_c
   reg03.x = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+6];
   reg03.y = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+7];
   reg03.z = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+8];
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
	   shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+9 ] = reg69.x;  // this is the inverse of I_x  
	   shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+10] = reg69.y;  // this is the inverse of I_y
	   shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+11] = reg69.z;  // this is the inverse of I_z
	   shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+12] = reg69.w;  // this is the inverse of mass

	   //bring in the Euler parameters associated with body 1; note that after this, reg03.w is not needed anymore, hang on to reguInt though
	   reg69 = deviceBodies[3*deviceBodyPitch+reg_uInt];

	   //start computing A_c^T \tilde s E G^T.  Start by getting the first row of A_c^T \tilde s:
	   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+1]*regS.z;
	   reg4 -= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+2]*regS.y;

	   reg5  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+2]*regS.x;
	   reg5 -= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x  ]*regS.z;

	   reg03.w  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x  ]*regS.y;  //<-- NOTE: reg03.w used to hold on to index for body 1, not needed anymore...
	   reg03.w -= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+1]*regS.x;

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
	   reg03.x *= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+ 9];
	   eta     += reg03.x;

	   reg03.y *= reg03.y;
	   reg03.y *= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+10];
	   eta     += reg03.y;

	   reg03.z *= reg03.z;
	   reg03.z *= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+11];
	   eta     += reg03.z;

	   //work now on the second row of the product A_c^T \tilde s E G^T; 
	   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+4]*regS.z;
	   reg4 -= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+5]*regS.y;

	   reg5  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+5]*regS.x;
	   reg5 -= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+3]*regS.z;

	   reg03.w  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+3]*regS.y;  //<-- NOTE: reg03.w used to hold on to index for body 1, not needed anymore...
	   reg03.w -= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+4]*regS.x;

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
	   reg03.x *= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+ 9];
	   eta     += reg03.x;

	   reg03.y *= reg03.y;
	   reg03.y *= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+10];
	   eta     += reg03.y;

	   reg03.z *= reg03.z;
	   reg03.z *= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+11];
	   eta     += reg03.z;


	   //work now on the third row of the product A_c^T \tilde s E G^T; 
	   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+7]*regS.z;
	   reg4 -= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+8]*regS.y;

	   reg5  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+8]*regS.x;
	   reg5 -= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+6]*regS.z;

	   reg03.w  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+6]*regS.y;  //<-- NOTE: reg03.w used to hold on to index for body 1, not needed anymore...
	   reg03.w -= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+7]*regS.x;

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
	   reg03.x *= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+ 9];
	   eta     += reg03.x;

	   reg03.y *= reg03.y;
	   reg03.y *= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+10];
	   eta     += reg03.y;

	   reg03.z *= reg03.z;
	   reg03.z *= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+11];
	   eta     += reg03.z;

	   //add to eta the contribution that comes out of the mass and matrix A_c.
	   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x  ];
	   reg4 *= reg4;
	   reg5  = reg4;

	   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+1];
	   reg4 *= reg4;
	   reg5 += reg4;

	   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+2];
	   reg4 *= reg4;
	   reg5 += reg4;

	   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+3];
	   reg4 *= reg4;
	   reg5 += reg4;

	   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+4];
	   reg4 *= reg4;
	   reg5 += reg4;

	   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+5];
	   reg4 *= reg4;
	   reg5 += reg4;

	   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+6];
	   reg4 *= reg4;
	   reg5 += reg4;

	   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+7];
	   reg4 *= reg4;
	   reg5 += reg4;

	   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+8];
	   reg4 *= reg4;
	   reg5 += reg4;
	   
	   //take care of what needs to be done for eta
	   reg5 *= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+12];  //multiply by inverse of mass matrix of B1
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
	   shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+9 ] = reg69.x;  // this is the inverse of I_x  
	   shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+10] = reg69.y;  // this is the inverse of I_y
	   shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+11] = reg69.z;  // this is the inverse of I_z
	   shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+12] = reg69.w;  // this is the inverse of mass

	   //bring in the Euler parameters associated with body 2; note that after this, reg03.w is not needed anymore, hang on to reguInt though
	   reg69 = deviceBodies[3*deviceBodyPitch+reg_uInt];

	   //start computing A_c^T \tilde s E G^T.  Start by getting the first row of A_c^T \tilde s:
	   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+1]*regS.z;
	   reg4 -= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+2]*regS.y;

	   reg5  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+2]*regS.x;
	   reg5 -= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x  ]*regS.z;

	   reg03.w  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x  ]*regS.y;  //<-- NOTE: reg03.w used to hold on to index for body 1, not needed anymore...
	   reg03.w -= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+1]*regS.x;

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
	   reg03.x *= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+ 9];
	   eta     += reg03.x;

	   reg03.y *= reg03.y;
	   reg03.y *= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+10];
	   eta     += reg03.y;

	   reg03.z *= reg03.z;
	   reg03.z *= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+11];
	   eta     += reg03.z;

	   //work now on the second row of the product A_c^T \tilde s E G^T; 
	   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+4]*regS.z;
	   reg4 -= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+5]*regS.y;

	   reg5  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+5]*regS.x;
	   reg5 -= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+3]*regS.z;

	   reg03.w  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+3]*regS.y;  //<-- NOTE: reg03.w used to hold on to index for body 1, not needed anymore...
	   reg03.w -= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+4]*regS.x;

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
	   reg03.x *= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+ 9];
	   eta     += reg03.x;

	   reg03.y *= reg03.y;
	   reg03.y *= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+10];
	   eta     += reg03.y;

	   reg03.z *= reg03.z;
	   reg03.z *= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+11];
	   eta     += reg03.z;

	   //work now on the third row of the product A_c^T \tilde s E G^T; 
	   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+7]*regS.z;
	   reg4 -= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+8]*regS.y;

	   reg5  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+8]*regS.x;
	   reg5 -= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+6]*regS.z;

	   reg03.w  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+6]*regS.y;  //<-- NOTE: reg03.w used to hold on to index for body 1, not needed anymore...
	   reg03.w -= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+7]*regS.x;

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
	   reg4 *= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+ 9];
	   eta     += reg4;

	   reg4  = reg03.y;
	   reg4 *= reg4;
	   reg4 *= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+10];
	   eta     += reg4;

	   reg4  = reg03.z;
	   reg4 *= reg4;
	   reg4 *= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+11];
	   eta     += reg4; 

	   //add to eta the contribution that comes out of the mass and matrix A_c.
	   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x  ];
	   reg4 *= reg4;
	   reg5  = reg4;

	   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+1];
	   reg4 *= reg4;
	   reg5 += reg4;

	   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+2];
	   reg4 *= reg4;
	   reg5 += reg4;

	   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+3];
	   reg4 *= reg4;
	   reg5 += reg4;

	   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+4];
	   reg4 *= reg4;
	   reg5 += reg4;

	   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+5];
	   reg4 *= reg4;
	   reg5 += reg4;

	   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+6];
	   reg4 *= reg4;
	   reg5 += reg4;

	   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+7];
	   reg4 *= reg4;
	   reg5 += reg4;

	   reg4  = shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+8];
	   reg4 *= reg4;
	   reg5 += reg4;
	   
	   //take care of what needs to be done for eta
	   reg5 *= shMemData[CH_PREPROCESSING_SH_MEM_BLOCK_SIZE*threadIdx.x+12];  //multiply by inverse of mass matrix of B1
	   eta += reg5;
	}

   eta = 3./eta;  // <-- final value of eta
   reg03.w = eta; // <-- value of eta is passed back to global
   deviceContacts[( blockIdx.x* blockDim.x + threadIdx.x) + (8*deviceContactPitch)] = reg03;
} 



extern "C"
void ChRunKernelContactsPreprocess(dim3 dim_grid, 
								   dim3 dim_threads,
								   CH_REALNUMBER4*  d_buffer_contacts, 
								   CH_REALNUMBER4*  d_buffer_bodies, 
								   unsigned int contacts_data_pitch,
								   unsigned int bodies_data_pitch,
								   CH_REALNUMBER mCfactor,
								   CH_REALNUMBER max_recovery_speed)
{
	//body_pitch should end up in constant memory
	CUDA_SAFE_CALL(cudaMemcpyToSymbol(deviceBodyPitch,&bodies_data_pitch,sizeof(bodies_data_pitch)));
	//contact pitch should end up in constant memory
	CUDA_SAFE_CALL(cudaMemcpyToSymbol(deviceContactPitch,&contacts_data_pitch,sizeof(contacts_data_pitch)));
	//stepsize should end up in constant memory
	CUDA_SAFE_CALL(cudaMemcpyToSymbol(Cfactor,&mCfactor,sizeof(mCfactor)));
	//clamp of recovery speed should end up in constant memory
	CH_REALNUMBER negated_recspeed = -max_recovery_speed;
	CUDA_SAFE_CALL(cudaMemcpyToSymbol(maxRecoverySpeedNeg,&negated_recspeed,sizeof(negated_recspeed)));

	// Execute the kernel
	ChKernelContactsPreprocess<<< dim_grid, dim_threads >>>(d_buffer_contacts, d_buffer_bodies);
}




////////////////////////////////////////////////////////////////////////////////////////////////////



///  The reduction algorithm iterates few times a reduction kernel (sequences sum) over repeated
///  data of the type  aaaaabbbcccccccddeeeeffgggggg, with varying numbers of repetitions.
///  The data to be reduced is described few lines below. This algorithm works if the data is
///  prepared with an increasing index 'repetition_counter' for each  aaaa-bbb-... subsequence.
///  The data must be already sorted for unique groups, i.e. aaaabbbcc or bbbccaaaa are ok, but
///  aabbbccaa is not good.
///                 -- number of registers: 15
  


  
__global__ void 
ChKernelLCPReduction(CH_REALNUMBER4* d_buffer_reduction, CH_REALNUMBER4* d_buffer_bodies, int treepower) 
{
	const unsigned int i =   blockIdx.x*blockDim.x + threadIdx.x;
	
	volatile CH_REALNUMBER4 row1 = d_buffer_reduction[i];
	volatile CH_REALNUMBER4 row2 = d_buffer_reduction[i+deviceReductionPitch];

	// row2.w is repetition counter: will gather if >= 8,4,2,1. Do nothing if =0.
	if (row2.w >= treepower) 
	{
		d_buffer_reduction[i+deviceReductionPitch].w= 0; // mark as processed
		// new coalescing-friendly code..
		CH_REALNUMBER4 to_row1 = d_buffer_reduction[i-treepower];  
		CH_REALNUMBER4 to_row2 = d_buffer_reduction[i+deviceReductionPitch - treepower]; 
		to_row1.x += row1.x;
		to_row1.y += row1.y;
		to_row1.z += row1.z;
		to_row2.x += row2.x;
		to_row2.y += row2.y;
		to_row2.z += row2.z;
		d_buffer_reduction[i-treepower] = to_row1;
		d_buffer_reduction[i+deviceReductionPitch - treepower] = to_row2;
	}
}



extern "C"
void ChRunKernelLCPreduction(			dim3 dim_reduction_grid, 
										dim3 dim_reduction_threads, 
										CH_REALNUMBER4* d_buffer_reduction, 
										CH_REALNUMBER4* d_buffer_bodies,
										unsigned int reduction_data_pitch,
										unsigned int bodies_data_pitch,
										int max_repetitions)
{
	//reduction pitch should end up in constant memory
	CUDA_SAFE_CALL(cudaMemcpyToSymbol(deviceReductionPitch, &reduction_data_pitch,sizeof(reduction_data_pitch)));

	if (max_repetitions > 0)
	{
		int kernelsteps =(int)ceil (log2((double)max_repetitions)); // max_repetitions assumed =1 as minimum (all entries are singletons)
		for (int nstep= kernelsteps-1; nstep >=0; nstep--)
		{
			int treepower = 0x01 << nstep;   // treepower=.....8,4,2,1
			ChKernelLCPReduction<<< dim_reduction_grid, dim_reduction_threads>>>(d_buffer_reduction, d_buffer_bodies, treepower);
		}
	}

}





////////////////////////////////////////////////////////////////////////////////////////////////////




// Updates the speeds in the body buffer with values accumulated in the 
// reduction buffer:   V_new = V_old + delta_speeds


__global__ void  
ChKernelLCPspeedupdate(CH_REALNUMBER4* bodies, CH_REALNUMBER4* reduction) 
{ 
	// Compute the i,devicePitch values used to access data inside the large 'contact'
	// array using pointer arithmetic.
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;

	// Fetch deltaspeed from reduction buffer

	int Rb = bodies[i].w;  // index of slot in reduction buffer

	if (Rb != -1)
	{
		bodies[i].x += reduction[Rb].x;	  // V_new = V_old + sum_of_all_delta_speeds
		bodies[i].y += reduction[Rb].y;
		bodies[i].z += reduction[Rb].z;
		bodies[i +deviceBodyPitch] += reduction[Rb+deviceReductionPitch];
	}
}


extern "C" 
void  ChRunKernelLCPspeedupdate(		dim3 dim_grid, 
										dim3 dim_threads, 
										CH_REALNUMBER4* d_buffer_bodies,
										CH_REALNUMBER4* d_buffer_reduction,
										unsigned int bodies_data_pitch,
										unsigned int reduction_data_pitch)
{
	//reduction pitch should end up in constant memory
	CUDA_SAFE_CALL(cudaMemcpyToSymbol(deviceReductionPitch, &reduction_data_pitch,sizeof(reduction_data_pitch)));
	//body_pitch should end up in constant memory
	CUDA_SAFE_CALL(cudaMemcpyToSymbol(deviceBodyPitch,&bodies_data_pitch,sizeof(bodies_data_pitch)));

	// Execute the kernel
	ChKernelLCPspeedupdate<<< dim_grid, dim_threads >>>(d_buffer_bodies, d_buffer_reduction);
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
 
__constant__ CH_REALNUMBER stepSize;
 
__global__ void  
ChKernelIntegrateTimeStep(CH_REALNUMBER4* bodies, CH_REALNUMBER4* reduction, bool normalize_quaternion) 
{ 
	// Compute the i,devicePitch values used to access data inside the large 'contact'
	// array using pointer arithmetic.
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;

	// In all phases of the following computations, try to use only those following
	// vars, hoping this will help the compiler to minimize register requirements..
	CH_REALNUMBER4 Rx; 
	CH_REALNUMBER4 Rv; 


	// Do 1st order integration of linear speeds
 
	Rx = bodies[deviceBodyPitch *2 + i];
	Rv = bodies[					 i];
	Rx.x += Rv.x*stepSize;
	Rx.y += Rv.y*stepSize;
	Rx.z += Rv.z*stepSize; 
	bodies[deviceBodyPitch *2 + i] = Rx;
 
	// Do 1st order integration of quaternion position as q[t+dt] = qw_abs^(dt) * q[dt] = q[dt] * qw_local^(dt)
	//   where qw^(dt) is the quaternion { cos(0.5|w|), wx/|w| sin(0.5|w|), wy/|w| sin(0.5|w|), wz/|w| sin(0.5|w|)}^dt
	//   that is argument of sine and cosines are multiplied by dt.
	Rv = bodies[deviceBodyPitch    + i];
	CH_REALNUMBER wlen = sqrtf(Rv.x*Rv.x + Rv.y*Rv.y + Rv.z*Rv.z);
	if (fabs(wlen)>10e-10)
	{
		Rx = fQuaternion_from_AngAxis ( (stepSize*wlen) , Rv.x/wlen, Rv.y/wlen, Rv.z/wlen);
	}
	else
	{
		// to avoid singularity for near zero angular speed:
		Rx.x = 1.f ; Rx.y=Rx.z=Rx.w=0.f;
	}
	CH_REALNUMBER4 mq = fQuaternion_product (bodies[deviceBodyPitch *3 + i], Rx); 

	// normalize quaternion (could be done only once in a while)
	if (normalize_quaternion)
	{
		CH_REALNUMBER rqlen = rsqrtf(mq.x*mq.x + mq.y*mq.y + mq.z*mq.z + mq.w*mq.w);
		mq *= rqlen;
	}

	bodies[deviceBodyPitch *3 + i] = mq;
} 




extern "C"
void ChRunKernelIntegrateTimeStep (dim3 dim_grid, 
								   dim3 dim_threads, 
								   CH_REALNUMBER4*  d_buffer_bodies,
								   CH_REALNUMBER4*  d_buffer_reduction,
								   unsigned int bodies_data_pitch,
								   unsigned int reduction_data_pitch,
								   CH_REALNUMBER mstepSize,
								   bool normalize_quaternion)
{
	//reduction pitch should end up in constant memory
	CUDA_SAFE_CALL(cudaMemcpyToSymbol(deviceReductionPitch, &reduction_data_pitch,sizeof(reduction_data_pitch)));
	//body_pitch should end up in constant memory
	CUDA_SAFE_CALL(cudaMemcpyToSymbol(deviceBodyPitch,&bodies_data_pitch,sizeof(bodies_data_pitch)));
	//stepsize should end up in constant memory
	CUDA_SAFE_CALL(cudaMemcpyToSymbol(stepSize,&mstepSize,sizeof(mstepSize)));

	// Execute the kernel
	ChKernelIntegrateTimeStep<<< dim_grid, dim_threads >>>(d_buffer_bodies, d_buffer_reduction, normalize_quaternion);
}





