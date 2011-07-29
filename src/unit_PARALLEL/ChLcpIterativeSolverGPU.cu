#include "ChLcpIterativeSolverGPU.h"
using namespace chrono;

//helper functions
template <class T, class U> // dot product of the first three elements of float3/float4 values
inline __host__ __device__ float dot3(const T & a, const U & b){ 
	return a.x * b.x + a.y * b.y + a.z * b.z;
}
__constant__ CH_REALNUMBER lcp_omega_const;
__constant__ unsigned int  number_of_bodies_const;
__constant__ unsigned int  number_of_contacts_const;
__constant__ unsigned int  number_of_bilaterals_const;
__constant__ unsigned int  number_of_updates_const;
__constant__ CH_REALNUMBER force_factor_const;			// usually, the step size
__constant__ CH_REALNUMBER negated_recovery_speed_const;
__constant__ CH_REALNUMBER c_factor_const;				// usually 1/dt
__constant__ CH_REALNUMBER step_size_const;
///////////////////////////////////////////////////////////////////////////////////

__host__ __device__ void compute_mat(float3 &A, float3 &B, float3 &C, float4 TT,const float3 &n,const float3 &u,const float3 &w,const float3 &pos){
	float t00 = pos.z * n.y - pos.y * n.z;
	float t01 = TT.x * TT.x;
	float t02 = TT.y * TT.y;
	float t03 = TT.z * TT.z;
	float t04 = TT.w * TT.w;
	float t05 = t01 + t02 - t03 - t04;
	float t06 = -pos.z * n.x + pos.x * n.z;
	float t07 = TT.y * TT.z;
	float t08 = TT.x * TT.w;
	float t09 = t07 + t08;
	float t10 = pos.y * n.x - pos.x * n.y;
	float t11 = TT.y * TT.w;
	float t12 = TT.x * TT.z;
	float t13 = t11 - t12;
	float t14 = t07 - t08;
	float t15 = t01 - t02 + t03 - t04;
	float t16 = TT.z * TT.w;
	float t17 = TT.x * TT.y;
	float t18 = t16 + t17;
	float t19 = t11 + t12;
	float t20 = t16 - t17;
	float t21 = t01 - t02 - t03 + t04;
	float t22 =  pos.z * u.y - pos.y * u.z;
	float t23 = -pos.z * u.x + pos.x * u.z;
	float t24 =  pos.y * u.x - pos.x * u.y;
	float t25 =  pos.z * w.y - pos.y * w.z;
	float t26 = -pos.z * w.x + pos.x * w.z;
	float t27 =  pos.y * w.x - pos.x * w.y;
	A.x =     t00 * t05 + 2 * t06 * t09 + 2 * t10 * t13;
	A.y = 2 * t00 * t14 +     t06 * t15 + 2 * t10 * t18;
	A.z = 2 * t00 * t19 + 2 * t06 * t20 +     t10 * t21;
	B.x =     t22 * t05 + 2 * t23 * t09 + 2 * t24 * t13;
	B.y = 2 * t22 * t14 +     t23 * t15 + 2 * t24 * t18;
	B.z = 2 * t22 * t19 + 2 * t23 * t20 +     t24 * t21;
	C.x =     t25 * t05 + 2 * t26 * t09 + 2 * t27 * t13;
	C.y = 2 * t25 * t14 +     t26 * t15 + 2 * t27 * t18;
	C.z = 2 * t25 * t19 + 2 * t26 * t20 +     t27 * t21;
}
// 	Kernel for a single iteration of the LCP over all contacts
//   	Version 2.0 - Tasora
//	Version 2.2- Hammad (optimized, cleaned etc)
__global__ void LCP_Iteration_Contacts( contactGPU* contacts, CH_REALNUMBER4* bodies, updateGPU* update, uint* offset) {
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
	if(i>=number_of_contacts_const){return;}
	float4 E1 ,B1,B2, E2,W1,W2;
	float3 vB,vA, gamma, N, U, W, T3, T4, T5, T6, T7, T8, gamma_old, sbar;
	float  reg, mu, eta=0;
	vB= contacts[i].I;
	reg=vB.x*c_factor_const;							//Scale contact distance, cfactor is usually 1
	int B1_i = int(vB.y);
	int B2_i = int(vB.z);
	B1=bodies[B1_i];
	B2=bodies[B2_i];
	N= contacts[i].N;								//assume: normalized, and if depth=0 norm=(1,0,0)
	W = fabs(N);									//Gramm Schmidt; work with the global axis that is "most perpendicular" to the contact normal vector;effectively this means looking for the smallest component (in abs) and using the corresponding direction to carry out cross product.
	U = F3(0.0,N.z, -N.y);								//it turns out that X axis is closest to being perpendicular to contact vector;
	if(W.x>W.y){U = F3(-N.z,0.0, N.x);}						//it turns out that Y axis is closest to being perpendicular to contact vector;
	if(W.y>W.z){U = F3(N.y,-N.x, 0.0);}						//it turns out that Z axis is closest to being perpendicular to contact vector;
	U=normalize(U);									//normalize the local contact Y,Z axis
	W=cross(N,U);									//carry out the last cross product to find out the contact local Z axis : multiply the contact normal by the local Y component										
	//if(i==0){printf("%f %f %f | %f %f %f | %f %f %f\n",N.x,N.y,N.z,contacts[i].Pa.x,contacts[i].Pa.y,contacts[i].Pa.z,contacts[i].Pb.x,contacts[i].Pb.y,contacts[i].Pb.z);}
	reg=reg+dot(N,(F3(B2-B1)))*10;

	//reg=min(0.0,max(reg,-1.));

	sbar =contacts[i].Pa-F3(bodies[2*number_of_bodies_const+B1_i]);	//Contact Point on A - Position of A                                
	E1 = bodies[3*number_of_bodies_const+B1_i];						//bring in the Euler parameters associated with body 1;
	compute_mat(T3,T4,T5,E1,N,U,W,sbar);							//A_i,p'*A_A*(sbar~_i,A)

	sbar =contacts[i].Pb-F3(bodies[2*number_of_bodies_const+B2_i]);	//Contact Point on B - Position of B                    
	E2 = bodies[3*number_of_bodies_const+B2_i];						//bring in the Euler parameters associated with body 2;
	compute_mat(T6,T7,T8,E2,N,U,W,sbar);							//A_i,p'*A_B*(sbar~_i,B)
	T6=-T6;	T7=-T7;	T8=-T8;
	//if(i==0){printf("%f %f %f | %f %f %f | %f %f %f\n",T3.x,T3.y,T3.z,T4.x,T4.y,T4.z,T5.x,T5.y,T5.z);}

	W1 = bodies[B1_i+number_of_bodies_const];
	W2 = bodies[B2_i+number_of_bodies_const];

	mu=(W1.w+W2.w)*.5;
	gamma.x = dot3(T3,W1)-dot3(N,B1)+dot3(T6,W2)+dot3(N,B2)+reg;			//+bi	
	gamma.y = dot3(T4,W1)-dot3(U,B1)+dot3(T7,W2)+dot3(U,B2);
	gamma.z = dot3(T5,W1)-dot3(W,B1)+dot3(T8,W2)+dot3(W,B2);
	B1 = bodies[4*number_of_bodies_const+B1_i];					// bring in the inertia attributes; to be used to compute \eta
	B2 = bodies[4*number_of_bodies_const+B2_i];					// bring in the inertia attributes; to be used to compute \eta
	eta =  dot3(T3*T3,B1)+dot3(T4*T4,B1)+dot3(T5*T5,B1);				// update expression of eta	
	eta+=  dot3(T6*T6,B2)+dot3(T7*T7,B2)+dot3(T8*T8,B2);
	eta+= (dot(N,N)+dot(U,U)+dot(W,W))*(B1.w+B2.w);					// multiply by inverse of mass matrix of B1 and B2, add contribution from mass and matrix A_c.
	eta=3.0f/eta;									// final value of eta

	gamma= lcp_omega_const*gamma*eta;						// perform gamma *= omega*eta
	gamma_old =contacts[i].G;
	gamma = gamma_old - gamma;							// perform gamma = gamma_old - gamma ;  in place.
	/// ---- perform projection of 'a' onto friction cone  --------													  
	reg = sqrtf(gamma.y*gamma.y+gamma.z*gamma.z);					// reg = f_tang
	if (reg > (mu * gamma.x)){							// inside upper cone? keep untouched!
		if ((mu * reg) < -gamma.x){						// inside lower cone? reset  normal,u,v to zero!
			gamma = F3(0.f,0.f,0.f);
		} else{									// remaining case: project orthogonally to generator segment of upper cone
			gamma.x =  ( reg * mu + gamma.x )/(mu*mu + 1.f) ;
			reg = (gamma.x * mu)/ reg;					//  reg = tproj_div_t
			gamma.y *= reg;
			gamma.z *= reg;
		}
	}
	contacts[i].G = gamma;								// store gamma_new
	gamma -= gamma_old;								// compute delta_gamma = gamma_new - gamma_old   = delta_gamma.
	//if(i==0){printf("%f %f %f\n",gamma.x,gamma.y,gamma.z);}
	vB=N*gamma.x+U*gamma.y+W*gamma.z;	
	int offset1=offset[i];
	int offset2=offset[i+number_of_contacts_const];
	update[offset1].vel	= -vB*B1.w;					// compute and store dv1
	update[offset1].omega	= (T3*gamma.x+T4*gamma.y+T5*gamma.z)*F3(B1);	// compute dw1 =  Inert.1' * J1w^ * deltagamma  and store  dw1		
	update[offset2].vel	= vB*B2.w;					// compute and store dv2  
	update[offset2].omega	= (T6*gamma.x+T7*gamma.y+T8*gamma.z)*F3(B2);	// compute dw2 =  Inert.2' * J2w^ * deltagamma  and store  dw2	
}
///////////////////////////////////////////////////////////////////////////////////
// Kernel for a single iteration of the LCP over all scalar bilateral contacts
// (a bit similar to the ChKernelLCPiteration above, but without projection etc.)
// Version 2.0 - Tasora
//
__global__ void LCP_Iteration_Bilaterals( CH_REALNUMBER4* bilaterals, CH_REALNUMBER4* bodies, updateGPU* update,uint* offset) { 
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
	if(i>=number_of_bilaterals_const){return;}
	CH_REALNUMBER4 vA,vB; 
	CH_REALNUMBER gamma_new=0, gamma_old; 
	int B1_index=0,B2_index=0;

	B1_index = bilaterals[i].w ; 
	B2_index = bilaterals[i+ number_of_bilaterals_const].w ; 

	// ---- perform   gamma_new = ([J1 J2] {v1 | v2}^ + b) 
	vA = bilaterals[i];								// line 0
	vB = bodies[B1_index];							// v1
	gamma_new += dot3 ( vA , vB);

	vA = bilaterals[i+2*number_of_bilaterals_const];// line 2
	vB = bodies[B1_index + number_of_bodies_const]; // w1
	gamma_new += dot3 ( vA , vB);

	vA = bilaterals[i+number_of_bilaterals_const];	// line 1
	vB = bodies[B2_index];							// v2
	gamma_new += dot3 ( vA , vB);

	vA = bilaterals[i+3*number_of_bilaterals_const];// line 3
	vB = bodies[B2_index + number_of_bodies_const];	// w2
	gamma_new += dot3 ( vA , vB);


	vA = bilaterals[i + 4*number_of_bilaterals_const];	// line 4   (eta, b, gamma, 0)
	gamma_new += vA.y;									// add known term     + b
	gamma_old = vA.z;									// old gamma
	/// ---- perform gamma_new *= omega/g_i
	gamma_new *= lcp_omega_const;						// lcp_omega_const is in constant memory 
	gamma_new *= vA.x;									// eta = 1/g_i;  
	/// ---- perform gamma_new = gamma_old - gamma_new ; in place.
	gamma_new = gamma_old - gamma_new;
	/// ---- perform projection of 'a' (only if simple unilateral behavior C>0 is requested)
	if (vA.w && gamma_new < 0.) {gamma_new=0.;}
	// ----- store gamma_new
	vA.z = gamma_new;
	bilaterals[i + 4*number_of_bilaterals_const] = vA;
	/// ---- compute delta in multipliers: gamma_new = gamma_new - gamma_old   = delta_gamma    , in place.
	gamma_new -= gamma_old; 
	updateGPU temp;
	/// ---- compute dv1 =  invInert.1 * J1^ * deltagamma  
	vB = bodies[B1_index + 4*number_of_bodies_const];					// iJ iJ iJ im
	vA = bilaterals[i]*vB.w*gamma_new; 									// line 0: J1(x)
	temp.vel=F3(vA);//  ---> store  v1 vel. in reduction buffer
	vA = bilaterals[i+2*number_of_bilaterals_const]*vB*gamma_new;		// line 2:  J1(w)		  
	temp.omega=F3(vA);// ---> store  w1 vel. in reduction buffer
	update[offset[2*number_of_contacts_const+i]]=temp;
	vB = bodies[B2_index + 4*number_of_bodies_const];					// iJ iJ iJ im
	vA = bilaterals[i+number_of_bilaterals_const]*vB.w*gamma_new; 		// line 1: J2(x)
	temp.vel=F3(vA);//  ---> store  v2 vel. in reduction buffer	
	vA = bilaterals[i+3*number_of_bilaterals_const]*vB*gamma_new;		// line 3:  J2(w)		  
	temp.omega=F3(vA);// ---> store  w2 vel. in reduction buffer
	update[offset[2*number_of_contacts_const+i+number_of_bilaterals_const]]=temp;
} 
////////////////////////////////////////////////////////////////////////////////////////////////
// Kernel for adding invmass*force*step_size_const to body speed vector.
// This kernel must be applied to the stream of the body buffer.
__global__ void ChKernelLCPaddForces(CH_REALNUMBER4* bodies) 
{ 
	// Compute the i values used to access data inside the large 
	// array using pointer arithmetic.
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
	if(i<number_of_bodies_const){
		if(bodies[i].w){
			CH_REALNUMBER4 mF, minvMasses;
			minvMasses = bodies[i+ 4*number_of_bodies_const];

			// v += m_inv * h * f
			mF =  bodies[i+ 5*number_of_bodies_const]; // vector with f (force)
			mF *= force_factor_const;
			mF *= minvMasses.w; 
			bodies[i] += mF;

			// w += J_inv * h * c
			mF =  bodies[i+ 6*number_of_bodies_const]; // vector with f (torque)
			mF *= force_factor_const;
			mF.x *= minvMasses.x;
			mF.y *= minvMasses.y;
			mF.z *= minvMasses.z;
			bodies[i+number_of_bodies_const] += mF;
		}
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// Updates the speeds in the body buffer with values accumulated in the 
// reduction buffer:   V_new = V_old + delta_speeds

__global__ void LCP_Reduce_Speeds(CH_REALNUMBER4* bodies , updateGPU* update, uint* d_body_num, uint* counter){
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
	if(i>=number_of_updates_const){return;}
	int start=(i==0)? 0:counter[i-1], end=counter[i];
	int id=d_body_num[end-1], j;
	if(bodies[id].w){
		float3 mUpdateV=F3(0);
		float3 mUpdateO=F3(0);
		updateGPU temp;
		for(j=0; j<end-start; j++){
			temp=update[j+start];
			mUpdateV+=temp.vel;
			mUpdateO+=temp.omega;
		}
		bodies[id]+=F4(mUpdateV);
		bodies[id+number_of_bodies_const]+=F4(mUpdateO);
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// Creates a quaternion as a function of a vector of rotation and an angle (the vector is assumed already 
// normalized, and angle is in radians).
inline __host__ __device__ float4 fQuat_from_AngAxis(const float angle, const float v_x,  const float v_y, const float v_z)
{
	float sinhalf= sinf (angle * 0.5f);
	float4 quat; 
	quat.x = cosf (angle * 0.5f);
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
//  number of registers: 12 (suggested 320 threads/block)

__global__ void  LCP_Integrate_Timestep(CH_REALNUMBER4* bodies, bool normalize_quaternion) 
{ 
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
	if(i>=number_of_bodies_const){return;}
	float4 velocity=bodies[i];
	if(velocity.w){
		//if(length(F3(velocity))<.000006){velocity.x=velocity.y=velocity.z=0;}
		bodies[number_of_bodies_const *2 + i] = bodies[number_of_bodies_const *2 + i]+velocity*step_size_const;  // Do 1st order integration of linear speeds
		bodies[i]=velocity;
		// Do 1st order integration of quaternion position as q[t+dt] = qw_abs^(dt) * q[dt] = q[dt] * qw_local^(dt)
		// where qw^(dt) is the quaternion { cos(0.5|w|), wx/|w| sin(0.5|w|), wy/|w| sin(0.5|w|), wz/|w| sin(0.5|w|)}^dt
		// that is argument of sine and cosines are multiplied by dt.

		CH_REALNUMBER4 Rw   = bodies[number_of_bodies_const *1 + i];
		CH_REALNUMBER  wlen = sqrtf(dot3(Rw,Rw));

		Rw=(fabs(wlen)>10e-10)? fQuat_from_AngAxis (step_size_const*wlen , Rw.x/wlen, Rw.y/wlen, Rw.z/wlen): F4(1.,0,0,0);// to avoid singularity for near zero angular speed

		CH_REALNUMBER4 mq = fQuaternion_product(bodies[number_of_bodies_const *3 + i], Rw); 
		mq *= rsqrtf(dot(mq,mq));
		bodies[number_of_bodies_const *3 + i] = mq;
	}
} 

__global__ void ChKernelLCPcomputeOffsets(contactGPU* contacts ,CH_REALNUMBER4* bilaterals,  uint* Body){
	uint i = blockIdx.x * blockDim.x + threadIdx.x;
	if(i<number_of_contacts_const){
		float3 temp=contacts[i].I;
		Body[i]															=temp.y;
		Body[i+number_of_contacts_const]								=temp.z;
	}
	if(i<number_of_bilaterals_const){
		Body[2*number_of_contacts_const+i]								=bilaterals[i].w;
		Body[2*number_of_contacts_const+i+number_of_bilaterals_const]	=bilaterals[i+number_of_bilaterals_const].w;
	}
}

ChLcpIterativeSolverGPU::~ChLcpIterativeSolverGPU(){
	body_number.clear();	
	update_number.clear();	
	offset_counter.clear();	
	update_offset.clear();	
	//delta_correction.clear();
	iteration_update.clear();
	device_body_data.clear();
	device_bilateral_data.clear();
}
void ChLcpIterativeSolverGPU::GPU_Version(){
	cudaEvent_t start, stop;
	float time;

	//START_TIMING(start,stop,time);

	body_number		.resize((number_of_constraints)*2,0);	//the body numbers for each constraint
	update_number	.resize((number_of_constraints)*2,0);	//
	offset_counter	.resize((number_of_constraints)*2,0);
	update_offset	.resize((number_of_constraints)*2,0);
	iteration_update.resize((number_of_constraints)*2);
	COPY_TO_CONST_MEM(c_factor);
	COPY_TO_CONST_MEM(negated_recovery_speed);
	COPY_TO_CONST_MEM(lcp_omega);
	COPY_TO_CONST_MEM(force_factor);
	COPY_TO_CONST_MEM(step_size);
	COPY_TO_CONST_MEM(number_of_contacts);
	COPY_TO_CONST_MEM(number_of_bilaterals);
	COPY_TO_CONST_MEM(number_of_bodies);
	if(number_of_contacts>0||number_of_bilaterals>0){
		device_bilateral_data	=host_bilateral_data;
		ChKernelLCPcomputeOffsets<<< BLOCKS(number_of_constraints),THREADS>>>(
				CONTCAST((*device_contact_data)),
				CASTF4(device_bilateral_data),
				CASTU1(body_number));
		Thrust_Sequence(update_number);
		Thrust_Sequence(update_offset);
		Thrust_Fill(offset_counter,0);
		Thrust_Sort_By_Key(body_number,  update_number);
		Thrust_Sort_By_Key(update_number,update_offset);
		Thrust_Reduce_By_KeyB(number_of_updates,body_number,update_number,offset_counter);
		Thrust_Inclusive_Scan(offset_counter);
	}
	COPY_TO_CONST_MEM(number_of_updates);
	device_body_data		=host_body_data;
	ChKernelLCPaddForces<<< BLOCKS(number_of_bodies), THREADS	>>>(CASTF4(device_body_data));	
	if(number_of_contacts>0||number_of_bilaterals>0){

		for (uint iteration_number = 0; iteration_number < maximum_iterations; iteration_number++){
			LCP_Iteration_Contacts<<< BLOCKS(number_of_contacts), THREADS >>>(
					CONTCAST((*device_contact_data)),
					CASTF4(device_body_data),
					UPDTCAST(iteration_update),
					CASTU1(update_offset));
			LCP_Iteration_Bilaterals<<< BLOCKS(number_of_bilaterals), THREADS >>>(
					CASTF4(device_bilateral_data),
					CASTF4(device_body_data),
					UPDTCAST(iteration_update),
					CASTU1(update_offset));
			LCP_Reduce_Speeds<<< BLOCKS(number_of_updates), THREADS >>>(
					CASTF4(device_body_data),
					UPDTCAST(iteration_update),
					CASTU1(body_number),
					CASTU1(offset_counter));
		}

	}
	//LCP_Integrate_Timestep<<< BLOCKS(number_of_bodies),THREADS>>>(CASTF4(device_body_data), true);
	host_body_data		=	device_body_data;
	//STOP_TIMING(start,stop,time);
	//printf("%f \n",time);
}

void ChLcpIterativeSolverGPU::RunTimeStep()
{
	number_of_constraints=number_of_contacts+number_of_bilaterals;
	force_factor=1.0;
	if(use_cpu){CPU_Version();}else{GPU_Version();}

}
void ChLcpIterativeSolverGPU::Warm_Start(){}
