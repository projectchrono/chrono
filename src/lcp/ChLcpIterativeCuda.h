#ifndef CHLCPITERATIVECUDA_H
#define CHLCPITERATIVECUDA_H

//////////////////////////////////////////////////
//
//   ChLcpIterativeCuda.h
//
///////////////////////////////////////////////////

#define CH_CONTACT_VSIZE 11
#define CH_CONTACT_HSIZE sizeof(CH_REALNUMBER4)

#define CH_BODY_VSIZE 7
#define CH_BODY_HSIZE sizeof(CH_REALNUMBER4)

#define CH_BILATERAL_VSIZE 6
#define CH_BILATERAL_HSIZE sizeof(CH_REALNUMBER4)

#define CH_REDUCTION_VSIZE 2
#define CH_REDUCTION_HSIZE sizeof(CH_REALNUMBER4)


#define CH_PREPROCESSING_SH_MEM_BLOCK_SIZE 13 //note that this odd number will ensure no bank conflicts

#ifndef CH_CUDAGPUEMULATION
	//***ALEX*** TO DO: FIND OPTIMAL VALUES FOR THESE DEFs, SEE OCCUPANCY & .cubin

   // optimized values for cuda 1.1 (to be improved!!)
 #define CH_PREPROCESSING_THREADS_PER_BLOCK 128   
 #define CH_LCPADDFORCES_THREADS_PER_BLOCK 256
 #define CH_LCPITERATION_THREADS_PER_BLOCK 128
 #define CH_LCPITERATIONBILATERALS_THREADS_PER_BLOCK 256
 #define CH_LCPINTEGRATE_THREADS_PER_BLOCK 320
 #define CH_REDUCTION_THREADS_PER_BLOCK 256
 #define CH_SPEEDUPDATE_THREADS_PER_BLOCK 128
#else
 // for device emulation (using too may threads slow down things too much..)
 #define CH_PREPROCESSING_THREADS_PER_BLOCK 8
 #define CH_LCPADDFORCES_THREADS_PER_BLOCK 8
 #define CH_LCPITERATION_THREADS_PER_BLOCK 8
 #define CH_LCPITERATIONBILATERALS_THREADS_PER_BLOCK 8
 #define CH_LCPINTEGRATE_THREADS_PER_BLOCK 8
 #define CH_REDUCTION_THREADS_PER_BLOCK 8
 #define CH_SPEEDUPDATE_THREADS_PER_BLOCK 8
#endif

#endif