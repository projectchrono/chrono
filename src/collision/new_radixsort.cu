/*
* Copyright 1993-2008 NVIDIA Corporation.  All rights reserved.
*
* NOTICE TO USER:   
*
* This source code is subject to NVIDIA ownership rights under U.S. and 
* international Copyright laws.  
*
* NVIDIA MAKES NO REPRESENTATION ABOUT THE SUITABILITY OF THIS SOURCE 
* CODE FOR ANY PURPOSE.  IT IS PROVIDED "AS IS" WITHOUT EXPRESS OR 
* IMPLIED WARRANTY OF ANY KIND.  NVIDIA DISCLAIMS ALL WARRANTIES WITH 
* REGARD TO THIS SOURCE CODE, INCLUDING ALL IMPLIED WARRANTIES OF 
* MERChANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE.   
* IN NO EVENT SHALL NVIDIA BE LIABLE FOR ANY SPECIAL, INDIRECT, INCIDENTAL, 
* OR CONSEQUENTIAL DAMAGES, OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS 
* OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE 
* OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE 
* OR PERFORMANCE OF THIS SOURCE CODE.  
*
* U.S. Government End Users.  This source code is a "commercial item" as 
* that term is defined at 48 C.F.R. 2.101 (OCT 1995), consisting  of 
* "commercial computer software" and "commercial computer software 
* documentation" as such terms are used in 48 C.F.R. 12.212 (SEPT 1995) 
* and is provided to the U.S. Government only as a commercial end item.  
* Consistent with 48 C.F.R.12.212 and 48 C.F.R. 227.7202-1 through 
* 227.7202-4 (JUNE 1995), all U.S. Government End Users acquire the 
* source code with only those rights set forth herein.
*/

/*
	CUDA-optimized Radix Sort see:
	
	Designing Efficient Sorting Algorithms for Manycore GPUs,
	Nadathur Satish, Mark Harris, and Michael Garland. Technical Report NVR-2008-001, September 2008.
*/

#include "new_radixsort.h"
#include <string.h>
#include <stdio.h>

#define CUDPP_STATIC_LIB
#include "cudpp/cudpp.h"

#include <algorithm>

#ifdef __DEVICE_EMULATION__
#define __SYNC __syncthreads();
#else
#define __SYNC
#endif

typedef unsigned int uint;

extern "C"
void checkCudaError(const char *msg)
{
#if defined(_DEBUG) || defined(DEBUG)
    cudaError_t e = cudaThreadSynchronize();
    if( e != cudaSuccess )
    {
        fprintf(stderr, "CUDA Error %s : %s\n", msg, cudaGetErrorString(e));
        exit(EXIT_FAILURE);
    }
    e = cudaGetLastError();
    if( e != cudaSuccess )
    {
        fprintf(stderr, "CUDA Error %s : %s\n", msg, cudaGetErrorString(e));
        exit(EXIT_FAILURE);
    }
#endif
}

// ================================================================================================
// flip a float for sorting
//  finds SIGN of fp number.
//  if it's 1 (negative float), it flips all bits
//  if it's 0 (positive float), it flips the sign only
// ================================================================================================
template <bool doFlip>
__device__ uint floatFlip(uint f)
{
    if (doFlip)
    {
        uint mask = -int(f >> 31) | 0x80000000;
	    return f ^ mask;
    }
    else
        return f;
}

// ================================================================================================
// flip a float back (invert FloatFlip)
//  signed was flipped from above, so:
//  if sign is 1 (negative), it flips the sign bit back
//  if sign is 0 (positive), it flips all bits back
// ================================================================================================
template <bool doFlip>
__device__ uint floatUnflip(uint f)
{
    if (doFlip)
    {
        uint mask = ((f >> 31) - 1) | 0x80000000;
	    return f ^ mask;
    }
    else
        return f;
}

__global__ void flipFloats(uint *values, uint numValues)
{
    uint index = __umul24(blockDim.x*4, blockIdx.x) + threadIdx.x; 
    if (index < numValues) values[index] = floatFlip<true>(values[index]);
    index += blockDim.x;
    if (index < numValues) values[index] = floatFlip<true>(values[index]);
    index += blockDim.x;
    if (index < numValues) values[index] = floatFlip<true>(values[index]);
    index += blockDim.x;
    if (index < numValues) values[index] = floatFlip<true>(values[index]);
}

__global__ void unflipFloats(uint *values, uint numValues)
{
    uint index = __umul24(blockDim.x*4, blockIdx.x) + threadIdx.x; 
    if (index < numValues) values[index] = floatUnflip<true>(values[index]);
    index += blockDim.x;
    if (index < numValues) values[index] = floatUnflip<true>(values[index]);
    index += blockDim.x;
    if (index < numValues) values[index] = floatUnflip<true>(values[index]);
    index += blockDim.x;
    if (index < numValues) values[index] = floatUnflip<true>(values[index]);
}


//----------------------------------------------------------------------------
// Scans each warp in parallel ("warp-scan"), one element per thread.
// uses 2 numElements of shared memory per thread (64 numElements per warp)
//----------------------------------------------------------------------------
template<class T, int maxlevel>
__device__ T scanwarp(T val, T* sData)
{
    // The following is the same as 2 * NewRadixSort::WARP_SIZE * warpId + threadInWarp = 
    // 64*(threadIdx.x >> 5) + (threadIdx.x & (NewRadixSort::WARP_SIZE - 1))
    int idx = 2 * threadIdx.x - (threadIdx.x & (NewRadixSort::WARP_SIZE - 1));
    sData[idx] = 0;
    idx += NewRadixSort::WARP_SIZE;
    sData[idx] = val;          __SYNC

#ifdef __DEVICE_EMULATION__
        T t = sData[idx -  1]; __SYNC 
        sData[idx] += t;       __SYNC
        t = sData[idx -  2];   __SYNC 
        sData[idx] += t;       __SYNC
        t = sData[idx -  4];   __SYNC 
        sData[idx] += t;       __SYNC
        t = sData[idx -  8];   __SYNC 
        sData[idx] += t;       __SYNC
        t = sData[idx - 16];   __SYNC 
        sData[idx] += t;       __SYNC
#else
        if (0 <= maxlevel) { sData[idx] += sData[idx - 1]; } __SYNC
        if (1 <= maxlevel) { sData[idx] += sData[idx - 2]; } __SYNC
        if (2 <= maxlevel) { sData[idx] += sData[idx - 4]; } __SYNC
        if (3 <= maxlevel) { sData[idx] += sData[idx - 8]; } __SYNC
        if (4 <= maxlevel) { sData[idx] += sData[idx -16]; } __SYNC
#endif

        return sData[idx] - val;  // convert inclusive -> exclusive
}

//----------------------------------------------------------------------------
// scan4 scans 4*NewNewRadixSort::CTA_SIZE numElements in a block (4 per thread), using 
// a warp-scan algorithm
//----------------------------------------------------------------------------
__device__ uint4 scan4(uint4 idata)
{    
    extern  __shared__  uint ptr[];
    
    uint idx = threadIdx.x;

    uint4 val4 = idata;
    uint sum[3];
    sum[0] = val4.x;
    sum[1] = val4.y + sum[0];
    sum[2] = val4.z + sum[1];
    
    uint val = val4.w + sum[2];
    
    val = scanwarp<uint, 4>(val, ptr);
    __syncthreads();

    if ((idx & (NewRadixSort::WARP_SIZE - 1)) == NewRadixSort::WARP_SIZE - 1)
    {
        ptr[idx >> 5] = val + val4.w + sum[2];
    }
    __syncthreads();

#ifndef __DEVICE_EMULATION__
    if (idx < NewRadixSort::WARP_SIZE)
#endif
    {
        ptr[idx] = scanwarp<uint, 2>(ptr[idx], ptr);
    }
    __syncthreads();

    val += ptr[idx >> 5];

    val4.x = val;
    val4.y = val + sum[0];
    val4.z = val + sum[1];
    val4.w = val + sum[2];

    return val4;
}

//----------------------------------------------------------------------------
//
// Rank is the core of the radix sort loop.  Given a predicate, it
// computes the output position for each thread in an ordering where all
// True threads come first, followed by all False threads.
// 
// This version handles 4 predicates per thread; hence, "rank4".
//
//----------------------------------------------------------------------------
template <int ctasize>
__device__ uint4 rank4(uint4 preds)
{
    uint4 address = scan4(preds);  

    __shared__ uint numtrue;
    if (threadIdx.x == ctasize-1)
    {
        numtrue = address.w + preds.w;
    }
    __syncthreads();

    uint4 rank;
    uint idx = threadIdx.x << 2;
    rank.x = (preds.x) ? address.x : numtrue + idx   - address.x;
    rank.y = (preds.y) ? address.y : numtrue + idx + 1 - address.y;
    rank.z = (preds.z) ? address.z : numtrue + idx + 2 - address.z;
    rank.w = (preds.w) ? address.w : numtrue + idx + 3 - address.w;	

    return rank;
}

//----------------------------------------------------------------------------
// Uses rank to sort one bit at a time: Sorts a block according
// to bits startbit -> nbits + startbit
//----------------------------------------------------------------------------
template<uint nbits, uint startbit, bool floatFlip>
__device__ void radixSortBlock(uint4 &key, uint4 &value)
{
    extern __shared__ uint sMem1[];

    for(uint shift = startbit; shift < (startbit + nbits); ++shift)
    {        
        uint4 lsb;
        lsb.x = !((key.x >> shift) & 0x1);
        lsb.y = !((key.y >> shift) & 0x1);
        lsb.z = !((key.z >> shift) & 0x1);
        lsb.w = !((key.w >> shift) & 0x1);

        uint4 r = rank4<NewRadixSort::CTA_SIZE>(lsb);

#if 1
        // This arithmetic strides the ranks across 4 CTA_SIZE regions
        sMem1[(r.x & 3) * NewRadixSort::CTA_SIZE + (r.x >> 2)] = key.x;
        sMem1[(r.y & 3) * NewRadixSort::CTA_SIZE + (r.y >> 2)] = key.y;
        sMem1[(r.z & 3) * NewRadixSort::CTA_SIZE + (r.z >> 2)] = key.z;
        sMem1[(r.w & 3) * NewRadixSort::CTA_SIZE + (r.w >> 2)] = key.w;
        __syncthreads();

        // The above allows us to read without 4-way bank conflicts:
        key.x = sMem1[threadIdx.x];
        key.y = sMem1[threadIdx.x +     NewRadixSort::CTA_SIZE];
        key.z = sMem1[threadIdx.x + 2 * NewRadixSort::CTA_SIZE];
        key.w = sMem1[threadIdx.x + 3 * NewRadixSort::CTA_SIZE];

        __syncthreads();

        sMem1[(r.x & 3) * NewRadixSort::CTA_SIZE + (r.x >> 2)] = value.x;
        sMem1[(r.y & 3) * NewRadixSort::CTA_SIZE + (r.y >> 2)] = value.y;
        sMem1[(r.z & 3) * NewRadixSort::CTA_SIZE + (r.z >> 2)] = value.z;
        sMem1[(r.w & 3) * NewRadixSort::CTA_SIZE + (r.w >> 2)] = value.w;
        __syncthreads();

        value.x = sMem1[threadIdx.x];
        value.y = sMem1[threadIdx.x +     NewRadixSort::CTA_SIZE];
        value.z = sMem1[threadIdx.x + 2 * NewRadixSort::CTA_SIZE];
        value.w = sMem1[threadIdx.x + 3 * NewRadixSort::CTA_SIZE];
#else
        sMem1[r.x] = key.x;
        sMem1[r.y] = key.y;
        sMem1[r.z] = key.z;
        sMem1[r.w] = key.w;
        __syncthreads();

        // This access has 4-way bank conflicts
        key = sMem[threadIdx.x];

        __syncthreads();

        sMem1[r.x] = value.x;
        sMem1[r.y] = value.y;
        sMem1[r.z] = value.z;
        sMem1[r.w] = value.w;
        __syncthreads();

        value = sMem[threadIdx.x];
#endif

        __syncthreads();
    }
}

//----------------------------------------------------------------------------
//
// radixSortBlocks sorts each block of data independently in shared
// memory.  
//
// Done in two separate stages.  This stage calls radixSortBlock on each block 
// independently, sorting on the basis of bits (startbit) -> (startbit + nbits)
//----------------------------------------------------------------------------
template<uint nbits, uint startbit, bool fullBlocks, bool flip>
__global__ void radixSortBlocks(uint4* keysOut, uint4* valuesOut, 
                                uint4* keysIn, uint4* valuesIn, 
                                uint numElements, uint startBlock)
{
    extern __shared__ uint4 sMem[];

    uint4 key, value;

    const uint blockId = blockIdx.x + startBlock;
    const uint i = blockId * blockDim.x + threadIdx.x;
    const uint idx = i << 2;

    // handle non-full last block if array is not multiple of 1024 numElements
    if (!fullBlocks && idx+3 >= numElements)
    {
        if (idx >= numElements)
        {
            key   = make_uint4(UINT_MAX, UINT_MAX, UINT_MAX, UINT_MAX);
            value = make_uint4(UINT_MAX, UINT_MAX, UINT_MAX, UINT_MAX);
        }
        else
        {
            // for non-full block, we handle uint1 values instead of uint4
            uint *keys1    = (uint*)keysIn;
            uint *values1  = (uint*)valuesIn;

            key.x = (idx   < numElements) ? floatFlip<flip>(keys1[idx])   : UINT_MAX;
            key.y = (idx+1 < numElements) ? floatFlip<flip>(keys1[idx+1]) : UINT_MAX;
            key.z = (idx+2 < numElements) ? floatFlip<flip>(keys1[idx+2]) : UINT_MAX;
            key.w = UINT_MAX;

            value.x = (idx   < numElements) ? values1[idx]   : UINT_MAX;
            value.y = (idx+1 < numElements) ? values1[idx+1] : UINT_MAX;
            value.z = (idx+2 < numElements) ? values1[idx+2] : UINT_MAX;
            value.w = UINT_MAX;
        }
    }
    else
    {
        key = keysIn[i];
        value = valuesIn[i];

        if (flip)
        {
            key.x = floatFlip<flip>(key.x);
            key.y = floatFlip<flip>(key.y);
            key.z = floatFlip<flip>(key.z);
            key.w = floatFlip<flip>(key.w);
        }
    }
    __syncthreads();
    radixSortBlock<nbits, startbit, false>(key, value);
    __syncthreads();

    // handle non-full last block if array is not multiple of 1024 numElements
    if(!fullBlocks && idx+3 >= numElements)
    {
        if (idx < numElements) 
        {
            // for non-full block, we handle uint1 values instead of uint4
            uint *keys1   = (uint*)keysOut;
            uint *values1 = (uint*)valuesOut;

            keys1[idx]   = key.x;
            values1[idx] = value.x;

            if (idx + 1 < numElements)
            {
                keys1[idx + 1]   = key.y;
                values1[idx + 1] = value.y;

                if (idx + 2 < numElements)
                {
                    keys1[idx + 2]   = key.z;
                    values1[idx + 2] = value.z;
                }
            }
        }
    }
    else
    {
        keysOut[i]   = key;
        valuesOut[i] = value;
    }
}

//----------------------------------------------------------------------------
// Given an array with blocks sorted according to a 4-bit radix group, each 
// block counts the number of keys that fall into each radix in the group, and 
// finds the starting offset of each radix in the block.  Writes the radix 
// counts to counters, and the starting offsets to blockOffsets.
//----------------------------------------------------------------------------
template<uint startbit, bool fullBlocks>
__global__ void findRadixOffsets(uint2 *keys, 
                                 uint  *counters, 
                                 uint  *blockOffsets, 
                                 uint   numElements,
                                 uint   totalBlocks,
                                 uint   startBlock)
{
    extern __shared__ uint2 sMem2[];

    uint2 *sRadix2         = (uint2*)sMem2;
    uint  *sRadix1         = (uint*) sRadix2; 
    uint  *sStartPointers  = (uint*)(sMem2 + NewRadixSort::CTA_SIZE);

    uint blockId = blockIdx.x + startBlock;
    const uint i = blockId * blockDim.x + threadIdx.x;

    uint2 radix2;

    // handle non-full last block if array is not multiple of 1024 numElements
    if(!fullBlocks && ((i + 1) << 1 ) > numElements )
    {
        // handle uint1 rather than uint2 for non-full blocks
        uint *keys1 = (uint*)keys;
        uint j = i << 1; 

        radix2.x = (j < numElements) ? keys1[j] : UINT_MAX; 
        j++;
        radix2.y = (j < numElements) ? keys1[j] : UINT_MAX;
    }
    else
    {
        radix2 = keys[i];
    }

    sRadix1[2 * threadIdx.x]     = (radix2.x >> startbit) & 0xF;
    sRadix1[2 * threadIdx.x + 1] = (radix2.y >> startbit) & 0xF;

    // Finds the position where the sRadix1 entries differ and stores start 
    // index for each radix.
    if(threadIdx.x < 16) 
    { 
        sStartPointers[threadIdx.x] = 0; 
    }
    __syncthreads();

    if((threadIdx.x > 0) && (sRadix1[threadIdx.x] != sRadix1[threadIdx.x - 1]) ) 
    {
        sStartPointers[sRadix1[threadIdx.x]] = threadIdx.x;
    }
    if(sRadix1[threadIdx.x + NewRadixSort::CTA_SIZE] != sRadix1[threadIdx.x + NewRadixSort::CTA_SIZE - 1]) 
    {
        sStartPointers[sRadix1[threadIdx.x + NewRadixSort::CTA_SIZE]] = threadIdx.x + NewRadixSort::CTA_SIZE;
    }
    __syncthreads();

    if(threadIdx.x < 16) 
    {
        blockOffsets[blockId*16 + threadIdx.x] = sStartPointers[threadIdx.x];
    }
    __syncthreads();

    // Compute the sizes of each block.
    if((threadIdx.x > 0) && (sRadix1[threadIdx.x] != sRadix1[threadIdx.x - 1]) ) 
    {
        sStartPointers[sRadix1[threadIdx.x - 1]] = 
            threadIdx.x - sStartPointers[sRadix1[threadIdx.x - 1]];
    }
    if(sRadix1[threadIdx.x + NewRadixSort::CTA_SIZE] != sRadix1[threadIdx.x + NewRadixSort::CTA_SIZE - 1] ) 
    {
        sStartPointers[sRadix1[threadIdx.x + NewRadixSort::CTA_SIZE - 1]] = 
            threadIdx.x + NewRadixSort::CTA_SIZE - sStartPointers[sRadix1[threadIdx.x + NewRadixSort::CTA_SIZE - 1]];
    }

    if(threadIdx.x == NewRadixSort::CTA_SIZE - 1) 
    {
        sStartPointers[sRadix1[2 * NewRadixSort::CTA_SIZE - 1]] = 
            2 * NewRadixSort::CTA_SIZE - sStartPointers[sRadix1[2 * NewRadixSort::CTA_SIZE - 1]];
    }
    __syncthreads();
    
    if(threadIdx.x < 16) 
    {
        counters[threadIdx.x * totalBlocks + blockId] = 
            sStartPointers[threadIdx.x];
    }
}


//----------------------------------------------------------------------------
// reorderData shuffles data in the array globally after the radix offsets 
// have been found. Depends on NewRadixSort::CTA_SIZE being 16 * number of radices 
// (i.e. 16 * 2^nbits).
// 
// This is quite fast and fully coalesces memory writes, albeit by doing extra 
// (potentially wasted) work allocating threads to portions of memory that are 
// not written out. Significantly faster than the generic approach on G80.
//----------------------------------------------------------------------------
template<uint startbit, bool fullBlocks, bool manualCoalesce, bool unflip>
__global__ void reorderData(uint  *outKeys, 
                            uint  *outValues, 
                            uint2 *keys, 
                            uint2 *values, 
                            uint  *blockOffsets, 
                            uint  *offsets, 
                            uint  *sizes, 
                            uint   numElements,
                            uint   totalBlocks,
                            uint   startBlock)
{
    __shared__ uint2 sKeys2[NewRadixSort::CTA_SIZE];
    __shared__ uint2 sValues2[NewRadixSort::CTA_SIZE];
    __shared__ uint sOffsets[16];
    __shared__ uint sBlockOffsets[16];

    uint *sKeys1   = (uint*)sKeys2; 
    uint *sValues1 = (uint*)sValues2; 

    const uint blockId = blockIdx.x + startBlock;
    const uint i = blockId * blockDim.x + threadIdx.x;

    // handle non-full last block if array is not multiple of 1024 numElements
    if(!fullBlocks && (((i + 1) << 1) > numElements))
    {
        uint *keys1   = (uint*)keys;
        uint *values1 = (uint*)values;
        uint j = i << 1; 

        sKeys1[threadIdx.x << 1]   = (j < numElements) ? keys1[j]   : UINT_MAX; 
        sValues1[threadIdx.x << 1] = (j < numElements) ? values1[j] : UINT_MAX; 
        j++; 
        sKeys1[(threadIdx.x << 1) + 1]   = (j < numElements) ? keys1[j]   : UINT_MAX; 
        sValues1[(threadIdx.x << 1) + 1] = (j < numElements) ? values1[j] : UINT_MAX; 
    }
    else
    {
        sKeys2[threadIdx.x]   = keys[i];
        sValues2[threadIdx.x] = values[i];
    }

    if (!manualCoalesce)
    {
        if(threadIdx.x < 16)  
        {
            sOffsets[threadIdx.x]      = offsets[threadIdx.x * totalBlocks + blockId];
            sBlockOffsets[threadIdx.x] = blockOffsets[blockId * 16 + threadIdx.x];
        }
        __syncthreads();

        uint radix = (sKeys1[threadIdx.x] >> startbit) & 0xF;
	    uint globalOffset = sOffsets[radix] + threadIdx.x - sBlockOffsets[radix];
	    
        if (fullBlocks || globalOffset < numElements)
        {
	        outKeys[globalOffset]   = floatUnflip<unflip>(sKeys1[threadIdx.x]);
	        outValues[globalOffset] = sValues1[threadIdx.x];
        }

        radix = (sKeys1[threadIdx.x + NewRadixSort::CTA_SIZE] >> startbit) & 0xF;
	    globalOffset = sOffsets[radix] + threadIdx.x + NewRadixSort::CTA_SIZE - sBlockOffsets[radix];
	    
        if (fullBlocks || globalOffset < numElements)
        {
	        outKeys[globalOffset]   = floatUnflip<unflip>(sKeys1[threadIdx.x + NewRadixSort::CTA_SIZE]);
	        outValues[globalOffset] = sValues1[threadIdx.x + NewRadixSort::CTA_SIZE];
        }
    }
    else
    {
        __shared__ uint sSizes[16];

        if(threadIdx.x < 16)  
        {
            sOffsets[threadIdx.x]      = offsets[threadIdx.x * totalBlocks + blockId];
            sBlockOffsets[threadIdx.x] = blockOffsets[blockId * 16 + threadIdx.x];
            sSizes[threadIdx.x]        = sizes[threadIdx.x * totalBlocks + blockId];
        }
        __syncthreads();

        // 1 half-warp is responsible for writing out all values for 1 radix. 
        // Loops if there are more than 16 values to be written out. 
        // All start indices are rounded down to the nearest multiple of 16, and
        // all end indices are rounded up to the nearest multiple of 16.
        // Thus it can do extra work if the start and end indices are not multiples of 16
        // This is bounded by a factor of 2 (it can do 2X more work at most).

        const uint halfWarpID     = threadIdx.x >> 4;

        const uint halfWarpOffset = threadIdx.x & 0xF;
        const uint leadingInvalid = sOffsets[halfWarpID] & 0xF;

        uint startPos = sOffsets[halfWarpID] & 0xFFFFFFF0;
        uint endPos   = (sOffsets[halfWarpID] + sSizes[halfWarpID]) + 15 - 
                        ((sOffsets[halfWarpID] + sSizes[halfWarpID] - 1) & 0xF);
        uint numIterations = endPos - startPos;

        uint outOffset = startPos + halfWarpOffset;
        uint inOffset  = sBlockOffsets[halfWarpID] - leadingInvalid + halfWarpOffset;

        for(uint j = 0; j < numIterations; j += 16, outOffset += 16, inOffset += 16)
        {       
            if( (outOffset >= sOffsets[halfWarpID]) && 
                (inOffset - sBlockOffsets[halfWarpID] < sSizes[halfWarpID])) 
            {
                if(blockId < totalBlocks - 1 || outOffset < numElements) 
                {
                    outKeys[outOffset]   = floatUnflip<unflip>(sKeys1[inOffset]);
                    outValues[outOffset] = sValues1[inOffset];
                }
            }       
        }
    }
}

//----------------------------------------------------------------------------
// Perform one step of the radix sort.  Sorts by nbits key bits per step, 
// starting at startbit.
//----------------------------------------------------------------------------
template<uint nbits, uint startbit, bool flip, bool unflip>
void radixSortStep(uint *keys, 
                   uint *values, 
                   uint *tempKeys, 
                   uint *tempValues, 
                   uint *counters, 
                   uint *countersSum, 
                   uint *blockOffsets, 
                   CUDPPHandle scanPlan,
                   uint numElements,
                   bool manualCoalesce)
{
    const uint eltsPerBlock = NewRadixSort::CTA_SIZE * 4;
    const uint eltsPerBlock2 = NewRadixSort::CTA_SIZE * 2;

    bool fullBlocks = ((numElements % eltsPerBlock) == 0);
    uint numBlocks = (fullBlocks) ? 
        (numElements / eltsPerBlock) : 
        (numElements / eltsPerBlock + 1);
    uint numBlocks2 = ((numElements % eltsPerBlock2) == 0) ?
        (numElements / eltsPerBlock2) : 
        (numElements / eltsPerBlock2 + 1);

    const uint max1DBlocks = 65535;

    
    for (uint block = 0; block < numBlocks; block += max1DBlocks)
    {
        uint blocks   = min(max1DBlocks, numBlocks - block);
        
        if (blocks < max1DBlocks && !fullBlocks)
        {
            radixSortBlocks<nbits, startbit, false, flip>
                <<<blocks, NewRadixSort::CTA_SIZE, 4 * NewRadixSort::CTA_SIZE * sizeof(uint)>>>
                    ((uint4*)tempKeys, (uint4*)tempValues, (uint4*)keys, (uint4*)values, numElements, block);
        }
        else
        {
            radixSortBlocks<nbits, startbit, true, flip>
                <<<blocks, NewRadixSort::CTA_SIZE, 4 * NewRadixSort::CTA_SIZE * sizeof(uint)>>>
                    ((uint4*)tempKeys, (uint4*)tempValues, (uint4*)keys, (uint4*)values, numElements, block);
        }
    }

    for (uint block = 0; block < numBlocks2; block += max1DBlocks)
    {
        uint blocks   = min(max1DBlocks, numBlocks2 - block);

        if (blocks < max1DBlocks && !fullBlocks)
        {
            findRadixOffsets<startbit, false>
                <<<blocks, NewRadixSort::CTA_SIZE, 3 * NewRadixSort::CTA_SIZE * sizeof(uint)>>>
                    ((uint2*)tempKeys, counters, blockOffsets, numElements, numBlocks2, block);
        }
        else
        {
            findRadixOffsets<startbit, true>
                <<<blocks, NewRadixSort::CTA_SIZE, 3 * NewRadixSort::CTA_SIZE * sizeof(uint)>>>
                    ((uint2*)tempKeys, counters, blockOffsets, numElements, numBlocks2, block);
        }
    }

    cudppScan(scanPlan, countersSum, counters, 16*numBlocks2);

    for (uint block = 0; block < numBlocks2; block += max1DBlocks)
    {
        uint blocks   = min(max1DBlocks, numBlocks2 - block);
        
        if (blocks < max1DBlocks && !fullBlocks)
        {
            if (manualCoalesce)
            {
                reorderData<startbit, false, true, unflip><<<blocks, NewRadixSort::CTA_SIZE>>>
                    (keys, values, (uint2*)tempKeys, (uint2*)tempValues, 
                     blockOffsets, countersSum, counters, numElements, numBlocks2, block);
            }
            else
            {
                reorderData<startbit, false, false, unflip><<<blocks, NewRadixSort::CTA_SIZE>>>
                    (keys, values, (uint2*)tempKeys, (uint2*)tempValues, 
                     blockOffsets, countersSum, counters, numElements, numBlocks2, block);
            }
        }
        else
        {
            if (manualCoalesce)
            {
                reorderData<startbit, true, true, unflip><<<blocks, NewRadixSort::CTA_SIZE>>>
                    (keys, values, (uint2*)tempKeys, (uint2*)tempValues, 
                     blockOffsets, countersSum, counters, numElements, numBlocks2, block);
            }
            else
            {
                reorderData<startbit, true, false, unflip><<<blocks, NewRadixSort::CTA_SIZE>>>
                    (keys, values, (uint2*)tempKeys, (uint2*)tempValues, 
                     blockOffsets, countersSum, counters, numElements, numBlocks2, block);
            }
        }
    }    

    checkCudaError("radixSortStep");
}

//----------------------------------------------------------------------------
// Optimization for sorts of fewer than 4 * CTA_SIZE elements
//----------------------------------------------------------------------------
template <bool flip>
void radixSortSingleBlock(uint *keys, 
                          uint *values, 
                          uint numElements)
{
    bool fullBlocks = (numElements % (NewRadixSort::CTA_SIZE * 4) == 0);
    if (fullBlocks)
    {
        radixSortBlocks<32, 0, true, flip>
            <<<1, NewRadixSort::CTA_SIZE, 4 * NewRadixSort::CTA_SIZE * sizeof(uint)>>>
                ((uint4*)keys, (uint4*)values, 
                 (uint4*)keys, (uint4*)values, 
                 numElements, 0);
    }
    else
    {
        radixSortBlocks<32, 0, false, flip>
            <<<1, NewRadixSort::CTA_SIZE, 4 * NewRadixSort::CTA_SIZE * sizeof(uint)>>>
                ((uint4*)keys, (uint4*)values, 
                 (uint4*)keys, (uint4*)values, 
                 numElements, 0);
    }

    if (flip)
            unflipFloats<<<1, NewRadixSort::CTA_SIZE>>>(keys, numElements);

    checkCudaError("radixSortSingleBlock");
}

//----------------------------------------------------------------------------
// Optimization for sorts of WARP_SIZE or fewer elements
//----------------------------------------------------------------------------
template <bool flip>
__global__ 
void radixSortSingleWarp(uint *keys, 
                         uint *values, 
                         uint numElements)
{
    __shared__ uint sKeys[NewRadixSort::WARP_SIZE];
    __shared__ uint sValues[NewRadixSort::WARP_SIZE];
    __shared__ uint sFlags[NewRadixSort::WARP_SIZE];

    sKeys[threadIdx.x]   = floatFlip<flip>(keys[threadIdx.x]);
    sValues[threadIdx.x] = values[threadIdx.x];
    
    __SYNC // emulation only

    for(uint i = 1; i < numElements; i++)
    {
        uint key_i = sKeys[i];
        uint val_i = sValues[i];
        
        sFlags[threadIdx.x] = 0;
        
        if( (threadIdx.x < i) && (sKeys[threadIdx.x] > key_i) ) 
        {
            uint temp = sKeys[threadIdx.x];
            uint tempval = sValues[threadIdx.x];
            sFlags[threadIdx.x] = 1;
            sKeys[threadIdx.x + 1] = temp;
            sValues[threadIdx.x + 1] = tempval;
            sFlags[threadIdx.x + 1] = 0;
        }
        if(sFlags[threadIdx.x] == 1 )
        {
            sKeys[threadIdx.x] = key_i;
            sValues[threadIdx.x] = val_i;
        }

        __SYNC // emulation only

    }
    keys[threadIdx.x]   = floatUnflip<flip>(sKeys[threadIdx.x]);
    values[threadIdx.x] = sValues[threadIdx.x];
}

//----------------------------------------------------------------------------
// Main radix sort function.  Sorts in place in the keys and values arrays,
// but uses the other device arrays as temporary storage.  All pointer 
// parameters are device pointers.  Uses cudppScan() for the prefix sum of
// radix counters.
//----------------------------------------------------------------------------
extern "C"
void radixSort(uint *keys, 
               uint *values, 
               uint *tempKeys, 
               uint *tempValues,
               uint *counters,
               uint *countersSum,
               uint *blockOffsets,
               CUDPPHandle scanPlan,
               uint numElements, 
               uint keyBits,
               bool manualCoalesce = true,
               bool flipBits = false)
{
    if(numElements <= NewRadixSort::WARP_SIZE)
    {
        if (flipBits)
            radixSortSingleWarp<true><<<1, numElements>>>(keys, values, numElements);
        else
            radixSortSingleWarp<false><<<1, numElements>>>(keys, values, numElements);
        checkCudaError("radixSortSingleWarp");
        return;
    }
    if(numElements <= NewRadixSort::CTA_SIZE * 4)
    {
        if (flipBits)
            radixSortSingleBlock<true>(keys, values, numElements);
        else
            radixSortSingleBlock<false>(keys, values, numElements);
        return;
    }

    // flip float bits on the first pass, unflip on the last pass
    if (flipBits) 
    {
            radixSortStep<4,  0, true, false>(keys, values, tempKeys, tempValues, 
                                              counters, countersSum, blockOffsets, 
                                              scanPlan, numElements, manualCoalesce);
    }
    else
    {       radixSortStep<4,  0, false, false>(keys, values, tempKeys, tempValues, 
                                               counters, countersSum, blockOffsets, 
                                               scanPlan, numElements, manualCoalesce);
    }

    if (keyBits > 4)
    {
            radixSortStep<4,  4, false, false>(keys, values, tempKeys, tempValues, 
                                               counters, countersSum, blockOffsets, 
                                               scanPlan, numElements, manualCoalesce);
    }
    if (keyBits > 8)
    {
            radixSortStep<4,  8, false, false>(keys, values, tempKeys, tempValues, 
                                               counters, countersSum, blockOffsets, 
                                               scanPlan, numElements, manualCoalesce);
    }
    if (keyBits > 12)
    {
            radixSortStep<4, 12, false, false>(keys, values, tempKeys, tempValues, 
                                               counters, countersSum, blockOffsets, 
                                               scanPlan, numElements, manualCoalesce);
    }
    if (keyBits > 16)
    {
            radixSortStep<4, 16, false, false>(keys, values, tempKeys, tempValues, 
                                               counters, countersSum, blockOffsets, 
                                               scanPlan, numElements, manualCoalesce);
    }
    if (keyBits > 20)
    {
            radixSortStep<4, 20, false, false>(keys, values, tempKeys, tempValues, 
                                               counters, countersSum, blockOffsets, 
                                               scanPlan, numElements, manualCoalesce);
    }
    if (keyBits > 24)
    {
            radixSortStep<4, 24, false, false>(keys, values, tempKeys, tempValues, 
                                               counters, countersSum, blockOffsets, 
                                               scanPlan, numElements, manualCoalesce);
    }
    if (keyBits > 28)
    {
        if (flipBits) // last pass
        {
            radixSortStep<4, 28, false, true>(keys, values, tempKeys, tempValues, 
                                              counters, countersSum, blockOffsets, 
                                              scanPlan, numElements, manualCoalesce);
        }
        else
        {
            radixSortStep<4, 28, false, false>(keys, values, tempKeys, tempValues, 
                                               counters, countersSum, blockOffsets, 
                                               scanPlan, numElements, manualCoalesce);
        }
    }

    checkCudaError("radixSort");
}

extern "C"
void radixSortFloatKeys(float *keys, 
                        uint  *values, 
                        float *tempKeys, 
                        uint  *tempValues,
                        uint  *counters,
                        uint  *countersSum,
                        uint  *blockOffsets,
                        CUDPPHandle scanPlan,
                        uint  numElements, 
                        uint  keyBits,
                        bool  manualCoalesce,
                        bool  negativeKeys)
{
    radixSort((uint*)keys, values, (uint*)tempKeys, tempValues, counters, 
              countersSum, blockOffsets, scanPlan, numElements, keyBits, 
              manualCoalesce, negativeKeys);
    checkCudaError("radixSortFloatKeys");
}

//----------------------------------------------------------------------------
// Key-only Sorts
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// Uses rank to sort one bit at a time: Sorts a block according
// to bits startbit -> nbits + startbit
//----------------------------------------------------------------------------
template<uint nbits, uint startbit>
__device__ void radixSortBlockKeysOnly(uint4 &key)
{
    extern __shared__ uint sMem1[];

    for(uint shift = startbit; shift < (startbit + nbits); ++shift)
    {        
        uint4 lsb;
        lsb.x = !((key.x >> shift) & 0x1);
        lsb.y = !((key.y >> shift) & 0x1);
        lsb.z = !((key.z >> shift) & 0x1);
        lsb.w = !((key.w >> shift) & 0x1);

        uint4 r = rank4<256>(lsb);

#if 1
        // This arithmetic strides the ranks across 4 CTA_SIZE regions
        sMem1[(r.x & 3) * NewRadixSort::CTA_SIZE + (r.x >> 2)] = key.x;
        sMem1[(r.y & 3) * NewRadixSort::CTA_SIZE + (r.y >> 2)] = key.y;
        sMem1[(r.z & 3) * NewRadixSort::CTA_SIZE + (r.z >> 2)] = key.z;
        sMem1[(r.w & 3) * NewRadixSort::CTA_SIZE + (r.w >> 2)] = key.w;
        __syncthreads();

        // The above allows us to read without 4-way bank conflicts:
        key.x = sMem1[threadIdx.x];
        key.y = sMem1[threadIdx.x +     NewRadixSort::CTA_SIZE];
        key.z = sMem1[threadIdx.x + 2 * NewRadixSort::CTA_SIZE];
        key.w = sMem1[threadIdx.x + 3 * NewRadixSort::CTA_SIZE];
#else
        sMem1[r.x] = key.x;
        sMem1[r.y] = key.y;
        sMem1[r.z] = key.z;
        sMem1[r.w] = key.w;
        __syncthreads();

        // This access has 4-way bank conflicts
        key = sMem[threadIdx.x];
#endif

        __syncthreads();
    }
}

//----------------------------------------------------------------------------
//
// radixSortBlocks sorts each block of data independently in shared
// memory.  
//
// Done in two separate stages.  This stage calls radixSortBlock on each block 
// independently, sorting on the basis of bits (startbit) -> (startbit + nbits)
//----------------------------------------------------------------------------
template<uint nbits, uint startbit, bool fullBlocks, bool flip>
__global__ void radixSortBlocksKeysOnly(uint4* keysOut, uint4* keysIn, uint numElements, uint startBlock)
{
    extern __shared__ uint4 sMem[];

    uint4 key;

    const uint blockId = blockIdx.x + startBlock;
    const uint i = blockId * blockDim.x + threadIdx.x;
    const uint idx = i << 2;

    // handle non-full last block if array is not multiple of 1024 numElements
    if (!fullBlocks && idx+3 >= numElements)
    {
        if (idx >= numElements)
        {
            key   = make_uint4(UINT_MAX, UINT_MAX, UINT_MAX, UINT_MAX);
        }
        else
        {
            // for non-full block, we handle uint1 values instead of uint4
            uint *keys1    = (uint*)keysIn;

            key.x = (idx   < numElements) ? floatFlip<flip>(keys1[idx])   : UINT_MAX;
            key.y = (idx+1 < numElements) ? floatFlip<flip>(keys1[idx+1]) : UINT_MAX;
            key.z = (idx+2 < numElements) ? floatFlip<flip>(keys1[idx+2]) : UINT_MAX;
            key.w = UINT_MAX;
        }
    }
    else
    {
        key = keysIn[i];
        if (flip)
        {
            key.x = floatFlip<flip>(key.x);
            key.y = floatFlip<flip>(key.y);
            key.z = floatFlip<flip>(key.z);
            key.w = floatFlip<flip>(key.w);
        }            
    }
    __syncthreads();
    radixSortBlockKeysOnly<nbits, startbit>(key);
    __syncthreads();

    // handle non-full last block if array is not multiple of 1024 numElements
    if(!fullBlocks && idx+3 >= numElements)
    {
        if (idx < numElements) 
        {
            // for non-full block, we handle uint1 values instead of uint4
            uint *keys1   = (uint*)keysOut;

            keys1[idx]   = key.x;

            if (idx + 1 < numElements)
            {
                keys1[idx + 1]   = key.y;

                if (idx + 2 < numElements)
                {
                    keys1[idx + 2]   = key.z;
                }
            }
        }
    }
    else
    {
        keysOut[i]   = key;
    }
}

//----------------------------------------------------------------------------
// reorderData shuffles data in the array globally after the radix offsets 
// have been found. Depends on NewRadixSort::CTA_SIZE being 16 * number of radices 
// (i.e. 16 * 2^nbits).
// 
// This is quite fast and fully coalesces memory writes, albeit by doing extra 
// (potentially wasted) work allocating threads to portions of memory that are 
// not written out. Significantly faster than the generic approach on G80.
//----------------------------------------------------------------------------
template<uint startbit, bool fullBlocks, bool manualCoalesce, bool unflip>
__global__ void reorderDataKeysOnly(uint  *outKeys, 
                            uint2 *keys, 
                            uint  *blockOffsets, 
                            uint  *offsets, 
                            uint  *sizes, 
                            uint   numElements,
                            uint   totalBlocks,
                            uint   startBlock)
{
    __shared__ uint2 sKeys2[NewRadixSort::CTA_SIZE];
    __shared__ uint sOffsets[16];
    __shared__ uint sBlockOffsets[16];
    
    uint *sKeys1   = (uint*)sKeys2; 

    const uint blockId = blockIdx.x + startBlock;
    const uint i = blockId * blockDim.x + threadIdx.x;

    // handle non-full last block if array is not multiple of 1024 numElements
    if(!fullBlocks && (((i + 1) << 1) > numElements))
    {
        uint *keys1   = (uint*)keys;
        uint j = i << 1; 

        sKeys1[threadIdx.x << 1]   = (j < numElements) ? keys1[j]   : UINT_MAX; 
        j++; 
        sKeys1[(threadIdx.x << 1) + 1]   = (j < numElements) ? keys1[j]   : UINT_MAX; 
    }
    else
    {
        sKeys2[threadIdx.x]   = keys[i];
    }

    if (!manualCoalesce)
    {
        if(threadIdx.x < 16)  
        {
            sOffsets[threadIdx.x]      = offsets[threadIdx.x * totalBlocks + blockId];
            sBlockOffsets[threadIdx.x] = blockOffsets[blockId * 16 + threadIdx.x];
        }
        __syncthreads();

        uint radix = (sKeys1[threadIdx.x] >> startbit) & 0xF;
	    uint globalOffset = sOffsets[radix] + threadIdx.x - sBlockOffsets[radix];
	    
        if (fullBlocks || globalOffset < numElements)
        {
	        outKeys[globalOffset]   = floatUnflip<unflip>(sKeys1[threadIdx.x]);
        }

        radix = (sKeys1[threadIdx.x + NewRadixSort::CTA_SIZE] >> startbit) & 0xF;
	    globalOffset = sOffsets[radix] + threadIdx.x + NewRadixSort::CTA_SIZE - sBlockOffsets[radix];
	    
        if (fullBlocks || globalOffset < numElements)
        {
	        outKeys[globalOffset]   = floatUnflip<unflip>(sKeys1[threadIdx.x + NewRadixSort::CTA_SIZE]);
        }
    }
    else
    {
        __shared__ uint sSizes[16];

        if(threadIdx.x < 16)  
        {
            sOffsets[threadIdx.x]      = offsets[threadIdx.x * totalBlocks + blockId];
            sBlockOffsets[threadIdx.x] = blockOffsets[blockId * 16 + threadIdx.x];
            sSizes[threadIdx.x]        = sizes[threadIdx.x * totalBlocks + blockId];
        }
        __syncthreads();

        // 1 half-warp is responsible for writing out all values for 1 radix. 
        // Loops if there are more than 16 values to be written out. 
        // All start indices are rounded down to the nearest multiple of 16, and
        // all end indices are rounded up to the nearest multiple of 16.
        // Thus it can do extra work if the start and end indices are not multiples of 16
        // This is bounded by a factor of 2 (it can do 2X more work at most).

        const uint halfWarpID     = threadIdx.x >> 4;

        const uint halfWarpOffset = threadIdx.x & 0xF;
        const uint leadingInvalid = sOffsets[halfWarpID] & 0xF;

        uint startPos = sOffsets[halfWarpID] & 0xFFFFFFF0;
        uint endPos   = (sOffsets[halfWarpID] + sSizes[halfWarpID]) + 15 - 
                        ((sOffsets[halfWarpID] + sSizes[halfWarpID] - 1) & 0xF);
        uint numIterations = endPos - startPos;

        uint outOffset = startPos + halfWarpOffset;
        uint inOffset  = sBlockOffsets[halfWarpID] - leadingInvalid + halfWarpOffset;

        for(uint j = 0; j < numIterations; j += 16, outOffset += 16, inOffset += 16)
        {       
            if( (outOffset >= sOffsets[halfWarpID]) && 
                (inOffset - sBlockOffsets[halfWarpID] < sSizes[halfWarpID])) 
            {
                if(blockId < totalBlocks - 1 || outOffset < numElements) 
                {
                    outKeys[outOffset] = floatUnflip<unflip>(sKeys1[inOffset]);
                }
            }       
        }
    }
}

//----------------------------------------------------------------------------
// Perform one step of the radix sort.  Sorts by nbits key bits per step, 
// starting at startbit.
//----------------------------------------------------------------------------
template<uint nbits, uint startbit, bool flip, bool unflip>
void radixSortStepKeysOnly(uint *keys, 
                   uint *tempKeys, 
                   uint *counters, 
                   uint *countersSum, 
                   uint *blockOffsets, 
                   CUDPPHandle scanPlan,
                   uint numElements, 
                   bool manualCoalesce)
{
    const uint eltsPerBlock = NewRadixSort::CTA_SIZE * 4;
    const uint eltsPerBlock2 = NewRadixSort::CTA_SIZE * 2;

    bool fullBlocks = ((numElements % eltsPerBlock) == 0);
    uint numBlocks = (fullBlocks) ? 
        (numElements / eltsPerBlock) : 
        (numElements / eltsPerBlock + 1);
    uint numBlocks2 = ((numElements % eltsPerBlock2) == 0) ?
        (numElements / eltsPerBlock2) : 
        (numElements / eltsPerBlock2 + 1);

    const uint max1DBlocks = 65535;

    
    for (uint block = 0; block < numBlocks; block += max1DBlocks)
    {
        uint blocks   = min(max1DBlocks, numBlocks - block);
        
        if (blocks < max1DBlocks && !fullBlocks)
        {
            radixSortBlocksKeysOnly<nbits, startbit, false, flip>
                <<<blocks, NewRadixSort::CTA_SIZE, 4 * NewRadixSort::CTA_SIZE * sizeof(uint)>>>
                    ((uint4*)tempKeys, (uint4*)keys, numElements, block);
        }
        else
        {
            radixSortBlocksKeysOnly<nbits, startbit, true, flip>
                <<<blocks, NewRadixSort::CTA_SIZE, 4 * NewRadixSort::CTA_SIZE * sizeof(uint)>>>
                    ((uint4*)tempKeys, (uint4*)keys, numElements, block);
        }
    }

    for (uint block = 0; block < numBlocks2; block += max1DBlocks)
    {
        uint blocks   = min(max1DBlocks, numBlocks2 - block);

        if (blocks < max1DBlocks && !fullBlocks)
        {
            findRadixOffsets<startbit, false>
                <<<blocks, NewRadixSort::CTA_SIZE, 3 * NewRadixSort::CTA_SIZE * sizeof(uint)>>>
                    ((uint2*)tempKeys, counters, blockOffsets, numElements, numBlocks2, block);
        }
        else
        {
            findRadixOffsets<startbit, true>
                <<<blocks, NewRadixSort::CTA_SIZE, 3 * NewRadixSort::CTA_SIZE * sizeof(uint)>>>
                    ((uint2*)tempKeys, counters, blockOffsets, numElements, numBlocks2, block);
        }
    }

    cudppScan(scanPlan, countersSum, counters, 16*numBlocks2);

    for (uint block = 0; block < numBlocks2; block += max1DBlocks)
    {
        uint blocks   = min(max1DBlocks, numBlocks2 - block);
        
        if (blocks < max1DBlocks && !fullBlocks)
        {
            if (manualCoalesce)
            {
                reorderDataKeysOnly<startbit, false, true, unflip><<<blocks, NewRadixSort::CTA_SIZE>>>
                    (keys, (uint2*)tempKeys, blockOffsets, countersSum, counters, 
                     numElements, numBlocks2, block);
            }
            else
            {
                reorderDataKeysOnly<startbit, false, false, unflip><<<blocks, NewRadixSort::CTA_SIZE>>>
                    (keys, (uint2*)tempKeys, blockOffsets, countersSum, counters, 
                     numElements, numBlocks2, block);
            }
        }
        else
        {
            if (manualCoalesce)
            {
                reorderDataKeysOnly<startbit, true, true, unflip><<<blocks, NewRadixSort::CTA_SIZE>>>
                    (keys, (uint2*)tempKeys, blockOffsets, countersSum, counters, numElements, numBlocks2, block);
            }
            else
            {
                reorderDataKeysOnly<startbit, true, false, unflip><<<blocks, NewRadixSort::CTA_SIZE>>>
                    (keys, (uint2*)tempKeys, blockOffsets, countersSum, counters, numElements, numBlocks2, block);
            }
        }
    }    

    checkCudaError("radixSortStepKeysOnly");
}

//----------------------------------------------------------------------------
// Optimization for sorts of fewer than 4 * CTA_SIZE elements
//----------------------------------------------------------------------------
template <bool flip>
void radixSortSingleBlockKeysOnly(uint *keys, 
                                  uint numElements)
{
    bool fullBlocks = (numElements % (NewRadixSort::CTA_SIZE * 4) == 0);
    if (fullBlocks)
    {
        radixSortBlocksKeysOnly<32, 0, true, flip>
            <<<1, NewRadixSort::CTA_SIZE, 4 * NewRadixSort::CTA_SIZE * sizeof(uint)>>>
                ((uint4*)keys, (uint4*)keys, numElements, 0);
    }
    else
    {
        radixSortBlocksKeysOnly<32, 0, false, flip>
            <<<1, NewRadixSort::CTA_SIZE, 4 * NewRadixSort::CTA_SIZE * sizeof(uint)>>>
                ((uint4*)keys, (uint4*)keys, numElements, 0);
    }

    if (flip)
        unflipFloats<<<1, NewRadixSort::CTA_SIZE>>>(keys, numElements);


    checkCudaError("radixSortSingleBlock");
}

//----------------------------------------------------------------------------
// Optimization for sorts of WARP_SIZE or fewer elements
//----------------------------------------------------------------------------
template <bool flip>
__global__ 
void radixSortSingleWarpKeysOnly(uint *keys, 
                                 uint numElements)
{
    __shared__ uint sKeys[NewRadixSort::WARP_SIZE];
    __shared__ uint sFlags[NewRadixSort::WARP_SIZE];

    sKeys[threadIdx.x]   = floatFlip<flip>(keys[threadIdx.x]);
    
    __SYNC // emulation only

    for(uint i = 1; i < numElements; i++)
    {
        uint key_i = sKeys[i];
        
        sFlags[threadIdx.x] = 0;
        
        if( (threadIdx.x < i) && (sKeys[threadIdx.x] > key_i) ) 
        {
            uint temp = sKeys[threadIdx.x];
            sFlags[threadIdx.x] = 1;
            sKeys[threadIdx.x + 1] = temp;
            sFlags[threadIdx.x + 1] = 0;
        }
        if(sFlags[threadIdx.x] == 1 )
        {
            sKeys[threadIdx.x] = key_i;
        }

        __SYNC // emulation only

    }
    keys[threadIdx.x]   = floatUnflip<flip>(sKeys[threadIdx.x]);
}

//----------------------------------------------------------------------------
// Main radix sort function.  Sorts in place in the keys and values arrays,
// but uses the other device arrays as temporary storage.  All pointer 
// parameters are device pointers.  Uses cudppScan() for the prefix sum of
// radix counters.
//----------------------------------------------------------------------------
extern "C"
void radixSortKeysOnly(uint *keys, 
                       uint *tempKeys, 
                       uint *counters,
                       uint *countersSum,
                       uint *blockOffsets,
                       CUDPPHandle scanPlan,
                       uint numElements, 
                       uint keyBits,
                       bool manualCoalesce = true,
                       bool flipBits = false)
{
    if(numElements <= NewRadixSort::WARP_SIZE)
    {
        if (flipBits)
            radixSortSingleWarpKeysOnly<true><<<1, numElements>>>(keys, numElements);
        else
            radixSortSingleWarpKeysOnly<false><<<1, numElements>>>(keys, numElements);
        checkCudaError("radixSortSingleWarp");
        return;
    }
    if(numElements <= NewRadixSort::CTA_SIZE * 4)
    {
        if (flipBits)
            radixSortSingleBlockKeysOnly<true>(keys, numElements);
        else
            radixSortSingleBlockKeysOnly<false>(keys, numElements);
        return;
    }

    // flip float bits on the first pass, unflip on the last pass
    if (flipBits) 
    {
            radixSortStepKeysOnly<4,  0, true, false>(keys, tempKeys, 
                                                      counters, countersSum, blockOffsets, 
                                                      scanPlan, numElements, manualCoalesce);
    }
    else
    {
            radixSortStepKeysOnly<4,  0, false, false>(keys, tempKeys, 
                                                       counters, countersSum, blockOffsets, 
                                                       scanPlan, numElements, manualCoalesce);
    }

    if (keyBits > 4)
    {
            radixSortStepKeysOnly<4,  4, false, false>(keys, tempKeys, 
                                                       counters, countersSum, blockOffsets, 
                                                       scanPlan, numElements, manualCoalesce);
    }
    if (keyBits > 8)
    {
            radixSortStepKeysOnly<4,  8, false, false>(keys, tempKeys, 
                                                       counters, countersSum, blockOffsets, 
                                                       scanPlan, numElements, manualCoalesce);
    }
    if (keyBits > 12)
    {
            radixSortStepKeysOnly<4, 12, false, false>(keys, tempKeys, 
                                                       counters, countersSum, blockOffsets, 
                                                       scanPlan, numElements, manualCoalesce);
    }
    if (keyBits > 16)
    {
            radixSortStepKeysOnly<4, 16, false, false>(keys, tempKeys, 
                                                       counters, countersSum, blockOffsets, 
                                                       scanPlan, numElements, manualCoalesce);
    }
    if (keyBits > 20)
    {
            radixSortStepKeysOnly<4, 20, false, false>(keys, tempKeys, 
                                                       counters, countersSum, blockOffsets, 
                                                       scanPlan, numElements, manualCoalesce);
    }
    if (keyBits > 24)
    {
            radixSortStepKeysOnly<4, 24, false, false>(keys, tempKeys, 
                                                       counters, countersSum, blockOffsets, 
                                                       scanPlan, numElements, manualCoalesce);
    }
    if (keyBits > 28)
    {
        if (flipBits) // last pass
        {
            radixSortStepKeysOnly<4, 28, false, true>(keys, tempKeys, 
                                                      counters, countersSum, blockOffsets, 
                                                      scanPlan, numElements, manualCoalesce);
        }
        else
        {
            radixSortStepKeysOnly<4, 28, false, false>(keys, tempKeys, 
                                                       counters, countersSum, blockOffsets, 
                                                       scanPlan, numElements, manualCoalesce);
        }
    }

    checkCudaError("radixSortKeysOnly");
}

extern "C"
void radixSortFloatKeysOnly(float *keys, 
                            float *tempKeys, 
                            uint  *counters,
                            uint  *countersSum,
                            uint  *blockOffsets,
                            CUDPPHandle scanPlan,
                            uint  numElements, 
                            uint  keyBits,
                            bool manualCoalesce,
                            bool  negativeKeys)
{
    radixSortKeysOnly((uint*)keys, (uint*)tempKeys, counters, countersSum, blockOffsets, 
                       scanPlan, numElements, keyBits, manualCoalesce, negativeKeys);
    checkCudaError("radixSortFloatKeys");
}
