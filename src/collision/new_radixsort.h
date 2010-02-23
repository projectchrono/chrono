/*
* Copyright 1993-2006 NVIDIA Corporation.  All rights reserved.
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
#ifndef   __RADIXSORT_H__
#define   __RADIXSORT_H__

#include "cuda_runtime_api.h"

#define CUDPP_STATIC_LIB
#include "cudpp/cudpp.h"

extern "C" void checkCudaError(const char *msg);

extern "C"
void radixSort(unsigned int *keys, 
               unsigned int *values, 
               unsigned int *tempKeys, 
               unsigned int *tempValues,
               unsigned int *counters,
               unsigned int *countersSum,
               unsigned int *blockOffsets,
               CUDPPHandle scanPlan,
               unsigned int numElements, 
               unsigned int keyBits,
               bool         manualCoalese,
               bool         flipBits);

extern "C"
void radixSortFloatKeys(float        *keys, 
                        unsigned int *values, 
                        float        *tempKeys, 
                        unsigned int *tempValues,
                        unsigned int *counters,
                        unsigned int *countersSum,
                        unsigned int *blockOffsets,
                        CUDPPHandle  scanPlan,
                        unsigned int numElements, 
                        unsigned int keyBits,
                        bool         manualCoalesce,
                        bool         negativeKeys);

extern "C"
void radixSortKeysOnly(unsigned int *keys, 
                       unsigned int *tempKeys, 
                       unsigned int *counters,
                       unsigned int *countersSum,
                       unsigned int *blockOffsets,
                       CUDPPHandle scanPlan,
                       unsigned int numElements, 
                       unsigned int keyBits,
                       bool         manualCoalesce,
                       bool         flipBits);

extern "C"
void radixSortFloatKeysOnly(float        *keys, 
                            float        *tempKeys, 
                            unsigned int *counters,
                            unsigned int *countersSum,
                            unsigned int *blockOffsets,
                            CUDPPHandle  scanPlan,
                            unsigned int numElements, 
                            unsigned int keyBits,
                            bool         manualCoalese,
                            bool         negativeKeys);

//-----------------------------------------------------------------------------
// CUDA Radix Sort class for arrays of unsigned integer keys and values
// 
// Usage: Invoke from host code.  Constructor takes the maximum number of 
// elements, M, to be sort.  Invoke the sort on an array of N <= M elements by 
// calling sort(), passing CUDA device pointers to an array of N keys and an 
// array of M values, the number of elements, and the number of bits to sort in 
// the keys (as a multiple of 4 bits). 
//
// initialize() (called by the constructor) allocates temporary storage for the 
// sort and the prefix sum that it uses.  Temporary storage is 
// (2*M + 3*8*M/CTA_SIZE) unsigned ints, with a default CTA_SIZE of 256 threads.  
// So for example, sorting 128K key/value pairs (1MB) requires 1MB+48KB of 
// temporary storage, in addition to the 1MB for the original arrays.
//
// Depends on cudpp.lib
// 
// (Note that the C++ portion of the implementation is entirely in this header
// simply due to the fact that if a RadixSort object is instantiated in host
// code in a .cu file compiled with nvcc, it fails to link if the code is in 
// an external file.)
//-----------------------------------------------------------------------------
class NewRadixSort
{
public: // methods
    
    //------------------------------------------------------------------------
    // Constructor
    // @param maxElements   Maximum number of elements to be sorted. 
    //                      
    // Allocates maxElements * (2 + 3*8/CTA_SIZE) unsigned ints of temp storage.
    //------------------------------------------------------------------------
    NewRadixSort(unsigned int maxElements, bool keysOnly = false)
      : mScanPlan(0),
        mNumElements(0),
        mTempKeys(0),
        mTempValues(0),
        mCounters(0),
        mCountersSum(0),
        mBlockOffsets(0)
    {
        // Allocate temporary storage
        initialize(maxElements, keysOnly);
    }

    //------------------------------------------------------------------------
    // Destructor
    //------------------------------------------------------------------------
    ~NewRadixSort()
    {
        finalize();
    }
    
    //------------------------------------------------------------------------
    // Sorts an input array of keys and values
    // 
    // @param keys        Array of keys for data to be sorted
    // @param values      Array of values to be sorted
    // @param numElements Number of elements to be sorted.  Must be <= 
    //                    maxElements passed to the constructor
    // @param keyBits     The number of bits in each key to use for ordering
    //------------------------------------------------------------------------
    void sort(unsigned int *keys, 
              unsigned int *values, 
              unsigned int  numElements,
              unsigned int  keyBits)
    {
        if (values == 0)
        {
            radixSortKeysOnly(keys, mTempKeys, 
                              mCounters, mCountersSum, mBlockOffsets,
                              mScanPlan, numElements, keyBits, manualCoalesce, false);
        }
        else
        {
            radixSort(keys, values, mTempKeys, mTempValues,
                      mCounters, mCountersSum, mBlockOffsets,
                      mScanPlan, numElements, keyBits, manualCoalesce, false);
        }
    }

    void sort(float        *keys, 
              unsigned int *values, 
              unsigned int  numElements,
              unsigned int  keyBits,
              bool          negativeKeys)
    {
        if (values == 0)
        {
            radixSortFloatKeysOnly(keys, (float*)mTempKeys, 
                                   mCounters, mCountersSum, mBlockOffsets,
                                   mScanPlan, numElements, keyBits, manualCoalesce, negativeKeys);
        }
        else
        {
            radixSortFloatKeys(keys, values, (float*)mTempKeys, mTempValues,
                               mCounters, mCountersSum, mBlockOffsets,
                               mScanPlan, numElements, keyBits, manualCoalesce, negativeKeys);
        }
    }

public: // constants
    static const unsigned int CTA_SIZE = 256;
    static const unsigned int WARP_SIZE = 32;

protected: // data

    bool          manualCoalesce;

    CUDPPHandle   mScanPlan;        // CUDPP plan handle for prefix sum
    
    unsigned int  mNumElements;     // Number of elements of temp storage allocated
    unsigned int *mTempKeys;        // Intermediate storage for keys
    unsigned int *mTempValues;      // Intermediate storage for values
    unsigned int *mCounters;        // Counter for each radix
    unsigned int *mCountersSum;     // Prefix sum of radix counters
    unsigned int *mBlockOffsets;    // Global offsets of each radix in each block

protected: // methods

    //------------------------------------------------------------------------
    // Initialization.  Allocates temporary storage and CUDPP scan plan.
    // @param numElements   Maximum number of elements to be sorted. 
    //                      
    // Allocates numElements * (2 + 3*8/CTA_SIZE) unsigned ints of temp storage.
    // Note, the scan plan allocates an additional (numElements * 8/CTA_SIZE)/512
    // elements of temp storage, but for a 1M element array to be sorted, this 
    // amounts to only 256 bytes extra.
    //------------------------------------------------------------------------
    void initialize(unsigned int numElements, bool keysOnly)
    {
        int deviceID = -1;
        if (cudaSuccess == cudaGetDevice(&deviceID))
        {
            cudaDeviceProp devprop;
            cudaGetDeviceProperties(&devprop, deviceID);
            // sm_12 and later devices don't need help with coalesce
            manualCoalesce = (devprop.major < 2 && devprop.minor < 2);
        }
        mNumElements = numElements;

        unsigned int numBlocks = ((numElements % (CTA_SIZE * 4)) == 0) ? 
            (numElements / (CTA_SIZE * 4)) : (numElements / (CTA_SIZE * 4) + 1);
        unsigned int numBlocks2 = ((numElements % (CTA_SIZE * 2)) == 0) ?
            (numElements / (CTA_SIZE * 2)) : (numElements / (CTA_SIZE * 2) + 1);

        // Initialize scan
        CUDPPConfiguration scanConfig;
        scanConfig.algorithm = CUDPP_SCAN;
        scanConfig.datatype  = CUDPP_UINT;
        scanConfig.op        = CUDPP_ADD;
        scanConfig.options   = CUDPP_OPTION_EXCLUSIVE | CUDPP_OPTION_FORWARD;
        cudppPlan(&mScanPlan, scanConfig, 16 * numBlocks2, 1, 0);

        cudaMalloc((void **)&mTempKeys,     numElements * sizeof(unsigned int));
        if (!keysOnly)
            cudaMalloc((void **)&mTempValues,   numElements * sizeof(unsigned int));
        cudaMalloc((void **)&mCounters,     WARP_SIZE * numBlocks * sizeof(unsigned int));
        cudaMalloc((void **)&mCountersSum,  WARP_SIZE * numBlocks * sizeof(unsigned int));
        cudaMalloc((void **)&mBlockOffsets, WARP_SIZE * numBlocks * sizeof(unsigned int));

        checkCudaError("NewRadixSort::initialize()");
    }

    //------------------------------------------------------------------------
    // Deallocate all temporary storage and destroy CUDPP scan plan.
    //------------------------------------------------------------------------
    void finalize()
    {
        cudaFree(mTempKeys);
        cudaFree(mTempValues);
        cudaFree(mCounters);
        cudaFree(mCountersSum);
        cudaFree(mBlockOffsets);
        mCounters = mCountersSum = mBlockOffsets = 0;

        checkCudaError("NewRadixSort::finalize()");

        cudppDestroyPlan(mScanPlan);
    }
};


#endif // __RADIXSORT_H__
