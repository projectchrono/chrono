/*
 *  Copyright 2008-2010 NVIDIA Corporation
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#include <thrust/detail/config.h>

// do not attempt to compile this file with any other compiler
#if THRUST_DEVICE_COMPILER == THRUST_DEVICE_COMPILER_NVCC

#include <thrust/copy.h>
#include <thrust/gather.h>
#include <thrust/sequence.h>
#include <thrust/iterator/iterator_traits.h>

#include <thrust/detail/raw_buffer.h>
#include <thrust/detail/type_traits.h>
#include <thrust/detail/util/align.h>

#if THRUST_HOST_COMPILER == THRUST_HOST_COMPILER_MSVC
// temporarily disable 'possible loss of data' warnings on MSVC
#pragma warning(push)
#pragma warning(disable : 4244 4267)
#endif

#include <thrust/detail/device/cuda/detail/b40c/radixsort_api.h>

namespace thrust
{
namespace detail
{
namespace device
{
namespace cuda
{
namespace detail
{

template<typename RandomAccessIterator>
void stable_radix_sort(RandomAccessIterator first,
                       RandomAccessIterator last)
{
    typedef typename thrust::iterator_value<RandomAccessIterator>::type K;
    
    unsigned int num_elements = last - first;

    // ensure data is properly aligned
    if (!thrust::detail::util::is_aligned(thrust::raw_pointer_cast(&*first), 2*sizeof(K)))
    {
        thrust::detail::raw_cuda_device_buffer<K> aligned_keys(first, last);
        stable_radix_sort(aligned_keys.begin(), aligned_keys.end());
        thrust::copy(aligned_keys.begin(), aligned_keys.end(), first);
        return;
    }
    
    thrust::detail::device::cuda::detail::b40c::RadixSortingEnactor<K> sorter(num_elements);
    thrust::detail::device::cuda::detail::b40c::RadixSortStorage<K>    storage;
    
    // allocate temporary buffers
    thrust::detail::raw_cuda_device_buffer<K>            temp_keys(num_elements);
    thrust::detail::raw_cuda_device_buffer<int>          temp_spine(sorter.SpineElements());
    thrust::detail::raw_cuda_device_buffer<bool>         temp_from_alt(2);

    // define storage
    storage.d_keys             = thrust::raw_pointer_cast(&*first);
    storage.d_alt_keys         = thrust::raw_pointer_cast(&temp_keys[0]);
    storage.d_spine            = thrust::raw_pointer_cast(&temp_spine[0]);
    storage.d_from_alt_storage = thrust::raw_pointer_cast(&temp_from_alt[0]);

    // perform the sort
    sorter.EnactSort(storage);
    
    // radix sort sometimes leaves results in the alternate buffers
    if (storage.using_alternate_storage)
    {
        thrust::copy(temp_keys.begin(), temp_keys.end(), first);
    }

    // temporary storage automatically freed
}

///////////////////////
// Key-Value Sorting //
///////////////////////

// sort values directly
template<typename RandomAccessIterator1,
         typename RandomAccessIterator2>
void stable_radix_sort_by_key(RandomAccessIterator1 first1,
                              RandomAccessIterator1 last1,
                              RandomAccessIterator2 first2,
                              thrust::detail::true_type)
{
    typedef typename thrust::iterator_value<RandomAccessIterator1>::type K;
    typedef typename thrust::iterator_value<RandomAccessIterator2>::type V;
    
    unsigned int num_elements = last1 - first1;

    // ensure data is properly aligned
    if (!thrust::detail::util::is_aligned(thrust::raw_pointer_cast(&*first1), 2*sizeof(K)))
    {
        thrust::detail::raw_cuda_device_buffer<K> aligned_keys(first1, last1);
        stable_radix_sort_by_key(aligned_keys.begin(), aligned_keys.end(), first2);
        thrust::copy(aligned_keys.begin(), aligned_keys.end(), first1);
        return;
    }
    if (!thrust::detail::util::is_aligned(thrust::raw_pointer_cast(&*first2), 2*sizeof(V)))
    {
        thrust::detail::raw_cuda_device_buffer<V> aligned_values(first2, first2 + num_elements);
        stable_radix_sort_by_key(first1, last1, aligned_values.begin());
        thrust::copy(aligned_values.begin(), aligned_values.end(), first2);
        return;
    }
   
    thrust::detail::device::cuda::detail::b40c::RadixSortingEnactor<K,V> sorter(num_elements);
    thrust::detail::device::cuda::detail::b40c::RadixSortStorage<K,V>    storage;
    
    // allocate temporary buffers
    thrust::detail::raw_cuda_device_buffer<K>            temp_keys(num_elements);
    thrust::detail::raw_cuda_device_buffer<V>            temp_values(num_elements);
    thrust::detail::raw_cuda_device_buffer<int>          temp_spine(sorter.SpineElements());
    thrust::detail::raw_cuda_device_buffer<bool>         temp_from_alt(2);

    // define storage
    storage.d_keys             = thrust::raw_pointer_cast(&*first1);
    storage.d_values           = thrust::raw_pointer_cast(&*first2);
    storage.d_alt_keys         = thrust::raw_pointer_cast(&temp_keys[0]);
    storage.d_alt_values       = thrust::raw_pointer_cast(&temp_values[0]);
    storage.d_spine            = thrust::raw_pointer_cast(&temp_spine[0]);
    storage.d_from_alt_storage = thrust::raw_pointer_cast(&temp_from_alt[0]);

    // perform the sort
    sorter.EnactSort(storage);
    
    // radix sort sometimes leaves results in the alternate buffers
    if (storage.using_alternate_storage)
    {
        thrust::copy(  temp_keys.begin(),   temp_keys.end(), first1);
        thrust::copy(temp_values.begin(), temp_values.end(), first2);
    }
    
    // temporary storage automatically freed
}


// sort values indirectly
template<typename RandomAccessIterator1,
         typename RandomAccessIterator2>
void stable_radix_sort_by_key(RandomAccessIterator1 first1,
                              RandomAccessIterator1 last1,
                              RandomAccessIterator2 first2,
                              thrust::detail::false_type)
{
    typedef typename thrust::iterator_value<RandomAccessIterator2>::type V;
    
    unsigned int num_elements = last1 - first1;

    // sort with integer values and then permute the real values accordingly
    thrust::detail::raw_cuda_device_buffer<unsigned int> permutation(num_elements);
    thrust::sequence(permutation.begin(), permutation.end());

    stable_radix_sort_by_key(first1, last1, permutation.begin());
    
    // copy values into temp vector and then permute
    thrust::detail::raw_cuda_device_buffer<V> temp_values(first2, first2 + num_elements);
   
    // permute values
    thrust::gather(permutation.begin(), permutation.end(),
                   temp_values.begin(),
                   first2);
}


template<typename RandomAccessIterator1,
         typename RandomAccessIterator2>
void stable_radix_sort_by_key(RandomAccessIterator1 first1,
                              RandomAccessIterator1 last1,
                              RandomAccessIterator2 first2)
{
    typedef typename thrust::iterator_value<RandomAccessIterator2>::type V;

    // decide how to handle values
    static const bool sort_values_directly = thrust::detail::is_trivial_iterator<RandomAccessIterator2>::value &&
                                             thrust::detail::is_arithmetic<V>::value &&
                                             sizeof(V) <= 8;    // TODO profile this

    // XXX WAR nvcc 3.0 unused variable warning
    (void) sort_values_directly;

    stable_radix_sort_by_key(first1, last1, first2, 
                             thrust::detail::integral_constant<bool, sort_values_directly>());
}

} // end namespace detail
} // end namespace cuda
} // end namespace device
} // end namespace detail
} // end namespace thrust


#if THRUST_HOST_COMPILER == THRUST_HOST_COMPILER_MSVC
// reenable 'possible loss of data' warnings
#pragma warning(pop)
#endif


#endif // THRUST_DEVICE_COMPILER == THRUST_DEVICE_COMPILER_NVCC

