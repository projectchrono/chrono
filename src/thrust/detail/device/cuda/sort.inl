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


/*! \file sort.inl
 *  \brief Inline file for sort.h
 */

#include <thrust/iterator/iterator_traits.h>
#include <thrust/detail/type_traits.h>

#include <thrust/detail/device/cuda/detail/stable_merge_sort.h>
#include <thrust/detail/device/cuda/detail/stable_radix_sort.h>

#include <thrust/gather.h>
#include <thrust/sequence.h>
#include <thrust/iterator/iterator_traits.h>
#include <thrust/detail/raw_buffer.h>

/*
 *  This file implements the following dispatch procedure for cuda::stable_sort()
 *  and cuda::stable_sort_by_key().  All iterators are assumed to be "trivial
 *  iterators" (i.e. pointer wrappers).  The first level inspects the KeyType
 *  and StrictWeakOrdering to determines whether Radix Sort may be applied.
 *  The second level inspects the KeyType to determine whether keys should 
 *  be sorted indirectly (i.e. sorting references to keys instead of keys 
 *  themselves). The third level inspects the ValueType to determine whether 
 *  the values should be sorted indirectly, again using references instead of
 *  the values themselves.
 *
 *  The second and third levels convert one sorting problem to another.
 *  The second level converts a sort on T to a sort on integers that index
 *  into an array of type T.  Similarly, the third level converts a (key,value) 
 *  sort into a (key,index) sort where the indices  record the permutation 
 *  used to sort the keys.  The permuted indices are then used to reorder the
 *  values.  In either case, the transformation converts an ill-suited problem
 *  (i.e. sorting with large keys or large values) into a problem more amenable
 *  to the underlying sorting algorithms.
 * 
 *   Summary of the stable_sort() dispatch procedure:
 *       Level 1:
 *          if is_primitive<KeyType> && is_equal< StrictWeakOrdering, less<KeyType> > 
 *              stable_radix_sort()
 *          else
 *              Level2 stable_merge_sort()
 *
 *       Level2:
 *          if sizeof(KeyType) > 16
 *               add indirection to keys
 *               stable_merge_sort()
 *               permute keys
 *          else
 *               stable_merge_sort()
 *     
 *   Summary of the stable_sort_by_key() dispatch procedure:
 *       Level 1:
 *          if is_primitive<KeyType> && is_equal< StrictWeakOrdering, less<KeyType> > 
 *              stable_radix_sort_by_key()
 *          else
 *              Level2 stable_merge_sort_by_key()
 *
 *       Level2:
 *          if sizeof(KeyType) > 16
 *               add indirection to keys
 *               Level3 stable_merge_sort_by_key()
 *               permute keys
 *          else
 *               Level3 stable_merge_sort_by_key()
 *
 *       Level3:
 *          if sizeof(ValueType) != 4
 *              add indirection to values
 *              stable_merge_sort_by_key()
 *              permute values
 *          else 
 *              stable_merge_sort_by_key()
 */


namespace thrust
{
namespace detail
{
namespace device
{
namespace cuda
{

namespace third_dispatch
{
    // thrid level of the dispatch decision tree
    
    template<typename RandomAccessIterator1,
             typename RandomAccessIterator2,
             typename StrictWeakOrdering>
    void stable_merge_sort_by_key(RandomAccessIterator1 keys_first,
                                  RandomAccessIterator1 keys_last,
                                  RandomAccessIterator2 values_first,
                                  StrictWeakOrdering comp,
                                  thrust::detail::true_type)
    {
        // sizeof(ValueType) != 4, use indirection and permute values
        typedef typename thrust::iterator_traits<RandomAccessIterator2>::value_type ValueType;
        thrust::detail::raw_cuda_device_buffer<unsigned int> permutation(keys_last - keys_first);
        thrust::sequence(permutation.begin(), permutation.end());
    
        thrust::detail::device::cuda::detail::stable_merge_sort_by_key
            (keys_first, keys_last, permutation.begin(), comp);
   
        RandomAccessIterator2 values_last = values_first + (keys_last - keys_first);
        thrust::detail::raw_cuda_device_buffer<ValueType> temp(values_first, values_last);
        thrust::next::gather(permutation.begin(), permutation.end(), temp.begin(), values_first);
    }
    
    template<typename RandomAccessIterator1,
             typename RandomAccessIterator2,
             typename StrictWeakOrdering>
    void stable_merge_sort_by_key(RandomAccessIterator1 keys_first,
                                  RandomAccessIterator1 keys_last,
                                  RandomAccessIterator2 values_first,
                                  StrictWeakOrdering comp,
                                  thrust::detail::false_type)
    {
        // sizeof(ValueType) == 4, sort values directly
        thrust::detail::device::cuda::detail::stable_merge_sort_by_key
            (keys_first, keys_last, values_first, comp);
    }

} // end namespace third_dispatch

namespace second_dispatch
{
    // second level of the dispatch decision tree

    // add one level of indirection to the StrictWeakOrdering comp
    template <typename RandomAccessIterator, typename StrictWeakOrdering> 
    struct indirect_comp
    {
        RandomAccessIterator first;
        StrictWeakOrdering   comp;
    
        indirect_comp(RandomAccessIterator first, StrictWeakOrdering comp)
            : first(first), comp(comp) {}
    
        template <typename IndexKeyTypeype>
        __host__ __device__
        bool operator()(IndexKeyTypeype a, IndexKeyTypeype b)
        {
            return comp(thrust::detail::device::dereference(first, a),
                        thrust::detail::device::dereference(first, b));
        }    
    };

    template<typename RandomAccessIterator,
             typename StrictWeakOrdering>
      void stable_merge_sort(RandomAccessIterator first,
                             RandomAccessIterator last,
                             StrictWeakOrdering comp,
                             thrust::detail::true_type)
    {
        // sizeof(KeyType) > 16, sort keys indirectly
        typedef typename thrust::iterator_traits<RandomAccessIterator>::value_type KeyType;
        thrust::detail::raw_cuda_device_buffer<unsigned int> permutation(last - first);
        thrust::sequence(permutation.begin(), permutation.end());
    
        thrust::detail::device::cuda::detail::stable_merge_sort
            (permutation.begin(), permutation.end(), indirect_comp<RandomAccessIterator,StrictWeakOrdering>(first, comp));
    
        thrust::detail::raw_cuda_device_buffer<KeyType> temp(first, last);
        thrust::next::gather(permutation.begin(), permutation.end(), temp.begin(), first);
    }
    
    template<typename RandomAccessIterator,
             typename StrictWeakOrdering>
      void stable_merge_sort(RandomAccessIterator first,
                             RandomAccessIterator last,
                             StrictWeakOrdering comp,
                             thrust::detail::false_type)
    {
        // sizeof(KeyType) <= 16, sort keys directly
        thrust::detail::device::cuda::detail::stable_merge_sort(first, last, comp);
    }
    
    template<typename RandomAccessIterator1,
             typename RandomAccessIterator2,
             typename StrictWeakOrdering>
    void stable_merge_sort_by_key(RandomAccessIterator1 keys_first,
                                  RandomAccessIterator1 keys_last,
                                  RandomAccessIterator2 values_first,
                                  StrictWeakOrdering comp,
                                  thrust::detail::true_type)
    {
        // sizeof(KeyType) > 16, sort keys indirectly
        typedef typename thrust::iterator_traits<RandomAccessIterator1>::value_type KeyType;
        thrust::detail::raw_cuda_device_buffer<unsigned int> permutation(keys_last - keys_first);
        thrust::sequence(permutation.begin(), permutation.end());
    
        // decide whether to sort values indirectly
        typedef typename thrust::iterator_traits<RandomAccessIterator2>::value_type ValueType;
        static const bool sort_values_indirectly = sizeof(ValueType) != 4;
        thrust::detail::device::cuda::third_dispatch::stable_merge_sort_by_key
            (permutation.begin(), permutation.end(), values_first, indirect_comp<RandomAccessIterator1,StrictWeakOrdering>(keys_first, comp),
             thrust::detail::integral_constant<bool, sort_values_indirectly>());
    
        thrust::detail::raw_cuda_device_buffer<KeyType> temp(keys_first, keys_last);
        thrust::next::gather(permutation.begin(), permutation.end(), temp.begin(), keys_first);
    }
    
    template<typename RandomAccessIterator1,
             typename RandomAccessIterator2,
             typename StrictWeakOrdering>
    void stable_merge_sort_by_key(RandomAccessIterator1 keys_first,
                                  RandomAccessIterator1 keys_last,
                                  RandomAccessIterator2 values_first,
                                  StrictWeakOrdering comp,
                                  thrust::detail::false_type)
    {
        // sizeof(KeyType) <= 16, sort keys directly
        
        // decide whether to sort values indirectly
        typedef typename thrust::iterator_traits<RandomAccessIterator2>::value_type ValueType;
        static const bool sort_values_indirectly = sizeof(ValueType) != 4;

        // XXX WAR nvcc 3.0 unused variable warning
        (void) sort_values_indirectly;

        thrust::detail::device::cuda::third_dispatch::stable_merge_sort_by_key
            (keys_first, keys_last, values_first, comp,
             thrust::detail::integral_constant<bool, sort_values_indirectly>());
    }

} // end namespace second_dispatch


namespace first_dispatch
{
    // first level of the dispatch decision tree

    template<typename RandomAccessIterator,
             typename StrictWeakOrdering>
      void stable_sort(RandomAccessIterator first,
                       RandomAccessIterator last,
                       StrictWeakOrdering comp,
                       thrust::detail::true_type)
    {
        // CUDA path for thrust::stable_sort with primitive keys
        // (e.g. int, float, short, etc.) and the default less<T> comparison
        // method is implemented with stable_radix_sort_by_key
        thrust::detail::device::cuda::detail::stable_radix_sort(first, last);
    }
    
    template<typename RandomAccessIterator,
             typename StrictWeakOrdering>
      void stable_sort(RandomAccessIterator first,
                       RandomAccessIterator last,
                       StrictWeakOrdering comp,
                       thrust::detail::false_type)
    {
        // decide whether to sort keys indirectly
        typedef typename thrust::iterator_traits<RandomAccessIterator>::value_type KeyType;
        static const bool sort_keys_indirectly = sizeof(KeyType) > 16;  

        // XXX WAR nvcc 3.0 unused variable warning
        (void) sort_keys_indirectly;
        
        // XXX  magic constant determined by limited empirical testing
        // TODO more extensive tuning, consider vector types (e.g. int4)
    
        // device path for thrust::stable_sort with general keys 
        // and comparison methods is implemented with stable_merge_sort
        thrust::detail::device::cuda::second_dispatch::stable_merge_sort
            (first, last, comp,
                thrust::detail::integral_constant<bool, sort_keys_indirectly>());
    }
    
    template<typename RandomAccessIterator1,
             typename RandomAccessIterator2,
             typename StrictWeakOrdering>
      void stable_sort_by_key(RandomAccessIterator1 keys_first,
                              RandomAccessIterator1 keys_last,
                              RandomAccessIterator2 values_first,
                              StrictWeakOrdering comp,
                              thrust::detail::true_type)
    {
        // device path for thrust::stable_sort_by_key with primitive keys
        // (e.g. int, float, short, etc.) and the default less<T> comparison
        // method is implemented with stable_radix_sort_by_key
        thrust::detail::device::cuda::detail::stable_radix_sort_by_key(keys_first, keys_last, values_first);
    }
    
    template<typename RandomAccessIterator1,
             typename RandomAccessIterator2,
             typename StrictWeakOrdering>
      void stable_sort_by_key(RandomAccessIterator1 keys_first,
                              RandomAccessIterator1 keys_last,
                              RandomAccessIterator2 values_first,
                              StrictWeakOrdering comp,
                              thrust::detail::false_type)
    {
        // decide whether to sort keys indirectly
        typedef typename thrust::iterator_traits<RandomAccessIterator1>::value_type KeyType;
        static const bool sort_keys_indirectly = sizeof(KeyType) > 16;  

        // XXX WAR nvcc 3.0 unused variable warning
        (void) sort_keys_indirectly;
        
        // XXX  magic constant determined by limited empirical testing
        // TODO more extensive tuning, consider vector types (e.g. int4)
    
        // device path for thrust::stable_sort with general keys 
        // and comparison methods is implemented with stable_merge_sort
        thrust::detail::device::cuda::second_dispatch::stable_merge_sort_by_key
            (keys_first, keys_last, values_first, comp,
                thrust::detail::integral_constant<bool, sort_keys_indirectly>());
    }

} // end namespace first_dispatch


template<typename RandomAccessIterator,
         typename StrictWeakOrdering>
  void stable_sort(RandomAccessIterator first,
                   RandomAccessIterator last,
                   StrictWeakOrdering comp)
{
    // we're attempting to launch a kernel, assert we're compiling with nvcc
    // ========================================================================
    // X Note to the user: If you've found this line due to a compiler error, X
    // X you need to compile your code using nvcc, rather than g++ or cl.exe  X
    // ========================================================================
    THRUST_STATIC_ASSERT( (depend_on_instantiation<RandomAccessIterator, THRUST_DEVICE_COMPILER == THRUST_DEVICE_COMPILER_NVCC>::value) );

    // dispatch on whether we can use radix_sort
    typedef typename thrust::iterator_traits<RandomAccessIterator>::value_type KeyType;
    static const bool use_radix_sort = thrust::detail::is_arithmetic<KeyType>::value &&
                                       thrust::detail::is_same<StrictWeakOrdering, typename thrust::less<KeyType> >::value;

    // XXX WAR nvcc 3.0 unused variable warning
    (void) use_radix_sort;

    first_dispatch::stable_sort(first, last, comp,
            thrust::detail::integral_constant<bool, use_radix_sort>());
}

template<typename RandomAccessIterator1,
         typename RandomAccessIterator2,
         typename StrictWeakOrdering>
  void stable_sort_by_key(RandomAccessIterator1 keys_first,
                          RandomAccessIterator1 keys_last,
                          RandomAccessIterator2 values_first,
                          StrictWeakOrdering comp)
{
    // we're attempting to launch a kernel, assert we're compiling with nvcc
    // ========================================================================
    // X Note to the user: If you've found this line due to a compiler error, X
    // X you need to compile your code using nvcc, rather than g++ or cl.exe  X
    // ========================================================================
    THRUST_STATIC_ASSERT( (depend_on_instantiation<RandomAccessIterator1, THRUST_DEVICE_COMPILER == THRUST_DEVICE_COMPILER_NVCC>::value) );

    // dispatch on whether we can use radix_sort_by_key
    typedef typename thrust::iterator_traits<RandomAccessIterator1>::value_type KeyType;
    static const bool use_radix_sort = thrust::detail::is_arithmetic<KeyType>::value &&
                                       thrust::detail::is_same<StrictWeakOrdering, typename thrust::less<KeyType> >::value;

    // XXX WAR nvcc 3.0 unused variable warning
    (void) use_radix_sort;
    
    first_dispatch::stable_sort_by_key(keys_first, keys_last, values_first, comp,
            thrust::detail::integral_constant<bool, use_radix_sort>());
}

} // end namespace cuda
} // end namespace device
} // end namespace detail
} // end namespace thrust

