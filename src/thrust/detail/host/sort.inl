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

#include <thrust/detail/trivial_sequence.h>

#include <thrust/detail/host/detail/stable_merge_sort.h>

#include <algorithm>

namespace thrust
{
namespace detail
{
namespace host
{

template<typename RandomAccessIterator,
         typename StrictWeakOrdering>
  void sort(RandomAccessIterator first,
            RandomAccessIterator last,
            StrictWeakOrdering comp)
{
    // ensure sequence has trivial iterators
    thrust::detail::trivial_sequence<RandomAccessIterator> keys(first, last);
 
    // perform the sort
    std::sort(keys.begin(), keys.end(), comp);
  
    // copy results back, if necessary
    if(!thrust::detail::is_trivial_iterator<RandomAccessIterator>::value)
        thrust::copy(keys.begin(), keys.end(), first);
}

template<typename RandomAccessIterator,
         typename StrictWeakOrdering>
  void stable_sort(RandomAccessIterator first,
                   RandomAccessIterator last,
                   StrictWeakOrdering comp)
{
    // ensure sequence has trivial iterators
    thrust::detail::trivial_sequence<RandomAccessIterator> keys(first, last);

    // perform the sort
    std::stable_sort(keys.begin(), keys.end(), comp);
  
    // copy results back, if necessary
    if(!thrust::detail::is_trivial_iterator<RandomAccessIterator>::value)
        thrust::copy(keys.begin(), keys.end(), first);
}

template<typename RandomAccessIterator1,
         typename RandomAccessIterator2,
         typename StrictWeakOrdering>
  void sort_by_key(RandomAccessIterator1 keys_first,
                   RandomAccessIterator1 keys_last,
                   RandomAccessIterator2 values_first,
                   StrictWeakOrdering comp)
{
    // forward to stable_sort_by_key
    thrust::detail::host::stable_sort_by_key(keys_first, keys_last, values_first, comp);
}

template<typename RandomAccessIterator1,
         typename RandomAccessIterator2,
         typename StrictWeakOrdering>
  void stable_sort_by_key(RandomAccessIterator1 keys_first,
                          RandomAccessIterator1 keys_last,
                          RandomAccessIterator2 values_first,
                          StrictWeakOrdering comp)
{
    // ensure sequences have trivial iterators
    RandomAccessIterator2 values_last = values_first + (keys_last - keys_first);
    thrust::detail::trivial_sequence<RandomAccessIterator1> keys(keys_first, keys_last);
    thrust::detail::trivial_sequence<RandomAccessIterator2> values(values_first, values_last);

    // perform the sort
    thrust::detail::host::detail::stable_merge_sort_by_key(keys.begin(), keys.end(), values.begin(), comp);

    // copy results back, if necessary
    if(!thrust::detail::is_trivial_iterator<RandomAccessIterator1>::value)
        thrust::copy(keys.begin(), keys.end(), keys_first);
    if(!thrust::detail::is_trivial_iterator<RandomAccessIterator2>::value)
       thrust::copy(values.begin(), values.end(), values_first);
}

} // end namespace host
} // end namespace detail
} // end namespace thrust

