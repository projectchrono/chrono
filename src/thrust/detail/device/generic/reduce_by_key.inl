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


/*! \file unique.inl
 *  \brief Inline file for unique.h.
 */

#pragma once

#include <thrust/iterator/iterator_traits.h>
#include <thrust/transform.h>
#include <thrust/scatter.h>
#include <thrust/functional.h>
#include <thrust/iterator/zip_iterator.h>
#include <limits>

#include <thrust/detail/device/scan.h>
#include <thrust/detail/device/copy.h>
#include <thrust/detail/raw_buffer.h>

namespace thrust
{
namespace detail
{
namespace device
{
namespace generic
{
namespace detail
{

template <typename ValueType, typename TailFlagType, typename AssociativeOperator>
struct reduce_by_key_functor
{
    AssociativeOperator binary_op;

    typedef typename thrust::tuple<ValueType, TailFlagType> result_type;

    __host__ __device__
    reduce_by_key_functor(AssociativeOperator _binary_op) : binary_op(_binary_op) {}

    __host__ __device__
    result_type operator()(result_type a, result_type b)
    {
        return result_type(thrust::get<1>(b) ? thrust::get<0>(b) : binary_op(thrust::get<0>(a), thrust::get<0>(b)),
                           thrust::get<1>(a) | thrust::get<1>(b));
    }
};

} // end namespace detail


template <typename InputIterator1,
          typename InputIterator2,
          typename OutputIterator1,
          typename OutputIterator2,
          typename BinaryPredicate,
          typename BinaryFunction>
  thrust::pair<OutputIterator1,OutputIterator2>
  reduce_by_key(InputIterator1 keys_first, 
                     InputIterator1 keys_last,
                     InputIterator2 values_first,
                     OutputIterator1 keys_output,
                     OutputIterator2 values_output,
                     BinaryPredicate binary_pred,
                     BinaryFunction binary_op)
{
    typedef typename thrust::iterator_traits<InputIterator1>::difference_type difference_type;
    typedef typename thrust::iterator_traits<InputIterator1>::value_type  KeyType;
    typedef typename thrust::iterator_traits<OutputIterator2>::value_type ValueType;
    typedef typename thrust::iterator_space<OutputIterator1>::type Space;
    typedef unsigned int FlagType;  // TODO use difference_type

    if (keys_first == keys_last)
        return thrust::make_pair(keys_output, values_output);

    // input size
    difference_type n = keys_last - keys_first;

    InputIterator2 values_last = values_first + n;
    
    // compute head flags
    thrust::detail::raw_buffer<FlagType,Space> head_flags(n);
    thrust::transform(keys_first, keys_last - 1, keys_first + 1, head_flags.begin() + 1, thrust::not2(binary_pred));
    head_flags[0] = 1;

    // compute tail flags
    thrust::detail::raw_buffer<FlagType,Space> tail_flags(n); //COPY INSTEAD OF TRANSFORM
    thrust::transform(keys_first, keys_last - 1, keys_first + 1, tail_flags.begin(), thrust::not2(binary_pred));
    tail_flags[n-1] = 1;

    // scan the values by flag
    thrust::detail::raw_buffer<ValueType,Space> scanned_values(n);
    thrust::detail::raw_buffer<FlagType,Space>  scanned_tail_flags(n);
    
    thrust::detail::device::inclusive_scan
        (thrust::make_zip_iterator(thrust::make_tuple(values_first,           head_flags.begin())),
         thrust::make_zip_iterator(thrust::make_tuple(values_last,            head_flags.end())),
         thrust::make_zip_iterator(thrust::make_tuple(scanned_values.begin(), scanned_tail_flags.begin())),
         detail::reduce_by_key_functor<ValueType, FlagType, BinaryFunction>(binary_op));

    thrust::exclusive_scan(tail_flags.begin(), tail_flags.end(), scanned_tail_flags.begin());

    // number of unique keys
    FlagType N = scanned_tail_flags[n - 1] + 1;
    
    // scatter the keys and accumulated values    
    thrust::scatter_if(keys_first,            keys_last,             scanned_tail_flags.begin(), head_flags.begin(), keys_output);
    thrust::scatter_if(scanned_values.begin(), scanned_values.end(), scanned_tail_flags.begin(), tail_flags.begin(), values_output);

    return thrust::make_pair(keys_output + N, values_output + N); 
}

} // end namespace generic
} // end namespace device
} // end namespace detail
} // end namespace thrust

