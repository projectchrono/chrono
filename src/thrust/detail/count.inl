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


/*! \file count.inl
 *  \brief Inline file for count.h.
 */

#include <thrust/count.h>
#include <thrust/functional.h>
#include <thrust/transform_reduce.h>

namespace thrust
{

namespace detail
{

template <typename InputType1, typename InputType2, typename CountType>
struct count_transform
{
  __host__ __device__ 
  count_transform(InputType2 val) : rhs(val){}

  __host__ __device__
  CountType operator()(const InputType1& lhs)
  {
    if(lhs == rhs)
      return 1;
    else
      return 0;
  } // end operator()()

  InputType2 rhs;
}; // end count_transform


template <typename InputType, typename Predicate, typename CountType>
struct count_if_transform
{
  __host__ __device__ 
  count_if_transform(Predicate _pred) : pred(_pred){}

  __host__ __device__
  CountType operator()(const InputType& val)
  {
    if(pred(val))
      return 1;
    else
      return 0;
  } // end operator()

  Predicate pred;
}; // end count_if_transform

} // end detail

template <typename InputIterator, typename EqualityComparable>
typename thrust::iterator_traits<InputIterator>::difference_type
count(InputIterator first, InputIterator last, const EqualityComparable& value)
{
  typedef typename thrust::iterator_traits<InputIterator>::value_type InputType;
  typedef typename thrust::iterator_traits<InputIterator>::difference_type CountType;
  
  thrust::detail::count_transform<InputType, EqualityComparable, CountType> unary_op(value);
  thrust::plus<CountType> binary_op;
  return thrust::transform_reduce(first, last, unary_op, CountType(0), binary_op);
} // end count()

template <typename InputIterator, typename Predicate>
typename thrust::iterator_traits<InputIterator>::difference_type
count_if(InputIterator first, InputIterator last, Predicate pred)
{
  typedef typename thrust::iterator_traits<InputIterator>::value_type InputType;
  typedef typename thrust::iterator_traits<InputIterator>::difference_type CountType;
  
  thrust::detail::count_if_transform<InputType, Predicate, CountType> unary_op(pred);
  thrust::plus<CountType> binary_op;
  return thrust::transform_reduce(first, last, unary_op, CountType(0), binary_op);
} // end count_if()

} // end namespace thrust

