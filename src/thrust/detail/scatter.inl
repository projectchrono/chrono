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


/*! \file scatter.inl
 *  \brief Inline file for scatter.h.
 */

#include <thrust/scatter.h>
#include <thrust/iterator/iterator_traits.h>
#include <thrust/detail/dispatch/scatter.h>

namespace thrust
{

template<typename InputIterator1,
         typename InputIterator2,
         typename RandomAccessIterator>
  void scatter(InputIterator1 first,
               InputIterator1 last,
               InputIterator2 map,
               RandomAccessIterator output)
{
  // dispatch on space
  thrust::detail::dispatch::scatter(first, last, map, output,
    typename thrust::iterator_space<InputIterator1>::type(),
    typename thrust::iterator_space<InputIterator2>::type(),
    typename thrust::iterator_space<RandomAccessIterator>::type());
} // end scatter()


template<typename InputIterator1,
         typename InputIterator2,
         typename InputIterator3,
         typename RandomAccessIterator>
  void scatter_if(InputIterator1 first,
                  InputIterator1 last,
                  InputIterator2 map,
                  InputIterator3 stencil,
                  RandomAccessIterator output)
{
  // default predicate is identity
  typedef typename thrust::iterator_traits<InputIterator3>::value_type StencilType;
  scatter_if(first, last, map, stencil, output, thrust::identity<StencilType>());
} // end scatter_if()


template<typename InputIterator1,
         typename InputIterator2,
         typename InputIterator3,
         typename RandomAccessIterator,
         typename Predicate>
  void scatter_if(InputIterator1 first,
                  InputIterator1 last,
                  InputIterator2 map,
                  InputIterator3 stencil,
                  RandomAccessIterator output,
                  Predicate pred)
{
  // dispatch on space
  thrust::detail::dispatch::scatter_if(first, last, map, stencil, output, pred,
    typename thrust::iterator_space<InputIterator1>::type(),
    typename thrust::iterator_space<InputIterator2>::type(),
    typename thrust::iterator_space<InputIterator3>::type(),
    typename thrust::iterator_space<RandomAccessIterator>::type());
} // end scatter_if()

} // end namespace thrust

