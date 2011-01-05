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

/*! \file set_operations.inl
 *  \brief Inline file for set_operations.h.
 */

#include <thrust/set_operations.h>
#include <thrust/iterator/iterator_traits.h>
#include <thrust/functional.h>
#include <thrust/detail/dispatch/set_operations.h>

namespace thrust
{

template<typename InputIterator1,
         typename InputIterator2,
         typename OutputIterator,
         typename StrictWeakOrdering>
  OutputIterator set_intersection(InputIterator1 first1,
                                  InputIterator1 last1,
                                  InputIterator2 first2,
                                  InputIterator2 last2,
                                  OutputIterator result,
                                  StrictWeakOrdering comp)
{
  return thrust::detail::dispatch::set_intersection(first1, last1,
                                                    first2, last2,
                                                    result, comp,
    typename thrust::iterator_space<InputIterator1>::type(),
    typename thrust::iterator_space<InputIterator2>::type(),
    typename thrust::iterator_space<OutputIterator>::type());
} // end set_intersection()

template<typename InputIterator1,
         typename InputIterator2,
         typename OutputIterator>
  OutputIterator set_intersection(InputIterator1 first1,
                                  InputIterator1 last1,
                                  InputIterator2 first2,
                                  InputIterator2 last2,
                                  OutputIterator result)
{
  typedef typename thrust::iterator_value<InputIterator1>::type value_type;
  return thrust::set_intersection(first1, last1, first2, last2, result, thrust::less<value_type>());
} // end set_intersection()

} // end thrust

