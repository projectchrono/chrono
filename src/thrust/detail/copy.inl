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


/*! \file copy.inl
 *  \brief Inline file for copy.h.
 */

#include <thrust/detail/dispatch/copy.h>

#include <thrust/functional.h>
#include <thrust/iterator/iterator_traits.h>
#include <thrust/transform.h>

namespace thrust
{

//////////
// copy //
//////////

template<typename InputIterator,
         typename OutputIterator>
  OutputIterator copy(InputIterator first,
                      InputIterator last,
                      OutputIterator result)
{
  // XXX make sure this isn't necessary
  if(first == last) 
    return result;

  return thrust::detail::dispatch::copy(first, last, result,
          typename thrust::iterator_space<InputIterator>::type(),
          typename thrust::iterator_space<OutputIterator>::type());
}


///////////////
// copy_when //
///////////////

template<typename InputIterator,
         typename PredicateIterator,
         typename OutputIterator>
  OutputIterator copy_when(InputIterator first,
                           InputIterator last,
                           PredicateIterator stencil,
                           OutputIterator result)
{
    // default predicate is identity
    typedef typename thrust::iterator_traits<PredicateIterator>::value_type StencilType;
    return thrust::copy_when(first, last, stencil, result, thrust::identity<StencilType>());
}

template<typename InputIterator,
         typename PredicateIterator,
         typename OutputIterator,
         typename Predicate>
  OutputIterator copy_when(InputIterator first,
                           InputIterator last,
                           PredicateIterator stencil,
                           OutputIterator result,
                           Predicate pred)
{
    typedef typename thrust::iterator_traits<InputIterator>::value_type InputType;
    return thrust::transform_if(first, last, stencil, result, thrust::identity<InputType>(), pred);
}


/////////////
// copy_if //
/////////////

template<typename InputIterator,
         typename OutputIterator,
         typename Predicate>
  OutputIterator copy_if(InputIterator first,
                         InputIterator last,
                         OutputIterator result,
                         Predicate pred)
{
  // XXX it's potentially expensive to send [first,last) twice
  //     we should probably specialize this case for POD
  //     since we can safely keep the input in a temporary instead
  //     of doing two loads
  return thrust::copy_if(first, last, first, result, pred);
}

template<typename InputIterator1,
         typename InputIterator2,
         typename OutputIterator,
         typename Predicate>
  OutputIterator copy_if(InputIterator1 first,
                         InputIterator1 last,
                         InputIterator2 stencil,
                         OutputIterator result,
                         Predicate pred)
{
  return detail::dispatch::copy_if(first, last, stencil, result, pred,
          typename thrust::iterator_space<InputIterator1>::type(),
          typename thrust::iterator_space<InputIterator2>::type(),
          typename thrust::iterator_space<OutputIterator>::type());
}

} // end namespace thrust

