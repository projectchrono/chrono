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


/*! \file remove.inl
 *  \brief Inline file for remove.h.
 */

#include <thrust/remove.h>
#include <thrust/iterator/iterator_traits.h>
#include <thrust/detail/dispatch/remove.h>

namespace thrust
{

namespace detail
{

template<typename T>
  struct equal_to_value
{
  equal_to_value(const T &v):value(v){}

  __host__ __device__
  inline bool operator()(const T &x) { return x == value; }

  const T value;
}; // end equal_to_value

} // end detail

template<typename ForwardIterator,
         typename T>
  ForwardIterator remove(ForwardIterator first,
                         ForwardIterator last,
                         const T &value)
{
  detail::equal_to_value<T> pred(value);

  return thrust::remove_if(first, last, pred);
} // end remove()

template<typename InputIterator,
         typename OutputIterator,
         typename T>
  OutputIterator remove_copy(InputIterator first,
                             InputIterator last,
                             OutputIterator result,
                             const T &value)
{
  detail::equal_to_value<T> pred(value);

  return thrust::remove_copy_if(first, last, result, pred);
} // end remove_copy()

template<typename ForwardIterator,
         typename Predicate>
  ForwardIterator remove_if(ForwardIterator first,
                            ForwardIterator last,
                            Predicate pred)
{
  return thrust::remove_if(first, last, first, pred);
} // end remove_if()

template<typename ForwardIterator,
         typename InputIterator,
         typename Predicate>
  ForwardIterator remove_if(ForwardIterator first,
                            ForwardIterator last,
                            InputIterator stencil,
                            Predicate pred)
{
  return detail::dispatch::remove_if(first, last, stencil, pred,
    typename thrust::iterator_space<ForwardIterator>::type(),
    typename thrust::iterator_space<InputIterator>::type());
} // end remove_if()

template<typename InputIterator,
         typename OutputIterator,
         typename Predicate>
  OutputIterator remove_copy_if(InputIterator first,
                                InputIterator last,
                                OutputIterator result,
                                Predicate pred)
{
  return detail::dispatch::remove_copy_if(first, last, result, pred,
    typename thrust::iterator_space<InputIterator>::type(),
    typename thrust::iterator_space<OutputIterator>::type());
} // end remove_copy_if()

template<typename InputIterator1,
         typename InputIterator2,
         typename OutputIterator,
         typename Predicate>
  OutputIterator remove_copy_if(InputIterator1 first,
                                InputIterator1 last,
                                InputIterator2 stencil,
                                OutputIterator result,
                                Predicate pred)
{
  return detail::dispatch::remove_copy_if(first, last, stencil, result, pred,
    typename thrust::iterator_space<InputIterator1>::type(),
    typename thrust::iterator_space<InputIterator2>::type(),
    typename thrust::iterator_space<OutputIterator>::type());
} // end remove_copy_if()

} // end namespace thrust

