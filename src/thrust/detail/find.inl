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


/*! \file find.inl
 *  \brief Inline file for find.h
 */

#include <thrust/detail/dispatch/find.h>
#include <thrust/detail/internal_functional.h>
#include <thrust/iterator/iterator_traits.h>

namespace thrust
{

template <typename InputIterator, typename T>
InputIterator find(InputIterator first,
                   InputIterator last,
                   const T& value)
{
    return thrust::find_if(first, last, thrust::detail::equal_to_value<T>(value));
}

template <typename InputIterator, typename Predicate>
InputIterator find_if(InputIterator first,
                      InputIterator last,
                      Predicate pred)
{
    // dispatch on space
    return thrust::detail::dispatch::find_if(first, last, pred,
            typename thrust::iterator_space<InputIterator>::type());
                                    
}

template <typename InputIterator, typename Predicate>
InputIterator find_if_not(InputIterator first,
                          InputIterator last,
                          Predicate pred)
{
    return thrust::find_if(first, last, thrust::detail::not1(pred));
}

} // end namespace thrust

