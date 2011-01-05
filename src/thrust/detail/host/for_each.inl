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

#pragma once

#include <thrust/detail/host/for_each.h>

namespace thrust
{
namespace detail
{
namespace host
{


template<typename InputIterator,
         typename UnaryFunction>
InputIterator for_each(InputIterator first,
                       InputIterator last,
                       UnaryFunction f)
{
  for(; first != last; ++first)
  {
    f(*first);
  }

  return first;
} // end for_each()


template<typename OutputIterator,
         typename Size,
         typename UnaryFunction>
OutputIterator for_each_n(OutputIterator first,
                          Size n,
                          UnaryFunction f)
{
  for(Size i = 0; i != n; i++)
  {
    // we can dereference an OutputIterator if f does not
    // try to use the reference for anything besides assignment
    f(*first);
    ++first;
  }

  return first;
} // end for_each_n()


} // end namespace host
} // end namespace detail
} // end namespace thrust

