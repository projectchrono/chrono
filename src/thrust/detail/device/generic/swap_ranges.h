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


/*! \file swap_ranges.h
 *  \brief Generic device implementation of swap_ranges.
 */

#pragma once

namespace thrust
{
namespace detail
{
namespace device
{
namespace generic
{

template<typename ForwardIterator1,
         typename ForwardIterator2>
  ForwardIterator2 swap_ranges(ForwardIterator1 first1,
                               ForwardIterator1 last1,
                               ForwardIterator2 first2);

} // end namespace generic
} // end namespace device
} // end namespace detail
} // end namespace thrust

#include "swap_ranges.inl"

