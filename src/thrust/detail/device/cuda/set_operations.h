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

namespace thrust
{

namespace detail
{

namespace device
{

namespace cuda
{

template<typename RandomAccessIterator1,
         typename RandomAccessIterator2,
         typename RandomAccessIterator3,
         typename StrictWeakOrdering>
  RandomAccessIterator3 set_intersection(RandomAccessIterator1 first1,
                                         RandomAccessIterator1 last1,
                                         RandomAccessIterator2 first2,
                                         RandomAccessIterator2 last2,
                                         RandomAccessIterator3 result,
                                         StrictWeakOrdering comp);

} // end cuda

} // end device

} // end detail

} // end thrust

#include <thrust/detail/device/cuda/set_operations.inl>

