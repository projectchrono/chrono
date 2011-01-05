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


/*! \file reduce_n.h
 *  \brief Reduce a sequence of elements with a given length.
 */

#pragma once

#include <thrust/pair.h>

namespace thrust
{
namespace detail
{
namespace device
{
namespace cuda
{

template<typename InputIterator,
         typename SizeType,
         typename OutputType,
         typename BinaryFunction>
  OutputType reduce_n(InputIterator first,
                      SizeType n,
                      OutputType init,
                      BinaryFunction binary_op);

template<typename RandomAccessIterator1,
         typename SizeType1,
         typename SizeType2,
         typename BinaryFunction,
         typename RandomAccessIterator2>
  void unordered_blocked_reduce_n(RandomAccessIterator1 first,
                                  SizeType1 n,
                                  SizeType2 num_blocks,
                                  BinaryFunction binary_op,
                                  RandomAccessIterator2 result);

template<typename RandomAccessIterator1,
         typename SizeType1,
         typename SizeType2,
         typename BinaryFunction,
         typename RandomAccessIterator2>
  void unordered_blocked_standard_reduce_n(RandomAccessIterator1 first,
                                           SizeType1 n,
                                           SizeType2 num_blocks,
                                           BinaryFunction binary_op,
                                           RandomAccessIterator2 result);

template<typename RandomAccessIterator1,
         typename SizeType1,
         typename SizeType2,
         typename BinaryFunction,
         typename RandomAccessIterator2>
  void unordered_blocked_wide_reduce_n(RandomAccessIterator1 first,
                                       SizeType1 n,
                                       SizeType2 num_blocks,
                                       BinaryFunction binary_op,
                                       RandomAccessIterator2 result);

template<typename RandomAccessIterator,
         typename SizeType,
         typename OutputType,
         typename BinaryFunction>
  SizeType get_unordered_blocked_reduce_n_schedule(RandomAccessIterator first,
                                                   SizeType n,
                                                   OutputType init,
                                                   BinaryFunction binary_op);

template<typename RandomAccessIterator,
         typename SizeType,
         typename OutputType,
         typename BinaryFunction>
  SizeType get_unordered_blocked_standard_reduce_n_schedule(RandomAccessIterator first,
                                                            SizeType n,
                                                            OutputType init,
                                                            BinaryFunction binary_op);

template<typename RandomAccessIterator,
         typename SizeType,
         typename OutputType,
         typename BinaryFunction>
  SizeType get_unordered_blocked_wide_reduce_n_schedule(RandomAccessIterator first,
                                                        SizeType n,
                                                        OutputType init,
                                                        BinaryFunction binary_op);

} // end namespace cuda
} // end namespace device
} // end namespace detail
} // end namespace thrust

#include <thrust/detail/device/cuda/reduce.inl>

