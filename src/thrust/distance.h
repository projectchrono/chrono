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


/*! \file distance.h
 *  \brief Defines the interface to a function for
 *         computing the size of an input range.
 */

#pragma once

#include <thrust/detail/config.h>
#include <thrust/iterator/iterator_traits.h>

namespace thrust
{

/*! \addtogroup iterators
 *  \{
 */

/*! \p distance finds the distance between \p first and \p last, i.e. the
 *  number of times that \p first must be incremented until it is equal to
 *  \p last.
 *
 *  \param first The beginning of an input range of interest.
 *  \param last The end of an input range of interest.
 *  \return The distance between the beginning and end of the input range.
 *
 *  \tparam InputIterator is a model of <a href="http://www.sgi.com/tech/stl/InputIterator.html">Input Iterator</a>.
 *
 *  \see http://www.sgi.com/tech/stl/distance.html
 */
template<typename InputIterator>
  inline typename thrust::iterator_traits<InputIterator>::difference_type
    distance(InputIterator first, InputIterator last);

/*! \} // end iterators
 */

} // end thrust

#include <thrust/detail/distance.inl>

