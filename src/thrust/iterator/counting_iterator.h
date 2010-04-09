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


/*! \file counting_iterator.h
 *  \brief Defines the interface to an iterator
 *         which adapts an incrementable type
 *         to return the current value of the incrementable
 *         upon operator*(). Based on Boost's counting_iterator
 *         class.
 */

// thrust::counting_iterator is derived from
// boost::counting_iterator of the Boost Iterator
// Library, which is the work of
// David Abrahams, Jeremy Siek, & Thomas Witt.
// See http://www.boost.org for details.

#pragma once

#include <thrust/detail/config.h>
#include <thrust/iterator/iterator_adaptor.h>
#include <thrust/iterator/iterator_categories.h>

// #include the details first
#include <thrust/iterator/detail/counting_iterator.inl>

namespace thrust
{

/*! \addtogroup iterators
 *  \{
 */

/*! \addtogroup fancyiterator Fancy Iterators
 *  \ingroup iterators
 *  \{
 */

/*! \p counting_iterator is an iterator which represents a pointer into a range
 *  of sequentially changing values. This iterator is useful for creating a range
 *  filled with a sequence without explicitly storing it in memory. Using
 *  \p counting_iterator saves memory capacity and bandwidth.
 *
 *  The following code snippet demonstrates how to create a \p counting_iterator whose
 *  \c value_type is \c int and which sequentially increments by \c 1.
 *
 *  \code
 *  #include <thrust/iterator/counting_iterator.h>
 *  ...
 *  // create iterators
 *  thrust::counting_iterator<int> first(10);
 *  thrust::counting_iterator<int> last = first + 3;
 *   
 *  first[0]   // returns 10
 *  first[1]   // returns 11
 *  first[100] // returns 110
 *   
 *  // sum of [first, last)
 *  thrust::reduce(first, last);   // returns 33 (i.e. 10 + 11 + 12)
 *   
 *  // initialize vector to [0,1,2,..]
 *  thrust::counting_iterator<int> iter(0);
 *  thrust::device_vector<int> vec(500);
 *  thrust::copy(iter, iter + vec.size(), vec.begin());
 *  \endcode
 *
 *  This next example demonstrates how to use a \p counting_iterator with the
 *  \p thrust::copy_if function to compute the indices of the non-zero elements
 *  of a \p device_vector. In this example, we use the \p make_counting_iterator
 *  function to avoid specifying the type of the \p counting_iterator.
 *
 *  \code
 *  #include <thrust/iterator/counting_iterator.h>
 *  #include <thrust/copy.h>
 *  #include <thrust/functional.h>
 *  #include <thrust/device_vector.h>
 *   
 *  int main(void)
 *  {
 *   // this example computes indices for all the nonzero values in a sequence
 *   
 *   // sequence of zero and nonzero values
 *   thrust::device_vector<int> stencil(8);
 *   stencil[0] = 0;
 *   stencil[1] = 1;
 *   stencil[2] = 1;
 *   stencil[3] = 0;
 *   stencil[4] = 0;
 *   stencil[5] = 1;
 *   stencil[6] = 0;
 *   stencil[7] = 1;
 *   
 *   // storage for the nonzero indices
 *   thrust::device_vector<int> indices(8);
 *   
 *   // compute indices of nonzero elements
 *   typedef thrust::device_vector<int>::iterator IndexIterator;
 *   
 *   // use make_counting_iterator to define the sequence [0, 8)
 *   IndexIterator indices_end = thrust::copy_if(thrust::make_counting_iterator(0),
 *                                               thrust::make_counting_iterator(8),
 *                                               stencil.begin(),
 *                                               indices.begin(),
 *                                               thrust::identity<int>());
 *   // indices now contains [1,2,5,7]
 *   
 *   return 0;
 *  }
 *  \endcode
 *
 *  \see make_counting_iterator
 */
template<typename Incrementable,
         typename Space = use_default,
         typename Traversal = use_default,
         typename Difference = use_default>
  class counting_iterator
    : public detail::counting_iterator_base<Incrementable, Space, Traversal, Difference>::type
{
    /*! \cond
     */
    typedef typename detail::counting_iterator_base<Incrementable, Space, Traversal, Difference>::type super_t;

    friend class thrust::experimental::iterator_core_access;

  public:
    typedef Incrementable const & reference;
    typedef typename super_t::difference_type difference_type;
    typedef counting_iterator device_dereferenceable_type;

    /*! \endcond
     */

    /*! Null constructor initializes this \p counting_iterator's \c Incrementable
     *  counter using its null constructor.
     */
    __host__ __device__
    counting_iterator(void){};

    /*! Copy constructor copies the value of another \p counting_iterator into a
     *  new \p counting_iterator.
     *
     *  \p rhs The \p counting_iterator to copy.
     */
    __host__ __device__
    counting_iterator(counting_iterator const &rhs):super_t(rhs.base()){}

    /*! This \c explicit constructor copies the value of an \c Incrementable
     *  into a new \p counting_iterator's \c Incrementable counter.
     *  
     *  \param x The initial value of the new \p counting_iterator's \c Incrementable
     *         counter.
     */
    __host__ __device__
    explicit counting_iterator(Incrementable x):super_t(x){}

    /*! \cond
     */
  private:
    __host__ __device__
    reference dereference(void) const
    {
      return this->base_reference();
    }

    template <class OtherIncrementable>
    __host__ __device__
    difference_type
    distance_to(counting_iterator<OtherIncrementable, Space, Traversal, Difference> const& y) const
    {
      typedef typename
      thrust::detail::eval_if<
        thrust::detail::is_numeric<Incrementable>::value,
        thrust::detail::identity_<thrust::detail::number_distance<difference_type, Incrementable, OtherIncrementable> >,
        thrust::detail::identity_<thrust::detail::iterator_distance<difference_type, Incrementable, OtherIncrementable> >
      >::type d;

      return d::distance(this->base(), y.base());
    }

    /*! \endcond
     */
}; // end counting_iterator


/*! \p make_counting_iterator creates a \p counting_iterator
 *  using an initial value for its \c Incrementable counter.
 *
 *  \param x The initial value of the new \p counting_iterator's counter.
 *  \return A new \p counting_iterator whose counter has been initialized to \p x.
 */
template <typename Incrementable>
inline __host__ __device__
counting_iterator<Incrementable> make_counting_iterator(Incrementable x)
{
  return counting_iterator<Incrementable>(x);
}

/*! \} // end fancyiterators
 */

/*! \} // end iterators
 */

} // end thrust

