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



#include <thrust/iterator/transform_iterator.h>
#include <thrust/iterator/iterator_adaptor.h>
#include <thrust/iterator/iterator_traits.h>
#include <thrust/detail/type_traits.h>
#include <thrust/detail/device/dereference.h>

namespace thrust
{

template <class UnaryFunction, class Iterator, class Reference, class Value>
  class transform_iterator;
  
namespace detail 
{

template <class UnaryFunc>
struct function_object_result
{
  typedef typename UnaryFunc::result_type type;
};

// XXX function pointers don't compile yet
//     enable this when they do
//template <class Return, class Argument>
//struct function_object_result<Return(*)(Argument)>
//{
//  typedef Return type;
//};

// Compute the iterator_adaptor instantiation to be used for transform_iterator
template <class UnaryFunc, class Iterator, class Reference, class Value>
struct transform_iterator_base
{
 private:
    // By default, dereferencing the iterator yields the same as
    // the function.  Do we need to adjust the way
    // function_object_result is computed for the standard
    // proposal (e.g. using Doug's result_of)?
    typedef typename thrust::experimental::detail::ia_dflt_help<
        Reference
      , function_object_result<UnaryFunc>
    >::type reference;

    // To get the default for Value: remove any reference on the
    // result type, but retain any constness to signal
    // non-writability.  Note that if we adopt Thomas' suggestion
    // to key non-writability *only* on the Reference argument,
    // we'd need to strip constness here as well.
    typedef typename thrust::experimental::detail::ia_dflt_help<
        Value
      , thrust::detail::remove_reference<reference>
    >::type cv_value_type;

    typedef typename thrust::iterator_traits<Iterator>::pointer pointer_;

 public:
    typedef thrust::experimental::iterator_adaptor
    <
        transform_iterator<UnaryFunc, Iterator, Reference, Value>
      , Iterator
      , pointer_
      , cv_value_type
      , thrust::use_default   // Leave the space alone
        //, thrust::use_default   // Leave the traversal alone
        // use the Iterator's category to let any space iterators remain random access even though
        // transform_iterator's reference type may not be a reference
        // XXX figure out why only iterators whose reference types are true references are random access
        , typename thrust::iterator_traits<Iterator>::iterator_category
      , reference
    > type;
};


namespace device
{


// specialize dereference_result for transform_iterator
// transform_iterator returns the same reference on the device as on the host
template<typename UnaryFunc, typename Iterator, typename Reference, typename Value>
  struct dereference_result< thrust::transform_iterator<UnaryFunc, Iterator, Reference, Value> >
{
  typedef typename thrust::iterator_traits< thrust::transform_iterator<UnaryFunc,Iterator,Reference,Value> >::reference type;
}; // end dereference_result


template<typename UnaryFunc, typename Iterator, typename Reference, typename Value>
  inline __host__ __device__
    typename dereference_result< thrust::transform_iterator<UnaryFunc,Iterator,Reference,Value> >::type
      dereference(const thrust::transform_iterator<UnaryFunc,Iterator,Reference,Value> &iter)
{
  return iter.functor()( dereference(iter.base()) );
} // end dereference()

template<typename UnaryFunc, typename Iterator, typename Reference, typename Value, typename IndexType>
  inline __host__ __device__
    typename dereference_result< thrust::transform_iterator<UnaryFunc,Iterator,Reference,Value> >::type
      dereference(const thrust::transform_iterator<UnaryFunc,Iterator,Reference,Value> &iter, IndexType n)
{
  return iter.functor()( dereference(iter.base(), n) );
} // end dereference()

} // end device

} // end detail

} // end thrust

