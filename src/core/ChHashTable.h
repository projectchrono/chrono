#ifndef CHHASHTABLE__H
#define CHHASHTABLE__H

//////////////////////////////////////////////////
//
//   ChHashTable.h
//
//   A simple chained hash table.
//
//   This hash table is inspired by the hash code in
//   the STLplus code - see Andy Rushton, 2004, 
//   but it is adapted to the needs of the  core
//   Chrono::Engine SDK.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "ChException.h"
#include "ChHashFunction.h"

namespace chrono
{



// Forward declarations

template<typename K, typename T, class H, class E, typename V>class ch_hash_iterator;
template<typename K, typename T> class hash_element;





/// The ChHashTable class. This container is most useful when you need
/// a quasi-constant-time retrieval of a value, given its key. In fact,
/// this container stores 'pairs' of values-keys, where keys should be 
/// unique (an example: a key could be the credit card number, and the 
/// value could be a structure with name, age and address of the credit 
/// card owner). 
/// When the hash table is filled too much, the performance get worse.
/// Also, when you fill it beyond the initial suggested number of bins, 
/// an automatic table resizing (with rehashing) is performed. This may
/// be a time-consuming cpu operation! So, choose initial size as close
/// as possible to the maximum number of elements that you foresee to 
/// insert, plus some more space (es: +50%) to avoid performance degradation.
///
/// K = key type
/// T = value type
/// H = hash function object with the profile 'unsigned H(const K&)'
/// E = equal function object with the profile 'bool E(const K&, const K&)' defaults to equal_to which in turn calls '=='

template<typename K, typename T, class H = HashFunction_Generic<K>, class E = std::equal_to<K> >
class ChHashTable
{
public:
  typedef unsigned                                size_type;
  typedef K                                       key_type;
  typedef T                                       data_type;
  typedef T                                       mapped_type;
  typedef std::pair<const K, T>                   value_type;
  typedef ch_hash_iterator<K,T,H,E,value_type>       iterator;
  typedef ch_hash_iterator<K,T,H,E,const value_type> const_iterator;

		/// Construct a ChHashTable table with specified number of bins.
		/// The default 0 bins means leave it to the table to decide.
		/// Specifying 0 bins also enables auto-rehashing, otherwise auto-rehashing defaults off
  ChHashTable(unsigned bins = 0);
  ~ChHashTable(void);

		/// Copy and equality copy the data elements but not the size of the copied table
  ChHashTable(const ChHashTable&);
  ChHashTable& operator = (const ChHashTable&);

		/// Test for an empty table and for the size of a table
		/// Efficient because the size is stored separately from the table contents
  bool empty(void) const;
  unsigned size(void) const;

		/// Test for equality - two hashes are equal if they contain equal values
  bool operator == (const ChHashTable&) const;
  bool operator != (const ChHashTable&) const;

		/// Switch auto-rehash on
  void auto_rehash(void);
		/// Switch auto-rehash off
  void manual_rehash(void);
		/// Force a rehash now
		/// Default of 0 means implement built-in size calculation for rehashing 
		/// (recommended - it doubles the number of bins)
  void rehash(unsigned bins = 0);

		/// Test the loading ratio, which is the size divided by the number of bins.
		/// Use this if you are doing your own rehashing.
  float loading(void) const;


		/// Test for the presence of a key
  bool present(const K& key) const;

		/// Provide map equivalent key count function (0 or 1, as not a multimap)
  size_type count(const K& key) const;

		/// Insert a new key/data pair - replaces any previous value for this key
  iterator insert(const K& key, const T& data);
  
		/// Insert a copy of the pair into the table (std::map compatible)
  std::pair<iterator, bool> insert(const value_type& value);

		/// Insert a new key and return the iterator so that the data can be filled in
  iterator insert(const K& key);

		/// Remove a key/data pair from the hash table
  bool erase(const K& key);
		
		/// Remove all elements from the hash table
  void erase(void);

		/// Provide the std::map equivalent clear function
  void clear(void);

		/// Find a key and return an iterator to it
		/// The iterator is like a pointer to a pair<const K,T>
		/// end() is returned if the find fails
  const_iterator find(const K& key) const;
  iterator find(const K& key);

		/// Returns the data corresponding to the key
		/// the const version is used by the compiler on const hashes and cannot change the hash, so find failure causes an exception
		/// the non-const version is used by the compiler on non-const hashes and is like map - it creates a new key/data pair if find fails
  const T& operator[] (const K& key) const;
  T& operator[] (const K& key);

		/// Iterators allow the hash table to be traversed
		/// Iterators remain valid unless an item is removed or unless a rehash happens
  const_iterator begin(void) const;
  iterator begin(void);
  const_iterator end(void) const;
  iterator end(void);

		
private:
		
	//
	// DATA
	//

  friend class hash_element<K,T>;
  friend class ch_hash_iterator<K,T,H,E,std::pair<const K,T> >;
  friend class ch_hash_iterator<K,T,H,E,const std::pair<const K,T> >;

  unsigned m_rehash;
  unsigned m_bins;
  unsigned m_size;
  hash_element<K,T>** m_values;
};







//
// Iterator class for the ChHashTable class
//

template<typename K, typename T, class H, class E, typename V>
class ch_hash_iterator
{
public:
  friend class ChHashTable<K,T,H,E>;

	  // Local type definitions
	  // An iterator points to a value whilst a const_iterator points to a const value
  typedef V                                                  value_type;
  typedef ch_hash_iterator<K,T,H,E,std::pair<const K,T> >       iterator;
  typedef ch_hash_iterator<K,T,H,E,const std::pair<const K,T> > const_iterator;
  typedef ch_hash_iterator<K,T,H,E,V>                           this_iterator;
  typedef V&                                                 reference;
  typedef V*                                                 pointer;

	  /// Constructor to create a null iterator - you must assign a valid value to 
	  /// this iterator before using it
	  /// Any attempt to dereference or use a null iterator is an error
	  /// the only valid thing you can do is assign an iterator to it
  ch_hash_iterator(void);

      /// Destructor
  ~ch_hash_iterator(void);


	  /// A null iterator is one that has not been initialised with a value yet
	  /// i.e. you just declared it but didn't assign to it
  bool null(void) const;

	  /// An end iterator is one that points to the end element of the list of nodes
	  /// in STL conventions this is one past the last valid element and must not be dereferenced
  bool end(void) const;

	  /// A valid iterator is one that can be dereferenced
	  /// i.e. non-null and non-end
  bool valid(void) const;

	  /// Get the hash container that created this iterator
	  /// a null iterator doesn't have an owner so returns a null pointer
  const ChHashTable<K,T,H,E>* owner(void) const;

	  /// Type conversion methods allow const_iterator and iterator to be converted
	  /// convert an iterator/const_iterator to a const_iterator
  const_iterator constify(void) const;

	  /// Convert an iterator/const_iterator to an iterator
  iterator deconstify(void) const;

	  /// increment operators used to step through the set of all values in a hash
	  /// it is only legal to increment a valid iterator
	  /// there's no decrement - I've only implemented this as a unidirectional iterator
	  /// pre-increment.
	  /// Throws ChException if unsuccesful.
  this_iterator& operator ++ (void);

	  /// Post-increment
	  /// Throws ChException if unsuccesful.
  this_iterator operator ++ (int);
  
	  /// Tests, for putting iterators into other STL structures and 
	  /// for testing whether iteration has completed.
  bool operator == (const this_iterator& r) const;
  bool operator != (const this_iterator& r) const;
  bool operator < (const this_iterator& r) const;

	  /// Access the value - a const_iterator gives you a const value, an iterator a non-const value
	  /// it is illegal to dereference an invalid (i.e. null or end) iterator
  reference operator*(void) const;
  pointer operator->(void) const;

  
private:
  friend class hash_element<K,T>;

  const ChHashTable<K,T,H,E>* m_owner;
  unsigned m_bin;
  hash_element<K,T>* m_element;

  void check_owner(const ChHashTable<K,T,H,E>* owner) const;

  void check_non_null(void) const;

  void check_non_end(void) const;

  void check_valid(void) const;

  void check(const ChHashTable<K,T,H,E>* owner) const;

  // Constructor used by ChHashTable.
  // (You cannot create a valid iterator except by calling a ChHashTable method that returns one)
  ch_hash_iterator(const ChHashTable<K,T,H,E>* owner, unsigned bin, hash_element<K,T>* element);
};







////////////////////////////////////////////////////////////////////////////////
// the element stored in the ChHashTable

template<typename K, typename T>
class hash_element
{
public:
  std::pair<const K, T> m_value;
  hash_element<K,T>* m_next;
  unsigned m_hash;

  hash_element(const K& key, const T& data, unsigned hash) : m_value(key,data), m_next(0), m_hash(hash) {}
  hash_element(const std::pair<const K,T>& value, unsigned hash) : m_value(value), m_next(0), m_hash(hash) {}
};

////////////////////////////////////////////////////////////////////////////////
// iterator

// checks

template<typename K, typename T, class H, class E, typename V>
void ch_hash_iterator<K,T,H,E,V>::check_owner(const ChHashTable<K,T,H,E>* owner) const
{
  if (owner != m_owner)
    throw ChException("hash iterator");
}

template<typename K, typename T, class H, class E, typename V>
void ch_hash_iterator<K,T,H,E,V>::check_non_null(void) const
{
  if (null())
    throw ChException("hash iterator");
}

template<typename K, typename T, class H, class E, typename V>
void ch_hash_iterator<K,T,H,E,V>::check_non_end(void) const
{
  if (end())
    throw ChException("hash iterator");
}
 
template<typename K, typename T, class H, class E, typename V>
void ch_hash_iterator<K,T,H,E,V>::check_valid(void) const
{
  check_non_null();
  check_non_end(); 
}

template<typename K, typename T, class H, class E, typename V>
void ch_hash_iterator<K,T,H,E,V>::check(const ChHashTable<K,T,H,E>* owner) const
{
  check_valid();
  if (owner) check_owner(owner);
}

// null constructor

template<typename K, typename T, class H, class E, typename V>
ch_hash_iterator<K,T,H,E,V>::ch_hash_iterator(void) :
  m_owner(0), m_bin(0), m_element(0) 
{
}

// this iterator points to a specific element and so allows an iterator to be constructed from a list node
// if the bin is set to bins and the element to null, then this is an end() iterator
// if the element parameter is null it scans for the first element so implements the begin() behaviour
// if the ChHashTable is empty, m_bin gets set to m_bins, so this becomes an end() iterator

template<typename K, typename T, class H, class E, typename V>
ch_hash_iterator<K,T,H,E,V>::ch_hash_iterator(const ChHashTable<K,T,H,E>* owner, unsigned bin, hash_element<K,T>* element) :
  m_owner(owner), m_bin(bin), m_element(element)
{
  //DEBUG_ASSERT(owner);
  if (!element)
  {
    for (; m_bin < m_owner->m_bins; m_bin++)
    {
      if (m_owner->m_values[m_bin])
      {
        m_element = m_owner->m_values[m_bin];
        break;
      }
    }
  }
}

// destructor

template<typename K, typename T, class H, class E, typename V>
ch_hash_iterator<K,T,H,E,V>::~ch_hash_iterator(void)
{
}

// validity tests

template<typename K, typename T, class H, class E, typename V>
bool ch_hash_iterator<K,T,H,E,V>::null(void) const
{
  return m_owner == 0;
}

template<typename K, typename T, class H, class E, typename V>
bool ch_hash_iterator<K,T,H,E,V>::end(void) const
{
  if (null()) return false;
  return (m_bin == m_owner->m_bins) && m_element == 0;
}

template<typename K, typename T, class H, class E, typename V>
bool ch_hash_iterator<K,T,H,E,V>::valid(void) const
{
  return !null() && !end();
}

// owner method

template<typename K, typename T, class H, class E, typename V>
const ChHashTable<K,T,H,E>* ch_hash_iterator<K,T,H,E,V>::owner(void) const
{
  return m_owner;
}

// mode conversions

template<typename K, typename T, class H, class E, typename V>
typename ch_hash_iterator<K,T,H,E,V>::const_iterator ch_hash_iterator<K,T,H,E,V>::constify(void) const
{
  return ch_hash_iterator<K,T,H,E,V>::const_iterator(m_owner, m_bin, m_element);
}

template<typename K, typename T, class H, class E, typename V>
typename ch_hash_iterator<K,T,H,E,V>::iterator ch_hash_iterator<K,T,H,E,V>::deconstify(void) const
{
  return ch_hash_iterator<K,T,H,E,V>::iterator(m_owner, m_bin, m_element);
}

// increment operator looks for the next element in the table
// if there isn't one, then this becomes an end() iterator - m_bin = m_bins and m_element = null

template<typename K, typename T, class H, class E, typename V>
typename ch_hash_iterator<K,T,H,E,V>::this_iterator& ch_hash_iterator<K,T,H,E,V>::operator ++ (void)
{
  check_valid();
  if (m_element->m_next)
    m_element = m_element->m_next;
  else
  {
    // failing that, subsequent hash values are tried until either an element is found or there are no more bins
    // in which case it becomes an end() iterator (bin == bins and element = null)
    m_element = 0;
    if (m_bin < m_owner->m_bins)
    {
      for(m_bin++; m_bin < m_owner->m_bins; m_bin++)
      {
        if (m_owner->m_values[m_bin])
        {
          m_element = m_owner->m_values[m_bin];
          break;
        }
      }
    }
  }
  return *this;
}

// post-increment is defined in terms of pre-increment

template<typename K, typename T, class H, class E, typename V>
typename ch_hash_iterator<K,T,H,E,V>::this_iterator ch_hash_iterator<K,T,H,E,V>::operator ++ (int)
{
  typename ch_hash_iterator<K,T,H,E,V>::this_iterator old = *this;
  ++(*this);
  return old;
}

// two iterators are equal if they point to the same element
// both iterators must be non-null and belong to the same table

template<typename K, typename T, class H, class E, typename V>
bool ch_hash_iterator<K,T,H,E,V>::operator == (const ch_hash_iterator<K,T,H,E,V>& r) const
{
  return m_element == r.m_element;
}

template<typename K, typename T, class H, class E, typename V>
bool ch_hash_iterator<K,T,H,E,V>::operator != (const ch_hash_iterator<K,T,H,E,V>& r) const
{
  return !operator==(r);
}

template<typename K, typename T, class H, class E, typename V>
bool ch_hash_iterator<K,T,H,E,V>::operator < (const ch_hash_iterator<K,T,H,E,V>& r) const
{
  return m_element < r.m_element;
}

// iterator dereferencing is only legal on a non-null iterator

template<typename K, typename T, class H, class E, typename V>
V& ch_hash_iterator<K,T,H,E,V>::operator*(void) const
{
  check_valid();
  return m_element->m_value;
}

template<typename K, typename T, class H, class E, typename V>
V* ch_hash_iterator<K,T,H,E,V>::operator->(void) const
{
  check_valid();
  return &(m_element->m_value);
}








//
//  ChHashTable
// 





// totally arbitrary initial size used for auto-rehashed tables
static unsigned hash_default_bins = 127;

// constructor
// tests whether the user wants auto-rehash
// sets the rehash point to be a loading of 1.0 by setting it to the number of bins
// uses the user's size unless this is zero, in which case implement the default

template<typename K, typename T, class H, class E>
ChHashTable<K,T,H,E>::ChHashTable(unsigned bins) :
  m_rehash(bins), m_bins(bins > 0 ? bins : hash_default_bins), m_size(0), m_values(0)
{ 
  m_values = new hash_element<K,T>*[m_bins];
  for (unsigned i = 0; i < m_bins; i++)
    m_values[i] = 0;
}

template<typename K, typename T, class H, class E>
ChHashTable<K,T,H,E>::~ChHashTable(void)
{
  // delete all the elements
  clear();
  // and delete the data structure
  delete[] m_values;
  m_values = 0;
}

// as usual, implement the copy constructor i.t.o. the assignment operator

template<typename K, typename T, class H, class E>
ChHashTable<K,T,H,E>::ChHashTable(const ChHashTable<K,T,H,E>& right) :
  m_rehash(right.m_rehash), m_bins(right.m_bins), m_size(0), m_values(0)
{
  m_values = new hash_element<K,T>*[right.m_bins];
  // copy the rehash behaviour as well as the size
  for (unsigned i = 0; i < m_bins; i++)
    m_values[i] = 0;
  *this = right;
}

// assignment operator
// this is done by copying the elements
// the source and target hashes can be different sizes
// the hash is self-copy safe, i.e. it is legal to say x = x;

template<typename K, typename T, class H, class E>
ChHashTable<K,T,H,E>& ChHashTable<K,T,H,E>::operator = (const ChHashTable<K,T,H,E>& r)
{
  // make self-copy safe
  if (&r == this) return *this;
  // remove all the existing elements
  clear();
  // copy the elements across - remember that this is rehashing because the two
  // tables can be different sizes so there is no quick way of doing this by
  // copying the lists
  for (typename ChHashTable<K,T,H,E>::const_iterator i = r.begin(); i != r.end(); ++i)
    insert(i->first, i->second);
  return *this;
}

// number of values in the hash

template<typename K, typename T, class H, class E>
bool ChHashTable<K,T,H,E>::empty(void) const
{
  return m_size == 0;
}

template<typename K, typename T, class H, class E>
unsigned ChHashTable<K,T,H,E>::size(void) const
{
  return m_size;
}

// equality

template<typename K, typename T, class H, class E>
bool ChHashTable<K,T,H,E>::operator == (const ChHashTable<K,T,H,E>& right) const
{
  // this table is the same as the right table if they are the same table!
  if (&right == this) return true;
  // they must be the same size to be equal
  if (m_size != right.m_size) return false;
  // now every key in this must be in right and have the same data
  for (typename ChHashTable<K,T,H,E>::const_iterator i = begin(); i != end(); i++)
  {
    typename ChHashTable<K,T,H,E>::const_iterator found = right.find(i->first);
    if (found == right.end()) return false;
    if (!(i->second == found->second)) return false;
  }
  return true;
}

template<typename K, typename T, class H, class E>
bool ChHashTable<K,T,H,E>::operator != (const ChHashTable<K,T,H,E>& right) const
{
  return !operator==(right);
}

// set up the ChHashTable to auto-rehash at a specific size
// setting the rehash size to 0 forces manual rehashing

template<typename K, typename T, class H, class E>
void ChHashTable<K,T,H,E>::auto_rehash(void)
{
  m_rehash = m_bins;
}

template<typename K, typename T, class H, class E>
void ChHashTable<K,T,H,E>::manual_rehash(void)
{
  m_rehash = 0;
}

// the rehash function
// builds a new hash table and moves the elements (without copying) from the old to the new
// I store the un-modulused hash value in the element for more efficient rehashing
// passing 0 to the bins parameter does auto-rehashing
// passing any other value forces the number of bins

template<typename K, typename T, class H, class E>
void ChHashTable<K,T,H,E>::rehash(unsigned bins)
{
  // user specified size: just take the user's value
  // auto calculate: if the load is high, increase the size; else do nothing
  unsigned new_bins = bins ? bins : m_bins;
  if (bins == 0 && m_size > 0)
  {
    // these numbers are pretty arbitrary
    // TODO - make them user-customisable?
    float load = loading();
    if (load > 2.0)
      new_bins = (unsigned)(m_bins * load);
    else if (load > 1.0)
      new_bins = m_bins * 2;
  }
  if (new_bins == m_bins) return;
  // set the new rehashing point if auto-rehashing is on
  if (m_rehash) m_rehash = new_bins;
  // move aside the old structure
  hash_element<K,T>** old_values = m_values;
  unsigned old_bins = m_bins;
  // create a replacement structure
  m_values = new hash_element<K,T>*[new_bins];
  for (unsigned i = 0; i < new_bins; i++)
    m_values[i] = 0;
  m_bins = new_bins;
  // move all the old elements across, rehashing each one
  for (unsigned j = 0; j < old_bins; j++)
  {
    while(old_values[j])
    {
      // unhook from the old structure
      hash_element<K,T>* current = old_values[j];
      old_values[j] = current->m_next;
      // rehash using the stored un-modulused hash value
      unsigned hash_value = current->m_hash % m_bins;
      // hook it into the new structure
      current->m_next = m_values[hash_value];
      m_values[hash_value] = current;
    }
  }
  // now delete the old structure
  delete[] old_values;
}

// the loading is the average number of elements per bin
// this simplifies to the total elements divided by the number of bins

template<typename K, typename T, class H, class E>
float ChHashTable<K,T,H,E>::loading(void) const
{
  return (float)m_size / (float)m_bins;
}

// remove all elements from the table

template<typename K, typename T, class H, class E>
void ChHashTable<K,T,H,E>::erase(void)
{
  // unhook the list elements and destroy them
  for (unsigned i = 0; i < m_bins; i++)
  {
    hash_element<K,T>* current = m_values[i];
    while(current)
    {
      hash_element<K,T>* next = current->m_next;
      delete current;
      current = next;
    }
    m_values[i] = 0;
  }
  m_size = 0;
}

// test for whether a key is present in the table

template<typename K, typename T, class H, class E>
bool ChHashTable<K,T,H,E>::present(const K& key) const
{
  return find(key) != end();
}

template<typename K, typename T, class H, class E>
typename ChHashTable<K,T,H,E>::size_type ChHashTable<K,T,H,E>::count(const K& key) const
{
  return present() ? 1 : 0;
}

// add a key and data element to the table - defined in terms of the general-purpose pair insert function

template<typename K, typename T, class H, class E>
typename ChHashTable<K,T,H,E>::iterator ChHashTable<K,T,H,E>::insert(const K& key, const T& data)
{
  return insert(std::pair<const K,T>(key,data)).first;
}

// insert a key/data pair into the table
// this removes any old value with the same key since there is no multihash functionality

template<typename K, typename T, class H, class E>
std::pair<typename ChHashTable<K,T,H,E>::iterator, bool> ChHashTable<K,T,H,E>::insert(const std::pair<const K,T>& value)
{
  // if auto-rehash is enabled, implement the auto-rehash before inserting the new value
  // the table is rehashed if this insertion makes the loading exceed 1.0
  if (m_rehash && (m_size >= m_rehash)) rehash();
  // calculate the new hash value
  unsigned hash_value_full = H()(value.first);
  unsigned hash_value = hash_value_full % m_bins;
  bool inserted = true;
  // unhook any previous value with this key
  // this has been inlined from erase(key) so that the hash value is not calculated twice
  hash_element<K,T>* previous = 0;
  for (hash_element<K,T>* current = m_values[hash_value]; current; previous = current, current = current->m_next)
  {
    // first check the full stored hash value
    if (current->m_hash != hash_value_full) continue;

    // next try the equality operator
    if (!E()(current->m_value.first, value.first)) continue;

    // unhook this value and destroy it
    if (previous)
      previous->m_next = current->m_next;
    else
      m_values[hash_value] = current->m_next;
    delete current;
    m_size--;

    // we've overwritten a previous value
    inserted = false;
    
    // assume there can only be one match so we can give up now
    break;
  }
  // now hook in a new list element at the start of the list for this hash value
  hash_element<K,T>* new_item = new hash_element<K,T>(value, hash_value_full);
  new_item->m_next = m_values[hash_value];
  m_values[hash_value] = new_item;
  // remember to increment the size count
  m_size++;
  // construct an iterator from the list node, and return whether inserted
  return std::make_pair(ChHashTable<K,T,H,E>::iterator(this,hash_value,new_item), inserted);
}

// insert a key with an empty data field ready to be filled in later

template<typename K, typename T, class H, class E>
typename ChHashTable<K,T,H,E>::iterator ChHashTable<K,T,H,E>::insert(const K& key)
{
  return insert(key,T());
}

// remove a key from the table - return true if the key was found and removed, false if it wasn't present

template<typename K, typename T, class H, class E>
bool ChHashTable<K,T,H,E>::erase(const K& key)
{
  unsigned hash_value_full = H()(key);
  unsigned hash_value = hash_value_full % m_bins;
  // scan the list for an element with this key
  // need to keep a previous pointer because the lists are single-linked
  hash_element<K,T>* previous = 0;
  for (hash_element<K,T>* current = m_values[hash_value]; current; previous = current, current = current->m_next)
  {
    // first check the full stored hash value
    if (current->m_hash != hash_value_full) continue;
    
    // next try the equality operator
    if (!E()(current->m_value.first, key)) continue;

    // found this key, so unhook the element from the list
    if (previous)
      previous->m_next = current->m_next;
    else
      m_values[hash_value] = current->m_next;
    // destroy it
    delete current;
    // remember to maintain the size count
    m_size--;
    return true;
  }
  return false;
}


template<typename K, typename T, class H, class E>
void ChHashTable<K,T,H,E>::clear(void)
{
  erase();
}

// search for a key in the table and return an iterator to it
// if the search fails, returns an end() iterator

template<typename K, typename T, class H, class E>
typename ChHashTable<K,T,H,E>::const_iterator ChHashTable<K,T,H,E>::find(const K& key) const
{
  // scan the list for this key's hash value for the element with a matching key
  unsigned hash_value_full = H()(key);
  unsigned hash_value = hash_value_full % m_bins;
  for (hash_element<K,T>* current = m_values[hash_value]; current; current = current->m_next)
  {
    if (current->m_hash == hash_value_full && E()(current->m_value.first, key))
      return ChHashTable<K,T,H,E>::const_iterator(this, hash_value, current);
  }
  return end();
}

template<typename K, typename T, class H, class E>
typename ChHashTable<K,T,H,E>::iterator ChHashTable<K,T,H,E>::find(const K& key)
{
  // scan the list for this key's hash value for the element with a matching key
  unsigned hash_value_full = H()(key);
  unsigned hash_value = hash_value_full % m_bins;
  for (hash_element<K,T>* current = m_values[hash_value]; current; current = current->m_next)
  {
    if (current->m_hash == hash_value_full && E()(current->m_value.first, key))
      return ChHashTable<K,T,H,E>::iterator(this, hash_value, current);
  }
  return end();
}

// table lookup by key using the index operator[], returning a reference to the data field, not an iterator
// this is rather like the std::map's [] operator
// the difference is that I have a const and non-const version
// the const version will not create the element if not present already, but the non-const version will
// the non-const version is compatible with the behaviour of the map

template<typename K, typename T, class H, class E>
const T& ChHashTable<K,T,H,E>::operator[] (const K& key) const
{
  // this const version cannot change the hash, so has to raise an exception if the key is missing
  // TODO make this a proper exception with a throw declaration
  typename ChHashTable<K,T,H,E>::const_iterator found = find(key);
  DEBUG_ASSERT(found != end());
  return found->second;
}

template<typename K, typename T, class H, class E>
T& ChHashTable<K,T,H,E>::operator[] (const K& key)
{
  // this non-const version can change the ChHashTable, so creates a new element if the key is missing
  typename ChHashTable<K,T,H,E>::iterator found = find(key);
  if (found == end())
    found = insert(key);
  return found->second;
}

// iterators

template<typename K, typename T, class H, class E>
typename ChHashTable<K,T,H,E>::const_iterator ChHashTable<K,T,H,E>::begin(void) const
{
  return ChHashTable<K,T,H,E>::const_iterator(this,0,0);
}

template<typename K, typename T, class H, class E>
typename ChHashTable<K,T,H,E>::iterator ChHashTable<K,T,H,E>::begin(void)
{
  return ChHashTable<K,T,H,E>::iterator(this,0,0);
}

template<typename K, typename T, class H, class E>
typename ChHashTable<K,T,H,E>::const_iterator ChHashTable<K,T,H,E>::end(void) const
{
  return ChHashTable<K,T,H,E>::const_iterator(this,m_bins,0);
}

template<typename K, typename T, class H, class E>
typename ChHashTable<K,T,H,E>::iterator ChHashTable<K,T,H,E>::end(void)
{
  return ChHashTable<K,T,H,E>::iterator(this,m_bins,0);
}









} // END_OF_NAMESPACE____




#endif  // END of ChHashTable.h
