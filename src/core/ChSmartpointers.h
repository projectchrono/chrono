#ifndef CHSMARTPOINTERS_H
#define CHSMARTPOINTERS_H

//////////////////////////////////////////////////
//  
//   ChSmartpointers.h
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <stdexcept>


namespace chrono 
{


///  Smart pointer to be used with generic objects to be shared
/// among multiple pointers, without the risk of deleting the 
/// pointed instance too early, hence avoids dangling pointers.
///  It does not need that instances are inherited from ChShared 
/// because it uses an auxiliary 
/// reference counter which is allocated on heap).
///  It should be initialized in the following two main ways:
///    ChSmartPtr<MyClass> foo_pointer(new MyClass);
///       or
///    ChSmartPtr<MyClass> foo_pointer(another_smart_pointer);
///  Doing so, you will _never_ need to call 'delete', because the reference
/// count mechanism will automatically delete the instance when no smart pointers 
/// reference it.
///  This kind of pointer does not perform copy of pointed data when copied.

template <class T> 
class ChSmartPtr
{
public:
    typedef T element_type;

			/// Constructor for initializing with dynamically allocated data, with new()
			/// Mandatory way of using it:
			///  ChSmartPtr<MyClass> pointerA(new MyClass);
    explicit ChSmartPtr(T* p = 0) 
        : itsCounter(0) 
			{
				if (p) itsCounter = new counter(p);
			}

			/// Copy constructor for the case 
			///  ChSmartPtr<MyClassA> pointerA(pointerB);
    ChSmartPtr(const ChSmartPtr& r) throw()
			{acquire(r);}

			/// Copy constructor and converter for the case 
			///  ChSmartPtr<MyClassA> pointerA(pointerB);
			/// when pointerB comes from a class MyClassB which is inherited from MyClassA. 
			/// Warnings! - no check on MyClassB being really inherited from MyClassB,
			///           - MyClassA & children should have virtual destructors.
	template <class T_other>
	ChSmartPtr(const ChSmartPtr<T_other>& r) throw()
			{acquire (r);}

			/// Destructor decrements the reference count and automatically delete only 
			/// when the last reference is destroyed
	~ChSmartPtr()
			{release();}

			/// Assignment form for an already-constructed smart-pointer.
    ChSmartPtr& operator=(const ChSmartPtr& r)
			{
				if (this != &r) {
					release();
					acquire(r);
				}
				return *this;
			}

			/// Dereference the smart pointer to get the object, as in the form *p1
    T& operator*()   throw()   {return *itsCounter->ptr;}
	const T& operator*()  const throw()   {return *itsCounter->ptr;}

			/// Used for member access to the contained object,
			/// e.g. pointer->Print() calls T::Print()
    T* operator->()  throw()   {return itsCounter->ptr;}
	const T* operator->(void) const throw() {return itsCounter->ptr;}

			/// Tells if this is shared by no one else.
    bool IsUnique()   const throw()
        {return (itsCounter ? itsCounter->count == 1 : true);}

			/// Returns the raw pointer to pointed instance.
			/// Note: If a correct programming style is used, you should 
			/// never need to use this.
	T* get_ptr() const throw() {return itsCounter ? itsCounter->ptr : 0;} 
	T* get()	 const throw() {return itsCounter ? itsCounter->ptr : 0;}

private:

    struct counter 
	{
        counter(T* p = 0, unsigned c = 1) : ptr(p), count(c) {}
        T*          ptr;
        unsigned    count;
    }* itsCounter;

			// increment the count
	template <class T_other>
    void acquire(const ChSmartPtr<T_other> &r) throw()
			{ 
				itsCounter = r.itsCounter;
				if (itsCounter) ++itsCounter->count;
			}

			// decrement the count, delete if it is 0
    void release()
			{ 
				if (itsCounter) {
					if (--itsCounter->count == 0) {
						delete itsCounter->ptr;
						delete itsCounter;
					}
					itsCounter = 0;
				}
			}
};

// Comparisons operators are required for using the shared pointer
// class in an STL container

template<typename T>
bool operator==(const ChSmartPtr<T>& left, const ChSmartPtr<T>& right)
{
	if (left.get_ptr() == right.get_ptr()) return true;
	return *left == *right;
}
template<typename T>
bool operator<(const ChSmartPtr<T>& left, const ChSmartPtr<T>& right)
{
	if (left.get_ptr() == right.get_ptr()) return false;
	return *left < *right;
}




/////////////////////////////////////////////////////////////////////



///  Smart pointer to be used with generic objects to be shared
/// among multiple pointers, without the risk of deleting the 
/// pointed instance too early, hence avoids dangling pointers.
///  When compared to the ChSharedPtr, this is a faster version
/// but requires that you use it only for data which is instanced
/// from the ChShared class.
///  It should be initialized in the following two main ways:
///    ChSmartPtr<MyClass> foo_pointer(new MyClass);
///       or
///    ChSmartPtr<MyClass> foo_pointer(another_smart_pointer);
///  Doing so, you will _never_ need to call 'delete', because the reference
/// count mechanism will automatically delete the instance when no smart pointers 
/// reference it.
///  This kind of pointer does not perform copy of pointed data when copied.

template <class T> 
class ChSharedPtr
{

public:
    typedef T element_type;

			/// Constructor for initializing with dynamically allocated data, with new()
			/// Mandatory way of using it:
			///  ChSharedPtr<MyClass> pointerA(new MyClass);
    explicit ChSharedPtr(T* p = 0) 
			{
				ptr = p;
			}

			/// Copy constructor for the case 
			///  ChSharedPtr<MyClassA> pointerA(pointerB);
	ChSharedPtr(const ChSharedPtr& r) throw()
			{
				acquire(r);
			}

			/// Copy constructor and converter for the case 
			///  ChSharedPtr<MyClassA> pointerA(pointerB);
			/// when pointerB comes from a class MyClassB which is inherited from MyClassA or viceversa.
			/// Warnings! - upcast (MyClassA is parent of MyClassB) exactness and 
			///			  - downcast (MyClassA is child of MyClassB) exactness and 
			///           - check that pointerB is really inherited from MyClassB/MyClassA 
			///				is done in runtime in debug version only! via assert() and
			///             dynamic cast, so MyClassA & MyClassB must be polimorphic.
			///           - MyClassA & MyClassB should have virtual destructors, if you want to use casting.
	template <class T_other>
	ChSharedPtr(const ChSharedPtr<T_other>& r) throw()
			{
				assert(dynamic_cast<T*>(r.get_ptr()));
				acquire (r);
			}

			/// Destructor decrements the reference count and automatically delete only 
			/// when the last reference is destroyed
	~ChSharedPtr()
			{
				release();
			}

			/// Assignment form for an already-constructed smart-pointer.
    ChSharedPtr& operator=(const ChSharedPtr& r)
			{
				if (this != &r) 
				{
					release();
					acquire(r);
				}
				return *this;
			}

			/// Dereference the smart pointer to get the object, as in the form *p1
    T& operator*()  throw()   {return *ptr;}
	const T& operator*()  const throw()   {return *ptr;}

			/// Used for member access to the contained object,
			/// e.g. pointer->Print() calls T::Print()
    T* operator->() throw()   {return ptr;}
	const T* operator->(void) const throw() {return ptr;}


			/// Tells if this is shared by no one else.
    bool IsUnique()   const throw()
        {return (ptr ? ptr->ReferenceCount() == 1 : true);}

			/// Returns the raw pointer to pointed instance.
			/// Note: If a correct programming style is used, you should 
			/// never need to use this.
	T* get_ptr() const throw()  {return ptr;} 
    T* get()     const throw()  {return ptr;}

private:

	T*          ptr;

			// increment the count
	template <class T_other>
    void acquire(const ChSharedPtr<T_other>& r) throw()
			{ 
				ptr = (T*)(r.get_ptr());
				if (ptr)
				{
					ptr->AddRef();
				}
			}

			// decrement the count, delete if it is 0
    void release()
			{ 
				// this should automatically delete the object when its ref.counter goes to 0.
				if (ptr)
				{
					ptr->RemoveRef(); 
				}
			}
};

// Comparisons operators are required for using the shared pointer
// class in an STL container

template<typename T>
bool operator==(const ChSharedPtr<T>& left, const ChSharedPtr<T>& right)
{
	if (left.get_ptr() == right.get_ptr()) return true;
	return *left == *right;
}
template<typename T>
bool operator<(const ChSharedPtr<T>& left, const ChSharedPtr<T>& right)
{
	if (left.get_ptr() == right.get_ptr()) return false;
	return *left < *right;
}




} // END_OF_NAMESPACE____

#endif
