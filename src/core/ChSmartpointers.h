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
			/// Thank to automatic reference counting, you never have to call delete()!
    explicit ChSmartPtr(T* p = 0) 
        : itsCounter(0) 
			{
				if (p) itsCounter = new ChCounter (p);
			}

			/// Copy constructor for the case 
			///  ChSmartPtr<MyClassA> pointerA(pointerB);
    ChSmartPtr(const ChSmartPtr& r) throw()
			{
				acquire(r);
			}

			/// Copy constructor and converter for the case 
			///  ChSmartPtr<MyClassA> pointerA(pointerB);
			/// when pointerB comes from a class MyClassB which is inherited from MyClassA. 
			/// If casting is not possible, the created shared pointer is invalidated (IsNull() = true).
			/// Warnings! - upcast (MyClassA is parent of MyClassB) exactness and..
			///			  - downcast (MyClassA is child of MyClassB) exactness  
			///				is done in runtime with dynamic_cast; MyClassA & MyClassB must be polimorphic.
			///           - MyClassA & MyClassB MUST have virtual destructors, if you want to use casting!
	template <class T_other>
	ChSmartPtr(const ChSmartPtr<T_other>& r) throw()
			{
				if (dynamic_cast<T*>(r.get_ptr()))
					acquire (r);
				else
					itsCounter = 0;
			}

			/// Destructor decrements the reference count and automatically delete only 
			/// when the last reference is destroyed
	~ChSmartPtr()
			{
				release();
			}

			/// Bool type casting, true if the pointer is still bound to an object, or
			/// false if unbound and invalidated (ex. after unsuccesfull casting). Example:
			///    if(mysharedptr) {...}
	operator bool() const { return itsCounter ? itsCounter->ptr!=0 : 0; }

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

			/// Occasionally, the shared pointer can be invalidated (unbound from
			/// object), for instance if you create it with null default
			/// ChSharedPtr<MyClass> pointerA;   instead of typical   
			/// ChSharedPtr<MyClass> pointerA(new MyClass);
	bool IsNull() const throw() {return itsCounter ? itsCounter->ptr==0 : true;};

			/// Unbind the shared pointer from referenced shared object,
			/// and automatically delete in case of last reference. It should
			/// be used sparingly, because this unbinding already happens automatically
			/// when the shared pointer dies. Use this only to 'force' premature unbinding.
    void SetNull() { this->release(); }

			/// Tells if the referenced object is inherited from a specific class
			/// and can be cast with copy constructor, for example 
			///   if (ptrA.IsType<classB>() ) { ChSharedPtr<classB> ptrB (ptrA); }
	template <class T_other>
	bool IsType() { return itsCounter ? dynamic_cast<T_other*>(this->get_ptr()) : false ;}

private:

	class ChCounter 
	{
	public:
        ChCounter(T* p = 0, unsigned int c = 1) : ptr(p), count(c) {}
        T*           ptr;
        unsigned int count;
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
					if (--itsCounter->count == 0) 
					{
						delete (T*)(itsCounter->ptr);
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
			///   ChSharedPtr<MyClass> pointerA(new MyClass);
			/// Thank to automatic reference counting, you never have to call delete()!
    explicit ChSharedPtr(T* p = 0) 
			{
				ptr = p;
			}

			/// Copy constructor for the case 
			///   ChSharedPtr<MyClassA> pointerA(pointerB);
	ChSharedPtr(const ChSharedPtr& r) throw()
			{
				acquire(r);
			}

			/// Copy constructor and converter for the case 
			///  ChSharedPtr<MyClassA> pointerA(pointerB);
			/// when pointerB comes from a class MyClassB which is inherited from MyClassA or viceversa.
			/// If casting is not possible, the created shared pointer is invalidated (IsNull() = true). 
			/// Warnings! - upcast (MyClassA is parent of MyClassB) exactness and..
			///			  - downcast (MyClassA is child of MyClassB) exactness  
			///				is done in runtime with dynamic_cast; MyClassA & MyClassB must be polimorphic.
			///           - MyClassA & MyClassB MUST have virtual destructors, if you want to use casting!
	template <class T_other>
	ChSharedPtr(const ChSharedPtr<T_other>& r) throw()
			{
				if (dynamic_cast<T*>(r.get_ptr()))
					acquire (r);
				else
					ptr = 0;
			}

			/// Destructor decrements the reference count and automatically delete only 
			/// when the last reference is destroyed
	~ChSharedPtr()
			{
				release();
			}


			/// Bool type casting, true if the pointer is still bound to an object, or
			/// false if unbound and invalidated (ex. after unsuccesfull casting). Example:
			///    if(mysharedptr) {...}
	operator bool() const { return ptr!=0; }

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

			/// Occasionally, the shared pointer can be invalidated (unbound from
			/// object), for instance if you create it with null default
			/// ChSharedPtr<MyClass> pointerA;   instead of typical   
			/// ChSharedPtr<MyClass> pointerA(new MyClass);
	bool IsNull() const throw() {return ptr == 0;};

			/// Unbind the shared pointer from referenced shared object,
			/// and automatically delete in case of last reference. It should
			/// be used sparingly, because this unbinding already happens automatically
			/// when the shared pointer dies. Use this only to 'force' premature unbinding.
    void SetNull() { this->release(); ptr =0;}

			/// Tells if the referenced object is inherited from a specific class
			/// and can be cast with copy constructor, for example 
			///   if (ptrA.IsType<classB>() ) { ChSharedPtr<classB> ptrB (ptrA); }
	template <class T_other>
	bool IsType() {return (bool)dynamic_cast<T_other*>(ptr); }

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
