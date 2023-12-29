
Shared Pointers                 {#shared_pointers}
==============================

In Chrono, most of the 'heavyweight objects', such as rigid bodies, links, etc., are managed
via **shared pointers**. One consequence is that the user does not need to be concerned about 
deleting objects after creating them. Indeed, these shared pointers provide in some sense a basic garbage-collection facility. In this context, Chrono uses the C++11 compliant shared pointer from the STL library: ```std::shared_ptr```.

Objects of shared_ptr types have the ability of sharing the ownership of a
pointer. Once they take shared ownership of a pointer, 
the group of owners become responsible for its 
deletion when the last one of them releases that ownership.
In this context, its useful to keep in mind that shared_ptr objects release ownership on the object 
they co-own as soon as they themselves are destroyed. 

Once all shared_ptr objects that share ownership over 
a pointer have released this ownership, the managed 
object is _automatically_ deleted.

The main effect of using a shared_ptr object is that the user will **never need to use delete**. This simplifies memory management and avoids errors associated with dangling pointers.

Example: create an object and handle it with via a shared pointer.

~~~{.cpp}
std::shared_ptr<ChBody> my_body(new ChBody);
~~~

A better alternative is to use ```chrono_types::make_shared``` (which requires a single memory allocation and is a bit faster). However, ```chrono_types::make_shared``` will use the incorrect allocator for objects of classes that contain fixed-size vecorizable Eigen types.  Chrono provides an alternative function that automatically infers whether or not it is safe to fallback on using ```chrono_types::make_shared```.  As such, user code should **always** use 

~~~{.cpp}
auto my_body = chrono_types::make_shared<ChBody>();
~~~

Note that ```auto```, as in other C++11 cases, means that there is no need to specify 
```std::shared_ptr<ChBody> my_body = ...``` 


In the next example, parameters are needed in the constructor call:

~~~{.cpp}
auto my_foo = chrono_types::make_shared<Foo>(par1, par2);
~~~

Other pointers can also be assigned to the same shared object.
Examples:

~~~{.cpp}
auto my_foo = chrono_types::make_shared<Foo>(par1, par2);
std::shared_ptr<Foo> my_fuu = my_foo;

auto my_fii = FunctionReturningSharedPointer();
~~~

Downcasting shared pointers falls back on the use of ```dynamic_pointer_cast<...>(...)```. A static cast is done via ```static_pointer_cast<...>(...)```:

~~~{.cpp}
std::shared_ptr<Base> var1;
auto var2 = std::dynamic_pointer_cast<Derived>(var1);
auto var3 = std::static_pointer_cast<Derived>(var1);

if (auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper())) {
    mystepper->SetAlpha(-0.2);
    ...
}
~~~


## Porting from a pre 3.0 version of the Chrono API

Pre 3.0 versions of the Chrono API used the custom shared pointer class ```ChSharedPtr```. This was made obsolete and replaced by the ```std::shared_ptr```.
Modifying old code to use the v.3.0 and newer versions of the Chrono API can be straightforward if one follows these guidelines:

- Types:
  - OLD:
  
		ChSharedPtr<Foo>
	
  - NEW: 
  
		std::shared_ptr<Foo>

- Creation of shared pointers:
  - OLD: 
  
		ChSharedPtr<Foo> my_var(new Foo(arg1, arg2));
	
  - NEW:
  
		auto my_var = chrono_types::make_shared<Foo>(arg1, arg2);

- If the constructor of Foo does not take any arguments, then:
  - OLD:
  
		ChSharedPtr<Foo>  my_var(new Foo);
	
  - NEW:
  
		auto my_var = chrono_types::make_shared<Foo>();
	
- If a variable has already been declared as a shared pointer somewhere else:
  - OLD:
  
		ChSharedPtr<Foo> my_var;
		my_var = ChSharedPtr<Foo>(new Foo(arg1, arg2));
	
  - NEW:
  
		std::shared_ptr<Foo> my_var;
		my_var = chrono_types::make_shared<Foo>(arg1, arg2);

- Copy constructors
  - OLD:
  
		ChSharedPtr<Foo>  var1;
		ChSharedPtr<Foo> var2(var1);
		ChSharedPtr<Foo> var3 = ChSharedPtr<Foo>(var1);
  - NEW:
  
		std::shared_ptr<Foo> var1;
		std::shared_ptr<Foo> var2(var1);
		auto var3 = std::shared_ptr<Foo>(var1);

- Constructor from raw pointer
  - OLD:
  
		Foo* my_pointer;
		ChSharedPtr<Foo> my_var(my_pointer);
	
  - NEW:
  
		Foo* my_pointer;
		std::shared_ptr<Foo> my_var(my_pointer);
	
- Casting
  - OLD:
  
		ChSharedPtr<Base>  var1;
		ChSharedPtr<Derived> var2 = var1.DynamicCastTo<Derived>();
		ChSharedPtr<Derived> var3 = var1.StaticCastTo<Derived>();
	
  - NEW:
  
		std::shared_ptr<Base> var1;
		auto var2 = std::dynamic_pointer_cast<Derived>(var1);
		auto var3 = std::static_pointer_cast <Derived>(var1);
	

- Access to the wrapped pointer
  - OLD:
  
		ChSharedPtr<Foo> my_var;
		Foo* my_pointer = my_var.get_ptr();
	
  - NEW:
  
		std::shared_ptr<Foo> my_var;
		Foo* my_pointer = my_var.get();
	

- Testing for NULL (empty) shared pointer, e.g. in an assert or a conditional
  - OLD:
  
		ChSharedPtr<Foo> my_var;
		assert(!my_var.IsNull());
	
  - NEW:
  
		std::shared_ptr<Foo> my_var;
		assert(!my_var);
	

- Testing for the type should be done explicitly with dynamic_pointer_cast
  - OLD:
  
		ChSharedPtr<Bar> my_var;
		if (my_var.IsType<Foo>()) {
			....
		}
	
  - NEW:
  
		std::shared_ptr<Bar> my_var;
		if (std::dynamic_pointer_cast<Foo>(my_var)) {
			...
		}
	
	  

