
Shared pointers                 {#shared_pointers}
==============================

In Chrono::Engine, most 'heavyweight objects' such as rigid bodies, links, etc., are managed
via **shared pointers**. This means that the user is freed from the need to take care of
deleting objects after he creates them, because shared pointers provide a (limited)
garbage-collection facility, possibly sharing that management with other objects.

We use the C++11 compliant shared pointer from the STL library: ```std::shared_ptr```.

Objects of shared_ptr types have the ability of taking ownership of a
pointer and share that ownership: once they take ownership, 
the group of owners of a pointer become responsible for its 
deletion when the last one of them releases that ownership.

Therefore shared_ptr objects release ownership on the object 
they co-own as soon as they themselves are destroyed. 

Once all shared_ptr objects that share ownership over 
a pointer have released this ownership, the managed 
object is _automatically_ deleted.

The main effect of using shared_ptr is that the user will **never need to use delete**.
This avoids a lot of headache in memory management and avoids lot of errors with dangling pointers.

An example: create an object and handle it with a shared pointer:

~~~{.cpp}
std::shared_ptr<ChBody> my_body (new ChBody);
~~~

An equivalent, better alternative is to use ```std::make_shared```, that allows a bit faster performance:

~~~{.cpp}
auto my_body = std::make_shared<ChBody>();
~~~

Note that ```auto```, as in other C++11 cases, means that you do not have to specify 
```std::shared_ptr<ChBody> my_body = ...``` and it means less typing.

If parameters are needed during the construction:

~~~{.cpp}
auto my_foo = std::make_shared<Foo>(par1, par2);
~~~

One can also assign other pointers to the same shared object, as in these examples:

~~~{.cpp}
auto my_foo = std::make_shared<Foo>(par1, par2);
std::shared_ptr<Foo> my_fuu = my_foo;

auto my_fii = FunctionReturningSharedPointer();
~~~

Note that if you want to downcast shared pointers, you use ```dynamic_pointer_cast<...>(...)```,
also you can do a static cast via ```static_pointer_cast<...>(...)```,
as in these examples:

~~~{.cpp}
std::shared_ptr<Base> var1;
auto var2 = std::dynamic_pointer_cast<Derived>(var1);
auto var3 = std::static_pointer_cast<Derived>(var1);

if (auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper())) {
    mystepper->SetAlpha(-0.2);
    ...
}
~~~


## Porting from old (previous than v.3.0) versions of the API

<div class="ce-info">
The following section can be skipped if you do not have to port your code
from an older version of Chrono::Engine.
</div>

The older API of Chrono used a custom shared pointer called ```ChSharedPtr```.
This is made obsolete and it has been replaced by the ```std::shared_ptr``` discussed above.
If you want to port old code to the new (greater than v.3.0) versions of Chrono API, follow 
these guidelines:

- Types:
  - OLD:
  
		ChSharedPtr<Foo>
	
  - NEW: 
  
		std::shared_ptr<Foo>

- Creation of shared pointers:
  - OLD: 
  
		ChSharedPtr<Foo> my_var(new Foo(arg1, arg2));
	
  - NEW:
  
		auto my_var = std::make_shared<Foo>(arg1, arg2);

- if the constructor of Foo does not take any arguments, then:
  - OLD:
  
		ChSharedPtr<Foo>  my_var(new Foo);
	
  - NEW:
  
		auto my_var = std::make_shared<Foo>();
	
- if you have a variable already declared as a shared pointer somewhere else:
  - OLD:
  
		ChSharedPtr<Foo> my_var;
		my_var = ChSharedPtr<Foo>(new Foo(arg1, arg2));
	
  - NEW:
  
		std::shared_ptr<Foo> my_var;
		my_var = std::make_shared<Foo>(arg1, arg2);

- copy constructors
  - OLD:
  
		ChSharedPtr<Foo>  var1;
		ChSharedPtr<Foo> var2(var1);
		ChSharedPtr<Foo> var3 = ChSharedPtr<Foo>(var1);
  - NEW:
  
		std::shared_ptr<Foo> var1;
		std::shared_ptr<Foo> var2(var1);
		auto var3 = std::shared_ptr<Foo>(var1);

- constructor from raw pointer
  - OLD:
  
		Foo* my_pointer;
		ChSharedPtr<Foo> my_var(my_pointer);
	
  - NEW:
  
		Foo* my_pointer;
		std::shared_ptr<Foo> my_var(my_pointer);
	
- casting
  - OLD:
  
		ChSharedPtr<Base>  var1;
		ChSharedPtr<Derived> var2 = var1.DynamicCastTo<Derived>();
		ChSharedPtr<Derived> var3 = var1.StaticCastTo<Derived>();
	
  - NEW:
  
		std::shared_ptr<Base> var1;
		auto var2 = std::dynamic_pointer_cast<Derived>(var1);
		auto var3 = std::static_pointer_cast<Derived>(var1);
	

- access to the wrapped pointer
  - OLD:
  
		ChSharedPtr<Foo> my_var;
		Foo* my_pointer = my_var.get_ptr();
	
  - NEW:
  
		std::shared_ptr<Foo> my_var;
		Foo* my_pointer = my_var.get();
	

- testing for NULL (empty) shared pointer  (e.g. in an assert, or a conditional)
  - OLD:
  
		ChSharedPtr<Foo> my_var;
		assert(!my_var.IsNull());
	
  - NEW:
  
		std::shared_ptr<Foo> my_var;
		assert(my_var);
	

- testing for the type should be done explicitly with dynamic_pointer_cast
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
	
	  

