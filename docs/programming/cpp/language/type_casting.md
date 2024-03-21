# Type Casting

!!! info 

    This note was written with the assistance of ChatGPT.

In C++, `static_cast`, `dynamic_cast`, `reinterpret_cast` and `const_cast` serve different purposes for type conversion.

## Cast Types

### static_cast

`static_cast` is used for conversions between types where the compiler can check the validity of the conversion **at compile time**. It is used for standard conversions of numeric types, such as converting an `int` to a `float`, and for converting between pointer types when there is a clear inheritance relationship.
  
**Use cases:**

* Converting numerical types (e.g., `int` to `double`)
* Converting enums to integers or vice versa
* **Upcasting**: converting a derived class pointer or reference to a base class pointer or reference
* Note: **downcasting** can also be performed with static_cast, which avoids the cost of the runtime check, but it's only safe if the program can **guarantee** (through some other logic) that the object pointed to by expression is definitely derived

**Example:**
  
```cpp
int i = 10;
double d = static_cast<double>(i);  // Convert int to double

class Base {};
class Derived : public Base {};
Derived* derived = new Derived();
Base* base = static_cast<Base*>(derived);  // Upcasting
```

### dynamic_cast

`dynamic_cast` is used for safe downcasting in class hierarchies, i.e., converting a base class pointer or reference to a derived class pointer or reference. It checks the type at runtime and returns `nullptr` (for pointers) or throws `std::bad_cast` (for references) if the conversion is not possible.

**Use cases:**
  
* **Downcasting** in class hierarchies with polymorphic types (i.e., classes with virtual functions)
* Checking type at runtime in a class hierarchy
* Note: dynamic_cast may be used for **upcasting**, but unnecessary

**Example:**

```cpp
Base* base = new Derived();
Derived* derived = dynamic_cast<Derived*>(base);  // Downcasting

if (derived) {
    // Successfully downcasted
}
```

### reinterpret_cast

`reinterpret_cast` is used for low-level casts that yield implementation-dependent results. It converts any pointer type to any other pointer type, even if the types are unrelated, without any runtime type check.

**Use cases:**

* Converting pointers to and from integer types for low-level manipulation
* Treating a block of memory as an array of a different type
* Function pointer type conversions

**Example:**

```cpp
long p = 12345678;
char* cp = reinterpret_cast<char*>(&p);  // Treat the long int as an array of chars

void (*funcPtr)(int);
void* ptr = reinterpret_cast<void*>(funcPtr);  // Convert function pointer to void pointer
```

### const_cast

`const_cast` is used to add or remove the const qualifier from a variable, allowing for temporary changes in constness.

**Use Cases:**

* Modifying a previously declared const variable
* Passing const objects to functions that require non-const parameters

**Example:**

```cpp
const int x = 10;
int& nonConstRef = const_cast<int&>(x);
```

Each of these casts serves a specific purpose, and choosing the right one depends on the context and the level of safety and type checking required. `static_cast` is the safest and most commonly used for general type conversions. `dynamic_cast` is more specialized for safe downcasting in polymorphic class hierarchies. `reinterpret_cast` is the least safe and should be used sparingly, as it essentially allows treating any pointer as any other type of pointer.

## Dynamic Casting Safety

`dynamic_cast` can fail under certain conditions, typically when it's used for downcasting or sidecasting in class hierarchies. Here are the situations where `dynamic_cast` will not succeed:

1. **Type is not polymorphic:** If the type you're casting from does not have at least one virtual function, then `dynamic_cast` cannot be used. It relies on runtime type information (RTTI) to check the object's type at runtime, which is available only for polymorphic types.

2. **Invalid downcast or sidecast:** `dynamic_cast` will fail if you attempt to cast to a type that is not the actual type of the object or a derived type thereof. For example, if you try to downcast a base class pointer to a derived class pointer, but the actual object is not of that derived class, the cast will fail.

3. **Casting away constness improperly:** While `dynamic_cast` can be used to cast between types within an inheritance hierarchy, it cannot change the constness of the object being cast. If you need to cast away constness, `const_cast` must be used in conjunction.

4. **Cross-casting in multiple inheritance incorrectly:** In cases of multiple inheritance, if you attempt to cast from one base class to another where neither is a base of the other (sidecasting), and the object is not actually an instance of the target class, `dynamic_cast` will fail.

**Example**

- **Polymorphic Base Required:**
  ```cpp
  class Base { /* no virtual functions */ };
  class Derived : public Base { };
  
  Base* base = new Derived();
  Derived* derived = dynamic_cast<Derived*>(base);  // Fails, Base is not polymorphic
  ```

- **Invalid Downcast:**
  ```cpp
  class Base { virtual void func() {} };
  class Derived1 : public Base { };
  class Derived2 : public Base { };

  Base* base = new Derived1();
  Derived2* derived = dynamic_cast<Derived2*>(base);  // Fails, base is not a Derived2
  ```

- **Improper Constness Casting:**
  ```cpp
  const Base* base = new Derived();
  Derived* derived = dynamic_cast<Derived*>(base);  // Fails, const to non-const
  ```

To ensure `dynamic_cast` succeeds, you must cast between compatible types within a polymorphic class hierarchy and adhere to the constness rules. When `dynamic_cast` fails during pointer casting, it returns `nullptr`, and when it fails during reference casting, it throws a `std::bad_cast` exception.