---
title: Convert C++ Function Pointer to C Function Pointer
description: 
---

Sometimes, you may want to pass a C++ member function as the callback function to a C library API function. Likely people will tell you it's only possible if the member function is static. But here is one possible solution  

```cpp
template <typename T>
struct Callback;

template <typename Ret, typename... Params>
struct Callback<Ret(Params...)> {
    template <typename... Args>
    static Ret callback(Args... args) { return func(args...); }
    static std::function<Ret(Params...)> func;
};

// Initialize the static member.
template <typename Ret, typename... Params>
std::function<Ret(Params...)> Callback<Ret(Params...)>::func;

//////////////////////////////////////////////

struct Foo {
    void print(int* x) { // Some member function.
        std::cout << *x << std::endl;
    }
};

int main() {
    Foo foo; // Create instance of Foo.

    // Store member function and the instance using std::bind.
    Callback<void(int*)>::func = std::bind(&Foo::print, foo, std::placeholders::_1);

    // Convert callback-function to c-pointer.
    void (*c_func)(int*) = static_cast<decltype(c_func)>(Callback<void(int*)>::callback);

    // Use in any way you wish.
    std::unique_ptr<int> iptr{new int(5)};
    c_func(iptr.get());
}
```

* Reference: https://stackoverflow.com/a/19809787/2200873