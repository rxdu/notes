# Static vs. Dynamic Libraries

!!! info 

    This note was written with the assistance of ChatGPT.

In C++, libraries are crucial for sharing code and functionalities across different programs. There are two main types of libraries: static and dynamic. Understanding the differences between them, how they manage symbols, and their respective advantages and disadvantages is essential for efficient C++ programming.

## Static Libraries

### Definition
A static library is a collection of object files that are linked into the program during the build process. The linker combines the static library with the program's object files to create a single executable.

### Characteristics

* File Extension: Typically .a on Unix/Linux and .lib on Windows.
* Linking Time: Linked at compile-time.
* Symbol Resolution: Symbols are resolved during the linking stage, and the required code from the static library is copied into the executable.
* Portability: Resulting executable is self-contained, with no dependencies on external library files at runtime.
* Memory Usage: Can lead to larger executables since the library code is included in every executable that uses it.

### Advantages

* Performance: Slightly faster at runtime because all symbols are resolved during linking.
* Deployment: Easier deployment since there are no external dependencies.
* Stability: Less prone to issues caused by library version changes.

### Disadvantages

* Size: Larger executable size due to included library code.
* Update Complexity: Updating a library requires recompiling the entire application.
* Dynamic Libraries

### Symbol Visibility

In static libraries, all symbols from the object files are included in the library. The linker uses these symbols when creating an executable. By default, all **non-static** functions and global variables are visible and can be linked by other object files or libraries.

* Static Functions and Variables: Functions and variables declared with the static keyword are local to the translation unit (source file) and are not visible outside it.
* Anonymous Namespaces: In C++, an anonymous namespace can be used to achieve similar encapsulation for variables and functions.

### Symbol Management

In static libraries, symbols are resolved during the linking stage. When a program depends on a static library, the linker copies the necessary symbols (functions, variables) from the library into the executable. This process includes:

* Symbol Table: The linker reads the symbol table of the library to identify required symbols.
Copying Symbols: Only the used symbols are copied into the final executable, making it self-contained.
* Resolution: All external references to these symbols are resolved during the linking process, ensuring no unresolved symbols remain.

## Dynamic Libraries

### Definition

A dynamic library (also known as shared library) is not included in the executable at compile-time. Instead, it is loaded at runtime by the operating system.

### Characteristics

* File Extension: Typically .so on Unix/Linux and .dll on Windows.
* Linking Time: Linked at runtime.
* Symbol Resolution: Symbols are resolved when the program loads the library.
* Portability: Requires the presence of the library file at runtime.
* Memory Usage: Multiple programs can share a single copy of the dynamic library, reducing overall memory usage.

### Advantages

* Size: Smaller executable size since the library code is not included.
* Flexibility: Easier to update as the library can be replaced without recompiling the executable.
* Memory Efficiency: Shared among multiple programs, saving memory.

### Disadvantages

* Performance: Slight overhead due to symbol resolution at runtime.
* Dependency Management: Requires careful management of library versions and dependencies.
* Stability: Potential issues with library version compatibility (DLL Hell).

### Symbol Visibility

Dynamic libraries have more sophisticated mechanisms to control symbol visibility. By default, all non-static symbols are visible, but this can be controlled using visibility attributes and export maps.

* Default Visibility: Functions and variables without any specific visibility attributes are visible to the dynamic linker.
* Hidden Visibility: Using compiler-specific attributes to hide symbols.

### Symbol Management

In dynamic libraries, symbols are resolved at runtime. The process includes:

* Symbol Table: The dynamic linker/loader uses the symbol table in the dynamic library to resolve symbols when the program starts or when a library is explicitly loaded.
* Dynamic Linking: The operating system loads the dynamic library into memory and links the symbols dynamically.
* Lazy Binding: In some systems, symbols are resolved on the first use rather than at load time, optimizing startup time.

## Dependency between Libraries

* Static Library A Depends on Static Library B

If a static library libA depends on another static library libB, the symbols from libB are not copied over to libA. Instead, when you build an executable that depends on libA, you will still need to link against both libA and libB.

* Dynamic Library Depends on Dynamic Library
  
When a dynamic library libX depends on another dynamic library libY, libX will contain references to the symbols in libY, but the actual symbol resolution is deferred until runtime. You will need to ensure that both libX.so and libY.so are available at runtime.

* Dynamic Library Depends on Static Library

If a dynamic library libX depends on a static library libY, the symbols from libY are included in libX during its creation. Therefore, you do not need libY when linking your executable against libX, as libX already contains all necessary symbols.

* Static Library Depends on Dynamic Library
  
Static libraries cannot directly depend on dynamic libraries because static libraries are archives of object files that are linked at compile time, while dynamic libraries are linked at runtime. If you need to use symbols from a dynamic library in a static library, the executable or another dynamic library that includes the static library must also link against the dynamic library.

The above 4 cases can be summarized as:

* Static Library A depends on Static Library B: You need **both libA.a and libB.a at link time** to create the executable.
* Dynamic Library X depends on Dynamic Library Y: You **need libY.so at runtime to resolve the dependencies of libX.so**.
* Dynamic Library X depends on Static Library Y: The symbols from libY.a are included in libX.so, so you **only need libX.so** at runtime.
* Static Library A depends on Dynamic Library Y: When linking an executable or another dynamic library that uses libA.a, you **need libY.so to resolve symbols used by libA.a**.

