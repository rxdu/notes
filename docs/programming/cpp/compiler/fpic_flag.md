# -fPIC Flag

!!! info 

    This note was written with the assistance of ChatGPT.

There are different types of symbols in an object file, and their relocatability depends on their nature and usage within the code. Not every symbol inside an object file (.o) is relocatable. 

The following are the main categories:

* Local Symbols: These are symbols that are only used within the object file. They are typically relocatable because their final addresses are not determined until the linker combines the object files into an executable or a shared library.

* Global Symbols: These symbols are meant to be visible to other object files. They can be either defined within the object file or undefined (to be resolved by other object files or libraries). The relocatability of these symbols depends on whether they are defined or undefined:

* Defined Global Symbols: These symbols are relocatable in the sense that their addresses can change when the final executable or shared library is created.
Undefined Global Symbols: These symbols are resolved during the linking process. The linker will replace them with the actual addresses from other object files or libraries.
Absolute Symbols: These symbols have fixed addresses and are not relocatable. Their values are absolute and will not change during the linking process.

* Section Symbols: These symbols refer to the beginning of sections within the object file (like .text, .data, .bss). They are relocatable in the sense that the entire section might be moved, but the relative offsets within the section remain fixed.

* Debug Symbols: These are used for debugging purposes and may not be directly involved in relocation. However, they might still reference relocatable addresses.

When compiling a static library with GCC, the -fPIC (Position Independent Code) flag affects how the code is generated. Here is a detailed explanation of the differences and implications:

With -fPIC Flag:

* Position Independent Code: The -fPIC flag tells the compiler to generate position-independent code, which means the generated machine code does not depend on being located at a specific memory address.
* Shared Libraries: PIC is primarily used for creating shared libraries (dynamic libraries) since these libraries are loaded into different memory locations in different processes. Position-independent code can be relocated without modification.
Performance: Generating position-independent code typically involves using an additional register to hold the address of the global offset table (GOT). This can introduce a slight performance overhead due to additional indirections.
* Usage: While it is not strictly necessary to use -fPIC for static libraries, doing so allows the same object files to be used to create both static and shared libraries without recompilation.

Without -fPIC Flag:

* Position Dependent Code: Without the -fPIC flag, the compiler generates position-dependent code. This means the generated code expects to be loaded at a specific address in memory.
Static Libraries: Position-dependent code is generally acceptable for static libraries because these libraries are linked directly into the executable, and the linker can resolve addresses at link time.
* Performance: Position-dependent code can be slightly more efficient than position-independent code since it does not require the extra indirections needed for position independence.
Flexibility: Object files compiled without -fPIC cannot be used to create shared libraries without recompilation, limiting their flexibility.

Key Differences:

* Relocability: Code compiled with -fPIC can be relocated at runtime, making it suitable for shared libraries, whereas code without -fPIC is fixed at link time.
* Performance: Position-independent code may have a slight performance penalty due to additional indirections.
* Flexibility: Using -fPIC makes the compiled object files more flexible, allowing them to be used in both static and shared libraries.

Summary:

* Use -fPIC: When you intend to create shared libraries or want the flexibility of using the same object files for both static and shared libraries.
* Do Not Use -fPIC: When you are certain that the object files will only be used to create static libraries and you want to avoid the slight performance overhead.

In practice, if you are working on a project that might require shared libraries in the future, it is generally a good idea to compile with -fPIC to keep your options open.