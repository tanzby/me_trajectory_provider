#pragma once

#include <cstddef>

// Put this in the declarations for a class to be uncopyable.
#define DISABLE_COPY(TypeName) TypeName(const TypeName &) = delete

// Put this in the declarations for a class to be unassignable.
#define DISABLE_ASSIGN(TypeName) void operator=(const TypeName &) = delete

// A macro to disallow the copy constructor and operator= functions.
// This should be used in the private: declarations for a class.
#define DISABLE_COPY_AND_ASSIGN(TypeName) \
  DISABLE_COPY(TypeName);                 \
  DISABLE_ASSIGN(TypeName)

// A macro to disallow all the implicit constructors, namely the
// default constructor, copy constructor and operator= functions.
//
// This should be used in the private: declarations for a class
// that wants to prevent anyone from instantiating it. This is
// especially useful for classes containing only static methods.
#define DISABLE_IMPLICIT_CONSTRUCTORS(TypeName) \
  TypeName() = delete;                          \
  DISABLE_COPY_AND_ASSIGN(TypeName)

// Creates a thread-safe singleton.
#define MAKE_SINGLETON(TypeName) \
 public:                         \
  static TypeName *Instance() {  \
    static TypeName Instance;    \
    return &Instance;            \
  }                              \
                                 \
 private:                        \
  DISABLE_COPY_AND_ASSIGN(TypeName)