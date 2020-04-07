#ifndef __DATATYPES_H__
#define __DATATYPES_H__

#include <iostream>
#include <algorithm>

#ifdef WIN32
    #if _MSC_VER < 1900 // prior to VS2015
        #define snprintf sprintf_s
    #endif
#endif

// Some data type defines to allow us to build on linux
#ifndef WIN32
  #ifndef __int64
  typedef long long __int64;
  #endif
  #ifndef WORD
      typedef unsigned short WORD;
  #endif
  #ifndef DWORD
      typedef unsigned long DWORD;
  #endif
  #ifndef BOOL
      typedef int BOOL;
  #endif
  #ifndef LONG
      typedef long LONG;
  #endif
  #ifndef HANDLE
      typedef void* HANDLE;
  #endif
  #ifndef BYTE
      typedef unsigned char BYTE;
  #endif
  #ifndef LPCWSTR
      typedef const wchar_t* LPCWSTR;
  #endif
  #ifndef LPCSTR
      typedef const char* LPCSTR;
  #endif
  #ifdef UNICODE
      typedef LPCWSTR LPCTSTR;
  #else
      typedef LPCSTR LPCTSTR;
  #endif
  #ifndef SOCKET
      typedef int SOCKET;
  #endif
  #ifndef SOCKET_ERROR
      const int SOCKET_ERROR = -1;
  #endif
  #ifndef cdecl
    //#define cdecl __attribute__((cdecl))
    #define cdecl
  #endif
  #ifndef FALSE
      #define FALSE 0
  #endif
  #ifndef TRUE
      #define TRUE 1
  #endif
  #ifndef NULL
      #define NULL 0
  #endif
  #ifndef errno_t
      typedef int errno_t;
  #endif
  #ifndef INVALID_SOCKET
      const int INVALID_SOCKET = -1;
  #endif
  #ifndef INFINITE
      const int INFINITE = -1;
  #endif
  #ifndef WAIT_OBJECT_0
      const int WAIT_OBJECT_0 = 0x00000000L;
  #endif
  #ifndef WAIT_TIMEOUT
      const int WAIT_TIMEOUT = 0x00000102L;
  #endif
  #ifndef ERROR_SUCCESS
      const int ERROR_SUCCESS = 0;
  #endif
#else
  #ifndef TRACE
  #define TRACE
  #endif
  #ifndef _T
  #define _T(x)  TEXT(x)
  #endif
#endif

#endif //__DATATYPES_H__
