/******************************************************************************************
 *
 * \file    ecdlogdatatypesglobal.h
 * \author  Phil Eccles
 * \date    26-09-2016
 * \brief   Header file containing dll export and import definitions.
 *
 ******************************************************************************************/

#ifndef ECDLOGDATATYPESGLOBAL_H
#define ECDLOGDATATYPESGLOBAL_H

#ifdef ECDLOGDATATYPES_STATIC_LIBRARY
  #define ECDLOGDATATYPESSHARED_EXPORT
#else
  #if defined WIN32 || defined __CYGWIN__
    #ifdef ECDLOGDATATYPES_LIBRARY
      #ifdef __GNUC__
        #define ECDLOGDATATYPESSHARED_EXPORT __attribute__((dllexport))
      #else
        #define ECDLOGDATATYPESSHARED_EXPORT __declspec(dllexport)
      #endif
    #else
      #ifdef __GNUC__
        #define ECDLOGDATATYPESSHARED_EXPORT __attribute__((dllimport))
      #else
        #define ECDLOGDATATYPESSHARED_EXPORT __declspec(dllimport)
      #endif
    #endif
  #else
    #if __GNUC__ >= 4
      #define ECDLOGDATATYPESSHARED_EXPORT __attribute__((visibility("default")))
    #else
      #define ECDLOGDATATYPESSHARED_EXPORT
    #endif
  #endif
#endif


#endif // ECDLOGDATATYPESGLOBAL_H

