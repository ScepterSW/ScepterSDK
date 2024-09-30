#ifndef SCEPTER_DEFINE_H
#define SCEPTER_DEFINE_H

#include "Scepter_enums.h"
#include "Scepter_types.h"

#ifdef PS_EXPORT_ON
    #ifdef _WIN32
        #define SCEPTER_API_EXPORT __declspec(dllexport)
    #else
        #define SCEPTER_API_EXPORT __attribute__((visibility("default")))
    #endif
#else
    #ifdef _WIN32
        #define SCEPTER_API_EXPORT __declspec(dllimport)
    #else
        #define SCEPTER_API_EXPORT __attribute__((visibility("default")))
    #endif
#endif

#ifdef __cplusplus
    #define SCEPTER_C_API_EXPORT extern "C" SCEPTER_API_EXPORT
#else
    #define SCEPTER_C_API_EXPORT SCEPTER_API_EXPORT
    #define bool                 uint8_t
#endif

#endif /* SCEPTER_DEFINE_H */
