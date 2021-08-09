
#pragma once

#if defined(_MSC_VER) && defined(LABMATH_SHARED)
# ifdef BUILDING_LabMath
#  define LM_CAPI extern "C" __declspec(dllexport)
#  define LM_API __declspec(dllexport)
#  define LM_CLASS __declspec(dllexport)
# else
#  define LM_CAPI extern "C" __declspec(dllimport)
#  define LM_API __declspec(dllimport)
#  define LM_CLASS __declspec(dllimport)
# endif
#else
# define LM_API
# define LM_CAPI extern "C"
# define LM_CLASS
#endif
