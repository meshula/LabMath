
#pragma once

#ifdef _MSC_VER
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
# define LM_CAPI
# define LM_CLASS
#endif
