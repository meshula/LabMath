
include_guard()

cmake_minimum_required (VERSION 3.11)
project (LabMath)

set(LABMATH_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../LabCMake/scripts
                      ${CMAKE_CURRENT_SOURCE_DIR}/../LabCMake/modules)

include(Utilities)

lab_library(LabMath
    TYPE SHARED

    ALIAS Lab::Math

    LIBRARIES

    INCLUDE_DIRS
        ${CMAKE_CURRENT_SOURCE_DIR}/include

    PUBLIC_HEADERS
        include/LabMath/LabMath.h
        include/LabMath/linalg.h
        include/LabMath/math-common.hpp
        include/LabMath/math-core.hpp
        include/LabMath/math-primitives.hpp
        include/LabMath/math-projection.hpp
        include/LabMath/math-ray.hpp
        include/LabMath/math-spatial.hpp

    PRIVATE_HEADERS

    CPPFILES
        src/MathTypes.cpp
)
