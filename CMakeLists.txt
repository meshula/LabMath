
include_guard()

cmake_minimum_required (VERSION 3.11)
project (LabMath)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(LABMATH_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

include(Settings)
include(Utilities)

lab_library(LabMath
    TYPE SHARED

    ALIAS Lab::Math

    LIBRARIES

    INCLUDE_DIRS
        ${CMAKE_CURRENT_SOURCE_DIR}/include

    PUBLIC_HEADERS
        include/LabMath/Export.h
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

configure_file(cmake/LabMathConfig.cmake.in 
    "${PROJECT_BINARY_DIR}/LabMathConfig.cmake" @ONLY)
install(FILES
  "${PROJECT_BINARY_DIR}/LabMathConfig.cmake"
  DESTINATION "${CMAKE_INSTALL_PREFIX}"
)

