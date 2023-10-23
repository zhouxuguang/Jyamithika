cmake_minimum_required (VERSION 3.0)
set (CMAKE_CXX_STANDARD 17)

project (Jyamithika)

find_package(OpenGL REQUIRED)

file(GLOB_RECURSE Jyamithika_core_base_source CONFIGURE_DEPENDS Jyamithika/Core/Base/*.cpp 
    Jyamithika/Core/Base/*.c Jyamithika/Core/Base/*.h)

add_library(Jyamithika OBJECT ${Jyamithika_core_base_source})

