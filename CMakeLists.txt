cmake_minimum_required (VERSION 3.0)
set (CMAKE_CXX_STANDARD 17)

project (Jyamithika)

find_package(OpenGL REQUIRED)

file(GLOB Jyamithika_core_base_source CONFIGURE_DEPENDS Jyamithika/Core/Base/*.cpp 
    Jyamithika/Core/Base/*.c Jyamithika/Core/Base/*.h)

file(GLOB Jyamithika_core_Primitives_source CONFIGURE_DEPENDS Jyamithika/Core/Primitives/*.cpp 
    Jyamithika/Core/Primitives/*.c Jyamithika/Primitives/Base/*.h)

file(GLOB Jyamithika_core_source CONFIGURE_DEPENDS Jyamithika/Core/*.cpp 
    Jyamithika/Core/*.c Jyamithika/Base/*.h)

file(GLOB Jyamithika_tree_source CONFIGURE_DEPENDS Jyamithika/Trees/*.cpp 
    Jyamithika/Trees/*.c Jyamithika/Trees/*.h)

file(GLOB Jyamithika_source CONFIGURE_DEPENDS Jyamithika/*.cpp 
    Jyamithika/*.c Jyamithika/*.h)

add_library(Jyamithika OBJECT ${Jyamithika_core_base_source}
                            ${Jyamithika_core_Primitives_source}
                            ${Jyamithika_core_source}
                            ${Jyamithika_tree_source}
                            ${Jyamithika_source})

