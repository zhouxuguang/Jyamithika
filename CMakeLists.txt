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

file(GLOB glad_source CONFIGURE_DEPENDS Graphics/Externals/GLAD/src/*.c)

file(GLOB imgui_source CONFIGURE_DEPENDS Graphics/GraphicUtils/Imgui/*.cpp 
    Graphics/GraphicUtils/Imgui/*.h)

file(GLOB GraphicUtils_source CONFIGURE_DEPENDS Graphics/GraphicUtils/*.cpp 
    Graphics/GraphicUtils/*.h)

add_library(Jyamithika OBJECT ${Jyamithika_core_base_source}
                            ${Jyamithika_core_Primitives_source}
                            ${Jyamithika_core_source}
                            ${Jyamithika_tree_source}
                            ${Jyamithika_source}
                            ${glad_source}
                            ${imgui_source}
                            ${GraphicUtils_source})

target_include_directories(Jyamithika PUBLIC
                                            ${CMAKE_SOURCE_DIR}
                                            Graphics/Externals/GLFW/include
                                            Graphics/Externals/GLAD/include
                                            Graphics/Externals)
target_link_directories(Jyamithika PUBLIC Graphics/Externals/GLFW/libs/mac)
#target_link_libraries(Jyamithika PUBLIC glfw3)

add_executable(Convexhull2D Samples/Convexhull2D/ConstructingConvexhull_2d_simulation.cpp)

target_include_directories(Convexhull2D PUBLIC
                                            ${CMAKE_SOURCE_DIR}
                                            ${CMAKE_SOURCE_DIR}/Graphics)
target_link_directories(Convexhull2D PUBLIC Graphics/Externals/GLFW/libs/mac)

target_link_libraries(Convexhull2D PRIVATE Jyamithika 
    ${CMAKE_CURRENT_SOURCE_DIR}/Graphics/Externals/GLFW/libs/mac/libglfw.3.dylib)