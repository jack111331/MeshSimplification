cmake_minimum_required(VERSION 3.1.3)
project(MeshSimplification)

set(GUI_SUPPORT FALSE)

set(wxUSE_UNICODE_UTF8 1)

#set(CMAKE_VERBOSE_MAKEFILE ON)

find_package(OpenGL REQUIRED)

#set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
#set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
#set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
#set(GLFW_USE_HYBRID_HPG ON CACHE BOOL "" FORCE)
#add_subdirectory("public/glfw")
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "${CMAKE_SOURCE_DIR}/dependency")
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/dependency/share/cmake/yaml-cpp")
find_package(glfw3 REQUIRED)

include_directories("public/glm")

add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")

SET(wxWidgets_ROOT_DIR "${CMAKE_SOURCE_DIR}/dependency/")
SET(wxWidgets_LIB_DIR "${CMAKE_SOURCE_DIR}/dependency/lib/vc_dll")
#SET(wxWidgets_LIBRARIES "${CMAKE_SOURCE_DIR}/public/wxWidgets/lib")
SET(wxWidgets_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/public/wxWidgets/include/wx")
find_package(wxWidgets REQUIRED COMPONENTS net core base gl)
message("${wxWidgets_LIBRARIES}")
include(${wxWidgets_USE_FILE})

add_definitions(-D_USE_MATH_DEFINES)

include_directories("./include")
include_directories("./include/glad")


if (GUI_SUPPORT)
    add_compile_definitions(GUI_SUPPORT)
endif()

set(SOURCE_LIST)
set(HEADER_LIST)

list(APPEND SOURCE_LIST
        src/Camera.cpp
        src/PhongShadingPipeline.cpp
        src/Pipeline.cpp
        src/PipelineFactory.cpp
        src/Shader.cpp
        src/ShaderProgram.cpp
        src/Texture.cpp
        src/Utility.cpp
        src/Timer.cpp
        src/Context.cpp

        src/OpenMeshDef.cpp
        src/Simplification.cpp
        src/LeastSquareMesh.cpp
        src/Skeleton.cpp

        src/GLPanel.cpp
        src/MainFrame.cpp

        src/glad.c
#        ${IMGUI_SOURCE_LIST}
        )
list(APPEND HEADER_LIST
        include/Camera.h
        include/PhongShadingPipeline.h
        include/Pipeline.h
        include/PipelineFactory.h
        include/Shader.h
        include/ShaderProgram.h
        include/ShaderInclude.h
        include/Texture.h
        include/Timer.h
        include/Context.h

        include/OpenMeshDef.h
        include/Simplification.h
        include/LeastSquareMesh.h
        include/Skeleton.h
        include/Matrix.h
        include/Vec.h

        include/Application.h
        include/MainFrame.h
        include/GLPanel.h

        include/stb_image.h
        )


# Add WIN32 to indicate cmake to build the windows GUI application
add_executable(MeshSimplification WIN32 src/main.cpp ${HEADER_LIST} ${SOURCE_LIST})
target_link_libraries(MeshSimplification glfw OpenGL::GL debug ${CMAKE_SOURCE_DIR}/dependency/lib/OpenMeshCored.lib
        debug ${CMAKE_SOURCE_DIR}/dependency/lib/OpenMeshToolsd.lib ${wxWidgets_LIBRARIES})