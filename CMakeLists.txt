cmake_minimum_required(VERSION 3.5)
project(libpanoramagrid VERSION 1.0.0 LANGUAGES CXX C)

# Make sure that custom modules like FindRapidJSON are found
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake)

##############################################
# Declare dependencies

find_package(Boost REQUIRED COMPONENTS python numpy)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs)
find_package(glfw3 REQUIRED)
find_package(PythonLibs 3 REQUIRED)

add_library(glad src/glad.c)
target_include_directories(glad
    PUBLIC
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(glad dl)

##############################################
# Create target and set properties

add_library(panoramagrid
    src/camera.cpp
    src/cubemapmaterial.cpp
    src/cubemesh.cpp
    src/glapplication.cpp
    src/glrenderer.cpp
    src/grid.cpp
    src/material.cpp
    src/mesh.cpp
    src/node.cpp
    src/renderer.cpp
    src/shader.cpp
    src/spheremesh.cpp
    src/uvmaterial.cpp
)
add_library(Panoramagrid::Panoramagrid ALIAS panoramagrid)
target_include_directories(panoramagrid
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_features(panoramagrid PRIVATE cxx_auto_type)
target_compile_options(panoramagrid PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wall>)
set_property(TARGET panoramagrid PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries(panoramagrid
    PUBLIC
        opencv_core
        opencv_imgcodecs
        glfw
        zip
        glad
        pthread
)

##############################################
# Installation instructions

include(GNUInstallDirs)
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/Panoramagrid)

install(TARGETS panoramagrid glad
    EXPORT panoramagrid-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

set_target_properties(panoramagrid PROPERTIES EXPORT_NAME Panoramagrid)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(DIRECTORY share/ DESTINATION ${CMAKE_INSTALL_PREFIX}/share)

install(EXPORT panoramagrid-targets
  FILE
    PanoramagridTargets.cmake
  NAMESPACE
    Panoramagrid::
  DESTINATION
    ${INSTALL_CONFIGDIR}
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/PanoramagridConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/PanoramagridConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/PanoramagridConfig.cmake
    INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/PanoramagridConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/PanoramagridConfigVersion.cmake
    DESTINATION ${INSTALL_CONFIGDIR}
)

export(EXPORT panoramagrid-targets FILE ${CMAKE_CURRENT_BINARY_DIR}/PanoramagridTargets.cmake NAMESPACE Panoramagrid::)

export(PACKAGE Panoramagrid)

add_subdirectory(tools)
add_subdirectory(examples)
