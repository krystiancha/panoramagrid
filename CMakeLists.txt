cmake_minimum_required(VERSION 3.5)
project(libpanoramagrid VERSION 1.0.0 LANGUAGES CXX C)

# Make sure that custom modules like FindRapidJSON are found
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake)

##############################################
# Declare dependencies

find_package(OpenCV REQUIRED COMPONENTS core imgcodecs)
find_package(glfw3 REQUIRED)

find_program(GLAD glad)
if (NOT GLAD)
    message(FATAL_ERROR "glad not found in your PATH. Go to https://github.com/Dav1dde/glad for details. "
            "You can install it via pip: pip install glad")
endif ()
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/src/glad.c
        ${PROJECT_SOURCE_DIR}/include/glad/glad.h
        ${PROJECT_SOURCE_DIR}/include/KHR/khrplatform.h
        COMMAND ${GLAD} --profile core --out-path ${PROJECT_SOURCE_DIR} --api gl= --generator c --spec gl
        COMMENT "Generating sources for glad")
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
    src/panoramagrid/camera.cpp
    src/panoramagrid/cubemapmaterial.cpp
    src/panoramagrid/cubemesh.cpp
    src/panoramagrid/grid.cpp
    src/panoramagrid/material.cpp
    src/panoramagrid/mesh.cpp
    src/panoramagrid/node.cpp
    src/panoramagrid/renderer.cpp
    src/panoramagrid/spheremesh.cpp
    src/panoramagrid/uvmaterial.cpp
    src/panoramagrid/gl/glrenderer.cpp
    src/panoramagrid/gl/shader.cpp
    src/panoramagrid/gl/applications/cubemapviewer.cpp
    src/panoramagrid/gl/applications/equirectviewer.cpp
    src/panoramagrid/gl/applications/glapplication.cpp
)
add_library(Panoramagrid::Panoramagrid ALIAS panoramagrid)
target_include_directories(panoramagrid
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_features(panoramagrid PRIVATE cxx_auto_type)
target_compile_options(panoramagrid PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wall>)
target_link_libraries(panoramagrid
    PUBLIC
        opencv_core
        opencv_imgcodecs
        glfw
        zip
        glad
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
