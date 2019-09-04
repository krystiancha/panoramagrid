cmake_minimum_required(VERSION 3.10)
project(Panoramagrid VERSION 0.0.1 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)


# Boost
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(PythonLibs 3 REQUIRED)
find_package(Boost COMPONENTS program_options python3 numpy3 REQUIRED)


# OpenCV
find_package(OpenCV REQUIRED core imgproc imgcodecs highgui)


# GLAD
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
add_library(glad src/glad.c include/glad/glad.h include/KHR/khrplatform.h)
install(FILES include/glad/glad.h DESTINATION include/glad)


# GLFW
find_package(glfw3 3.2 REQUIRED)


# GLM
find_package(glm REQUIRED)


include_directories(include)


# Panoramagrid

## Library

### base

foreach(CLASS camera cubemapmaterial cubemesh grid material mesh node renderer spheremesh uvmaterial)
    set(Panoramagrid_INCLUDE ${Panoramagrid_INCLUDE} include/panoramagrid/${CLASS}.hpp)
    set(Panoramagrid_SRC ${Panoramagrid_SRC} src/panoramagrid/${CLASS}.cpp)
endforeach()
set(Panoramagrid_INCLUDE ${Panoramagrid_INCLUDE} include/panoramagrid/panoramagrid.hpp)  # all-in-one header
install(FILES ${Panoramagrid_INCLUDE} DESTINATION include/panoramagrid)

### gl

foreach(CLASS glrenderer shader)
    set(Panoramagrid_gl_INCLUDE ${Panoramagrid_gl_INCLUDE} include/panoramagrid/gl/${CLASS}.hpp)
    set(Panoramagrid_gl_SRC ${Panoramagrid_gl_SRC} src/panoramagrid/gl/${CLASS}.cpp)
endforeach()
set(Panoramagrid_gl_INCLUDE ${Panoramagrid_gl_INCLUDE} include/panoramagrid/gl/gl.hpp)  # all-in-one header
install(FILES ${Panoramagrid_gl_INCLUDE} DESTINATION include/panoramagrid/gl)

### gl applications

foreach(CLASS cubemapviewer equirectviewer glapplication)
    set(Panoramagrid_gl_applications_INCLUDE ${Panoramagrid_gl_applications_INCLUDE} include/panoramagrid/gl/applications/${CLASS}.hpp)
    set(Panoramagrid_gl_applications_SRC ${Panoramagrid_gl_applications_SRC} src/panoramagrid/gl/applications/${CLASS}.cpp)
endforeach()
install(FILES ${Panoramagrid_gl_applications_INCLUDE} DESTINATION include/panoramagrid/gl/applications)

### Build
add_library(panoramagrid SHARED
        ${Panoramagrid_SRC}
        ${Panoramagrid_gl_SRC}
        ${Panoramagrid_gl_applications_SRC}
        ${Panoramagrid_INCLUDE}
        ${Panoramagrid_gl_INCLUDE}
        ${Panoramagrid_gl_applications_INCLUDE}
)
target_include_directories(panoramagrid PUBLIC ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(panoramagrid PUBLIC Boost::program_options ${OpenCV_LIBRARIES} glad glfw glm zip)
install(TARGETS panoramagrid glad EXPORT panoramagrid-targets LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
install(EXPORT panoramagrid-targets FILE PanoramagridTargets.cmake DESTINATION lib/cmake/Panoramagrid)
include(CMakePackageConfigHelpers)
configure_package_config_file(
        ${CMAKE_CURRENT_LIST_DIR}/cmake/PanoramagridConfig.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/PanoramagridConfig.cmake
        INSTALL_DESTINATION lib/cmake/Panoramagrid
)
install(
        FILES
        ${CMAKE_CURRENT_BINARY_DIR}/PanoramagridConfig.cmake
        DESTINATION lib/cmake/Panoramagrid
)

## Gridpy

add_library(gridpy SHARED include/panoramagrid/gridpy.hpp src/panoramagrid/gridpy.cpp)
set_target_properties(gridpy PROPERTIES PREFIX "")
target_include_directories(gridpy PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(gridpy panoramagrid Boost::python3 Boost::numpy3 ${PYTHON_LIBRARIES})
install(TARGETS gridpy DESTINATION lib)

## Tools

### cubemapviewer
add_executable(cubemapviewer tools/cubemapviewer.cpp)
target_link_libraries(cubemapviewer panoramagrid)
install (TARGETS cubemapviewer DESTINATION bin)

### cubemapstitch
add_executable(cubemapstitch tools/cubemapstitch.cpp)
target_link_libraries(cubemapstitch panoramagrid)
install (TARGETS cubemapstitch DESTINATION bin)

### equirectviewer
add_executable(equirectviewer tools/equirectviewer.cpp)
target_link_libraries(equirectviewer panoramagrid)
install (TARGETS equirectviewer DESTINATION bin)

### gridexplorer
add_executable(gridexplorer tools/gridexplorer.cpp)
#target_include_directories(gridexplorer PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(gridexplorer panoramagrid)
install (TARGETS gridexplorer DESTINATION lib)

## Shared files
file(GLOB Panoramagrid_SHARE share/panoramagrid/*)
install(FILES ${Panoramagrid_SHARE} DESTINATION share/panoramagrid)