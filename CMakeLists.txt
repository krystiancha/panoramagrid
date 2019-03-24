cmake_minimum_required(VERSION 3.10)
project(Panoramagrid)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)


# Boost
find_package(Boost COMPONENTS program_options REQUIRED)


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
        COMMAND ${GLAD} --profile core --out-path ${PROJECT_SOURCE_DIR} --api gl= --generator c-debug --spec gl
        COMMENT "Generating sources for glad")
add_library(glad src/glad.c include/glad/glad.h include/KHR/khrplatform.h)


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
install (FILES ${Panoramagrid_INCLUDE} DESTINATION include/panoramagrid)

### gl

foreach(CLASS glrenderer shader)
    set(Panoramagrid_gl_INCLUDE ${Panoramagrid_gl_INCLUDE} include/panoramagrid/gl/${CLASS}.hpp)
    set(Panoramagrid_gl_SRC ${Panoramagrid_gl_SRC} src/panoramagrid/gl/${CLASS}.cpp)
endforeach()
set(Panoramagrid_gl_INCLUDE ${Panoramagrid_gl_INCLUDE} include/panoramagrid/gl/gl.hpp)  # all-in-one header
install (FILES ${Panoramagrid_gl_INCLUDE} DESTINATION include/panoramagrid/gl)

### gl applications

foreach(CLASS cubemapviewer equirectviewer glapplication)
    set(Panoramagrid_gl_applications_INCLUDE ${Panoramagrid_gl_applications_INCLUDE} include/panoramagrid/gl/applications/${CLASS}.hpp)
    set(Panoramagrid_gl_applications_SRC ${Panoramagrid_gl_applications_SRC} src/panoramagrid/gl/applications/${CLASS}.cpp)
endforeach()
install (FILES ${Panoramagrid_gl_applications_INCLUDE} DESTINATION include/panoramagrid/gl/applications)

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
target_link_libraries(panoramagrid ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} glad glfw glm zip)
install(TARGETS panoramagrid DESTINATION lib)

## Tools

### cubemapviewer
add_executable(cubemapviewer tools/cubemapviewer.cpp)
target_link_libraries(cubemapviewer dl glad glfw panoramagrid)
install (TARGETS cubemapviewer DESTINATION bin)

### cubemapstitch
add_executable(cubemapstitch tools/cubemapstitch.cpp)
target_include_directories(cubemapstitch PUBLIC ${Boost_INCLUDE_DIRS})
target_link_libraries(cubemapstitch dl ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})
install (TARGETS cubemapstitch DESTINATION bin)

### equirectviewer
add_executable(equirectviewer tools/equirectviewer.cpp)
target_link_libraries(equirectviewer dl glad glfw panoramagrid)
install (TARGETS equirectviewer DESTINATION bin)
