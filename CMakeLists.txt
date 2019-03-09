cmake_minimum_required(VERSION 3.13)
project(panoramagrid)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

include_directories(include)

# opencv
find_package(OpenCV REQUIRED core imgproc imgcodecs highgui)

# glad
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

# glfw
find_package(glfw3 3.2 REQUIRED)

# glm
find_package(glm REQUIRED)

# panoramagrid
foreach (CLASS camera cubemapmaterial cubemesh grid material mesh node renderer)
    set(PANORAMAGRID_SRC ${PANORAMAGRID_SRC} src/${CLASS}.cpp include/panoramagrid/${CLASS}.hpp)
endforeach ()
foreach (CLASS glrenderer shader)
    set(PANORAMAGRID_SRC ${PANORAMAGRID_SRC} src/${CLASS}.cpp include/panoramagrid/gl/${CLASS}.hpp)
endforeach ()
set(PANORAMAGRID_SRC ${PANORAMAGRID_SRC} include/panoramagrid/utilities.hpp)  # utilities
set(PANORAMAGRID_SRC ${PANORAMAGRID_SRC} include/panoramagrid/gl/utilities.hpp)  # gl utilities
set(PANORAMAGRID_SRC ${PANORAMAGRID_SRC} include/panoramagrid/panoramagrid.hpp include/panoramagrid/gl/gl.hpp)  # all-in-one headers
add_library(panoramagrid SHARED ${PANORAMAGRID_SRC})
target_include_directories(panoramagrid PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(panoramagrid ${OpenCV_LIBS} glad glm)

# cubemapviewer
add_executable(cubemapviewer tools/cubemapviewer.cpp)
target_link_libraries(cubemapviewer dl glad glfw panoramagrid)
