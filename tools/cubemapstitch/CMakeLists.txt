find_package(Boost REQUIRED COMPONENTS program_options)

add_executable(cubemapstitch cubemapstitch.cpp)
target_compile_features(cubemapstitch PRIVATE cxx_auto_type)
target_link_libraries(cubemapstitch Panoramagrid::Panoramagrid Boost::program_options)
