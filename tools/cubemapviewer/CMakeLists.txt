find_package(Boost REQUIRED COMPONENTS program_options)

add_executable(cubemapviewer cubemapviewer.cpp)
target_compile_features(cubemapviewer PRIVATE cxx_auto_type)
target_link_libraries(cubemapviewer Panoramagrid::Panoramagrid Boost::program_options)

install(TARGETS cubemapviewer DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
