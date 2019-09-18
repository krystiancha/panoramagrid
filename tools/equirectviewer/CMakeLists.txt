find_package(Boost REQUIRED COMPONENTS program_options)

add_executable(equirectviewer equirectviewer.cpp)
target_compile_features(equirectviewer PRIVATE cxx_auto_type)
target_link_libraries(equirectviewer Panoramagrid::Panoramagrid Boost::program_options)
