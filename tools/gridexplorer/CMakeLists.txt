find_package(Boost REQUIRED COMPONENTS program_options)

add_executable(gridexplorer gridexplorer.cpp)
target_compile_features(gridexplorer PRIVATE cxx_auto_type)
target_link_libraries(gridexplorer Panoramagrid::Panoramagrid Boost::program_options)
