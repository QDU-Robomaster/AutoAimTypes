cmake_minimum_required(VERSION 3.16)
project(AutoAimTypesTests LANGUAGES CXX)

add_executable(autoaim_types_test autoaim_types_test.cpp)
target_include_directories(autoaim_types_test PRIVATE ${CMAKE_CURRENT_LIST_DIR}/..)
target_link_libraries(autoaim_types_test PRIVATE xr)
target_compile_features(autoaim_types_test PRIVATE cxx_std_20)
add_test(NAME autoaim_types_test COMMAND autoaim_types_test)
