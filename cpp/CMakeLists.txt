cmake_minimum_required(VERSION 3.10)
project(openrobot_motor CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(openrobot_motor INTERFACE)
target_include_directories(openrobot_motor INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

option(OPENROBOT_MOTOR_EXAMPLES "Build PC examples" ON)
if(OPENROBOT_MOTOR_EXAMPLES)
    find_package(Threads REQUIRED)
    foreach(ex scan speed mit multi)
        add_executable(${ex} examples/${ex}.cpp)
        target_link_libraries(${ex} openrobot_motor Threads::Threads ${CMAKE_DL_LIBS})
        if(WIN32)
            target_link_libraries(${ex} winmm)
        endif()
    endforeach()
endif()

option(OPENROBOT_MOTOR_TESTS "Build tests" ON)
if(OPENROBOT_MOTOR_TESTS)
    enable_testing()
    add_executable(test_protocol tests/test_protocol.cpp)
    target_link_libraries(test_protocol openrobot_motor)
    target_include_directories(test_protocol PRIVATE tests)
    add_test(NAME protocol COMMAND test_protocol)
endif()
