cmake_minimum_required(VERSION 3.5)
project(路工大干扰设备)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -pthread -W -Wall -O3 -g -msse2")
set(Boost_USE_STATIC_LIBS        OFF)
set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME     OFF)
set(BOOST_ALL_DYN_LINK           ON)


SET(BOOST_REQUIRED_COMPONENTS
        date_time
        filesystem
        program_options
        regex
        system
        thread
        unit_test_framework
        )

include_directories(include)


find_package(Boost 1.49 COMPONENTS ${BOOST_REQUIRED_COMPONENTS} REQUIRED)
message(status "** Boost Include: ${Boost_INCLUDE_DIR}")
message(status "** Boost Libraries: ${Boost_LIBRARY_DIRS}")
message(status "** Boost Libraries: ${Boost_LIBRARIES}")

INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
INCLUDE_DIRECTORIES(BEFORE ${CMAKE_CURRENT_SOURCE_DIR})
LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})

set(DISTURB_LINK_LIBS
        ${Boost_LIBRARIES}
        fftw3
        uhd
        jsoncpp
        glog
        zmq
        easy_connect
        )

add_subdirectory(lib)
add_subdirectory(include)
add_subdirectory(src)


