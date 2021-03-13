cmake_minimum_required(VERSION 3.10)
project(Policy)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake_modules")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

if ("${CMAKE_CXX_COMPILER}" MATCHES ".*arm.*")
    MESSAGE(STATUS "ARM ")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mno-unaligned-access")
    #-munaligned-access
elseif ("${CMAKE_CXX_COMPILER}" MATCHES ".*aarch.*")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -pthread -fPIC -O3 -Wall")
else ()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fPIC -O3 -Wall -g")
endif ()


if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")

else ()
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(MOSQUITTO REQUIRED IMPORTED_TARGET libmosquitto)
    pkg_check_modules(MOSQUITTOPP REQUIRED IMPORTED_TARGET libmosquittopp)
endif ()

#caffe
include_directories(/home/wen-li13/caffe/caffe/build/install/include)
link_directories(/home/wen-li13/caffe/caffe/build/install/lib)

#cuda
include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)

#lapack
include_directories(/usr/local/opt/lapack/include)
link_directories(/usr/local/opt/lapack/lib)

#openblas
include_directories(/usr/local/opt/openblas/include)
link_directories(/usr/local/opt/openblas/lib)

#opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


# BOOST
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
set(BOOST_ALL_DYN_LINK ON)
SET(BOOST_REQUIRED_COMPONENTS
        date_time
        filesystem
        program_options
        regex
        system
        thread
        #        python3.7
        )
set(BOOST_MIN_VERSION "1.58")
set(Boost_ADDITIONAL_VERSIONS
        "1.56.0" "1.56" "1.57" "1.57" "1.58" "1.59" "1.60" "1.61" "1.62" "1.63"
        "1.64" "1.65" "1.66" "1.67" "1.68" "1.69" "1.70"
        )
find_package(Boost ${BOOST_MIN_VERSION} COMPONENTS ${BOOST_REQUIRED_COMPONENTS})
message(STATUS "** Boost Include: ${Boost_INCLUDE_DIR}")
message(STATUS "** Boost Libraries: ${Boost_LIBRARY_DIRS}")
message(STATUS "** Boost Libraries: ${Boost_LIBRARIES}")
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})
#include_directories(include)

#include
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

message(STATUS "${CMAKE_BINARY_DIR}")
message(STATUS "${CMAKE_SOURCE_DIR}")

#file copy for prototxt
file(GLOB PROTOTXT_FILES "${CMAKE_SOURCE_DIR}/prototxt/*.prototxt")
message(STATUS ${PROTOTXT_FILES})
file(COPY ${PROTOTXT_FILES} DESTINATION ${CMAKE_BINARY_DIR} )

#find_package(nlohmann_json CONFIG REQUIRED)
find_package(cppzmq CONFIG REQUIRED)
#find_package(glog CONFIG REQUIRED)
#find_package(easy_connect CONFIG REQUIRED)
#add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)
set(TORCH_CXX_FLAGS -D_GLIBCXX_USE_CXX11_ABI=1)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

message(STATUS "${CMAKE_CXX_FLAGS}")

set(POLICY_LINK_LIBS
        glog
        cppzmq
        ${Boost_LIBRARIES}
        ${OpenCV_LIBS}
        easy_connect
        uhd
        fftw3
        usb-1.0
        caffe
        caffeproto
        protobuf
        gflags)


add_subdirectory(include)
add_subdirectory(lib)
add_subdirectory(src)


