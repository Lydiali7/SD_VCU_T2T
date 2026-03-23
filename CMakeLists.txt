cmake_minimum_required(VERSION 3.10)
project(SDVCU_Project)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#support AVX512
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mavx512f -mavx512dq -mavx512bw -O3")

include_directories(include)


find_package(Threads REQUIRED)

add_executable(sdvcu_app src/main.cpp src/perception.cpp)

target_link_libraries(sdvcu_app Threads::Threads)