cmake_minimum_required(VERSION 3.10)
project(raisim_examples LANGUAGES CXX)

set(raisim_DIR "/Users/yushin/raisim_build/lib/cmake/raisim")
set( jointImpedance_SRCS test/jointImpedance.cpp controller/src/includeTest.cpp)
set( taskHOQP_SRCS test/taskHOQP.cpp controller/src/includeTest.cpp)
find_package(raisim CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)


include_directories (${EIGEN3_INCLUDE_DIRS})
include_directories (controller/include)

add_executable(jointImpedance ${jointImpedance_SRCS})
add_executable(taskHOQP ${taskHOQP_SRCS})

target_link_libraries(jointImpedance PUBLIC raisim::raisim)
target_link_libraries(taskHOQP PUBLIC raisim::raisim)
target_link_libraries(taskHOQP PRIVATE osqp::osqp)
target_link_libraries(taskHOQP PRIVATE OsqpEigen::OsqpEigen)

if(NOT WIN32)
  target_link_libraries(jointImpedance PUBLIC pthread)
else()
  target_link_libraries(jointImpedance PUBLIC Ws2_32 Winmm)
endif()

target_include_directories(jointImpedance PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/rsc DESTINATION ${CMAKE_BINARY_DIR})
