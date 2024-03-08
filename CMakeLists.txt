cmake_minimum_required(VERSION 3.10)
project(raisim_examples LANGUAGES CXX)

set(raisim_DIR "/Users/yushin/raisim_build/lib/cmake/raisim")
set(jointImpedance_SRCS test/jointImpedance.cpp controller/src/includeTest.cpp)
set(id_osc_SRCS test/id_osc.cpp)
set(osc_SRCS test/osc.cpp)
set(nullspace_SRCS test/nullspace_control.cpp)
set(joint_impedance_SRCS test/joint_impedance.cpp)
set(taskHOQP_SRCS 
    test/taskHOQP.cpp 
    controller/src/InverseDynamicsQPSolver.cpp 
    controller/src/OsqpEigenSolver.cpp)
set(testHOQP_SRCS 
    test/hoqp_test.cpp
    controller/src/OsqpEigenSolver.cpp
    controller/src/HOQP.cpp
    controller/src/tasks/TaskLS.cpp
    controller/src/tasks/TaskLS_ID.cpp
    controller/src/tasks/TaskLS_StationaryFeet.cpp
    controller/src/tasks/TaskLS_MoveBase.cpp)

include(FetchContent)

FetchContent_Declare(matplotplusplus
        GIT_REPOSITORY https://github.com/alandefreitas/matplotplusplus
        GIT_TAG origin/master)

FetchContent_GetProperties(matplotplusplus)
if(NOT matplotplusplus_POPULATED)
    FetchContent_Populate(matplotplusplus)
    add_subdirectory(${matplotplusplus_SOURCE_DIR} ${matplotplusplus_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

find_package(raisim CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(matplot++ REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)


include_directories (${EIGEN3_INCLUDE_DIRS})
include_directories (controller/include)
include_directories (controller/include/tasks)

add_executable(jointImpedance ${jointImpedance_SRCS})
add_executable(id_osc ${id_osc_SRCS})
add_executable(osc ${osc_SRCS})
add_executable(nullspace ${nullspace_SRCS})
add_executable(joint_impedance ${joint_impedance_SRCS})
add_executable(taskHOQP ${taskHOQP_SRCS})
add_executable(testHOQP ${testHOQP_SRCS})
find_package(PythonLibs 2.7)
target_link_libraries(jointImpedance PUBLIC raisim::raisim)
target_link_libraries(joint_impedance PUBLIC raisim::raisim)
target_link_libraries(id_osc PUBLIC raisim::raisim matplot)
target_link_libraries(osc PUBLIC raisim::raisim matplot)
target_link_libraries(nullspace PUBLIC raisim::raisim matplot)

target_link_libraries(taskHOQP PUBLIC raisim::raisim)
target_link_libraries(taskHOQP PRIVATE osqp::osqp)
target_link_libraries(taskHOQP PRIVATE OsqpEigen::OsqpEigen)

target_link_libraries(testHOQP PUBLIC raisim::raisim)
target_link_libraries(testHOQP PRIVATE osqp::osqp)
target_link_libraries(testHOQP PRIVATE OsqpEigen::OsqpEigen)

if(NOT WIN32)
  target_link_libraries(jointImpedance PUBLIC pthread)
  target_link_libraries(joint_impedance PUBLIC pthread)
  target_link_libraries(taskHOQP PUBLIC pthread)
  target_link_libraries(testHOQP PUBLIC pthread)
else()
  target_link_libraries(jointImpedance PUBLIC Ws2_32 Winmm)
  target_link_libraries(joint_impedance PUBLIC Ws2_32 Winmm)
  target_link_libraries(taskHOQP PUBLIC Ws2_32 Winmm)
  target_link_libraries(testHOQP PUBLIC Ws2_32 Winmm)
endif()

target_include_directories(jointImpedance PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(taskHOQP PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(testHOQP PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/rsc DESTINATION ${CMAKE_BINARY_DIR})
