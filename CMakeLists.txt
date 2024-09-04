cmake_minimum_required(VERSION 3.10)
project(raisim_examples LANGUAGES CXX)
set(CMAKE_OSX_ARCHITECTURES "arm64")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a")
set(raisim_DIR "/Users/yushin/raisim_build/lib/cmake/raisim")
set(EQUALITY_TASKS_PATH controller/src/tasks/Equality)
set(INEQUALITY_TASKS_PATH controller/src/tasks/Inequality)

set(jointImpedance_SRCS test/jointImpedance.cpp controller/src/includeTest.cpp)
set(id_osc_SRCS test/id_osc.cpp)
set(osc_SRCS test/osc.cpp)
set(nullspace_SRCS test/nullspace_control.cpp)
set(bezier_test_SRCS test/bezier_test.cpp)
set(joint_impedance_SRCS test/joint_impedance.cpp)
set(base_qp_SRCS 
    test/base_qp.cpp 
    controller/src/HOQP.cpp
    controller/src/OsqpEigenSolver.cpp
    controller/src/Tasks/TaskSet.cpp
    controller/src/tasks/TaskLS.cpp
    ${EQUALITY_TASKS_PATH}/TaskLS_ID.cpp
    ${EQUALITY_TASKS_PATH}/TaskLS_StationaryFeet.cpp
    ${EQUALITY_TASKS_PATH}/TaskLS_MoveBase.cpp
    utils/Utils.cpp)
set(testHOQP_SRCS 
    test/hoqp_test.cpp
    controller/src/OsqpEigenSolver.cpp
    controller/src/HOQP.cpp
    controller/src/Tasks/TaskSet.cpp
    controller/src/tasks/TaskLS.cpp
    ${EQUALITY_TASKS_PATH}/TaskLS_ID.cpp
    ${EQUALITY_TASKS_PATH}/TaskLS_StationaryFeet.cpp
    ${EQUALITY_TASKS_PATH}/TaskLS_MoveBase.cpp
    ${EQUALITY_TASKS_PATH}/TaskLS_EnergyOpt.cpp
    utils/Utils.cpp)
set(testQP_SRCS
    test/osqp_test.cpp)
set(testMPC_SRCS
    test/convex_mpc.cpp
    controller/src/ConvexMPC.cpp
    controller/src/OsqpEigenSolver.cpp
    utils/Utils.cpp)
set(test_z1_spawn_SRCS test/z1_spawn.cpp utils/Utils.cpp)
set(test_wbc_legged_arm_SRCS test/wbc_legged_arm.cpp
    controller/src/OsqpEigenSolver.cpp
    controller/src/HOQP.cpp
    controller/src/HOQP_Slack.cpp
    controller/src/Tasks/TaskSet.cpp
    controller/src/tasks/TaskLS.cpp
    ${EQUALITY_TASKS_PATH}/TaskLS_ID.cpp
    ${EQUALITY_TASKS_PATH}/TaskLS_StationaryFeet.cpp
    ${EQUALITY_TASKS_PATH}/TaskLS_MoveBase.cpp
    ${EQUALITY_TASKS_PATH}/TaskLS_StationaryEE.cpp
    ${EQUALITY_TASKS_PATH}/TaskLS_MinMotion.cpp
    ${EQUALITY_TASKS_PATH}/TaskLS_EnergyOpt.cpp
    ${INEQUALITY_TASKS_PATH}/TaskLS_TorqueLimits.cpp
    ${INEQUALITY_TASKS_PATH}/TaskLS_FrictionCone.cpp
    utils/Utils.cpp)
set(test_utils_SRCS test/utils_test.cpp utils/Utils.cpp)
set(test_HOQP_slack_SRCS test/hoqp_slack_test.cpp
    controller/src/OsqpEigenSolver.cpp
    controller/src/HOQP_Slack.cpp
    controller/src/Tasks/TaskSet.cpp
    controller/src/tasks/TaskLS.cpp
    ${EQUALITY_TASKS_PATH}/TaskLS_ID.cpp
    ${EQUALITY_TASKS_PATH}/TaskLS_StationaryFeet.cpp
    ${EQUALITY_TASKS_PATH}/TaskLS_MoveBase.cpp
    ${EQUALITY_TASKS_PATH}/TaskLS_StationaryEE.cpp
    ${EQUALITY_TASKS_PATH}/TaskLS_MinMotion.cpp
    ${EQUALITY_TASKS_PATH}/TaskLS_EnergyOpt.cpp
    ${INEQUALITY_TASKS_PATH}/TaskLS_TorqueLimits.cpp
    ${INEQUALITY_TASKS_PATH}/TaskLS_FrictionCone.cpp
    utils/Utils.cpp
    )
set(test_slope_SRCS test/slope_test.cpp
    controller/src/OsqpEigenSolver.cpp
    controller/src/HOQP_Slack.cpp
    controller/src/Tasks/TaskSet.cpp
    controller/src/tasks/TaskLS.cpp
    ${EQUALITY_TASKS_PATH}/TaskLS_ID.cpp
    ${EQUALITY_TASKS_PATH}/TaskLS_StationaryFeet.cpp
    ${EQUALITY_TASKS_PATH}/TaskLS_MoveBase.cpp
    ${EQUALITY_TASKS_PATH}/TaskLS_StationaryEE.cpp
    ${EQUALITY_TASKS_PATH}/TaskLS_MinMotion.cpp
    ${EQUALITY_TASKS_PATH}/TaskLS_EnergyOpt.cpp
    ${INEQUALITY_TASKS_PATH}/TaskLS_TorqueLimits.cpp
    ${INEQUALITY_TASKS_PATH}/TaskLS_FrictionCone.cpp
    utils/Utils.cpp)
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
include_directories (controller/include/tasks/Equality)
include_directories (controller/include/tasks/Inequality)
include_directories(utils)

# 실행 파일 추가 및 라이브러리 링크를 위한 함수 정의 
function(add_raisim_executable target sources)
    add_executable(${target} ${sources})
    target_link_libraries(${target} PUBLIC raisim::raisim)
    if(NOT WIN32)
        target_link_libraries(${target} PUBLIC pthread)
    else()
        target_link_libraries(${target} PUBLIC Ws2_32 Winmm)
    endif()
    target_include_directories(${target} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
endfunction()

# 특정 링크 요구 사항을 처리하는 함수 정의
function(add_custom_executable target sources libs)
    add_raisim_executable(${target} "${sources}")
    foreach(lib IN LISTS libs)
        target_link_libraries(${target} PRIVATE ${lib})
    endforeach()
endfunction()

# 함수를 사용하여 실행 파일 추가
add_raisim_executable(jointImpedance "${jointImpedance_SRCS}")
add_raisim_executable(joint_impedance "${joint_impedance_SRCS}")
add_custom_executable(id_osc "${id_osc_SRCS}" "matplot")
add_custom_executable(osc "${osc_SRCS}" "matplot")
add_custom_executable(nullspace "${nullspace_SRCS}" "matplot")
add_custom_executable(bezier_test "${bezier_test_SRCS}" "matplot")
add_custom_executable(testQP "${testQP_SRCS}" "OsqpEigen::OsqpEigen")
add_custom_executable(base_qp "${base_qp_SRCS}" "OsqpEigen::OsqpEigen")
add_custom_executable(testMPC "${testMPC_SRCS}" "osqp::osqp;OsqpEigen::OsqpEigen")
add_raisim_executable(test_z1_spawn "${test_z1_spawn_SRCS}")
add_custom_executable(test_wbc_legged_arm "${test_wbc_legged_arm_SRCS}" "osqp::osqp;OsqpEigen::OsqpEigen")
add_custom_executable(testHOQP "${testHOQP_SRCS}" "osqp::osqp;OsqpEigen::OsqpEigen")
add_custom_executable(test_utils "${test_utils_SRCS}" "")
add_custom_executable(test_HOQP_slack "${test_HOQP_slack_SRCS}" "osqp::osqp;OsqpEigen::OsqpEigen")
add_custom_executable(test_slope "${test_slope_SRCS}" "osqp::osqp;OsqpEigen::OsqpEigen")
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/rsc DESTINATION ${CMAKE_BINARY_DIR})
