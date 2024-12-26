cmake_minimum_required(VERSION 3.10)
project(LGRC_Practice LANGUAGES CXX)

# Use C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler optimizations
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -march=armv8-a")
# Raisim 설정
set(raisim_DIR "/Users/yushin/raisim_build/lib/cmake/raisim")
find_package(raisim REQUIRED)

# External dependencies
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OsqpEigen REQUIRED)

# Include directories
include_directories(controller/include)
include_directories(controller/include/tasks)
include_directories(controller/include/tasks/Equality)
include_directories(controller/include/tasks/Inequality)
include_directories(utils)
include_directories(robot/include)

# 빌드 결과물 디렉토리 설정
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)   # 라이브러리 위치
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)  # 정적 라이브러리 위치

# Add Utils library
add_library(Utils_lib
    utils/Utils.cpp
)
target_include_directories(Utils_lib PUBLIC
    utils
)
target_link_libraries(Utils_lib PUBLIC Eigen3::Eigen)

# Add RobotStateRaisim library
add_library(RobotStateRaisim_lib
    robot/src/RobotState.cpp
    robot/src/RobotStateRaisim.cpp
)
target_include_directories(RobotStateRaisim_lib PUBLIC
    robot/include
)
target_link_libraries(RobotStateRaisim_lib PUBLIC raisim::raisim Eigen3::Eigen)

# Add OsqpSolver library
add_library(OsqpEigenSolver_lib
    controller/src/OsqpEigenSolver.cpp
)
target_include_directories(OsqpEigenSolver_lib PUBLIC
    controller/include
)
target_link_libraries(OsqpEigenSolver_lib PUBLIC OsqpEigen::OsqpEigen)

# Add TaskLS library
add_library(TaskLS_lib
    controller/src/Tasks/TaskSet.cpp
    controller/src/Tasks/TaskLS.cpp
    controller/src/tasks/Equality/TaskLS_ID.cpp
    controller/src/tasks/Equality/TaskLS_StationaryFeet.cpp
    controller/src/tasks/Equality/TaskLS_MoveBase.cpp
    controller/src/tasks/Equality/TaskLS_StationaryEE.cpp
    controller/src/tasks/Equality/TaskLS_MinMotion.cpp
    controller/src/tasks/Equality/TaskLS_EnergyOpt.cpp
    controller/src/tasks/Equality/TaskLS_LinearMomentum.cpp
    controller/src/tasks/Equality/TaskLS_AngularMomentum.cpp
    controller/src/tasks/Inequality/TaskLS_TorqueLimits.cpp
    controller/src/tasks/Inequality/TaskLS_FrictionCone.cpp
)
target_include_directories(TaskLS_lib PUBLIC
    controller/include
    controller/include/tasks
    controller/include/tasks/Equality
    controller/include/tasks/Inequality
)
target_link_libraries(TaskLS_lib PUBLIC raisim::raisim Eigen3::Eigen RobotStateRaisim_lib)

# Add HOQP library
add_library(HOQP_lib
    controller/src/HOQP.cpp
    controller/src/HOQP_Slack.cpp
)
target_include_directories(HOQP_lib PUBLIC
    controller/include
)
target_link_libraries(HOQP_lib PUBLIC OsqpEigenSolver_lib TaskLS_lib Utils_lib)

# Add ConvexMPC library
add_library(ConvexMPC_lib
    controller/src/ConvexMPC.cpp
)
target_include_directories(ConvexMPC_lib PUBLIC
    controller/include
)
target_link_libraries(ConvexMPC_lib PUBLIC Utils_lib OsqpEigenSolver_lib)



# Add executables
add_executable(test_joint_impedance test/joint_impedance.cpp)
target_link_libraries(test_joint_impedance PRIVATE raisim::raisim Utils_lib)

add_executable(test_id test/id_test.cpp)
target_link_libraries(test_id PRIVATE raisim::raisim Utils_lib)

# add_executable(test_osc test/osc.cpp)
# target_link_libraries(test_osc PRIVATE HOQP_lib)

# add_executable(test_nullspace test/nullspace_control.cpp)
# target_link_libraries(test_nullspace PRIVATE HOQP_lib)

# add_executable(test_bezier test/bezier_test.cpp)
# target_link_libraries(test_bezier PRIVATE TaskLS_lib)

# add_executable(test_base_qp test/base_qp.cpp)
# target_link_libraries(test_base_qp PRIVATE HOQP_lib)

# add_executable(test_osqp test/osqp_test.cpp)
# target_link_libraries(test_osqp PRIVATE OsqpEigen::OsqpEigen HOQP_lib)

add_executable(test_convex_mpc test/convex_mpc.cpp)
target_link_libraries(test_convex_mpc PUBLIC raisim::raisim)
target_link_libraries(test_convex_mpc PRIVATE ConvexMPC_lib)
# target_link_libraries(test_convex_mpc PUBLIC pthread)
# target_include_directories(test_convex_mpc PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(test_z1_spawn test/z1_spawn.cpp)
target_link_libraries(test_z1_spawn PRIVATE raisim::raisim Utils_lib)

add_executable(test_wbc_legged_arm test/wbc_legged_arm.cpp)
target_link_libraries(test_wbc_legged_arm PRIVATE HOQP_lib)

add_executable(test_hoqp test/hoqp_test.cpp)
target_link_libraries(test_hoqp PRIVATE HOQP_lib)

add_executable(test_utils test/utils_test.cpp)
target_link_libraries(test_utils PRIVATE Utils_lib)

add_executable(test_hoqp_slack test/hoqp_slack_test.cpp)
target_link_libraries(test_hoqp_slack PRIVATE HOQP_lib)

# add_executable(test_slope test/slope_test.cpp)
# target_link_libraries(test_slope PRIVATE HOQP_lib)

add_executable(test_linear_momentum test/linear_momentum_test.cpp)
target_link_libraries(test_linear_momentum PRIVATE HOQP_lib)

add_executable(test_momentum test/momentum_test.cpp)
target_link_libraries(test_momentum PRIVATE HOQP_lib yaml-cpp::yaml-cpp)

add_executable(test_robot_state test/robot_state_test.cpp)
target_link_libraries(test_robot_state PRIVATE RobotStateRaisim_lib)
# Copy resources (if applicable)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/rsc DESTINATION ${CMAKE_BINARY_DIR})