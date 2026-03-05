project(marm_code)
cmake_minimum_required(VERSION 3.5)

SET(CMAKE_CXX_STANDARD 17)

include_directories(/usr/local/include/ddscxx /usr/local/include/iceoryx/v2.0.2)
link_libraries(unitree_sdk2 ddsc ddscxx rt pthread)

# Add Eigen and MuJoCo headers
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(/home/icon-labtop/anaconda3/envs/go2_arm/include)
link_directories(/home/icon-labtop/anaconda3/envs/go2_arm/lib)

# Compile the deployment executable
add_executable(deploy src/deploy.cpp src/msg/ArmString_.cpp src/msg/PubServoInfo_.cpp)

# Link libraries
target_link_libraries(deploy 
    mujoco 
)
# Compile the diagnostic executable
add_executable(diagnostic src/diagnostic.cpp src/msg/ArmString_.cpp src/msg/PubServoInfo_.cpp)
target_include_directories(diagnostic PRIVATE include/)
target_link_libraries(diagnostic unitree_sdk2 pthread)

# Origin control executables
add_executable(joint_angle_control src/joint_angle_control.cpp src/msg/ArmString_.cpp)
add_executable(multiple_joint_angle_control src/multiple_joint_angle_control.cpp src/msg/ArmString_.cpp)
add_executable(joint_enable_control src/joint_enable_control.cpp src/msg/ArmString_.cpp)
add_executable(arm_zero_control src/arm_zero_control.cpp src/msg/ArmString_.cpp)
add_executable(get_arm_joint_angle src/get_arm_joint_angle.cpp src/msg/ArmString_.cpp src/msg/PubServoInfo_.cpp)