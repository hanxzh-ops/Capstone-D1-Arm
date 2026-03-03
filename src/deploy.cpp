#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <cmath>
#include <mutex>

// Unitree DDS Headers
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/common/time/time_tool.hpp>

// Messages
#include "msg/ArmString_.hpp"
#include "msg/PubServoInfo_.hpp" 

// Your MuJoCo IK Engine
#include "kinematics.hpp" 

#define CMD_TOPIC "rt/arm_Command"
#define STATE_TOPIC "rt/arm_State"

using namespace unitree::robot;
using namespace unitree::common;

const std::vector<double> FOLD_ANGLES = {-0.5, -88.3, 92.0, -3.3, 5.6, 0.0, -0.1};
const double GRIPPER_CLOSE = -0.1; 
const double GRIPPER_OPEN = 45.0;  

class D1TaskSequence {
private:
    ChannelPublisher<unitree_arm::msg::dds_::ArmString_> publisher;
    ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_> subscriber;
    
    D1Kinematics ik_engine;
    std::vector<double> current_q; 
    
    // Closed-Loop State Tracking
    std::vector<double> real_joint_deg; 
    std::mutex state_mutex;

    void sendJsonCommand(const std::string& json_str) {
        unitree_arm::msg::dds_::ArmString_ msg{};
        msg.data_() = json_str;
        publisher.Write(msg);
    }

    void sleep_ms(int ms) {
        std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    }

    // FIXED: Changed msg->units() to msg->servo_info()
    void stateHandler(const void* message) {
        auto msg = (const unitree_arm::msg::dds_::PubServoInfo_*)message;
        
        std::lock_guard<std::mutex> lock(state_mutex);
        if (real_joint_deg.size() != 6) real_joint_deg.resize(6);

        // PubServoInfo_ has 7 individual floats, NOT an array.
        // servo0_data_() through servo5_data_() are joint angles in DEGREES already.
        real_joint_deg[0] = static_cast<double>(msg->servo0_data_());
        real_joint_deg[1] = static_cast<double>(msg->servo1_data_());
        real_joint_deg[2] = static_cast<double>(msg->servo2_data_());
        real_joint_deg[3] = static_cast<double>(msg->servo3_data_());
        real_joint_deg[4] = static_cast<double>(msg->servo4_data_());
        real_joint_deg[5] = static_cast<double>(msg->servo5_data_());
        // servo6_data_() is the gripper — not used in waitForArrival's 6-joint check
    }

public:
    D1TaskSequence() : 
        publisher(CMD_TOPIC), 
        subscriber(STATE_TOPIC),
        ik_engine("/home/icon-labtop/go2_arm/d1_sdk/d1_kinematics.xml"),
        current_q(6, 0.0),
        real_joint_deg(6, 0.0)
    {
        ChannelFactory::Instance()->Init(0);
        publisher.InitChannel();
        subscriber.InitChannel(std::bind(&D1TaskSequence::stateHandler, this, std::placeholders::_1));
    }

    void enableArm() {
        std::cout << "[SYSTEM] Enabling Arm Motors..." << std::endl;
        sendJsonCommand("{\"seq\":1,\"address\":1,\"funcode\":5,\"data\":{\"mode\":0}}");
        sleep_ms(1000); 
    }

    void moveJoints(const std::vector<double>& angles, double gripper_angle) {
        if(angles.size() < 6) return;
        
        std::string json = "{\"seq\":2,\"address\":1,\"funcode\":2,\"data\":{\"mode\":1,"
                           "\"angle0\":" + std::to_string(angles[0]) + ","
                           "\"angle1\":" + std::to_string(angles[1]) + ","
                           "\"angle2\":" + std::to_string(angles[2]) + ","
                           "\"angle3\":" + std::to_string(angles[3]) + ","
                           "\"angle4\":" + std::to_string(angles[4]) + ","
                           "\"angle5\":" + std::to_string(angles[5]) + ","
                           "\"angle6\":" + std::to_string(gripper_angle) + "}}";
        sendJsonCommand(json);
    }

    bool waitForArrival(const std::vector<double>& target_deg, double tolerance = 2.0, int timeout_ms = 6000) {
        auto start_time = std::chrono::steady_clock::now();
        std::cout << "[WAIT] Waiting for arm to reach target..." << std::endl;
        
        while (true) {
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count() > timeout_ms) {
                std::cout << "[WARN] Timeout reached. Proceeding anyway." << std::endl;
                return false;
            }

            bool all_reached = true;
            {
                std::lock_guard<std::mutex> lock(state_mutex);
                for (int i = 0; i < 6; i++) {
                    double error = std::abs(target_deg[i] - real_joint_deg[i]);
                    if (error > tolerance) {
                        all_reached = false;
                        break;
                    }
                }
            }

            if (all_reached) {
                std::cout << "[SUCCESS] Target reached!" << std::endl;
                return true;
            }
            sleep_ms(50);
        }
    }

    void moveToFold() {
        std::cout << "[ACTION] Moving to Folding Pose..." << std::endl;
        moveJoints(FOLD_ANGLES, GRIPPER_CLOSE);
        waitForArrival(FOLD_ANGLES); 
        current_q = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
    }

    void performPick(double x, double y, double z) {
        std::cout << "\n--- STARTING PICK TASK ---" << std::endl;
        Eigen::Vector3d target_pos(x, y, z);
        
        try {
            std::vector<double> target_angles_rad = ik_engine.solve_ik(target_pos, current_q);
            current_q = target_angles_rad; 
            
            std::vector<double> target_deg(6);
            for(int i = 0; i < 6; i++) {
                target_deg[i] = target_angles_rad[i] * (180.0 / M_PI);
            }

            moveToFold();

            std::cout << "[ACTION] Opening Gripper..." << std::endl;
            moveJoints(FOLD_ANGLES, GRIPPER_OPEN);
            sleep_ms(800);

            std::cout << "[ACTION] Moving to Target (" << x << ", " << y << ")..." << std::endl;
            moveJoints(target_deg, GRIPPER_OPEN);
            waitForArrival(target_deg);

            std::cout << "[ACTION] Pausing 0.5s for stability..." << std::endl;
            sleep_ms(500); 

            std::cout << "[ACTION] Closing Gripper..." << std::endl;
            moveJoints(target_deg, GRIPPER_CLOSE);
            sleep_ms(1000); 

            moveToFold();
            std::cout << "--- PICK TASK COMPLETE ---\n" << std::endl;

        } catch (const std::exception& e) {
            std::cout << "[ERROR] IK Failed: " << e.what() << std::endl;
        }
    }

    void performPut(double x, double y, double z) {
        std::cout << "\n--- STARTING PUT TASK ---" << std::endl;
        Eigen::Vector3d target_pos(x, y, z);
        try {
            std::vector<double> target_angles_rad = ik_engine.solve_ik(target_pos, current_q);
            current_q = target_angles_rad;
            std::vector<double> target_deg(6);
            for(int i = 0; i < 6; i++) target_deg[i] = target_angles_rad[i] * (180.0 / M_PI);

            moveToFold(); 
            std::cout << "[ACTION] Moving to Put Target..." << std::endl;
            moveJoints(target_deg, GRIPPER_CLOSE); 
            waitForArrival(target_deg);

            std::cout << "[ACTION] Pausing 0.5s..." << std::endl;
            sleep_ms(500);

            std::cout << "[ACTION] Releasing Object..." << std::endl;
            moveJoints(target_deg, GRIPPER_OPEN); 
            sleep_ms(1000);

            moveToFold();
            std::cout << "--- PUT TASK COMPLETE ---\n" << std::endl;
        } catch (const std::exception& e) {
            std::cout << "[ERROR] IK Failed: " << e.what() << std::endl;
        }
    }
};

int main() {
    D1TaskSequence arm;
    arm.enableArm();
    arm.moveToFold();

    std::string command;
    double x, y, z;
    std::cout << "Ready for commands! Type 'pick x y z' or 'put x y z'." << std::endl;

    while (std::cout << "> " && std::cin >> command) {
        if (command == "exit") break;
        if (command == "pick" || command == "put") {
            if (!(std::cin >> x >> y >> z)) break;
            if (command == "pick") arm.performPick(x, y, z);
            else arm.performPut(x, y, z);
        } else {
            std::cout << "Unknown command." << std::endl;
        }
    }
    return 0;
}