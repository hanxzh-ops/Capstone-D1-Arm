#pragma once

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/common/time/time_tool.hpp>
#include "msg/ArmString_.hpp"
#include <iostream>
#include <sstream>
#include <vector>
#include <cmath>
#include <unistd.h>

#define COMMAND_TOPIC "rt/arm_Command"

using namespace unitree::robot;

class D1ArmController {
private:
    ChannelPublisher<unitree_arm::msg::dds_::ArmString_> publisher;

    void publish_cmd(const std::string& json_str) {
        unitree_arm::msg::dds_::ArmString_ msg{};
        msg.data_() = json_str;
        publisher.Write(msg);
        std::cout << "[DDS] Sent Command: " << json_str << std::endl;
    }

public:
    // IMPORTANT: Make sure the network interface matches what worked for you (e.g., "eth0", "enp3s0")
    D1ArmController(const std::string& network_iface = "eth0") : publisher(COMMAND_TOPIC) {
        ChannelFactory::Instance()->Init(0, network_iface);
        publisher.InitChannel();
    }

    void enable() {
        std::cout << "Enabling Arm..." << std::endl;
        publish_cmd("{\"seq\":1,\"address\":1,\"funcode\":5,\"data\":{\"mode\":0}}");
        sleep(1);
    }

    void zero() {
        std::cout << "Moving to Zero Position..." << std::endl;
        publish_cmd("{\"seq\":2,\"address\":1,\"funcode\":7}");
        sleep(3); // Give it time to physically move to zero
    }

    // Takes arm angles in RADIANS (from MuJoCo) and gripper angle in DEGREES
    void move_joints(const std::vector<double>& q_rad, double gripper_deg) {
        if(q_rad.size() < 6) {
            std::cerr << "Error: Need 6 joint angles." << std::endl;
            return;
        }
        
        // Convert radians to degrees for the D1 Hardware
        int a0 = std::round(q_rad[0] * 180.0 / M_PI);
        int a1 = std::round(q_rad[1] * 180.0 / M_PI);
        int a2 = std::round(q_rad[2] * 180.0 / M_PI);
        int a3 = std::round(q_rad[3] * 180.0 / M_PI);
        int a4 = std::round(q_rad[4] * 180.0 / M_PI);
        int a5 = std::round(q_rad[5] * 180.0 / M_PI);
        int a6 = std::round(gripper_deg);

        // Dynamically build the JSON string for funcode 2 (Multiple Joint Control)
        std::ostringstream oss;
        oss << "{\"seq\":4,\"address\":1,\"funcode\":2,\"data\":{"
            << "\"mode\":1,"
            << "\"angle0\":" << a0 << ","
            << "\"angle1\":" << a1 << ","
            << "\"angle2\":" << a2 << ","
            << "\"angle3\":" << a3 << ","
            << "\"angle4\":" << a4 << ","
            << "\"angle5\":" << a5 << ","
            << "\"angle6\":" << a6 << "}}";
            
        publish_cmd(oss.str());
    }
};