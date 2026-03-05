#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <cmath>
#include <mutex>
#include <atomic>
#include <iomanip>

// Unitree DDS Headers
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/common/time/time_tool.hpp>

// Messages
#include "msg/ArmString_.hpp"
#include "msg/PubServoInfo_.hpp"

// Your MuJoCo IK Engine
#include "kinematics.hpp"

#define CMD_TOPIC   "rt/arm_Command"
#define STATE_TOPIC "current_servo_angle"   // ← was "rt/arm_State"

// Set 1 to print live joint feedback every ~1s — confirms subscriber is working.
// Set 0 for clean production output.
#define DEBUG_FEEDBACK 1

using namespace unitree::robot;
using namespace unitree::common;

const std::vector<double> FOLD_ANGLES = {-0.5, -88.3, 92.0, -3.3, 5.6, 0.0, -0.1};
const double GRIPPER_CLOSE = -0.1;
const double GRIPPER_OPEN  =  45.0;

class D1TaskSequence {
private:
    ChannelPublisher<unitree_arm::msg::dds_::ArmString_>     publisher;
    ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_> subscriber;

    D1Kinematics        ik_engine;
    std::vector<double> current_q;

    // Closed-loop state — written by DDS background thread, read by main thread
    std::vector<double> real_joint_deg;
    std::mutex          state_mutex;

    // BUG 3 FIX: count how many callbacks have fired.
    // If this stays 0 after startup, the topic name or network is wrong.
    std::atomic<int> feedback_count{0};

    // Sequence counter — must increment every command so arm doesn't deduplicate
    int seq = 2;

    void sendJsonCommand(const std::string& json_str) {
        unitree_arm::msg::dds_::ArmString_ msg{};
        msg.data_() = json_str;
        publisher.Write(msg);
    }

    void sleep_ms(int ms) {
        std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    }

    // =========================================================================
    // stateHandler — called by DDS background thread on every feedback message
    //
    // BUG 2: The field names servo0_data_() … servo5_data_() may not match
    // what your generated PubServoInfo_.hpp actually defines.
    // Open ~/go2_arm/d1_sdk/src/msg/PubServoInfo_.hpp and search for _()
    // methods to find the real accessor names.
    //
    // Common variants in different D1 SDK builds:
    //   OPTION A  msg->servo0_data_() … msg->servo5_data_()   ← individual floats
    //   OPTION B  msg->q_()[i]                                 ← array, radians → need *180/π
    //   OPTION C  msg->servo_info_()[i].pos()                  ← array of structs
    //
    // The DEBUG_FEEDBACK print below will immediately show if values are
    // all-zero (wrong field name) or sensible degrees (correct).
    // =========================================================================
    void stateHandler(const void* message) {
        auto msg = (const unitree_arm::msg::dds_::PubServoInfo_*)message;

        std::lock_guard<std::mutex> lock(state_mutex);

        // ── OPTION A: individual float fields (most common in D1 SDK) ────
        real_joint_deg[0] = static_cast<double>(msg->servo0_data_());
        real_joint_deg[1] = static_cast<double>(msg->servo1_data_());
        real_joint_deg[2] = static_cast<double>(msg->servo2_data_());
        real_joint_deg[3] = static_cast<double>(msg->servo3_data_());
        real_joint_deg[4] = static_cast<double>(msg->servo4_data_());
        real_joint_deg[5] = static_cast<double>(msg->servo5_data_());
        // servo6_data_() = gripper, not checked in waitForArrival

        // ── OPTION B (uncomment if OPTION A compiles but reads all zeros):
        // for (int i = 0; i < 6; i++)
        //     real_joint_deg[i] = msg->q_()[i] * (180.0 / M_PI);

        // ── OPTION C (uncomment if the above don't match your header):
        // for (int i = 0; i < 6; i++)
        //     real_joint_deg[i] = static_cast<double>(msg->servo_info_()[i].pos());

        int count = ++feedback_count;
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
        subscriber.InitChannel(
            std::bind(&D1TaskSequence::stateHandler, this, std::placeholders::_1)
        );
        sleep_ms(500); // let DDS establish before first command
    }

    // =========================================================================
    // checkFeedback — call ONCE after construction before anything else.
    // Tells you immediately if the subscriber is working.
    // =========================================================================
    void checkFeedback(int wait_ms = 2000) {
        std::cout << "[CHECK] Waiting " << wait_ms
                  << "ms for PubServoInfo_ feedback on topic: " << STATE_TOPIC
                  << std::endl;
        sleep_ms(wait_ms);

        int count = feedback_count.load();
        if (count == 0) {
            std::cout << "\n[CHECK] FAIL: 0 feedback messages received." << std::endl;
            std::cout << "        This is WHY waitForArrival always times out." << std::endl;
            std::cout << "        → Try topic: \"rt/arm_Feedback\" or \"arm_Feedback\"" << std::endl;
            std::cout << "        → Check network interface in ChannelFactory::Init()" << std::endl;
        } else {
            std::cout << "\n[CHECK] OK: " << count << " messages received ("
                      << (count * 1000 / wait_ms) << " Hz)" << std::endl;

            std::lock_guard<std::mutex> lock(state_mutex);
            bool all_zero = true;
            for (double v : real_joint_deg) if (std::abs(v) > 0.01) { all_zero = false; break; }

            if (all_zero) {
                std::cout << "[CHECK] WARN: All joint values are 0.0 — field names may be wrong." << std::endl;
                std::cout << "        Open PubServoInfo_.hpp and check accessor method names." << std::endl;
                std::cout << "        Try OPTION B or OPTION C in stateHandler." << std::endl;
            } else {
                std::cout << "[CHECK] Joint readings look valid." << std::endl;
            }

            for (int i = 0; i < 6; i++)
                std::cout << "        J" << i << " = " << std::fixed << std::setprecision(2)
                          << real_joint_deg[i] << " deg" << std::endl;
        }
        std::cout << std::endl;
    }

    void enableArm() {
        std::cout << "[SYSTEM] Enabling Arm Motors (mode:1 = LOCK)..." << std::endl;
        // mode:1 = LOCK/enable, mode:0 = FREE/discharge
        sendJsonCommand("{\"seq\":1,\"address\":1,\"funcode\":5,\"data\":{\"mode\":1}}");
        sleep_ms(1000);
    }

    void moveJoints(const std::vector<double>& angles, double gripper_angle) {
        if (angles.size() < 6) return;

        std::string json =
            "{\"seq\":"  + std::to_string(seq++) + ","
            "\"address\":1,\"funcode\":2,\"data\":{\"mode\":1,"
            "\"angle0\":" + std::to_string(angles[0]) + ","
            "\"angle1\":" + std::to_string(angles[1]) + ","
            "\"angle2\":" + std::to_string(angles[2]) + ","
            "\"angle3\":" + std::to_string(angles[3]) + ","
            "\"angle4\":" + std::to_string(angles[4]) + ","
            "\"angle5\":" + std::to_string(angles[5]) + ","
            "\"angle6\":" + std::to_string(gripper_angle) + "}}";
        sendJsonCommand(json);
    }

    // =========================================================================
    // waitForArrival — blocks until all 6 joints are within tolerance,
    // or until timeout. Prints a detailed error table on timeout so you can
    // see exactly which joint is not converging and by how much.
    // =========================================================================
    bool waitForArrival(const std::vector<double>& target_deg,
                        double tolerance  = 3.0,
                        int    timeout_ms = 18000) {
        auto start = std::chrono::steady_clock::now();
        std::cout << "\n[WAIT] Target: ";
        for (int i = 0; i < 6; i++)
            std::cout << "J" << i << ":" << std::fixed << std::setprecision(1)
                      << target_deg[i] << "  ";
        std::cout << std::endl;

        while (true) {
            long elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                               std::chrono::steady_clock::now() - start).count();

            if (elapsed > timeout_ms) {
                std::cout << "\n[WARN] Timeout after " << timeout_ms << "ms." << std::endl;

                // Print per-joint breakdown — shows which joint is the problem
                {
                    std::lock_guard<std::mutex> lock(state_mutex);
                    for (int i = 0; i < 6; i++) {
                        double err = std::abs(target_deg[i] - real_joint_deg[i]);
                        std::cout << "       J" << i
                                  << " target=" << std::setw(8) << std::fixed
                                  << std::setprecision(1) << target_deg[i]
                                  << "  actual=" << std::setw(8) << real_joint_deg[i]
                                  << "  err=" << std::setw(6) << err
                                  << (err > tolerance ? "  NOT REACHED <--" : "  OK")
                                  << std::endl;
                    }
                }

                if (feedback_count.load() == 0)
                    std::cout << "[WARN] feedback_count=0: subscriber never fired! Fix topic name." << std::endl;

                return false;
            }

            bool all_reached = true;
            {
                std::lock_guard<std::mutex> lock(state_mutex);
                for (int i = 0; i < 6; i++) {
                    if (std::abs(target_deg[i] - real_joint_deg[i]) > tolerance) {
                        all_reached = false;
                        break;
                    }
                }
            }

            if (all_reached) {
                std::cout << "[SUCCESS] Arm reached target." << std::endl;
                return true;
            }

            sleep_ms(50);
        }
    }

    void moveToFold() {
        std::cout << "[ACTION] Moving to Folding Pose..." << std::endl;
        moveJoints(FOLD_ANGLES, GRIPPER_CLOSE);
        waitForArrival(std::vector<double>(FOLD_ANGLES.begin(), FOLD_ANGLES.begin() + 6));
        current_q = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    }

    void performPick(double x, double y, double z) {
        std::cout << "\n--- STARTING PICK TASK ---" << std::endl;
        Eigen::Vector3d target_pos(x, y, z);

        try {
            std::vector<double> target_angles_rad = ik_engine.solve_ik(target_pos, current_q);
            current_q = target_angles_rad;

            std::vector<double> target_deg(6);
            for (int i = 0; i < 6; i++)
                target_deg[i] = target_angles_rad[i] * (180.0 / M_PI);

            moveToFold();

            std::cout << "[ACTION] Opening Gripper..." << std::endl;
            moveJoints(std::vector<double>(FOLD_ANGLES.begin(), FOLD_ANGLES.begin() + 6),
                       GRIPPER_OPEN);
            sleep_ms(800);

            std::cout << "[ACTION] Moving to Target ("
                      << x << ", " << y << ", " << z << ")..." << std::endl;
            moveJoints(target_deg, GRIPPER_OPEN);
            waitForArrival(target_deg);

            std::cout << "[ACTION] Closing Gripper..." << std::endl;
            moveJoints(target_deg, GRIPPER_CLOSE);
            sleep_ms(800);

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
            for (int i = 0; i < 6; i++)
                target_deg[i] = target_angles_rad[i] * (180.0 / M_PI);

            moveToFold();

            std::cout << "[ACTION] Moving to Put Target..." << std::endl;
            moveJoints(target_deg, GRIPPER_CLOSE);
            waitForArrival(target_deg);

            std::cout << "[ACTION] Releasing Object..." << std::endl;
            moveJoints(target_deg, GRIPPER_OPEN);
            sleep_ms(800);

            moveToFold();
            std::cout << "--- PUT TASK COMPLETE ---\n" << std::endl;

        } catch (const std::exception& e) {
            std::cout << "[ERROR] IK Failed: " << e.what() << std::endl;
        }
    }
};

int main() {
    D1TaskSequence arm;

    // Run feedback health check FIRST — tells you immediately if topic/fields are wrong
    arm.checkFeedback(2000);

    arm.enableArm();
    arm.moveToFold();

    std::string command;
    double x, y, z;
    std::cout << "Ready! Commands: 'pick x y z'  |  'put x y z'  |  'exit'" << std::endl;

    while (std::cout << "> " && std::cin >> command) {
        if (command == "exit") break;
        if (command == "pick" || command == "put") {
            if (!(std::cin >> x >> y >> z)) break;
            if (command == "pick") arm.performPick(x, y, z);
            else                   arm.performPut(x, y, z);
        } else {
            std::cout << "Unknown command." << std::endl;
        }
    }
    return 0;
}