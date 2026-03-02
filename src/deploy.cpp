#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <cmath>
#include <mutex>
#include <atomic>

// Unitree DDS Headers
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/common/time/time_tool.hpp>
#include "msg/ArmString_.hpp"

// Your MuJoCo IK Engine
#include "kinematics.hpp"

#define TOPIC_CMD      "rt/arm_Command"
#define TOPIC_FEEDBACK "rt/arm Feedback"   // NOTE: space in topic name (confirmed from D1 SDK)

// How close (in degrees) each joint must be to the target before we act
#define POSITION_THRESHOLD_DEG 3.0

// Maximum time (ms) to wait for position before falling back
#define POSITION_WAIT_TIMEOUT_MS 8000

using namespace unitree::robot;
using namespace unitree::common;

// --- ARM CONFIGURATION ---
const std::vector<double> FOLD_ANGLES   = {-0.5, -88.3, 92.0, -3.3, 5.6, 0.0, -0.1};
const double GRIPPER_CLOSE = -0.1;
const double GRIPPER_OPEN  =  45.0;

// ---------------------------------------------------------------------------
// Feedback parser
// The D1 arm publishes a JSON string on "rt/arm Feedback", e.g.:
//   {"angle0":1.2,"angle1":-88.0,"angle2":91.5,"angle3":-3.1,
//    "angle4":5.5,"angle5":0.1,"angle6":-0.1,"power":1,"enable":1}
// We do a lightweight parse (no external JSON lib needed).
// ---------------------------------------------------------------------------
static double extractJsonDouble(const std::string& json, const std::string& key) {
    // Find  "key":VALUE
    std::string search = "\"" + key + "\":";
    size_t pos = json.find(search);
    if (pos == std::string::npos) return 0.0;
    pos += search.size();
    size_t end = json.find_first_of(",}", pos);
    return std::stod(json.substr(pos, end - pos));
}

// ---------------------------------------------------------------------------
// D1TaskSequence
// ---------------------------------------------------------------------------
class D1TaskSequence {
private:
    ChannelPublisher<unitree_arm::msg::dds_::ArmString_>  publisher;
    ChannelSubscriber<unitree_arm::msg::dds_::ArmString_> subscriber;
    D1Kinematics ik_engine;

    // Latest feedback from the arm (protected by mutex)
    std::mutex          feedback_mutex;
    std::vector<double> feedback_angles;   // degrees, 7 elements (joints 0-5 + gripper)
    std::atomic<bool>   feedback_received{false};

    // -----------------------------------------------------------------------
    void sendJsonCommand(const std::string& json_str) {
        unitree_arm::msg::dds_::ArmString_ msg{};
        msg.data_() = json_str;
        publisher.Write(msg);
    }

    void sleep_ms(int ms) {
        std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    }

    // -----------------------------------------------------------------------
    // Feedback callback — called by the DDS subscriber thread
    // -----------------------------------------------------------------------
    void onFeedback(const void* msg_ptr) {
        const auto* arm_msg = static_cast<const unitree_arm::msg::dds_::ArmString_*>(msg_ptr);
        const std::string& json = arm_msg->data_();

        std::vector<double> angles(7);
        for (int i = 0; i < 7; ++i) {
            angles[i] = extractJsonDouble(json, "angle" + std::to_string(i));
        }

        {
            std::lock_guard<std::mutex> lk(feedback_mutex);
            feedback_angles = angles;
            feedback_received = true;
        }
    }

    // -----------------------------------------------------------------------
    // Returns a snapshot of the latest joint angles (degrees).
    // Returns empty vector if no feedback has arrived yet.
    // -----------------------------------------------------------------------
    std::vector<double> getLatestAngles() {
        std::lock_guard<std::mutex> lk(feedback_mutex);
        return feedback_angles;
    }

    // -----------------------------------------------------------------------
    // Block until the arm joints (indices 0-5) are all within
    // POSITION_THRESHOLD_DEG of target_deg[], OR the timeout elapses.
    // Returns true if position reached, false if timed out.
    // -----------------------------------------------------------------------
    bool waitUntilReached(const std::vector<double>& target_deg,
                          int timeout_ms = POSITION_WAIT_TIMEOUT_MS) {
        auto start = std::chrono::steady_clock::now();

        while (true) {
            // Check timeout
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                               std::chrono::steady_clock::now() - start).count();
            if (elapsed >= timeout_ms) {
                std::cout << "[WARN] waitUntilReached: timeout after "
                          << timeout_ms << " ms" << std::endl;
                return false;
            }

            if (feedback_received) {
                std::vector<double> current = getLatestAngles();

                if (current.size() >= 6) {
                    bool reached = true;
                    for (int i = 0; i < 6; ++i) {
                        if (std::fabs(current[i] - target_deg[i]) > POSITION_THRESHOLD_DEG) {
                            reached = false;
                            break;
                        }
                    }
                    if (reached) {
                        std::cout << "[INFO] Position reached." << std::endl;
                        return true;
                    }
                }
            }

            sleep_ms(50); // poll at 20 Hz
        }
    }

    // -----------------------------------------------------------------------
    // Same as waitUntilReached but also checks FOLD_ANGLES
    // -----------------------------------------------------------------------
    bool waitUntilFolded(int timeout_ms = POSITION_WAIT_TIMEOUT_MS) {
        std::vector<double> fold_deg = {
            FOLD_ANGLES[0], FOLD_ANGLES[1], FOLD_ANGLES[2],
            FOLD_ANGLES[3], FOLD_ANGLES[4], FOLD_ANGLES[5]
        };
        return waitUntilReached(fold_deg, timeout_ms);
    }

    // -----------------------------------------------------------------------
    std::vector<double> current_q; // IK seed (joint angles in radians)

public:
    // -----------------------------------------------------------------------
    D1TaskSequence()
        : publisher(TOPIC_CMD),
          subscriber(TOPIC_FEEDBACK),
          ik_engine("/home/icon-labtop/go2_arm/d1_sdk/d1_kinematics.xml"),
          current_q(6, 0.0),
          feedback_angles(7, 0.0)
    {
        ChannelFactory::Instance()->Init(0);
        publisher.InitChannel();

        // Register feedback callback
        subscriber.InitChannel(
            std::bind(&D1TaskSequence::onFeedback, this, std::placeholders::_1));

        // Give DDS a moment to start receiving
        sleep_ms(500);
    }

    // -----------------------------------------------------------------------
    void enableArm() {
        std::cout << "[SYSTEM] Enabling Arm Motors..." << std::endl;
        sendJsonCommand("{\"seq\":1,\"address\":1,\"funcode\":5,\"data\":{\"mode\":0}}");
        sleep_ms(1000);
    }

    // -----------------------------------------------------------------------
    void moveJoints(const std::vector<double>& angles, double gripper_angle) {
        if (angles.size() < 6) return;

        std::string json =
            "{\"seq\":2,\"address\":1,\"funcode\":2,\"data\":{\"mode\":1,"
            "\"angle0\":" + std::to_string(angles[0]) + ","
            "\"angle1\":" + std::to_string(angles[1]) + ","
            "\"angle2\":" + std::to_string(angles[2]) + ","
            "\"angle3\":" + std::to_string(angles[3]) + ","
            "\"angle4\":" + std::to_string(angles[4]) + ","
            "\"angle5\":" + std::to_string(angles[5]) + ","
            "\"angle6\":" + std::to_string(gripper_angle) + "}}";
        sendJsonCommand(json);
    }

    // -----------------------------------------------------------------------
    void moveToFold() {
        std::cout << "[ACTION] Moving to Folding Pose..." << std::endl;

        std::vector<double> fold_deg(FOLD_ANGLES.begin(), FOLD_ANGLES.begin() + 6);
        moveJoints(fold_deg, GRIPPER_CLOSE);

        waitUntilFolded(); // <-- feedback-based wait

        // Reset IK seed
        current_q.assign(6, 0.0);
    }

    // -----------------------------------------------------------------------
    void performPick(double x, double y, double z) {
        std::cout << "\n--- STARTING PICK TASK ---" << std::endl;

        Eigen::Vector3d target_pos(x, y, z);

        try {
            // Solve IK
            std::vector<double> target_rad = ik_engine.solve_ik(target_pos, current_q);
            current_q = target_rad;

            // Convert to degrees
            std::vector<double> target_deg(6);
            for (int i = 0; i < 6; ++i)
                target_deg[i] = target_rad[i] * (180.0 / M_PI);

            // 1. Fold arm first
            moveToFold();

            // 2. Open gripper while still folded
            std::cout << "[ACTION] Opening Gripper..." << std::endl;
            std::vector<double> fold_deg(FOLD_ANGLES.begin(), FOLD_ANGLES.begin() + 6);
            moveJoints(fold_deg, GRIPPER_OPEN);
            sleep_ms(500); // gripper servo settle time (mechanical, not position-based)

            // 3. Move arm to target — wait until position is actually reached
            std::cout << "[ACTION] Moving to Target ("
                      << x << ", " << y << ", " << z << ")..." << std::endl;
            moveJoints(target_deg, GRIPPER_OPEN);

            bool reached = waitUntilReached(target_deg);

            // 4. Close gripper once at position (or after timeout)
            if (reached) {
                std::cout << "[ACTION] Position confirmed — Closing Gripper." << std::endl;
            } else {
                std::cout << "[WARN] Closing Gripper after timeout (best-effort)." << std::endl;
            }
            moveJoints(target_deg, GRIPPER_CLOSE);
            sleep_ms(600); // gripper close mechanical settle

            // 5. Retreat to fold
            moveToFold();
            std::cout << "--- PICK TASK COMPLETE ---\n" << std::endl;

        } catch (const std::exception& e) {
            std::cout << "[ERROR] IK Failed: " << e.what() << std::endl;
        }
    }

    // -----------------------------------------------------------------------
    void performPut(double x, double y, double z) {
        std::cout << "\n--- STARTING PUT TASK ---" << std::endl;

        Eigen::Vector3d target_pos(x, y, z);

        try {
            std::vector<double> target_rad = ik_engine.solve_ik(target_pos, current_q);
            current_q = target_rad;

            std::vector<double> target_deg(6);
            for (int i = 0; i < 6; ++i)
                target_deg[i] = target_rad[i] * (180.0 / M_PI);

            // 1. Fold first
            moveToFold();

            // 2. Move to put target (gripper stays closed, object is held)
            std::cout << "[ACTION] Moving to Put Target..." << std::endl;
            moveJoints(target_deg, GRIPPER_CLOSE);

            bool reached = waitUntilReached(target_deg);

            // 3. Release once actually at position
            if (reached) {
                std::cout << "[ACTION] Position confirmed — Releasing Object." << std::endl;
            } else {
                std::cout << "[WARN] Releasing after timeout (best-effort)." << std::endl;
            }
            moveJoints(target_deg, GRIPPER_OPEN);
            sleep_ms(600); // gripper open settle

            // 4. Retreat
            moveToFold();
            std::cout << "--- PUT TASK COMPLETE ---\n" << std::endl;

        } catch (const std::exception& e) {
            std::cout << "[ERROR] IK Failed: " << e.what() << std::endl;
        }
    }
};

// ---------------------------------------------------------------------------
int main() {
    D1TaskSequence arm;

    arm.enableArm();
    arm.moveToFold();

    std::string command;
    double x, y, z;
    std::cout << "Ready! Commands: 'pick x y z'  |  'put x y z'  |  'exit'" << std::endl;

    while (true) {
        std::cout << "> ";
        std::cin >> command;

        if (command == "exit") {
            break;
        } else if (command == "pick") {
            std::cin >> x >> y >> z;
            arm.performPick(x, y, z);
        } else if (command == "put") {
            std::cin >> x >> y >> z;
            arm.performPut(x, y, z);
        } else {
            std::cout << "Unknown command. Use 'pick', 'put', or 'exit'." << std::endl;
        }
    }

    std::cout << "Shutting down..." << std::endl;
    return 0;
}