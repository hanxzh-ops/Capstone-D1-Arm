#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

#include <unitree/robot/channel/channel_publisher.hpp>
#include "msg/ArmString_.hpp"

using namespace unitree::robot;

static constexpr const char* kTopic = "rt/arm_Command";

static void sleep_ms(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

static std::string sh_quote(const std::string& s) {
    // Safe single-quote shell escaping
    std::string out = "'";
    for (char c : s) {
        if (c == '\'') out += "'\\''";
        else out += c;
    }
    out += "'";
    return out;
}

static std::array<double, 6> solve_ik_with_python(
    const std::string& py_script,
    const std::string& urdf,
    double x, double y, double z
) {
    std::ostringstream cmd;
    cmd << "python3 " << sh_quote(py_script)
        << " --urdf " << sh_quote(urdf)
        << " --x " << x
        << " --y " << y
        << " --z " << z;

    FILE* pipe = popen(cmd.str().c_str(), "r");
    if (!pipe) throw std::runtime_error("Failed to start IK script");

    char buffer[512];
    std::string output;
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        output += buffer;
    }
    int rc = pclose(pipe);
    if (rc != 0) {
        throw std::runtime_error("IK script failed. Output:\n" + output);
    }

    std::array<double, 6> q{};
    std::stringstream ss(output);
    for (int i = 0; i < 6; ++i) {
        if (!(ss >> q[i])) {
            throw std::runtime_error("Failed to parse IK output: " + output);
        }
    }
    return q;
}

class D1ArmApi {
public:
    D1ArmApi() {
        ChannelFactory::Instance()->Init(0);
        pub_ = std::make_unique<ChannelPublisher<unitree_arm::msg::dds_::ArmString_>>(kTopic);
        pub_->InitChannel();
    }

    void power(bool on) {
        std::ostringstream data;
        data << "{\"power\":" << (on ? 1 : 0) << "}";
        send(6, data.str(), true);
    }

    void enable_all(bool enable) {
        std::ostringstream data;
        data << "{\"mode\":" << (enable ? 1 : 0) << "}";
        send(5, data.str(), true);
    }

    void set_joint(int id, double angle_deg, int delay_ms = 0) {
        std::ostringstream data;
        data << "{\"id\":" << id
             << ",\"angle\":" << angle_deg
             << ",\"delay_ms\":" << delay_ms << "}";
        send(1, data.str(), true);
    }

    void set_all_joints(const std::array<double, 7>& q_deg, int mode = 1) {
        std::ostringstream data;
        data << "{\"mode\":" << mode
             << ",\"angle0\":" << q_deg[0]
             << ",\"angle1\":" << q_deg[1]
             << ",\"angle2\":" << q_deg[2]
             << ",\"angle3\":" << q_deg[3]
             << ",\"angle4\":" << q_deg[4]
             << ",\"angle5\":" << q_deg[5]
             << ",\"angle6\":" << q_deg[6] << "}";
        send(2, data.str(), true);
    }

    void home() {
        send(7, "", false);
    }

private:
    std::unique_ptr<ChannelPublisher<unitree_arm::msg::dds_::ArmString_>> pub_;
    uint32_t seq_{1};

    void send(int funcode, const std::string& data_json, bool has_data) {
        std::ostringstream payload;
        payload << "{\"seq\":" << seq_++
                << ",\"address\":1"
                << ",\"funcode\":" << funcode;
        if (has_data) {
            payload << ",\"data\":" << data_json;
        }
        payload << "}";

        unitree_arm::msg::dds_::ArmString_ msg{};
        msg.data_() = payload.str();
        pub_->Write(msg);

        std::cout << "[D1 CMD] " << msg.data_() << std::endl;
    }
};

static double stroke_mm_to_angle6_deg(double stroke_mm,
                                      double open_deg = 5.0,
                                      double close_deg = 55.0) {
    // Calibrate once on your hardware:
    // assume 65mm -> open_deg, 0mm -> close_deg
    if (stroke_mm < 0.0) stroke_mm = 0.0;
    if (stroke_mm > 65.0) stroke_mm = 65.0;
    const double t = (65.0 - stroke_mm) / 65.0; // 0(open) ... 1(close)
    return open_deg + t * (close_deg - open_deg);
}

int main(int argc, char** argv) {
    // Usage:
    // ./d1_pick_xyz <x> <y> <z> <ik_script.py> <D1-550.urdf>
    if (argc < 6) {
        std::cerr << "Usage: " << argv[0]
                  << " <x> <y> <z> <ik_script.py> <D1-550.urdf>\n";
        return 1;
    }

    const double x = std::stod(argv[1]);
    const double y = std::stod(argv[2]);
    const double z = std::stod(argv[3]);
    const std::string ik_script = argv[4];
    const std::string urdf = argv[5];

    // 1) Solve IK for J0..J5
    std::array<double, 6> q6 = solve_ik_with_python(ik_script, urdf, x, y, z);

    // 2) Build target with gripper open first
    const double gripper_open_deg  = stroke_mm_to_angle6_deg(65.0); // open
    const double gripper_close_deg = stroke_mm_to_angle6_deg(10.0); // close target

    std::array<double, 7> q_target_open{};
    for (int i = 0; i < 6; ++i) q_target_open[i] = q6[i];
    q_target_open[6] = gripper_open_deg;

    D1ArmApi arm;

    // Safe bring-up
    arm.power(true);
    sleep_ms(300);
    arm.enable_all(true);
    sleep_ms(300);

    // === Required sequence ===
    // A) Open gripper
    arm.set_joint(6, gripper_open_deg, 0);
    sleep_ms(500);

    // B) Move arm to XYZ (keeping gripper open)
    arm.set_all_joints(q_target_open, 1);   // mode=1: trajectory smoothing
    sleep_ms(1800);

    // C) Close gripper
    arm.set_joint(6, gripper_close_deg, 0);
    sleep_ms(900);

    // D) Move back to default/home
    arm.home();
    sleep_ms(2200);

    std::cout << "Task complete.\n";
    return 0;
}
