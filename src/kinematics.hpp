#ifndef D1_KINEMATICS_HPP
#define D1_KINEMATICS_HPP

#include <mujoco/mujoco.h>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <iostream>
#include <cmath>

class D1Kinematics {
private:
    mjModel* model;
    mjData* data;
    int site_id;
    std::vector<int> qpos_idx;
    std::vector<int> dof_idx;
    int nv;

public:
    D1Kinematics(const std::string& xml_path) {
        char error[1000] = "Could not load XML model";
        model = mj_loadXML(xml_path.c_str(), 0, error, 1000);
        if (!model) {
            std::cerr << "Load model error: " << error << std::endl;
            exit(1);
        }
        data = mj_makeData(model);
        nv = model->nv;

        site_id = mj_name2id(model, mjOBJ_SITE, "end_effector");
        if(site_id == -1) site_id = 0;

        // Get indices for Joint1 through Joint6
        for (int i = 1; i <= 6; i++) {
            std::string j_name = "Joint" + std::to_string(i);
            int j_id = mj_name2id(model, mjOBJ_JOINT, j_name.c_str());
            qpos_idx.push_back(model->jnt_qposadr[j_id]);
            dof_idx.push_back(model->jnt_dofadr[j_id]);
        }
    }

    ~D1Kinematics() {
        mj_deleteData(data);
        mj_deleteModel(model);
    }

    // Solves IK and returns exactly 6 joint angles in RADIANS
    std::vector<double> solve_ik(const Eigen::Vector3d& target_pos, const std::vector<double>& start_q) {
        double step_size = 1.0;
        double damping = 0.05;
        double tol = 0.005;

        std::vector<double> desired_q = start_q;
        Eigen::Vector3d current_pos;
        Eigen::Vector3d error;

        for (int attempt = 0; attempt < 3; attempt++) {
            for (int iter = 0; iter < 500; iter++) {
                // Apply desired_q to MuJoCo data
                for (size_t i = 0; i < 6; i++) data->qpos[qpos_idx[i]] = desired_q[i];
                
                mj_kinematics(model, data);
                mj_comPos(model, data);

                current_pos << data->site_xpos[3*site_id], data->site_xpos[3*site_id+1], data->site_xpos[3*site_id+2];
                error = target_pos - current_pos;

                if (error.norm() < tol) return desired_q;

                // Step-capping
                if (error.norm() > 0.05) {
                    error = error.normalized() * 0.05;
                }

                // Get full Jacobian
                std::vector<double> jacp(3 * nv, 0.0);
                mj_jacSite(model, data, jacp.data(), nullptr, site_id);

                // Extract Arm Jacobian using Eigen
                Eigen::MatrixXd J_arm(3, 6);
                for (int r = 0; r < 3; r++) {
                    for (int c = 0; c < 6; c++) {
                        J_arm(r, c) = jacp[r * nv + dof_idx[c]];
                    }
                }

                // DLS Math: delta_q = J^T * (J*J^T + lambda^2 * I)^-1 * error
                Eigen::Matrix3d J_Jt = J_arm * J_arm.transpose();
                Eigen::Matrix3d lambda_I = (damping * damping) * Eigen::Matrix3d::Identity();
                Eigen::VectorXd delta_q = J_arm.transpose() * (J_Jt + lambda_I).inverse() * error;

                for (int i = 0; i < 6; i++) {
                    desired_q[i] += step_size * delta_q(i);
                }
            }
            // Random restart if stuck
            for (int i = 0; i < 6; i++) {
                desired_q[i] += ((double)rand() / RAND_MAX - 0.5) * 0.6; 
            }
        }
        return desired_q;
    }
};

#endif