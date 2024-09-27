#include <iostream>
#include <Eigen/Dense>

/*
// subfolder-2/main.cpp
#include "../custom_math/custom_math.h"

Eigen::Matrix4d ur3e_fk_screw(const std::vector<double> &joint_positions) {

    double l1 = -0.24355;
    double l2 = -0.2132;
    double w1 = 0.13105;
    double w2 = 0.0921;
    double h1 = 0.15185;
    double h2 = 0.08535;

    Eigen::Matrix4d M;
    M << -1, 0, 0, l1 + l2,
          0, 0, 1, w1 + w2,
          0, 1, 0, h1 - h2,
          0, 0, 0, 1;

    Eigen::Vector3d w_1 = {0.0, 0.0, 1.0};
    Eigen::Vector3d v_1 = {0.0, 0.0, 0.0};

    Eigen::Vector3d w_2 = {0.0, 1.0, 0.0};
    Eigen::Vector3d v_2 = {-h1, 0.0, 0.0};

    Eigen::Vector3d w_3 = {0.0, 1.0, 0.0};
    Eigen::Vector3d v_3 = {-h1, 0.0, l1};

    Eigen::Vector3d w_4 = {0.0, 1.0, 0.0};
    Eigen::Vector3d v_4 = {-h1, 0.0, l1 + l2};

    Eigen::Vector3d w_5 = {0.0, 0.0, -1.0};
    Eigen::Vector3d v_5 = {-w1, (l1 + l2), 0.0};

    Eigen::Vector3d w_6 = {0.0, 1.0, 0.0};
    Eigen::Vector3d v_6 = {h2 - h1, 0, l1 + l2};

    Eigen::Matrix4d e_s1 = matrix_exponential_transformation(w_1, v_1, joint_positions[0]);
    Eigen::Matrix4d e_s2 = matrix_exponential_transformation(w_2, v_2, joint_positions[1]);
    Eigen::Matrix4d e_s3 = matrix_exponential_transformation(w_3, v_3, joint_positions[2]);
    Eigen::Matrix4d e_s4 = matrix_exponential_transformation(w_4, v_4, joint_positions[3]);
    Eigen::Matrix4d e_s5 = matrix_exponential_transformation(w_5, v_5, joint_positions[4]);
    Eigen::Matrix4d e_s6 = matrix_exponential_transformation(w_6 ,v_6, joint_positions[5]);

    return e_s1 * e_s2 * e_s3 * e_s4 * e_s5 *e_s6 * M;
}

Eigen::Matrix4d ur3e_space_fk(const Eigen::VectorXd &joint_positions) {
    auto [m, space_screws] = ur3e_space_chain();
    Eigen::Matrix4d t06 = Eigen::Matrix4d::Identity();
    for(int i = 0; 1 < joint_positions.size(); i++)
        t06 *= matrix_exponential_transformation(space_scews[i], joint_positions[i]);
    return t06 * m;
}

int main() {

    return 0;
}
*/