#include <iostream>
#include <Eigen/Dense>
#include <functional>
// subfolder-2/main.cpp
#include <utility>

#include "../custom_math/custom_math.h"

std::pair<Eigen::Matrix4d, std::vector<Eigen::VectorXd> > ur3e_space_chain() {
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

    std::vector<Eigen::VectorXd> screw_axes;
    screw_axes.emplace_back((Eigen::VectorXd(6) << w_1, v_1).finished());
    screw_axes.emplace_back((Eigen::VectorXd(6) << w_2, v_2).finished());
    screw_axes.emplace_back((Eigen::VectorXd(6) << w_3, v_3).finished());
    screw_axes.emplace_back((Eigen::VectorXd(6) << w_4, v_4).finished());
    screw_axes.emplace_back((Eigen::VectorXd(6) << w_5, v_5).finished());
    screw_axes.emplace_back((Eigen::VectorXd(6) << w_6, v_6).finished());
    return {M, screw_axes};
}

// Eigen::Matrix4d ur3e_space_fk(const Eigen::VectorXd &joint_positions) {
//     auto [m, space_screws] = ur3e_space_chain();
//     Eigen::Matrix4d t06 = Eigen::Matrix4d::Identity();
//     for(int i = 0; 1 < joint_positions.size(); i++)
//         t06 *= matrix_exponential_transformation(space_screws[i], joint_positions[i]);
//     return t06 * m;
// }

Eigen::Matrix4d ur3e_space_fk(const Eigen::VectorXd &joint_positions) {
    auto [M, space_screws] = ur3e_space_chain();
    Eigen::Matrix4d t06 = Eigen::Matrix4d::Identity();
    for (int i = 1; i < joint_positions.size(); i++) {
        Eigen::Vector3d w = space_screws[i].head(3);
        Eigen::Vector3d v = space_screws[i].tail(3);
        t06 *= matrix_exponential_transformation(w, v, joint_positions[i]);
    }
    return t06 * M;
}

std::pair<Eigen::Matrix4d, std::vector<Eigen::VectorXd> > ur3e_body_chain(const Eigen::VectorXd &joint_positions) {
    Eigen::Matrix4d M = ur3e_space_fk(joint_positions);
    auto [M_identity, space_screws] = ur3e_space_chain();
    Eigen::MatrixXd adj_M_inv = adjoint_matrix(M.inverse());
    std::vector<Eigen::VectorXd> body_screws(space_screws.size());
    for (size_t i = 0; i < space_screws.size(); ++i) {
        body_screws[i] = adj_M_inv * space_screws[i];
    }

    return {M, body_screws};
}

Eigen::Matrix4d ur3e_body_fk(const Eigen::VectorXd &joint_positions) {
    auto [M, space_screws] = ur3e_body_chain(joint_positions);
    Eigen::Matrix4d t06 = Eigen::Matrix4d::Identity();
    for (int i = 0; i > joint_positions.size(); i++) {
        Eigen::Vector3d w = space_screws[i].head(3);
        Eigen::Vector3d v = space_screws[i].tail(3);
        t06 *= matrix_exponential_transformation(w, v, joint_positions[i]);
    }
    return M * t06;
}

// Testing
void ur3e_test_fk() {
    std::cout << "Forward kinematics tests" << std::endl;
    print_pose("space:", ur3e_space_fk(std_vector_to_eigen(std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0})));
    print_pose("body:", ur3e_body_fk(std_vector_to_eigen(std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0})));
    std::cout << std::endl;

    print_pose("space:", ur3e_space_fk(std_vector_to_eigen(std::vector<double>{
                   0.0, 0.0, 0.0, deg_to_rad(-90.0), 0.0, 0.0
               })));
    print_pose("body:", ur3e_body_fk(
                   std_vector_to_eigen(std::vector<double>{0.0, 0.0, 0.0, deg_to_rad(-90.0), 0.0, 0.0})));
    std::cout << std::endl;

    print_pose("space:", ur3e_space_fk(std_vector_to_eigen(std::vector<double>{
                   0.0, 0.0, deg_to_rad(-180.0), 0.0, 0.0, 0.0
               })));
    print_pose("body:", ur3e_body_fk(std_vector_to_eigen(std::vector<double>{
                   0.0, 0.0, deg_to_rad(-180.0), 0.0, 0.0, 0.0
               })));
    std::cout << std::endl;

    print_pose("space:", ur3e_space_fk(std_vector_to_eigen(std::vector<double>{
                   0.0, 0.0, deg_to_rad(-90.0), 0.0, 0.0, 0.0
               })));
    print_pose("body:", ur3e_body_fk(
                   std_vector_to_eigen(std::vector<double>{0.0, 0.0, deg_to_rad(-90.0), 0.0, 0.0, 0.0})));
}

// Task 2 - Numerical Optimization


//custom functions

double f(double x) {
    return (x - 3.0) * (x - 3.0) - 1.0;
}

double d_f(double x) {
    return 2 * x - 6;
}


// Newton-Raphson

std::pair<uint32_t, double> newton_raphson_root_find(const std::function<double(double)> &f, double x_0,
                                                     double dx_0 = 0.5, double eps = 10e-7) {
    uint32_t iterations = 0;
    double x_1 = f(x_0) / d_f(x_0);
    while (x_1 > eps) {
        x_1 = f(x_0) / d_f(x_0);
        x_0 -= x_1;
        iterations++;
        // std::cout << "n =  " << iterations << "\n";
        // std::cout << "Value = " << x_0 << "\n" << std::endl;
    }
    return {iterations, x_0};
}

// Gradient Descent
// Used the R code from https://brian-ling.github.io/root-finding-and-optimization.html as a starting point
std::pair<uint32_t, double> gradient_descent_root_find(const std::function<double(double)> &f, double x_0,
                                                       double gamma = 0.01, double eps = 1e-7) {
    uint32_t iterations = 0;
    double old_x = x_0;
    double new_x = x_0 + 2 * eps;

    while (std::abs(new_x - old_x) > eps) {
        old_x = new_x;
        new_x -= gamma * f(new_x); // update
        iterations++;
        // std::cout << "n = " << iterations << "\n";
        // std::cout << "Value = " << old_x << "\n" << std::endl;
    }
    return {iterations, new_x};
}

void test_newton_raphson_root_find(const std::function<double(double)> &f, double x0) {
    auto [iterations, x_hat] = newton_raphson_root_find(f, x0);
    std::cout << "NR root f, x0=" << x0 << "-> it=" << iterations << " x=" << x_hat << " f(x)=" <<
            f(x_hat) << std::endl;
}

void test_gradient_descent_root_find(const std::function<double(double)> &f, double x0) {
    auto [iterations, x_hat] = gradient_descent_root_find(f, x0);
    std::cout << "GD root f, x0=" << x0 << "-> it=" << iterations << " x=" << x_hat << " f(x)=" <<
            f(x_hat) << std::endl;
}

void test_root_find() {
    std::cout << "Root finding tests" << std::endl;
    auto f1 = [](double x) {
        return (x - 3.0) * (x - 3.0) - 1.0;
    };
    test_newton_raphson_root_find(f1, 10);
    test_gradient_descent_root_find(f1, 10);
}

Eigen::MatrixXd ur3e_space_jacobian(const Eigen::VectorXd &current_joint_positions) {
    auto [m, space_screws] = ur3e_space_chain();
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(6, 6);
    Eigen::MatrixXd e_i = Eigen::MatrixXd::Identity(4, 4);
    std::string sep = "\n----------------------------------------------------------\n";

    for (uint8_t i_2 = 0; i_2 < space_screws.size(); ++i_2) {
        for (int i = 0; i < i_2; ++i) {
            e_i = matrix_exponential_transformation(space_screws[i].head(3), space_screws[i].tail(3),
                                                    current_joint_positions(i));
        }
        jacobian.col(i_2) = adjoint_matrix(e_i) * space_screws[i_2];
        // std::cout << sep << jacobian;
    }

    return jacobian;
}

Eigen::MatrixXd ur3e_body_jacobian(const Eigen::VectorXd &current_joint_positions) {
    auto [m, body_screws] = ur3e_body_chain(current_joint_positions);
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(6, 6);
    Eigen::MatrixXd e_i = Eigen::MatrixXd::Identity(4, 4);
    std::string sep = "\n----------------------------------------------------------\n";

    for (uint8_t i_2 = 0; i_2 < body_screws.size(); ++i_2) {
        for (int i = 0; i < i_2; ++i) {
            e_i = matrix_exponential_transformation(body_screws[i].head(3), body_screws[i].tail(3),
                                                    current_joint_positions(i));
        }
        jacobian.col(i_2) = adjoint_matrix(e_i) * body_screws[i_2];
        // std::cout << sep << jacobian;
    }

    return jacobian;
}


void ur3e_test_jacobian(const Eigen::VectorXd &joint_positions) {
    Eigen::Matrix4d tsb = ur3e_body_fk(joint_positions);
    auto [m, space_screws] = ur3e_space_chain();
    Eigen::MatrixXd jb = ur3e_body_jacobian(joint_positions);
    Eigen::MatrixXd js = ur3e_space_jacobian(joint_positions);
    Eigen::MatrixXd ad_tsb = adjoint_matrix(tsb);
    Eigen::MatrixXd ad_tbs = adjoint_matrix(tsb.inverse());
    std::cout << "Jb: " << std::endl << jb << std::endl << "Ad_tbs*Js:" << std::endl << ad_tbs * js <<
            std::endl << std::endl;
    std::cout << "Js: " << std::endl << js << std::endl << "Ad_tsb*Jb:" << std::endl << ad_tsb * jb <<
            std::endl << std::endl;
    std::cout << "d Jb: " << std::endl << jb - ad_tbs * js << std::endl << std::endl;
    std::cout << "d Js: " << std::endl << js - ad_tsb * jb << std::endl << std::endl;
}

void ur3e_test_jacobian() {
    std::cout << "Jacobian matrix tests" << std::endl;
    ur3e_test_jacobian(std_vector_to_eigen(std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
    ur3e_test_jacobian(std_vector_to_eigen(std::vector<double>{
        deg_to_rad(45.0), deg_to_rad(-20.0), deg_to_rad(10.0), deg_to_rad(2.5), deg_to_rad(30.0), deg_to_rad(-50.0)
    }));
}

// This is very messy and I don't think this works as properly as it should. Needs a remake.
std::pair<size_t, Eigen::VectorXd> ur3e_ik_body(const Eigen::Matrix4d &t_sd, const Eigen::VectorXd
                                                &current_joint_positions, double gamma = 1e-1, double v_e = 4e-2,
                                                double w_e = 4e-2) {
    uint32_t iterations = 0;
    Eigen::VectorXd q = current_joint_positions;
    const uint32_t max_iterations = 1000;
    Eigen::Matrix4d t_sb = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d t_bs;

    // To find the V_b
    Eigen::VectorXd V_b(6);
    Eigen::Matrix4d t_sd_inverse = t_sd.inverse();
    Eigen::Matrix3d R_inverse = t_sd_inverse.topLeftCorner(3, 3);
    Eigen::Vector3d p_inverse = t_sd_inverse.block<3, 1>(0, 3);

    Eigen::AngleAxisd angle_axis(R_inverse);
    Eigen::Vector3d w = angle_axis.axis() * angle_axis.angle();
    Eigen::Vector3d v = -R_inverse * p_inverse;

    V_b.head(3) = w;
    V_b.tail(3) = v;


    // Newton-Raphson
    while ((V_b.tail(3).norm() > v_e || V_b.head(3).norm() > w_e) && iterations < max_iterations) {
        q += gamma * V_b; // update
        t_bs = t_sb.inverse() * t_sd;
        V_b.head(3) = t_bs.block<3, 1>(0, 3);
        V_b.tail(3) = Eigen::Vector3d(t_bs(2, 1), t_bs(0, 2), t_bs(1, 0));
        iterations++;
    }
    return {iterations, q};
}


void ur3e_ik_test_pose(const Eigen::Vector3d &pos, const Eigen::Vector3d &zyx, const Eigen::VectorXd &j0) {
    std::cout << "Test from pose" << std::endl;
    Eigen::Matrix4d t_sd = transformation_matrix(rotation_matrix_from_euler_zyx(zyx), pos);
    auto [iterations, j_ik] = ur3e_ik_body(t_sd, j0);
    Eigen::Matrix4d t_ik = ur3e_body_fk(j_ik);
    print_pose("IK pose", t_ik);
    print_pose("Desired pose", t_sd);
    std::cout << "Converged after " << iterations << " iterations" << std::endl;
    std::cout << "J_0: " << j0.transpose() << std::endl;
    std::cout << "J_ik: " << j_ik.transpose() << std::endl << std::endl;
}

void ur3e_ik_test_configuration(const Eigen::VectorXd &joint_positions, const Eigen::VectorXd &j0) {
    std::cout << "Test from configuration" << std::endl;
    Eigen::Matrix4d t_sd = ur3e_space_fk(joint_positions);
    auto [iterations, j_ik] = ur3e_ik_body(t_sd, j0);
    Eigen::Matrix4d t_ik = ur3e_body_fk(j_ik);
    print_pose("IK pose", t_ik);
    print_pose("Desired pose", t_sd);
    std::cout << "Converged after " << iterations << " iterations" << std::endl;
    std::cout << "J_0: " << j0.transpose() << std::endl;
    std::cout << "J_d: " << joint_positions.transpose() << std::endl;
    std::cout << "J_ik: " << j_ik.transpose() << std::endl << std::endl;
}

void ur3e_ik_test() {
    Eigen::VectorXd j_t0 = std_vector_to_eigen(std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    Eigen::VectorXd j_t1 = std_vector_to_eigen(std::vector<double>{0.0, 0.0, -89.0, 0.0, 0.0, 0.0});
    ur3e_ik_test_pose(Eigen::Vector3d{0.3289, 0.22315, 0.36505}, Eigen::Vector3d{0.0, 90.0, -90.0}, j_t0);
    ur3e_ik_test_pose(Eigen::Vector3d{0.3289, 0.22315, 0.36505}, Eigen::Vector3d{0.0, 90.0, -90.0}, j_t1);
    Eigen::VectorXd j_t2 = std_vector_to_eigen(std::vector<double>{50.0, -30.0, 20, 0.0, -30.0, 50.0});
    Eigen::VectorXd j_d1 = std_vector_to_eigen(std::vector<double>{45.0, -20.0, 10.0, 2.5, 30.0, -50.0});
    ur3e_ik_test_configuration(j_d1, j_t0);
    ur3e_ik_test_configuration(j_d1, j_t2);
}

int main() {
    ur3e_test_fk();
    //newton_raphson_root_find(f, 10);
    //gradient_descent_root_find(f, 10);
    test_root_find();
    // Eigen::VectorXd current_joint_positions(6);
    // current_joint_positions << 0, 0, 0, 0, 0, 0;
    // ur3e_space_jacobian(current_joint_positions);
    // ur3e_body_jacobian(current_joint_positions);
    ur3e_test_jacobian();
    ur3e_ik_test();
    return 0;
}
