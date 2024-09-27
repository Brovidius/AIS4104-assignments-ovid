//
// Created by Ovidius on 12/09/2024.
//

#include <Eigen/Dense>

#ifndef CUSTOM_MATH_H
#define CUSTOM_MATH_H

double zero_round(double value);
double deg_to_rad(double degree);
double rad_to_deg(double rad);
double determinant(Eigen::Matrix3d matrix);
double trace(Eigen::MatrixXd matrix);
Eigen::Vector3d xyz_to_zyx(const Eigen::Vector3d& xyz);
Eigen::Vector3d xyz_to_yzx(const Eigen::Vector3d& xyz);
Eigen::Vector3d zyx_to_xyz(const Eigen::Vector3d& zyx);
Eigen::Vector3d zyx_to_yzx(const Eigen::Vector3d& zyx);
Eigen::Vector3d yzx_to_xyz(const Eigen::Vector3d& yzx);
Eigen::Vector3d yzx_to_zyx(const Eigen::Vector3d& yzx);

Eigen::Matrix3d skew_symmetric(Eigen::Vector3d& v);
Eigen::Matrix3d rotation_matrix_from_frame_axes(const Eigen::Vector3d &x, const Eigen::Vector3d &y,
                                                const Eigen::Vector3d &z);
Eigen::Matrix3d rotate_x(double degrees);
Eigen::Matrix3d rotate_y(double degrees);
Eigen::Matrix3d rotate_z(double degrees);
Eigen::Matrix3d rotation_matrix_from_axis_angle(const Eigen::Vector3d &axis, double degrees);
Eigen::Matrix3d rotation_matrix_from_euler_zyx(const Eigen::Vector3d &e);
Eigen::Matrix4d transformation_matrix(const Eigen::Matrix3d &r, const Eigen::Vector3d &p);
Eigen::Vector3d euler_zyx_from_rotation_matrix(const Eigen::Matrix3d &r);
Eigen::Matrix3d rotation_matrix_from_euler_yzx(const Eigen::Vector3d &e);
Eigen::VectorXd twist(const Eigen::Vector3d &w, const Eigen::Vector3d &v);
Eigen::VectorXd screw_axis(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h);
Eigen::Matrix4d screw_matrix(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h);
Eigen::MatrixXd adjoint_matrix(const Eigen::Matrix4d &tf);
Eigen::Matrix4d matrix_exponential_transformation(const Eigen::Vector3d &w, const Eigen::Vector3d &v, double theta);
std::pair<Eigen::Vector3d, double> matrix_logarithm(const Eigen::Matrix3d &r);
Eigen::Matrix3d matrix_exponential_rotational(const Eigen::Vector3d &w, double theta);
bool is_average_below_eps(const std::vector<double> &values, double eps = 10e-7, uint8_t n_values = 5u);
Eigen::VectorXd std_vector_to_eigen(const std::vector<double> &v);
double cot(double x);

#endif //CUSTOM_MATH_H
