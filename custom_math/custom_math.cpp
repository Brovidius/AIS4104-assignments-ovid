//
// Created by Ovidius on 12/09/2024.
//

#include "custom_math.h"

#include <numeric>
#include <Eigen/Dense>
#ifndef FUNCTIONS_H
#define FUNCTIONS_H
#include <iostream>


// Tool functions

double zero_round(double value) {
    if (std::abs(value) < 1e-10) {
        return value = 0.0;
    }
    return value;
}

double deg_to_rad(double degree) {
    return degree * M_PI / 180; //0.01745329252;
}
double rad_to_deg(double rad) {
    return rad * 57.295779513;
}

double determinant(Eigen::Matrix3d matrix) {
    double det_num_1 = matrix(0,0) * (matrix(1,1) * matrix(2,2) - matrix(1,2) * matrix(2,1));
    double det_num_2 = matrix(0,1) * (matrix(1,0) * matrix(2,2) - matrix(1,2) * matrix(2,0));
    double det_num_3 = matrix(0,2) * (matrix(1,0) * matrix(2,1) - matrix(1,1) * matrix(2,0));
    double det = det_num_1 - det_num_2 + det_num_3;
    return det;
}

double trace(Eigen::MatrixXd matrix) {
    double trace = 0.0;
    for (int i = 0; i < matrix.rows(); ++i) {
        trace += matrix(i, i);
    }
    return trace;
}

Eigen::Vector3d xyz_to_zyx(const Eigen::Vector3d& xyz) {
    return Eigen::Vector3d(xyz(2), xyz(1), xyz(0));
}

Eigen::Vector3d xyz_to_yzx(const Eigen::Vector3d& xyz) {
    return Eigen::Vector3d(xyz(1), xyz(2), xyz(0));
}

Eigen::Vector3d yzx_to_xyz(const Eigen::Vector3d& yzx) {
    return Eigen::Vector3d(yzx(2), yzx(0), yzx(1));
}

Eigen::Vector3d yzx_to_zyx(const Eigen::Vector3d& yzx) {
    return Eigen::Vector3d(yzx(1), yzx(0), yzx(2));
}

Eigen::Vector3d zyx_to_xyz(const Eigen::Vector3d& zyx) {
    return Eigen::Vector3d(zyx(2), zyx(1), zyx(0));
}

Eigen::Vector3d zyx_to_yzx(const Eigen::Vector3d& zyx) {
    return Eigen::Vector3d(zyx(1), zyx(0), zyx(2));
}

Eigen::Matrix3d skew_symmetric(Eigen::Vector3d& v) {

    Eigen::Matrix3d skew_matrix;
    skew_matrix << 0, -v(2), v(1),
                   v(2), 0, -v(0),
                   -v(1), v(0), 0;
    return skew_matrix;
}

Eigen::Matrix3d rotation_matrix_from_frame_axes(const Eigen::Vector3d &x, const Eigen::Vector3d &y,
                                                const Eigen::Vector3d &z)
{
    Eigen::Matrix3d matrix;
    matrix.col(0) = x;
    matrix.col(1) = y;
    matrix.col(2) = z;
    return matrix;
}

Eigen::Matrix3d rotate_x(double degrees)
{
    double local_rad = degrees;
    Eigen::Matrix3d matrix;
    matrix << 1, 0, 0,
              0, std::cos(local_rad), -std::sin(local_rad),
              0, std::sin(local_rad), std::cos(local_rad);
    return matrix;
}

Eigen::Matrix3d rotate_y(double degrees)
{
    double local_rad = degrees;
    Eigen::Matrix3d matrix;
    matrix << std::cos(local_rad), 0, std::sin(local_rad),
              0, 1, 0,
              -std::sin(local_rad), 0, std::cos(local_rad);
    return matrix;
}

Eigen::Matrix3d rotate_z(double degrees)
{
    double local_rad = degrees;
    Eigen::Matrix3d matrix;
    matrix << std::cos(local_rad), -std::sin(local_rad), 0,
              std::sin(local_rad), std::cos(local_rad), 0,
              0, 0, 1;
    return matrix;
}

Eigen::Matrix3d rotation_matrix_from_axis_angle(const Eigen::Vector3d &axis, double degrees)
{
    double rad = deg_to_rad(degrees);
    Eigen::Matrix3d matrix;
    // I feel like there is (or should) be a simpler way of representing this, but don't think so :D
    matrix << //row 1
              std::cos(rad) + axis(0) * axis(0) * (1 - std::cos(rad)),
              axis(0) * axis(1) * (1 - std::cos(rad)) - axis(2)*std::sin(rad),
              axis(0)*axis(2)*(1 - std::cos(rad)) + axis(1)*std::sin(rad),
              //row 2
              axis(0)*axis(1)*(1-std::cos(rad)) + axis(2)*std::sin(rad),
              std::cos(rad) + axis(1)*axis(1)*(1-std::cos(rad)),
              axis(1)*axis(2)*(1-std::cos(rad)) - axis(0)*std::sin(rad),
              // row 3
              axis(0)*axis(2)*(1-std::cos(rad)) - axis(1)*std::sin(rad),
              axis(1)*axis(2)*(1-std::cos(rad)) + axis(0)*std::sin(rad),
              std::cos(rad) + axis(2)*axis(2) * (1 - std::cos(rad));
    return matrix;
}

Eigen::Matrix3d rotation_matrix_from_euler_zyx(const Eigen::Vector3d &e)
{
    Eigen::Vector3d e_xyz = zyx_to_xyz(e);
    Eigen::Matrix3d matrix_x = rotate_x(e_xyz(0));
    Eigen::Matrix3d matrix_y = rotate_y(e_xyz(1));
    Eigen::Matrix3d matrix_z = rotate_z(e_xyz(2));
    return matrix_z*matrix_y*matrix_x;
}

Eigen::Matrix3d rotation_matrix_from_euler_yzx(const Eigen::Vector3d &e)
{
    Eigen::Vector3d e_xyz = yzx_to_xyz(e);
    Eigen::Matrix3d matrix_x = rotate_x(e_xyz(0));
    Eigen::Matrix3d matrix_y = rotate_y(e_xyz(1));
    Eigen::Matrix3d matrix_z = rotate_z(e_xyz(2));
    return matrix_y*matrix_z*matrix_x;
}



Eigen::Matrix4d transformation_matrix(const Eigen::Matrix3d &r, const Eigen::Vector3d &p)
{
    Eigen::Matrix4d matrix;
    matrix = Eigen::Matrix4d::Identity();
    matrix.block<3,3>(0,0) = r;
    matrix.block<3,1>(0,3)  = p;
    return matrix;
}

Eigen::Vector3d euler_zyx_from_rotation_matrix(const Eigen::Matrix3d &r) {

    /*
        1 Tasks

        Task 1: Functions and algorithms

        T.1 a) Calculate an Euler ZYX vector from a rotation matrix.

        Implement an algorithm to calculate the Euler ZYX angles from a rotation matrix [2, ch. B.1.1].

        Use the following function signature:
        Eigen::Vector3d euler_zyx_from_rotation_matrix(const Eigen::Matrix3d &r)
    */

    // Version 2 - Braced initializer lists. My IDE (Clion) says this is better.

    if (r(2,0) == -1)
    {
        return {std::numbers::pi / 2, 0, atan2(r(0,1), r(1,1))};
    }

    if (r(2,0) == 1)
    {
        return { - std::numbers::pi / 2, 0, -atan2(r(0,1), r(1,1))};
    }

    return {atan2(-r(2,0), sqrt((pow(r(0,0), 2) + pow(r(1,0), 2)))),
              atan2(r(1, 0), r(0,0)),
              atan2(r(2,1), r(2,2))};


    /* // Version 1 - Without braced initializer list. Not sure if worse or better?
    double euler_z, euler_y, euler_x;

    if (r(2,0) == -1)
    {
        euler_z = std::numbers::pi / 2;
        euler_y = 0;
        euler_x = atan2(r(0,1), r(1,1));
    }

    else if (r(2,0) == 1)
    {
        euler_z = - std::numbers::pi / 2;
        euler_y = 0;
        euler_x = -atan2(r(0,1), r(1,1));
    }

    else
    {
        euler_z = atan2(-r(2,0), sqrt((pow(r(0,0), 2) + pow(r(1,0), 2))));
        euler_y = atan2(r(1, 0), r(0,0));
        euler_x = atan2(r(2,1), r(2,2));
    }

    return Eigen::Vector3d(euler_z, euler_y, euler_x);
    /*
    /*
    // Warning: sin(pi - theta) = sin(theta)
    double euler_z = -asin(r(2,0));

    // Valid for all theta except when cos(theta) = 0, i.e, theta = 90, 270 deg
    double euler_y = atan2(r(2,1)/cos(euler_z), r(2,2)/cos(euler_z));
    double euler_x = atan2(r(1,0)/cos(euler_z), r(0,0)/cos(euler_z));
    */

}


// Creating a twist from angular w and linear v velocity real number 3D vectors, returns a 6D vector [w, v]^T
Eigen::VectorXd twist(const Eigen::Vector3d &w, const Eigen::Vector3d &v) {

    Eigen::VectorXd twist(6);
    twist << w(0), w(1), w(2), v(0), v(1), v(2);
    return twist;
}

// Creating a screw axis 6D vector
Eigen::VectorXd screw_axis(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h) {
    Eigen::VectorXd screw(6);
    screw << q(0), q(1), q(2), s(0), s(1), s(2);
    return screw;
}

// adjoint representation, [AdT] ∈ R6× 6, of a homogeneous transformation,

    Eigen::MatrixXd adjoint_matrix(const Eigen::Matrix4d &tf)
{
    Eigen::MatrixXd matrix = Eigen::MatrixXd::Identity(6,6);
    matrix.block(0,0, 3, 3) = tf.block(0,0, 3, 3);
    matrix.block(3, 3, 3, 3) = tf.block(0,0, 3, 3);

    Eigen::Vector3d position = tf.block<3, 1>(0, 3);
    Eigen::Matrix3d skewed_position_vector = skew_symmetric(position);
    Eigen::Matrix3d matrix_product_pR = skewed_position_vector * tf.block(0, 0, 3, 3);

    matrix.block(3, 0, 3, 3) = matrix_product_pR;
    return matrix;
}

double cot(double x) {
    x = deg_to_rad(x);
    return 1 / (std::sin(x)/std::cos(x));
}

Eigen::Matrix3d matrix_exponential_rotational(const Eigen::Vector3d &w, double theta) {

    // Unit axis of rotation
    Eigen::Vector3d w_ = w;
    Eigen::Matrix3d skewed_w = skew_symmetric(w_);

    // Rotation angle about axis
    double l_theta = theta;

    // Closed form of the exponential, Rodrigue's formula for rotations
    Eigen::Matrix3d rot = Eigen::Matrix3d::Identity() + std::sin(l_theta) * skewed_w +
                              (1 - std::cos(l_theta)) * skewed_w * skewed_w;

    return rot;

}

// Calculates the matrix logarithm from a rotation matrix
std::pair<Eigen::Vector3d, double> matrix_logarithm(const Eigen::Matrix3d &r) {

    Eigen::Matrix3d R = r;
    Eigen::Vector3d rotation_axis;
    double theta;
    if (R == Eigen::Matrix3d::Identity()) {
        theta = 0;
        rotation_axis = Eigen::Vector3d(0, 0, 0);
    }

    if (R.trace() == -1) {
        theta = M_PI;
        rotation_axis = 1/(sqrt(2*(1 + R(2,2)))) *
                        Eigen::Vector3d(R(0,2), R(1, 2), 1 + (R(2,2)));

        if (rotation_axis == Eigen::Vector3d(0,0,0)) {
            rotation_axis = 1/(sqrt(2*(1 + R(1,1)))) *
                            Eigen::Vector3d(R(0,1), 1 + R(1, 1), (R(2,1)));

            if (rotation_axis == Eigen::Vector3d(0,0,0)) {
                rotation_axis = 1/(sqrt(2*(1 + R(0,0)))) *
                                Eigen::Vector3d(1 + R(0,0), R(1, 0), (R(2,1)));
            }
        }
    }
    else  {
        theta = std::acos((R.trace() - 1)/2);
        if (0 <= theta && theta <= M_PI) {
            Eigen::Matrix3d skewed_rotation_axis = (R - R.transpose())/(2 * std::sin(theta));
            rotation_axis << skewed_rotation_axis(1, 2),
                             skewed_rotation_axis(0, 2),
                             skewed_rotation_axis(0, 1);
        }
    }

    return std::make_pair(rotation_axis, theta);
}

// Matrix exponential for homogeneous transformation matrices
Eigen::Matrix4d matrix_exponential_transformation(const Eigen::Vector3d &w, const Eigen::Vector3d &v, double theta) {
    Eigen::Matrix3d rotation_matrix = matrix_exponential_rotational(w, theta);
    double l_theta = theta;
    Eigen::Vector3d m_w = w;
    Eigen::Matrix3d skewed_w = skew_symmetric(m_w);
    //G(theta)
    Eigen::Matrix3d G =  Eigen::Matrix3d::Identity() * l_theta + (1 - std::cos(l_theta)) * skewed_w + (l_theta - std::sin(l_theta)) * skewed_w * skewed_w;
    Eigen::Matrix4d matrix = transformation_matrix(rotation_matrix, G * v);
    return matrix;
}

Eigen::VectorXd std_vector_to_eigen(const std::vector<double> &v) {
    Eigen::VectorXd r(v.size());
    // Itterators

    //For loop
    for(int i = 0; i < v.size(); i++) {
        r[1] = v[i];
    }

    return r;
}

bool is_average_below_eps(const std::vector<double> &values, double eps, uint8_t n_values) {
    if(values.size() < n_values)
        return false;
    double sum = std::accumulate(values.end() - n_values, values.end(), 0.0);
    return std::abs(sum / n_values) < eps;
}

#endif