#include <iostream>
#include <Eigen/Dense>


// Tool functions
double deg_to_rad(double degree) {
    return degree * 0.01745329252;
}
double rad_to_deg(double rad) {
    return rad * 57.295779513;
}

/* double determinant(Eigen::Matrix3d matrix) {
    double det_num_1 = matrix(0,0) * (matrix(1,1) * matrix(2,2) - matrix(1,2) * matrix(2,1));
    double det_num_2 = matrix(0,1) * (matrix(1,0) * matrix(2,2) - matrix(1,2) * matrix(2,0));
    double det_num_3 = matrix(0,2) * (matrix(1,0) * matrix(2,1) - matrix(1,1) * matrix(2,0));

    double det = det_num_1 - det_num_2 + det_num_3;
    return det;
}
*/

// Task 2.1

// a)
Eigen::Matrix3d skew_symmetric(Eigen::Vector3d& v) {

    Eigen::Matrix3d skew_matrix;
    skew_matrix << 0, -v(2), v(1),
                   v(2), 0, -v(0),
                   -v(1), v(0), 0;
    return skew_matrix;
}

// b)
void skew_symmetric_test()
{
    Eigen::Vector3d test_vector = {0.5, 0.5, 0.707107};
    Eigen::Matrix3d skew_matrix = skew_symmetric(test_vector);
    std::cout << "Skew-symmetric matrix: " << std::endl;
    std::cout << skew_matrix << std::endl;
    std::cout << "Skew-symmetric matrix transposition: " << std::endl;
    std::cout << -skew_matrix.transpose() << std::endl;
    // std::cout << std::endl << "Determinant: " << std::endl << determinant(skew_matrix);
}

// Task 2.2

// a)
Eigen::Matrix3d rotation_matrix_from_frame_axes(const Eigen::Vector3d &x, const Eigen::Vector3d &y,
                                                const Eigen::Vector3d &z)
{
    Eigen::Matrix3d matrix;
    matrix.col(0) = x;
    matrix.col(1) = y;
    matrix.col(2) = z;
    return matrix;
}

// b)
Eigen::Matrix3d rotate_x(double degrees)
{
    double local_rad = deg_to_rad(degrees);
    Eigen::Matrix3d matrix;
    matrix << 1, 0, 0,
              0, std::cos(local_rad), -std::sin(local_rad),
              0, std::sin(local_rad), std::cos(local_rad);
    return matrix;
}

// c)
Eigen::Matrix3d rotate_y(double degrees)
{
    double local_rad = deg_to_rad(degrees);
    Eigen::Matrix3d matrix;
    matrix << std::cos(local_rad), 0, std::sin(local_rad),
              0, 1, 0,
              -std::sin(local_rad), 0, std::cos(local_rad);
    return matrix;
}

// d)
Eigen::Matrix3d rotate_z(double degrees)
{
    double local_rad = deg_to_rad(degrees);
    Eigen::Matrix3d matrix;
    matrix << std::cos(local_rad), -std::sin(local_rad), 0,
              std::sin(local_rad), std::cos(local_rad), 0,
              0, 0, 1;
    return matrix;
}

// e)
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

// f)
Eigen::Matrix3d rotation_matrix_from_euler_zyx(const Eigen::Vector3d &e)
{
    Eigen::Matrix3d matrix_x = rotate_x(e(2));
    Eigen::Matrix3d matrix_y = rotate_y(e(1));
    Eigen::Matrix3d matrix_z = rotate_z(e(0));
    return matrix_z*matrix_y*matrix_x;
}

// g)
void rotation_matrix_test()
{
    Eigen::Matrix3d rot =
    rotation_matrix_from_euler_zyx(Eigen::Vector3d{45.0,-45.0, 90.0});
    Eigen::Matrix3d rot_aa =
    rotation_matrix_from_axis_angle(Eigen::Vector3d{0.8164966, 0.0, 0.5773503}, 120.0);
    Eigen::Matrix3d rot_fa =
    rotation_matrix_from_frame_axes(Eigen::Vector3d{0.5, 0.5, 0.707107},
    Eigen::Vector3d{-0.5,-0.5, 0.707107},
    Eigen::Vector3d{0.707107,-0.707107, 0.0});
    std::cout << "Rotation matrix from Euler: " << std::endl;
    std::cout << rot << std::endl << std::endl;
    std::cout << "Rotation matrix from axis-angle pair: " << std::endl;
    std::cout << rot_aa << std::endl << std::endl;
    std::cout << "Rotation matrix from frame axes: " << std::endl;
    std::cout << rot_fa << std::endl << std::endl;
}

// Task 2.3

// a)
Eigen::Matrix4d transformation_matrix(const Eigen::Matrix3d &r, const Eigen::Vector3d &p)
{
    Eigen::Matrix4d matrix;
    matrix = Eigen::Matrix4d::Identity();
    matrix.block<3,3>(0,0) = r;
    matrix.block<3,1>(0,3) = p;
    return matrix;
}

// b)
void transformation_matrix_test()
{
    Eigen::Matrix3d r = rotation_matrix_from_euler_zyx(Eigen::Vector3d{45,-45.0, 90.0});
    Eigen::Vector3d v{1.0,-2.0, 3.0};
    std::cout << "transformation_matrix: " << std::endl;
    std::cout << transformation_matrix(r, v) << std::endl;
}

// c)
void transform_vector() {

    // I just discovered about "auto". I've read about it and the C++ community
    // have an opinion that it's nice if it's an obvious datatype, but also that is long.
    // In my opinion, it makes sense to use 'auto' when handling Eigen lib datatypes due to its length.
    // However, Eigen documentation specifically calls out 'auto' as bad practice for its lib.


    // We wish to find vector V_a expressed with {w} coordinates: V_w:
    // V_w = R_wa * V_a

    // The rotation matrix and position vector
    Eigen::Matrix3d r = rotation_matrix_from_euler_zyx(Eigen::Vector3d(60, 45, 0));
    Eigen::Vector3d p = Eigen::Vector3d(0, 0, 10);

    // Making the transformation matrix R_wa
    Eigen::Matrix4d R_wa = transformation_matrix(r, p);

    // Declaring our vector expressed in {a}, V_a
    Eigen::Vector4d V_a = Eigen::Vector4d(2.5, 3.0, -10.0, 1.0);

    // Expressing V_w
    Eigen::Vector4d V_w = R_wa * V_a;

    std::cout << V_w << std::endl;
}


int main()
{
    skew_symmetric_test();
    rotation_matrix_test();
    transformation_matrix_test();
    transform_vector();
    return 0;
}
