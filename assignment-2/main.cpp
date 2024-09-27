#include <iostream>
#include <Eigen/Dense>


// subfolder-2/main.cpp
#include "../custom_math/custom_math.h"


// Task 2: Wrenches
// T.2 a)

void robot_wrench() {

     // Sensor seen by the world
     Eigen::Vector3d f_w = Eigen::Vector3d(-30, 0, 0);

     // World seen by the sensor
     Eigen::Vector3d m_s = Eigen::Vector3d(0, 0, 2);

     // The sensors orientation e_ws (sensor seen from the world) is in:
     Eigen::Matrix3d e_ws = rotation_matrix_from_euler_yzx(Eigen::Vector3d(60, -60, 0));

     Eigen::Vector3d f_s = e_ws.transpose() * f_w;
     Eigen::Vector3d m_w = e_ws * m_s;

}

// T.2 b)

void example_328() {
     double apple_mass = 0.1;
     double gravity = 10;
     double hand_mass = 0.5;
     // {f} is a frame at the sensor
     // {h} is center of hand
     // {a} is in apple

     // Gravitational wrench on the hand in {h} in N
     Eigen::VectorXd F_h(6);
     F_h << 0, 0, 0, 0, -5, 0;
     // Gravitational wrench on the apple in {a} in N
     Eigen::VectorXd F_a(6);
     F_a << 0, 0, 0, 0, 0, 1;

     // The transformation matrices T_hf and T_af

     Eigen::Matrix4d T_hf;
     T_hf << 1, 0, 0, -0.1,
             0, 1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;


     Eigen::Matrix4d T_af;
     T_af << 1, 0, 0, -0.25,
             0, 0, 1, 0,
             0, -1, 0, 0,
             0, 0, 0, 1;

    // Wrench measured by the six-axis force-torque sensor is

    Eigen::MatrixXd F_f = adjoint_matrix(T_hf).transpose() * F_h + adjoint_matrix(T_af).transpose() * F_a;
    std::cout << F_f << std::endl;
}

// Task 3: Matrix exponentials and logarithms

// Creates a rotation matrix from exponential coordinates


// Calculates the matrix logarithm from homogeneous transformation matrices
std::pair<Eigen::VectorXd, double> matrix_logarithm(const Eigen::Matrix4d &t){
    Eigen::Matrix3d R = t.block<3,3>(0,0);
    return matrix_logarithm(R);
}

/*
// Matrix logarithm transformationator
igen::Matrix4d matrix_logarithm_transformation(const Eigen::Vector3d &w, const Eigen::Vector3d &v, double theta) {
   const double theta_copy = deg_to_rad(theta);
   Eigen::Vector3d w_copy = w;
   Eigen::Matrix3d skewed_w = skew_symmetric(w_copy);
   Eigen::Matrix4d matrix;
   matrix.block<3,3>(0,0) = skewed_w * theta_copy;
   matrix.block<3,1>(3,0) = v * theta;

   if (matrix.block<3,3>(0,0).isApprox(Eigen::Matrix3d::Identity())) {
       w_copy = 0;

   }
*/


// Task 4: Forward kinematics: 3R planar open chain
// T.4 a)
// Euler ZYX angles and linear position of a homogenous transformation matrix

void print_pose(const std::string &label, const Eigen::Matrix4d &tf) {

    Eigen::Matrix3d rotation = tf.block<3,3>(0,0);
    Eigen::Vector3d zyx_rotation = euler_zyx_from_rotation_matrix(rotation);
    Eigen::Vector3d translation = tf.block<3,1>(0,3);

    // I didn't understand what was meant with "label of pose". Going with some descriptions instead.

    // Actually, I don't think this is correct at all :|
    /* std::string label = "Euler Rotation Vector ZYX\n" +
        std::to_string(zyx_rotation(0)) + " " + std::to_string(zyx_rotation(1)) + " " + std::to_string(zyx_rotation(2)) + "\n" +
        "Translation vector:\n" +
        std::to_string(translation(0)) + " " + std::to_string(translation(1)) + " " + std::to_string(translation(2)) + "\n" +
        "Joint {0} is rotated: " + std::to_string(zero_round(rad_to_deg(rotation(0)))) + " degrees on Z.\n" +
        "Joint {1} is rotated: " + std::to_string(zero_round(rad_to_deg(rotation(1)))) + " degrees on Y.\n" +
        "Joint {2} is rotated: " + std::to_string(zero_round(rad_to_deg(rotation(2)))) + " degrees on X.\n" +
        "X: " + std::to_string(zero_round(translation(0))) + "\n" +
        "Y: " + std::to_string(zero_round(translation(1))) + "\n" +
        "Z: " + std::to_string(zero_round(translation(2))) + "\n";
    */
    std::cout << label << ":" << '\n';
    std::cout << "Euler ZYX \n" << zyx_rotation << '\n' << '\n';
    std::cout << "Position \n" << translation << '\n' << std::endl;
}

//  T.4 b) Calculate forward kinematics using transformation matrices.
/*
    mplement a function that calculates the forward kinematics (FK) of the 3R planar open chain in figure 4.1 from
    Modern Robotics [2], using homogeneous transformation matrices only.
*/


Eigen::Matrix4d planar_3r_fk_transform(const std::vector<double> &joint_positions) {

    const double L_1 = 10;
    const double L_2 = 10;
    const double L_3 = 10;

    Eigen::Matrix3d R_01 = rotate_z(joint_positions[0]);
    Eigen::Matrix4d T_01 = transformation_matrix(R_01, Eigen::Vector3d(0, 0, 0));

    Eigen::Matrix3d R_12 = rotate_z(joint_positions[1]);
    Eigen::Matrix4d T_12 = transformation_matrix(R_12, Eigen::Vector3d(L_1, 0, 0));

    Eigen::Matrix3d R_23 = rotate_z(joint_positions[2]);
    Eigen::Matrix4d T_23 = transformation_matrix(R_23, Eigen::Vector3d(L_2, 0, 0));

    Eigen::Matrix4d T_34 = transformation_matrix(Eigen::Matrix3d::Identity(), Eigen::Vector3d(L_3, 0, 0));

    // Eigen::Matrix4d T_04 = T_01 * T_12 * T_23 * T_34;


    // Eigen::Vector3d zyx_rotation = euler_zyx_from_rotation_matrix(T_04.block<3,3>(0,0));
    // Eigen::Vector3d translation = T_04.block<3,1>(0,3);
    // std::cout << "Configuration: " << "[" << joint_positions[0] << " " <<
    //                                          joint_positions[1] << " " <<
    //                                          joint_positions[2]  << "]\370" << std::endl;
    // std::cout << "Euler ZYX: " << '\n' << zyx_rotation << '\n' <<
    //              "Position XYZ: " << '\n' << translation << std::endl;
    //
//
    return T_01 * T_12 * T_23 * T_34;
}


//  T.4 c) Calculate forward kinematics using the product of exponentials.

Eigen::Matrix4d planar_3r_fk_screw(const std::vector<double> &joint_positions) {

    const double L_1 = 10;
    const double L_2 = 10;
    const double L_3 = 10;

    Eigen::Vector3d angles;
    angles << joint_positions[0], joint_positions[1], joint_positions[2];

    // w_3 and v_3 as from the book's S_3
    Eigen::Vector3d w_1 = Eigen::Vector3d(0, 0, 1);
    Eigen::Vector3d v_1 = Eigen::Vector3d(0, 0, 0);
    Eigen::Matrix4d e_S1 = matrix_exponential_transformation(w_1, v_1, joint_positions[0]);

    Eigen::Vector3d w_2 = Eigen::Vector3d(0, 0, 1);
    Eigen::Vector3d v_2 = Eigen::Vector3d(0, -(L_1), 0);
    Eigen::Matrix4d e_S2 = matrix_exponential_transformation(w_2, v_2, joint_positions[1]);

    Eigen::Vector3d w_3 = Eigen::Vector3d(0, 0, 1);
    Eigen::Vector3d v_3 = Eigen::Vector3d(0, -(L_1 + L_2), 0);
    Eigen::Matrix4d e_S3 = matrix_exponential_transformation(w_3, v_3, joint_positions[2]);

    Eigen::Matrix4d M = transformation_matrix(Eigen::Matrix3d::Identity(), Eigen::Vector3d(L_1 + L_2 + L_3, 0, 0));

    return  e_S1 * e_S2 * e_S3 * M;
}

// T.5 a)

Eigen::Matrix4d ur3e_fk_screw(const std::vector<double> &joint_positions) {

    double l1 = 0.24355;
    double l2 = 0.2132;
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

Eigen::Matrix4d ur3e_fk_transform(const std::vector<double> &joint_positions) {

    double l1 = 0.24355;
    double l2 = 0.2132;
    double w1 = 0.13105;
    double w2 = 0.0921;
    double h1 = 0.15185;
    double h2 = 0.08535;

    // 1
    Eigen::Matrix3d R_12 = rotate_z(joint_positions[0]);
    Eigen::Vector3d p_12 = Eigen::Vector3d(0, 0, h1);
    Eigen::Matrix4d T_12 = transformation_matrix(R_12, p_12);

    // 2
    Eigen::Matrix3d R_23 = rotate_y(joint_positions[1]);
    Eigen::Vector3d p_23 = Eigen::Vector3d(0, w1, 0);
    Eigen::Matrix4d T_23 = transformation_matrix(R_23, p_23);

    // 3
    Eigen::Matrix3d R_34 = rotate_y(joint_positions[2]);
    Eigen::Vector3d p_34 = Eigen::Vector3d(l1, 0, 0);
    Eigen::Matrix4d T_34 = transformation_matrix(R_34, p_34);

    // 4
    Eigen::Matrix3d R_45 = rotate_y(joint_positions[3]);
    Eigen::Vector3d p_45 = Eigen::Vector3d(l2, 0, 0);
    Eigen::Matrix4d T_45 = transformation_matrix(R_45, p_45);

    // 5
    Eigen::Matrix3d R_56 = rotate_z(joint_positions[4]);
    Eigen::Vector3d p_56 = Eigen::Vector3d(0, 0, -h2);
    Eigen::Matrix4d T_56 = transformation_matrix(R_56, p_56);

    // 6
    Eigen::Matrix3d R_67 = Eigen::Matrix3d::Identity();
    Eigen::Vector3d p_67 = Eigen::Vector3d(0, w2, 0);
    Eigen::Matrix4d T_67 = transformation_matrix(R_67, p_67);

    return T_12 * T_23 * T_34 * T_45 * T_56 * T_67;
}

int main() {

    // Eigen::Matrix3d rotation = rotation_matrix_from_euler_yzx(Eigen::Vector3d(90, 180, 90));
    // Eigen::Vector3d translation = Eigen::Vector3d(0, 0, 0);
    // Eigen::Matrix4d T = transformation_matrix(rotation, translation);
    // print_pose("hello lol", T);
/*
    std::cout << '\n' << "~--~ Using Tf ~--~" << std::endl;
    planar_3r_fk_transform({0, 0, 0});
    std::cout << '\n'<< "~--~ Using PoE ~--~" << std::endl;
    planar_3r_fk_screw({0, 0, 0});

    std::cout << '\n' << "~--~ Using Tf ~--~" << std::endl;
    planar_3r_fk_transform({90, 0, 0});
    std::cout << '\n'<< "~--~ Using PoE ~--~" << std::endl;
    planar_3r_fk_screw({90, 0, 0});

    std::cout << '\n' << "~--~ Using Tf ~--~" << std::endl;
    planar_3r_fk_transform({0, 90, 0});
    std::cout << '\n'<< "~--~ Using PoE ~--~" << std::endl;
    planar_3r_fk_screw({0, 90, 0});

    std::cout << '\n' << "~--~ Using Tf ~--~" << std::endl;
    planar_3r_fk_transform({0, 0, 90});
    std::cout << '\n'<< "~--~ Using PoE ~--~" << std::endl;
    planar_3r_fk_screw({0, 0, 90});

    std::cout << '\n' << "~--~ Using Tf ~--~" << std::endl;
    planar_3r_fk_transform({10, -15, 2.75});
    std::cout << '\n'<< "~--~ Using PoE ~--~" << std::endl;
    planar_3r_fk_screw({10, -15, 2.75});
*/

    // T.4 b and c)
    std::cout << "T.4 b) and c)";
    std::vector<double> joint_position = {0, 0, 0};
    std::cout << "[" <<joint_position[0] << " " << joint_position[1] << " " << joint_position[2] <<"]\370 \n";
    print_pose("~--~ transform ~--~", planar_3r_fk_transform(joint_position));
    print_pose("~--~ screw ~--~", planar_3r_fk_screw(joint_position));

    joint_position = {90, 0, 0};
    std::cout << "[" <<joint_position[0] << " " << joint_position[1] << " " << joint_position[2] <<"]\370 \n";
    print_pose("~--~ transform ~--~", planar_3r_fk_transform(joint_position));
    print_pose("~--~ screw ~--~", planar_3r_fk_screw(joint_position));

    joint_position = {0, 90, 0};
    std::cout << "[" <<joint_position[0] << " " << joint_position[1] << " " << joint_position[2] <<"]\370 \n";
    print_pose("~--~ transform ~--~", planar_3r_fk_transform(joint_position));
    print_pose("~--~ screw ~--~", planar_3r_fk_screw(joint_position));

    joint_position = {0, 0, 90};
    std::cout << "[" <<joint_position[0] << " " << joint_position[1] << " " << joint_position[2] <<"]\370 \n";
    print_pose("~--~ transform ~--~", planar_3r_fk_transform(joint_position));
    print_pose("~--~ screw ~--~", planar_3r_fk_screw(joint_position));

    joint_position = {10, -15, 2.75};
    std::cout << "[" <<joint_position[0] << " " << joint_position[1] << " " << joint_position[2] <<"]\370 \n";
    print_pose("~--~ transform ~--~", planar_3r_fk_transform(joint_position));
    print_pose("~--~ screw ~--~", planar_3r_fk_screw(joint_position));

    // T.5 a) and b)
    std::cout << "T.5 a, b)";
    std::cout << "\n Screw \n";
    std::vector<double> joint_positions = {0, 0, 0, deg_to_rad(-90), 0, 0};
    print_pose("Pose j_1", ur3e_fk_screw(joint_positions));
    std::cout << "\n Transform \n";
    print_pose("Pose j_1", ur3e_fk_transform(joint_positions));

    joint_positions = {0, deg_to_rad(-180), 0, 0, 0, 0};
    std::cout << "\n Screw \n";
    print_pose("Pose j_2", ur3e_fk_screw(joint_positions));
    std::cout << "\n Transform \n";
    print_pose("Pose j_2", ur3e_fk_transform(joint_positions));

    joint_positions = {0, deg_to_rad(-90), 0, 0, 0, 0};
    std::cout << "\n Screw \n";
    print_pose("Pose j_3", ur3e_fk_screw(joint_positions));
    std::cout << "\n Transform \n";
    print_pose("Pose j_3", ur3e_fk_transform(joint_positions));

    return 0;
}