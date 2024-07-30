#include "Utils.hpp"
#include <iostream>
int main (int argc, char* argv[]) {
    Eigen::Quaterniond q(0.5, 0.5, 0.5, 0.5);
    Eigen::Vector3d euler = Utils::quat_to_euler(q);
    std::cout << "Euler angles: \n" << euler << std::endl;
    std::cout << "--------------------------------" << std::endl;

    Eigen::Vector3d v(1, 2, 3);
    Eigen::Matrix3d skew = Utils::skew(v);
    std::cout << "Skew matrix: \n" << skew << std::endl;
    std::cout << "--------------------------------" << std::endl;

    
    //create random matrix and compute its nullspace
    Eigen::MatrixXd A = Eigen::MatrixXd::Random(3, 15);
    // std::cout << "Random matrix A: \n" << A << std::endl;
    Eigen::MatrixXd N = Utils::compute_nullspace_QR(A);
    std::cout << "Nullspace matrix of A ("<< N.rows()<< "x"<< N.cols() << std::endl;
    //create random vector and check if it is in the nullspace of A
    Eigen::VectorXd x = Eigen::VectorXd::Random(N.cols());
    // std::cout << "Random vector x: \n" << x << std::endl;
    std::cout << "A*N: \n" << A*N << std::endl;
    std::cout << "A*N*x: \n" << A*N*x << std::endl;
    std::cout << "--------------------------------" << std::endl;


    Eigen::MatrixXd A2 = Eigen::MatrixXd::Random(5, 15);
    Eigen::MatrixXd N2 = Utils::compute_nullspace_QR(A2);
    Eigen::VectorXd x2 = Eigen::VectorXd::Random(N2.cols());
    std::cout << "A2*N2*x2: \n" << A2*N2*x2 << std::endl;
    std::cout << "--------------------------------" << std::endl;

    Eigen::MatrixXd Z2 = N*Utils::compute_nullspace_QR(A2*N);
    std::cout << "A1*Z2 : \n" << A*Z2 << std::endl;
    Eigen::MatrixXd A2Z1 = A2*N;
    std::cout << "N(A2*Z1) : \n" << Utils::compute_nullspace_QR(A2Z1) << std::endl;
    std::cout << "A2*Z2 : \n" << A2*Z2 << std::endl;
    std::cout << "--------------------------------" << std::endl;

    Eigen::MatrixXd A3 = Eigen::MatrixXd::Random(5, 15);
    Eigen::MatrixXd Z3 = Z2*Utils::compute_nullspace_QR(A3*Z2);
    std::cout << "Z3 is " << Z3.rows() << "x" << Z3.cols() << std::endl;
    std::cout << "A3*Z3 : \n" << A3*Z3 << std::endl;
    
    Eigen::MatrixXd A4 = Eigen::MatrixXd::Random(2, 15);
    Eigen::MatrixXd Z4 = Z3*Utils::compute_nullspace_QR(A4*Z3);
    std::cout << "Z4 is " << Z4.rows() << "x" << Z4.cols() << std::endl;
    std::cout << "N(A4*Z3) : \n" << Utils::compute_nullspace_QR(A4*Z3) << std::endl;
    return 0;
}