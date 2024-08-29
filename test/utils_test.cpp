#include "Utils.hpp"
#include <iostream>


// Function to print a matrix
void printMatrix(const Eigen::MatrixXd& mat, const std::string& name) {
    std::cout << name << ":\n" << mat << "\n\n";
}


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

    // Define the matrices and vector of matrices
    Eigen::MatrixXd Z_(3, 3);
    Z_ << 1, 2, 3,
          4, 5, 6,
          7, 8, 9;

    Eigen::MatrixXd Z_1(2, 3);
    Z_1 << 1, 2, 3,
         4, 5, 6;

    Eigen::MatrixXd Z_2(3, 3);
    Z_2 << 7, 8, 9,
         10, 11, 12,
         13, 14, 15;

    Eigen::MatrixXd Z_3(1, 3);
    Z_3 << 16, 17, 18;

    std::vector<Eigen::MatrixXd> D_ineq_vec_ = {Z_1, Z_2, Z_3};

    // Calculate the total number of rows for D_tilde
    int total_rows = 0;
    for (const auto& mat : D_ineq_vec_) {
        total_rows += mat.rows();
    }

    // Initialize D_tilde with the appropriate size
    Eigen::MatrixXd D_tilde(total_rows, Z_.cols());

    // Fill D_tilde using reverse iterator
    int row = 0;
    for (auto it = D_ineq_vec_.rbegin(); it != D_ineq_vec_.rend(); ++it) {
        D_tilde.block(row, 0, it->rows(), Z_.cols()) = (*it) * Z_;
        // Debugging purpose
        std::cout << "Matrix:\n" << *it << "\n";
        std::cout << "Z_:\n" << Z_ << "\n";
        std::cout << "(*it) * Z_:\n" << (*it) * Z_ << "\n\n";
        
        row += it->rows();
    }

    // Print the final D_tilde matrix
    printMatrix(D_tilde, "D_tilde");

    return 0;
}