#pragma once 
#include <Eigen/Dense>
#include <Eigen/Sparse> 
#include <iostream>
#include "OsqpEigen/OsqpEigen.h"
#include "RobotParams.hpp"
#include "../../utils/Utils.hpp"
class ConvexMPC {
    public: 
        ConvexMPC(Eigen::VectorXd& l_weights_, Eigen::VectorXd& k_weights);
        
        void reset();

        void calculate_Ac(double yaw);

        void calculate_Bc(double base_mass, const Eigen::Matrix3d& base_inertia, Eigen::Matrix3d& wRb, Eigen::Matrix<double,3,4> foot_position);

        void state_discretization(double dt);

        void calculate_qp_matrices();

        void calculate_hessian();
        
        void calculate_gradient(Eigen::VectorXd& x_0,Eigen::VectorXd& x_ref);

        void calculate_constraints();

        Eigen::MatrixXd get_hessian();

        Eigen::VectorXd get_gradient();

        Eigen::MatrixXd get_constraint_matrix();

        Eigen::VectorXd get_lb();
        
        Eigen::VectorXd get_ub();


    private:
        double mu_;
        double fz_min_;
        double fz_max_;

        Eigen::DiagonalMatrix<double, MPC_STATE_DIM*MPC_HORIZON> L_;
        Eigen::DiagonalMatrix<double, NUM_ACTUATED_DOF*MPC_HORIZON> K_;

        Eigen::Matrix<double,MPC_STATE_DIM,MPC_STATE_DIM> Ac_;
        Eigen::Matrix<double, MPC_STATE_DIM, NUM_ACTUATED_DOF> Bc_;
        Eigen::Matrix<double, MPC_STATE_DIM*MPC_HORIZON, NUM_ACTUATED_DOF> Bc_list_;
        
        Eigen::Matrix<double,MPC_STATE_DIM,MPC_STATE_DIM> Ad_;
        Eigen::Matrix<double, MPC_STATE_DIM, NUM_ACTUATED_DOF> Bd_;
        Eigen::Matrix<double, MPC_STATE_DIM*MPC_HORIZON, NUM_ACTUATED_DOF> Bd_list_;

        Eigen::Matrix<double, MPC_STATE_DIM*MPC_HORIZON, MPC_STATE_DIM> A_qp_;
        Eigen::Matrix<double, MPC_STATE_DIM*MPC_HORIZON, NUM_ACTUATED_DOF*MPC_HORIZON> B_qp_;
        Eigen::SparseMatrix<double> A_qp_sparse_;
        Eigen::SparseMatrix<double> B_qp_sparse_;

        Eigen::MatrixXd H_;
        Eigen::VectorXd g_;
        Eigen::MatrixXd C_;
        Eigen::Vector<double, MPC_CONSTRAINT_DIM*MPC_HORIZON> lb_;
        Eigen::Vector<double, MPC_CONSTRAINT_DIM*MPC_HORIZON> ub_;

        Eigen::Matrix<double, MPC_STATE_DIM, 1> x0_;
};