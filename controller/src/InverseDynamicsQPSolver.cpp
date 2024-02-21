#include "InverseDynamicsQPSolver.hpp"

InverseDynamicsQPSolver::InverseDynamicsQPSolver(raisim::World* world, raisim::ArticulatedSystem* robot)
{
    world_ = world;
    robot_ = robot;
    gravity_ = world_->getGravity();
    M_ = Eigen::MatrixXd::Zero(robot_->getDOF(), robot_->getDOF());
    h_ = Eigen::VectorXd::Zero(robot_->getDOF());
    A_tmp = Eigen::MatrixXd::Zero(18,42);
    // A_tmp = Eigen::MatrixXd::Zero(18,54);
    b_tmp = Eigen::VectorXd::Zero(robot_->getDOF());
    J_c_FR_ = Eigen::MatrixXd::Zero(6, robot_->getDOF());
    J_c_FL_ = Eigen::MatrixXd::Zero(6, robot_->getDOF());
    J_c_RR_ = Eigen::MatrixXd::Zero(6, robot_->getDOF());
    J_c_RL_ = Eigen::MatrixXd::Zero(6, robot_->getDOF());
    J_c_ = Eigen::MatrixXd::Zero(12, robot_->getDOF());
    // J_c_ = Eigen::MatrixXd::Zero(24, robot_->getDOF());

    S_ = Eigen::MatrixXd::Zero(robot_->getDOF()-6, robot_->getDOF()); //12x18
    S_.block(0,6,12,12) = Eigen::MatrixXd::Identity(robot_->getDOF()-6,robot_->getDOF()-6);
}

InverseDynamicsQPSolver::~InverseDynamicsQPSolver(){}

void InverseDynamicsQPSolver::init(Eigen::SparseMatrix<double>& P, Eigen::VectorXd& q, Eigen::SparseMatrix<double>& A, Eigen::VectorXd& l, Eigen::VectorXd& u, bool verbose)
{
    updateHessianMatrix(P);
    updateGradientVector(q);
    updateLinearConstraintsMatrix(A);
    updateLowerBoundVector(l);
    updateUpperBoundVector(u);
    OsqpEigenSolver::init(P, q, A, l, u, verbose);
}

void InverseDynamicsQPSolver::updateVectors(Eigen::VectorXd& q, Eigen::VectorXd& l, Eigen::VectorXd& u)
{
    updateGradientVector(q);
    updateLowerBoundVector(l);
    updateUpperBoundVector(u);
    OsqpEigenSolver::updateVectors(q, l, u);
}

void InverseDynamicsQPSolver::updateMatrices(Eigen::SparseMatrix<double>& P, Eigen::SparseMatrix<double>& A)
{
    updateHessianMatrix(P);
    updateLinearConstraintsMatrix(A);
    OsqpEigenSolver::updateMatrices(P, A);
}

void InverseDynamicsQPSolver::updateHessianMatrix(Eigen::SparseMatrix<double> &P)
{
    M_ = robot_->getMassMatrix().e();

    robot_->getDenseFrameJacobian("FR_foot_fixed", J_c_FR_); // 6x18
    robot_->getDenseFrameJacobian("FL_foot_fixed", J_c_FL_); // 6x18
    robot_->getDenseFrameJacobian("RR_foot_fixed", J_c_RR_); // 6x18
    robot_->getDenseFrameJacobian("RL_foot_fixed", J_c_RL_); // 6x18

    // J_c_.block(0, 0, 6, robot_->getDOF()) = J_c_FR_;
    // J_c_.block(6, 0, 6, robot_->getDOF()) = J_c_FL_;
    // J_c_.block(12, 0, 6, robot_->getDOF()) = J_c_RR_;
    // J_c_.block(18, 0, 6, robot_->getDOF()) = J_c_RL_;

    J_c_.block(0, 0, 3, 18) = J_c_FR_.block(0,0,3,18);
    J_c_.block(3, 0, 3, 18) = J_c_FL_.block(0,0,3,18);
    J_c_.block(6, 0, 3, 18) = J_c_RR_.block(0,0,3,18);
    J_c_.block(9, 0, 3, 18) = J_c_RL_.block(0,0,3,18); //12x18

    // A_tmp.block(0,0,18,18) = M_;
    // A_tmp.block(0,18,18,24) = -J_c_.transpose();
    // A_tmp.block(0,42,18,12) = -S_.transpose();

    A_tmp.block(0,0,18,18) = M_;
    A_tmp.block(0,18,18,12) = -J_c_.transpose(); //18x12
    A_tmp.block(0,30,18,12) = -S_.transpose();

    P = (A_tmp.transpose()*A_tmp).sparseView();
}

void InverseDynamicsQPSolver::updateGradientVector(Eigen::VectorXd &q)
{
    h_ = robot_->getNonlinearities(gravity_).e();
    b_tmp = -h_;

    q = -A_tmp.transpose()*b_tmp;
}

void InverseDynamicsQPSolver::updateLinearConstraintsMatrix(Eigen::SparseMatrix<double> &A)
{
    Eigen::MatrixXd A_dense = Eigen::MatrixXd::Zero(12, 42);
    // Eigen::MatrixXd A_dense = Eigen::MatrixXd::Zero(12, 54);
    // A_dense.block(0,42,12,12) = Eigen::MatrixXd::Identity(12,12);
    A_dense.block(0,30,12,12) = Eigen::MatrixXd::Identity(12,12);
    A = A_dense.sparseView();
}

void InverseDynamicsQPSolver::updateLowerBoundVector(Eigen::VectorXd &l)
{
    l = robot_->getActuationLowerLimits().e().tail(12);
}

void InverseDynamicsQPSolver::updateUpperBoundVector(Eigen::VectorXd &u)
{
    u = robot_->getActuationUpperLimits().e().tail(12);
}
