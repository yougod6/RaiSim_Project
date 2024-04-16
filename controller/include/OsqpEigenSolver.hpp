// OsqpEigenSolver.h

#pragma once
#include "IOsqpEigenSolver.hpp"
#include "OsqpEigen/OsqpEigen.h"

class OsqpEigenSolver : public IOsqpEigenSolver
{
public:
    OsqpEigenSolver();
    ~OsqpEigenSolver();
    void init( Eigen::MatrixXd& P,  Eigen::VectorXd& q,bool verbose) override;
    void init( Eigen::MatrixXd& P,  Eigen::VectorXd& q,  Eigen::MatrixXd& A,  Eigen::VectorXd& l,  Eigen::VectorXd& u, bool verbose) override;
    void updateVectors( Eigen::VectorXd& q,  Eigen::VectorXd& l,  Eigen::VectorXd& u) override;
    void updateMatrices( Eigen::MatrixXd& P,  Eigen::MatrixXd& A) override;
    bool solve() override;
    Eigen::VectorXd getSolution() override;

private:
    OsqpEigen::Solver solver_;
};
