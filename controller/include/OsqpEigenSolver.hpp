// OsqpEigenSolver.h

#pragma once
#include "IOsqpEigenSolver.hpp"
#include "OsqpEigen/OsqpEigen.h"

class OsqpEigenSolver : public IOsqpEigenSolver
{
public:
    OsqpEigenSolver();
    ~OsqpEigenSolver();
    void init( Eigen::SparseMatrix<double>& P,  Eigen::VectorXd& q,bool verbose) override;
    void init( Eigen::SparseMatrix<double>& P,  Eigen::VectorXd& q,  Eigen::SparseMatrix<double>& A,  Eigen::VectorXd& l,  Eigen::VectorXd& u, bool verbose) override;
    void updateVectors( Eigen::VectorXd& q,  Eigen::VectorXd& l,  Eigen::VectorXd& u) override;
    void updateMatrices( Eigen::SparseMatrix<double>& P,  Eigen::SparseMatrix<double>& A) override;
    bool solve() override;
    Eigen::VectorXd getSolution() override;

private:
    OsqpEigen::Solver solver_;
};
