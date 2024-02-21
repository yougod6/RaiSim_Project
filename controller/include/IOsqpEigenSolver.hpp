#pragma once

#include<Eigen/Sparse>
#include<Eigen/Dense>
class IOsqpEigenSolver
{
public:
    virtual ~IOsqpEigenSolver()=default;
    virtual void init( Eigen::SparseMatrix<double>& P,  Eigen::VectorXd& q,bool verbose)=0;
    virtual void init( Eigen::SparseMatrix<double>& P,  Eigen::VectorXd& q,  Eigen::SparseMatrix<double>& A,  Eigen::VectorXd& l,  Eigen::VectorXd& u,bool verbose)=0;
    virtual void updateVectors( Eigen::VectorXd& q,  Eigen::VectorXd& l,  Eigen::VectorXd& u)=0;
    virtual void updateMatrices( Eigen::SparseMatrix<double>& P,  Eigen::SparseMatrix<double>& A)=0;
    virtual bool solve()=0;
    virtual Eigen::VectorXd getSolution()=0;
};
