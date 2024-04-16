#pragma once

#include<Eigen/Sparse>
#include<Eigen/Dense>
class IOsqpEigenSolver
{
public:
    virtual ~IOsqpEigenSolver()=default;
    virtual void init( Eigen::MatrixXd& P,  Eigen::VectorXd& q,bool verbose)=0;
    virtual void init( Eigen::MatrixXd& P,  Eigen::VectorXd& q,  Eigen::MatrixXd& A,  Eigen::VectorXd& l,  Eigen::VectorXd& u,bool verbose)=0;
    virtual void updateVectors( Eigen::VectorXd& q,  Eigen::VectorXd& l,  Eigen::VectorXd& u)=0;
    virtual void updateMatrices( Eigen::MatrixXd& P,  Eigen::MatrixXd& A)=0;
    virtual bool solve()=0;
    virtual Eigen::VectorXd getSolution()=0;
};
