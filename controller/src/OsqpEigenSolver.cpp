#include "OsqpEigenSolver.hpp"

OsqpEigenSolver::OsqpEigenSolver(){}

OsqpEigenSolver::~OsqpEigenSolver(){}

void OsqpEigenSolver::init(Eigen::MatrixXd& P, Eigen::VectorXd& q,bool verbose){
    Eigen::SparseMatrix<double> P_ = P.sparseView();
    solver_.clearSolver();
    solver_.data()->clearHessianMatrix();
    solver_.data()->clearLinearConstraintsMatrix();
    
    solver_.settings()->setWarmStart(true);
    solver_.data()->setNumberOfVariables(P_.cols());
    solver_.data()->setNumberOfConstraints(0);   
    solver_.data()->setHessianMatrix(P_);
    solver_.data()->setGradient(q);

    solver_.settings()->setVerbosity(verbose);
   
    if (!solver_.initSolver()) {
        std::cerr << "Error initializing the solver." << std::endl;
    }
}


void OsqpEigenSolver::init( Eigen::MatrixXd& P,  Eigen::VectorXd& q,  Eigen::MatrixXd& A,  Eigen::VectorXd& l,  Eigen::VectorXd& u,bool verbose=false){
    Eigen::SparseMatrix<double> P_ = P.sparseView();
    Eigen::SparseMatrix<double> A_ = A.sparseView();
    solver_.clearSolver();
    solver_.data()->clearHessianMatrix();
    solver_.data()->clearLinearConstraintsMatrix();
    
    
    solver_.data()->setNumberOfVariables(P_.cols());
    solver_.data()->setNumberOfConstraints(A_.rows());
    solver_.data()->setHessianMatrix(P_);
    solver_.data()->setGradient(q);
    solver_.data()->setLinearConstraintsMatrix(A_);
    solver_.data()->setLowerBound(l);
    solver_.data()->setUpperBound(u);
    solver_.settings()->setWarmStart(true);
    // solver_.settings()->setMaxIteration(1000);
    solver_.settings()->setVerbosity(verbose);
    if (!solver_.initSolver()) {
        std::cerr << "Error initializing the solver." << std::endl;
    }
}

void OsqpEigenSolver::updateVectors( Eigen::VectorXd& q,  Eigen::VectorXd& l,  Eigen::VectorXd& u){
    solver_.updateGradient(q);
    solver_.updateBounds(l, u);
}

void OsqpEigenSolver::updateMatrices( Eigen::MatrixXd& P,  Eigen::MatrixXd& A){
    Eigen::SparseMatrix<double> P_ = P.sparseView();
    Eigen::SparseMatrix<double> A_ = A.sparseView();
    solver_.updateHessianMatrix(P_);
    solver_.updateLinearConstraintsMatrix(A_);
}

bool OsqpEigenSolver::solve(){

    // Solve the QP problem and get the solution
    if (solver_.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
        std::cerr << "Error solving the QP problem." << std::endl;
        return false;
    }
    return true;
}

Eigen::VectorXd OsqpEigenSolver::getSolution(){
    return solver_.getSolution();
}

