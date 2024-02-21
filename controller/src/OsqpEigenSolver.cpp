#include "OsqpEigenSolver.hpp"

OsqpEigenSolver::OsqpEigenSolver(){}

OsqpEigenSolver::~OsqpEigenSolver(){}

void OsqpEigenSolver::init(Eigen::SparseMatrix<double>&P, Eigen::VectorXd& q,bool verbose){
    solver_.clearSolver();
    solver_.data()->clearHessianMatrix();
    solver_.data()->clearLinearConstraintsMatrix();
    
    solver_.data()->setNumberOfVariables(P.cols());
    solver_.data()->setHessianMatrix(P);
    solver_.data()->setGradient(q);

    // Define empty linear constraint matrix A (no linear constraints)
    Eigen::SparseMatrix<double> A;
    // Define empty lower bound vector l
    Eigen::VectorXd l;
    // Define empty upper bound vector u
    Eigen::VectorXd u;
    solver_.data()->setNumberOfConstraints(A.rows());

    solver_.data()->setLinearConstraintsMatrix(A);
    solver_.data()->setLowerBound(l);
    solver_.data()->setUpperBound(u);
    solver_.settings()->setVerbosity(verbose);

    if (!solver_.initSolver()) {
        std::cerr << "Error initializing the solver." << std::endl;
    }
}


void OsqpEigenSolver::init( Eigen::SparseMatrix<double>& P,  Eigen::VectorXd& q,  Eigen::SparseMatrix<double>& A,  Eigen::VectorXd& l,  Eigen::VectorXd& u,bool verbose=false){
    solver_.clearSolver();
    solver_.data()->clearHessianMatrix();
    solver_.data()->clearLinearConstraintsMatrix();
    
    solver_.data()->setNumberOfVariables(P.cols());
    solver_.data()->setNumberOfConstraints(A.rows());
    solver_.data()->setHessianMatrix(P);
    solver_.data()->setGradient(q);
    solver_.data()->setLinearConstraintsMatrix(A);
    solver_.data()->setLowerBound(l);
    solver_.data()->setUpperBound(u);
    solver_.settings()->setWarmStart(true);
    solver_.settings()->setMaxIteration(1000);
    solver_.settings()->setVerbosity(verbose);
    if (!solver_.initSolver()) {
        std::cerr << "Error initializing the solver." << std::endl;
    }
}

void OsqpEigenSolver::updateVectors( Eigen::VectorXd& q,  Eigen::VectorXd& l,  Eigen::VectorXd& u){
    solver_.updateGradient(q);
    solver_.updateBounds(l, u);
}

void OsqpEigenSolver::updateMatrices( Eigen::SparseMatrix<double>& P,  Eigen::SparseMatrix<double>& A){
    solver_.updateHessianMatrix(P);
    solver_.updateLinearConstraintsMatrix(A);
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

