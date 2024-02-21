#include "OsqpEigenSolver.hpp"
#include "raisim/World.hpp"

class InverseDynamicsQPSolver: public OsqpEigenSolver
{
    public:
        InverseDynamicsQPSolver(raisim::World* world, raisim::ArticulatedSystem* robot);
        ~InverseDynamicsQPSolver(); 

        void init(Eigen::SparseMatrix<double>& P, Eigen::VectorXd& q, Eigen::SparseMatrix<double>& A, Eigen::VectorXd& l, Eigen::VectorXd& u, bool verbose=false);
        void updateVectors(Eigen::VectorXd& q, Eigen::VectorXd& l, Eigen::VectorXd& u);
        void updateMatrices(Eigen::SparseMatrix<double>& P, Eigen::SparseMatrix<double>& A);
        void updateHessianMatrix(Eigen::SparseMatrix<double> &P);
        void updateGradientVector(Eigen::VectorXd &q);
        void updateLinearConstraintsMatrix(Eigen::SparseMatrix<double> &A);
        void updateLowerBoundVector(Eigen::VectorXd &l);
        void updateUpperBoundVector(Eigen::VectorXd &u);


    private:
        raisim::World* world_;
        raisim::ArticulatedSystem* robot_;
        raisim::Vec<3> gravity_;
        Eigen::MatrixXd M_;
        Eigen::VectorXd h_;
        
        Eigen::MatrixXd J_c_FR_;
        Eigen::MatrixXd J_c_FL_;
        Eigen::MatrixXd J_c_RR_;
        Eigen::MatrixXd J_c_RL_;
        Eigen::MatrixXd J_c_;
        Eigen::MatrixXd S_;

        Eigen::MatrixXd A_tmp;
        Eigen::VectorXd b_tmp;
};