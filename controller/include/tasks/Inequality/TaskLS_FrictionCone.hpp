#pragma once 
#include "TaskLS.hpp"

class TaskLS_FrictionCone: public TaskLS
{

    public:
        TaskLS_FrictionCone(RobotState* robot_state, int task_dim=24, int var_dim=30, double mu=0.8);  
        ~TaskLS_FrictionCone();
        void updateVector()override;
        void updateMatrix()override;
        Eigen::VectorXd getStackedContactForces();

    private:
        RobotState* robot_state_;   
        double dof_;

        Eigen::MatrixXd S_; //selection matrix
        Eigen::MatrixXd M_;
        Eigen::VectorXd h_;

        Eigen::MatrixXd J_c_;
        Eigen::MatrixXd Q_;
        Eigen::MatrixXd Qc_;
        Eigen::MatrixXd R_;       
        Eigen::MatrixXd F_block_; //block matrix for friction cone of a single contact point
        Eigen::MatrixXd F_; // stacked F_block_ for all contact points
        Eigen::VectorXd k_; // F_*lambda_ <= k_   
        Eigen::MatrixXd A_c_; 
        Eigen::VectorXd b_c_;
        Eigen::VectorXd lambda_; // lambda_ = A_c * x + b_c       
        double mu_;
        double lambda_min_; //minimum contact force (normal force)
        double lambda_max_; //maximum contact force (normal force)
};