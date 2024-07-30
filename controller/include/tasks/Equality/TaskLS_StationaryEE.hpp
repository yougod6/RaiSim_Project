#pragma once
#include "TaskLS.hpp"
#include "Utils.hpp"

class TaskLS_StationaryEE: public TaskLS
{

    public:
        TaskLS_StationaryEE(raisim::World* world, raisim::ArticulatedSystem* robot, const int task_dim=6, const int var_dim=30, const double kp=100, const double kd=(2*sqrt(100)));
        ~TaskLS_StationaryEE();

        void update_dJ_EE(const double dt=0.001);
        void update_J_EE();
        void updateVector()override;
        void updateMatrix()override;
        //retrun desired_q_b, desired_dq_b 
        void makeEETrajectory(double time);
        //return desired_ddq_b by pd control
        void updateDesiredBaseAcceleration();
    
    private:
        raisim::World* world_;
        raisim::ArticulatedSystem* robot_;
        int dof_; 
        raisim::Vec<3> gravity_;
        double kp_;
        double kd_;
        
        Eigen::MatrixXd J_EE_position;
        Eigen::MatrixXd J_EE_rotation;
        Eigen::MatrixXd J_EE_;
        Eigen::MatrixXd dJ_EE_;
        Eigen::VectorXd dq_;

        Eigen::VectorXd desired_x_;
        Eigen::VectorXd desired_xdot_;
        Eigen::VectorXd desired_xddot_;
};