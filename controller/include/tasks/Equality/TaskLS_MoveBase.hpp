#pragma once
#include "TaskLS.hpp"
#include "Utils.hpp"

class TaskLS_MoveBase: public TaskLS
{

    public:
        TaskLS_MoveBase(raisim::World* world, raisim::ArticulatedSystem* robot, const int task_dim, const int var_dim, Eigen::VectorXd desired_x ,  const double kp=100, const double kd=(2*sqrt(100)));
        ~TaskLS_MoveBase();

        void update_dJ_B(const double dt=0.001);
        void update_J_B();
        void updateVector()override;
        void updateMatrix()override;
        void updateDesiredBasePose(Eigen::VectorXd desired_x);
        // void makeBaseTrajectory(double time);
        void updateDesiredBaseAcceleration();
    
    private:
        raisim::World* world_;
        raisim::ArticulatedSystem* robot_;
        double dof_; 
        raisim::Vec<3> gravity_;
        double kp_;
        double kd_;
        
        Eigen::MatrixXd J_B_position;
        Eigen::MatrixXd J_B_rotation;
        Eigen::MatrixXd J_B_;
        Eigen::MatrixXd dJ_B_;
        Eigen::VectorXd dq_;

        Eigen::VectorXd desired_x_;
        Eigen::VectorXd desired_xdot_;
        Eigen::VectorXd desired_xddot_;
};