#pragma once
#include "TaskLS.hpp"
#include "Utils.hpp"

class TaskLS_MoveBase: public TaskLS
{

    public:
        TaskLS_MoveBase(RobotState* robot_state, const int task_dim, const int var_dim, Eigen::VectorXd desired_x ,  const double kp=100, const double kd=(2*sqrt(100)));
        ~TaskLS_MoveBase();

        void updateVector()override;
        void updateMatrix()override;
        void updateDesiredBasePose(Eigen::VectorXd desired_x);
        void updateDesiredBaseAcceleration();
    
    private:
        RobotState* robot_state_;
        double dof_; 
        double kp_;
        double kd_;
        
        Eigen::Vector3d base_position_;
        Eigen::Quaterniond base_quaternion_;    

        Eigen::MatrixXd J_B_;
        Eigen::MatrixXd dJ_B_;
        Eigen::VectorXd dq_;
        Eigen::VectorXd desired_x_;
        Eigen::VectorXd desired_xdot_;
        Eigen::VectorXd desired_xddot_;
};