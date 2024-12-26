#pragma once
#include "TaskLS.hpp"
#include "Utils.hpp"

class TaskLS_StationaryEE: public TaskLS
{

    public:
        TaskLS_StationaryEE(RobotState* robot_state, const int task_dim, const int var_dim, Eigen::VectorXd desired_x ,const double kp=100, const double kd=(2*sqrt(100)));
        ~TaskLS_StationaryEE();

        void updateVector()override;
        void updateMatrix()override;
        // void makeEETrajectory(double time);
        void updateDesiredEEPose(Eigen::VectorXd desired_x);
        void updateDesiredBaseAcceleration();
    
    private:
        RobotState* robot_state_;
        int dof_; 
        double kp_;
        double kd_;
        
        Eigen::Vector3d ee_position_;
        Eigen::Quaterniond ee_quaternion_;
        Eigen::Vector3d ee_euler_;
        Eigen::Vector3d ee_linear_velocity_;
        Eigen::Vector3d ee_angular_velocity_;
        Eigen::MatrixXd J_EE_position;
        Eigen::MatrixXd J_EE_rotation;
        Eigen::MatrixXd J_EE_;
        Eigen::MatrixXd dJ_EE_;
        Eigen::VectorXd dq_;

        Eigen::VectorXd desired_x_;
        Eigen::VectorXd desired_xdot_;
        Eigen::VectorXd desired_xddot_;
};