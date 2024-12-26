#pragma once
#include "TaskLS.hpp"
#include "Utils.hpp"

class TaskLS_MinMotion: public TaskLS
{
    public:
        TaskLS_MinMotion(RobotState* robot_state, const int task_dim=6, const int var_dim=30, const double kp=100, const double kd=(2*sqrt(100)));
        ~TaskLS_MinMotion();
        void updateVector()override;
        void updateMatrix()override;

    private:
        RobotState* robot_state_;
        int dof_; 
        int actuated_dim_;
    
        double kp_;
        double kd_;
        
        Eigen::VectorXd nominal_actuated_q_;
        Eigen::VectorXd nominal_actuated_dq_;
        Eigen::VectorXd actuated_q_;
        Eigen::VectorXd actuated_dq_;
};