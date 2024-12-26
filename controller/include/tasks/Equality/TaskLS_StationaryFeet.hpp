#pragma once
#include "TaskLS.hpp"

class TaskLS_StationaryFeet: public TaskLS
{

    public:
        TaskLS_StationaryFeet(RobotState* robot_state, const int task_dim=12, const int var_dim=30);
        ~TaskLS_StationaryFeet();

        void updateVector()override;
        void updateMatrix()override;


    private:
        RobotState* robot_state_;
        double dof_;    
        
        Eigen::MatrixXd J_c_;
        Eigen::MatrixXd dJ_c_;
        Eigen::VectorXd dq_;
};
