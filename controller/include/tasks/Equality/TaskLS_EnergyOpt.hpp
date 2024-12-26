#pragma once
#include "TaskLS.hpp"

class TaskLS_EnergyOpt: public TaskLS
{

    public:
        TaskLS_EnergyOpt(RobotState* robot_state_, const int var_dim=30);
        ~TaskLS_EnergyOpt();
        void updateVector()override;
        void updateMatrix()override;
    
    private:
        RobotState* robot_state_;
        int dof_; 
};