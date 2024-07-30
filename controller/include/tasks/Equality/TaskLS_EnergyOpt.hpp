#pragma once
#include "TaskLS.hpp"

class TaskLS_EnergyOpt: public TaskLS
{

    public:
        TaskLS_EnergyOpt(raisim::World* world, raisim::ArticulatedSystem* robot, const int var_dim=30);
        ~TaskLS_EnergyOpt();
        void updateVector()override;
        void updateMatrix()override;
    
    private:
        raisim::World* world_;
        raisim::ArticulatedSystem* robot_;
        int dof_; 
};