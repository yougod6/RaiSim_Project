#pragma once
#include "TaskLS.hpp"

class TaskLS_TorqueLimits: public TaskLS
{
    public:
        TaskLS_TorqueLimits(Eigen::VectorXd tau_min, Eigen::VectorXd tau_max);
        ~TaskLS_TorqueLimits();
        void updateVector()override;
        void updateMatrix()override;
    private:
        int actuated_dim_;
        Eigen::VectorXd tau_min_;
        Eigen::VectorXd tau_max_;
};