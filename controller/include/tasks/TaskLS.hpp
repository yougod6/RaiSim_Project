#pragma once
#include<Eigen/Sparse>
#include<Eigen/Dense>
#include "raisim/World.hpp"
#include "RobotState.hpp"
class TaskLS
{

    public:
        TaskLS(int task_dim, int var_dim);
        virtual ~TaskLS();

        virtual void updateMatrix();
        virtual void updateVector();
        std::string getTaskName();
        Eigen::MatrixXd getMatrix();
        Eigen::VectorXd getVector();
        int getTaskDim();
        int getVarDim();

    protected:
        std::string task_name_;
        int task_dim_;
        int var_dim_;
        Eigen::MatrixXd A_;
        Eigen::VectorXd b_;
};


