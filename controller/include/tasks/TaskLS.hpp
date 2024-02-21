#pragma once
#include<Eigen/Sparse>
#include<Eigen/Dense>
#include "raisim/World.hpp"

class TaskLS
{

    public:
        TaskLS();
        virtual ~TaskLS();

        virtual void updateMatrix();
        virtual void updateVector();
        std::string getTaskName();
        Eigen::MatrixXd getMatrix();
        Eigen::VectorXd getVector();

    protected:
        std::string task_name_;
        Eigen::MatrixXd A_;
        Eigen::VectorXd b_;
};


