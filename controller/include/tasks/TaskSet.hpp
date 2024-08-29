#pragma once 

#include "TaskLS.hpp"

class TaskSet
{
    public:
        TaskSet(const int var_dim);
        ~TaskSet();

        void addEqualityTask(TaskLS* task);
        void addInequalityTask(TaskLS* task);
        void updateAllTasks();
        void updateEqualityTask();
        void updateInequalityTask();
        Eigen::MatrixXd getEqualityMatrix();
        Eigen::VectorXd getEqualityVector();
        Eigen::MatrixXd getInequalityMatrix();
        Eigen::VectorXd getInequalityVector();
        int getEqualityTaskDim();
        int getInequalityTaskDim();
        int getVarDim();
        std::vector<std::string> getEqualityTaskNames();
        std::vector<std::string> getInequalityTaskNames();

    private:
        Eigen::MatrixXd A_eqs_;
        Eigen::VectorXd b_eqs_;
        Eigen::MatrixXd D_ineqs_;
        Eigen::VectorXd f_ineqs_;
        int equality_task_dim_;
        int inequality_task_dim_;
        int var_dim_;
        std::vector<TaskLS*> equality_task_;
        std::vector<TaskLS*> inequality_task_;
};