#pragma once
#include "OsqpEigenSolver.hpp"
#include "TaskLS.hpp"
#include "Utils.hpp"

class HOQP_Slack
{
    public:
        HOQP_Slack();
        ~HOQP_Slack();
        void addTask(TaskLS* task);
        void init();
        void updateAllTasks();
        void solve();
        void solveAllTasks();
        Eigen::VectorXd getSolution();

    private:
        OsqpEigenSolver* solver_;
        std::vector<TaskLS*> stacked_tasks_;
        TaskLS* current_task_;
        int task_num_;
        int var_dim_;
        int equality_contraints_dim_;
        Eigen::MatrixXd A_eqs_;
        Eigen::VectorXd lb_eqs_;
        Eigen::VectorXd ub_eqs_;
        Eigen::MatrixXd A_cur_;
        Eigen::VectorXd b_cur_;
        Eigen::MatrixXd H_;
        Eigen::VectorXd g_;
        Eigen::VectorXd xopt_;
        Eigen::VectorXd zopt_; //slack variable vector for the equality task
        Eigen::MatrixXd Z_; //Nullspace 
};