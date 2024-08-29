#pragma once
#include "OsqpEigenSolver.hpp"
#include "TaskSet.hpp"
#include "Utils.hpp"

class HOQP_Slack
{
    public:
        HOQP_Slack();
        ~HOQP_Slack();
        void addTask(TaskSet* task);
        void init();
        void updateAllTasks();
        void solve();
        void solveAllTasks();
        Eigen::VectorXd getSolution();

    private:
        OsqpEigenSolver* solver_;
        std::vector<TaskSet*> stacked_tasks_;
        TaskSet* current_task_;
        int task_num_;
        int var_dim_;
        int inequality_contraints_dim_;
        int stacked_constraints_dim_; // determine D_tilde's dimension
        int z_dim_; 
        int v_dim_; 
        Eigen::MatrixXd C_mat_;
        Eigen::VectorXd lb_vec_;
        Eigen::VectorXd ub_vec_;
        Eigen::MatrixXd D_ineq_;
        std::vector<Eigen::MatrixXd> D_ineq_vec_;
        std::vector<Eigen::VectorXd> f_ineq_vec_;
        Eigen::MatrixXd D_tilde;
        Eigen::VectorXd f_tilde;
        Eigen::VectorXd f_ineq_;
        Eigen::MatrixXd A_eq_;
        Eigen::VectorXd b_eq_;
        Eigen::MatrixXd H_;
        Eigen::VectorXd g_;
        Eigen::VectorXd xopt_;
        Eigen::VectorXd zopt_; //slack variable vector for the equality task
        std::vector<Eigen::VectorXd> vopt_vec_; //vector of slack variable vector for the inequality task
        Eigen::MatrixXd Z_; //Nullspace 
};