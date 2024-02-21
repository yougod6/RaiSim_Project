#pragma once
#include "OsqpEigenSolver.hpp"
#include "TaskLS.hpp"

class HOQP{
    public:
        HOQP();
        ~HOQP();
        void addTask(TaskLS* task);
        void init();
        void solve();
        void updateAllTasks();
        void solveAllTasks();
        double getTaskNum();
        Eigen::VectorXd getSolution();

    private:
        double task_num_;
        OsqpEigenSolver* solver_;
        std::vector<TaskLS*> stacked_tasks_;
        Eigen::MatrixXd A_eqs_;
        Eigen::VectorXd lb_eqs_;
        Eigen::VectorXd ub_eqs_;
        Eigen::SparseMatrix<double> H_;
        Eigen::VectorXd g_;
        Eigen::VectorXd xopt_;
};

