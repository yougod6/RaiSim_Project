#include "HOQP_Slack.hpp"

HOQP_Slack::HOQP_Slack()
{
    solver_ = new OsqpEigenSolver();
    task_num_ = 0;
    equality_contraints_dim_ = 0;
    var_dim_ = 0;
}

HOQP_Slack::~HOQP_Slack()
{
    delete solver_;
}

void HOQP_Slack::addTask(TaskLS* task)
{
    stacked_tasks_.push_back(task);
    task_num_++;
}

void HOQP_Slack::init()
{
    if(task_num_<1){
        std::cout << "No task added" << std::endl;
        return;
    }
    updateAllTasks();
    var_dim_ = stacked_tasks_[0]->getVarDim();
    xopt_ = Eigen::VectorXd::Zero(var_dim_);
    Z_ = Eigen::MatrixXd::Identity(var_dim_, var_dim_);
}

void HOQP_Slack::solve()
{
    A_cur_ = current_task_->getMatrix();
    b_cur_ = current_task_->getVector();
    H_ = Z_.transpose()*A_cur_.transpose()*A_cur_*Z_;
    g_ = Z_.transpose()*A_cur_.transpose()*(A_cur_*xopt_-b_cur_);
    solver_->init(H_, g_,false);
    solver_->solve();
    zopt_ = solver_->getSolution();
}

void HOQP_Slack::solveAllTasks()
{
    updateAllTasks();
    xopt_ = Eigen::VectorXd::Zero(var_dim_);
    Z_ = Eigen::MatrixXd::Identity(var_dim_, var_dim_);
    for (TaskLS* task : stacked_tasks_) {
        //첫번째 task 인 경우
        if(task == stacked_tasks_[0]){
            current_task_ = task;
            A_cur_ = current_task_->getMatrix();
            b_cur_ = current_task_->getVector();
            H_ = A_cur_.transpose()*A_cur_;
            g_ = -A_cur_.transpose()*b_cur_;
            solver_->init(H_, g_,false);
            solver_->solve();
            xopt_ = solver_->getSolution();
            Z_ = Z_*Utils::compute_nullspace_QR(A_cur_*Z_); //Z1
        }
        else{
            current_task_ = task;
            solve();
            xopt_ = xopt_ + Z_*zopt_;
            if(current_task_!=stacked_tasks_[task_num_-1])
                Z_ = Z_*Utils::compute_nullspace_QR(A_cur_*Z_);
        }
    }
}

void HOQP_Slack::updateAllTasks()
{
    for (TaskLS* task : stacked_tasks_) {
        task->updateMatrix();
        task->updateVector();
    }
}

Eigen::VectorXd HOQP_Slack::getSolution()
{
    return xopt_;
}