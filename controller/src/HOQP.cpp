#include "HOQP.hpp"

HOQP::HOQP()
{
    solver_ = new OsqpEigenSolver();
    task_num_ = 0;
    equality_contraints_dim_ = 0;
    var_dim_ = 0;
}

HOQP::~HOQP()
{
    delete solver_;
}


void HOQP::addTask(TaskLS* task)
{
    stacked_tasks_.push_back(task);
    task_num_++;
}

void HOQP::init()
{
    equality_contraints_dim_=0;
    if(task_num_<1){
        std::cout << "No task added" << std::endl;
        return;
    }
    updateAllTasks();
    
    for(int i=0; i<task_num_-1; i++)
    {
        equality_contraints_dim_ += stacked_tasks_[i]->getTaskDim();
    }
    var_dim_ = stacked_tasks_[0]->getVarDim();
    A_eqs_ = Eigen::MatrixXd::Zero(equality_contraints_dim_,var_dim_);
    lb_eqs_ = Eigen::VectorXd::Zero(equality_contraints_dim_);
    ub_eqs_ = Eigen::VectorXd::Zero(equality_contraints_dim_);
}

int HOQP::getTaskNum()
{
    return task_num_;
}   

int HOQP::getEqualityConstraintsDim()
{
    return equality_contraints_dim_;
}

int HOQP::getVarDim()
{
    return var_dim_;
}

void HOQP::solve()
{
    // solver_->init(H, g, A, l, u, Aeq, beq);
    // solver_->solve();
}

void HOQP::solveAllTasks()
{
    init();
    int stacked_row_idx = 0;
    for (TaskLS* task : stacked_tasks_) {
        Eigen::MatrixXd A = task->getMatrix();
        H_ = A.transpose()*A;
        g_ = -A.transpose()*task->getVector();
        solver_->init(H_, g_, A_eqs_, lb_eqs_, ub_eqs_, false);
        solver_->solve();
        xopt_ = solver_->getSolution();
        // task가 stacked_task_의 마지막 task가 아니라면
        if(task != stacked_tasks_.back())
        {
            Eigen::VectorXd b = A*xopt_;
            A_eqs_.block(stacked_row_idx,0,task->getTaskDim(),var_dim_) = A;
            lb_eqs_.segment(stacked_row_idx,task->getTaskDim()) = b;
            ub_eqs_.segment(stacked_row_idx,task->getTaskDim()) = b;
            stacked_row_idx += task->getTaskDim();
        }
   
    }
}

void HOQP::updateAllTasks()
{
    for (TaskLS* task : stacked_tasks_) {
        task->updateMatrix();
        task->updateVector();
    }
}

Eigen::VectorXd HOQP::getSolution()
{
    return xopt_;
}