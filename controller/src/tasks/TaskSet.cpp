#include "TaskSet.hpp"

TaskSet::TaskSet(const int var_dim) : var_dim_(var_dim)
{
    equality_task_dim_ = 0;
    inequality_task_dim_ = 0;
}

TaskSet::~TaskSet()
{
}

void TaskSet::addEqualityTask(TaskLS* task)
{
    if(task->getVarDim() != var_dim_)
    {
        std::cout << "Variable dimension mismatch" << std::endl;
        return;
    }
    equality_task_.push_back(task);
    equality_task_dim_ += task->getTaskDim();
    
    A_eqs_ = Eigen::MatrixXd::Zero(equality_task_dim_, var_dim_);
    b_eqs_ = Eigen::VectorXd::Zero(equality_task_dim_);
}

void TaskSet::addInequalityTask(TaskLS* task)
{
    if(task->getVarDim() != var_dim_)
    {
        std::cout << "Variable dimension mismatch" << std::endl;
        return;
    }

    inequality_task_.push_back(task);
    inequality_task_dim_ += task->getTaskDim();

    D_ineqs_ = Eigen::MatrixXd::Zero(inequality_task_dim_, var_dim_);
    f_ineqs_ = Eigen::VectorXd::Zero(inequality_task_dim_);
}

void TaskSet::updateAllTasks()
{
    updateEqualityTask();
    updateInequalityTask();
}

void TaskSet::updateEqualityTask()
{
    for(auto task : equality_task_)
    {
        task->updateMatrix();
        task->updateVector();
    }
}

void TaskSet::updateInequalityTask()
{
    for(auto task : inequality_task_)
    {
        task->updateMatrix();
        task->updateVector();
    }
}

Eigen::MatrixXd TaskSet::getEqualityMatrix()
{
    int row = 0;    
    for(auto task : equality_task_)
    {
        A_eqs_.block(row,0,task->getTaskDim(),var_dim_) = task->getMatrix();
        row += task->getTaskDim();
    }
    return A_eqs_;
}

Eigen::VectorXd TaskSet::getEqualityVector()
{
    int row = 0;
    for(auto task : equality_task_)
    {
        b_eqs_.segment(row,task->getTaskDim()) = task->getVector();
        row += task->getTaskDim();
    }
    return b_eqs_;
}

Eigen::MatrixXd TaskSet::getInequalityMatrix()
{
    if(inequality_task_dim_ == 0)
    {
        return Eigen::MatrixXd::Zero(1,var_dim_);
    }
    int row = 0;
    for(auto task : inequality_task_)
    {
        D_ineqs_.block(row,0,task->getTaskDim(),var_dim_) = task->getMatrix();
        row += task->getTaskDim();
    }
    return D_ineqs_;
}

Eigen::VectorXd TaskSet::getInequalityVector()
{
    if (inequality_task_dim_ == 0)
    {
        // 무한대 크기 1짜리 벡터 리턴
        return std::numeric_limits<double>::infinity()*Eigen::VectorXd::Ones(1); 
    }
    
    int row = 0;
    for(auto task : inequality_task_)
    {
        f_ineqs_.segment(row,task->getTaskDim()) = task->getVector();
        row += task->getTaskDim();
    }
    return f_ineqs_;
}

std::vector<std::string> TaskSet::getEqualityTaskNames()
{
    std::vector<std::string> task_names;
    for(auto task : equality_task_)
    {
        task_names.push_back(task->getTaskName());
    }
    return task_names;
}

std::vector<std::string> TaskSet::getInequalityTaskNames()
{
    std::vector<std::string> task_names;
    for(auto task : inequality_task_)
    {
        task_names.push_back(task->getTaskName());
    }
    return task_names;
}

int TaskSet::getEqualityTaskDim()
{
    return equality_task_dim_;
}

int TaskSet::getInequalityTaskDim()
{
    if(inequality_task_dim_ == 0)
    {
        return 1;
    }
    return inequality_task_dim_;
}

int TaskSet::getVarDim()
{
    return var_dim_;
}