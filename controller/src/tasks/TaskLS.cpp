#include "TaskLS.hpp"

TaskLS::TaskLS(int task_dim, int var_dim)
: task_dim_(task_dim), var_dim_(var_dim)
{
}

TaskLS::~TaskLS()
{
}

void TaskLS::updateVector()
{
    std::cout << "Not implemented" << std::endl;
}

void TaskLS::updateMatrix()
{
    std::cout << "Not implemented" << std::endl;
}

std::string TaskLS::getTaskName()
{
    return task_name_;
}

Eigen::MatrixXd TaskLS::getMatrix()
{
    return A_;
}

Eigen::VectorXd TaskLS::getVector()
{
    return b_;
}

int TaskLS::getTaskDim()
{
    return task_dim_;
}

int TaskLS::getVarDim()
{
    return var_dim_;
}