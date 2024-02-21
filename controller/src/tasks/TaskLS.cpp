#include "TaskLS.hpp"

TaskLS::TaskLS()
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

