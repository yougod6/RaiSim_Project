#include "TaskLS_EnergyOpt.hpp"

TaskLS_EnergyOpt::TaskLS_EnergyOpt(raisim::World* world, raisim::ArticulatedSystem* robot, const int var_dim)
: world_(world), robot_(robot), TaskLS(robot->getDOF()-6, var_dim)
{
    task_name_ = "Energy Optimization";
    dof_ = robot_->getDOF();

    A_ = Eigen::MatrixXd::Zero(task_dim_, var_dim_);
    A_.block(0,dof_,task_dim_,task_dim_) = Eigen::MatrixXd::Identity(task_dim_,task_dim_);
    b_ = Eigen::VectorXd::Zero(task_dim_);
}

TaskLS_EnergyOpt::~TaskLS_EnergyOpt(){}

void TaskLS_EnergyOpt::updateMatrix()
{
}

void TaskLS_EnergyOpt::updateVector()
{
}


