#include "TaskLS_EnergyOpt.hpp"

TaskLS_EnergyOpt::TaskLS_EnergyOpt(raisim::World* world, raisim::ArticulatedSystem* robot, const int var_dim)
{
    world_ = world;
    robot_ = robot;
    var_dim_ = var_dim;
    dof_ = robot_->getDOF();
    task_dim_ = dof_-6;

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


