#include "TaskLS_StationaryFeet.hpp"

TaskLS_StationaryFeet::TaskLS_StationaryFeet(RobotState* robot_state, const int task_dim, const int var_dim)
: TaskLS(task_dim, var_dim), robot_state_(robot_state)
{
    
    task_name_ = "Stationary Feet";
    dof_ = robot_state_->getDOF();

    J_c_ = Eigen::MatrixXd::Zero(task_dim_, dof_);
    dJ_c_ = Eigen::MatrixXd::Zero(task_dim_, dof_);
    dq_ = Eigen::VectorXd::Zero(dof_);

    A_ = Eigen::MatrixXd::Zero(task_dim_,var_dim_);
    b_ = Eigen::VectorXd::Zero(task_dim_);
}

TaskLS_StationaryFeet::~TaskLS_StationaryFeet(){}

void TaskLS_StationaryFeet::updateMatrix()
{
    J_c_ = robot_state_->getContactJacobian();
    A_.block(0,0,task_dim_,dof_) = J_c_;
}

void TaskLS_StationaryFeet::updateVector()
{
    dJ_c_ = robot_state_->getContactJacobianRate();
    b_ = - dJ_c_*robot_state_->getGeneralizedVelocities();
}


