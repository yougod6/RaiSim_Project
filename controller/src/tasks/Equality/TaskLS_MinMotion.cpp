#include "TaskLS_MinMotion.hpp"

TaskLS_MinMotion::TaskLS_MinMotion(RobotState* robot_state, const int task_dim, const int var_dim, const double kp, const double kd)
: TaskLS(task_dim, var_dim), robot_state_(robot_state), kp_(kp), kd_(kd)
{
    task_name_ = "Minimum Motion";
    dof_ = robot_state_->getDOF();
    actuated_dim_ = robot_state_->getActuatedDOF();
    nominal_actuated_q_ = Eigen::VectorXd::Zero(actuated_dim_);
    nominal_actuated_dq_ = Eigen::VectorXd::Zero(actuated_dim_);
    actuated_q_ = Eigen::VectorXd::Zero(actuated_dim_);
    actuated_dq_ = Eigen::VectorXd::Zero(actuated_dim_);
    A_ = Eigen::MatrixXd::Zero(task_dim_, var_dim_);
    A_.block(0, 6, actuated_dim_, actuated_dim_) = Eigen::MatrixXd::Identity(actuated_dim_, actuated_dim_);
    b_ = Eigen::VectorXd::Zero(task_dim_);
}

TaskLS_MinMotion::~TaskLS_MinMotion(){}

void TaskLS_MinMotion::updateVector()
{
    nominal_actuated_q_ = actuated_q_;
    nominal_actuated_dq_ = actuated_dq_;
    actuated_q_ = robot_state_->getGeneralizedCoordinates().tail(actuated_dim_);
    actuated_dq_ = robot_state_->getGeneralizedVelocities().tail(actuated_dim_);
    b_ = kp_*(nominal_actuated_q_ - actuated_q_) + kd_*(nominal_actuated_dq_ - actuated_dq_);
}

void TaskLS_MinMotion::updateMatrix()
{
}

