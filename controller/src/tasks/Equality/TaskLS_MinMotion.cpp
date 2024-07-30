#include "TaskLS_MinMotion.hpp"

TaskLS_MinMotion::TaskLS_MinMotion(raisim::World* world, raisim::ArticulatedSystem* robot, const int task_dim, const int var_dim, const double kp, const double kd)
{
    task_name_ = "Minimum Motion";
    task_dim_ = task_dim; // number of actuated joints 
    var_dim_ = var_dim;
    world_ = world;
    robot_ = robot;
    dof_ = robot_->getDOF();
    actuated_dim_ = dof_ - 6;
    gravity_ = world_->getGravity();
    kp_ = kp;
    kd_ = kd;
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
    actuated_q_ = robot_->getGeneralizedCoordinate().e().tail(actuated_dim_);
    actuated_dq_ = robot_->getGeneralizedVelocity().e().tail(actuated_dim_);
    b_ = kp_*(nominal_actuated_q_ - actuated_q_) + kd_*(nominal_actuated_dq_ - actuated_dq_);
}

void TaskLS_MinMotion::updateMatrix()
{
}

