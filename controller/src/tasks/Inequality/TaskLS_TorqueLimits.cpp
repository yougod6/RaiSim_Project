#include "TaskLS_TorqueLimits.hpp"

TaskLS_TorqueLimits::TaskLS_TorqueLimits(Eigen::VectorXd tau_min, Eigen::VectorXd tau_max)
: tau_min_(tau_min), tau_max_(tau_max), TaskLS(tau_min.size()*2, tau_min.size()*2+6)
{
    if(tau_min_.size() != tau_max_.size()){
        std::cerr << "Torque limits size mismatch" << std::endl;
    }
    actuated_dim_ = tau_min_.size();
    
    task_name_ = "Torque Limits";
    task_dim_ = 2*actuated_dim_;
    var_dim_ = task_dim_+6;
    A_ = Eigen::MatrixXd::Zero(task_dim_, var_dim_);
    b_ = Eigen::VectorXd::Zero(task_dim_);

    A_.block(0, 6+actuated_dim_, actuated_dim_,actuated_dim_) = Eigen::MatrixXd::Identity(actuated_dim_,actuated_dim_);
    A_.block(actuated_dim_, 6+actuated_dim_, actuated_dim_,actuated_dim_) = -Eigen::MatrixXd::Identity(actuated_dim_,actuated_dim_);
    b_.head(actuated_dim_) = tau_max_;
    b_.tail(actuated_dim_) = -tau_min_;
}

TaskLS_TorqueLimits::~TaskLS_TorqueLimits(){}

void TaskLS_TorqueLimits::updateMatrix()
{
}

void TaskLS_TorqueLimits::updateVector()
{
}
