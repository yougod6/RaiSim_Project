#include "TaskLS_StationaryFeet.hpp"

TaskLS_StationaryFeet::TaskLS_StationaryFeet(raisim::World* world, raisim::ArticulatedSystem* robot, const int task_dim, const int var_dim)
: TaskLS(task_dim, var_dim), world_(world), robot_(robot)
{
    task_name_ = "Stationary Feet";
    dof_ = robot->getDOF();
    gravity_ = world_->getGravity();

    J_c_FR_ = Eigen::MatrixXd::Zero(3, dof_);
    J_c_FL_ = Eigen::MatrixXd::Zero(3, dof_);
    J_c_RR_ = Eigen::MatrixXd::Zero(3, dof_);
    J_c_RL_ = Eigen::MatrixXd::Zero(3, dof_);
    J_c_ = Eigen::MatrixXd::Zero(task_dim_, dof_);
    dJ_c_ = Eigen::MatrixXd::Zero(task_dim_, dof_);
    dq_ = Eigen::VectorXd::Zero(dof_);
    

    A_ = Eigen::MatrixXd::Zero(task_dim_,var_dim_);
    b_ = Eigen::VectorXd::Zero(task_dim_);

}

TaskLS_StationaryFeet::~TaskLS_StationaryFeet(){}

void TaskLS_StationaryFeet::update_J_c()
{
    robot_->getDenseFrameJacobian("FR_foot_fixed", J_c_FR_); // 3x18
    robot_->getDenseFrameJacobian("FL_foot_fixed", J_c_FL_); // 3x18
    robot_->getDenseFrameJacobian("RR_foot_fixed", J_c_RR_); // 3x18
    robot_->getDenseFrameJacobian("RL_foot_fixed", J_c_RL_); // 3x18

    J_c_.block(0, 0, 3, dof_) = J_c_FR_;
    J_c_.block(3, 0, 3, dof_) = J_c_FL_;
    J_c_.block(6, 0, 3, dof_) = J_c_RR_;
    J_c_.block(9, 0, 3, dof_) = J_c_RL_;
}

void TaskLS_StationaryFeet::update_dJ_c(const double dt)
{
    Eigen::MatrixXd J_c_tmp = J_c_;
    
    update_J_c();
    dJ_c_ = (J_c_ - J_c_tmp)/dt;
}

void TaskLS_StationaryFeet::updateMatrix()
{
    // std::cout << "Stationary Feet Matrix PreUpdated" << std::endl;
    // update_J_c();
    update_dJ_c(world_->getTimeStep());
   
    A_.block(0,0,task_dim_,dof_) = J_c_;

    // std::cout << "Stationary Feet Matrix Updated" << std::endl;
}

void TaskLS_StationaryFeet::updateVector()
{
    //w_c_star = 0;
    b_ = - dJ_c_*robot_->getGeneralizedVelocity().e();
    // std::cout << "Stationary Feet Vector Updated" << std::endl;
}


