#include "TaskLS_StationaryEE.hpp"

TaskLS_StationaryEE::TaskLS_StationaryEE(RobotState* robot_state, const int task_dim, const int var_dim, Eigen::VectorXd desired_x, const double kp, const double kd)
: TaskLS(task_dim, var_dim), robot_state_(robot_state), desired_x_(desired_x), kp_(kp), kd_(kd)
{
    task_name_ = "Stationary End-Effector";
    dof_ = robot_state_->getDOF();
    J_EE_position = Eigen::MatrixXd::Zero(3, dof_);
    J_EE_rotation = Eigen::MatrixXd::Zero(3, dof_);
    J_EE_ = Eigen::MatrixXd::Zero(6, dof_);
    dJ_EE_ = Eigen::MatrixXd::Zero(6, dof_);
    dq_ = Eigen::VectorXd::Zero(dof_);

    desired_xdot_ = Eigen::VectorXd::Zero(6);
    desired_xddot_ = Eigen::VectorXd::Zero(6);

    A_ = Eigen::MatrixXd::Zero(task_dim_, var_dim_);
    b_ = Eigen::VectorXd::Zero(var_dim_);

}

TaskLS_StationaryEE::~TaskLS_StationaryEE(){}


void TaskLS_StationaryEE::updateMatrix()
{
    J_EE_ = robot_state_->getEEJacobian();
    if(task_dim_==3){
        A_.block(0,0,task_dim_,dof_) = J_EE_.block(0,0,3,dof_);
    }
    else{
        A_.block(0, 0, task_dim_, dof_) = J_EE_;
    }
}

void TaskLS_StationaryEE::updateVector()
{
    dJ_EE_ = robot_state_->getEEJacobianRate();
    updateDesiredBaseAcceleration();
    if(task_dim_==3){
        b_ = desired_xddot_.head(3) - dJ_EE_.block(0,0,3,dof_)*robot_state_->getGeneralizedVelocities();
    }
    else{
        b_ = desired_xddot_ - dJ_EE_*robot_state_->getGeneralizedVelocities();
    }
}

void TaskLS_StationaryEE::updateDesiredEEPose(Eigen::VectorXd desired_x){
    desired_x_ = desired_x;
}


void TaskLS_StationaryEE::updateDesiredBaseAcceleration()
{
    Eigen::VectorXd x = Eigen::VectorXd::Zero(6);
    ee_position_ = robot_state_->getEEPosition();   
    ee_quaternion_ = robot_state_->getEEQuaternion();
    ee_euler_ = Utils::quat_to_euler(ee_quaternion_);
    x.head(3) = ee_position_;
    x.tail(3) = ee_euler_;
    Eigen::VectorXd x_err = Eigen::VectorXd::Zero(6);
    x_err.head(3) = desired_x_.head(3) - x.head(3);
    Eigen::Quaterniond desired_ee_quat(desired_x_[3], desired_x_[4], desired_x_[5], desired_x_[6]);
    x_err.tail(3) = Utils::box_minus_operator(desired_ee_quat,ee_quaternion_);
    Eigen::VectorXd xdot = Eigen::VectorXd::Zero(6);
    xdot.head(3) = robot_state_->getEELinearVelocity();
    xdot.tail(3) = robot_state_->getEEAngularVelocity();
    Eigen::VectorXd kp_vec = kp_*Eigen::VectorXd::Ones(6);
    Eigen::VectorXd kd_vec = kd_*Eigen::VectorXd::Ones(6);
    kp_vec.tail(3) = 100*Eigen::Vector3d::Ones();
    kd_vec.tail(3) = 2*sqrt(100)*Eigen::Vector3d::Ones();
    desired_xddot_ = kp_*(x_err) - kd_*xdot;
}

