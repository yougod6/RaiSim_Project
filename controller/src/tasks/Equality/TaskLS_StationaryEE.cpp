#include "TaskLS_StationaryEE.hpp"

TaskLS_StationaryEE::TaskLS_StationaryEE(raisim::World* world, raisim::ArticulatedSystem* robot, const int task_dim, const int var_dim, Eigen::VectorXd desired_x, const double kp, const double kd)
: TaskLS(task_dim, var_dim), world_(world), robot_(robot), desired_x_(desired_x), kp_(kp), kd_(kd)
{
    task_name_ = "Stationary End-Effector";
    dof_ = robot_->getDOF();
    gravity_ = world_->getGravity();
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

void TaskLS_StationaryEE::update_dJ_EE(const double dt)
{
    Eigen::MatrixXd J_EE_tmp = J_EE_;

    update_J_EE();
    dJ_EE_ = (J_EE_ - J_EE_tmp)/dt;
}

void TaskLS_StationaryEE::update_J_EE()
{
    robot_->getDenseFrameJacobian("joint6", J_EE_position);
    robot_->getDenseFrameRotationalJacobian("joint6", J_EE_rotation);
    J_EE_.block(0, 0, 3, dof_) = J_EE_position;
    J_EE_.block(3, 0, 3, dof_) = J_EE_rotation;
}

void TaskLS_StationaryEE::updateMatrix()
{
    update_dJ_EE(world_->getTimeStep());
    if(task_dim_==3){
        A_.block(0,0,task_dim_,dof_) = J_EE_.block(0,0,3,dof_);
    }
    else{
        A_.block(0, 0, task_dim_, dof_) = J_EE_;
    }
}

void TaskLS_StationaryEE::updateVector()
{
    // makeEETrajectory(world_->getWorldTime());
    updateDesiredBaseAcceleration();
    if(task_dim_==3){
        b_ = desired_xddot_.head(3) - dJ_EE_.block(0,0,3,dof_)*robot_->getGeneralizedVelocity().e();
    }
    else{
        b_ = desired_xddot_ - dJ_EE_*robot_->getGeneralizedVelocity().e();
    }
}

// void TaskLS_StationaryEE::makeEETrajectory(double time)
// {
//     desired_x_ << 0.15, 0.0, 0.8,
//                 0.0, 0.122025, 0.0;
// }

void TaskLS_StationaryEE::updateDesiredEEPose(Eigen::VectorXd desired_x){
    desired_x_ = desired_x;
}


void TaskLS_StationaryEE::updateDesiredBaseAcceleration()
{
    Eigen::VectorXd x = Eigen::VectorXd::Zero(6);
    raisim::Vec<3> ee_position = Eigen::VectorXd::Zero(3);
    raisim::Mat<3,3> wRee;
    robot_->getFramePosition("joint6",ee_position);
    robot_->getFrameOrientation("joint6",wRee);
    Eigen::Quaterniond ee_quat = Eigen::Quaterniond(wRee.e());
    Eigen::Vector3d ee_euler = Utils::quat_to_euler(ee_quat);
    x.head(3) = ee_position.e();
    x.tail(3) = ee_euler;

    Eigen::VectorXd xdot = Eigen::VectorXd::Zero(6);
    raisim::Vec<3> ee_velocity;
    raisim::Vec<3> ee_angular_velocity;
    robot_->getFrameVelocity("joint6",ee_velocity);
    robot_->getFrameAngularVelocity("joint6",ee_angular_velocity);
    xdot.head(3) = ee_velocity.e();
    xdot.tail(3) = ee_angular_velocity.e();
    Eigen::VectorXd kp_vec = kp_*Eigen::VectorXd::Ones(6);
    Eigen::VectorXd kd_vec = kd_*Eigen::VectorXd::Ones(6);
    kp_vec.tail(3) = 100*Eigen::Vector3d::Ones();
    kd_vec.tail(3) = 2*sqrt(100)*Eigen::Vector3d::Ones();
    desired_xddot_ = kp_*(desired_x_ - x) - kd_*xdot;
}

