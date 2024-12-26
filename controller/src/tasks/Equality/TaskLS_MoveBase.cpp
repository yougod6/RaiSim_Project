#include "TaskLS_MoveBase.hpp"
#define _USE_MATH_DEFINES 


TaskLS_MoveBase::TaskLS_MoveBase(RobotState* robot_state, const int task_dim, const int var_dim, Eigen::VectorXd desired_x, const double kp, const double kd)
: TaskLS(task_dim, var_dim), robot_state_(robot_state), desired_x_(desired_x) ,  kp_(kp), kd_(kd)
{
    task_name_ = "Move Base";
    dof_ = robot_state_->getDOF();

    J_B_ = Eigen::MatrixXd::Zero(6, dof_);

    dJ_B_ = Eigen::MatrixXd::Zero(6, dof_);

    dq_ = Eigen::VectorXd::Zero(dof_);
   
    desired_xdot_ = Eigen::VectorXd::Zero(6);
    desired_xddot_ = Eigen::VectorXd::Zero(6);

    A_ = Eigen::MatrixXd::Zero(task_dim_,var_dim_);
    b_ = Eigen::VectorXd::Zero(var_dim_);

}

TaskLS_MoveBase::~TaskLS_MoveBase(){}

void TaskLS_MoveBase::updateMatrix()
{
    J_B_ = robot_state_->getBaseJacobian();

    // z position and orientation
    if(task_dim_==4){
        A_.block(0,0,task_dim_,dof_) = J_B_.block(2,0,4,dof_);  
    }
    else{
        A_.block(0, 0, task_dim_, dof_) = J_B_;
    }

}

void TaskLS_MoveBase::updateVector()
{
    dJ_B_ = robot_state_->getBaseJacobianRate();
    updateDesiredBaseAcceleration();
    if(task_dim_==4){
        b_ = desired_xddot_.tail(4) - dJ_B_.block(2,0,4,dof_)*robot_state_->getGeneralizedVelocities();
    }
    else{
        b_ = desired_xddot_ - dJ_B_*robot_state_->getGeneralizedVelocities();
    }
}

void TaskLS_MoveBase::updateDesiredBasePose(Eigen::VectorXd desired_x){
    desired_x_ = desired_x;
}

void TaskLS_MoveBase::updateDesiredBaseAcceleration(){
    Eigen::VectorXd q_B = Eigen::VectorXd::Zero(6);
    base_position_ = robot_state_->getBasePosition();
    base_quaternion_ = robot_state_->getBaseQuaternion();
    Eigen::Vector3d base_euler = Utils::quat_to_euler(base_quaternion_);
    q_B.head(3) = base_position_;
    q_B.tail(3) = base_euler;
    desired_xddot_ = kp_*(desired_x_ - q_B) - kd_*(robot_state_->getGeneralizedVelocities().head(6)); 
}