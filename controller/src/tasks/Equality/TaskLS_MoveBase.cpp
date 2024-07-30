#include "TaskLS_MoveBase.hpp"
#define _USE_MATH_DEFINES 


TaskLS_MoveBase::TaskLS_MoveBase(raisim::World* world, raisim::ArticulatedSystem* robot, const int task_dim, const int var_dim, const double kp, const double kd)
{
    task_name_ = "Move Base";
    task_dim_ = task_dim;
    var_dim_ = var_dim; // 18(qddot) + 12(torque)
    world_ = world;
    robot_ = robot;
    dof_ = robot->getDOF();
    gravity_ = world_->getGravity();
    kp_ = kp;
    kd_ = kd;

    J_B_position = Eigen::MatrixXd::Zero(3, dof_);
    J_B_rotation = Eigen::MatrixXd::Zero(3, dof_);
    J_B_ = Eigen::MatrixXd::Zero(task_dim_, dof_);
    // J_B_.block(0,0,6,6) = Eigen::MatrixXd::Identity(6,6);

    dJ_B_ = Eigen::MatrixXd::Zero(task_dim_, dof_);

    dq_ = Eigen::VectorXd::Zero(dof_);
   
    desired_x_ = Eigen::VectorXd::Zero(task_dim_);
    desired_xdot_ = Eigen::VectorXd::Zero(task_dim_);
    desired_xddot_ = Eigen::VectorXd::Zero(task_dim_);

    A_ = Eigen::MatrixXd::Zero(task_dim_,var_dim_);
    b_ = Eigen::VectorXd::Zero(var_dim_);

    A_.block(0,0,task_dim_,dof_) = J_B_; 
}

TaskLS_MoveBase::~TaskLS_MoveBase(){}

void TaskLS_MoveBase::update_dJ_B(const double dt)
{
    Eigen::MatrixXd J_B_tmp = J_B_;
    
    update_J_B();
    dJ_B_ = (J_B_ - J_B_tmp)/dt;
}

void TaskLS_MoveBase::update_J_B()
{
    robot_->getDenseFrameJacobian("floating_base", J_B_position);
    robot_->getDenseFrameRotationalJacobian("floating_base", J_B_rotation);
    J_B_.block(0, 0, 3, dof_) = J_B_position;
    J_B_.block(3, 0, 3, dof_) = J_B_rotation;
}

void TaskLS_MoveBase::updateMatrix()
{
    // std::cout << "Move Base Matrix PreUpdated" << std::endl;
    update_dJ_B(world_->getTimeStep());
    
    A_.block(0, 0, task_dim_, dof_) = J_B_;
    // std::cout << "Move Base Matrix Updated" << std::endl;

}

void TaskLS_MoveBase::updateVector()
{
    // std::cout << "Move Base Vector PreUpdated" << std::endl;
   
    double time = world_->getWorldTime();
    makeBaseTrajectory(world_->getWorldTime());
    // std::cout << "Base Trajectory Made" << std::endl;
    updateDesiredBaseAcceleration();
    b_ = desired_xddot_ - dJ_B_*robot_->getGeneralizedVelocity().e();
    // std::cout << "Move Base Vector Updated" << std::endl;
}

void TaskLS_MoveBase::makeBaseTrajectory(double time){
    // Defines circular trajectory parameters in xz plane
    const double amplitude = 0.05; //0.15
    const double freq = 0.2;

   desired_x_ << -0.00250028 ,
                    0.000455424+ amplitude*sin(2*M_PI*freq*time) ,
                    0.34,
                    0,0,0;
    // desired_x_ << -0.0490382 ,
    //                0.00157048 + amplitude*sin(2*M_PI*freq*time) ,
    //                 0.401518,
    //                 0,0,0;
}

void TaskLS_MoveBase::updateDesiredBaseAcceleration(){
    Eigen::VectorXd q_B = Eigen::VectorXd::Zero(6);
    raisim::Vec<3> base_position;
    raisim::Vec<4> base_quat;
    robot_->getBasePosition(base_position);
    robot_->getBaseOrientation(base_quat);
    Eigen::Quaterniond base_quat_eigen(base_quat[0], base_quat[1], base_quat[2], base_quat[3]);
    Eigen::Vector3d base_euler = Utils::quat_to_euler(base_quat_eigen);
    q_B.head(3) = base_position.e();
    q_B.tail(3) = base_euler;
    desired_xddot_ = kp_*(desired_x_ - q_B) - kd_*(robot_->getGeneralizedVelocity().e().head(6)); 
}
