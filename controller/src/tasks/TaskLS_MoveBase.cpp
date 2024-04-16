#include "TaskLS_MoveBase.hpp"
#define _USE_MATH_DEFINES 


TaskLS_MoveBase::TaskLS_MoveBase(raisim::World* world, raisim::ArticulatedSystem* robot, const double kp, const double kd)
{
    task_name_ = "Move Base";
    task_dim_ = 6;
    var_dim_ = 30; // 18(qddot) + 12(torque)
    world_ = world;
    robot_ = robot;
    gravity_ = world_->getGravity();
    kp_ = kp;
    kd_ = kd;

    J_B_position = Eigen::MatrixXd::Zero(3, robot_->getDOF());
    J_B_rotation = Eigen::MatrixXd::Zero(3, robot_->getDOF());
    J_B_ = Eigen::MatrixXd::Zero(6, robot_->getDOF());
    // J_B_.block(0,0,6,6) = Eigen::MatrixXd::Identity(6,6);

    dJ_B_ = Eigen::MatrixXd::Zero(6, robot_->getDOF());

    dq_ = Eigen::VectorXd::Zero(robot_->getDOF());
   
    desired_x_ = Eigen::VectorXd::Zero(6);
    desired_xdot_ = Eigen::VectorXd::Zero(6);
    desired_xddot_ = Eigen::VectorXd::Zero(6);

    A_ = Eigen::MatrixXd::Zero(task_dim_,var_dim_);
    b_ = Eigen::VectorXd::Zero(var_dim_);

    A_.block(0,0,6,18) = J_B_; 
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
    J_B_.block(0, 0, 3, robot_->getDOF()) = J_B_position;
    J_B_.block(3, 0, 3, robot_->getDOF()) = J_B_rotation;
}

void TaskLS_MoveBase::updateMatrix()
{
    // std::cout << "Move Base Matrix PreUpdated" << std::endl;
    update_dJ_B(world_->getTimeStep());
    
    A_.block(0, 0, 6, 18) = J_B_;
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
    const double freq = 0.5;

   desired_x_ << -0.00270028 ,
                    0.000455424+ amplitude*sin(2*M_PI*freq*time) ,
                    0.35,
                    0,0,0;
}

void TaskLS_MoveBase::updateDesiredBaseAcceleration(){
    Eigen::VectorXd q_B = Eigen::VectorXd::Zero(6);
    raisim::Vec<3> base_position;
    raisim::Vec<4> base_quat;
    robot_->getBasePosition(base_position);
    robot_->getBaseOrientation(base_quat);
    q_B.head(3) = base_position.e();
    q_B.tail(3) = base_quat.e().tail(3);

    desired_xddot_ = kp_*(desired_x_ - q_B) - kd_*(robot_->getGeneralizedVelocity().e().head(6)); 
}
