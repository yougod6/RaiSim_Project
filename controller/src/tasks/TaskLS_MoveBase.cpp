#include "TaskLS_MoveBase.hpp"
#define _USE_MATH_DEFINES 


TaskLS_MoveBase::TaskLS_MoveBase(raisim::World* world, raisim::ArticulatedSystem* robot, const double kp, const double kd)
{
    task_name_ = "Move Base";
    world_ = world;
    robot_ = robot;
    gravity_ = world_->getGravity();
    kp_ = kp;
    kd_ = kd;

    J_B_position = Eigen::MatrixXd::Zero(3, robot_->getDOF());
    J_B_rotation = Eigen::MatrixXd::Zero(3, robot_->getDOF());
    J_B_ = Eigen::MatrixXd::Zero(6, robot_->getDOF());
    J_B_.block(0,0,6,6) = Eigen::MatrixXd::Identity(6,6);

    dJ_B_ = Eigen::MatrixXd::Zero(6, robot_->getDOF());

    dq_ = Eigen::VectorXd::Zero(robot_->getDOF());
   
    desired_q_B_ = Eigen::VectorXd::Zero(6);
    desired_dq_B_ = Eigen::VectorXd::Zero(6);
    desired_ddq_B_ = Eigen::VectorXd::Zero(6);

    A_ = Eigen::MatrixXd::Zero(6,30);
    b_ = Eigen::VectorXd::Zero(6);

    A_.block(0, 0, 6, 18) = J_B_; 
}

TaskLS_MoveBase::~TaskLS_MoveBase(){}

void TaskLS_MoveBase::update_dJ_B(const double dt)
{
    // Eigen::MatrixXd J_B_tmp = J_B_;
    
    // update_J_B();
    // dJ_B_ = (J_B_ - J_B_tmp)/dt;
}

void TaskLS_MoveBase::update_J_B()
{
    // robot_->getDenseFrameJacobian("floating_base", J_B_position);
    // robot_->getDenseFrameRotationalJacobian("floating_base", J_B_rotation);
    // J_B_.block(0, 0, 3, robot_->getDOF()) = J_B_position;
    // J_B_.block(3, 0, 3, robot_->getDOF()) = J_B_rotation;
}

void TaskLS_MoveBase::updateMatrix()
{
    // std::cout << "Move Base Matrix PreUpdated" << std::endl;
    // update_J_B();
    
    // A_.block(0, 0, 6, 18) = J_B_;
    // std::cout << "Move Base Matrix Updated" << std::endl;

}

void TaskLS_MoveBase::updateVector()
{
    // std::cout << "Move Base Vector PreUpdated" << std::endl;
    //w_c_star = 0;
    /**
     * world_->getTimeStep() 은 dt 받아오는 메소드
     * 현재 시간을 받아오는 메소드 필요 
    */
    double time = world_->getWorldTime();
    makeBaseTrajectory(time);
    // std::cout << "Base Trajectory Made" << std::endl;
    updateDesiredBaseAcceleration(kp_, kd_);
    // std::cout << "Base Acceleration Updated" << std::endl;
    // update_dJ_B();
    // std::cout << "dJ_B Updated" << std::endl;
    b_ = desired_ddq_B_ - dJ_B_*robot_->getGeneralizedVelocity().e();
    // std::cout << "Move Base Vector Updated" << std::endl;
}

void TaskLS_MoveBase::makeBaseTrajectory(double time){
    // Defines circular trajectory parameters in xz plane
    Eigen::VectorXd BaseLine = Eigen::VectorXd::Zero(6);
    BaseLine << 0.0, 0.0, 0.45, 0, 0, 0;
    const double amplitude = 0.5; //0.15
    const double freq = 1.0;

    // Base X-Z references
    desired_q_B_ << amplitude*sin(2*M_PI*freq*time),
                    0.0,
                    amplitude*cos(2*M_PI*freq*time),
                    0.0,
                    0.0,
                    0.0;
    desired_q_B_ += BaseLine;

    // Base X-Z velocities
    desired_dq_B_ <<amplitude*freq*cos(2*M_PI*freq*time),
                    0.0,
                    -amplitude*freq*sin(2*M_PI*freq*time),
                    0.0,
                    0.0,
                    0.0;
}

void TaskLS_MoveBase::updateDesiredBaseAcceleration(const double kp, const double kd){
    Eigen::VectorXd q_B = Eigen::VectorXd::Zero(6);
    q_B.block(0,0,3,1) = robot_->getBasePosition().e();
    
    Eigen::Vector4d q_B_vec = robot_->getGeneralizedCoordinate().e().block(3,0,4,1);
    Eigen::Quaterniond q_B_quat;
    q_B_quat.x() = q_B_vec(0);
    q_B_quat.y() = q_B_vec(1);
    q_B_quat.z() = q_B_vec(2);
    q_B_quat.w() = q_B_vec(3);

    q_B.block(3,0,3,1) = robot_->getBaseOrientation().e().eulerAngles(0,1,2);
    desired_ddq_B_ = kp_*(desired_q_B_ - q_B) + kd_*(desired_dq_B_ - robot_->getGeneralizedVelocity().e().head(6));
    // std::cout << "desired_ddq_B_ : " << desired_ddq_B_.transpose() << std::endl;
}
