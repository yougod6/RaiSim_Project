#include "TaskLS_ID.hpp"

TaskLS_ID::TaskLS_ID(raisim::World* world, raisim::ArticulatedSystem* robot)
{
    task_name_ = "Iverse Dynamics";
    world_ = world;
    robot_ = robot;
    gravity_ = world_->getGravity();
    M_ = Eigen::MatrixXd::Zero(robot_->getDOF(), robot_->getDOF());
    h_ = Eigen::VectorXd::Zero(robot_->getDOF());

    J_c_FR_ = Eigen::MatrixXd::Zero(3, robot_->getDOF());
    J_c_FL_ = Eigen::MatrixXd::Zero(3, robot_->getDOF());
    J_c_RR_ = Eigen::MatrixXd::Zero(3, robot_->getDOF());
    J_c_RL_ = Eigen::MatrixXd::Zero(3, robot_->getDOF());
    J_c_ = Eigen::MatrixXd::Zero(12, robot_->getDOF());

    S_ = Eigen::MatrixXd::Zero(robot_->getDOF()-6, robot_->getDOF()); //12x18
    S_.block(0,6,12,12) = Eigen::MatrixXd::Identity(robot_->getDOF()-6,robot_->getDOF()-6);

    A_ = Eigen::MatrixXd::Zero(6,30);
    b_ = Eigen::VectorXd::Zero(6);
}

TaskLS_ID::~TaskLS_ID(){}

void TaskLS_ID::updateMatrix(){
    // std::cout << "ID Matrix PreUpdated" << std::endl;

    M_ = robot_->getMassMatrix().e();
    
    robot_->getDenseFrameJacobian("FR_foot_fixed", J_c_FR_); // 3x18
    robot_->getDenseFrameJacobian("FL_foot_fixed", J_c_FL_); // 3x18
    robot_->getDenseFrameJacobian("RR_foot_fixed", J_c_RR_); // 3x18
    robot_->getDenseFrameJacobian("RL_foot_fixed", J_c_RL_); // 3x18
    // std::cout << "set J_c_" << std::endl;
    // J_c_.block(0, 0, 6, robot_->getDOF()) = J_c_FR_;
    J_c_.block(0, 0, 3, robot_->getDOF()) = J_c_FR_;
    J_c_.block(3, 0, 3, robot_->getDOF()) = J_c_FL_;
    J_c_.block(6, 0, 3, robot_->getDOF()) = J_c_RR_;
    J_c_.block(9, 0, 3, robot_->getDOF()) = J_c_RL_;
    QR_decomposition();
    Eigen::MatrixXd A_tmp = Eigen::MatrixXd::Zero(18, 30);
    A_tmp.block(0, 0, 18, 18) = - M_;
    A_tmp.block(0, 18, 18, 12) = S_.transpose();
    A_ = Qu_.transpose()*A_tmp;
}

void TaskLS_ID::updateVector(){
    b_ = (Qu_.transpose()*(robot_->getNonlinearities(gravity_).e()));
    // std::cout << "ID Vector Updated" << std::endl;
}

void TaskLS_ID::QR_decomposition(){
    //QR Decomposition
    Eigen::MatrixXd J_c_T = J_c_.transpose();
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(J_c_T);
    Eigen::MatrixXd Q = qr.householderQ();
    Eigen::MatrixXd R = qr.matrixQR().triangularView<Eigen::Upper>();
    
    Sc_ = Eigen::MatrixXd::Zero(robot_->getDOF(), robot_->getDOF()-6); //18x12
    Sc_.block(0,0,12,12) = Eigen::MatrixXd::Identity(robot_->getDOF()-6,robot_->getDOF()-6); //12x12
    
    Eigen::MatrixXd Su_= Eigen::MatrixXd::Zero(robot_->getDOF(), 6); //18x6
    Su_.block(12,0,6,6) = Eigen::MatrixXd::Identity(6,6); //6x6  

    Qc_ = Q*Sc_;
    Qu_ = Q*Su_;
}