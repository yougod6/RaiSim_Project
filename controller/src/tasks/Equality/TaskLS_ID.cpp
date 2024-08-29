#include "TaskLS_ID.hpp"

TaskLS_ID::TaskLS_ID(raisim::World* world, raisim::ArticulatedSystem* robot, const int task_dim, const int var_dim)
: TaskLS(task_dim, var_dim), world_(world), robot_(robot)
{
    task_name_ = "Inverse Dynamics";
    contact_dim_ = 12;
    gravity_ = world_->getGravity();
    dof_ = robot_->getDOF();

    M_ = Eigen::MatrixXd::Zero(dof_, dof_);
    h_ = Eigen::VectorXd::Zero(dof_);

    J_c_FR_ = Eigen::MatrixXd::Zero(3, dof_);
    J_c_FL_ = Eigen::MatrixXd::Zero(3, dof_);
    J_c_RR_ = Eigen::MatrixXd::Zero(3, dof_);
    J_c_RL_ = Eigen::MatrixXd::Zero(3, dof_);
    J_c_ = Eigen::MatrixXd::Zero(contact_dim_, dof_);

    S_ = Eigen::MatrixXd::Zero(dof_-6, dof_);
    S_.block(0,6,dof_-6,dof_-6) = Eigen::MatrixXd::Identity(dof_-6,dof_-6);

    A_ = Eigen::MatrixXd::Zero(task_dim_,var_dim_);
    b_ = Eigen::VectorXd::Zero(task_dim_);
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
    J_c_.block(0, 0, 3, dof_) = J_c_FR_;
    J_c_.block(3, 0, 3, dof_) = J_c_FL_;
    J_c_.block(6, 0, 3, dof_) = J_c_RR_;
    J_c_.block(9, 0, 3, dof_) = J_c_RL_;
    QR_decomposition();
    Eigen::MatrixXd A_tmp = Eigen::MatrixXd::Zero(dof_, var_dim_);
    A_tmp.block(0, 0, dof_, dof_) = M_;
    A_tmp.block(0, dof_, dof_, dof_-6) = -S_.transpose();
    A_ = Qu_.transpose()*A_tmp;
}

void TaskLS_ID::updateVector(){
    b_ = -(Qu_.transpose()*(robot_->getNonlinearities(gravity_).e()));
}

void TaskLS_ID::QR_decomposition(){
    //QR Decomposition
    Eigen::MatrixXd J_c_T = J_c_.transpose();
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(J_c_T);
    Eigen::MatrixXd Q = qr.householderQ();
    Eigen::MatrixXd R = qr.matrixQR().triangularView<Eigen::Upper>();
    
    Sc_ = Eigen::MatrixXd::Zero(dof_, contact_dim_); //24x18
    Sc_.block(0,0,contact_dim_,contact_dim_) = Eigen::MatrixXd::Identity(contact_dim_,contact_dim_); //12x12
    
    Eigen::MatrixXd Su_= Eigen::MatrixXd::Zero(dof_, dof_-contact_dim_); //18x6
    Su_.block(contact_dim_,0,dof_-contact_dim_,dof_-contact_dim_) = Eigen::MatrixXd::Identity(dof_-contact_dim_,dof_-contact_dim_); //6x6  

    Qc_ = Q*Sc_;
    Qu_ = Q*Su_;
}