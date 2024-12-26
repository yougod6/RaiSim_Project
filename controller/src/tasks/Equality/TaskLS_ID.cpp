#include "TaskLS_ID.hpp"

TaskLS_ID::TaskLS_ID(RobotState* robot_state, const int task_dim, const int var_dim)
: TaskLS(task_dim, var_dim), robot_state_(robot_state)
{
    task_name_ = "Inverse Dynamics";
    dof_ = robot_state_->getDOF();
    actuated_dof_ = robot_state_->getActuatedDOF();
    contact_dim_ = robot_state_->getContactDim();
    M_ = Eigen::MatrixXd::Zero(dof_, dof_);
    h_ = Eigen::VectorXd::Zero(dof_);

    J_c_FR_ = Eigen::MatrixXd::Zero(3, dof_);
    J_c_FL_ = Eigen::MatrixXd::Zero(3, dof_);
    J_c_RR_ = Eigen::MatrixXd::Zero(3, dof_);
    J_c_RL_ = Eigen::MatrixXd::Zero(3, dof_);
    J_c_ = Eigen::MatrixXd::Zero(contact_dim_, dof_);

    S_ = Eigen::MatrixXd::Zero(actuated_dof_, dof_);
    S_.block(0,6,actuated_dof_,actuated_dof_) = Eigen::MatrixXd::Identity(actuated_dof_,actuated_dof_);

    A_ = Eigen::MatrixXd::Zero(task_dim_,var_dim_);
    b_ = Eigen::VectorXd::Zero(task_dim_);
}

TaskLS_ID::~TaskLS_ID(){}

void TaskLS_ID::updateMatrix(){
    // std::cout << "ID Matrix PreUpdated" << std::endl;

    M_ = robot_state_->getMassMatrix();
    J_c_ = robot_state_->getContactJacobian();
    QR_decomposition();
    Eigen::MatrixXd A_tmp = Eigen::MatrixXd::Zero(dof_, var_dim_);
    A_tmp.block(0, 0, dof_, dof_) = M_;
    A_tmp.block(0, dof_, dof_, dof_-6) = -S_.transpose();
    A_ = Qu_.transpose()*A_tmp;
}

void TaskLS_ID::updateVector(){
    b_ = -(Qu_.transpose()*(robot_state_->getNonlinearVector()));
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