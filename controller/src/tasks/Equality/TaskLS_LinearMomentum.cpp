#include "TaskLS_LinearMomentum.hpp"

TaskLS_LinearMomentum::TaskLS_LinearMomentum(RobotState* robot_state, const int task_dim, const int var_dim, Eigen::Vector3d rate_weight, Eigen::Vector3d position_weight)
: TaskLS(task_dim, var_dim), robot_state_(robot_state)
{
    task_name_ = "Linear Momentum";
    rate_weight_ = rate_weight.asDiagonal();
    position_weight_ = position_weight.asDiagonal();    
    contact_dim_ = 12;
    g_ = robot_state_->getGravityVector();
    dof_ = robot_state_->getDOF();
    actuated_dof_ = robot_state_->getActuatedDOF();

    M_ = Eigen::MatrixXd::Zero(dof_,dof_);
    h_ = Eigen::VectorXd::Zero(dof_);

    J_c_FR_ = Eigen::MatrixXd::Zero(3,dof_);
    J_c_FL_ = Eigen::MatrixXd::Zero(3,dof_);
    J_c_RR_ = Eigen::MatrixXd::Zero(3,dof_);
    J_c_RL_ = Eigen::MatrixXd::Zero(3,dof_);
    J_c_ = Eigen::MatrixXd::Zero(12,dof_);

    S_ = Eigen::MatrixXd::Zero(actuated_dof_, dof_);
    P_ = Eigen::MatrixXd::Zero(3,contact_dim_);
    for(int i=0; i<(contact_dim_/3); i++){
        P_.block(0,3*i,3,3) = Eigen::MatrixXd::Identity(3,3);
    }
    CoM_mass_ = robot_state_->getTotalMass();
    desired_linear_momentum_rate_ = Eigen::VectorXd::Zero(3);
    desired_CoM_position_ = Eigen::VectorXd::Zero(3);
    desired_CoM_position_rate_ = Eigen::VectorXd::Zero(3);
    center_of_contacts_ = Eigen::VectorXd::Zero(3);
    center_of_contacts_ << -0.277321, 0., 0.625;
}

TaskLS_LinearMomentum::~TaskLS_LinearMomentum(){}


void TaskLS_LinearMomentum::updateMatrix(){
    M_ = robot_state_->getMassMatrix();
    J_c_ = robot_state_->getContactJacobian();
    
    QR_decomposition();

    A_tmp = Eigen::MatrixXd::Zero(dof_, var_dim_);
    A_tmp.block(0, 0, dof_, dof_) = M_;
    A_tmp.block(0, dof_, dof_, dof_-6) = -S_.transpose();
    A_ = P_*R_.inverse()*Qc_.transpose()*A_tmp;
}

void TaskLS_LinearMomentum::updateVector(){
    h_ = robot_state_->getNonlinearVector();
    g_ = robot_state_->getGravityVector();
    CoM_position_ = robot_state_->getCompositeCoM();
    CoM_position_rate_ = robot_state_->getCompositeCoMRate();
    updeateDesiredCoMPosition();
    desired_linear_momentum_rate_ = rate_weight_*(desired_CoM_position_rate_ - CoM_position_rate_) + position_weight_*(desired_CoM_position_ - CoM_position_);
    desired_linear_momentum_rate_ *= CoM_mass_;
    b_ = desired_linear_momentum_rate_ -(P_*R_.inverse()*Qc_.transpose()*h_ + CoM_mass_*g_);
    // std::cout << "desired_linear_momentum_rate_ : " << desired_linear_momentum_rate_.transpose() << std::endl;
}

void TaskLS_LinearMomentum::updeateDesiredCoMPosition(){
    contact_points_ = robot_state_->getContactPoints();
    contact_state_ = robot_state_->getContactState();

    if(contact_points_.size() != 4){
        return;
    }
    if(contact_state_[0] && contact_state_[1] && contact_state_[2] && contact_state_[3]){
        for(auto contact : contact_points_){
            center_of_contacts_ += contact;
        }
        center_of_contacts_ = center_of_contacts_/4;
    }
 
    desired_CoM_position_(0) = center_of_contacts_(0);
    desired_CoM_position_(1) = center_of_contacts_(1);
    desired_CoM_position_(2) = CoM_position_(2);
}

void TaskLS_LinearMomentum::QR_decomposition(){
    //QR Decomposition
    Eigen::MatrixXd J_c_T = J_c_.transpose();
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(J_c_T);
    Q_ = qr.householderQ();
    R_ = qr.matrixQR().triangularView<Eigen::Upper>();
    R_ = R_.block(0, 0, contact_dim_, contact_dim_);
    Qc_ = Q_.block(0, 0, dof_, 12);
}
