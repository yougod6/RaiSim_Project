#include "TaskLS_AngularMomentum.hpp"

TaskLS_AngularMomentum::TaskLS_AngularMomentum(RobotState* robot_state, const int task_dim, const int var_dim, Eigen::Vector3d weight)
: TaskLS(task_dim, var_dim), robot_state_(robot_state)
{
    task_name_ = "Angular Momentum";
    dof_ = robot_state_->getDOF();
    actuated_dof_ = robot_state_->getActuatedDOF();
    contact_dim_ = robot_state_->getContactDim();
    weight_ = weight.asDiagonal();
    contact_dim_ = 12;

    M_ = Eigen::MatrixXd::Zero(dof_,dof_);
    h_ = Eigen::VectorXd::Zero(dof_);

    r_c_FR_ << 0.206574, -0.132344,  0.254512;
    r_c_FL_ << 0.20622, 0.132171,  0.25225;
    r_c_RR_ << -0.252118, -0.134013,  0.342678;
    r_c_RL_ << -0.252122,  0.134047,  0.342684;

    J_c_FR_ = Eigen::MatrixXd::Zero(3,dof_);
    J_c_FL_ = Eigen::MatrixXd::Zero(3,dof_);
    J_c_RR_ = Eigen::MatrixXd::Zero(3,dof_);
    J_c_RL_ = Eigen::MatrixXd::Zero(3,dof_);
    J_c_ = Eigen::MatrixXd::Zero(12,dof_);

    S_ = Eigen::MatrixXd::Zero(actuated_dof_, dof_);
    D_ = Eigen::MatrixXd::Zero(3,contact_dim_);

    CoM_mass_= robot_state_->getTotalMass();
    desired_angular_momentum_ = Eigen::VectorXd::Zero(3);
    desired_angular_momentum_rate_ = Eigen::VectorXd::Zero(3);
    angular_momentum_ = Eigen::VectorXd::Zero(3);
}

TaskLS_AngularMomentum::~TaskLS_AngularMomentum(){}

void TaskLS_AngularMomentum::updateMatrix(){
    M_ = robot_state_->getMassMatrix();
    J_c_ = robot_state_->getContactJacobian();

    QR_decomposition();
  
    A_tmp_ = Eigen::MatrixXd::Zero(dof_, var_dim_);
    A_tmp_.block(0, 0, dof_, dof_) = M_;
    A_tmp_.block(0, dof_, dof_, actuated_dof_) = -S_.transpose();
    CoM_position_ = robot_state_->getCompositeCoM();
    contact_points_ = robot_state_->getContactPoints();
    if(contact_points_.size() == 4){
        r_c_FR_ = contact_points_[0];
        r_c_FL_ = contact_points_[1];
        r_c_RR_ = contact_points_[2];
        r_c_RL_ = contact_points_[3];
        D_.block(0,0,3,3) = Utils::skew(r_c_FR_ - CoM_position_);
        D_.block(0,3,3,3) = Utils::skew(r_c_FL_ - CoM_position_);
        D_.block(0,6,3,3) = Utils::skew(r_c_RR_ - CoM_position_);
        D_.block(0,9,3,3) = Utils::skew(r_c_RL_ - CoM_position_);
    }
    A_ = D_*R_.inverse()*Qc_.transpose()*A_tmp_;
}

void TaskLS_AngularMomentum::updateVector(){
    h_ = robot_state_->getNonlinearVector();
    b_ = weight_*(desired_angular_momentum_ - angular_momentum_) - D_*R_.inverse()*Qc_.transpose()*h_;
}

void TaskLS_AngularMomentum::updateAngularMomentum(){
    angular_momentum_ = robot_state_->getAngularMomentum();
}

void TaskLS_AngularMomentum::QR_decomposition(){
    //QR Decomposition
    Eigen::MatrixXd J_c_T = J_c_.transpose();
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(J_c_T);
    Q_ = qr.householderQ();
    R_ = qr.matrixQR().triangularView<Eigen::Upper>();
    R_ = R_.block(0, 0, contact_dim_, contact_dim_);
    Qc_ = Q_.block(0, 0, dof_, 12);
}