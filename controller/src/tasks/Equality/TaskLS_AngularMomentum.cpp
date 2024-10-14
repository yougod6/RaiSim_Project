#include "TaskLS_AngularMomentum.hpp"

TaskLS_AngularMomentum::TaskLS_AngularMomentum(raisim::World* world, raisim::ArticulatedSystem* robot, const int task_dim, const int var_dim, Eigen::Vector3d weight)
: TaskLS(task_dim, var_dim), world_(world), robot_(robot)
{
    task_name_ = "Angular Momentum";
    weight_ = weight.asDiagonal();
    contact_dim_ = 12;
    gravity_ = world_->getGravity();
    dof_ = robot_->getDOF();

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

    S_ = Eigen::MatrixXd::Zero(dof_-6, dof_);
    D_ = Eigen::MatrixXd::Zero(3,contact_dim_);

    CoM_mass_ = 0;
    auto mass_list = robot_->getMass();
    for(auto mass : mass_list){
        CoM_mass_ += mass;
    }     
    desired_angular_momentum_ = Eigen::VectorXd::Zero(3);
    desired_angular_momentum_rate_ = Eigen::VectorXd::Zero(3);
    angular_momentum_ = Eigen::VectorXd::Zero(3);
}

TaskLS_AngularMomentum::~TaskLS_AngularMomentum(){}

void TaskLS_AngularMomentum::updateMatrix(){
    M_ = robot_->getMassMatrix().e();
    robot_->getDenseFrameJacobian("FR_foot_fixed", J_c_FR_);
    robot_->getDenseFrameJacobian("FL_foot_fixed", J_c_FL_);
    robot_->getDenseFrameJacobian("RR_foot_fixed", J_c_RR_);
    robot_->getDenseFrameJacobian("RL_foot_fixed", J_c_RL_);
    J_c_.block(0, 0, 3, dof_) = J_c_FR_;
    J_c_.block(3, 0, 3, dof_) = J_c_FL_;
    J_c_.block(6, 0, 3, dof_) = J_c_RR_;
    J_c_.block(9, 0, 3, dof_) = J_c_RL_;

    QR_decomposition();
  
    A_tmp_ = Eigen::MatrixXd::Zero(dof_, var_dim_);
    A_tmp_.block(0, 0, dof_, dof_) = M_;
    A_tmp_.block(0, dof_, dof_, dof_-6) = -S_.transpose();

    auto contacts = robot_->getContacts();
    for(auto contact : contacts){
        auto idx = contact.getlocalBodyIndex();
        if(idx == 3) r_c_FR_ = contact.getPosition().e();
        if(idx == 6) r_c_FL_ = contact.getPosition().e();
        if(idx == 9) r_c_RR_ = contact.getPosition().e();
        if(idx == 12) r_c_RL_ = contact.getPosition().e();
    }
    D_.block(0,0,3,3) = Utils::skew(r_c_FR_ - CoM_position_);
    D_.block(0,3,3,3) = Utils::skew(r_c_FL_ - CoM_position_);
    D_.block(0,6,3,3) = Utils::skew(r_c_RR_ - CoM_position_);
    D_.block(0,9,3,3) = Utils::skew(r_c_RL_ - CoM_position_);
    A_ = D_*R_.inverse()*Qc_.transpose()*A_tmp_;
}

void TaskLS_AngularMomentum::updateVector(){
    b_ = weight_*(desired_angular_momentum_ - angular_momentum_) - D_*R_.inverse()*Qc_.transpose()*robot_->getNonlinearities(gravity_).e();
}

void TaskLS_AngularMomentum::updateAngularMomentum(){
    auto CoM_position_tmp = robot_->getCompositeCOM();
    CoM_position_ = CoM_position_tmp[0].e();
    raisim::Vec<3> angular_momentum_tmp;
    robot_->getAngularMomentum(CoM_position_,angular_momentum_tmp);
    angular_momentum_ = angular_momentum_tmp.e();
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